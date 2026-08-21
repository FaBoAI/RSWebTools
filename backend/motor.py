"""RobStride モータへの高水準クライアント。

すべて同期 API。FastAPI 側からは asyncio.to_thread 経由で呼ぶ。
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

from . import params as P
from .models import MotorProfile, get_profile
from .protocol import (
    DEFAULT_HOST_ID,
    CanFrame,
    CommType,
    Feedback,
    decode_feedback,
    encode_motion_control,
    pack_ext_id,
)
from .transport import SerialTransport, TransportError

log = logging.getLogger(__name__)

ZERO8 = b"\x00" * 8

#: 通信タイプ 22/23/24/25 の固定プリフィックス (マニュアル記載の 01 02 03 04 05 06)
_MAGIC = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06])

#: 通信タイプ 23 の F_CMD
BAUD_CODES = {1_000_000: 0x01, 500_000: 0x02, 250_000: 0x03, 125_000: 0x04}

#: 通信タイプ 25 の F_CMD
PROTOCOL_CODES = {"private": 0x00, "canopen": 0x01, "mit": 0x02}


class MotorError(RuntimeError):
    pass


class NoResponse(MotorError):
    def __init__(self, what: str) -> None:
        super().__init__(f"モータから応答がありません ({what})")


@dataclass
class DeviceInfo:
    motor_id: int
    uid: str          # 64bit MCU 固有 ID の hex
    raw_id: str


class MotorClient:
    def __init__(self, transport: SerialTransport, host_id: int = DEFAULT_HOST_ID,
                 timeout: float = 0.5, retries: int = 1) -> None:
        self.transport = transport
        self.host_id = host_id
        self.timeout = timeout
        self.retries = retries
        self._lock = threading.RLock()
        #: motor_id -> 機種キー。フィードバックのスケーリングに使う。
        self.motor_models: Dict[int, str] = {}

    # -- 補助 -------------------------------------------------------------
    def profile(self, motor_id: int) -> MotorProfile:
        return get_profile(self.motor_models.get(motor_id))

    def set_model(self, motor_id: int, model: str) -> None:
        self.motor_models[motor_id] = model

    def _request(self, comm_type: int, data2: int, motor_id: int, data: bytes,
                 predicate, what: str, timeout: Optional[float] = None,
                 required: bool = True) -> Optional[CanFrame]:
        ext_id = pack_ext_id(comm_type, data2, motor_id)
        attempts = self.retries + 1
        for attempt in range(attempts):
            with self._lock:
                frame = self.transport.request(ext_id, data, predicate,
                                               timeout or self.timeout)
            if frame is not None:
                return frame
            log.debug("%s: 応答なし (%d/%d)", what, attempt + 1, attempts)
        if required:
            raise NoResponse(what)
        return None

    def _feedback_predicate(self, motor_id: int, strict_host: bool = True):
        """フィードバックフレームの照合条件。

        通常のコマンド (運転許可・停止・書込など) はデータ領域2 にホスト CAN_ID を
        載せるため、モータはそれをそのまま宛先にして返す。一方 **運動制御
        (通信タイプ 1) はデータ領域2 がトルク値**でホスト ID を運べないので、
        モータは自身に保存された CAN_MASTER を宛先にして返す。CAN_MASTER の
        工場出荷値は 0 で、こちらの host_id (既定 0xFD) とは一致しないため、
        宛先の一致を要求すると運動制御の応答を取りこぼす。
        """
        def pred(f: CanFrame) -> bool:
            if f.comm_type != CommType.FEEDBACK or f.motor_id != motor_id:
                return False
            return f.target_id == self.host_id if strict_host else True
        return pred

    def _decode_fb(self, frame: CanFrame) -> Feedback:
        pr = self.profile(frame.motor_id)
        return decode_feedback(frame, pr.p_min, pr.p_max, pr.v_min, pr.v_max,
                               pr.t_min, pr.t_max)

    def _send_only(self, comm_type: int, motor_id: int, data: bytes) -> None:
        self.transport.send_frame(pack_ext_id(comm_type, self.host_id, motor_id), data)

    # -- 基本操作 ---------------------------------------------------------
    def enable(self, motor_id: int) -> Feedback:
        """運転許可 (通信タイプ 3)。"""
        f = self._request(CommType.ENABLE, self.host_id, motor_id, ZERO8,
                          self._feedback_predicate(motor_id), "運転許可")
        return self._decode_fb(f)

    def stop(self, motor_id: int, clear_fault: bool = False) -> Feedback:
        """停止 (通信タイプ 4)。clear_fault=True で Byte0=1 とし故障をクリア。"""
        data = bytes([1 if clear_fault else 0]) + b"\x00" * 7
        f = self._request(CommType.STOP, self.host_id, motor_id, data,
                          self._feedback_predicate(motor_id), "停止")
        return self._decode_fb(f)

    def stop_nowait(self, motor_id: int) -> None:
        """非常停止用。応答を待たずに停止フレームだけ送る。"""
        self._send_only(CommType.STOP, motor_id, ZERO8)

    def set_zero(self, motor_id: int) -> Feedback:
        """機械原点を現在位置に設定 (通信タイプ 6)。"""
        data = bytes([1]) + b"\x00" * 7
        f = self._request(CommType.SET_ZERO, self.host_id, motor_id, data,
                          self._feedback_predicate(motor_id), "原点設定")
        return self._decode_fb(f)

    def set_can_id(self, motor_id: int, new_id: int) -> DeviceInfo:
        """CAN_ID を変更 (通信タイプ 7、即時反映)。

        新 ID は拡張 ID の bit23-16 に載せる = データ領域2 の上位バイト。
        応答は通信タイプ 0 のブロードキャストフレーム。
        """
        if not 0 <= new_id <= 0x7F:
            raise MotorError("CAN_ID は 0〜127 (0x00〜0x7F) の範囲で指定してください")
        data2 = ((new_id & 0xFF) << 8) | (self.host_id & 0xFF)
        f = self._request(CommType.SET_CAN_ID, data2, motor_id, ZERO8,
                          lambda fr: fr.comm_type == CommType.GET_DEVICE_ID,
                          "CAN_ID 変更", timeout=1.0)
        if motor_id in self.motor_models:
            self.motor_models[new_id] = self.motor_models.pop(motor_id)
        return DeviceInfo(motor_id=new_id, uid=f.data.hex().upper(),
                          raw_id=f"{f.ext_id:08X}")

    def get_device_id(self, motor_id: int, timeout: float = 0.25) -> Optional[DeviceInfo]:
        """デバイス ID + 64bit MCU 固有 ID を取得 (通信タイプ 0)。"""
        f = self._request(CommType.GET_DEVICE_ID, self.host_id, motor_id, ZERO8,
                          lambda fr: fr.comm_type == CommType.GET_DEVICE_ID,
                          "デバイス ID 取得", timeout=timeout, required=False)
        if f is None:
            return None
        return DeviceInfo(motor_id=f.motor_id or motor_id,
                          uid=f.data.hex().upper(), raw_id=f"{f.ext_id:08X}")

    # -- パラメータ -------------------------------------------------------
    def read_param_raw(self, motor_id: int, index: int) -> bytes:
        """通信タイプ 17。Byte0-1 に index、応答の Byte4-7 が値。"""
        data = struct.pack("<H", index) + b"\x00" * 6

        def pred(f: CanFrame) -> bool:
            return (f.comm_type == CommType.READ_PARAM
                    and f.motor_id == motor_id
                    and len(f.data) >= 8
                    and struct.unpack("<H", f.data[:2])[0] == index)

        f = self._request(CommType.READ_PARAM, self.host_id, motor_id, data, pred,
                          f"パラメータ読出 0x{index:04X}")
        return f.data[4:8]

    def read_param(self, motor_id: int, index: int, ptype: Optional[str] = None):
        pdef = P.PARAMS_BY_INDEX.get(index)
        ptype = ptype or (pdef.type if pdef else P.TYPE_F32)
        return P.decode_value(ptype, self.read_param_raw(motor_id, index))

    def write_param(self, motor_id: int, index: int, value,
                    ptype: Optional[str] = None) -> Optional[Feedback]:
        """通信タイプ 18。応答はフィードバックフレーム。"""
        pdef = P.PARAMS_BY_INDEX.get(index)
        if pdef is not None and not pdef.writable:
            raise MotorError(f"{pdef.name} (0x{index:04X}) は読み出し専用です")
        ptype = ptype or (pdef.type if pdef else P.TYPE_F32)
        data = struct.pack("<H", index) + b"\x00\x00" + P.encode_value(ptype, value)
        f = self._request(CommType.WRITE_PARAM, self.host_id, motor_id, data,
                          self._feedback_predicate(motor_id),
                          f"パラメータ書込 0x{index:04X}", required=False)
        return self._decode_fb(f) if f is not None else None

    def read_all(self, motor_id: int, indices: Optional[List[int]] = None) -> Dict[str, dict]:
        """パラメータ表を一括読み出し。個別の失敗は握って error として返す。"""
        result: Dict[str, dict] = {}
        targets = indices if indices is not None else [p.index for p in P.PARAMS]
        for index in targets:
            pdef = P.PARAMS_BY_INDEX.get(index)
            name = pdef.name if pdef else f"0x{index:04X}"
            try:
                result[name] = {"index": index, "value": self.read_param(motor_id, index)}
            except MotorError as exc:
                result[name] = {"index": index, "value": None, "error": str(exc)}
        return result

    def scan_indices(self, motor_id: int, start: int, end: int,
                     timeout: float = 0.12) -> List[dict]:
        """未知のインデックスを総当たりして応答するものを列挙する。

        マニュアルに記載のない 0x20xx / 0x30xx 系を実機で調べる用。
        読み出しのみで書き込みは一切しない。
        """
        found = []
        for index in range(start, end + 1):
            data = struct.pack("<H", index) + b"\x00" * 6

            def pred(f: CanFrame, idx=index) -> bool:
                return (f.comm_type == CommType.READ_PARAM and f.motor_id == motor_id
                        and len(f.data) >= 8
                        and struct.unpack("<H", f.data[:2])[0] == idx)

            frame = self._request(CommType.READ_PARAM, self.host_id, motor_id, data,
                                  pred, "index スキャン", timeout=timeout, required=False)
            if frame is None:
                continue
            raw = frame.data[4:8]
            found.append({
                "index": index,
                "index_hex": f"0x{index:04X}",
                "raw": raw.hex().upper(),
                "as_float": P.decode_value(P.TYPE_F32, raw),
                "as_uint32": P.decode_value(P.TYPE_U32, raw),
                "known": P.PARAMS_BY_INDEX.get(index).name if index in P.PARAMS_BY_INDEX else None,
            })
        return found

    # -- 保存・アダプタ設定 ----------------------------------------------
    def save(self, motor_id: int) -> Optional[Feedback]:
        """パラメータ保存 (通信タイプ 22)。データ部は 01 02 03 04 05 06 07 08。"""
        data = _MAGIC + bytes([0x07, 0x08])
        f = self._request(CommType.SAVE, self.host_id, motor_id, data,
                          self._feedback_predicate(motor_id), "保存",
                          timeout=1.5, required=False)
        return self._decode_fb(f) if f is not None else None

    def set_active_report(self, motor_id: int, enable: bool) -> None:
        """能動送信の ON/OFF (通信タイプ 24)。周期は EPScan_time で調整。"""
        data = _MAGIC + bytes([0x01 if enable else 0x00, 0x00])
        self._send_only(CommType.ACTIVE_REPORT, motor_id, data)

    def set_can_baudrate(self, motor_id: int, baudrate: int) -> None:
        """モータ側 CAN ボーレート変更 (通信タイプ 23、再投入で反映)。"""
        if baudrate not in BAUD_CODES:
            raise MotorError(f"対応していないボーレートです: {baudrate}")
        data = _MAGIC + bytes([BAUD_CODES[baudrate], 0x00])
        self._send_only(CommType.SET_BAUD, motor_id, data)

    def set_protocol(self, motor_id: int, protocol: str) -> None:
        """プロトコル切替 (通信タイプ 25、再投入で反映)。"""
        key = protocol.lower()
        if key not in PROTOCOL_CODES:
            raise MotorError(f"未知のプロトコルです: {protocol}")
        data = _MAGIC + bytes([PROTOCOL_CODES[key], 0x00])
        self._send_only(CommType.SET_PROTOCOL, motor_id, data)

    def read_version(self, motor_id: int) -> Optional[dict]:
        """バージョン読出 (通信タイプ 26)。Byte0=0x00, Byte1=0xC4 を送る。"""
        data = bytes([0x00, 0xC4]) + b"\x00" * 6
        f = self._request(CommType.READ_VERSION, self.host_id, motor_id, data,
                          lambda fr: fr.comm_type == CommType.READ_VERSION
                          and fr.motor_id == motor_id,
                          "バージョン読出", required=False)
        if f is None or len(f.data) < 7:
            return None
        return {
            "raw": f.data.hex().upper(),
            "version": ".".join(str(b) for b in f.data[3:7]),
        }

    # -- 運動制御 ---------------------------------------------------------
    def motion_control(self, motor_id: int, torque: float, position: float,
                       velocity: float, kp: float, kd: float,
                       wait: bool = True) -> Optional[Feedback]:
        pr = self.profile(motor_id)
        ext_id, data = encode_motion_control(
            torque, position, velocity, kp, kd, motor_id,
            pr.p_min, pr.p_max, pr.v_min, pr.v_max, pr.t_min, pr.t_max,
            pr.kp_max, pr.kd_max,
        )
        if not wait:
            self.transport.send_frame(ext_id, data)
            return None
        with self._lock:
            f = self.transport.request(
                ext_id, data,
                self._feedback_predicate(motor_id, strict_host=False),
                self.timeout,
            )
        return self._decode_fb(f) if f is not None else None

    # -- 探索 -------------------------------------------------------------
    def scan(self, start: int = 0, end: int = 127, gap: float = 0.008,
             settle: float = 0.6) -> List[dict]:
        """CAN_ID を総当たりしてモータを探す。

        1 台ずつ応答を待つと 0〜127 で 50 秒近くかかるため、**問い合わせを
        一斉に送ってから応答をまとめて拾う**方式にしている。応答フレームには
        送信元のモータ CAN_ID が入っているので、どの問い合わせに対する応答か
        を待って対応付ける必要がない。

        通信タイプ 0 に応答しない個体もあるため、応答が無かった ID には
        run_mode (0x7005) の読み出しでも当たる。
        """
        found: Dict[int, dict] = {}

        def collector(kind: str, payload) -> None:
            if kind != "rx":
                return
            frame = payload
            if frame.comm_type == CommType.GET_DEVICE_ID:
                found.setdefault(frame.motor_id, {
                    "motor_id": frame.motor_id,
                    "uid": frame.data.hex().upper(),
                    "detected_by": "device_id",
                })
            elif frame.comm_type == CommType.READ_PARAM:
                found.setdefault(frame.motor_id, {
                    "motor_id": frame.motor_id,
                    "uid": None,
                    "detected_by": "read_param",
                })

        self.transport.subscribe(collector)
        try:
            for motor_id in range(start, end + 1):
                self.transport.send_frame(
                    pack_ext_id(CommType.GET_DEVICE_ID, self.host_id, motor_id), ZERO8)
                time.sleep(gap)
            time.sleep(settle)

            missing = [m for m in range(start, end + 1) if m not in found]
            if missing:
                data = struct.pack("<H", 0x7005) + b"\x00" * 6
                for motor_id in missing:
                    self.transport.send_frame(
                        pack_ext_id(CommType.READ_PARAM, self.host_id, motor_id), data)
                    time.sleep(gap)
                time.sleep(settle)
        finally:
            self.transport.unsubscribe(collector)

        return [found[k] for k in sorted(found)]
