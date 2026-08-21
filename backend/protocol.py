"""RobStride 私有プロトコル (CAN 2.0B 拡張フレーム) と
RobStride 純正 USB-CAN アダプタ (AT モード) のシリアルフレーム変換。

出典 (すべて公式マニュアルで確認済み):
  * 29bit 拡張 ID = [bit28-24 通信タイプ][bit23-8 データ領域2][bit7-0 宛先 CAN_ID]
  * シリアルフレーム = 'AT' + BE32((ext_id << 3) | 0x04) + DLC + data[DLC] + CRLF
    マニュアル記載例:
      41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a
      -> 0x9007E80C >> 3 == 0x1200FD01
         type=0x12(書込) / host=0x00FD / motor=0x01 / index=0x7005 / value=1
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterator, List, Tuple

HEADER = b"AT"
TAIL = b"\r\n"

#: 拡張フレームであることを示す下位 3bit のフラグ (IDE ビット)
EXT_FLAG = 0x04

#: ホスト側 CAN_ID の既定値。マニュアルの例が 0xFD を使用している。
DEFAULT_HOST_ID = 0xFD


class CommType(IntEnum):
    """通信タイプ (拡張 ID の bit28-24)。"""

    GET_DEVICE_ID = 0x00      # デバイス ID / MCU 固有 ID 取得
    MOTION_CONTROL = 0x01     # 運動制御モード指令
    FEEDBACK = 0x02           # モータフィードバック
    ENABLE = 0x03             # 運転許可
    STOP = 0x04               # 停止 (Byte0=1 で故障クリア)
    SET_ZERO = 0x06           # 機械原点設定
    SET_CAN_ID = 0x07         # CAN_ID 変更 (即時反映)
    READ_PARAM = 0x11         # 単一パラメータ読出
    WRITE_PARAM = 0x12        # 単一パラメータ書込 (電源断で消失)
    FAULT_FEEDBACK = 0x15     # 故障フィードバック
    SAVE = 0x16               # パラメータ保存 (0x20xx 系を不揮発化)
    SET_BAUD = 0x17           # CAN ボーレート変更 (再投入で反映)
    ACTIVE_REPORT = 0x18      # 能動送信の ON/OFF・その報告フレーム
    SET_PROTOCOL = 0x19       # プロトコル切替 (再投入で反映)
    READ_VERSION = 0x1A       # バージョン読出


class RunMode(IntEnum):
    """0x7005 run_mode の値。"""

    OPERATION = 0   # 運動制御モード (MIT 風 5 パラメータ)
    POSITION_PP = 1 # 位置モード (PP: 台形加減速)
    SPEED = 2       # 速度モード
    CURRENT = 3     # 電流モード
    POSITION_CSP = 5  # 位置モード (CSP: 速度制限付きサイクリック)


class MotorMode(IntEnum):
    """フィードバックフレーム bit23-22 のモードステータス。"""

    RESET = 0
    CALIBRATION = 1
    RUN = 2


#: フィードバックフレーム bit21-16 の故障ビット (LSB = bit16)
FEEDBACK_FAULT_BITS: Tuple[Tuple[int, str], ...] = (
    (0, "低電圧"),
    (1, "三相電流異常"),
    (2, "過温度"),
    (3, "磁気エンコーダ異常"),
    (4, "ロック/過負荷"),
    (5, "未キャリブレーション"),
)

#: 通信タイプ 0x15 (故障フィードバック) Byte0-3 の故障ビット
FAULT_FRAME_BITS: Tuple[Tuple[int, str], ...] = (
    (0, "モータ過温度 (既定 145℃)"),
    (1, "ドライバチップ故障"),
    (2, "低電圧故障"),
    (3, "過電圧故障"),
    (4, "B 相電流サンプリング過電流"),
    (5, "C 相電流サンプリング過電流"),
    (7, "エンコーダ未キャリブレーション"),
    (8, "ハードウェア識別故障"),
    (9, "位置初期化故障"),
    (14, "ロック過負荷アルゴリズム保護"),
    (16, "A 相電流サンプリング過電流"),
)

#: 通信タイプ 0x15 Byte4-7 の警告ビット
WARNING_FRAME_BITS: Tuple[Tuple[int, str], ...] = (
    (0, "モータ過温度警告 (既定 135℃)"),
)


# --------------------------------------------------------------------------
# 拡張 ID の組立/分解
# --------------------------------------------------------------------------

def pack_ext_id(comm_type: int, data2: int, target_id: int) -> int:
    return ((int(comm_type) & 0x1F) << 24) | ((int(data2) & 0xFFFF) << 8) | (int(target_id) & 0xFF)


def unpack_ext_id(ext_id: int) -> Tuple[int, int, int]:
    """-> (通信タイプ, データ領域2, 宛先 CAN_ID)"""
    return (ext_id >> 24) & 0x1F, (ext_id >> 8) & 0xFFFF, ext_id & 0xFF


# --------------------------------------------------------------------------
# 値のスケーリング (マニュアルの float_to_uint と同一式: 分母は 2^bits - 1)
# --------------------------------------------------------------------------

def float_to_uint(x: float, x_min: float, x_max: float, bits: int = 16) -> int:
    span = x_max - x_min
    x = min(max(float(x), x_min), x_max)
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(v: int, x_min: float, x_max: float, bits: int = 16) -> float:
    span = x_max - x_min
    return x_min + (v * span) / ((1 << bits) - 1)


# --------------------------------------------------------------------------
# CAN フレーム
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class CanFrame:
    ext_id: int
    data: bytes
    extended: bool = True

    @property
    def comm_type(self) -> int:
        return (self.ext_id >> 24) & 0x1F

    @property
    def data2(self) -> int:
        return (self.ext_id >> 8) & 0xFFFF

    @property
    def target_id(self) -> int:
        return self.ext_id & 0xFF

    @property
    def motor_id(self) -> int:
        """応答フレームでは bit15-8 が送信元モータの CAN_ID。"""
        return (self.ext_id >> 8) & 0xFF

    def hex(self) -> str:
        return " ".join(f"{b:02X}" for b in self.data)

    def __str__(self) -> str:  # ログ表示用
        kind = "" if self.extended else "S"
        return f"{self.ext_id:08X}{kind}#{self.hex()}"


def encode_frame(ext_id: int, data: bytes, extended: bool = True) -> bytes:
    """CAN フレームを USB-CAN アダプタのシリアルフレームへ。

    RobStride 私有プロトコルは拡張フレームのみを使うが、MIT プロトコルへ
    切り替えたモータは 11bit 標準フレームで通信する。標準フレームでは
    IDE ビット (0x04) を落とす — マニュアルに明示的な記載はないため、
    拡張フレームの記載から類推した実装。
    """
    if len(data) > 8:
        raise ValueError("CAN のデータ長は最大 8 バイトです")
    mask = 0x1FFFFFFF if extended else 0x7FF
    wire = ((ext_id & mask) << 3) | (EXT_FLAG if extended else 0x00)
    return HEADER + struct.pack(">I", wire) + bytes([len(data)]) + bytes(data) + TAIL


def build_frame(comm_type: int, data2: int, target_id: int, data: bytes = b"\x00" * 8) -> bytes:
    return encode_frame(pack_ext_id(comm_type, data2, target_id), data)


class FrameParser:
    """バイトストリームからシリアルフレームを取り出す耐ノイズパーサ。

    アダプタは AT コマンドへの ASCII 応答 (``OK`` など) も返すため、
    フレームとして解釈できなかったバイト列は ``junk`` イベントとして
    生バイトのまま通知する。
    """

    #: 未同期バッファがこの長さを超えたら先頭を捨てる
    MAX_BUFFER = 4096

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> List[Tuple[str, object]]:
        """-> [('frame', CanFrame) | ('junk', bytes), ...]

        フレームとして解釈できなかったバイト列は ``junk`` として **生バイトのまま**
        返す。ASCII に変換してしまうと 0x80 以上のバイトが失われ、ボーレート違いや
        別フレーム形式の診断ができなくなるため。
        """
        self._buf.extend(chunk)
        events: List[Tuple[str, object]] = []
        junk = bytearray()

        while True:
            start = self._buf.find(HEADER)
            if start < 0:
                # ヘッダが無い。'A' で終わる可能性があるので 1 バイトだけ残す。
                keep = 1 if self._buf.endswith(b"A") else 0
                if len(self._buf) > keep:
                    junk.extend(self._buf[: len(self._buf) - keep])
                    del self._buf[: len(self._buf) - keep]
                break

            if start:
                junk.extend(self._buf[:start])
                del self._buf[:start]

            if len(self._buf) < 7:  # AT + id(4) + dlc(1) が揃うまで待つ
                break
            dlc = self._buf[6]
            if dlc > 8:
                junk.extend(self._buf[:2])
                del self._buf[:2]
                continue
            total = 7 + dlc + len(TAIL)
            if len(self._buf) < total:
                if len(self._buf) > self.MAX_BUFFER:
                    junk.extend(self._buf[:2])
                    del self._buf[:2]
                    continue
                break
            if bytes(self._buf[total - 2 : total]) != TAIL:
                # 末尾が合わない = 偽ヘッダ。2 バイト進めて再同期。
                junk.extend(self._buf[:2])
                del self._buf[:2]
                continue

            wire = struct.unpack(">I", bytes(self._buf[2:6]))[0]
            data = bytes(self._buf[7 : 7 + dlc])
            del self._buf[:total]
            extended = bool(wire & EXT_FLAG)
            raw_id = (wire >> 3) & (0x1FFFFFFF if extended else 0x7FF)
            events.append(("frame", CanFrame(ext_id=raw_id, data=data, extended=extended)))

        if junk:
            events.append(("junk", bytes(junk)))
        return events


# --------------------------------------------------------------------------
# フレームのデコード
# --------------------------------------------------------------------------

@dataclass
class Feedback:
    """通信タイプ 0x02 / 0x18 のフィードバック。"""

    motor_id: int
    angle: float          # rad
    velocity: float       # rad/s
    torque: float         # Nm
    temperature: float    # ℃
    mode: int
    mode_name: str
    faults: List[str] = field(default_factory=list)
    fault_bits: int = 0

    def as_dict(self) -> dict:
        return {
            "motor_id": self.motor_id,
            "angle": self.angle,
            "velocity": self.velocity,
            "torque": self.torque,
            "temperature": self.temperature,
            "mode": self.mode,
            "mode_name": self.mode_name,
            "faults": self.faults,
            "fault_bits": self.fault_bits,
        }


_MODE_NAMES = {0: "Reset", 1: "Cali", 2: "Run"}


def decode_feedback(frame: CanFrame, p_min: float, p_max: float,
                    v_min: float, v_max: float,
                    t_min: float, t_max: float) -> Feedback:
    """フィードバックフレームを物理量へ変換する。

    スケーリングレンジはモータ機種ごとに異なるため呼び出し側が渡す。
    """
    if len(frame.data) < 8:
        raise ValueError("フィードバックフレームは 8 バイト必要です")
    angle_raw, vel_raw, tor_raw, temp_raw = struct.unpack(">HHHH", frame.data[:8])
    fault_bits = (frame.ext_id >> 16) & 0x3F
    mode = (frame.ext_id >> 22) & 0x03
    return Feedback(
        motor_id=frame.motor_id,
        angle=uint_to_float(angle_raw, p_min, p_max),
        velocity=uint_to_float(vel_raw, v_min, v_max),
        torque=uint_to_float(tor_raw, t_min, t_max),
        temperature=temp_raw / 10.0,
        mode=mode,
        mode_name=_MODE_NAMES.get(mode, f"不明({mode})"),
        faults=[name for bit, name in FEEDBACK_FAULT_BITS if fault_bits & (1 << bit)],
        fault_bits=fault_bits,
    )


def decode_fault_frame(frame: CanFrame) -> dict:
    """通信タイプ 0x15 のデコード。"""
    fault = warn = 0
    if len(frame.data) >= 4:
        fault = struct.unpack("<I", frame.data[0:4])[0]
    if len(frame.data) >= 8:
        warn = struct.unpack("<I", frame.data[4:8])[0]
    return {
        "motor_id": frame.motor_id,
        "fault_value": fault,
        "warning_value": warn,
        "faults": [n for b, n in FAULT_FRAME_BITS if fault & (1 << b)],
        "warnings": [n for b, n in WARNING_FRAME_BITS if warn & (1 << b)],
    }


def encode_motion_control(torque: float, position: float, velocity: float,
                          kp: float, kd: float, motor_id: int,
                          p_min: float, p_max: float, v_min: float, v_max: float,
                          t_min: float, t_max: float,
                          kp_max: float, kd_max: float) -> Tuple[int, bytes]:
    """通信タイプ 0x01 の (ext_id, data) を作る。

    トルクは 8 バイトのデータ領域ではなく拡張 ID のデータ領域2 に入る。
    データ領域はすべて上位バイト先行 (ビッグエンディアン)。
    """
    ext_id = pack_ext_id(
        CommType.MOTION_CONTROL,
        float_to_uint(torque, t_min, t_max),
        motor_id,
    )
    data = struct.pack(
        ">HHHH",
        float_to_uint(position, p_min, p_max),
        float_to_uint(velocity, v_min, v_max),
        float_to_uint(kp, 0.0, kp_max),
        float_to_uint(kd, 0.0, kd_max),
    )
    return ext_id, data


def iter_hex(data: bytes) -> Iterator[str]:
    for b in data:
        yield f"{b:02X}"


def looks_like_frame(raw: bytes) -> bool:
    """AT フレームとして「構造だけ」合っているかを判定する。

    ヘッダが化けていてもフレーム長・DLC・末尾 CRLF が揃っていれば True。
    リンク品質の診断に使う (デコードはしない — ID 部も化けている可能性が
    あるため、値として信用してはいけない)。
    """
    return (len(raw) == 7 + 8 + len(TAIL)
            and raw[-2:] == TAIL
            and raw[6] == 8)
