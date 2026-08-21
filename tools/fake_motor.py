#!/usr/bin/env python3
"""仮想 RobStride モータ (RS00) を PTY 上に立てる。

実機なしで Web アプリの動作確認をするための擬似デバイス。
USB-CAN アダプタの AT シリアルプロトコルを話し、私有プロトコルの
主要な通信タイプに応答する。

  python tools/fake_motor.py            # CAN_ID 1 のモータを 1 台
  python tools/fake_motor.py --ids 1,5  # 複数台
"""
import argparse
import math
import os
import pty
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from backend import params as P  # noqa: E402
from backend.models import get_profile  # noqa: E402
from backend.protocol import (  # noqa: E402
    CommType,
    FrameParser,
    encode_frame,
    float_to_uint,
    pack_ext_id,
)

DEFAULTS = {
    0x7005: 0,        # run_mode
    0x7006: 0.0,      # iq_ref
    0x700A: 0.0,      # spd_ref
    0x700B: 14.0,     # limit_torque
    0x7010: 0.17,     # cur_kp
    0x7011: 0.012,    # cur_ki
    0x7014: 0.1,      # cur_filt_gain
    0x7016: 0.0,      # loc_ref
    0x7017: 33.0,     # limit_spd
    0x7018: 16.0,     # limit_cur
    0x701E: 40.0,     # loc_kp
    0x701F: 6.0,      # spd_kp
    0x7020: 0.02,     # spd_ki
    0x7021: 0.1,      # spd_filt_gain
    0x7022: 20.0,     # acc_rad
    0x7024: 10.0,     # vel_max
    0x7025: 10.0,     # acc_set
    0x7026: 1,        # EPScan_time
    0x7028: 0,        # canTimeout
    0x7029: 0,        # zero_sta
    0x702A: 0,        # damper
    0x702B: 0.0,      # add_offset
    0x2005: 4.6195827,  # mechOffset
}


class FakeMotor:
    def __init__(self, motor_id: int, model: str = "RS00"):
        self.id = motor_id
        self.profile = get_profile(model)
        self.params = dict(DEFAULTS)
        self.uid = bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, motor_id])
        self.mode = 0          # 0=Reset 1=Cali 2=Run
        self.faults = 0
        self.angle = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.temperature = 31.5
        self.reporting = False
        # CAN_MASTER (0x200B) 相当。実機の工場出荷値は 0。
        # 通信タイプ 1 はデータ領域2 がトルク値でホスト ID を運べないため、
        # 応答の宛先にはこちらを使う。
        self.can_master = 0x00
        self.report_period = 0.10
        self.next_report = 0.0
        self.saved = dict(self.params)

    # -- 簡易な物理モデル ------------------------------------------------
    def step(self, dt: float):
        if self.mode != 2:
            self.velocity *= 0.6
            self.torque *= 0.6
        else:
            run_mode = int(self.params[0x7005])
            if run_mode == 2:                       # 速度モード
                target = self.params[0x700A]
                self.velocity += (target - self.velocity) * min(1.0, dt * 6)
            elif run_mode in (1, 5):                # 位置モード
                err = self.params[0x7016] - self.angle
                limit = self.params[0x7017] if run_mode == 5 else self.params[0x7024]
                self.velocity = max(-limit, min(limit, err * 3.0))
            elif run_mode == 3:                     # 電流モード
                self.velocity += self.params[0x7006] * dt * 2
            self.torque = self.velocity * 0.05
        self.angle += self.velocity * dt
        # -4π〜4π で巻き戻す (通信タイプ 2 の仕様)
        span = self.profile.p_max - self.profile.p_min
        while self.angle > self.profile.p_max:
            self.angle -= span
        while self.angle < self.profile.p_min:
            self.angle += span
        self.temperature += (31.5 + abs(self.velocity) * 0.4 - self.temperature) * dt

    # -- フレーム生成 ----------------------------------------------------
    def feedback(self, host_id: int, comm_type=CommType.FEEDBACK):
        pr = self.profile
        ext = pack_ext_id(comm_type, self.id, host_id)
        ext |= (self.faults & 0x3F) << 16
        ext |= (self.mode & 0x03) << 22
        data = struct.pack(
            ">HHHH",
            float_to_uint(self.angle, pr.p_min, pr.p_max),
            float_to_uint(self.velocity, pr.v_min, pr.v_max),
            float_to_uint(self.torque, pr.t_min, pr.t_max),
            int(max(0, min(6553, self.temperature * 10))),
        )
        return ext, data

    def handle(self, ext_id: int, data: bytes):
        """-> [(ext_id, data), ...] 応答フレーム"""
        comm_type = (ext_id >> 24) & 0x1F
        data2 = (ext_id >> 8) & 0xFFFF
        target = ext_id & 0xFF
        host = data2 & 0xFF
        if target != self.id:
            return []

        if comm_type == CommType.GET_DEVICE_ID:
            return [(pack_ext_id(0, 0xFE00 | self.id, host), self.uid)]

        if comm_type == CommType.ENABLE:
            self.mode = 2
            return [self.feedback(host)]

        if comm_type == CommType.STOP:
            self.mode = 0
            if data and data[0] == 1:
                self.faults = 0
            return [self.feedback(host)]

        if comm_type == CommType.SET_ZERO:
            self.angle = 0.0
            return [self.feedback(host)]

        if comm_type == CommType.SET_CAN_ID:
            new_id = (data2 >> 8) & 0xFF
            old = self.id
            self.id = new_id
            print(f"  [motor {old}] CAN_ID -> {new_id}")
            return [(pack_ext_id(0, 0xFE00 | self.id, host), self.uid)]

        if comm_type == CommType.READ_PARAM:
            index = struct.unpack("<H", data[:2])[0]
            value = self._read(index)
            if value is None:
                return []
            pdef = P.PARAMS_BY_INDEX.get(index)
            ptype = pdef.type if pdef else P.TYPE_F32
            payload = struct.pack("<H", index) + b"\x00\x00" + P.encode_value(ptype, value)
            return [(pack_ext_id(CommType.READ_PARAM, self.id, host), payload)]

        if comm_type == CommType.WRITE_PARAM:
            index = struct.unpack("<H", data[:2])[0]
            pdef = P.PARAMS_BY_INDEX.get(index)
            ptype = pdef.type if pdef else P.TYPE_F32
            self.params[index] = P.decode_value(ptype, data[4:8])
            return [self.feedback(host)]

        if comm_type == CommType.MOTION_CONTROL:
            # データ領域2 はホスト ID ではなくトルク指令
            pr = self.profile
            self.torque = (data2 / 65535) * (pr.t_max - pr.t_min) + pr.t_min
            self.mode = 2
            return [self.feedback(self.can_master)]

        if comm_type == CommType.SAVE:
            self.saved = dict(self.params)
            print(f"  [motor {self.id}] パラメータ保存")
            return [self.feedback(host)]

        if comm_type == CommType.ACTIVE_REPORT:
            self.reporting = bool(data[6]) if len(data) > 6 else False
            self.report_period = max(0.01, 0.010 + (self.params[0x7026] - 1) * 0.005)
            print(f"  [motor {self.id}] 能動送信 {'ON' if self.reporting else 'OFF'}")
            return []

        if comm_type == CommType.READ_VERSION:
            payload = bytes([0x00, 0xC4, 0x56, 1, 0, 3, 7, 0])
            return [(pack_ext_id(CommType.READ_VERSION, self.id, host), payload)]

        return []

    def _read(self, index):
        if index in self.params:
            return self.params[index]
        # 観測パラメータは実時間の状態を返す
        return {
            0x7019: self.angle,
            0x701A: self.torque / 0.5,
            0x701B: self.velocity,
            0x701C: 48.2,
            0x3016: self.angle,
            0x3017: self.velocity,
            0x302C: self.torque,
        }.get(index)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default="1", help="カンマ区切りの CAN_ID (既定: 1)")
    ap.add_argument("--model", default="RS00")
    args = ap.parse_args()

    motors = [FakeMotor(int(x), args.model) for x in args.ids.split(",")]
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    name = os.ttyname(slave)

    print(f"仮想 RobStride ({args.model}) を起動しました")
    print(f"  CAN_ID: {', '.join(str(m.id) for m in motors)}")
    print(f"  ポート: {name}")
    print(f"\nWeb UI のシリアルポート欄に上のパスを入力して接続してください。")
    print("Ctrl-C で終了\n", flush=True)

    parser = FrameParser()
    host_id = 0xFD
    last = time.time()
    try:
        while True:
            now = time.time()
            dt = now - last
            last = now
            for m in motors:
                m.step(dt)

            try:
                chunk = os.read(master, 4096)
            except BlockingIOError:
                chunk = b""
            except OSError:
                break

            out = []
            if chunk:
                for kind, payload in parser.feed(chunk):
                    if kind != "frame":
                        continue
                    for m in motors:
                        out.extend(m.handle(payload.ext_id, payload.data))

            for m in motors:
                if m.reporting and now >= m.next_report:
                    m.next_report = now + m.report_period
                    out.append(m.feedback(host_id, CommType.ACTIVE_REPORT))

            for ext_id, data in out:
                os.write(master, encode_frame(ext_id, data))

            time.sleep(0.002)
    except KeyboardInterrupt:
        print("\n終了しました")


if __name__ == "__main__":
    main()
