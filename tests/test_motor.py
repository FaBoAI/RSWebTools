"""MotorClient のフレーム組立を、シリアルを模したトランスポートで検証する。"""

import struct
import threading

import pytest

from backend import params as P
from backend.motor import MotorClient, NoResponse
from backend.protocol import (
    CanFrame,
    CommType,
    FrameParser,
    encode_frame,
    pack_ext_id,
    unpack_ext_id,
)


class FakeTransport:
    """送信フレームを記録し、任意の応答を注入できるスタブ。"""

    def __init__(self):
        self.sent = []
        self.responder = None      # fn(CanFrame) -> CanFrame | None
        self.connected = True
        self.subscribers = []

    def subscribe(self, cb):
        self.subscribers.append(cb)

    def unsubscribe(self, cb):
        if cb in self.subscribers:
            self.subscribers.remove(cb)

    def send_frame(self, ext_id, data, extended=True):
        req = CanFrame(ext_id, bytes(data))
        self.sent.append(req)
        # 実機同様、応答は購読者へ非同期に配られる想定
        if self.responder is not None:
            reply = self.responder(req)
            if reply is not None:
                for cb in list(self.subscribers):
                    cb("rx", reply)

    def request(self, ext_id, data, predicate, timeout):
        self.send_frame(ext_id, data)
        if self.responder is None:
            return None
        reply = self.responder(CanFrame(ext_id, bytes(data)))
        if reply is not None and predicate(reply):
            return reply
        return None

    @property
    def last(self):
        return self.sent[-1]


def feedback_for(motor_id, host_id=0xFD, mode=2):
    ext = pack_ext_id(CommType.FEEDBACK, motor_id, host_id) | (mode << 22)
    return CanFrame(ext, struct.pack(">HHHH", 0x7FFF, 0x7FFF, 0x7FFF, 250))


@pytest.fixture
def client():
    t = FakeTransport()
    c = MotorClient(t, host_id=0xFD, retries=0)
    c.set_model(1, "RS00")
    return c


def test_enable_frame(client):
    client.transport.responder = lambda req: feedback_for(1)
    fb = client.enable(1)
    ct, d2, tgt = unpack_ext_id(client.transport.last.ext_id)
    assert (ct, d2, tgt) == (CommType.ENABLE, 0xFD, 1)
    assert client.transport.last.data == b"\x00" * 8
    assert fb.mode_name == "Run"


def test_stop_clear_fault_sets_byte0(client):
    client.transport.responder = lambda req: feedback_for(1)
    client.stop(1, clear_fault=True)
    assert client.transport.last.data[0] == 1
    client.stop(1, clear_fault=False)
    assert client.transport.last.data[0] == 0


def test_set_zero_sets_byte0(client):
    client.transport.responder = lambda req: feedback_for(1)
    client.set_zero(1)
    ct, _, tgt = unpack_ext_id(client.transport.last.ext_id)
    assert ct == CommType.SET_ZERO and tgt == 1
    assert client.transport.last.data[0] == 1


def test_set_can_id_places_new_id_in_bits_23_16(client):
    """マニュアル: 新 ID は bit23-16 = データ領域2 の上位バイト。"""
    client.transport.responder = lambda req: CanFrame(
        pack_ext_id(CommType.GET_DEVICE_ID, 0xFE05, 0xFD), b"\x11" * 8)
    info = client.set_can_id(1, 5)
    ct, d2, tgt = unpack_ext_id(client.transport.last.ext_id)
    assert ct == CommType.SET_CAN_ID
    assert tgt == 1
    assert (d2 >> 8) == 5        # 新 ID
    assert (d2 & 0xFF) == 0xFD   # ホスト ID
    assert info.motor_id == 5
    # 機種設定が新 ID へ移ること
    assert client.motor_models[5] == "RS00" and 1 not in client.motor_models


def test_set_can_id_rejects_out_of_range(client):
    with pytest.raises(Exception):
        client.set_can_id(1, 200)


def test_read_param_layout_and_decode(client):
    def responder(req):
        index = struct.unpack("<H", req.data[:2])[0]
        payload = struct.pack("<H", index) + b"\x00\x00" + struct.pack("<f", 40.0)
        return CanFrame(pack_ext_id(CommType.READ_PARAM, 1, 0xFD), payload)

    client.transport.responder = responder
    value = client.read_param(1, 0x701E)     # loc_kp
    assert value == pytest.approx(40.0)
    sent = client.transport.last
    assert unpack_ext_id(sent.ext_id)[0] == CommType.READ_PARAM
    assert struct.unpack("<H", sent.data[:2])[0] == 0x701E
    assert sent.data[2:] == b"\x00" * 6


def test_read_param_ignores_mismatched_index(client):
    """別インデックスの応答は採用しない。"""
    client.transport.responder = lambda req: CanFrame(
        pack_ext_id(CommType.READ_PARAM, 1, 0xFD),
        struct.pack("<H", 0x1234) + b"\x00\x00" + struct.pack("<f", 1.0))
    with pytest.raises(NoResponse):
        client.read_param(1, 0x701E)


def test_write_param_uint8_layout(client):
    client.transport.responder = lambda req: feedback_for(1)
    client.write_param(1, 0x7005, 2)
    d = client.transport.last.data
    assert struct.unpack("<H", d[:2])[0] == 0x7005
    assert d[2:4] == b"\x00\x00"
    assert d[4] == 2 and d[5:] == b"\x00\x00\x00"


def test_write_param_float_layout(client):
    client.transport.responder = lambda req: feedback_for(1)
    client.write_param(1, 0x701E, 30.0)
    d = client.transport.last.data
    assert struct.unpack("<f", d[4:8])[0] == pytest.approx(30.0)


def test_write_param_rejects_readonly(client):
    with pytest.raises(Exception):
        client.write_param(1, 0x7019, 1.0)      # mechPos は R のみ


def test_save_frame_payload(client):
    client.save(1)
    assert client.transport.last.data == bytes([1, 2, 3, 4, 5, 6, 7, 8])
    assert unpack_ext_id(client.transport.last.ext_id)[0] == CommType.SAVE


def test_active_report_payload(client):
    client.set_active_report(1, True)
    assert client.transport.last.data == bytes([1, 2, 3, 4, 5, 6, 0x01, 0])
    client.set_active_report(1, False)
    assert client.transport.last.data == bytes([1, 2, 3, 4, 5, 6, 0x00, 0])


def test_can_baudrate_codes(client):
    for baud, code in [(1_000_000, 1), (500_000, 2), (250_000, 3), (125_000, 4)]:
        client.set_can_baudrate(1, baud)
        assert client.transport.last.data[6] == code
    with pytest.raises(Exception):
        client.set_can_baudrate(1, 800_000)


def test_protocol_codes(client):
    for name, code in [("private", 0), ("canopen", 1), ("mit", 2)]:
        client.set_protocol(1, name)
        assert client.transport.last.data[6] == code


def test_read_version_request(client):
    client.transport.responder = lambda req: CanFrame(
        pack_ext_id(CommType.READ_VERSION, 1, 0xFD),
        bytes([0x00, 0xC4, 0x56, 1, 2, 3, 4, 0]))
    v = client.read_version(1)
    assert client.transport.last.data[:2] == bytes([0x00, 0xC4])
    assert v["version"] == "1.2.3.4"


def test_read_all_reports_per_param_errors(client):
    """一部が無応答でも全体は完走し、error として返る。"""
    def responder(req):
        index = struct.unpack("<H", req.data[:2])[0]
        if index == 0x7005:
            return None
        payload = struct.pack("<H", index) + b"\x00\x00" + struct.pack("<f", 1.0)
        return CanFrame(pack_ext_id(CommType.READ_PARAM, 1, 0xFD), payload)

    client.transport.responder = responder
    result = client.read_all(1)
    assert "error" in result["run_mode"]
    assert result["loc_kp"]["value"] == pytest.approx(1.0)


def test_scan_finds_motors_without_per_id_waiting(client):
    """スキャンは応答を 1 台ずつ待たず、一斉送信して応答をまとめて拾う。"""
    def responder(req):
        ct, _, tgt = unpack_ext_id(req.ext_id)
        if ct == CommType.GET_DEVICE_ID and tgt in (6, 127):
            return CanFrame(pack_ext_id(CommType.GET_DEVICE_ID, 0xFE00 | tgt, 0xFD),
                            bytes([tgt]) * 8)
        return None

    client.transport.responder = responder
    found = client.scan(0, 127, gap=0, settle=0)
    assert [m["motor_id"] for m in found] == [6, 127]
    assert found[0]["detected_by"] == "device_id"
    assert found[1]["uid"] == "7F" * 8


def test_scan_falls_back_to_read_param(client):
    """通信タイプ 0 に応答しない個体も read_param で検出できる。"""
    def responder(req):
        ct, _, tgt = unpack_ext_id(req.ext_id)
        if ct == CommType.READ_PARAM and tgt == 3:
            return CanFrame(pack_ext_id(CommType.READ_PARAM, 3, 0xFD),
                            struct.pack("<H", 0x7005) + b"\x00" * 6)
        return None

    client.transport.responder = responder
    found = client.scan(0, 5, gap=0, settle=0)
    assert [m["motor_id"] for m in found] == [3]
    assert found[0]["detected_by"] == "read_param"


def test_scan_does_not_leak_subscribers(client):
    """スキャン後に購読者が残らないこと (残るとログが二重になる)。"""
    client.transport.responder = lambda req: None
    before = len(client.transport.subscribers)
    client.scan(0, 3, gap=0, settle=0)
    assert len(client.transport.subscribers) == before


def test_motion_control_uses_selected_model(client):
    client.set_model(1, "RS04")
    client.transport.responder = lambda req: feedback_for(1)
    client.motion_control(1, torque=120.0, position=0, velocity=0, kp=0, kd=0)
    _, d2, _ = unpack_ext_id(client.transport.last.ext_id)
    assert d2 == 0xFFFF          # RS04 の T_MAX = 120Nm -> 上限


def test_motion_control_accepts_feedback_addressed_to_can_master(client):
    """運動制御の応答はホスト ID ではなく CAN_MASTER 宛に返る。

    通信タイプ 1 はデータ領域2 がトルク値なのでホスト ID を運べない。
    宛先の一致を要求すると、CAN_MASTER が既定値 0 のモータからの応答を
    取りこぼす。
    """
    def responder(req):
        # 宛先はこちらの host_id (0xFD) ではなく CAN_MASTER (0x00)
        return CanFrame(pack_ext_id(CommType.FEEDBACK, 1, 0x00),
                        struct.pack(">HHHH", 0x7FFF, 0x7FFF, 0x7FFF, 250))

    client.transport.responder = responder
    fb = client.motion_control(1, torque=0, position=0, velocity=0, kp=0, kd=0)
    assert fb is not None, "CAN_MASTER 宛のフィードバックを受け取れていない"
    assert fb.motor_id == 1


def test_normal_commands_still_require_matching_host(client):
    """一方、通常コマンドは宛先の一致を要求する (他ホスト宛の取り込み防止)。"""
    client.transport.responder = lambda req: CanFrame(
        pack_ext_id(CommType.FEEDBACK, 1, 0x11),   # 別ホスト宛
        struct.pack(">HHHH", 0, 0, 0, 250))
    with pytest.raises(NoResponse):
        client.enable(1)
