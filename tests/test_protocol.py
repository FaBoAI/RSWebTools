"""プロトコル層の検証。期待値は公式マニュアル記載の数値をそのまま使う。"""

import math
import struct

import pytest

from backend import params as P
from backend.models import PROFILES, get_profile
from backend.protocol import (
    looks_like_frame,
    CanFrame,
    CommType,
    FrameParser,
    decode_fault_frame,
    decode_feedback,
    encode_frame,
    encode_motion_control,
    float_to_uint,
    pack_ext_id,
    uint_to_float,
    unpack_ext_id,
)

# マニュアル記載の実例:
#   41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a
#   -> ext_id 0x1200FD01 (type=0x12 書込 / host=0x00FD / motor=0x01)
#      data   index=0x7005 (run_mode), value=1
MANUAL_FRAME = bytes.fromhex("415490 07e80c 08 0570000001000000 0d0a".replace(" ", ""))
MANUAL_EXT_ID = 0x1200FD01
MANUAL_DATA = bytes.fromhex("0570000001000000")


def test_encode_matches_manual_example():
    assert encode_frame(MANUAL_EXT_ID, MANUAL_DATA) == MANUAL_FRAME


def test_parse_matches_manual_example():
    events = FrameParser().feed(MANUAL_FRAME)
    assert len(events) == 1
    kind, frame = events[0]
    assert kind == "frame"
    assert frame.ext_id == MANUAL_EXT_ID
    assert frame.data == MANUAL_DATA


def test_manual_example_decodes_to_run_mode_write():
    _, frame = FrameParser().feed(MANUAL_FRAME)[0]
    comm_type, data2, target = unpack_ext_id(frame.ext_id)
    assert comm_type == CommType.WRITE_PARAM
    assert data2 == 0x00FD          # ホスト CAN_ID
    assert target == 0x01           # 宛先モータ
    assert struct.unpack("<H", frame.data[:2])[0] == 0x7005
    assert P.decode_value(P.TYPE_U8, frame.data[4:8]) == 1


def test_ext_id_roundtrip():
    for ct, d2, tgt in [(0, 0, 0), (0x1F, 0xFFFF, 0xFF), (0x12, 0x00FD, 0x7F)]:
        assert unpack_ext_id(pack_ext_id(ct, d2, tgt)) == (ct, d2, tgt)


def test_parser_returns_unparsed_bytes_raw():
    """解析できなかったバイトは ASCII 変換せず生のまま返すこと。

    0x80 以上のバイトを取りこぼすと、ボーレート違いや別フレーム形式の
    診断ができなくなる。
    """
    parser = FrameParser()
    noise = bytes([0x9C, 0xFF, 0x00, 0x80])
    events = parser.feed(noise + MANUAL_FRAME)
    kinds = [k for k, _ in events]
    assert "frame" in kinds and "junk" in kinds
    assert next(v for k, v in events if k == "junk") == noise
    frame = next(v for k, v in events if k == "frame")
    assert frame.ext_id == MANUAL_EXT_ID


def test_parser_handles_split_chunks():
    parser = FrameParser()
    assert parser.feed(MANUAL_FRAME[:5]) == []
    assert parser.feed(MANUAL_FRAME[5:9]) == []
    events = parser.feed(MANUAL_FRAME[9:])
    assert [k for k, _ in events] == ["frame"]


def test_parser_handles_back_to_back_frames():
    events = FrameParser().feed(MANUAL_FRAME * 3)
    assert [k for k, _ in events] == ["frame"] * 3


def test_parser_rejects_bad_tail_and_resyncs():
    broken = bytearray(MANUAL_FRAME)
    broken[-1] = 0x00           # 末尾を壊す
    events = FrameParser().feed(bytes(broken) + MANUAL_FRAME)
    frames = [v for k, v in events if k == "frame"]
    assert len(frames) == 1
    assert frames[0].ext_id == MANUAL_EXT_ID


def test_parser_ignores_oversize_dlc():
    bogus = b"AT" + b"\x00\x00\x00\x0c" + bytes([9]) + b"\x00" * 9 + b"\r\n"
    frames = [v for k, v in FrameParser().feed(bogus + MANUAL_FRAME) if k == "frame"]
    assert len(frames) == 1


@pytest.mark.parametrize("lo,hi", [(-12.57, 12.57), (-33.0, 33.0), (0.0, 500.0)])
def test_float_uint_endpoints(lo, hi):
    assert float_to_uint(lo, lo, hi) == 0
    assert float_to_uint(hi, lo, hi) == 0xFFFF
    assert uint_to_float(0, lo, hi) == pytest.approx(lo)
    assert uint_to_float(0xFFFF, lo, hi) == pytest.approx(hi)


def test_float_uint_clamps_out_of_range():
    assert float_to_uint(999.0, -12.57, 12.57) == 0xFFFF
    assert float_to_uint(-999.0, -12.57, 12.57) == 0


def test_float_uint_roundtrip_precision():
    lo, hi = -12.57, 12.57
    for value in (-9.0, -1.0, 0.0, 0.5, 7.25):
        back = uint_to_float(float_to_uint(value, lo, hi), lo, hi)
        assert back == pytest.approx(value, abs=(hi - lo) / 65535)


def test_motion_control_layout():
    """トルクは拡張 ID のデータ領域2、他は 8 バイトへ上位バイト先行で入る。"""
    pr = get_profile("RS00")
    ext_id, data = encode_motion_control(
        torque=0.0, position=0.0, velocity=0.0, kp=0.0, kd=0.0, motor_id=0x7F,
        p_min=pr.p_min, p_max=pr.p_max, v_min=pr.v_min, v_max=pr.v_max,
        t_min=pr.t_min, t_max=pr.t_max, kp_max=pr.kp_max, kd_max=pr.kd_max,
    )
    comm_type, data2, target = unpack_ext_id(ext_id)
    assert comm_type == CommType.MOTION_CONTROL
    assert target == 0x7F
    # 0Nm は ±14Nm レンジの中央 = 0x7FFF
    assert data2 == float_to_uint(0.0, pr.t_min, pr.t_max) == 0x7FFF
    pos, vel, kp, kd = struct.unpack(">HHHH", data)
    assert (pos, vel) == (0x7FFF, 0x7FFF)     # 中央
    assert (kp, kd) == (0, 0)                 # 0 は下限


def test_motion_control_position_is_big_endian():
    pr = get_profile("RS00")
    _, data = encode_motion_control(
        torque=0.0, position=pr.p_max, velocity=pr.v_min, kp=pr.kp_max, kd=0.0,
        motor_id=1, p_min=pr.p_min, p_max=pr.p_max, v_min=pr.v_min, v_max=pr.v_max,
        t_min=pr.t_min, t_max=pr.t_max, kp_max=pr.kp_max, kd_max=pr.kd_max,
    )
    assert data[0:2] == b"\xff\xff"    # 位置上限
    assert data[2:4] == b"\x00\x00"    # 速度下限
    assert data[4:6] == b"\xff\xff"    # Kp 上限
    assert data[6:8] == b"\x00\x00"    # Kd 0


def test_decode_feedback_rs00():
    """RS00 レンジ (角度 ±4π / 速度 ±33 / トルク ±14) での復号。"""
    pr = get_profile("RS00")
    ext_id = pack_ext_id(CommType.FEEDBACK, 0x0000, 0xFD)
    ext_id |= 0x0C << 8              # bit15-8 = motor id 0x0C
    ext_id |= (1 << 18)              # bit18 = 過温度
    ext_id |= (2 << 22)              # bit23-22 = Run
    data = struct.pack(">HHHH", 0xFFFF, 0x0000, 0x7FFF, 253)
    fb = decode_feedback(CanFrame(ext_id, data), pr.p_min, pr.p_max,
                         pr.v_min, pr.v_max, pr.t_min, pr.t_max)
    assert fb.motor_id == 0x0C
    assert fb.angle == pytest.approx(12.57)
    assert fb.velocity == pytest.approx(-33.0)
    assert fb.torque == pytest.approx(0.0, abs=1e-3)
    assert fb.temperature == pytest.approx(25.3)
    assert fb.mode == 2 and fb.mode_name == "Run"
    assert "過温度" in fb.faults


def test_decode_feedback_uses_model_ranges():
    """同じ生値でも機種プロファイルが違えば物理量が変わる。"""
    data = struct.pack(">HHHH", 0xFFFF, 0xFFFF, 0xFFFF, 250)
    frame = CanFrame(pack_ext_id(CommType.FEEDBACK, 0x01, 0xFD), data)
    rs00, rs04 = get_profile("RS00"), get_profile("RS04")
    a = decode_feedback(frame, rs00.p_min, rs00.p_max, rs00.v_min, rs00.v_max,
                        rs00.t_min, rs00.t_max)
    b = decode_feedback(frame, rs04.p_min, rs04.p_max, rs04.v_min, rs04.v_max,
                        rs04.t_min, rs04.t_max)
    assert a.torque == pytest.approx(14.0)
    assert b.torque == pytest.approx(120.0)
    assert a.velocity == pytest.approx(33.0)
    assert b.velocity == pytest.approx(15.0)


def test_decode_fault_frame():
    data = struct.pack("<I", (1 << 0) | (1 << 3)) + struct.pack("<I", 1 << 0)
    d = decode_fault_frame(CanFrame(pack_ext_id(CommType.FAULT_FEEDBACK, 0x02, 0xFD), data))
    assert d["motor_id"] == 0x02
    assert "モータ過温度 (既定 145℃)" in d["faults"]
    assert "過電圧故障" in d["faults"]
    assert d["warnings"] == ["モータ過温度警告 (既定 135℃)"]


# ---------------------------------------------------------------------------
# パラメータ表
# ---------------------------------------------------------------------------

def test_param_value_codec_roundtrip():
    assert P.decode_value(P.TYPE_F32, P.encode_value(P.TYPE_F32, 1.5)) == pytest.approx(1.5)
    assert P.decode_value(P.TYPE_U8, P.encode_value(P.TYPE_U8, 5)) == 5
    assert P.decode_value(P.TYPE_U16, P.encode_value(P.TYPE_U16, 1234)) == 1234
    assert P.decode_value(P.TYPE_U32, P.encode_value(P.TYPE_U32, 20000)) == 20000


def test_param_value_is_always_four_bytes():
    for ptype, value in [(P.TYPE_U8, 3), (P.TYPE_U16, 9), (P.TYPE_U32, 7), (P.TYPE_F32, 1.0)]:
        assert len(P.encode_value(ptype, value)) == 4


def test_run_mode_write_matches_manual_bytes():
    """run_mode=1 の書込データがマニュアル実例と一致すること。"""
    data = struct.pack("<H", 0x7005) + b"\x00\x00" + P.encode_value(P.TYPE_U8, 1)
    assert data == MANUAL_DATA


def test_schema_resolves_model_specific_ranges():
    rs00 = P.schema_for(get_profile("RS00"))
    by_name = {p["name"]: p for p in rs00}
    assert by_name["limit_torque"]["max"] == 14.0        # RS00 の T_MAX
    assert by_name["spd_ref"]["max"] == 33.0             # RS00 の V_MAX
    assert by_name["iq_ref"]["max"] == 16.0              # RS00 のパラメータ表
    assert by_name["mechPos"]["writable"] is False

    rs04 = {p["name"]: p for p in P.schema_for(get_profile("RS04"))}
    assert rs04["limit_torque"]["max"] == 120.0
    # 電流レンジは RS00 以外は原典未確認 -> None (UI 側で制限しない)
    assert rs04["iq_ref"]["max"] is None


def test_all_profiles_have_sane_ranges():
    for key, pr in PROFILES.items():
        assert pr.p_max == pytest.approx(12.57), key
        assert pr.p_min == -pr.p_max, key
        assert pr.v_max > 0 and pr.v_min == -pr.v_max, key
        assert pr.t_max > 0 and pr.t_min == -pr.t_max, key
        assert pr.kp_max > 0 and pr.kd_max > 0, key


def test_unknown_model_falls_back_to_default():
    assert get_profile("RS99").key == get_profile(None).key


def test_param_indices_are_unique():
    indices = [p.index for p in P.PARAMS]
    assert len(indices) == len(set(indices))


def test_looks_like_frame_detects_header_corruption():
    """ヘッダだけ化けた実機由来のバイト列を「構造は正しい」と判定できること。

    実機 (CH340 @ 1Mbps) で観測された、末尾 CRLF と DLC=8 は正しいのに
    先頭 2 バイトの上位ビットが化けたフレーム。
    """
    corrupted = bytes.fromhex("8194000377F408D1B67B0A00F430150D0A")
    assert len(corrupted) == 17
    assert looks_like_frame(corrupted)
    assert FrameParser().feed(corrupted) == [("junk", corrupted)]


def test_looks_like_frame_rejects_non_frames():
    assert not looks_like_frame(b"")
    assert not looks_like_frame(b"hello")
    assert not looks_like_frame(bytes(17))                       # 末尾が CRLF でない
    assert not looks_like_frame(bytes(6) + bytes([7]) + bytes(8) + b"\r\n")  # DLC != 8
    assert looks_like_frame(MANUAL_FRAME)                        # 正常フレームも当然 True
