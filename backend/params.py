"""読み書き可能パラメータ表 (通信タイプ 0x11 / 0x12)。

内容は公式マニュアル「Read and write a single parameter list」の表に準拠。
レンジのうち機種依存のもの (速度・トルク・電流) は MotorProfile から
実行時に埋める。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import struct

from .models import MotorProfile

#: 値の型。ワイヤ上は必ず Byte4-7 の 4 バイト枠にリトルエンディアンで載る。
TYPE_U8 = "uint8"
TYPE_U16 = "uint16"
TYPE_U32 = "uint32"
TYPE_F32 = "float"

#: 機種依存レンジのプレースホルダ
V_LIM = "@v"      # ±v_max
V_POS = "@v+"     # 0..v_max
T_POS = "@t+"     # 0..t_max
I_LIM = "@i"      # ±i_max
I_POS = "@i+"     # 0..i_max


@dataclass(frozen=True)
class ParamDef:
    index: int
    name: str
    label: str            # 日本語表示名
    type: str
    rw: str               # "rw" | "r"
    group: str
    unit: str = ""
    min: Optional[object] = None   # 数値 or プレースホルダ文字列
    max: Optional[object] = None
    default: Optional[float] = None
    step: Optional[float] = None
    choices: Optional[List[dict]] = None
    note: str = ""

    @property
    def writable(self) -> bool:
        return self.rw == "rw"


RUN_MODE_CHOICES = [
    {"value": 0, "label": "0: 運動制御 (Operation)"},
    {"value": 1, "label": "1: 位置 PP (台形加減速)"},
    {"value": 2, "label": "2: 速度 (Speed)"},
    {"value": 3, "label": "3: 電流 (Current)"},
    {"value": 5, "label": "5: 位置 CSP (速度制限付き)"},
]

ON_OFF = [{"value": 0, "label": "0: 無効"}, {"value": 1, "label": "1: 有効"}]

PARAMS: List[ParamDef] = [
    ParamDef(0x7005, "run_mode", "運転モード", TYPE_U8, "rw", "mode",
             choices=RUN_MODE_CHOICES,
             note="モード切替は必ず停止状態で行うこと"),

    # --- 各モードの指令値 -------------------------------------------------
    ParamDef(0x7006, "iq_ref", "電流指令 Iq", TYPE_F32, "rw", "command",
             unit="A", min=I_LIM, max=I_LIM, step=0.1, note="電流モード用"),
    ParamDef(0x700A, "spd_ref", "速度指令", TYPE_F32, "rw", "command",
             unit="rad/s", min=V_LIM, max=V_LIM, step=0.1, note="速度モード用"),
    ParamDef(0x7016, "loc_ref", "位置指令", TYPE_F32, "rw", "command",
             unit="rad", min=-12.57, max=12.57, step=0.01, note="位置モード用"),

    # --- 制限値 -----------------------------------------------------------
    ParamDef(0x700B, "limit_torque", "トルク制限", TYPE_F32, "rw", "limit",
             unit="Nm", min=0, max=T_POS, step=0.1),
    ParamDef(0x7017, "limit_spd", "速度制限 (CSP)", TYPE_F32, "rw", "limit",
             unit="rad/s", min=0, max=V_POS, step=0.1),
    ParamDef(0x7018, "limit_cur", "電流制限 (速度/位置)", TYPE_F32, "rw", "limit",
             unit="A", min=0, max=I_POS, step=0.1),

    # --- ゲイン -----------------------------------------------------------
    ParamDef(0x7010, "cur_kp", "電流ループ Kp", TYPE_F32, "rw", "gain",
             min=0, max=10, step=0.001, default=0.17),
    ParamDef(0x7011, "cur_ki", "電流ループ Ki", TYPE_F32, "rw", "gain",
             min=0, max=10, step=0.001, default=0.012),
    ParamDef(0x7014, "cur_filt_gain", "電流フィルタゲイン", TYPE_F32, "rw", "gain",
             min=0, max=1.0, step=0.001, default=0.1),
    ParamDef(0x701E, "loc_kp", "位置ループ Kp", TYPE_F32, "rw", "gain",
             min=0, max=1000, step=0.1, default=40),
    ParamDef(0x701F, "spd_kp", "速度ループ Kp", TYPE_F32, "rw", "gain",
             min=0, max=1000, step=0.01, default=6),
    ParamDef(0x7020, "spd_ki", "速度ループ Ki", TYPE_F32, "rw", "gain",
             min=0, max=100, step=0.001, default=0.02),
    ParamDef(0x7021, "spd_filt_gain", "速度フィルタゲイン", TYPE_F32, "rw", "gain",
             min=0, max=1.0, step=0.001, default=0.1),

    # --- 動作プロファイル -------------------------------------------------
    ParamDef(0x7022, "acc_rad", "加速度 (速度モード)", TYPE_F32, "rw", "profile",
             unit="rad/s^2", min=0, max=1000, step=0.1, default=20),
    ParamDef(0x7024, "vel_max", "最大速度 (PP)", TYPE_F32, "rw", "profile",
             unit="rad/s", min=0, max=V_POS, step=0.1, default=10),
    ParamDef(0x7025, "acc_set", "加速度 (PP)", TYPE_F32, "rw", "profile",
             unit="rad/s^2", min=0, max=1000, step=0.1, default=10),

    # --- 動作設定 ---------------------------------------------------------
    ParamDef(0x7026, "EPScan_time", "能動送信周期", TYPE_U16, "rw", "config",
             min=1, max=1000, step=1, default=1,
             note="1 = 10ms、以降 +1 ごとに 5ms 増加"),
    ParamDef(0x7028, "canTimeout", "CAN タイムアウト閾値", TYPE_U32, "rw", "config",
             min=0, max=4_000_000, step=1, default=0,
             note="20000 = 1s、0 で無効。超過するとモータは Reset モードへ"),
    ParamDef(0x7029, "zero_sta", "原点フラグ", TYPE_U8, "rw", "config", default=0,
             choices=[{"value": 0, "label": "0: 電源投入時 0〜2π"},
                      {"value": 1, "label": "1: 電源投入時 -π〜π"}],
             note="変更後は通信タイプ 22 で保存が必要"),
    ParamDef(0x702A, "damper", "ダンパースイッチ", TYPE_U8, "rw", "config", default=0,
             choices=[{"value": 0, "label": "0: 電源断時のダンピング有効 (既定)"},
                      {"value": 1, "label": "1: ダンピング無効"}]),
    ParamDef(0x702B, "add_offset", "原点オフセット", TYPE_F32, "rw", "config",
             unit="rad", min=-12.57, max=12.57, step=0.01, default=0,
             note="現在の原点を (現在位置 + offset) へずらす"),

    # --- 読み出し専用 -----------------------------------------------------
    ParamDef(0x7019, "mechPos", "出力軸機械角", TYPE_F32, "r", "monitor", unit="rad"),
    ParamDef(0x701A, "iqf", "Iq (フィルタ後)", TYPE_F32, "r", "monitor", unit="A"),
    ParamDef(0x701B, "mechVel", "出力軸速度", TYPE_F32, "r", "monitor", unit="rad/s"),
    ParamDef(0x701C, "VBUS", "バス電圧", TYPE_F32, "r", "monitor", unit="V"),

    # 公式 Python SDK (RobStride/Python_Sample) が使用している実測値インデックス。
    # マニュアルの読み書きパラメータ表には載っていないが SDK 側で定義されている。
    ParamDef(0x3016, "mechPos_measured", "実測位置", TYPE_F32, "r", "monitor", unit="rad",
             note="公式 Python SDK の MEASURED_POSITION"),
    ParamDef(0x3017, "mechVel_measured", "実測速度", TYPE_F32, "r", "monitor", unit="rad/s",
             note="公式 Python SDK の MEASURED_VELOCITY"),
    ParamDef(0x302C, "torque_fdb", "実測トルク", TYPE_F32, "r", "monitor", unit="Nm",
             note="公式 Python SDK の MEASURED_TORQUE"),

    # 磁気エンコーダのゼロ点オフセット (0x20xx 系 = 保存対象)。
    ParamDef(0x2005, "mechOffset", "磁気エンコーダ角オフセット", TYPE_F32, "rw", "config",
             unit="rad", min=-7, max=7, step=0.000001,
             note="キャリブレーション値。通常は変更しないこと"),
]

PARAMS_BY_NAME: Dict[str, ParamDef] = {p.name: p for p in PARAMS}
PARAMS_BY_INDEX: Dict[int, ParamDef] = {p.index: p for p in PARAMS}

GROUP_LABELS = {
    "mode": "運転モード",
    "command": "指令値",
    "limit": "制限値",
    "gain": "制御ゲイン",
    "profile": "動作プロファイル",
    "config": "動作設定 (保存対象)",
    "monitor": "モニタ (読み出し専用)",
}


def resolve(value, profile: MotorProfile, positive: bool):
    """レンジのプレースホルダを機種プロファイルで解決する。"""
    if not isinstance(value, str):
        return value
    if value == V_LIM:
        return profile.v_max if positive else profile.v_min
    if value == V_POS:
        return profile.v_max
    if value == T_POS:
        return profile.t_max
    if value in (I_LIM, I_POS):
        if profile.i_max is None:
            return None  # 不明: UI 側でレンジ制限しない
        if value == I_POS:
            return profile.i_max
        return profile.i_max if positive else -profile.i_max
    return None


def schema_for(profile: MotorProfile) -> List[dict]:
    """フロントエンドに渡すパラメータ定義 (レンジ解決済み)。"""
    out = []
    for p in PARAMS:
        out.append({
            "index": p.index,
            "index_hex": f"0x{p.index:04X}",
            "name": p.name,
            "label": p.label,
            "type": p.type,
            "rw": p.rw,
            "writable": p.writable,
            "group": p.group,
            "group_label": GROUP_LABELS[p.group],
            "unit": p.unit,
            "min": resolve(p.min, profile, positive=False),
            "max": resolve(p.max, profile, positive=True),
            "default": p.default,
            "step": p.step,
            "choices": p.choices,
            "note": p.note,
        })
    return out


# --------------------------------------------------------------------------
# 値 <-> 4 バイト枠
# --------------------------------------------------------------------------

def encode_value(ptype: str, value) -> bytes:
    """パラメータ値を Byte4-7 の 4 バイト枠へ (リトルエンディアン)。"""
    if ptype == TYPE_F32:
        return struct.pack("<f", float(value))
    if ptype == TYPE_U8:
        return bytes([int(value) & 0xFF, 0, 0, 0])
    if ptype == TYPE_U16:
        return struct.pack("<H", int(value) & 0xFFFF) + b"\x00\x00"
    if ptype == TYPE_U32:
        return struct.pack("<I", int(value) & 0xFFFFFFFF)
    raise ValueError(f"未知のパラメータ型: {ptype}")


def decode_value(ptype: str, raw: bytes):
    raw = (bytes(raw) + b"\x00" * 4)[:4]
    if ptype == TYPE_F32:
        return struct.unpack("<f", raw)[0]
    if ptype == TYPE_U8:
        return raw[0]
    if ptype == TYPE_U16:
        return struct.unpack("<H", raw[:2])[0]
    if ptype == TYPE_U32:
        return struct.unpack("<I", raw)[0]
    raise ValueError(f"未知のパラメータ型: {ptype}")
