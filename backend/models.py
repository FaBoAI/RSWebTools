"""機種別プロファイル。

P/V/T/KP/KD のレンジは各機種の公式ユーザーマニュアル
「Control mode instructions / Program sample」章に記載された
``#define P_MIN`` … の値をそのまま採用している (2025-11-12 版マニュアル)。
運動制御モードの 16bit スケーリングとフィードバックの復号に使うため、
ここが機種と一致していないと角度・速度・トルクの値がすべてずれる。

``i_max`` (相電流指令の上限) は RS00 のパラメータ表でしか原典を確認できて
いないため、他機種は None (不明) にしてある。None の場合 UI は電流値の
入力レンジを制限せず、モータから読み出した limit_cur を目安として表示する。

【公式 Python SDK との相違について】
RobStride/Python_Sample の ``robstride_dynamics/table.py`` は速度・トルクの
レンジについて一部異なる値を持つ。KP/KD はマニュアルと完全一致、RS01/RS02/RS04
の速度・トルクも一致するが、以下だけが食い違う。

    機種    マニュアル(本実装)      公式 SDK
    RS00    V=33  T=14            V=50  T=17
    RS03    V=20  T=60            V=50  T=60
    RS05    V=50  T=5.5           V=33  T=17
    RS06    V=50  T=36            V=20  T=60

RS00↔RS05 と RS03↔RS06 で速度値が入れ替わった形になっており、SDK 側の
転記ミスの可能性が高い (RS05 の T=17 は RS01/RS02 と同値で、小型機の仕様
としては不自然)。本実装は各機種の自機マニュアルに書かれた ``#define`` を
採用している。実機の挙動が合わない場合はここの数値を差し替えること。
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Dict, Optional


@dataclass(frozen=True)
class MotorProfile:
    key: str
    label: str
    p_min: float
    p_max: float
    v_min: float
    v_max: float
    t_min: float
    t_max: float
    kp_max: float
    kd_max: float
    i_max: Optional[float] = None
    note: str = ""

    def as_dict(self) -> dict:
        return asdict(self)


def _profile(key: str, label: str, v: float, t: float, kp: float, kd: float,
             i_max: Optional[float] = None, note: str = "") -> MotorProfile:
    # P レンジは全機種共通で ±12.57 rad (=±4π)。
    return MotorProfile(
        key=key, label=label,
        p_min=-12.57, p_max=12.57,
        v_min=-v, v_max=v,
        t_min=-t, t_max=t,
        kp_max=kp, kd_max=kd,
        i_max=i_max, note=note,
    )


PROFILES: Dict[str, MotorProfile] = {
    p.key: p
    for p in (
        _profile("RS00", "RobStride 00", v=33.0, t=14.0, kp=500.0, kd=5.0, i_max=16.0,
                 note="電流レンジ ±16A / トルク上限 14Nm はマニュアルのパラメータ表に記載あり"),
        _profile("RS01", "RobStride 01", v=44.0, t=17.0, kp=500.0, kd=5.0),
        _profile("RS02", "RobStride 02", v=44.0, t=17.0, kp=500.0, kd=5.0),
        _profile("RS03", "RobStride 03", v=20.0, t=60.0, kp=5000.0, kd=100.0),
        _profile("RS04", "RobStride 04", v=15.0, t=120.0, kp=5000.0, kd=100.0),
        _profile("RS05", "RobStride 05", v=50.0, t=5.5, kp=500.0, kd=5.0),
        _profile("RS06", "RobStride 06", v=50.0, t=36.0, kp=5000.0, kd=100.0),
    )
}

DEFAULT_PROFILE = "RS02"


def get_profile(key: Optional[str]) -> MotorProfile:
    return PROFILES.get((key or "").upper(), PROFILES[DEFAULT_PROFILE])
