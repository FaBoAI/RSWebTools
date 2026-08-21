#!/usr/bin/env python3
"""backend の機種プロファイルとパラメータ表から webserial/tables.js を生成する。

Python 版 (ローカル) と Web Serial 版 (静的ホスティング) で同じ表を使うため、
**backend/models.py と backend/params.py を唯一の情報源**とし、JS 側は
生成物を読む。手で二重管理するとレンジが食い違い、角度・速度・トルクの
換算がずれる。
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from backend import params as P  # noqa: E402
from backend.models import DEFAULT_PROFILE, PROFILES  # noqa: E402

OUT = Path(__file__).resolve().parent.parent / "webserial" / "tables.js"

HEADER = """// 自動生成ファイル - 直接編集しないこと
// backend/models.py と backend/params.py から生成:
//   python tools/gen_js_tables.py
"""


def main() -> None:
    profiles = {k: v.as_dict() for k, v in PROFILES.items()}
    params = [
        {
            "index": p.index,
            "indexHex": f"0x{p.index:04X}",
            "name": p.name,
            "label": p.label,
            "type": p.type,
            "rw": p.rw,
            "writable": p.writable,
            "group": p.group,
            "unit": p.unit,
            "min": p.min,
            "max": p.max,
            "default": p.default,
            "step": p.step,
            "choices": p.choices,
            "note": p.note,
        }
        for p in P.PARAMS
    ]

    def dump(obj):
        return json.dumps(obj, ensure_ascii=False, indent=2)

    OUT.write_text(
        HEADER
        + f"\nexport const DEFAULT_PROFILE = {json.dumps(DEFAULT_PROFILE)};\n"
        + f"\nexport const PROFILES = {dump(profiles)};\n"
        + f"\nexport const PARAMS = {dump(params)};\n"
        + f"\nexport const GROUP_LABELS = {dump(P.GROUP_LABELS)};\n"
        + f"\nexport const RUN_MODE_CHOICES = {dump(P.RUN_MODE_CHOICES)};\n"
        + "\n// レンジのプレースホルダ (機種プロファイルで実行時に解決する)\n"
        + f"export const PLACEHOLDERS = {dump({'V_LIM': P.V_LIM, 'V_POS': P.V_POS, 'T_POS': P.T_POS, 'I_LIM': P.I_LIM, 'I_POS': P.I_POS})};\n",
        encoding="utf-8",
    )
    print(f"生成: {OUT}  ({len(params)} パラメータ / {len(profiles)} 機種)")


if __name__ == "__main__":
    main()
