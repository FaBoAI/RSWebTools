#!/usr/bin/env python3
"""受信バイトを一切加工せず 16 進で記録する。

「フレームとして解釈できないが何かは届いている」ときに、実際のバイト値を
確認するためのツール。ボーレート違いなのか、フレーム形式違いなのかを
判定する材料になる。
"""
import argparse
import sys
import time
from pathlib import Path

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baudrate", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    s = serial.Serial(args.port, args.baudrate, timeout=0.05)
    # 開放直後の過渡バイトだけ捨てる (ボーレート変更時のライン揺れ)
    time.sleep(0.4)
    s.reset_input_buffer()

    print(f"{args.port} @ {args.baudrate} で {args.seconds:.0f} 秒キャプチャします")
    print("★ この間にモータの電源を入れ直してください ★", flush=True)

    chunks = []
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        r = s.read(1024)
        if r:
            ts = time.time() - t0
            chunks.append((ts, r))
            print(f"  [{ts:6.2f}s] {len(r):3d}B  {r.hex(' ').upper()}", flush=True)
    s.close()

    total = sum(len(c) for _, c in chunks)
    print(f"\n=== 合計 {total} バイト / {len(chunks)} チャンク ===")
    if not total:
        print("  何も受信しませんでした")
        return

    blob = b"".join(c for _, c in chunks)
    print(f"  連結 16 進: {blob.hex(' ').upper()}")
    print(f"  'AT'(41 54) の出現位置: {[i for i in range(len(blob)-1) if blob[i:i+2] == b'AT'] or 'なし'}")
    print(f"  CRLF(0D 0A) の出現位置: {[i for i in range(len(blob)-1) if blob[i:i+2] == b'\\r\\n'] or 'なし'}")
    hi = sum(1 for b in blob if b >= 0x80)
    print(f"  0x80 以上のバイト: {hi}/{total} ({100*hi/total:.0f}%)")

    if args.out:
        Path(args.out).write_bytes(blob)
        print(f"  生バイトを {args.out} に保存しました")


if __name__ == "__main__":
    main()
