#!/usr/bin/env python3
"""CAN バス総当たり診断。

モータが応答しないとき、どのプロトコルなら反応するかを調べる。
すべて読み出し / 停止系のみで、モータを動かす指令は送らない。

  使い方:
    python tools/probe_bus.py /dev/cu.usbserial-XXX            # 能動プローブ
    python tools/probe_bus.py /dev/cu.usbserial-XXX --listen 30 # 受信待ち受けのみ
"""
import argparse
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from backend.protocol import CommType, pack_ext_id  # noqa: E402
from backend.transport import SerialTransport  # noqa: E402

received = []


junk = bytearray()


def on_event(kind, payload):
    if kind == "rx":
        received.append(payload)
        print(f"    <<< フレーム受信: {payload}")
    elif kind == "junk":
        junk.extend(payload)
        print(f"    <<< 未解析 {len(payload)}B: {bytes(payload).hex(' ').upper()}")
    elif kind == "error":
        print(f"    <<< error: {payload}")


def banner(title):
    print(f"\n--- {title} ---")


def listen(t, seconds, note=""):
    banner(f"パッシブ受信 {seconds}s {note}")
    before = len(received)
    end = time.time() + seconds
    while time.time() < end:
        time.sleep(0.1)
    got = len(received) - before
    print(f"  → {got} フレーム受信" if got else "  → 何も受信せず")
    return got


def probe_private(t, ids, host_id=0xFD):
    """RobStride 私有プロトコル (拡張フレーム)。"""
    banner(f"私有プロトコル / 拡張フレーム / host=0x{host_id:02X}")
    before = len(received)
    for mid in ids:
        # 通信タイプ 0: デバイス ID 取得 (読み出しのみ)
        t.send_frame(pack_ext_id(CommType.GET_DEVICE_ID, host_id, mid), b"\x00" * 8)
        time.sleep(0.012)
        # 通信タイプ 17: run_mode 読み出し (読み出しのみ)
        t.send_frame(pack_ext_id(CommType.READ_PARAM, host_id, mid),
                     struct.pack("<H", 0x7005) + b"\x00" * 6)
        time.sleep(0.012)
    time.sleep(0.3)
    return len(received) - before


def probe_mit(t, ids):
    """MIT プロトコル (11bit 標準フレーム)。

    コマンド 2 = 停止 (FF..FD) とコマンド 5 = 故障値読み出し (F_CMD=0x00 なので
    クリアせず値を返すだけ) を使う。どちらもモータを動かさない。
    """
    banner("MIT プロトコル / 標準フレーム")
    before = len(received)
    stop_cmd = bytes([0xFF] * 7 + [0xFD])
    fault_cmd = bytes([0xFF] * 6 + [0x00, 0xFB])
    for mid in ids:
        t.send_frame(mid, stop_cmd, extended=False)
        time.sleep(0.012)
        t.send_frame(mid, fault_cmd, extended=False)
        time.sleep(0.012)
    time.sleep(0.3)
    return len(received) - before


def probe_canopen(t, ids):
    """CANopen (11bit 標準フレーム)。SDO で 0x1000 Device Type を読む。"""
    banner("CANopen / 標準フレーム / SDO 0x1000 読み出し")
    before = len(received)
    sdo_read = bytes([0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00])
    for node in ids:
        if node == 0:
            continue
        t.send_frame(0x600 + node, sdo_read, extended=False)
        time.sleep(0.015)
    # NMT でノードをリセットせず、状態問い合わせだけ行う
    time.sleep(0.3)
    return len(received) - before


#: 試すシリアル速度。純正モジュールの既定は 921600。
CANDIDATE_BAUDS = [921600, 1_000_000, 115200, 460800, 230400, 500000,
                   250000, 57600, 2_000_000, 38400, 9600]


def sweep(port, max_id=15):
    """シリアル速度を変えながら能動ポーリングし、応答が返る速度を探す。

    UART の速度が合っていないと、こちらの送信フレームをモジュールが解釈できず
    CAN に流れない。その結果モータは一切応答しないが、モータが自発送信した
    フレームは「壊れたバイト列」として届く — という症状になる。
    """
    print("シリアル速度を掃引します (各速度で ID 0-%d をポーリング)\n" % max_id)
    results = []
    for baud in CANDIDATE_BAUDS:
        received.clear()
        junk.clear()
        t = SerialTransport()
        t.subscribe(on_event)
        try:
            t.open(port, baud)
        except Exception as exc:
            print(f"  {baud:>9}: オープン失敗 {exc}")
            continue
        time.sleep(0.2)
        for mid in range(0, max_id + 1):
            t.send_frame(pack_ext_id(CommType.GET_DEVICE_ID, 0xFD, mid), b"\x00" * 8)
            time.sleep(0.02)
            t.send_frame(pack_ext_id(CommType.READ_PARAM, 0xFD, mid),
                         struct.pack("<H", 0x7005) + b"\x00" * 6)
            time.sleep(0.02)
        time.sleep(0.4)
        nf, nj = len(received), len(junk)
        results.append((baud, nf, nj))
        mark = "  <<< 正常なフレームを受信!" if nf else ""
        print(f"  {baud:>9}: フレーム {nf} / 未解析 {nj}B{mark}")
        t.close()

    print("\n=========== 掃引結果 ===========")
    ok = [r for r in results if r[1] > 0]
    noisy = [r for r in results if r[1] == 0 and r[2] > 0]
    if ok:
        best = max(ok, key=lambda r: r[1])
        print(f"  正しいシリアル速度: {best[0]} bps (フレーム {best[1]} 件)")
        print(f"  → run.sh 起動後、UI のシリアル速度を {best[0]} にして接続してください")
    elif noisy:
        print("  どの速度でも正常フレームは得られませんでしたが、以下でバイトは届いています:")
        for baud, _, nj in noisy:
            print(f"    {baud} bps: {nj} バイト")
        print("  → 速度が候補外か、フレーム形式が異なる可能性があります")
    else:
        print("  どの速度でも 1 バイトも受信できませんでした")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baudrate", type=int, default=921600)
    ap.add_argument("--listen", type=int, default=0,
                    help="受信待ち受けのみを指定秒数行う (電源投入時のフレームを捕まえる用)")
    ap.add_argument("--max-id", type=int, default=127)
    ap.add_argument("--sweep", action="store_true",
                    help="シリアル速度を掃引しながら能動ポーリングし、正しい速度を特定する")
    ap.add_argument("--raw", type=int, default=0,
                    help="指定秒数、受信バイトを 16 進でそのまま出力する")
    args = ap.parse_args()

    if args.sweep:
        sweep(args.port, args.max_id)
        return

    t = SerialTransport()
    t.subscribe(on_event)
    t.open(args.port, args.baudrate)
    print(f"{args.port} @ {args.baudrate} を開きました")
    ids = range(0, args.max_id + 1)

    try:
        if args.raw:
            banner(f"生バイト取得 {args.raw}s @ {args.baudrate}")
            print("★ この間にモータの電源を入れ直してください ★")
            listen(t, args.raw)
            print(f"\n  未解析バイト合計: {len(junk)}")
            if junk:
                print(f"  16 進: {bytes(junk).hex(' ').upper()}")
        elif args.listen:
            print("\n★ いまモータの電源を入れ直してください ★")
            listen(t, args.listen, "(電源投入フレーム待ち)")
        else:
            listen(t, 2, "(バス上の自発送信を確認)")
            n1 = probe_private(t, ids)
            n2 = probe_mit(t, ids)
            n3 = probe_canopen(t, ids)

            print("\n=========== 結果 ===========")
            print(f"  私有プロトコル (拡張): {n1} 応答")
            print(f"  MIT      (標準)      : {n2} 応答")
            print(f"  CANopen  (標準)      : {n3} 応答")
            print(f"  送信フレーム総数     : {t.tx_count}")
            print(f"  受信フレーム総数     : {t.rx_count}")
            if not received:
                print("\n  どのプロトコルでも応答なし。")
                print("  → モータの電源 / CAN H・L / CAN ボーレート / 終端抵抗 を確認してください。")
                print(f"  → 電源投入時のフレームを捕まえる: "
                      f"python tools/probe_bus.py {args.port} --listen 30")
            else:
                print(f"\n  応答あり ({len(received)} フレーム)。上のログを確認してください。")
    finally:
        t.close()


if __name__ == "__main__":
    main()
