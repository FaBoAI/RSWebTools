"""アダプタ MCU が生きているか総当たりで切り分ける。"""
import serial, time, itertools

PORT = __import__('sys').argv[1] if len(__import__('sys').argv) > 1 else '/dev/cu.usbserial-110'
BAUDS = [921600, 115200, 1000000, 460800, 230400, 57600, 38400, 9600, 2000000]

PROBES = [
    ('AT+AT',        b'AT+AT\r\n'),
    ('AT',           b'AT\r\n'),
    ('AT?',          b'AT?\r\n'),
    ('AT+VER',       b'AT+VER\r\n'),
    ('AT+RESET',     b'AT+RESET\r\n'),
    ('AT+CG',        b'AT+CG\r\n'),
    ('slcan V',      b'V\r'),        # CANable / slcan 系ならバージョンを返す
    ('slcan v',      b'v\r'),
    ('slcan CR',     b'\r'),
    ('slcan O',      b'S8\rO\r'),
    ('canfd N',      b'N\r'),
]

hits = []
for baud in BAUDS:
    try:
        s = serial.Serial(PORT, baud, timeout=0.25)
    except Exception as e:
        print(f'{baud}: open fail {e}'); continue
    s.dtr = False; s.rts = False
    time.sleep(0.15); s.reset_input_buffer()
    got_any = False
    for name, payload in PROBES:
        s.reset_input_buffer()
        s.write(payload); s.flush()
        time.sleep(0.25)
        r = s.read(4096)
        if r:
            got_any = True
            hits.append((baud, name, r))
            print(f'  !! {baud} {name}: {len(r)}B {r[:80]!r}')
    if not got_any:
        print(f'{baud}: 全プローブ無応答')
    s.close()

print()
print('=== 結果:', f'{len(hits)} 件の応答' if hits else 'どの速度・どのコマンドでもアダプタは 1 バイトも返さない')
