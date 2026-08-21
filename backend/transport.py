"""USB-CAN アダプタとのシリアル接続。

送信は同期、受信は専用スレッド。受信フレームは
  1. 応答待ち (waiter) へのマッチング
  2. 購読者コールバック (ログ/テレメトリ配信)
の順で配られる。
"""

from __future__ import annotations

import errno
import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

import serial
from serial.tools import list_ports

from .protocol import CanFrame, FrameParser, encode_frame

log = logging.getLogger(__name__)

#: 純正 USB-CAN アダプタは CH340。既定 921600bps。
DEFAULT_BAUDRATE = 921600
SUPPORTED_BAUDRATES = [115200, 230400, 460800, 921600, 1000000]

#: CH340 / CP210x / FTDI の VID:PID。ポート候補の並べ替えに使う。
_KNOWN_USB_SERIAL = {
    (0x1A86, 0x7523): "CH340",
    (0x1A86, 0x5523): "CH341",
    (0x1A86, 0x55D4): "CH9102",
    (0x10C4, 0xEA60): "CP210x",
    (0x0403, 0x6001): "FT232",
}


class TransportError(RuntimeError):
    pass


@dataclass
class _Waiter:
    predicate: Callable[[CanFrame], bool]
    event: threading.Event
    frame: Optional[CanFrame] = None


def list_serial_ports() -> List[dict]:
    """利用可能なシリアルポートを列挙する。USB-CAN らしいものを先頭に。"""
    ports = []
    for p in list_ports.comports():
        chip = _KNOWN_USB_SERIAL.get((p.vid, p.pid)) if p.vid is not None else None
        ports.append({
            "device": p.device,
            "description": p.description or "",
            "manufacturer": p.manufacturer or "",
            "vid": p.vid,
            "pid": p.pid,
            "chip": chip,
            "likely_adapter": chip is not None,
        })
    ports.sort(key=lambda x: (not x["likely_adapter"], x["device"]))
    return ports


class SerialTransport:
    """1 本のシリアルポートに対する送受信。"""

    def __init__(self) -> None:
        self._ser: Optional[serial.Serial] = None
        self._parser = FrameParser()
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._waiters: List[_Waiter] = []
        self._waiters_lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._subscribers: List[Callable[[str, object], None]] = []
        self.port: Optional[str] = None
        self.baudrate: int = DEFAULT_BAUDRATE
        self.rx_count = 0
        self.tx_count = 0
        self.junk_count = 0
        self.last_error: Optional[str] = None

    # -- 接続管理 ---------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def open(self, port: str, baudrate: int = DEFAULT_BAUDRATE) -> None:
        self.close()
        try:
            self._ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.05,
                                      write_timeout=1.0)
        except OSError as exc:
            # macOS では非標準ボーレートの設定に IOSSIOSPEED を使うため、
            # 対応していないデバイス (PTY など) では Errno 25 になる。
            # SerialException は OSError の派生なのでここで両方を拾う。
            if getattr(exc, "errno", None) == errno.ENOTTY:
                raise TransportError(
                    f"{port} は {baudrate} bps に対応していません。"
                    f"標準的な速度 (115200 など) を指定してください"
                ) from exc
            raise TransportError(f"ポート {port} を開けません: {exc}") from exc
        self.port = port
        self.baudrate = baudrate
        self.rx_count = self.tx_count = self.junk_count = 0
        self.last_error = None
        self._parser = FrameParser()
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="usbcan-rx", daemon=True)
        self._rx_thread.start()
        log.info("シリアルポート %s @ %d を開きました", port, baudrate)

    def close(self) -> None:
        self._stop.set()
        thread, self._rx_thread = self._rx_thread, None
        if thread and thread.is_alive():
            thread.join(timeout=1.0)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # クローズ時の失敗は握りつぶす
                pass
        self._ser = None
        with self._waiters_lock:
            for w in self._waiters:
                w.event.set()
            self._waiters.clear()

    # -- 購読 -------------------------------------------------------------
    def subscribe(self, cb: Callable[[str, object], None]) -> None:
        self._subscribers.append(cb)

    def unsubscribe(self, cb: Callable[[str, object], None]) -> None:
        if cb in self._subscribers:
            self._subscribers.remove(cb)

    def _publish(self, kind: str, payload: object) -> None:
        for cb in list(self._subscribers):
            try:
                cb(kind, payload)
            except Exception:
                log.exception("購読者コールバックで例外")

    # -- 送信 -------------------------------------------------------------
    def send_frame(self, ext_id: int, data: bytes, extended: bool = True) -> None:
        if not self.connected:
            raise TransportError("シリアルポートが開かれていません")
        raw = encode_frame(ext_id, data, extended)
        with self._tx_lock:
            try:
                self._ser.write(raw)
            except serial.SerialException as exc:
                raise TransportError(f"送信に失敗しました: {exc}") from exc
        self.tx_count += 1
        self._publish("tx", CanFrame(ext_id=ext_id, data=bytes(data), extended=extended))

    def send_raw(self, raw: bytes) -> None:
        """AT コマンドなど、CAN フレーム以外のバイト列をそのまま送る。"""
        if not self.connected:
            raise TransportError("シリアルポートが開かれていません")
        with self._tx_lock:
            self._ser.write(raw)
        self._publish("tx_raw", raw.hex(" ").upper())

    # -- 応答待ち ---------------------------------------------------------
    def request(self, ext_id: int, data: bytes,
                predicate: Callable[[CanFrame], bool],
                timeout: float = 0.5) -> Optional[CanFrame]:
        """送信して条件に合う受信フレームを 1 つ待つ。"""
        waiter = _Waiter(predicate=predicate, event=threading.Event())
        with self._waiters_lock:
            self._waiters.append(waiter)
        try:
            self.send_frame(ext_id, data)
            waiter.event.wait(timeout)
            return waiter.frame
        finally:
            with self._waiters_lock:
                if waiter in self._waiters:
                    self._waiters.remove(waiter)

    # -- 受信ループ -------------------------------------------------------
    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            ser = self._ser
            if ser is None:
                break
            try:
                chunk = ser.read(256)
                if not chunk:
                    # timeout=0.05 なので空読みは正常。取りこぼし分を追加取得。
                    waiting = ser.in_waiting
                    if waiting:
                        chunk = ser.read(waiting)
                    else:
                        continue
            except serial.SerialException as exc:
                self.last_error = str(exc)
                self._publish("error", f"受信エラー: {exc}")
                break
            except Exception as exc:  # ポートが閉じられた等
                if not self._stop.is_set():
                    self.last_error = str(exc)
                break

            for kind, payload in self._parser.feed(chunk):
                if kind == "frame":
                    self.rx_count += 1
                    self._dispatch(payload)  # type: ignore[arg-type]
                else:
                    self.junk_count += len(payload)  # type: ignore[arg-type]
                    self._publish("junk", payload)

    def _dispatch(self, frame: CanFrame) -> None:
        with self._waiters_lock:
            for w in self._waiters:
                if not w.event.is_set() and w.predicate(frame):
                    w.frame = frame
                    w.event.set()
                    break
        self._publish("rx", frame)


def wait_for_settle(seconds: float = 0.05) -> None:
    time.sleep(seconds)
