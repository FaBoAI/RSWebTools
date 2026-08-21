"""FastAPI アプリケーション。REST + WebSocket で Web UI に機能を公開する。"""

from __future__ import annotations

import asyncio
import logging
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from . import params as P
from .models import PROFILES, get_profile
from .motor import BAUD_CODES, PROTOCOL_CODES, MotorClient, MotorError
from .protocol import (
    DEFAULT_HOST_ID,
    CanFrame,
    CommType,
    decode_fault_frame,
    decode_feedback,
    looks_like_frame,
)
from .transport import (
    DEFAULT_BAUDRATE,
    SUPPORTED_BAUDRATES,
    SerialTransport,
    TransportError,
    list_serial_ports,
)

log = logging.getLogger(__name__)

WEB_DIR = Path(__file__).resolve().parent.parent / "web"

#: CAN ログのリングバッファ長
LOG_LIMIT = 500


class AppState:
    def __init__(self) -> None:
        self.transport = SerialTransport()
        self.client = MotorClient(self.transport)
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.websockets: List[WebSocket] = []
        self.log: List[dict] = []
        self.telemetry: Dict[int, dict] = {}
        self.armed = False          # 動作許可 (ソフトウェアインターロック)
        self.transport.subscribe(self._on_event)

    # -- transport スレッドから呼ばれる ------------------------------------
    def _on_event(self, kind: str, payload: Any) -> None:
        ts = time.time()
        entry: dict
        if kind in ("tx", "rx") and isinstance(payload, CanFrame):
            entry = {
                "t": ts, "dir": kind,
                "id": f"{payload.ext_id:08X}",
                "type": payload.comm_type,
                "type_name": _comm_name(payload.comm_type),
                "data": payload.hex(),
                "motor_id": payload.motor_id if kind == "rx" else payload.target_id,
            }
            if kind == "rx":
                self._decode_rx(payload, ts)
        elif kind == "tx_raw":
            entry = {"t": ts, "dir": "tx", "id": "-", "type": None,
                     "type_name": "RAW", "data": str(payload)}
        elif kind == "junk":
            # フレーム化できなかった受信バイト。生の 16 進で残す。
            raw = bytes(payload)
            # 長さ・DLC・末尾 CRLF が揃っているのにヘッダだけ違う = ビット化け。
            # ボーレートやリンク品質の問題を切り分けるための表示。
            label = ("ヘッダ化け (要ボーレート確認)" if looks_like_frame(raw)
                     else f"未解析 {len(raw)}B")
            entry = {"t": ts, "dir": "junk", "id": "-", "type": None,
                     "type_name": label, "data": raw.hex(" ").upper()}
        else:
            entry = {"t": ts, "dir": kind, "id": "-", "type": None,
                     "type_name": kind.upper(), "data": str(payload)}

        self.log.append(entry)
        if len(self.log) > LOG_LIMIT:
            del self.log[: len(self.log) - LOG_LIMIT]
        self._broadcast({"event": "can", "frame": entry})

    def _decode_rx(self, frame: CanFrame, ts: float) -> None:
        """フィードバック/故障フレームを解釈してテレメトリを更新する。"""
        try:
            if frame.comm_type in (CommType.FEEDBACK, CommType.ACTIVE_REPORT) \
                    and len(frame.data) >= 8:
                pr = self.client.profile(frame.motor_id)
                fb = decode_feedback(frame, pr.p_min, pr.p_max, pr.v_min, pr.v_max,
                                     pr.t_min, pr.t_max)
                data = fb.as_dict()
                data["t"] = ts
                self.telemetry[fb.motor_id] = data
                self._broadcast({"event": "telemetry", "data": data})
            elif frame.comm_type == CommType.FAULT_FEEDBACK:
                self._broadcast({"event": "fault", "data": decode_fault_frame(frame)})
        except Exception:
            log.exception("受信フレームの解釈に失敗")

    def _broadcast(self, message: dict) -> None:
        loop = self.loop
        if loop is None or loop.is_closed():
            return
        loop.call_soon_threadsafe(self._queue_broadcast, message)

    def _queue_broadcast(self, message: dict) -> None:
        for ws in list(self.websockets):
            asyncio.create_task(self._safe_send(ws, message))

    async def _safe_send(self, ws: WebSocket, message: dict) -> None:
        try:
            await ws.send_json(message)
        except Exception:
            if ws in self.websockets:
                self.websockets.remove(ws)


_COMM_NAMES = {
    0x00: "デバイスID", 0x01: "運動制御", 0x02: "フィードバック", 0x03: "運転許可",
    0x04: "停止", 0x06: "原点設定", 0x07: "CAN_ID変更", 0x11: "パラメータ読出",
    0x12: "パラメータ書込", 0x15: "故障通知", 0x16: "保存", 0x17: "ボーレート変更",
    0x18: "能動送信", 0x19: "プロトコル変更", 0x1A: "バージョン",
}


def _comm_name(t: Optional[int]) -> str:
    if t is None:
        return "-"
    return _COMM_NAMES.get(t, f"0x{t:02X}")


state = AppState()


@asynccontextmanager
async def lifespan(app: FastAPI):
    state.loop = asyncio.get_running_loop()
    yield
    state.transport.close()


app = FastAPI(title="RobStride QDD Configurator", lifespan=lifespan)


# --------------------------------------------------------------------------
# リクエストモデル
# --------------------------------------------------------------------------

class ConnectReq(BaseModel):
    port: str
    baudrate: int = DEFAULT_BAUDRATE
    host_id: int = Field(DEFAULT_HOST_ID, ge=0, le=0xFF)


class ScanReq(BaseModel):
    start: int = Field(0, ge=0, le=255)
    end: int = Field(127, ge=0, le=255)
    gap: float = Field(0.008, gt=0, le=0.1)
    settle: float = Field(0.6, gt=0, le=3.0)


class ModelReq(BaseModel):
    model: str


class ParamWriteReq(BaseModel):
    name: Optional[str] = None
    index: Optional[int] = None
    value: float
    type: Optional[str] = None


class ParamReadReq(BaseModel):
    indices: Optional[List[int]] = None


class StopReq(BaseModel):
    clear_fault: bool = False


class CanIdReq(BaseModel):
    new_id: int = Field(..., ge=0, le=127)


class ControlReq(BaseModel):
    torque: float = 0.0
    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0


class ReportReq(BaseModel):
    enable: bool


class BaudReq(BaseModel):
    baudrate: int


class ProtocolReq(BaseModel):
    protocol: str


class ArmReq(BaseModel):
    armed: bool


class RawReq(BaseModel):
    ext_id: int
    data: str = ""     # 空白区切り hex


class IndexScanReq(BaseModel):
    start: int = Field(0x2000, ge=0, le=0xFFFF)
    end: int = Field(0x202F, ge=0, le=0xFFFF)
    timeout: float = Field(0.12, gt=0, le=1.0)


# --------------------------------------------------------------------------
# 補助
# --------------------------------------------------------------------------

def _require_connection() -> None:
    if not state.transport.connected:
        raise HTTPException(409, "USB-CAN アダプタに接続されていません")


def _require_armed() -> None:
    if not state.armed:
        raise HTTPException(423, "動作許可 (ARM) が無効です。UI の動作許可を ON にしてください")


async def _call(fn, *args, **kwargs):
    try:
        return await asyncio.to_thread(fn, *args, **kwargs)
    except (MotorError, TransportError) as exc:
        raise HTTPException(502, str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from exc


# --------------------------------------------------------------------------
# 接続
# --------------------------------------------------------------------------

@app.get("/api/ports")
def api_ports():
    return {"ports": list_serial_ports(), "baudrates": SUPPORTED_BAUDRATES,
            "default_baudrate": DEFAULT_BAUDRATE}


@app.get("/api/status")
def api_status():
    t = state.transport
    return {
        "connected": t.connected,
        "port": t.port,
        "baudrate": t.baudrate,
        "host_id": state.client.host_id,
        "tx_count": t.tx_count,
        "rx_count": t.rx_count,
        "junk_count": t.junk_count,
        "last_error": t.last_error,
        "armed": state.armed,
        "motor_models": state.client.motor_models,
        "telemetry": state.telemetry,
    }


@app.post("/api/connect")
def api_connect(req: ConnectReq):
    try:
        state.transport.open(req.port, req.baudrate)
    except TransportError as exc:
        raise HTTPException(502, str(exc)) from exc
    state.client.host_id = req.host_id
    return api_status()


@app.post("/api/disconnect")
def api_disconnect():
    state.armed = False
    state.transport.close()
    return api_status()


@app.post("/api/adapter/at-mode")
def api_at_mode():
    """アダプタへ ``AT+AT`` を送り透過 (AT) モードへ入れる。

    純正モジュールは既定で AT モードのため通常は不要。無応答時の切り分け用。
    """
    _require_connection()
    state.transport.send_raw(b"AT+AT\r\n")
    return {"ok": True}


@app.post("/api/arm")
def api_arm(req: ArmReq):
    _require_connection()
    if not req.armed:
        # 解除時は保険として既知モータへ停止フレームを送る
        for motor_id in list(state.telemetry) or list(state.client.motor_models):
            try:
                state.client.stop_nowait(motor_id)
            except TransportError:
                pass
    state.armed = req.armed
    return {"armed": state.armed}


@app.post("/api/estop")
def api_estop():
    """非常停止。既知の全モータへ応答待ちなしで停止フレームを送る。"""
    _require_connection()
    state.armed = False
    ids = set(state.telemetry) | set(state.client.motor_models)
    for motor_id in sorted(ids) or [0]:
        try:
            state.client.stop_nowait(motor_id)
        except TransportError:
            pass
    return {"ok": True, "stopped": sorted(ids)}


# --------------------------------------------------------------------------
# 探索・機種
# --------------------------------------------------------------------------

@app.get("/api/models")
def api_models():
    return {"models": [p.as_dict() for p in PROFILES.values()]}


@app.post("/api/scan")
async def api_scan(req: ScanReq):
    _require_connection()
    if req.end < req.start:
        raise HTTPException(400, "end は start 以上にしてください")
    found = await _call(state.client.scan, req.start, req.end, req.gap, req.settle)
    return {"motors": found}


@app.post("/api/motor/{motor_id}/model")
def api_set_model(motor_id: int, req: ModelReq):
    if req.model.upper() not in PROFILES:
        raise HTTPException(400, f"未知の機種です: {req.model}")
    state.client.set_model(motor_id, req.model.upper())
    return {"motor_id": motor_id, "model": req.model.upper()}


@app.get("/api/motor/{motor_id}/schema")
def api_schema(motor_id: int):
    profile = state.client.profile(motor_id)
    return {
        "profile": profile.as_dict(),
        "groups": [{"key": k, "label": v} for k, v in P.GROUP_LABELS.items()],
        "params": P.schema_for(profile),
        "run_modes": P.RUN_MODE_CHOICES,
        "can_baudrates": sorted(BAUD_CODES),
        "protocols": sorted(PROTOCOL_CODES),
    }


@app.get("/api/motor/{motor_id}/info")
async def api_info(motor_id: int):
    _require_connection()
    info = await _call(state.client.get_device_id, motor_id)
    version = await _call(state.client.read_version, motor_id)
    return {
        "motor_id": motor_id,
        "device": info.__dict__ if info else None,
        "version": version,
        "model": state.client.motor_models.get(motor_id),
    }


# --------------------------------------------------------------------------
# パラメータ
# --------------------------------------------------------------------------

@app.post("/api/motor/{motor_id}/params/read")
async def api_read_params(motor_id: int, req: ParamReadReq):
    _require_connection()
    values = await _call(state.client.read_all, motor_id, req.indices)
    return {"motor_id": motor_id, "values": values}


@app.post("/api/motor/{motor_id}/param")
async def api_write_param(motor_id: int, req: ParamWriteReq):
    _require_connection()
    if req.name:
        pdef = P.PARAMS_BY_NAME.get(req.name)
        if pdef is None:
            raise HTTPException(400, f"未知のパラメータです: {req.name}")
        index, ptype = pdef.index, pdef.type
    elif req.index is not None:
        index = req.index
        pdef = P.PARAMS_BY_INDEX.get(index)
        ptype = req.type or (pdef.type if pdef else P.TYPE_F32)
    else:
        raise HTTPException(400, "name か index のどちらかを指定してください")

    # 指令値の書き込みは動作を伴うため ARM を必須にする
    if pdef is not None and pdef.group == "command":
        _require_armed()

    fb = await _call(state.client.write_param, motor_id, index, req.value, ptype)
    readback = await _call(state.client.read_param, motor_id, index, ptype)
    return {"motor_id": motor_id, "index": index, "written": req.value,
            "readback": readback, "feedback": fb.as_dict() if fb else None}


@app.post("/api/motor/{motor_id}/params/scan")
async def api_index_scan(motor_id: int, req: IndexScanReq):
    """未文書のインデックスを読み出しのみで総当たり調査する。"""
    _require_connection()
    if req.end < req.start:
        raise HTTPException(400, "end は start 以上にしてください")
    if req.end - req.start > 512:
        raise HTTPException(400, "一度に調べられる範囲は 512 個までです")
    found = await _call(state.client.scan_indices, motor_id, req.start, req.end, req.timeout)
    return {"motor_id": motor_id, "found": found}


@app.post("/api/motor/{motor_id}/save")
async def api_save(motor_id: int):
    _require_connection()
    fb = await _call(state.client.save, motor_id)
    return {"ok": True, "feedback": fb.as_dict() if fb else None}


# --------------------------------------------------------------------------
# モータ操作
# --------------------------------------------------------------------------

@app.post("/api/motor/{motor_id}/enable")
async def api_enable(motor_id: int):
    _require_connection()
    _require_armed()
    fb = await _call(state.client.enable, motor_id)
    return {"feedback": fb.as_dict()}


@app.post("/api/motor/{motor_id}/stop")
async def api_stop(motor_id: int, req: StopReq):
    _require_connection()
    fb = await _call(state.client.stop, motor_id, req.clear_fault)
    return {"feedback": fb.as_dict()}


@app.post("/api/motor/{motor_id}/zero")
async def api_zero(motor_id: int):
    _require_connection()
    fb = await _call(state.client.set_zero, motor_id)
    return {"feedback": fb.as_dict()}


@app.post("/api/motor/{motor_id}/can-id")
async def api_can_id(motor_id: int, req: CanIdReq):
    _require_connection()
    info = await _call(state.client.set_can_id, motor_id, req.new_id)
    return {"device": info.__dict__}


@app.post("/api/motor/{motor_id}/control")
async def api_control(motor_id: int, req: ControlReq):
    _require_connection()
    _require_armed()
    fb = await _call(state.client.motion_control, motor_id, req.torque, req.position,
                     req.velocity, req.kp, req.kd)
    return {"feedback": fb.as_dict() if fb else None}


@app.post("/api/motor/{motor_id}/report")
async def api_report(motor_id: int, req: ReportReq):
    _require_connection()
    await _call(state.client.set_active_report, motor_id, req.enable)
    return {"ok": True, "enabled": req.enable}


@app.post("/api/motor/{motor_id}/can-baudrate")
async def api_can_baud(motor_id: int, req: BaudReq):
    _require_connection()
    await _call(state.client.set_can_baudrate, motor_id, req.baudrate)
    return {"ok": True, "note": "モータの再電源投入後に反映されます"}


@app.post("/api/motor/{motor_id}/protocol")
async def api_protocol(motor_id: int, req: ProtocolReq):
    _require_connection()
    await _call(state.client.set_protocol, motor_id, req.protocol)
    return {"ok": True, "note": "モータの再電源投入後に反映されます"}


# --------------------------------------------------------------------------
# 生フレーム / ログ
# --------------------------------------------------------------------------

@app.post("/api/raw")
def api_raw(req: RawReq):
    _require_connection()
    try:
        data = bytes.fromhex(req.data.replace(",", " ").replace("0x", "").strip())
    except ValueError as exc:
        raise HTTPException(400, f"データは 16 進数で指定してください: {exc}") from exc
    if len(data) > 8:
        raise HTTPException(400, "データは最大 8 バイトです")
    try:
        state.transport.send_frame(req.ext_id, data)
    except TransportError as exc:
        raise HTTPException(502, str(exc)) from exc
    return {"ok": True}


@app.get("/api/log")
def api_log(limit: int = 200):
    return {"log": state.log[-limit:]}


@app.delete("/api/log")
def api_clear_log():
    state.log.clear()
    return {"ok": True}


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    state.websockets.append(ws)
    try:
        await ws.send_json({"event": "hello", "status": api_status()})
        while True:
            await ws.receive_text()   # クライアントからの ping を消費
    except WebSocketDisconnect:
        pass
    finally:
        if ws in state.websockets:
            state.websockets.remove(ws)


# --------------------------------------------------------------------------
# 静的ファイル
# --------------------------------------------------------------------------

@app.get("/")
def index():
    return FileResponse(WEB_DIR / "index.html")


app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")
