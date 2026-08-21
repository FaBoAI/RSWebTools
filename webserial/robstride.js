// Web Serial API 経由で RobStride モータを操作するクライアント。
// backend/transport.py + backend/motor.py の移植。

import {
  CanFrame, CommType, DEFAULT_HOST_ID, FrameParser, PARAMS_BY_INDEX,
  decodeFeedback, decodeValue, encodeFrame, encodeMotionControl, encodeValue,
  getProfile, packExtId,
} from './protocol.js';
import { PARAMS } from './tables.js';

export const DEFAULT_BAUDRATE = 921600;
export const SUPPORTED_BAUDRATES = [115200, 230400, 460800, 921600, 1000000];

const ZERO8 = new Uint8Array(8);
/** 通信タイプ 22/23/24/25 の固定プリフィックス (マニュアル記載の 01 02 03 04 05 06) */
const MAGIC = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];

export const BAUD_CODES = { 1000000: 0x01, 500000: 0x02, 250000: 0x03, 125000: 0x04 };
export const PROTOCOL_CODES = { private: 0x00, canopen: 0x01, mit: 0x02 };

export class MotorError extends Error {}

export function isSupported() {
  return typeof navigator !== 'undefined' && 'serial' in navigator;
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// --------------------------------------------------------------------------
// トランスポート
// --------------------------------------------------------------------------

export class WebSerialTransport {
  constructor() {
    this.port = null;
    this.reader = null;
    this.writer = null;
    this.parser = new FrameParser();
    this.subscribers = [];
    this.waiters = [];
    this.txCount = 0;
    this.rxCount = 0;
    this.junkCount = 0;
    this.baudRate = DEFAULT_BAUDRATE;
    this.lastError = null;
    this._reading = false;
  }

  get connected() { return this.port !== null; }

  /** ポート選択ダイアログ。ユーザー操作 (クリック) の中から呼ぶ必要がある。 */
  static async requestPort() {
    if (!isSupported()) {
      throw new MotorError('このブラウザは Web Serial に対応していません。Chrome または Edge をお使いください。');
    }
    // CH340 / CH9102 / CP210x / FT232 を候補に出す
    return navigator.serial.requestPort({
      filters: [
        { usbVendorId: 0x1a86 },   // WCH (CH340/CH341/CH9102)
        { usbVendorId: 0x10c4 },   // Silicon Labs (CP210x)
        { usbVendorId: 0x0403 },   // FTDI
      ],
    });
  }

  async open(port, baudRate = DEFAULT_BAUDRATE) {
    await this.close();
    try {
      await port.open({ baudRate, dataBits: 8, stopBits: 1, parity: 'none', bufferSize: 4096 });
    } catch (e) {
      throw new MotorError(`ポートを開けません: ${e.message}`);
    }
    this.port = port;
    this.baudRate = baudRate;
    this.txCount = this.rxCount = this.junkCount = 0;
    this.lastError = null;
    this.parser = new FrameParser();
    this.writer = port.writable.getWriter();
    this._readLoop();
  }

  async close() {
    this._reading = false;
    try { if (this.reader) { await this.reader.cancel(); this.reader.releaseLock(); } } catch { /* 無視 */ }
    try { if (this.writer) this.writer.releaseLock(); } catch { /* 無視 */ }
    try { if (this.port) await this.port.close(); } catch { /* 無視 */ }
    this.reader = this.writer = this.port = null;
    for (const w of this.waiters) w.resolve(null);
    this.waiters = [];
  }

  subscribe(cb) { this.subscribers.push(cb); }
  unsubscribe(cb) {
    const i = this.subscribers.indexOf(cb);
    if (i >= 0) this.subscribers.splice(i, 1);
  }

  _publish(kind, payload) {
    for (const cb of [...this.subscribers]) {
      try { cb(kind, payload); } catch (e) { console.error('購読者で例外', e); }
    }
  }

  async _readLoop() {
    this._reading = true;
    while (this._reading && this.port?.readable) {
      this.reader = this.port.readable.getReader();
      try {
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          if (value) this._onChunk(value);
        }
      } catch (e) {
        if (this._reading) {
          this.lastError = String(e);
          this._publish('error', `受信エラー: ${e.message ?? e}`);
        }
      } finally {
        try { this.reader.releaseLock(); } catch { /* 無視 */ }
      }
      if (!this._reading) break;
    }
  }

  _onChunk(chunk) {
    for (const ev of this.parser.feed(chunk)) {
      if (ev.kind === 'frame') {
        this.rxCount += 1;
        this._dispatch(ev.frame);
      } else {
        this.junkCount += ev.data.length;
        this._publish('junk', ev.data);
      }
    }
  }

  _dispatch(frame) {
    for (const w of this.waiters) {
      if (!w.done && w.predicate(frame)) { w.done = true; w.resolve(frame); break; }
    }
    this.waiters = this.waiters.filter((w) => !w.done);
    this._publish('rx', frame);
  }

  async sendFrame(extId, data, extended = true) {
    if (!this.connected) throw new MotorError('シリアルポートが開かれていません');
    await this.writer.write(encodeFrame(extId, data, extended));
    this.txCount += 1;
    this._publish('tx', new CanFrame(extId, Uint8Array.from(data), extended));
  }

  async sendRaw(bytes) {
    if (!this.connected) throw new MotorError('シリアルポートが開かれていません');
    await this.writer.write(bytes);
    this._publish('tx_raw', bytes);
  }

  /** 送信して条件に合う受信フレームを 1 つ待つ。タイムアウトで null。 */
  async request(extId, data, predicate, timeout = 500) {
    let resolve;
    const p = new Promise((r) => { resolve = r; });
    const waiter = { predicate, resolve, done: false };
    this.waiters.push(waiter);
    await this.sendFrame(extId, data);
    const timer = setTimeout(() => {
      if (!waiter.done) { waiter.done = true; resolve(null); }
    }, timeout);
    const frame = await p;
    clearTimeout(timer);
    this.waiters = this.waiters.filter((w) => w !== waiter);
    return frame;
  }
}

// --------------------------------------------------------------------------
// モータクライアント
// --------------------------------------------------------------------------

export class MotorClient {
  constructor(transport, { hostId = DEFAULT_HOST_ID, timeout = 500, retries = 1 } = {}) {
    this.transport = transport;
    this.hostId = hostId;
    this.timeout = timeout;
    this.retries = retries;
    this.motorModels = {};        // motorId -> 機種キー
  }

  profile(motorId) { return getProfile(this.motorModels[motorId]); }
  setModel(motorId, model) { this.motorModels[motorId] = model; }

  async _request(commType, data2, motorId, data, predicate, what,
                 { timeout = null, required = true } = {}) {
    const extId = packExtId(commType, data2, motorId);
    for (let i = 0; i <= this.retries; i += 1) {
      const f = await this.transport.request(extId, data, predicate, timeout ?? this.timeout);
      if (f) return f;
    }
    if (required) throw new MotorError(`モータから応答がありません (${what})`);
    return null;
  }

  /**
   * フィードバックフレームの照合条件。
   * 運動制御 (通信タイプ 1) はデータ領域2 がトルク値でホスト ID を運べないため、
   * モータは自身に保存された CAN_MASTER (工場出荷値 0) を宛先にして返す。
   * そのため運動制御では宛先の一致を要求しない。
   */
  _feedbackPredicate(motorId, strictHost = true) {
    return (f) => f.commType === CommType.FEEDBACK && f.motorId === motorId
      && (!strictHost || f.targetId === this.hostId);
  }

  _decodeFb(frame) {
    return decodeFeedback(frame, this.profile(frame.motorId));
  }

  async enable(motorId) {
    const f = await this._request(CommType.ENABLE, this.hostId, motorId, ZERO8,
      this._feedbackPredicate(motorId), '運転許可');
    return this._decodeFb(f);
  }

  async stop(motorId, clearFault = false) {
    const data = new Uint8Array(8);
    data[0] = clearFault ? 1 : 0;
    const f = await this._request(CommType.STOP, this.hostId, motorId, data,
      this._feedbackPredicate(motorId), '停止');
    return this._decodeFb(f);
  }

  /** 非常停止用。応答を待たずに停止フレームだけ送る。 */
  async stopNoWait(motorId) {
    await this.transport.sendFrame(packExtId(CommType.STOP, this.hostId, motorId), ZERO8);
  }

  async setZero(motorId) {
    const data = new Uint8Array(8);
    data[0] = 1;
    const f = await this._request(CommType.SET_ZERO, this.hostId, motorId, data,
      this._feedbackPredicate(motorId), '原点設定');
    return this._decodeFb(f);
  }

  async setCanId(motorId, newId) {
    if (!(newId >= 0 && newId <= 0x7f)) {
      throw new MotorError('CAN_ID は 0〜127 の範囲で指定してください');
    }
    // 新 ID は拡張 ID の bit23-16 = データ領域2 の上位バイト
    const data2 = ((newId & 0xff) << 8) | (this.hostId & 0xff);
    const f = await this._request(CommType.SET_CAN_ID, data2, motorId, ZERO8,
      (fr) => fr.commType === CommType.GET_DEVICE_ID, 'CAN_ID 変更', { timeout: 1000 });
    if (this.motorModels[motorId]) {
      this.motorModels[newId] = this.motorModels[motorId];
      delete this.motorModels[motorId];
    }
    return { motorId: newId, uid: hexOf(f.data), rawId: f.idHex };
  }

  async getDeviceId(motorId, timeout = 250) {
    const f = await this._request(CommType.GET_DEVICE_ID, this.hostId, motorId, ZERO8,
      (fr) => fr.commType === CommType.GET_DEVICE_ID, 'デバイス ID 取得',
      { timeout, required: false });
    return f ? { motorId: f.motorId || motorId, uid: hexOf(f.data), rawId: f.idHex } : null;
  }

  async readParamRaw(motorId, index) {
    const data = new Uint8Array(8);
    new DataView(data.buffer).setUint16(0, index, true);
    const pred = (f) => f.commType === CommType.READ_PARAM && f.motorId === motorId
      && f.data.length >= 8
      && new DataView(f.data.buffer, f.data.byteOffset).getUint16(0, true) === index;
    const f = await this._request(CommType.READ_PARAM, this.hostId, motorId, data, pred,
      `パラメータ読出 0x${index.toString(16).toUpperCase()}`);
    return f.data.slice(4, 8);
  }

  async readParam(motorId, index, type = null) {
    const def = PARAMS_BY_INDEX[index];
    return decodeValue(type ?? def?.type ?? 'float', await this.readParamRaw(motorId, index));
  }

  async writeParam(motorId, index, value, type = null) {
    const def = PARAMS_BY_INDEX[index];
    if (def && !def.writable) {
      throw new MotorError(`${def.name} (${def.indexHex}) は読み出し専用です`);
    }
    const t = type ?? def?.type ?? 'float';
    const data = new Uint8Array(8);
    new DataView(data.buffer).setUint16(0, index, true);
    data.set(encodeValue(t, value), 4);
    const f = await this._request(CommType.WRITE_PARAM, this.hostId, motorId, data,
      this._feedbackPredicate(motorId), `パラメータ書込 ${index}`, { required: false });
    return f ? this._decodeFb(f) : null;
  }

  /** パラメータ表を一括読み出し。個別の失敗は握って error として返す。 */
  async readAll(motorId, indices = null, onProgress = null) {
    const targets = indices ?? PARAMS.map((p) => p.index);
    const out = {};
    for (let i = 0; i < targets.length; i += 1) {
      const index = targets[i];
      const def = PARAMS_BY_INDEX[index];
      const name = def?.name ?? `0x${index.toString(16).toUpperCase()}`;
      try {
        out[name] = { index, value: await this.readParam(motorId, index) };
      } catch (e) {
        out[name] = { index, value: null, error: e.message };
      }
      onProgress?.(i + 1, targets.length);
    }
    return out;
  }

  async save(motorId) {
    const data = Uint8Array.from([...MAGIC, 0x07, 0x08]);
    const f = await this._request(CommType.SAVE, this.hostId, motorId, data,
      this._feedbackPredicate(motorId), '保存', { timeout: 1500, required: false });
    return f ? this._decodeFb(f) : null;
  }

  async setActiveReport(motorId, enable) {
    const data = Uint8Array.from([...MAGIC, enable ? 1 : 0, 0]);
    await this.transport.sendFrame(packExtId(CommType.ACTIVE_REPORT, this.hostId, motorId), data);
  }

  async setCanBaudrate(motorId, baudrate) {
    const code = BAUD_CODES[baudrate];
    if (!code) throw new MotorError(`対応していないボーレートです: ${baudrate}`);
    await this.transport.sendFrame(packExtId(CommType.SET_BAUD, this.hostId, motorId),
      Uint8Array.from([...MAGIC, code, 0]));
  }

  async setProtocol(motorId, protocol) {
    const code = PROTOCOL_CODES[String(protocol).toLowerCase()];
    if (code === undefined) throw new MotorError(`未知のプロトコルです: ${protocol}`);
    await this.transport.sendFrame(packExtId(CommType.SET_PROTOCOL, this.hostId, motorId),
      Uint8Array.from([...MAGIC, code, 0]));
  }

  async readVersion(motorId) {
    const data = new Uint8Array(8);
    data[0] = 0x00; data[1] = 0xc4;
    const f = await this._request(CommType.READ_VERSION, this.hostId, motorId, data,
      (fr) => fr.commType === CommType.READ_VERSION && fr.motorId === motorId,
      'バージョン読出', { required: false });
    if (!f || f.data.length < 7) return null;
    return { raw: hexOf(f.data), version: Array.from(f.data.slice(3, 7)).join('.') };
  }

  async motionControl(motorId, { torque = 0, position = 0, velocity = 0, kp = 0, kd = 0 },
                      wait = true) {
    const pr = this.profile(motorId);
    const { extId, data } = encodeMotionControl({ torque, position, velocity, kp, kd, motorId, pr });
    if (!wait) { await this.transport.sendFrame(extId, data); return null; }
    const f = await this.transport.request(extId, data,
      this._feedbackPredicate(motorId, false), this.timeout);
    return f ? this._decodeFb(f) : null;
  }

  /**
   * CAN_ID を総当たりしてモータを探す。
   * 1 台ずつ応答を待つと 0〜127 で 50 秒近くかかるため、問い合わせを一斉に
   * 送ってから応答をまとめて拾う。応答フレームには送信元の CAN_ID が入る。
   */
  async scan(start = 0, end = 127, { gap = 8, settle = 600, onProgress = null } = {}) {
    const found = new Map();
    const collector = (kind, payload) => {
      if (kind !== 'rx') return;
      const f = payload;
      if (f.commType === CommType.GET_DEVICE_ID && !found.has(f.motorId)) {
        found.set(f.motorId, { motorId: f.motorId, uid: hexOf(f.data), detectedBy: 'device_id' });
      } else if (f.commType === CommType.READ_PARAM && !found.has(f.motorId)) {
        found.set(f.motorId, { motorId: f.motorId, uid: null, detectedBy: 'read_param' });
      }
    };
    this.transport.subscribe(collector);
    try {
      for (let id = start; id <= end; id += 1) {
        await this.transport.sendFrame(packExtId(CommType.GET_DEVICE_ID, this.hostId, id), ZERO8);
        onProgress?.(id - start + 1, (end - start + 1) * 2);
        await sleep(gap);
      }
      await sleep(settle);

      const missing = [];
      for (let id = start; id <= end; id += 1) if (!found.has(id)) missing.push(id);
      if (missing.length) {
        const data = new Uint8Array(8);
        new DataView(data.buffer).setUint16(0, 0x7005, true);
        for (let i = 0; i < missing.length; i += 1) {
          await this.transport.sendFrame(
            packExtId(CommType.READ_PARAM, this.hostId, missing[i]), data);
          onProgress?.((end - start + 1) + i + 1, (end - start + 1) * 2);
          await sleep(gap);
        }
        await sleep(settle);
      }
    } finally {
      this.transport.unsubscribe(collector);
    }
    return [...found.values()].sort((a, b) => a.motorId - b.motorId);
  }

  /** 未文書のインデックスを読み出しのみで総当たり調査する。 */
  async scanIndices(motorId, start, end, timeout = 120) {
    const out = [];
    for (let index = start; index <= end; index += 1) {
      const data = new Uint8Array(8);
      new DataView(data.buffer).setUint16(0, index, true);
      const pred = (f) => f.commType === CommType.READ_PARAM && f.motorId === motorId
        && f.data.length >= 8
        && new DataView(f.data.buffer, f.data.byteOffset).getUint16(0, true) === index;
      const f = await this._request(CommType.READ_PARAM, this.hostId, motorId, data, pred,
        'index スキャン', { timeout, required: false });
      if (!f) continue;
      const raw = f.data.slice(4, 8);
      out.push({
        index,
        indexHex: `0x${index.toString(16).padStart(4, '0').toUpperCase()}`,
        raw: hexOf(raw),
        asFloat: decodeValue('float', raw),
        asUint32: decodeValue('uint32', raw),
        known: PARAMS_BY_INDEX[index]?.name ?? null,
      });
    }
    return out;
  }
}

function hexOf(bytes) {
  return Array.from(bytes, (b) => b.toString(16).padStart(2, '0').toUpperCase()).join('');
}
