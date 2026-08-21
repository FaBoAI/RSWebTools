// RobStride 私有プロトコル (CAN 2.0B 拡張フレーム) と
// RobStride 純正 USB-CAN アダプタ (AT モード) のシリアルフレーム変換。
//
// backend/protocol.py の移植。実装根拠は docs/protocol.md を参照。
//   シリアルフレーム = 'AT' + BE32((extId << 3) | 0x04) + DLC + data[DLC] + CRLF

import { PARAMS, PLACEHOLDERS, PROFILES, DEFAULT_PROFILE, GROUP_LABELS } from './tables.js';

export const HEADER = Uint8Array.of(0x41, 0x54);   // 'AT'
export const TAIL = Uint8Array.of(0x0d, 0x0a);     // CRLF
export const EXT_FLAG = 0x04;                      // 拡張フレーム (IDE) ビット
export const DEFAULT_HOST_ID = 0xfd;

export const CommType = {
  GET_DEVICE_ID: 0x00,
  MOTION_CONTROL: 0x01,
  FEEDBACK: 0x02,
  ENABLE: 0x03,
  STOP: 0x04,
  SET_ZERO: 0x06,
  SET_CAN_ID: 0x07,
  READ_PARAM: 0x11,
  WRITE_PARAM: 0x12,
  FAULT_FEEDBACK: 0x15,
  SAVE: 0x16,
  SET_BAUD: 0x17,
  ACTIVE_REPORT: 0x18,
  SET_PROTOCOL: 0x19,
  READ_VERSION: 0x1a,
};

export const COMM_NAMES = {
  0x00: 'デバイスID', 0x01: '運動制御', 0x02: 'フィードバック', 0x03: '運転許可',
  0x04: '停止', 0x06: '原点設定', 0x07: 'CAN_ID変更', 0x11: 'パラメータ読出',
  0x12: 'パラメータ書込', 0x15: '故障通知', 0x16: '保存', 0x17: 'ボーレート変更',
  0x18: '能動送信', 0x19: 'プロトコル変更', 0x1a: 'バージョン',
};

// フィードバックフレーム bit21-16 の故障ビット (LSB = bit16)
export const FEEDBACK_FAULT_BITS = [
  [0, '低電圧'], [1, '三相電流異常'], [2, '過温度'],
  [3, '磁気エンコーダ異常'], [4, 'ロック/過負荷'], [5, '未キャリブレーション'],
];

// 通信タイプ 0x15 Byte0-3 の故障ビット
export const FAULT_FRAME_BITS = [
  [0, 'モータ過温度 (既定 145℃)'], [1, 'ドライバチップ故障'], [2, '低電圧故障'],
  [3, '過電圧故障'], [4, 'B 相電流サンプリング過電流'], [5, 'C 相電流サンプリング過電流'],
  [7, 'エンコーダ未キャリブレーション'], [8, 'ハードウェア識別故障'],
  [9, '位置初期化故障'], [14, 'ロック過負荷アルゴリズム保護'],
  [16, 'A 相電流サンプリング過電流'],
];

const MODE_NAMES = { 0: 'Reset', 1: 'Cali', 2: 'Run' };

// --------------------------------------------------------------------------
// 拡張 ID
// --------------------------------------------------------------------------

export function packExtId(commType, data2, targetId) {
  // 29bit を扱うので符号なしにするため >>> 0 を通す
  return ((((commType & 0x1f) << 24) | ((data2 & 0xffff) << 8) | (targetId & 0xff)) >>> 0);
}

export function unpackExtId(extId) {
  return {
    commType: (extId >>> 24) & 0x1f,
    data2: (extId >>> 8) & 0xffff,
    targetId: extId & 0xff,
  };
}

// --------------------------------------------------------------------------
// スケーリング (マニュアルの float_to_uint と同一式: 分母は 2^bits - 1)
// --------------------------------------------------------------------------

export function floatToUint(x, xMin, xMax, bits = 16) {
  const span = xMax - xMin;
  const v = Math.min(Math.max(Number(x), xMin), xMax);
  return Math.trunc(((v - xMin) * ((1 << bits) - 1)) / span);
}

export function uintToFloat(v, xMin, xMax, bits = 16) {
  const span = xMax - xMin;
  return xMin + (v * span) / ((1 << bits) - 1);
}

// --------------------------------------------------------------------------
// フレーム
// --------------------------------------------------------------------------

export function encodeFrame(extId, data, extended = true) {
  if (data.length > 8) throw new Error('CAN のデータ長は最大 8 バイトです');
  const mask = extended ? 0x1fffffff : 0x7ff;
  const wire = (((extId & mask) * 8) + (extended ? EXT_FLAG : 0)) >>> 0;
  const out = new Uint8Array(7 + data.length + 2);
  out[0] = 0x41; out[1] = 0x54;
  out[2] = (wire >>> 24) & 0xff;
  out[3] = (wire >>> 16) & 0xff;
  out[4] = (wire >>> 8) & 0xff;
  out[5] = wire & 0xff;
  out[6] = data.length;
  out.set(data, 7);
  out[7 + data.length] = 0x0d;
  out[8 + data.length] = 0x0a;
  return out;
}

export function buildFrame(commType, data2, targetId, data = new Uint8Array(8)) {
  return encodeFrame(packExtId(commType, data2, targetId), data);
}

export function hex(bytes) {
  return Array.from(bytes, (b) => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
}

/** フレームとして「構造だけ」合っているか (ヘッダ化けの診断用)。 */
export function looksLikeFrame(raw) {
  return raw.length === 17 && raw[15] === 0x0d && raw[16] === 0x0a && raw[6] === 8;
}

/** バイトストリームからフレームを取り出す耐ノイズパーサ。 */
export class FrameParser {
  static MAX_BUFFER = 4096;

  constructor() { this.buf = []; }

  /** -> [{kind:'frame'|'junk', ...}] */
  feed(chunk) {
    for (const b of chunk) this.buf.push(b);
    const events = [];
    const junk = [];

    for (;;) {
      let start = -1;
      for (let i = 0; i + 1 < this.buf.length; i += 1) {
        if (this.buf[i] === 0x41 && this.buf[i + 1] === 0x54) { start = i; break; }
      }
      if (start < 0) {
        // ヘッダ無し。'A' で終わる可能性があるので 1 バイトだけ残す
        const keep = this.buf.length && this.buf[this.buf.length - 1] === 0x41 ? 1 : 0;
        if (this.buf.length > keep) {
          junk.push(...this.buf.splice(0, this.buf.length - keep));
        }
        break;
      }
      if (start) junk.push(...this.buf.splice(0, start));
      if (this.buf.length < 7) break;

      const dlc = this.buf[6];
      if (dlc > 8) { junk.push(...this.buf.splice(0, 2)); continue; }
      const total = 7 + dlc + 2;
      if (this.buf.length < total) {
        if (this.buf.length > FrameParser.MAX_BUFFER) { junk.push(...this.buf.splice(0, 2)); continue; }
        break;
      }
      if (this.buf[total - 2] !== 0x0d || this.buf[total - 1] !== 0x0a) {
        junk.push(...this.buf.splice(0, 2));   // 偽ヘッダ。2 バイト進めて再同期
        continue;
      }

      const raw = this.buf.splice(0, total);
      const wire = ((raw[2] << 24) | (raw[3] << 16) | (raw[4] << 8) | raw[5]) >>> 0;
      const extended = Boolean(wire & EXT_FLAG);
      const idMask = extended ? 0x1fffffff : 0x7ff;
      events.push({
        kind: 'frame',
        frame: new CanFrame(Math.floor(wire / 8) & idMask, Uint8Array.from(raw.slice(7, 7 + dlc)), extended),
      });
    }

    if (junk.length) events.push({ kind: 'junk', data: Uint8Array.from(junk) });
    return events;
  }
}

export class CanFrame {
  constructor(extId, data, extended = true) {
    this.extId = extId >>> 0;
    this.data = data;
    this.extended = extended;
  }

  get commType() { return (this.extId >>> 24) & 0x1f; }
  get data2() { return (this.extId >>> 8) & 0xffff; }
  get targetId() { return this.extId & 0xff; }
  /** 応答フレームでは bit15-8 が送信元モータの CAN_ID */
  get motorId() { return (this.extId >>> 8) & 0xff; }
  get idHex() { return this.extId.toString(16).padStart(8, '0').toUpperCase(); }
  get dataHex() { return hex(this.data); }
  toString() { return `${this.idHex}${this.extended ? '' : 'S'}#${this.dataHex}`; }
}

// --------------------------------------------------------------------------
// デコード
// --------------------------------------------------------------------------

export function decodeFeedback(frame, pr) {
  if (frame.data.length < 8) throw new Error('フィードバックフレームは 8 バイト必要です');
  const dv = new DataView(frame.data.buffer, frame.data.byteOffset, 8);
  const faultBits = (frame.extId >>> 16) & 0x3f;
  const mode = (frame.extId >>> 22) & 0x03;
  return {
    motorId: frame.motorId,
    angle: uintToFloat(dv.getUint16(0), pr.p_min, pr.p_max),
    velocity: uintToFloat(dv.getUint16(2), pr.v_min, pr.v_max),
    torque: uintToFloat(dv.getUint16(4), pr.t_min, pr.t_max),
    temperature: dv.getUint16(6) / 10,
    mode,
    modeName: MODE_NAMES[mode] ?? `不明(${mode})`,
    faults: FEEDBACK_FAULT_BITS.filter(([b]) => faultBits & (1 << b)).map(([, n]) => n),
    faultBits,
  };
}

export function decodeFaultFrame(frame) {
  const dv = new DataView(frame.data.buffer, frame.data.byteOffset, frame.data.length);
  const fault = frame.data.length >= 4 ? dv.getUint32(0, true) : 0;
  const warn = frame.data.length >= 8 ? dv.getUint32(4, true) : 0;
  return {
    motorId: frame.motorId,
    faultValue: fault,
    warningValue: warn,
    faults: FAULT_FRAME_BITS.filter(([b]) => fault & (1 << b)).map(([, n]) => n),
    warnings: warn & 1 ? ['モータ過温度警告 (既定 135℃)'] : [],
  };
}

/**
 * 通信タイプ 1 の (extId, data) を作る。
 * トルクは 8 バイトのデータ領域ではなく拡張 ID のデータ領域2 に入る。
 * データ領域はすべて上位バイト先行 (ビッグエンディアン)。
 */
export function encodeMotionControl({ torque, position, velocity, kp, kd, motorId, pr }) {
  const extId = packExtId(CommType.MOTION_CONTROL,
    floatToUint(torque, pr.t_min, pr.t_max), motorId);
  const data = new Uint8Array(8);
  const dv = new DataView(data.buffer);
  dv.setUint16(0, floatToUint(position, pr.p_min, pr.p_max));
  dv.setUint16(2, floatToUint(velocity, pr.v_min, pr.v_max));
  dv.setUint16(4, floatToUint(kp, 0, pr.kp_max));
  dv.setUint16(6, floatToUint(kd, 0, pr.kd_max));
  return { extId, data };
}

// --------------------------------------------------------------------------
// パラメータ値 <-> 4 バイト枠 (Byte4-7, リトルエンディアン)
// --------------------------------------------------------------------------

export function encodeValue(type, value) {
  const out = new Uint8Array(4);
  const dv = new DataView(out.buffer);
  if (type === 'float') dv.setFloat32(0, Number(value), true);
  else if (type === 'uint8') out[0] = Number(value) & 0xff;
  else if (type === 'uint16') dv.setUint16(0, Number(value) & 0xffff, true);
  else if (type === 'uint32') dv.setUint32(0, Number(value) >>> 0, true);
  else throw new Error(`未知のパラメータ型: ${type}`);
  return out;
}

export function decodeValue(type, raw) {
  const buf = new Uint8Array(4);
  buf.set(raw.slice(0, 4));
  const dv = new DataView(buf.buffer);
  if (type === 'float') return dv.getFloat32(0, true);
  if (type === 'uint8') return buf[0];
  if (type === 'uint16') return dv.getUint16(0, true);
  if (type === 'uint32') return dv.getUint32(0, true);
  throw new Error(`未知のパラメータ型: ${type}`);
}

// --------------------------------------------------------------------------
// プロファイルとパラメータ表
// --------------------------------------------------------------------------

export function getProfile(key) {
  return PROFILES[String(key ?? '').toUpperCase()] ?? PROFILES[DEFAULT_PROFILE];
}

/** レンジのプレースホルダを機種プロファイルで解決する (params.py の resolve と同じ)。 */
export function resolveRange(value, pr, positive) {
  if (typeof value !== 'string') return value;
  const P_ = PLACEHOLDERS;
  if (value === P_.V_LIM) return positive ? pr.v_max : pr.v_min;
  if (value === P_.V_POS) return pr.v_max;
  if (value === P_.T_POS) return pr.t_max;
  if (value === P_.I_LIM || value === P_.I_POS) {
    if (pr.i_max === null || pr.i_max === undefined) return null;  // 不明: 制限しない
    if (value === P_.I_POS) return pr.i_max;
    return positive ? pr.i_max : -pr.i_max;
  }
  return null;
}

export function schemaFor(pr) {
  return PARAMS.map((p) => ({
    ...p,
    groupLabel: GROUP_LABELS[p.group],
    min: resolveRange(p.min, pr, false),
    max: resolveRange(p.max, pr, true),
  }));
}

export const PARAMS_BY_NAME = Object.fromEntries(PARAMS.map((p) => [p.name, p]));
export const PARAMS_BY_INDEX = Object.fromEntries(PARAMS.map((p) => [p.index, p]));
