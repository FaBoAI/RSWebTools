// Web Serial 版プロトコル層の検証。Python 側 tests/test_protocol.py と同じ根拠
// (公式マニュアル記載の実フレーム) を基準にしている。
//   node --test webserial/

import test from 'node:test';
import assert from 'node:assert/strict';
import {
  CanFrame, CommType, FrameParser, decodeFaultFrame, decodeFeedback,
  decodeValue, encodeFrame, encodeMotionControl, encodeValue, floatToUint,
  getProfile, hex, looksLikeFrame, packExtId, schemaFor, uintToFloat, unpackExtId,
} from './protocol.js';
import { PROFILES } from './tables.js';

// マニュアル記載の実例:
//   41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a
//   -> ext_id 0x1200FD01 (type=0x12 書込 / host=0x00FD / motor=0x01)
//      data   index=0x7005 (run_mode), value=1
const MANUAL_HEX = '41 54 90 07 E8 0C 08 05 70 00 00 01 00 00 00 0D 0A';
const MANUAL_FRAME = Uint8Array.from(MANUAL_HEX.split(' ').map((h) => parseInt(h, 16)));
const MANUAL_EXT_ID = 0x1200fd01;
const MANUAL_DATA = MANUAL_FRAME.slice(7, 15);

test('マニュアル記載の実例フレームをバイト単位で再現する', () => {
  assert.equal(hex(encodeFrame(MANUAL_EXT_ID, MANUAL_DATA)), MANUAL_HEX);
});

test('マニュアル記載の実例フレームを解析できる', () => {
  const events = new FrameParser().feed(MANUAL_FRAME);
  assert.equal(events.length, 1);
  assert.equal(events[0].kind, 'frame');
  assert.equal(events[0].frame.extId, MANUAL_EXT_ID);
  assert.deepEqual(events[0].frame.data, MANUAL_DATA);
});

test('実例フレームは run_mode=1 の書込として解釈できる', () => {
  const { commType, data2, targetId } = unpackExtId(MANUAL_EXT_ID);
  assert.equal(commType, CommType.WRITE_PARAM);
  assert.equal(data2, 0x00fd);      // ホスト CAN_ID
  assert.equal(targetId, 0x01);     // 宛先モータ
  const dv = new DataView(MANUAL_DATA.buffer, MANUAL_DATA.byteOffset);
  assert.equal(dv.getUint16(0, true), 0x7005);
  assert.equal(decodeValue('uint8', MANUAL_DATA.slice(4)), 1);
});

test('拡張 ID を往復できる', () => {
  for (const [ct, d2, tgt] of [[0, 0, 0], [0x1f, 0xffff, 0xff], [0x12, 0x00fd, 0x7f]]) {
    const r = unpackExtId(packExtId(ct, d2, tgt));
    assert.deepEqual([r.commType, r.data2, r.targetId], [ct, d2, tgt]);
  }
});

test('先頭のノイズを捨てて再同期する', () => {
  const noise = Uint8Array.from([0x9c, 0xff, 0x00, 0x80]);
  const events = new FrameParser().feed(Uint8Array.from([...noise, ...MANUAL_FRAME]));
  const kinds = events.map((e) => e.kind);
  assert.ok(kinds.includes('frame') && kinds.includes('junk'));
  assert.deepEqual(events.find((e) => e.kind === 'junk').data, noise);
});

test('分割して届いても組み立てられる', () => {
  const p = new FrameParser();
  assert.deepEqual(p.feed(MANUAL_FRAME.slice(0, 5)), []);
  assert.deepEqual(p.feed(MANUAL_FRAME.slice(5, 9)), []);
  assert.equal(p.feed(MANUAL_FRAME.slice(9)).filter((e) => e.kind === 'frame').length, 1);
});

test('連続したフレームを分離できる', () => {
  const three = Uint8Array.from([...MANUAL_FRAME, ...MANUAL_FRAME, ...MANUAL_FRAME]);
  assert.equal(new FrameParser().feed(three).filter((e) => e.kind === 'frame').length, 3);
});

test('末尾が壊れたフレームは捨てて次で再同期する', () => {
  const broken = Uint8Array.from(MANUAL_FRAME);
  broken[broken.length - 1] = 0x00;
  const frames = new FrameParser().feed(Uint8Array.from([...broken, ...MANUAL_FRAME]))
    .filter((e) => e.kind === 'frame');
  assert.equal(frames.length, 1);
  assert.equal(frames[0].frame.extId, MANUAL_EXT_ID);
});

test('ヘッダが化けたフレームを構造一致として判定できる', () => {
  // 実機 (macOS 標準 CH340 ドライバ) で観測されたバイト列。
  // 末尾 CRLF と DLC=8 は正しいのに先頭 2 バイトの上位ビットが化けている。
  const corrupted = Uint8Array.from(
    '81 94 00 03 FF F4 08 D1 B6 7B 0A 00 F4 30 15 0D 0A'.split(' ').map((h) => parseInt(h, 16)));
  assert.ok(looksLikeFrame(corrupted));
  assert.ok(looksLikeFrame(MANUAL_FRAME));
  assert.ok(!looksLikeFrame(Uint8Array.from([1, 2, 3])));
  // パーサはヘッダが合わないので junk として返す (誤ってデコードしない)
  const events = new FrameParser().feed(corrupted);
  assert.deepEqual(events.map((e) => e.kind), ['junk']);
});

test('スケーリングの端点が一致する', () => {
  for (const [lo, hi] of [[-12.57, 12.57], [-33, 33], [0, 500]]) {
    assert.equal(floatToUint(lo, lo, hi), 0);
    assert.equal(floatToUint(hi, lo, hi), 0xffff);
    assert.ok(Math.abs(uintToFloat(0, lo, hi) - lo) < 1e-9);
    assert.ok(Math.abs(uintToFloat(0xffff, lo, hi) - hi) < 1e-9);
  }
});

test('レンジ外の値は丸められる', () => {
  assert.equal(floatToUint(999, -12.57, 12.57), 0xffff);
  assert.equal(floatToUint(-999, -12.57, 12.57), 0);
});

test('運動制御はトルクを拡張 ID に、他を上位バイト先行で載せる', () => {
  const pr = getProfile('RS00');
  const { extId, data } = encodeMotionControl({
    torque: 0, position: 0, velocity: 0, kp: 0, kd: 0, motorId: 0x7f, pr,
  });
  const { commType, data2, targetId } = unpackExtId(extId);
  assert.equal(commType, CommType.MOTION_CONTROL);
  assert.equal(targetId, 0x7f);
  assert.equal(data2, 0x7fff);              // 0Nm は ±14Nm レンジの中央
  const dv = new DataView(data.buffer);
  assert.equal(dv.getUint16(0), 0x7fff);    // 位置中央
  assert.equal(dv.getUint16(2), 0x7fff);    // 速度中央
  assert.equal(dv.getUint16(4), 0);         // Kp 下限
  assert.equal(dv.getUint16(6), 0);         // Kd 下限
});

test('運動制御の位置はビッグエンディアン', () => {
  const pr = getProfile('RS00');
  const { data } = encodeMotionControl({
    torque: 0, position: pr.p_max, velocity: pr.v_min, kp: pr.kp_max, kd: 0, motorId: 1, pr,
  });
  assert.deepEqual(Array.from(data.slice(0, 2)), [0xff, 0xff]);
  assert.deepEqual(Array.from(data.slice(2, 4)), [0x00, 0x00]);
  assert.deepEqual(Array.from(data.slice(4, 6)), [0xff, 0xff]);
  assert.deepEqual(Array.from(data.slice(6, 8)), [0x00, 0x00]);
});

test('フィードバックを RS00 レンジで復号できる', () => {
  const pr = getProfile('RS00');
  let extId = packExtId(CommType.FEEDBACK, 0x0c, 0xfd);
  extId |= (1 << 18);       // bit18 = 過温度
  extId |= (2 << 22);       // bit23-22 = Run
  const data = new Uint8Array(8);
  const dv = new DataView(data.buffer);
  dv.setUint16(0, 0xffff); dv.setUint16(2, 0); dv.setUint16(4, 0x7fff); dv.setUint16(6, 253);
  const fb = decodeFeedback(new CanFrame(extId >>> 0, data), pr);
  assert.equal(fb.motorId, 0x0c);
  assert.ok(Math.abs(fb.angle - 12.57) < 1e-6);
  assert.ok(Math.abs(fb.velocity + 33) < 1e-6);
  assert.ok(Math.abs(fb.torque) < 1e-3);
  assert.ok(Math.abs(fb.temperature - 25.3) < 1e-6);
  assert.equal(fb.modeName, 'Run');
  assert.ok(fb.faults.includes('過温度'));
});

test('同じ生値でも機種が違えば物理量が変わる', () => {
  const data = new Uint8Array(8);
  const dv = new DataView(data.buffer);
  dv.setUint16(0, 0xffff); dv.setUint16(2, 0xffff); dv.setUint16(4, 0xffff); dv.setUint16(6, 250);
  const frame = new CanFrame(packExtId(CommType.FEEDBACK, 0x01, 0xfd), data);
  const a = decodeFeedback(frame, getProfile('RS00'));
  const b = decodeFeedback(frame, getProfile('RS04'));
  assert.ok(Math.abs(a.torque - 14) < 1e-6);
  assert.ok(Math.abs(b.torque - 120) < 1e-6);
  assert.ok(Math.abs(a.velocity - 33) < 1e-6);
  assert.ok(Math.abs(b.velocity - 15) < 1e-6);
});

test('故障フレームを復号できる', () => {
  const data = new Uint8Array(8);
  const dv = new DataView(data.buffer);
  dv.setUint32(0, (1 << 0) | (1 << 3), true);
  dv.setUint32(4, 1, true);
  const d = decodeFaultFrame(new CanFrame(packExtId(CommType.FAULT_FEEDBACK, 0x02, 0xfd), data));
  assert.equal(d.motorId, 0x02);
  assert.ok(d.faults.includes('モータ過温度 (既定 145℃)'));
  assert.ok(d.faults.includes('過電圧故障'));
  assert.equal(d.warnings.length, 1);
});

test('パラメータ値の符号化は常に 4 バイト', () => {
  for (const [t, v] of [['uint8', 3], ['uint16', 9], ['uint32', 7], ['float', 1.0]]) {
    assert.equal(encodeValue(t, v).length, 4);
    assert.equal(decodeValue(t, encodeValue(t, v)), v);
  }
});

test('run_mode=1 の書込データがマニュアル実例と一致する', () => {
  const data = new Uint8Array(8);
  new DataView(data.buffer).setUint16(0, 0x7005, true);
  data.set(encodeValue('uint8', 1), 4);
  assert.deepEqual(data, MANUAL_DATA);
});

test('スキーマが機種依存レンジを解決する', () => {
  const rs00 = Object.fromEntries(schemaFor(getProfile('RS00')).map((p) => [p.name, p]));
  assert.equal(rs00.limit_torque.max, 14);     // RS00 の T_MAX
  assert.equal(rs00.spd_ref.max, 33);          // RS00 の V_MAX
  assert.equal(rs00.iq_ref.max, 16);           // RS00 のパラメータ表
  assert.equal(rs00.mechPos.writable, false);

  const rs04 = Object.fromEntries(schemaFor(getProfile('RS04')).map((p) => [p.name, p]));
  assert.equal(rs04.limit_torque.max, 120);
  // 電流レンジは RS00 以外は原典未確認 -> null (UI 側で制限しない)
  assert.equal(rs04.iq_ref.max, null);
});

test('全機種のレンジが整合している', () => {
  for (const [key, pr] of Object.entries(PROFILES)) {
    assert.ok(Math.abs(pr.p_max - 12.57) < 1e-9, key);
    assert.equal(pr.p_min, -pr.p_max, key);
    assert.equal(pr.v_min, -pr.v_max, key);
    assert.equal(pr.t_min, -pr.t_max, key);
    assert.ok(pr.kp_max > 0 && pr.kd_max > 0, key);
  }
});

test('未知の機種は既定プロファイルにフォールバックする', () => {
  assert.equal(getProfile('RS99').key, getProfile(null).key);
});
