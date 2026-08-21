// MotorClient のフレーム組立を、シリアルを模したトランスポートで検証する。
// Python 側 tests/test_motor.py と対になる。
//   node --test webserial/

import test from 'node:test';
import assert from 'node:assert/strict';
import { CanFrame, CommType, packExtId, unpackExtId } from './protocol.js';
import { MotorClient } from './robstride.js';

/** 送信フレームを記録し、任意の応答を注入できるスタブ。 */
class FakeTransport {
  constructor() {
    this.sent = [];
    this.responder = null;      // fn(CanFrame) -> CanFrame | null
    this.subscribers = [];
    this.txCount = 0;
  }

  subscribe(cb) { this.subscribers.push(cb); }
  unsubscribe(cb) {
    const i = this.subscribers.indexOf(cb);
    if (i >= 0) this.subscribers.splice(i, 1);
  }

  async sendFrame(extId, data, extended = true) {
    const req = new CanFrame(extId, Uint8Array.from(data), extended);
    this.sent.push(req);
    this.txCount += 1;
    if (this.responder) {
      const reply = this.responder(req);
      if (reply) for (const cb of [...this.subscribers]) cb('rx', reply);
    }
  }

  async request(extId, data, predicate) {
    const req = new CanFrame(extId, Uint8Array.from(data));
    this.sent.push(req);
    this.txCount += 1;
    if (!this.responder) return null;
    const reply = this.responder(req);
    return reply && predicate(reply) ? reply : null;
  }

  get last() { return this.sent[this.sent.length - 1]; }
}

function feedbackFor(motorId, hostId = 0xfd, mode = 2) {
  const ext = (packExtId(CommType.FEEDBACK, motorId, hostId) | (mode << 22)) >>> 0;
  const data = new Uint8Array(8);
  const dv = new DataView(data.buffer);
  dv.setUint16(0, 0x7fff); dv.setUint16(2, 0x7fff); dv.setUint16(4, 0x7fff); dv.setUint16(6, 250);
  return new CanFrame(ext, data);
}

function makeClient() {
  const t = new FakeTransport();
  const c = new MotorClient(t, { hostId: 0xfd, retries: 0, timeout: 50 });
  c.setModel(1, 'RS00');
  return { t, c };
}

test('運転許可フレームを正しく組み立てる', async () => {
  const { t, c } = makeClient();
  t.responder = () => feedbackFor(1);
  const fb = await c.enable(1);
  const { commType, data2, targetId } = unpackExtId(t.last.extId);
  assert.equal(commType, CommType.ENABLE);
  assert.equal(data2, 0xfd);
  assert.equal(targetId, 1);
  assert.equal(fb.modeName, 'Run');
});

test('停止の故障クリアは Byte0=1', async () => {
  const { t, c } = makeClient();
  t.responder = () => feedbackFor(1);
  await c.stop(1, true);
  assert.equal(t.last.data[0], 1);
  await c.stop(1, false);
  assert.equal(t.last.data[0], 0);
});

test('原点設定は Byte0=1', async () => {
  const { t, c } = makeClient();
  t.responder = () => feedbackFor(1);
  await c.setZero(1);
  assert.equal(unpackExtId(t.last.extId).commType, CommType.SET_ZERO);
  assert.equal(t.last.data[0], 1);
});

test('CAN_ID 変更は新 ID を bit23-16 に載せる', async () => {
  const { t, c } = makeClient();
  t.responder = () => new CanFrame(packExtId(CommType.GET_DEVICE_ID, 0xfe05, 0xfd),
    new Uint8Array(8).fill(0x11));
  const info = await c.setCanId(1, 5);
  const { commType, data2, targetId } = unpackExtId(t.last.extId);
  assert.equal(commType, CommType.SET_CAN_ID);
  assert.equal(targetId, 1);
  assert.equal(data2 >> 8, 5);        // 新 ID
  assert.equal(data2 & 0xff, 0xfd);   // ホスト ID
  assert.equal(info.motorId, 5);
  assert.equal(c.motorModels[5], 'RS00');
  assert.equal(c.motorModels[1], undefined);
});

test('パラメータ読出の組立と復号', async () => {
  const { t, c } = makeClient();
  t.responder = (req) => {
    const index = new DataView(req.data.buffer, req.data.byteOffset).getUint16(0, true);
    const payload = new Uint8Array(8);
    const dv = new DataView(payload.buffer);
    dv.setUint16(0, index, true);
    dv.setFloat32(4, 40.0, true);
    return new CanFrame(packExtId(CommType.READ_PARAM, 1, 0xfd), payload);
  };
  assert.ok(Math.abs(await c.readParam(1, 0x701e) - 40) < 1e-6);
  const dv = new DataView(t.last.data.buffer, t.last.data.byteOffset);
  assert.equal(dv.getUint16(0, true), 0x701e);
  assert.deepEqual(Array.from(t.last.data.slice(2)), [0, 0, 0, 0, 0, 0]);
});

test('別インデックスの応答は採用しない', async () => {
  const { t, c } = makeClient();
  t.responder = () => {
    const payload = new Uint8Array(8);
    new DataView(payload.buffer).setUint16(0, 0x1234, true);
    return new CanFrame(packExtId(CommType.READ_PARAM, 1, 0xfd), payload);
  };
  await assert.rejects(() => c.readParam(1, 0x701e));
});

test('uint8 書込のバイト配置', async () => {
  const { t, c } = makeClient();
  t.responder = () => feedbackFor(1);
  await c.writeParam(1, 0x7005, 2);
  const d = t.last.data;
  assert.equal(new DataView(d.buffer, d.byteOffset).getUint16(0, true), 0x7005);
  assert.deepEqual(Array.from(d.slice(2, 4)), [0, 0]);
  assert.equal(d[4], 2);
  assert.deepEqual(Array.from(d.slice(5)), [0, 0, 0]);
});

test('float 書込のバイト配置', async () => {
  const { t, c } = makeClient();
  t.responder = () => feedbackFor(1);
  await c.writeParam(1, 0x701e, 30.0);
  const d = t.last.data;
  assert.ok(Math.abs(new DataView(d.buffer, d.byteOffset).getFloat32(4, true) - 30) < 1e-6);
});

test('読み出し専用への書込は拒否される', async () => {
  const { c } = makeClient();
  await assert.rejects(() => c.writeParam(1, 0x7019, 1.0));   // mechPos は R のみ
});

test('保存フレームのデータ部は 01..08', async () => {
  const { t, c } = makeClient();
  await c.save(1);
  assert.deepEqual(Array.from(t.last.data), [1, 2, 3, 4, 5, 6, 7, 8]);
  assert.equal(unpackExtId(t.last.extId).commType, CommType.SAVE);
});

test('能動送信の F_CMD', async () => {
  const { t, c } = makeClient();
  await c.setActiveReport(1, true);
  assert.deepEqual(Array.from(t.last.data), [1, 2, 3, 4, 5, 6, 0x01, 0]);
  await c.setActiveReport(1, false);
  assert.deepEqual(Array.from(t.last.data), [1, 2, 3, 4, 5, 6, 0x00, 0]);
});

test('CAN ボーレートのコード', async () => {
  const { t, c } = makeClient();
  for (const [baud, code] of [[1000000, 1], [500000, 2], [250000, 3], [125000, 4]]) {
    await c.setCanBaudrate(1, baud);
    assert.equal(t.last.data[6], code);
  }
  await assert.rejects(() => c.setCanBaudrate(1, 800000));
});

test('運動制御の応答は CAN_MASTER 宛でも受け取る', async () => {
  // 通信タイプ 1 はデータ領域2 がトルク値でホスト ID を運べないため、
  // モータは自身の CAN_MASTER (工場出荷値 0) を宛先にして返す。
  const { t, c } = makeClient();
  t.responder = () => feedbackFor(1, 0x00);   // 宛先は 0xFD ではなく 0x00
  const fb = await c.motionControl(1, { torque: 0, position: 0, velocity: 0, kp: 0, kd: 0 });
  assert.ok(fb, 'CAN_MASTER 宛のフィードバックを受け取れていない');
  assert.equal(fb.motorId, 1);
});

test('通常コマンドは宛先の一致を要求する', async () => {
  const { t, c } = makeClient();
  t.responder = () => feedbackFor(1, 0x11);   // 別ホスト宛
  await assert.rejects(() => c.enable(1));
});

test('運動制御は選択中の機種でスケーリングする', async () => {
  const { t, c } = makeClient();
  c.setModel(1, 'RS04');
  t.responder = () => feedbackFor(1, 0x00);
  await c.motionControl(1, { torque: 120, position: 0, velocity: 0, kp: 0, kd: 0 });
  assert.equal(unpackExtId(t.last.extId).data2, 0xffff);   // RS04 の T_MAX = 120Nm
});

test('スキャンは一斉送信して応答をまとめて拾う', async () => {
  const { t, c } = makeClient();
  t.responder = (req) => {
    const { commType, targetId } = unpackExtId(req.extId);
    if (commType === CommType.GET_DEVICE_ID && (targetId === 6 || targetId === 127)) {
      return new CanFrame(packExtId(CommType.GET_DEVICE_ID, 0xfe00 | targetId, 0xfd),
        new Uint8Array(8).fill(targetId));
    }
    return null;
  };
  const found = await c.scan(0, 127, { gap: 0, settle: 0 });
  assert.deepEqual(found.map((m) => m.motorId), [6, 127]);
  assert.equal(found[0].detectedBy, 'device_id');
});

test('通信タイプ 0 に応答しない個体も read_param で検出できる', async () => {
  const { t, c } = makeClient();
  t.responder = (req) => {
    const { commType, targetId } = unpackExtId(req.extId);
    if (commType === CommType.READ_PARAM && targetId === 3) {
      const payload = new Uint8Array(8);
      new DataView(payload.buffer).setUint16(0, 0x7005, true);
      return new CanFrame(packExtId(CommType.READ_PARAM, 3, 0xfd), payload);
    }
    return null;
  };
  const found = await c.scan(0, 5, { gap: 0, settle: 0 });
  assert.deepEqual(found.map((m) => m.motorId), [3]);
  assert.equal(found[0].detectedBy, 'read_param');
});

test('スキャン後に購読者が残らない', async () => {
  const { t, c } = makeClient();
  t.responder = () => null;
  const before = t.subscribers.length;
  await c.scan(0, 3, { gap: 0, settle: 0 });
  assert.equal(t.subscribers.length, before);
});

test('一括読み出しは個別の失敗を握って完走する', async () => {
  const { t, c } = makeClient();
  t.responder = (req) => {
    const index = new DataView(req.data.buffer, req.data.byteOffset).getUint16(0, true);
    if (index === 0x7005) return null;
    const payload = new Uint8Array(8);
    const dv = new DataView(payload.buffer);
    dv.setUint16(0, index, true);
    dv.setFloat32(4, 1.0, true);
    return new CanFrame(packExtId(CommType.READ_PARAM, 1, 0xfd), payload);
  };
  const res = await c.readAll(1);
  assert.ok(res.run_mode.error);
  assert.ok(Math.abs(res.loc_kp.value - 1.0) < 1e-6);
});
