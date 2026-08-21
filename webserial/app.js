// Web Serial 版 UI。ローカル版 (web/app.js) と同じ操作性で、
// バックエンドを介さずブラウザから直接 USB-CAN を叩く。

import {
  COMM_NAMES, decodeFaultFrame, decodeFeedback, getProfile, hex, looksLikeFrame,
  schemaFor, PARAMS_BY_NAME,
} from './protocol.js';
import {
  BAUD_CODES, DEFAULT_BAUDRATE, MotorClient, PROTOCOL_CODES, SUPPORTED_BAUDRATES,
  WebSerialTransport, isSupported,
} from './robstride.js';
import { GROUP_LABELS, PROFILES, RUN_MODE_CHOICES } from './tables.js';

const $ = (id) => document.getElementById(id);
const el = (tag, cls, text) => {
  const n = document.createElement(tag);
  if (cls) n.className = cls;
  if (text !== undefined) n.textContent = text;
  return n;
};
const isMacOS = () => /Mac/i.test(navigator.userAgentData?.platform ?? navigator.platform ?? '');
const fmt = (v, d = 3) => (v === null || v === undefined || Number.isNaN(v) ? '–' : Number(v).toFixed(d));
const parseHex = (s) => {
  const v = parseInt(String(s).trim().replace(/^0x/i, ''), 16);
  if (Number.isNaN(v)) throw new Error(`16進数として解釈できません: ${s}`);
  return v;
};

function toast(msg, kind = '') {
  const n = el('div', `toast ${kind}`, msg);
  $('toasts').appendChild(n);
  setTimeout(() => {
    n.style.transition = 'opacity .3s';
    n.style.opacity = '0';
    setTimeout(() => n.remove(), 300);
  }, kind === 'err' ? 7000 : 3500);
}

// =========================================================================
// 状態
// =========================================================================
const transport = new WebSerialTransport();
const client = new MotorClient(transport);

const S = {
  connected: false,
  armed: false,
  motorId: null,
  model: 'RS00',
  schema: null,
  values: {},
  dirty: {},
  motors: [],
  motorMode: null,
  streamTimer: null,
  busy: false,
};

// =========================================================================
// 有効/無効の制御
// =========================================================================
function updateEnabled() {
  const c = S.connected;
  const m = c && S.motorId !== null;
  const arm = m && S.armed;
  const set = (id, v) => { const n = $(id); if (n) n.disabled = !v; };

  set('connectBtn', !c); set('disconnectBtn', c);
  set('scanBtn', c); set('armToggle', c); set('estopBtn', c);
  set('atModeBtn', c); set('rawSendBtn', c);
  ['readAllBtn', 'infoBtn', 'zeroBtn', 'canIdBtn', 'idxScanBtn', 'reportToggle',
   'stopBtn', 'clearFaultBtn', 'applyModeBtn', 'canBaudBtn', 'protocolBtn',
   'saveBtn', 'exportBtn', 'importBtn', 'writeDirtyBtn'].forEach((id) => set(id, m));
  ['enableBtn', 'motionSendBtn', 'motionStreamBtn'].forEach((id) => set(id, arm));
  document.querySelectorAll('.cmd-input').forEach((n) => { n.disabled = !arm; });
}

function setStatus() {
  $('connPill').classList.toggle('on', S.connected);
  $('connText').textContent = S.connected
    ? `${transport.baudRate} bps  TX ${transport.txCount} / RX ${transport.rxCount}`
    : '未接続';
}

function progress(id, done, total) {
  const box = $(id);
  if (!box) return;
  box.hidden = done >= total;
  box.firstElementChild.style.width = `${Math.round((done / total) * 100)}%`;
}

// =========================================================================
// 受信の配線
// =========================================================================
const MAX_LOG_ROWS = 500;

transport.subscribe((kind, payload) => {
  const now = new Date();
  if (kind === 'tx' || kind === 'rx') {
    appendLog(now, kind, payload.idHex, COMM_NAMES[payload.commType] ?? `0x${payload.commType.toString(16)}`,
      payload.dataHex);
    if (kind === 'rx') handleRx(payload);
  } else if (kind === 'junk') {
    const label = looksLikeFrame(payload) ? 'ヘッダ化け (要ボーレート確認)' : `未解析 ${payload.length}B`;
    appendLog(now, 'junk', '-', label, hex(payload));
  } else if (kind === 'tx_raw') {
    appendLog(now, 'tx', '-', 'RAW', hex(payload));
  } else if (kind === 'error') {
    appendLog(now, 'error', '-', 'ERROR', String(payload));
    toast(String(payload), 'err');
  }
  setStatus();
});

function handleRx(frame) {
  try {
    if ((frame.commType === 0x02 || frame.commType === 0x18) && frame.data.length >= 8) {
      renderTelemetry(decodeFeedback(frame, client.profile(frame.motorId)));
    } else if (frame.commType === 0x15) {
      const d = decodeFaultFrame(frame);
      if (d.faults.length) toast(`故障通知 (ID ${d.motorId}): ${d.faults.join(' / ')}`, 'err');
    }
  } catch (e) {
    console.error('受信フレームの解釈に失敗', e);
  }
}

function appendLog(t, dir, id, typeName, data) {
  const body = $('logBody');
  const ts = `${String(t.getHours()).padStart(2, '0')}:${String(t.getMinutes()).padStart(2, '0')}:`
    + `${String(t.getSeconds()).padStart(2, '0')}.${String(t.getMilliseconds()).padStart(3, '0')}`;
  const tr = el('tr');
  [ts, dir.toUpperCase(), id, typeName, data].forEach((v, i) => {
    tr.appendChild(el('td', i === 1 ? `dir-${dir}` : '', v));
  });
  body.appendChild(tr);
  while (body.children.length > MAX_LOG_ROWS) body.removeChild(body.firstChild);
  if ($('autoScroll').checked) body.parentElement.parentElement.scrollTop = 1e9;
}

function renderTelemetry(d) {
  if (S.motorId !== null && d.motorId !== S.motorId) return;
  S.motorMode = d.mode;
  renderModeWarning();
  $('teleAngle').textContent = fmt(d.angle);
  $('teleVel').textContent = fmt(d.velocity);
  $('teleTorque').textContent = fmt(d.torque);
  $('teleTemp').textContent = fmt(d.temperature, 1);
  const mode = $('teleMode');
  mode.textContent = `モード ${d.modeName}`;
  mode.className = `badge${d.mode === 2 ? ' run' : ''}`;
  const f = $('teleFaults');
  f.innerHTML = '';
  d.faults.forEach((n) => f.appendChild(el('span', 'badge fault', n)));
}

// =========================================================================
// モータ探索
// =========================================================================
function renderMotorList() {
  const ul = $('motorList');
  ul.innerHTML = '';
  if (!S.motors.length) { ul.appendChild(el('li', 'empty', '未検出')); return; }
  S.motors.forEach((m) => {
    const li = el('li', m.motorId === S.motorId ? 'sel' : '');
    li.appendChild(el('span', null, `CAN_ID ${m.motorId}`));
    li.appendChild(el('span', 'uid', m.uid ? m.uid.slice(0, 12) : m.detectedBy));
    li.onclick = () => selectMotor(m.motorId);
    ul.appendChild(li);
  });
}

function selectMotor(id) {
  S.motorId = id;
  S.values = {};
  S.dirty = {};
  S.motorMode = null;
  client.setModel(id, S.model);
  S.schema = { profile: getProfile(S.model), params: schemaFor(getProfile(S.model)) };
  renderMotorList();
  renderRunModes();
  renderMotionGrid();
  renderParams();
  updateEnabled();
}

// =========================================================================
// パラメータ表
// =========================================================================
const paramDefs = () => (S.schema ? S.schema.params : []);

function renderParams() {
  const host = $('paramGroups');
  host.innerHTML = '';
  if (!S.schema) {
    host.appendChild(el('p', 'empty-state', '接続してモータを選択してください。'));
    return;
  }
  const groups = {};
  paramDefs().forEach((p) => { (groups[p.group] ||= []).push(p); });
  Object.entries(GROUP_LABELS).forEach(([key, label]) => {
    if (!groups[key]) return;
    const box = el('div', 'pgroup');
    box.appendChild(el('h3', null, label));
    groups[key].forEach((p) => box.appendChild(paramRow(p)));
    host.appendChild(box);
  });
}

function paramRow(p) {
  const row = el('div', `prow${p.writable ? '' : ' readonly'}`);
  const name = el('div', 'pname');
  name.appendChild(el('b', null, p.label));
  name.appendChild(el('small', null, `${p.name}  ${p.indexHex}`));
  if (p.note) name.title = p.note;
  row.appendChild(name);

  const cur = S.values[p.name];
  row.appendChild(el('div', 'pcur',
    cur === undefined || cur === null ? '–' : (p.type === 'float' ? fmt(cur, 4) : String(cur))));

  if (p.writable) {
    let input;
    if (p.choices) {
      input = el('select');
      p.choices.forEach((c) => {
        const o = el('option', null, c.label);
        o.value = c.value;
        input.appendChild(o);
      });
      if (cur !== undefined && cur !== null) input.value = String(cur);
    } else {
      input = el('input');
      input.type = 'number';
      if (p.step) input.step = p.step;
      if (p.min !== null && p.min !== undefined) input.min = p.min;
      if (p.max !== null && p.max !== undefined) input.max = p.max;
      if (cur !== undefined && cur !== null) input.value = p.type === 'float' ? Number(cur).toFixed(4) : cur;
    }
    input.oninput = () => { S.dirty[p.name] = Number(input.value); input.classList.add('dirty'); };
    row.appendChild(input);

    const range = (p.min !== null && p.min !== undefined && p.max !== null && p.max !== undefined)
      ? `${p.min} 〜 ${p.max}` : (p.choices ? '' : 'レンジ不明');
    row.appendChild(el('div', 'punit', [p.unit, range].filter(Boolean).join('  ')));

    const btn = el('button', 'ghost', '書込');
    btn.onclick = () => writeParam(p, Number(input.value));
    row.appendChild(btn);
  } else {
    row.appendChild(el('div'));
    row.appendChild(el('div', 'punit', p.unit));
    const btn = el('button', 'ghost', '読出');
    btn.onclick = () => readOne(p);
    row.appendChild(btn);
  }
  return row;
}

async function guarded(fn, label) {
  if (S.busy) { toast('前の処理が実行中です', 'warn'); return; }
  S.busy = true;
  try { await fn(); } catch (e) { toast(`${label}: ${e.message}`, 'err'); } finally { S.busy = false; }
}

async function readAll() {
  await guarded(async () => {
    const res = await client.readAll(S.motorId, null, (d, t) => progress('readProgress', d, t));
    let errors = 0;
    Object.entries(res).forEach(([name, v]) => {
      if (v.error) { errors += 1; return; }
      S.values[name] = v.value;
    });
    S.dirty = {};
    renderParams();
    syncRunMode();
    toast(errors ? `読出完了 (${errors} 個は応答なし)` : 'パラメータを読み出しました', errors ? 'warn' : 'ok');
  }, '読出失敗');
}

async function readOne(p) {
  await guarded(async () => {
    S.values[p.name] = await client.readParam(S.motorId, p.index);
    renderParams();
  }, `${p.name} の読出失敗`);
}

async function writeParam(p, value) {
  // 指令値の書き込みは動作を伴うため動作許可を必須にする
  if (p.group === 'command' && !S.armed) {
    toast('動作許可 (ARM) が無効です。ヘッダのトグルを ON にしてください', 'err');
    return;
  }
  await guarded(async () => {
    await client.writeParam(S.motorId, p.index, value);
    const back = await client.readParam(S.motorId, p.index);
    S.values[p.name] = back;
    delete S.dirty[p.name];
    renderParams();
    if (p.name === 'run_mode') syncRunMode();
    toast(`${p.label} = ${p.type === 'float' ? fmt(back, 4) : back}`, 'ok');
  }, `${p.name} の書込失敗`);
}

async function writeDirty() {
  const entries = Object.entries(S.dirty);
  if (!entries.length) { toast('変更されたパラメータはありません', 'warn'); return; }
  await guarded(async () => {
    let ok = 0;
    for (const [name, value] of entries) {
      const p = PARAMS_BY_NAME[name];
      if (!p) continue;
      try {
        await client.writeParam(S.motorId, p.index, value);
        S.values[name] = await client.readParam(S.motorId, p.index);
        delete S.dirty[name];
        ok += 1;
      } catch (e) { toast(`${name}: ${e.message}`, 'err'); }
    }
    renderParams();
    toast(`${ok} / ${entries.length} 件を書き込みました`, ok === entries.length ? 'ok' : 'warn');
  }, '書込失敗');
}

// =========================================================================
// 運転タブ
// =========================================================================
const MODE_COMMANDS = {
  0: [],
  1: ['loc_ref', 'vel_max', 'acc_set', 'limit_cur'],
  2: ['spd_ref', 'acc_rad', 'limit_cur'],
  3: ['iq_ref'],
  5: ['loc_ref', 'limit_spd', 'limit_cur'],
};

function renderRunModes() {
  const sel = $('runModeSelect');
  sel.innerHTML = '';
  RUN_MODE_CHOICES.forEach((c) => {
    const o = el('option', null, c.label);
    o.value = c.value;
    sel.appendChild(o);
  });
  sel.onchange = renderCommandFields;
  renderCommandFields();
}

function syncRunMode() {
  const v = S.values.run_mode;
  if (v === undefined || v === null) return;
  const sel = $('runModeSelect');
  if (!sel || sel.value === String(v)) return;
  if ([...sel.options].some((o) => o.value === String(v))) {
    sel.value = String(v);
    renderCommandFields();
  }
}

/** 指令値を書いても動かない状態 (Reset/Cali) を画面で知らせる。 */
function renderModeWarning() {
  const box = $('modeWarning');
  if (!box) return;
  const mode = Number($('runModeSelect').value);
  if (!MODE_COMMANDS[mode] || !MODE_COMMANDS[mode].length) { box.hidden = true; return; }
  box.hidden = false;
  if (S.motorMode === 2) {
    box.className = 'mode-warning ok';
    box.textContent = 'モータは Run モードです。指令値を書くと動作します。';
  } else if (S.motorMode === null || S.motorMode === undefined) {
    box.className = 'mode-warning';
    box.textContent = '「運転許可」を押すまでモータは動きません。'
      + '位置モードでは無効の間 loc_ref が現在位置に追従するため、値を書いても動かないように見えます。';
  } else {
    box.className = 'mode-warning';
    box.textContent = `モータは ${S.motorMode === 1 ? 'Cali' : 'Reset'} モードです。`
      + '「運転許可」を押すまで指令値を書いても動きません。';
  }
}

function renderCommandFields() {
  const host = $('commandFields');
  host.innerHTML = '';
  const mode = Number($('runModeSelect').value);
  const names = MODE_COMMANDS[mode] || [];
  if (!names.length) {
    host.appendChild(el('p', 'empty-state', '運動制御モードでは下の「運動制御モード」パネルを使用します。'));
    renderModeWarning();
    return;
  }
  names.forEach((name) => {
    const p = paramDefs().find((x) => x.name === name);
    if (!p) return;
    const wrap = el('label', 'field');
    wrap.appendChild(el('span', null, `${p.label} (${p.name})${p.unit ? ` [${p.unit}]` : ''}`));
    const row = el('div', 'slider-row');
    const hasRange = p.min !== null && p.min !== undefined && p.max !== null && p.max !== undefined;
    const num = el('input');
    num.type = 'number';
    num.className = 'cmd-input';
    num.step = p.step || 0.01;
    num.value = S.values[p.name] !== undefined ? Number(S.values[p.name]).toFixed(3) : 0;
    if (hasRange) {
      const rng = el('input');
      rng.type = 'range';
      rng.className = 'cmd-input';
      rng.min = p.min; rng.max = p.max; rng.step = p.step || 0.01;
      rng.value = num.value;
      rng.oninput = () => { num.value = rng.value; };
      num.oninput = () => { rng.value = num.value; };
      row.appendChild(rng);
    }
    row.appendChild(num);
    const btn = el('button', 'primary cmd-input', '送信');
    btn.onclick = () => writeParam(p, Number(num.value));
    row.appendChild(btn);
    wrap.appendChild(row);
    host.appendChild(wrap);
  });
  renderModeWarning();
  updateEnabled();
}

const MOTION_FIELDS = [
  { key: 'position', label: '目標位置 p_set', unit: 'rad', bounds: (pr) => [pr.p_min, pr.p_max], step: 0.01 },
  { key: 'velocity', label: '目標速度 v_set', unit: 'rad/s', bounds: (pr) => [pr.v_min, pr.v_max], step: 0.1 },
  { key: 'torque', label: 'トルク t_ff', unit: 'Nm', bounds: (pr) => [pr.t_min, pr.t_max], step: 0.05 },
  { key: 'kp', label: 'Kp', unit: '', bounds: (pr) => [0, pr.kp_max], step: 0.1 },
  { key: 'kd', label: 'Kd', unit: '', bounds: (pr) => [0, pr.kd_max], step: 0.01 },
];

function renderMotionGrid() {
  const host = $('motionGrid');
  host.innerHTML = '';
  const pr = S.schema.profile;
  MOTION_FIELDS.forEach((f) => {
    const [lo, hi] = f.bounds(pr);
    const wrap = el('label', 'field');
    wrap.appendChild(el('span', null, `${f.label}${f.unit ? ` [${f.unit}]` : ''}  ${lo} 〜 ${hi}`));
    const input = el('input');
    input.type = 'number';
    input.className = 'cmd-input';
    input.id = `motion_${f.key}`;
    input.min = lo; input.max = hi; input.step = f.step;
    input.value = 0;
    wrap.appendChild(input);
    host.appendChild(wrap);
  });
}

function motionPayload() {
  const out = {};
  MOTION_FIELDS.forEach((f) => { out[f.key] = Number($(`motion_${f.key}`).value) || 0; });
  return out;
}

async function motionSend() {
  try {
    await client.motionControl(S.motorId, motionPayload());
  } catch (e) {
    stopStream();
    toast(`運動制御失敗: ${e.message}`, 'err');
  }
}

function stopStream() {
  if (S.streamTimer) clearInterval(S.streamTimer);
  S.streamTimer = null;
  $('motionStreamBtn').textContent = '連続送信 開始';
  $('motionRate').textContent = '';
}

function toggleStream() {
  if (S.streamTimer) { stopStream(); return; }
  S.streamTimer = setInterval(motionSend, 50);
  $('motionStreamBtn').textContent = '連続送信 停止';
  $('motionRate').textContent = '20 Hz で送信中';
}

// =========================================================================
// イベント配線
// =========================================================================
function wire() {
  $('connectBtn').onclick = async () => {
    try {
      const port = await WebSerialTransport.requestPort();
      await transport.open(port, Number($('baudSelect').value));
      client.hostId = Number($('hostId').value);
      S.connected = true;
      setStatus();
      updateEnabled();
      toast('接続しました', 'ok');
    } catch (e) {
      if (e?.name === 'NotFoundError') return;   // ユーザーがダイアログをキャンセル
      toast(`接続失敗: ${e.message}`, 'err');
    }
  };

  $('disconnectBtn').onclick = async () => {
    stopStream();
    await transport.close();
    S.connected = false;
    S.armed = false;
    $('armToggle').checked = false;
    setStatus();
    updateEnabled();
    toast('切断しました');
  };

  $('armToggle').onchange = async (ev) => {
    S.armed = ev.target.checked;
    if (!S.armed) {
      for (const m of S.motors) {
        try { await client.stopNoWait(m.motorId); } catch { /* 無視 */ }
      }
    }
    updateEnabled();
    toast(S.armed ? '動作許可 ON — 可動範囲に注意してください' : '動作許可 OFF', S.armed ? 'warn' : '');
  };

  $('estopBtn').onclick = async () => {
    stopStream();
    S.armed = false;
    $('armToggle').checked = false;
    updateEnabled();
    const ids = S.motors.length ? S.motors.map((m) => m.motorId) : [S.motorId ?? 0];
    for (const id of ids) {
      try { await client.stopNoWait(id); } catch { /* 無視 */ }
    }
    toast(`非常停止を送信しました (ID: ${ids.join(', ')})`, 'warn');
  };

  $('scanBtn').onclick = async () => {
    const btn = $('scanBtn');
    btn.disabled = true; btn.textContent = 'スキャン中…';
    try {
      S.motors = await client.scan(Number($('scanStart').value), Number($('scanEnd').value),
        { onProgress: (d, t) => progress('scanProgress', d, t) });
      renderMotorList();
      if (S.motors.length) {
        selectMotor(S.motors[0].motorId);
        toast(`${S.motors.length} 台のモータを検出しました`, 'ok');
      } else {
        toast('モータが見つかりません。モータの電源・CAN H/L の結線・終端抵抗を確認してください。'
          + (isMacOS() ? ' macOS では WCH 公式ドライバが必要です (左の案内を参照)。' : ''), 'warn');
      }
    } catch (e) { toast(`スキャン失敗: ${e.message}`, 'err'); }
    progress('scanProgress', 1, 1);
    btn.textContent = 'スキャン';
    updateEnabled();
  };

  $('modelSelect').onchange = () => {
    S.model = $('modelSelect').value;
    if (S.motorId !== null) selectMotor(S.motorId);
    toast(`機種を ${S.model} に設定しました`);
  };

  $('readAllBtn').onclick = readAll;
  $('writeDirtyBtn').onclick = writeDirty;

  $('saveBtn').onclick = () => guarded(async () => {
    if (!confirm('現在のパラメータをモータの不揮発メモリへ保存します。よろしいですか?')) return;
    await client.save(S.motorId);
    toast('保存コマンドを送信しました', 'ok');
  }, '保存失敗');

  $('exportBtn').onclick = () => {
    const blob = new Blob([JSON.stringify({
      motor_id: S.motorId, model: S.model, values: S.values,
      exported_at: new Date().toISOString(),
    }, null, 2)], { type: 'application/json' });
    const a = el('a');
    a.href = URL.createObjectURL(blob);
    a.download = `robstride_${S.model}_id${S.motorId}.json`;
    a.click();
    URL.revokeObjectURL(a.href);
  };

  $('importBtn').onclick = () => $('importFile').click();
  $('importFile').onchange = async (ev) => {
    const file = ev.target.files[0];
    if (!file) return;
    try {
      const data = JSON.parse(await file.text());
      let n = 0;
      Object.entries(data.values || {}).forEach(([name, value]) => {
        const p = paramDefs().find((x) => x.name === name);
        if (p && p.writable && typeof value === 'number') { S.dirty[name] = value; n += 1; }
      });
      renderParams();
      toast(`${n} 件を読み込みました。「変更分を書込」で反映します。`, 'ok');
    } catch (e) { toast(`インポート失敗: ${e.message}`, 'err'); }
    ev.target.value = '';
  };

  $('applyModeBtn').onclick = async () => {
    const p = paramDefs().find((x) => x.name === 'run_mode');
    await writeParam(p, Number($('runModeSelect').value));
    renderCommandFields();
  };

  $('enableBtn').onclick = () => motorAction(() => client.enable(S.motorId), '運転許可しました');
  $('stopBtn').onclick = () => { stopStream(); motorAction(() => client.stop(S.motorId, false), '停止しました'); };
  $('clearFaultBtn').onclick = () => motorAction(() => client.stop(S.motorId, true), '故障をクリアしました');

  $('zeroBtn').onclick = () => {
    if (!confirm('現在位置を機械原点 (0 rad) として設定します。よろしいですか?')) return;
    motorAction(() => client.setZero(S.motorId), '機械原点を設定しました');
  };

  $('motionSendBtn').onclick = motionSend;
  $('motionStreamBtn').onclick = toggleStream;

  $('reportToggle').onchange = async (ev) => {
    try {
      await client.setActiveReport(S.motorId, ev.target.checked);
      toast(ev.target.checked ? '能動送信を有効にしました' : '能動送信を無効にしました');
    } catch (e) { toast(e.message, 'err'); ev.target.checked = !ev.target.checked; }
  };

  $('infoBtn').onclick = () => guarded(async () => {
    const info = await client.getDeviceId(S.motorId);
    const ver = await client.readVersion(S.motorId);
    const dl = $('deviceInfo');
    dl.innerHTML = '';
    [['CAN_ID', S.motorId],
     ['機種 (UI 設定)', S.model],
     ['MCU 固有 ID', info ? info.uid : '取得できません'],
     ['応答 ID', info ? info.rawId : '—'],
     ['ファームウェア', ver ? ver.version : '取得できません']].forEach(([k, v]) => {
      dl.appendChild(el('dt', null, k));
      dl.appendChild(el('dd', null, String(v)));
    });
  }, '情報取得失敗');

  $('canIdBtn').onclick = () => guarded(async () => {
    const newId = Number($('newCanId').value);
    if (!confirm(`CAN_ID を ${S.motorId} → ${newId} に変更します。即時反映されます。よろしいですか?`)) return;
    await client.setCanId(S.motorId, newId);
    toast(`CAN_ID を ${newId} に変更しました。再スキャンしてください。`, 'ok');
    S.motorId = null; S.motors = [];
    renderMotorList(); updateEnabled();
  }, 'CAN_ID 変更失敗');

  $('canBaudBtn').onclick = () => guarded(async () => {
    const v = Number($('canBaudSelect').value);
    if (!confirm(`モータ側の CAN ボーレートを ${v} bps に変更します。\n`
      + 'アダプタ側の設定も合わせないと通信できなくなります。よろしいですか?')) return;
    await client.setCanBaudrate(S.motorId, v);
    toast('ボーレート変更を送信しました。モータを再起動してください。', 'warn');
  }, 'ボーレート変更失敗');

  $('protocolBtn').onclick = () => guarded(async () => {
    const v = $('protocolSelect').value;
    if (!confirm(`通信プロトコルを ${v} に変更します。\n`
      + 'private 以外にすると本ツールからは操作できなくなります。よろしいですか?')) return;
    await client.setProtocol(S.motorId, v);
    toast('プロトコル変更を送信しました。モータを再起動してください。', 'warn');
  }, 'プロトコル変更失敗');

  $('idxScanBtn').onclick = () => guarded(async () => {
    const btn = $('idxScanBtn');
    btn.textContent = '調査中…';
    const found = await client.scanIndices(S.motorId, parseHex($('idxStart').value), parseHex($('idxEnd').value));
    const host = $('idxResult');
    host.innerHTML = '';
    if (!found.length) {
      host.appendChild(el('p', 'empty-state', '応答するインデックスはありませんでした。'));
    } else {
      const table = el('table');
      const thead = el('thead'); const htr = el('tr');
      ['index', '既知名', 'raw (LE)', 'float 解釈', 'uint32 解釈'].forEach((h) => htr.appendChild(el('th', null, h)));
      thead.appendChild(htr); table.appendChild(thead);
      const tb = el('tbody');
      found.forEach((r) => {
        const tr = el('tr');
        [r.indexHex, r.known || '—', r.raw, fmt(r.asFloat, 5), r.asUint32]
          .forEach((v) => tr.appendChild(el('td', null, String(v))));
        tb.appendChild(tr);
      });
      table.appendChild(tb); host.appendChild(table);
    }
    btn.textContent = '調査開始';
    toast(`${found.length} 個のインデックスが応答しました`, 'ok');
  }, '調査失敗');

  $('rawSendBtn').onclick = async () => {
    try {
      const bytes = $('rawData').value.replace(/,/g, ' ').replace(/0x/gi, '').trim().split(/\s+/)
        .filter(Boolean).map((h) => parseInt(h, 16));
      if (bytes.some(Number.isNaN)) throw new Error('データは 16 進数で指定してください');
      if (bytes.length > 8) throw new Error('データは最大 8 バイトです');
      await transport.sendFrame(parseHex($('rawId').value), Uint8Array.from(bytes));
    } catch (e) { toast(`送信失敗: ${e.message}`, 'err'); }
  };

  $('atModeBtn').onclick = async () => {
    try {
      await transport.sendRaw(new TextEncoder().encode('AT+AT\r\n'));
      toast('AT+AT を送信しました');
    } catch (e) { toast(e.message, 'err'); }
  };

  $('clearLogBtn').onclick = () => { $('logBody').innerHTML = ''; };

  document.querySelectorAll('.tab').forEach((t) => {
    t.onclick = () => {
      document.querySelectorAll('.tab').forEach((x) => x.classList.remove('active'));
      document.querySelectorAll('.panel').forEach((x) => x.classList.remove('active'));
      t.classList.add('active');
      $(`tab-${t.dataset.tab}`).classList.add('active');
    };
  });

  document.addEventListener('keydown', (ev) => {
    if (ev.code === 'Space' && ev.target === document.body) {
      ev.preventDefault();
      $('estopBtn').click();
    }
  });

  // ケーブルを抜かれたときの後始末
  navigator.serial?.addEventListener?.('disconnect', () => {
    if (!S.connected) return;
    stopStream();
    S.connected = false; S.armed = false;
    $('armToggle').checked = false;
    setStatus(); updateEnabled();
    toast('アダプタが切断されました', 'err');
  });
}

async function motorAction(fn, okMsg) {
  await guarded(async () => {
    const fb = await fn();
    if (fb) renderTelemetry(fb);
    toast(okMsg, 'ok');
  }, okMsg.replace(/しました$/, '失敗'));
}

// =========================================================================
// 起動
// =========================================================================
(function init() {
  if (!isSupported()) {
    $('app').hidden = true;
    $('unsupported').hidden = false;
    return;
  }
  wire();

  const b = $('baudSelect');
  SUPPORTED_BAUDRATES.forEach((v) => {
    const o = el('option', null, `${v} bps`);
    o.value = v;
    if (v === DEFAULT_BAUDRATE) o.selected = true;
    b.appendChild(o);
  });

  const sel = $('modelSelect');
  Object.values(PROFILES).forEach((m) => {
    const o = el('option', null, `${m.label} — ±${m.t_max}Nm / ±${m.v_max}rad/s`);
    o.value = m.key;
    sel.appendChild(o);
  });
  sel.value = S.model;

  const cb = $('canBaudSelect');
  Object.keys(BAUD_CODES).map(Number).sort((a, x) => x - a).forEach((v) => {
    const o = el('option', null, `${v / 1000} kbps`);
    o.value = v;
    cb.appendChild(o);
  });
  const ps = $('protocolSelect');
  [['private', 'private (RobStride 私有プロトコル)'], ['canopen', 'CANopen'], ['mit', 'MIT']]
    .forEach(([v, label]) => {
      const o = el('option', null, label);
      o.value = v;
      ps.appendChild(o);
    });
  void PROTOCOL_CODES;

  // macOS は標準の CH340 ドライバでは通信できないため案内を出す
  const isMac = /Mac/i.test(navigator.userAgentData?.platform ?? navigator.platform ?? '');
  if (isMac) $('macNote').hidden = false;

  setStatus();
  updateEnabled();
}());
