'use strict';

// =========================================================================
// 共通ユーティリティ
// =========================================================================
const $ = (id) => document.getElementById(id);
const el = (tag, cls, text) => {
  const n = document.createElement(tag);
  if (cls) n.className = cls;
  if (text !== undefined) n.textContent = text;
  return n;
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

async function api(path, opts = {}) {
  const res = await fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...opts,
    body: opts.body ? JSON.stringify(opts.body) : undefined,
  });
  let payload = null;
  try { payload = await res.json(); } catch { /* 本文なし */ }
  if (!res.ok) {
    throw new Error((payload && (payload.detail || payload.message)) || `HTTP ${res.status}`);
  }
  return payload;
}

const fmt = (v, digits = 3) =>
  (v === null || v === undefined || Number.isNaN(v)) ? '–' : Number(v).toFixed(digits);

const parseHex = (s) => {
  const t = String(s).trim().replace(/^0x/i, '');
  const v = parseInt(t, 16);
  if (Number.isNaN(v)) throw new Error(`16進数として解釈できません: ${s}`);
  return v;
};

// =========================================================================
// 状態
// =========================================================================
const S = {
  connected: false,
  armed: false,
  motorId: null,
  model: 'RS00',
  schema: null,
  values: {},        // name -> 値 (最後に読んだ値)
  dirty: {},         // name -> 入力中の値
  motors: [],
  motorMode: null,      // 直近のフィードバックが示すモータの状態
  streamTimer: null,
  statusTimer: null,
  ws: null,
};

// =========================================================================
// WebSocket
// =========================================================================
function connectWs() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  S.ws = ws;
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.event === 'can') appendLog(msg.frame);
    else if (msg.event === 'telemetry') renderTelemetry(msg.data);
    else if (msg.event === 'fault') {
      const d = msg.data;
      if (d.faults.length) toast(`故障通知 (ID ${d.motor_id}): ${d.faults.join(' / ')}`, 'err');
    } else if (msg.event === 'hello') applyStatus(msg.status);
  };
  ws.onclose = () => setTimeout(connectWs, 1500);
  // keepalive
  setInterval(() => { if (ws.readyState === 1) ws.send('ping'); }, 20000);
}

// =========================================================================
// 接続まわり
// =========================================================================
async function loadPorts() {
  const d = await api('/api/ports');
  const list = $('portOptions');
  list.innerHTML = '';
  d.ports.forEach((p) => {
    const o = el('option');
    o.value = p.device;
    o.label = p.chip ? `${p.device} (${p.chip})` : p.device;
    list.appendChild(o);
  });
  // 未入力なら USB-CAN らしきポートを既定で入れる (手入力も可)
  const likely = d.ports.find((p) => p.likely_adapter) || d.ports[0];
  const sel = $('portSelect');
  if (!sel.value && likely) sel.value = likely.device;
  $('portHint').textContent = likely && likely.likely_adapter
    ? `USB-CAN アダプタらしきポートを検出: ${likely.device} (${likely.chip})`
    : 'CH340/CP210x 系のポートが見つかりません。パスを直接入力することもできます。';

  const b = $('baudSelect');
  b.innerHTML = '';
  d.baudrates.forEach((v) => {
    const o = el('option', null, `${v} bps`);
    o.value = v;
    if (v === d.default_baudrate) o.selected = true;
    b.appendChild(o);
  });
}

function applyStatus(st) {
  const wasConnected = S.connected;
  S.connected = st.connected;
  S.armed = st.armed;
  $('connPill').classList.toggle('on', st.connected);
  $('connText').textContent = st.connected
    ? `${st.port} @ ${st.baudrate}  TX ${st.tx_count} / RX ${st.rx_count}`
    : '未接続';
  $('armToggle').checked = st.armed;
  // リロード後に接続中のポートと速度を欄へ反映する (表示と実接続の食い違い防止)
  if (st.connected) {
    $('portSelect').value = st.port;
    $('baudSelect').value = String(st.baudrate);
  }
  if (st.last_error && !wasConnected) toast(`シリアルエラー: ${st.last_error}`, 'err');
  updateEnabled();
  // 接続中は TX/RX カウンタと ARM 状態をサーバから定期取得する
  if (st.connected && !S.statusTimer) {
    S.statusTimer = setInterval(pollStatus, 1000);
  } else if (!st.connected && S.statusTimer) {
    clearInterval(S.statusTimer);
    S.statusTimer = null;
  }
}

async function pollStatus() {
  try {
    const st = await api('/api/status');
    S.connected = st.connected;
    S.armed = st.armed;
    $('connPill').classList.toggle('on', st.connected);
    $('connText').textContent = st.connected
      ? `${st.port} @ ${st.baudrate}  TX ${st.tx_count} / RX ${st.rx_count}`
      : '未接続';
    $('armToggle').checked = st.armed;
    if (!st.connected) {
      clearInterval(S.statusTimer);
      S.statusTimer = null;
      updateEnabled();
    }
  } catch { /* 一時的な失敗は無視 */ }
}

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

// =========================================================================
// モータ探索
// =========================================================================
function renderMotorList() {
  const ul = $('motorList');
  ul.innerHTML = '';
  if (!S.motors.length) {
    ul.appendChild(el('li', 'empty', '未検出'));
    return;
  }
  S.motors.forEach((m) => {
    const li = el('li', m.motor_id === S.motorId ? 'sel' : '');
    li.appendChild(el('span', null, `CAN_ID ${m.motor_id}`));
    li.appendChild(el('span', 'uid', m.uid ? m.uid.slice(0, 12) : m.detected_by));
    li.onclick = () => selectMotor(m.motor_id);
    ul.appendChild(li);
  });
}

async function selectMotor(id) {
  S.motorId = id;
  S.values = {};
  S.dirty = {};
  S.motorMode = null;
  renderMotorList();
  await api(`/api/motor/${id}/model`, { method: 'POST', body: { model: S.model } });
  S.schema = await api(`/api/motor/${id}/schema`);
  renderRunModes();
  renderMotionGrid();
  renderParams();
  updateEnabled();
}

// =========================================================================
// パラメータ表
// =========================================================================
function paramDefs() { return S.schema ? S.schema.params : []; }

function renderParams() {
  const host = $('paramGroups');
  host.innerHTML = '';
  if (!S.schema) {
    host.appendChild(el('p', 'empty-state', '接続してモータを選択してください。'));
    return;
  }
  const groups = {};
  paramDefs().forEach((p) => (groups[p.group] ||= []).push(p));

  S.schema.groups.forEach((g) => {
    const list = groups[g.key];
    if (!list) return;
    const box = el('div', 'pgroup');
    box.appendChild(el('h3', null, g.label));
    list.forEach((p) => box.appendChild(paramRow(p)));
    host.appendChild(box);
  });
}

function paramRow(p) {
  const row = el('div', `prow${p.writable ? '' : ' readonly'}`);

  const name = el('div', 'pname');
  name.appendChild(el('b', null, p.label));
  name.appendChild(el('small', null, `${p.name}  ${p.index_hex}`));
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
    input.dataset.param = p.name;
    input.oninput = () => {
      S.dirty[p.name] = Number(input.value);
      input.classList.add('dirty');
    };
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

async function readAll() {
  try {
    const d = await api(`/api/motor/${S.motorId}/params/read`, { method: 'POST', body: {} });
    let errors = 0;
    Object.entries(d.values).forEach(([name, v]) => {
      if (v.error) { errors += 1; return; }
      S.values[name] = v.value;
    });
    S.dirty = {};
    renderParams();
    syncRunMode();
    toast(errors ? `読出完了 (${errors} 個は応答なし)` : 'パラメータを読み出しました', errors ? 'warn' : 'ok');
  } catch (e) { toast(`読出失敗: ${e.message}`, 'err'); }
}

async function readOne(p) {
  try {
    const d = await api(`/api/motor/${S.motorId}/params/read`, {
      method: 'POST', body: { indices: [p.index] },
    });
    const v = Object.values(d.values)[0];
    if (v.error) throw new Error(v.error);
    S.values[p.name] = v.value;
    renderParams();
  } catch (e) { toast(`${p.name} の読出失敗: ${e.message}`, 'err'); }
}

async function writeParam(p, value) {
  try {
    const d = await api(`/api/motor/${S.motorId}/param`, {
      method: 'POST', body: { name: p.name, value },
    });
    S.values[p.name] = d.readback;
    delete S.dirty[p.name];
    renderParams();
    if (p.name === 'run_mode') syncRunMode();
    toast(`${p.label} = ${p.type === 'float' ? fmt(d.readback, 4) : d.readback}`, 'ok');
  } catch (e) { toast(`${p.name} の書込失敗: ${e.message}`, 'err'); }
}

async function writeDirty() {
  const entries = Object.entries(S.dirty);
  if (!entries.length) { toast('変更されたパラメータはありません', 'warn'); return; }
  let ok = 0;
  for (const [name, value] of entries) {
    const p = paramDefs().find((x) => x.name === name);
    if (!p) continue;
    try {
      const d = await api(`/api/motor/${S.motorId}/param`, { method: 'POST', body: { name, value } });
      S.values[name] = d.readback;
      delete S.dirty[name];
      ok += 1;
    } catch (e) { toast(`${name}: ${e.message}`, 'err'); }
  }
  renderParams();
  toast(`${ok} / ${entries.length} 件を書き込みました`, ok === entries.length ? 'ok' : 'warn');
}

// =========================================================================
// 運転タブ
// =========================================================================
/** 運転タブのモード選択を、モータから読んだ run_mode に合わせる。 */
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

function renderRunModes() {
  const sel = $('runModeSelect');
  sel.innerHTML = '';
  (S.schema.run_modes || []).forEach((c) => {
    const o = el('option', null, c.label);
    o.value = c.value;
    sel.appendChild(o);
  });
  sel.onchange = renderCommandFields;
  renderCommandFields();
}

// モード別に「触るべき指令パラメータ」を並べる (マニュアルの制御手順に対応)
const MODE_COMMANDS = {
  0: [],
  1: ['loc_ref', 'vel_max', 'acc_set', 'limit_cur'],
  2: ['spd_ref', 'acc_rad', 'limit_cur'],
  3: ['iq_ref'],
  5: ['loc_ref', 'limit_spd', 'limit_cur'],
};

/** 指令値を書いても動かない状態 (Reset/Cali) を画面で知らせる。 */
function renderModeWarning() {
  const box = $('modeWarning');
  if (!box) return;
  const mode = Number($('runModeSelect').value);
  // 運動制御モードは専用パネルを使うので、この欄には出さない
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
    const name = S.motorMode === 1 ? 'Cali' : 'Reset';
    box.className = 'mode-warning';
    box.textContent = `モータは ${name} モードです。「運転許可」を押すまで指令値を書いても動きません。`;
  }
}

function renderCommandFields() {
  const host = $('commandFields');
  host.innerHTML = '';
  const mode = Number($('runModeSelect').value);
  const names = MODE_COMMANDS[mode] || [];
  if (!names.length) {
    host.appendChild(el('p', 'empty-state', '運動制御モードでは下の「運動制御モード」パネルを使用します。'));
    return;
  }
  names.forEach((name) => {
    const p = paramDefs().find((x) => x.name === name);
    if (!p) return;
    const wrap = el('label', 'field');
    wrap.appendChild(el('span', null,
      `${p.label} (${p.name})${p.unit ? ' [' + p.unit + ']' : ''}`));
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
    wrap.appendChild(el('span', null, `${f.label}${f.unit ? ' [' + f.unit + ']' : ''}  ${lo} 〜 ${hi}`));
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
  const body = {};
  MOTION_FIELDS.forEach((f) => { body[f.key] = Number($(`motion_${f.key}`).value) || 0; });
  return body;
}

async function motionSend() {
  try {
    await api(`/api/motor/${S.motorId}/control`, { method: 'POST', body: motionPayload() });
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
  S.streamTimer = setInterval(motionSend, 50);   // 20Hz
  $('motionStreamBtn').textContent = '連続送信 停止';
  $('motionRate').textContent = '20 Hz で送信中';
}

// =========================================================================
// テレメトリ / ログ
// =========================================================================
function renderTelemetry(d) {
  if (S.motorId !== null && d.motor_id !== S.motorId) return;
  S.motorMode = d.mode;
  renderModeWarning();
  $('teleAngle').textContent = fmt(d.angle);
  $('teleVel').textContent = fmt(d.velocity);
  $('teleTorque').textContent = fmt(d.torque);
  $('teleTemp').textContent = fmt(d.temperature, 1);
  const mode = $('teleMode');
  mode.textContent = `モード ${d.mode_name}`;
  mode.className = `badge${d.mode === 2 ? ' run' : ''}`;
  const f = $('teleFaults');
  f.innerHTML = '';
  d.faults.forEach((name) => f.appendChild(el('span', 'badge fault', name)));
}

const MAX_LOG_ROWS = 500;

function appendLog(fr) {
  const body = $('logBody');
  const tr = el('tr');
  const t = new Date(fr.t * 1000);
  const ts = `${String(t.getHours()).padStart(2, '0')}:${String(t.getMinutes()).padStart(2, '0')}:` +
             `${String(t.getSeconds()).padStart(2, '0')}.${String(t.getMilliseconds()).padStart(3, '0')}`;
  [ts, fr.dir.toUpperCase(), fr.id, fr.type_name, fr.data].forEach((v, i) => {
    const td = el('td', i === 1 ? `dir-${fr.dir}` : '', v);
    tr.appendChild(td);
  });
  body.appendChild(tr);
  while (body.children.length > MAX_LOG_ROWS) body.removeChild(body.firstChild);
  if ($('autoScroll').checked) body.parentElement.parentElement.scrollTop = 1e9;
}

// =========================================================================
// イベント配線
// =========================================================================
function wire() {
  $('refreshPorts').onclick = () => loadPorts().then(() => toast('ポートを再検索しました'));

  $('connectBtn').onclick = async () => {
    try {
      const st = await api('/api/connect', {
        method: 'POST',
        body: {
          port: $('portSelect').value,
          baudrate: Number($('baudSelect').value),
          host_id: Number($('hostId').value),
        },
      });
      applyStatus(st);
      toast(`接続しました: ${st.port}`, 'ok');
    } catch (e) { toast(`接続失敗: ${e.message}`, 'err'); }
  };

  $('disconnectBtn').onclick = async () => {
    stopStream();
    applyStatus(await api('/api/disconnect', { method: 'POST' }));
    toast('切断しました');
  };

  $('armToggle').onchange = async (ev) => {
    try {
      const d = await api('/api/arm', { method: 'POST', body: { armed: ev.target.checked } });
      S.armed = d.armed;
      updateEnabled();
      toast(d.armed ? '動作許可 ON — 可動範囲に注意してください' : '動作許可 OFF', d.armed ? 'warn' : '');
    } catch (e) {
      ev.target.checked = false;
      toast(e.message, 'err');
    }
  };

  $('estopBtn').onclick = async () => {
    stopStream();
    try {
      const d = await api('/api/estop', { method: 'POST' });
      S.armed = false;
      $('armToggle').checked = false;
      updateEnabled();
      toast(`非常停止を送信しました (ID: ${d.stopped.join(', ') || 'なし'})`, 'warn');
    } catch (e) { toast(e.message, 'err'); }
  };

  $('scanBtn').onclick = async () => {
    $('scanBtn').textContent = 'スキャン中…';
    $('scanBtn').disabled = true;
    try {
      const d = await api('/api/scan', {
        method: 'POST',
        body: { start: Number($('scanStart').value), end: Number($('scanEnd').value) },
      });
      S.motors = d.motors;
      renderMotorList();
      if (d.motors.length) {
        await selectMotor(d.motors[0].motor_id);
        toast(`${d.motors.length} 台のモータを検出しました`, 'ok');
      } else {
        toast('モータが見つかりません。電源・結線・終端抵抗・CAN ボーレートを確認してください。', 'warn');
      }
    } catch (e) { toast(`スキャン失敗: ${e.message}`, 'err'); }
    $('scanBtn').textContent = 'スキャン';
    updateEnabled();
  };

  $('modelSelect').onchange = async () => {
    S.model = $('modelSelect').value;
    if (S.motorId !== null) await selectMotor(S.motorId);
    toast(`機種を ${S.model} に設定しました`);
  };

  $('readAllBtn').onclick = readAll;
  $('writeDirtyBtn').onclick = writeDirty;

  $('saveBtn').onclick = async () => {
    if (!confirm('現在のパラメータをモータの不揮発メモリへ保存します。よろしいですか?')) return;
    try {
      await api(`/api/motor/${S.motorId}/save`, { method: 'POST' });
      toast('保存コマンドを送信しました', 'ok');
    } catch (e) { toast(`保存失敗: ${e.message}`, 'err'); }
  };

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

  $('enableBtn').onclick = () => motorAction('enable', {}, '運転許可しました');
  $('stopBtn').onclick = () => { stopStream(); motorAction('stop', { clear_fault: false }, '停止しました'); };
  $('clearFaultBtn').onclick = () => motorAction('stop', { clear_fault: true }, '故障をクリアしました');

  $('zeroBtn').onclick = async () => {
    if (!confirm('現在位置を機械原点 (0 rad) として設定します。よろしいですか?')) return;
    motorAction('zero', {}, '機械原点を設定しました');
  };

  $('motionSendBtn').onclick = motionSend;
  $('motionStreamBtn').onclick = toggleStream;

  $('reportToggle').onchange = async (ev) => {
    try {
      await api(`/api/motor/${S.motorId}/report`, {
        method: 'POST', body: { enable: ev.target.checked },
      });
      toast(ev.target.checked ? '能動送信を有効にしました' : '能動送信を無効にしました');
    } catch (e) { toast(e.message, 'err'); ev.target.checked = !ev.target.checked; }
  };

  $('infoBtn').onclick = async () => {
    try {
      const d = await api(`/api/motor/${S.motorId}/info`);
      const dl = $('deviceInfo');
      dl.innerHTML = '';
      const rows = [
        ['CAN_ID', d.motor_id],
        ['機種 (UI 設定)', d.model || '未設定'],
        ['MCU 固有 ID', d.device ? d.device.uid : '取得できません'],
        ['応答 ID', d.device ? d.device.raw_id : '—'],
        ['ファームウェア', d.version ? d.version.version : '取得できません'],
      ];
      rows.forEach(([k, v]) => { dl.appendChild(el('dt', null, k)); dl.appendChild(el('dd', null, String(v))); });
    } catch (e) { toast(`情報取得失敗: ${e.message}`, 'err'); }
  };

  $('canIdBtn').onclick = async () => {
    const newId = Number($('newCanId').value);
    if (!confirm(`CAN_ID を ${S.motorId} → ${newId} に変更します。即時反映されます。よろしいですか?`)) return;
    try {
      await api(`/api/motor/${S.motorId}/can-id`, { method: 'POST', body: { new_id: newId } });
      toast(`CAN_ID を ${newId} に変更しました。再スキャンしてください。`, 'ok');
      S.motorId = null;
      S.motors = [];
      renderMotorList();
      updateEnabled();
    } catch (e) { toast(`CAN_ID 変更失敗: ${e.message}`, 'err'); }
  };

  $('canBaudBtn').onclick = async () => {
    const v = Number($('canBaudSelect').value);
    if (!confirm(`モータ側の CAN ボーレートを ${v} bps に変更します。\n` +
                 'アダプタ側の設定も合わせないと通信できなくなります。よろしいですか?')) return;
    try {
      await api(`/api/motor/${S.motorId}/can-baudrate`, { method: 'POST', body: { baudrate: v } });
      toast('ボーレート変更を送信しました。モータを再起動してください。', 'warn');
    } catch (e) { toast(e.message, 'err'); }
  };

  $('protocolBtn').onclick = async () => {
    const v = $('protocolSelect').value;
    if (!confirm(`通信プロトコルを ${v} に変更します。\n` +
                 'private 以外にすると本ツールからは操作できなくなります。よろしいですか?')) return;
    try {
      await api(`/api/motor/${S.motorId}/protocol`, { method: 'POST', body: { protocol: v } });
      toast('プロトコル変更を送信しました。モータを再起動してください。', 'warn');
    } catch (e) { toast(e.message, 'err'); }
  };

  $('idxScanBtn').onclick = async () => {
    const btn = $('idxScanBtn');
    btn.disabled = true; btn.textContent = '調査中…';
    try {
      const d = await api(`/api/motor/${S.motorId}/params/scan`, {
        method: 'POST',
        body: { start: parseHex($('idxStart').value), end: parseHex($('idxEnd').value) },
      });
      const host = $('idxResult');
      host.innerHTML = '';
      if (!d.found.length) { host.appendChild(el('p', 'empty-state', '応答するインデックスはありませんでした。')); }
      else {
        const table = el('table');
        const thead = el('thead');
        const htr = el('tr');
        ['index', '既知名', 'raw (LE)', 'float 解釈', 'uint32 解釈'].forEach((h) => htr.appendChild(el('th', null, h)));
        thead.appendChild(htr); table.appendChild(thead);
        const tb = el('tbody');
        d.found.forEach((r) => {
          const tr = el('tr');
          [r.index_hex, r.known || '—', r.raw, fmt(r.as_float, 5), r.as_uint32]
            .forEach((v) => tr.appendChild(el('td', null, String(v))));
          tb.appendChild(tr);
        });
        table.appendChild(tb); host.appendChild(table);
      }
      toast(`${d.found.length} 個のインデックスが応答しました`, 'ok');
    } catch (e) { toast(`調査失敗: ${e.message}`, 'err'); }
    btn.disabled = false; btn.textContent = '調査開始';
  };

  $('rawSendBtn').onclick = async () => {
    try {
      await api('/api/raw', {
        method: 'POST',
        body: { ext_id: parseHex($('rawId').value), data: $('rawData').value },
      });
    } catch (e) { toast(`送信失敗: ${e.message}`, 'err'); }
  };

  $('atModeBtn').onclick = async () => {
    try {
      await api('/api/adapter/at-mode', { method: 'POST' });
      toast('AT+AT を送信しました');
    } catch (e) { toast(e.message, 'err'); }
  };

  $('clearLogBtn').onclick = async () => {
    await api('/api/log', { method: 'DELETE' });
    $('logBody').innerHTML = '';
  };

  document.querySelectorAll('.tab').forEach((t) => {
    t.onclick = () => {
      document.querySelectorAll('.tab').forEach((x) => x.classList.remove('active'));
      document.querySelectorAll('.panel').forEach((x) => x.classList.remove('active'));
      t.classList.add('active');
      $(`tab-${t.dataset.tab}`).classList.add('active');
    };
  });

  // スペースキーで非常停止
  document.addEventListener('keydown', (ev) => {
    if (ev.code === 'Space' && ev.target === document.body) {
      ev.preventDefault();
      $('estopBtn').click();
    }
  });
}

async function motorAction(path, body, okMsg) {
  try {
    const d = await api(`/api/motor/${S.motorId}/${path}`, { method: 'POST', body });
    if (d.feedback) renderTelemetry(d.feedback);
    toast(okMsg, 'ok');
  } catch (e) { toast(`${okMsg.replace(/しました$/, '')}失敗: ${e.message}`, 'err'); }
}

// =========================================================================
// 起動
// =========================================================================
(async function init() {
  wire();
  connectWs();
  await loadPorts();

  const models = await api('/api/models');
  const sel = $('modelSelect');
  models.models.forEach((m) => {
    const o = el('option', null,
      `${m.label} — ±${m.t_max}Nm / ±${m.v_max}rad/s`);
    o.value = m.key;
    sel.appendChild(o);
  });
  sel.value = S.model;

  const st = await api('/api/status');
  applyStatus(st);

  // 静的な選択肢 (schema 取得前でも埋めておく)
  const cb = $('canBaudSelect');
  [1000000, 500000, 250000, 125000].forEach((v) => {
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

  const log = await api('/api/log?limit=200');
  log.log.forEach(appendLog);
})();
