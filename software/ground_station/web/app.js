/* Icarus ground station - the page's logic.
 *
 * One WebSocket to the Python back end carries everything (message table in
 * docs/ground-station-plan.md §4). With no back end reachable, or with ?demo
 * in the address, a DemoFeed produces the same messages locally so the page
 * can be shown and developed anywhere; the DEMO DATA pill says so.
 *
 * Safety on this side: motor setpoints go out 10 times a second only while
 * "Motors enabled" is on and the socket is open; STOP (button, Space, Esc)
 * sends a stop and switches the motors off. The thrust limit clamps every
 * slider. Readback bars draw what the blimp reports, never the slider.
 */
'use strict';

const $ = (sel) => document.querySelector(sel);
const MOTORS = ['FL', 'RL', 'FR', 'RR'];                 // wire order (motor_id_t)
const PAIR = { FL: 'FR', FR: 'FL', RL: 'RR', RR: 'RL' };
const SERIES = ['--s-x', '--s-y', '--s-z'];
const HOVER_BAND_M = [1.85, 2.25];
const GAUGE_MAX_M = 3.0;
const FOV_DEG = 54;
const HISTORY_S = 10;
const MOTOR_SEND_MS = 100;

const state = {
  transport: 'none',          // 'ws' | 'demo'
  wsOpen: false,
  demo: false,
  enabled: false,
  limit: 0.30,
  pairs: true,
  slider: { FL: 0, RL: 0, FR: 0, RR: 0 },   // -1..1 of the limit
  tel: null, telAt: 0, alive: false,
  sen: null, senAt: 0, senEverSeen: false,
  hist: { t: [], acc: [[], [], []], gyro: [[], [], []] },
  cam: { status: null, frame: null, row: null, rowAt: 0, rotate: false, live: false, rowsSeen: 0, rowTimes: [] },
  events: [],
  conHistory: [], conIndex: -1,
  dockTab: 'console',
};

// ------------------------------------------------------------------ utils

function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }
function clamp(v, a, b) { return Math.max(a, Math.min(b, v)); }
function fmt(v, d = 2) { return (v === null || v === undefined || Number.isNaN(v)) ? '—' : Number(v).toFixed(d); }
function sfmt(v, d = 2) { return (v === null || v === undefined || Number.isNaN(v)) ? '—' : (v >= 0 ? '+' : '') + Number(v).toFixed(d); }
function hhmmss(ts) { const d = new Date(ts * 1000); return d.toTimeString().slice(0, 8); }
function setPill(id, cls, text) { const el = $(id); el.className = 'pill ' + cls; el.querySelector('b').textContent = text; }

// -------------------------------------------------------------- transport

const link = {
  ws: null,
  send(obj) {
    if (state.transport === 'ws' && state.wsOpen) this.ws.send(JSON.stringify(obj));
    else if (state.transport === 'demo') demo.receive(obj);
  },
  connect() {
    if (window.ICARUS_DEMO || new URLSearchParams(location.search).has('demo') || !/^https?:$/.test(location.protocol)) {
      demo.start(); return;
    }
    const url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws';
    let everOpen = false;
    let ws;
    try { ws = new WebSocket(url); } catch (e) { demo.start(); return; }
    this.ws = ws;
    ws.onopen = () => { everOpen = true; state.transport = 'ws'; state.wsOpen = true; addEvent('ok', 'connected to the ground station back end'); };
    ws.onmessage = (e) => handle(JSON.parse(e.data));
    ws.onclose = () => {
      state.wsOpen = false;
      if (!everOpen && state.transport === 'none') {
        addEvent('warn', 'no ground station back end at ' + location.host + ': showing demo data');
        demo.start();
      } else if (state.transport === 'ws') {
        addEvent('crit', 'lost the back end; motors are released by it. Reconnecting…');
        setPill('#pill-link', 'crit', 'OFFLINE');
        setTimeout(() => link.connect(), 2000);
      }
    };
    ws.onerror = () => {};
  },
};

// ----------------------------------------------------------- demo feed

/* The same message shapes the back end sends, from a toy model: the mock
   blimp's physics (mock_blimp.py) and the demo camera's scene (camera.py),
   kept in step by hand. */
const demo = {
  timer: null, t0: 0, seq: 0,
  motor: [0, 0, 0, 0], motorAt: 0, failsafe: 1, tripped: false, lastThrust: 0,
  heading: 42, height: 1.5, vz: 0, lastStep: 0, rxOk: 0,
  camLive: false, camGrab: 0, camSeq: 0,
  start() {
    if (state.transport === 'demo') return;
    state.transport = 'demo'; state.demo = true;
    $('#pill-demo').hidden = false;
    handle({ t: 'hello', demo: true, blimp: 'demo', camera: { source: 'demo', connected: true, live: false, port: 'demo' },
             deadman_s: 0.5, stream_hz: 20, events: [] });
    addEvent('info', 'demo data: nothing here touches a real blimp');
    this.t0 = performance.now(); this.lastStep = this.t0;
    this.timer = setInterval(() => this.tick(), 100);
    this.camTimer = setInterval(() => this.camTick(), 80);
  },
  receive(m) {
    const now = performance.now();
    if (m.t === 'motor') { this.motor = m.cmd.map((c) => clamp(c, -1, 1)); this.motorAt = now; this.lastThrust = now; this.tripped = false; }
    else if (m.t === 'stop') { this.motor = [0, 0, 0, 0]; this.motorAt = 0; this.tripped = false; addEvent('ok', 'STOP sent: all motors ' + (m.brake ? 'braked' : 'coasting')); }
    else if (m.t === 'con') setTimeout(() => this.console(m.line), 120);
    else if (m.t === 'cam') {
      if (m.action === 'grab') this.camGrab++;
      if (m.action === 'live') this.camLive = !!m.on;
      handle({ t: 'camstat', source: 'demo', connected: true, live: this.camLive, port: 'demo', baud: 0, frames_ok: this.camSeq, frames_bad: 0 });
      addEvent('info', 'camera: ' + (m.action === 'live' ? 'live ' + (this.camLive ? 'on' : 'off') : m.action));
    }
  },
  tick() {
    const now = performance.now();
    // setpoint deadman (back end) then the motor failsafe (firmware)
    if (this.motorAt && now - this.motorAt > 500 && this.motor.some((c) => c)) { this.motor = [0, 0, 0, 0]; this.motorAt = 0; addEvent('warn', 'setpoint from the page went stale for 500 ms: motors released'); }
    if (this.motor.some((c) => c) && now - this.lastThrust > 1000) { this.motor = [0, 0, 0, 0]; this.tripped = true; addEvent('crit', 'motor failsafe TRIPPED on the blimp: all motors coasting'); }
    const fs = this.tripped ? 'TRIPPED' : (this.motor.some((c) => c) ? 'armed' : 'idle');
    this.rxOk += 1;
    const t_us = Math.round((now - this.t0) * 1000) >>> 0;
    handle({ t: 'tel', alive: true, age_s: 0.02, rx: this.rxOk, bad: 0, streaming: !!this.motorAt,
      tel: { t_us, motor: { FL: this.motor[0], RL: this.motor[1], FR: this.motor[2], RR: this.motor[3] }, failsafe: fs,
             link_alive: true, link_motors: !!this.motorAt, stations: 1, rx_ok: this.rxOk, rx_bad: 0, free_heap: 184320, link_age_ms: 20 } });
    // toy airframe
    const dt = Math.min(0.2, (now - this.lastStep) / 1000); this.lastStep = now;
    const [fl, rl, fr, rr] = this.motor;
    const yawRate = 25 * (fl - fr);
    this.heading = (this.heading + yawRate * dt + 360) % 360;
    this.vz += (0.8 * (rl + rr) - 0.05) * dt; this.vz *= 0.96;
    this.height = clamp(this.height + this.vz * dt, 0.05, 3.0);
    const vib = 0.02 * (Math.abs(fl) + Math.abs(rl) + Math.abs(fr) + Math.abs(rr));
    const n = () => (Math.random() * 2 - 1);
    const h = this.heading * Math.PI / 180;
    handle({ t: 'sen', t_us, imu_valid: true, acc: [0.01 + 0.03 * (fl + fr) + vib * n(), -0.02 + vib * n(), 0.93 + vib * n()],
      gyro: [0.3 * n(), -2.66 + 0.3 * n(), yawRate + 0.5 * n()], imu_temp_c: 31.5 + 0.1 * n(),
      mag_valid: true, mag: [0.24 * Math.cos(h), -0.24 * Math.sin(h), 0.31], heading_deg: this.heading,
      baro_valid: true, pressure_pa: 75000 - 12 * this.height + 2 * n(), baro_temp_c: 22.4, baro_alt_m: this.height - 1.5,
      tof_valid: true, tof_mm: Math.max(0, Math.round(this.height * 1000 + 8 * n())), tof_status: this.height < 2.4 ? 0 : 2 });
  },
  scene(t) { return { cx: 80 + 62 * Math.sin(t * 0.21), cy: 60 + 30 * Math.sin(t * 0.33 + 1), r: 9 + 11 * (0.5 + 0.5 * Math.sin(t * 0.17)) }; },
  camTick() {
    const t = (performance.now() - this.t0) / 1000;
    const s = this.scene(t);
    const visible = s.cx > -s.r + 4 && s.cx < 160 + s.r - 4 && s.cy > -s.r + 4 && s.cy < 120 + s.r - 4;
    const x0 = Math.max(0, s.cx - s.r), x1 = Math.min(159, s.cx + s.r), y0 = Math.max(0, s.cy - s.r), y1 = Math.min(119, s.cy + s.r);
    const row = { t_ms: Math.round(t * 1000), ok: visible ? 1 : 0, cx_px: s.cx, cy_px: s.cy,
      az_deg: (s.cx - 80) / 160 * FOV_DEG, el_deg: -(s.cy - 60) / 120 * FOV_DEG * 0.75,
      area_px: visible ? Math.round(Math.PI * s.r * s.r) : 0, w_px: x1 - x0, h_px: y1 - y0, fill: 0.78,
      range_mm: visible ? Math.round(44000 / Math.max(2 * s.r, 1)) : 0, clipped: visible && (x0 === 0 || y0 === 0 || x1 === 159 || y1 === 119) ? 1 : 0,
      blobs: visible ? 1 : 0, matched_px: visible ? Math.round(Math.PI * s.r * s.r * 0.9) : 0, dt_ms: 80, proc_us: 21500, stage: 1,
      conf: visible ? 0.93 : -1, b2_conf: -1, b3_conf: -1, b4_conf: -1, t_px_us: 9800, t_b_us: 1200, t_c_us: null, loop_us: 61000, csv_v: 2,
      b2_az: 0, b2_el: 0, b2_range_mm: 0, b3_az: 0, b3_el: 0, b3_range_mm: 0, b4_az: 0, b4_el: 0, b4_range_mm: 0 };
    if (this.camLive || this.camGrab > 0) {
      this.camGrab = Math.max(0, this.camGrab - 1); this.camSeq++;
      handle({ t: 'cam', seq: this.camSeq, t_ms: row.t_ms, w: 160, h: 120, png: null, scene: s, row });
    } else handle({ t: 'camrow', row });
  },
  console(line) {
    const out = {
      'help': 'help \n  Print the list of registered commands\n\nmotor [-m <id>] [-t <-1..1>] [-d <ms>] [--hold] [-s] [-b] [--sweep] [--status]\n  Drive the two DRV8833 motor drivers\n\nimu [-n <n>] [-i <ms>]\n  Read the MPU-6050 and print decoded accel/gyro/temperature\n\ntof [-n <n>] [-m <1|2>] [-b <ms>]\n  VL53L1X time-of-flight rangefinder\n\nwifi [--start] [--stop]\n  Status of the blimp\'s Wi-Fi access point\n\nlink \n  Status of the UDP command/telemetry link\n',
      'motor --status': 'PWM 19999 Hz, 11 bit (2048 steps), slow decay, deadband 12.0%\nfailsafe 1000 ms, ' + (this.motor.some((c) => c) ? 'armed' : 'idle - nothing is being driven') + '\nnSLEEP not on a GPIO - must be strapped high in hardware\n\nmotor  driver       GPIO         command       duty\n' + MOTORS.map((m, i) => `${m.padEnd(6)} ${['#1 IN1/IN2', '#1 IN3/IN4', '#2 IN1/IN2', '#2 IN3/IN4'][i].padEnd(12)} ${['13/12', '11/10', '9/8', '7/14'][i].padEnd(9)} ${sfmt(this.motor[i], 3).padStart(10)} ${(this.motor[i] ? (12 + 88 * Math.abs(this.motor[i])).toFixed(1) : '0.0').padStart(9)}%`).join('\n') + '\n',
      'wifi': 'access point UP\nSSID ICARUS-DEMO00   channel 1   WPA2   up to 2 station(s)\naddress 192.168.4.1   UDP link port 7200, console port 7201\n1 station(s) associated\n  a4:34:d9:1c:22:7e   rssi -41 dBm\n',
      'link': 'UDP port 7200   telemetry 20 Hz   timeout 500 ms\npeer 192.168.4.2:51234   alive, last packet 18 ms ago\nrx ok ' + this.rxOk + '   rx rejected 0   tx ' + this.rxOk + '\nmotors ' + (this.motorAt ? 'driven over the link' : 'not driven over the link') + '\n',
      'imu -n 3': 'MPU-6050 @ 0x68   accel +/-2g   gyro +/-250 dps\n    ax[g]     ay[g]     az[g]   gx[dps]   gy[dps]   gz[dps]     T[C]\n    0.012    -0.021     0.931     -0.15     -2.66      0.08     31.5\n    0.010    -0.019     0.933     -0.12     -2.70      0.05     31.5\n    0.011    -0.020     0.932     -0.18     -2.64      0.11     31.6\n',
      'tof -n 3': 'VL53L1X @ 0x29   long mode, budget 100 ms, inter 100 ms\n   #   dist[mm]   status        sigma   signal   ambient\n   1       1498   valid          2.1     1.42      0.31\n   2       1502   valid          2.0     1.40      0.31\n   3       1499   valid          2.2     1.41      0.30\n',
    };
    handle({ t: 'con', text: '> ' + line + '\n' });
    handle({ t: 'con', text: out[line] !== undefined ? out[line] : (line === 'motor -s' ? (this.motor = [0, 0, 0, 0], this.motorAt = 0, 'all motors coasting\n') : 'Unrecognized command\n') });
    handle({ t: 'con_end' });
  },
};

// ---------------------------------------------------------------- handle

function handle(m) {
  switch (m.t) {
    case 'hello': onHello(m); break;
    case 'tel': onTel(m); break;
    case 'sen': onSen(m); break;
    case 'cam': onCam(m); break;
    case 'camrow': onCamRow(m.row); break;
    case 'camstat': onCamStat(m); break;
    case 'con': conWrite(m.text); break;
    case 'con_end': conDone(); break;
    case 'ev': pushEvent(m); break;
  }
}

function onHello(m) {
  state.demo = !!m.demo;
  $('#pill-demo').hidden = !m.demo;
  $('#cam-note').textContent = m.camera && m.camera.source !== 'none'
    ? `source: ${m.camera.source}${m.camera.port ? ' · ' + m.camera.port : ''}` : 'no camera configured (--cam PORT or --cam demo)';
  (m.events || []).forEach(pushEvent);
  onCamStat(m.camera || {});
}

function onTel(m) {
  state.tel = m.tel; state.telAt = performance.now(); state.alive = !!m.alive;
  if (m.alive && m.tel) {
    setPill('#pill-link', m.tel.link_alive ? 'ok' : 'warn', (m.age_s !== null ? Math.round(m.age_s * 1000) : '—') + ' ms');
    const fs = m.tel.failsafe;
    setPill('#pill-fs', fs === 'TRIPPED' ? 'crit' : fs === 'armed' ? 'live' : 'ok', fs);
    setPill('#pill-sta', m.tel.stations > 0 ? 'ok' : 'warn', String(m.tel.stations));
  } else {
    setPill('#pill-link', 'crit', 'NO TELEMETRY');
    setPill('#pill-fs', 'idle', '—');
    setPill('#pill-sta', 'idle', '—');
  }
  renderTower();
}

function onSen(m) {
  state.sen = m; state.senAt = performance.now(); state.senEverSeen = true;
  const h = state.hist; const t = performance.now() / 1000;
  h.t.push(t);
  for (let i = 0; i < 3; i++) { h.acc[i].push(m.imu_valid ? m.acc[i] : NaN); h.gyro[i].push(m.imu_valid ? m.gyro[i] : NaN); }
  while (h.t.length && t - h.t[0] > HISTORY_S) { h.t.shift(); for (let i = 0; i < 3; i++) { h.acc[i].shift(); h.gyro[i].shift(); } }
  renderSensorsText();
}

function onCam(m) {
  state.cam.frame = m; state.cam.frameAt = performance.now();
  if (m.row) onCamRow(m.row);
  if (m.png) {
    const img = new Image();
    img.onload = () => { state.cam.image = img; drawCamera(); };
    img.src = m.png;
  } else drawCamera();
}

function onCamRow(row) {
  state.cam.row = row; state.cam.rowAt = performance.now(); state.cam.rowsSeen++;
  const rt = state.cam.rowTimes; rt.push(state.cam.rowAt); while (rt.length && state.cam.rowAt - rt[0] > 2000) rt.shift();
  renderCameraSide();
}

function onCamStat(st) {
  state.cam.status = st; state.cam.live = !!st.live;
  $('#btn-cam-live').setAttribute('aria-pressed', String(!!st.live));
  $('#btn-cam-connect').textContent = st.connected ? 'Disconnect' : 'Connect';
  if (!st.source || st.source === 'none') setPill('#pill-cam', 'idle', 'NONE');
  else setPill('#pill-cam', st.connected ? (st.live ? 'live' : 'ok') : 'warn', st.connected ? (st.live ? 'LIVE' : 'UP') : 'DOWN');
  renderCameraSide();
}

// ----------------------------------------------------------------- events

function addEvent(level, text) { pushEvent({ t: 'ev', ts: Date.now() / 1000, level, text }); }

function pushEvent(ev) {
  state.events.push(ev); if (state.events.length > 200) state.events.shift();
  const li = document.createElement('li'); li.className = ev.level;
  li.innerHTML = `<span class="ts">${hhmmss(ev.ts)}</span><span class="lv ${ev.level}">${ev.level}</span><span>${escapeHtml(ev.text)}</span>`;
  const list = $('#ev-list'); list.appendChild(li); while (list.children.length > 200) list.removeChild(list.firstChild);
  list.parentElement.scrollTop = list.parentElement.scrollHeight;
  const n = (lv) => state.events.filter((e) => e.level === lv).length;
  const nc = n('crit'), nw = n('warn');
  const badge = $('#ev-counts'); badge.textContent = `${nc} crit · ${nw} warn`;
  badge.className = 'badge mono' + (nc ? ' crit' : nw ? ' warn' : '');
  // the tab is not in view: mark it so the dot says there is something to read
  if (state.dockTab !== 'events') $('#tab-events').classList.add('new');
}

function escapeHtml(s) { return String(s).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c])); }

// ----------------------------------------------------------------- motors

function buildTower() {
  const tower = $('#tower');
  for (const m of MOTORS) {
    const row = document.createElement('div'); row.className = 'trow';
    row.innerHTML = `<span class="name">${m}</span>
      <input type="range" class="slider" id="sl-${m}" min="-100" max="100" step="1" value="0" disabled aria-label="${m} thrust, percent of the limit">
      <span class="val" id="val-${m}">+0.000</span>
      <div class="segbar" id="seg-${m}">${'<i></i>'.repeat(20)}</div>`;
    tower.appendChild(row);
    const sl = row.querySelector('input');
    sl.addEventListener('input', () => {
      const v = Number(sl.value) / 100;
      state.slider[m] = v;
      if (state.pairs) { state.slider[PAIR[m]] = v; $('#sl-' + PAIR[m]).value = sl.value; }
      sendMotors();
    });
    sl.addEventListener('dblclick', () => { sl.value = 0; sl.dispatchEvent(new Event('input')); });
  }
}

function motorCommand() { return MOTORS.map((m) => clamp(state.slider[m] * state.limit, -1, 1)); }

let motorTimer = null;
function sendMotors() {
  if (!state.enabled) return;
  link.send({ t: 'motor', cmd: motorCommand() });
}

function setEnabled(on) {
  state.enabled = on;
  $('#chk-enable').checked = on;
  MOTORS.forEach((m) => { $('#sl-' + m).disabled = !on; });
  if (on) { sendMotors(); motorTimer = setInterval(sendMotors, MOTOR_SEND_MS); addEvent('info', `motors enabled, limit ${Math.round(state.limit * 100)} %`); }
  else { clearInterval(motorTimer); motorTimer = null; }
}

function stopAll(brake = false) {
  link.send({ t: 'stop', brake });
  MOTORS.forEach((m) => { state.slider[m] = 0; $('#sl-' + m).value = 0; });
  if (state.enabled) setEnabled(false);
  const b = $('#btn-stop'); b.classList.add('flash'); setTimeout(() => b.classList.remove('flash'), 180);
}

function renderTower() {
  const tel = state.tel; const stale = !state.alive;
  for (const m of MOTORS) {
    const v = tel && !stale ? tel.motor[m] : NaN;
    $('#val-' + m).textContent = sfmt(v, 3);
    const segs = $('#seg-' + m).children; const on = Number.isNaN(v) ? 0 : Math.round(Math.abs(v) * 20);
    for (let i = 0; i < 20; i++) { segs[i].className = i < on ? ('on' + (v < 0 ? ' rev' : '')) : ''; }
    // top view: front arrows scale with thrust, rear rings grow with lift
    if (m === 'FL' || m === 'FR') {
      const a = $('#tv-' + m + '-arrow'); const len = Number.isNaN(v) ? 0 : v * 34;
      a.setAttribute('y2', String(-9 - len)); a.classList.toggle('rev', v < 0);
      a.style.visibility = Math.abs(len) < 2 ? 'hidden' : 'visible';
    } else {
      const r = $('#tv-' + m + '-ring'); const rr = Number.isNaN(v) ? 7 : 7 + Math.abs(v) * 16;
      r.setAttribute('r', String(rr)); r.style.visibility = rr < 8 ? 'hidden' : 'visible';
    }
  }
}

// ---------------------------------------------------------------- sensors

function renderSensorsText() {
  const s = state.sen; if (!s) return;
  if (s.imu_valid) {
    ['x', 'y', 'z'].forEach((a, i) => { $('#acc-' + a).textContent = sfmt(s.acc[i], 3); $('#gyro-' + a).textContent = sfmt(s.gyro[i], 1); });
    $('#imu-temp').textContent = fmt(s.imu_temp_c, 1);
    const [ax, ay, az] = s.acc;
    $('#att-roll').textContent = sfmt(Math.atan2(ay, az) * 180 / Math.PI, 1);
    $('#att-pitch').textContent = sfmt(Math.atan2(-ax, Math.hypot(ay, az)) * 180 / Math.PI, 1);
  }
  if (s.mag_valid) {
    ['x', 'y', 'z'].forEach((a, i) => { $('#mag-' + a).textContent = sfmt(s.mag[i], 3); });
    $('#needle').setAttribute('transform', `rotate(${s.heading_deg.toFixed(1)} 60 60)`);   // fixed dial, needle is the nose
    $('#hdg-text').textContent = fmt(s.heading_deg, 0) + '°';
  }
  if (s.baro_valid) {
    $('#baro-p').textContent = fmt(s.pressure_pa / 100, 1);
    $('#baro-t').textContent = fmt(s.baro_temp_c, 1);
    $('#baro-alt').textContent = sfmt(s.baro_alt_m, 2);
  }
  if (s.tof_valid) {
    const m = s.tof_mm / 1000; const ok = s.tof_status === 0;
    const y = 110 - clamp(m / GAUGE_MAX_M, 0, 1) * 100;
    $('#tof-fill').setAttribute('y', y.toFixed(1)); $('#tof-fill').setAttribute('height', (110 - y).toFixed(1));
    $('#tof-line').setAttribute('y1', y.toFixed(1)); $('#tof-line').setAttribute('y2', y.toFixed(1));
    $('#tof-text').textContent = ok ? fmt(m, 2) + ' m' : '— m';
    const names = { 0: 'valid', 1: 'sigma fail', 2: 'signal fail', 3: 'below min range', 4: 'out of bounds', 5: 'hardware fail', 6: 'no update', 7: 'wraparound', 255: 'no measurement' };
    $('#tof-status').textContent = `status ${s.tof_status}: ${names[s.tof_status] || 'other'}` + (m >= HOVER_BAND_M[0] && m <= HOVER_BAND_M[1] && ok ? ' · in the hover band' : '');
  }
}

class Sparkline {
  constructor(canvas, series, opts) {
    this.c = canvas; this.ctx = canvas.getContext('2d'); this.series = series; this.opts = opts;
    this.hover = null;
    canvas.addEventListener('mousemove', (e) => { const r = canvas.getBoundingClientRect(); this.hover = { x: (e.clientX - r.left) / r.width, cx: e.clientX, cy: e.clientY }; });
    canvas.addEventListener('mouseleave', () => { this.hover = null; $('#plot-tip').hidden = true; });
  }
  draw(t, data, unit) {
    const c = this.c, ctx = this.ctx;
    const w = c.clientWidth || 300, h = c.clientHeight || 90, dpr = window.devicePixelRatio || 1;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const padL = 36, padR = 8, padT = 6, padB = 14;
    const n = t.length; const now = performance.now() / 1000; const tmin = now - HISTORY_S;
    let lo = this.opts.min, hi = this.opts.max;
    if (lo === undefined) { lo = Infinity; hi = -Infinity; for (const s of data) for (const v of s) if (!Number.isNaN(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); } if (!(lo < hi)) { lo = -1; hi = 1; } const pad = (hi - lo) * 0.15 || 1; lo -= pad; hi += pad; if (this.opts.symmetric) { const m = Math.max(Math.abs(lo), Math.abs(hi)); lo = -m; hi = m; } }
    const X = (tt) => padL + (tt - tmin) / HISTORY_S * (w - padL - padR);
    const Y = (v) => padT + (1 - (v - lo) / (hi - lo)) * (h - padT - padB);
    ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1; ctx.fillStyle = css('--faint'); ctx.font = '10px ' + css('--font-mono');
    for (let k = 0; k <= 2; k++) { const v = lo + (hi - lo) * k / 2; const y = Y(v); ctx.beginPath(); ctx.moveTo(padL, y + 0.5); ctx.lineTo(w - padR, y + 0.5); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(v.toFixed(this.opts.digits), padL - 4, y + 3); }
    if (lo < 0 && hi > 0) { ctx.strokeStyle = css('--border'); ctx.beginPath(); ctx.moveTo(padL, Y(0) + 0.5); ctx.lineTo(w - padR, Y(0) + 0.5); ctx.stroke(); }
    ctx.textAlign = 'left'; ctx.fillText('−' + HISTORY_S + ' s', padL, h - 3); ctx.textAlign = 'right'; ctx.fillText('now', w - padR, h - 3);
    ctx.textAlign = 'left'; ctx.fillStyle = css('--muted'); ctx.fillText(unit, padL + 4, padT + 9);
    data.forEach((s, i) => {
      ctx.strokeStyle = css(this.series[i]); ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.beginPath(); let started = false;
      for (let k = 0; k < n; k++) { const v = s[k]; if (Number.isNaN(v)) { started = false; continue; } const x = X(t[k]), y = Y(v); if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y); }
      ctx.stroke();
      if (n) { const v = s[n - 1]; if (!Number.isNaN(v)) { ctx.fillStyle = css(this.series[i]); ctx.beginPath(); ctx.arc(X(t[n - 1]), Y(v), 3, 0, Math.PI * 2); ctx.fill(); } }
    });
    if (this.hover && n) {
      const tt = tmin + clamp((this.hover.x * w - padL) / (w - padL - padR), 0, 1) * HISTORY_S;
      let k = 0; for (let i = 0; i < n; i++) if (Math.abs(t[i] - tt) < Math.abs(t[k] - tt)) k = i;
      const x = X(t[k]); ctx.strokeStyle = css('--muted'); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x + 0.5, padT); ctx.lineTo(x + 0.5, h - padB); ctx.stroke();
      const tip = $('#plot-tip'); tip.hidden = false;
      tip.textContent = `${(t[k] - now).toFixed(1)} s   x ${sfmt(data[0][k], this.opts.digits)}  y ${sfmt(data[1][k], this.opts.digits)}  z ${sfmt(data[2][k], this.opts.digits)} ${unit}`;
      tip.style.left = (this.hover.cx + 14) + 'px'; tip.style.top = (this.hover.cy - 30) + 'px';
    }
  }
}

const plotAcc = new Sparkline($('#plot-acc'), SERIES, { symmetric: false, digits: 2 });
const plotGyro = new Sparkline($('#plot-gyro'), SERIES, { symmetric: true, digits: 1 });

function buildGauges() {
  const ticks = $('#compass-ticks');
  for (let a = 0; a < 360; a += 30) { const r1 = a % 90 === 0 ? 46 : 50, rad = a * Math.PI / 180; const l = document.createElementNS('http://www.w3.org/2000/svg', 'line'); l.setAttribute('x1', 60 + r1 * Math.sin(rad)); l.setAttribute('y1', 60 - r1 * Math.cos(rad)); l.setAttribute('x2', 60 + 54 * Math.sin(rad)); l.setAttribute('y2', 60 - 54 * Math.cos(rad)); ticks.appendChild(l); }
  const band = $('#tof-band'); const y1 = 110 - HOVER_BAND_M[1] / GAUGE_MAX_M * 100, y0 = 110 - HOVER_BAND_M[0] / GAUGE_MAX_M * 100;
  band.setAttribute('y', y1.toFixed(1)); band.setAttribute('height', (y0 - y1).toFixed(1));
  const tt = $('#tof-ticks');
  for (let m = 0; m <= GAUGE_MAX_M; m += 0.5) { const y = 110 - m / GAUGE_MAX_M * 100; const l = document.createElementNS('http://www.w3.org/2000/svg', 'line'); l.setAttribute('x1', 20); l.setAttribute('x2', 140); l.setAttribute('y1', y); l.setAttribute('y2', y); l.setAttribute('class', 'tick'); tt.appendChild(l); const tx = document.createElementNS('http://www.w3.org/2000/svg', 'text'); tx.setAttribute('x', 142); tx.setAttribute('y', y + 2.5); tx.setAttribute('class', 'tick-label'); tx.textContent = m.toFixed(1); tt.appendChild(tx); }
}

// ----------------------------------------------------------------- camera

const camCanvas = $('#cam-canvas'); const camCtx = camCanvas.getContext('2d');

function drawCamera() {
  const f = state.cam.frame; if (!f) return;
  $('#cam-empty').hidden = true;
  const ctx = camCtx; const W = camCanvas.width, H = camCanvas.height, sx = W / f.w, sy = H / f.h;
  ctx.save(); ctx.imageSmoothingEnabled = false;
  if (state.cam.rotate) { ctx.translate(W, H); ctx.rotate(Math.PI); }
  if (state.cam.image && f.png) ctx.drawImage(state.cam.image, 0, 0, W, H);
  else if (f.scene) {   // demo frame drawn directly
    const g = ctx.createLinearGradient(0, 0, W, H); g.addColorStop(0, '#8a8a86'); g.addColorStop(1, '#a9a7a2'); ctx.fillStyle = g; ctx.fillRect(0, 0, W, H);
    ctx.fillStyle = '#965f3c'; ctx.fillRect(0, 80 * sy, 45 * sx, H - 80 * sy);
    const rg = ctx.createRadialGradient((f.scene.cx - f.scene.r * 0.3) * sx, (f.scene.cy - f.scene.r * 0.3) * sy, 2, f.scene.cx * sx, f.scene.cy * sy, f.scene.r * sx); rg.addColorStop(0, '#e24a4a'); rg.addColorStop(1, '#8c1218');
    ctx.fillStyle = rg; ctx.beginPath(); ctx.arc(f.scene.cx * sx, f.scene.cy * sy, f.scene.r * sx, 0, Math.PI * 2); ctx.fill();
  }
  const row = f.row;
  if (row && row.ok) {
    const x = (row.cx_px - row.w_px / 2) * sx, y = (row.cy_px - row.h_px / 2) * sy;
    ctx.strokeStyle = css('--un-green'); ctx.lineWidth = 3; ctx.strokeRect(x, y, row.w_px * sx, row.h_px * sy);
    ctx.strokeStyle = css('--papaya'); ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(row.cx_px * sx - 10, row.cy_px * sy); ctx.lineTo(row.cx_px * sx + 10, row.cy_px * sy); ctx.moveTo(row.cx_px * sx, row.cy_px * sy - 10); ctx.lineTo(row.cx_px * sx, row.cy_px * sy + 10); ctx.stroke();
    ctx.fillStyle = css('--un-green'); ctx.font = 'bold 18px ' + css('--font-ui'); ctx.fillText(`${(row.range_mm / 1000).toFixed(2)} m`, x + 4, y - 6 < 16 ? y + row.h_px * sy + 20 : y - 6);
  }
  ctx.restore();
  ctx.strokeStyle = 'rgba(236,239,234,0.25)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(W / 2 + 0.5, H / 2 - 14); ctx.lineTo(W / 2 + 0.5, H / 2 + 14); ctx.moveTo(W / 2 - 14, H / 2 + 0.5); ctx.lineTo(W / 2 + 14, H / 2 + 0.5); ctx.stroke();
  $('#cam-stamp').textContent = `frame ${f.seq} · t ${(f.t_ms / 1000).toFixed(1)} s · ${f.w}×${f.h}${state.cam.rotate ? ' · rotated' : ''}`;
}

function renderCameraSide() {
  const row = state.cam.row; const fresh = row && performance.now() - state.cam.rowAt < 2000;
  const v = $('#verdict');
  if (!fresh) { v.className = 'verdict idle'; $('#verdict-word').textContent = 'NO DATA'; $('#verdict-sub').textContent = state.cam.status && state.cam.status.connected ? 'no rows from the camera for 2 s' : 'camera not connected'; }
  else if (row.ok) { v.className = 'verdict target'; $('#verdict-word').textContent = 'TARGET'; $('#verdict-sub').textContent = `${row.blobs} blob${row.blobs === 1 ? '' : 's'} · area ${row.area_px} px${row.clipped ? ' · clipped at the edge' : ''}`; }
  else { v.className = 'verdict none'; $('#verdict-word').textContent = 'NO TARGET'; $('#verdict-sub').textContent = `${row.blobs} blob${row.blobs === 1 ? '' : 's'} · stage ${row.stage}`; }
  $('#ro-az').textContent = fresh && row.ok ? sfmt(row.az_deg, 1) : '—';
  $('#ro-el').textContent = fresh && row.ok ? sfmt(row.el_deg, 1) : '—';
  $('#ro-range').textContent = fresh && row.ok ? fmt(row.range_mm / 1000, 2) : '—';
  $('#ro-conf').textContent = fresh && row.ok && row.conf >= 0 ? fmt(row.conf, 2) : '—';
  const tb = $('#blobs tbody'); tb.innerHTML = '';
  if (fresh) {
    const blobs = [];
    if (row.ok) blobs.push({ k: 1, az: row.az_deg, el: row.el_deg, r: row.range_mm, c: row.conf, main: true });
    for (let k = 2; k <= 4; k++) if (row.blobs >= k) blobs.push({ k, az: row['b' + k + '_az'], el: row['b' + k + '_el'], r: row['b' + k + '_range_mm'], c: row['b' + k + '_conf'] });
    for (const b of blobs) { const tr = document.createElement('tr'); if (b.main) tr.className = 'main'; tr.innerHTML = `<td>${b.k}</td><td>${sfmt(b.az, 1)}</td><td>${sfmt(b.el, 1)}</td><td>${fmt(b.r / 1000, 2)}</td><td>${b.c >= 0 ? fmt(b.c, 2) : '—'}</td>`; tb.appendChild(tr); }
    drawFov(blobs);
  } else drawFov([]);
  const st = state.cam.status || {}; const rps = state.cam.rowTimes.length / 2;
  $('#cam-health').textContent = st.source && st.source !== 'none'
    ? `link: ${st.connected ? 'up' : 'down'}${st.baud ? ' · ' + st.baud + ' baud' : ''} · ${rps.toFixed(1)} rows/s · frames ok ${st.frames_ok || 0}, bad ${st.frames_bad || 0}${st.junk ? ' · junk lines ' + st.junk : ''}`
    : 'link: no camera source';
}

function drawFov(blobs) {
  const c = $('#fov-canvas'), ctx = c.getContext('2d'); const W = c.width, H = c.height;
  ctx.clearRect(0, 0, W, H);
  const hx = FOV_DEG / 2, hy = FOV_DEG * 0.75 / 2;
  ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1;
  for (let a = -20; a <= 20; a += 10) { const x = W / 2 + a / hx * W / 2; ctx.beginPath(); ctx.moveTo(x + 0.5, 0); ctx.lineTo(x + 0.5, H); ctx.stroke(); }
  for (let e = -20; e <= 20; e += 10) { const y = H / 2 - e / hy * H / 2; ctx.beginPath(); ctx.moveTo(0, y + 0.5); ctx.lineTo(W, y + 0.5); ctx.stroke(); }
  ctx.strokeStyle = css('--border'); ctx.beginPath(); ctx.moveTo(W / 2 + 0.5, 0); ctx.lineTo(W / 2 + 0.5, H); ctx.moveTo(0, H / 2 + 0.5); ctx.lineTo(W, H / 2 + 0.5); ctx.stroke();
  ctx.fillStyle = css('--faint'); ctx.font = '10px ' + css('--font-mono'); ctx.fillText(`field of view ±${hx}° × ±${hy}°`, 6, H - 6);
  for (const b of blobs) {
    const x = W / 2 + b.az / hx * W / 2, y = H / 2 - b.el / hy * H / 2;
    const r = clamp(12000 / Math.max(b.r, 300), 3, 22);
    ctx.fillStyle = b.main ? css('--un-green') : css('--muted'); ctx.globalAlpha = 0.85; ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill(); ctx.globalAlpha = 1;
    ctx.fillStyle = css('--fg'); ctx.fillText(String(b.k), x + r + 3, y + 3);
  }
}

// ---------------------------------------------------------------- console

let conBusy = false;
function conWrite(text) { const o = $('#con-out'); o.textContent += text; o.scrollTop = o.scrollHeight; }
function conDone() { conBusy = false; $('#con-send').disabled = false; }
function conSend(line) {
  line = line.trim(); if (!line) return;
  if (conBusy) { conWrite('[a command is still running on the blimp]\n'); return; }
  conBusy = true; $('#con-send').disabled = true;
  state.conHistory.push(line); if (state.conHistory.length > 50) state.conHistory.shift(); state.conIndex = -1;
  link.send({ t: 'con', line });
  setTimeout(() => { if (conBusy) { conBusy = false; $('#con-send').disabled = false; } }, 320000);
}

const QUICK = ['help', 'motor --status', 'wifi', 'link', 'imu -n 3', 'tof -n 3', 'mag -n 3', 'baro', 'motor -s'];

// ------------------------------------------------------------------- dock
// The console and the event log share one strip under the panels: two tabs,
// a handle on the top edge that drags the height, a button that collapses it
// to the tab bar. Height, tab and collapsed state persist in localStorage,
// which is a per-browser convenience: every read and write is guarded, and
// the page works the same when storage is empty or unavailable.

const DOCK_MIN_PX = 110;
// First-run height: about a quarter of the window, 230 px on a 1080p screen,
// less on a laptop with the browser chrome showing.
const dockDefaultPx = () => clamp(Math.round(window.innerHeight * 0.23), 140, 230);

function wireDock() {
  const dock = $('#dock'), handle = $('#dock-handle'), toggle = $('#dock-toggle');
  const store = (k, v) => { try { localStorage.setItem('icarus.dock.' + k, String(v)); } catch (e) { /* storage unavailable */ } };
  const load = (k) => { try { return localStorage.getItem('icarus.dock.' + k); } catch (e) { return null; } };
  // leave the panels above at least ~320 px, whatever the window
  const maxPx = () => Math.max(DOCK_MIN_PX, window.innerHeight - 320);
  const setHeight = (px) => { px = clamp(Math.round(px), DOCK_MIN_PX, maxPx()); dock.style.setProperty('--dock-h', px + 'px'); return px; };
  let height = setHeight(Number(load('h')) || dockDefaultPx());

  const setCollapsed = (c) => {
    dock.classList.toggle('collapsed', c);
    toggle.setAttribute('aria-expanded', String(!c));
    toggle.innerHTML = c ? '&#9652;' : '&#9662;';
    toggle.title = c ? 'expand the console and events' : 'collapse to the tab bar';
    store('collapsed', c ? 1 : 0);
  };
  setCollapsed(load('collapsed') === '1');
  toggle.addEventListener('click', () => setCollapsed(!dock.classList.contains('collapsed')));

  const tabs = [...dock.querySelectorAll('.tab')];
  const show = (name) => {
    tabs.forEach((t) => { const on = t.dataset.tab === name; t.setAttribute('aria-selected', String(on)); $('#pane-' + t.dataset.tab).hidden = !on; });
    $('#note-console').hidden = name !== 'console'; $('#note-events').hidden = name !== 'events';
    state.dockTab = name; store('tab', name);
    if (name === 'events') { $('#tab-events').classList.remove('new'); const p = $('#pane-events'); p.scrollTop = p.scrollHeight; }
    else $('#con-in').focus({ preventScroll: true });
  };
  tabs.forEach((t) => t.addEventListener('click', () => { if (dock.classList.contains('collapsed')) setCollapsed(false); show(t.dataset.tab); }));
  show(load('tab') === 'events' ? 'events' : 'console');

  let drag = null;
  handle.addEventListener('pointerdown', (e) => {
    if (dock.classList.contains('collapsed')) return;
    drag = { y: e.clientY, h: dock.getBoundingClientRect().height };
    handle.setPointerCapture(e.pointerId); document.body.classList.add('resizing'); e.preventDefault();
  });
  handle.addEventListener('pointermove', (e) => { if (drag) height = setHeight(drag.h + (drag.y - e.clientY)); });
  const end = () => { if (!drag) return; drag = null; document.body.classList.remove('resizing'); store('h', height); };
  handle.addEventListener('pointerup', end); handle.addEventListener('pointercancel', end);
  handle.addEventListener('dblclick', () => { height = setHeight(dockDefaultPx()); store('h', height); });
  window.addEventListener('resize', () => { height = setHeight(height); });
}

// ----------------------------------------------------------------- wiring

function wire() {
  buildTower(); buildGauges(); wireDock();
  const chips = $('#chips');
  for (const q of QUICK) { const b = document.createElement('button'); b.type = 'button'; b.className = 'chip'; b.textContent = q; b.addEventListener('click', () => conSend(q)); chips.appendChild(b); }
  $('#con-form').addEventListener('submit', (e) => { e.preventDefault(); const i = $('#con-in'); conSend(i.value); i.value = ''; });
  $('#con-in').addEventListener('keydown', (e) => {
    const h = state.conHistory; if (!h.length) return;
    if (e.key === 'ArrowUp') { e.preventDefault(); state.conIndex = state.conIndex < 0 ? h.length - 1 : Math.max(0, state.conIndex - 1); e.target.value = h[state.conIndex]; }
    if (e.key === 'ArrowDown') { e.preventDefault(); if (state.conIndex < 0) return; state.conIndex = Math.min(h.length, state.conIndex + 1); e.target.value = state.conIndex === h.length ? '' : h[state.conIndex]; if (state.conIndex === h.length) state.conIndex = -1; }
  });
  $('#btn-stop').addEventListener('click', () => stopAll(false));
  $('#chk-enable').addEventListener('change', (e) => { if (!e.target.checked) { stopAll(false); } else setEnabled(true); });
  $('#in-limit').addEventListener('change', (e) => { state.limit = clamp(Number(e.target.value) / 100, 0.05, 1); e.target.value = Math.round(state.limit * 100); addEvent(state.limit > 0.5 ? 'warn' : 'info', `thrust limit ${Math.round(state.limit * 100)} %`); sendMotors(); });
  $('#chk-pairs').addEventListener('change', (e) => { state.pairs = e.target.checked; });
  $('#btn-zero').addEventListener('click', () => { MOTORS.forEach((m) => { state.slider[m] = 0; $('#sl-' + m).value = 0; }); sendMotors(); });
  $('#btn-cam-connect').addEventListener('click', () => link.send({ t: 'cam', action: state.cam.status && state.cam.status.connected ? 'disconnect' : 'connect' }));
  $('#btn-cam-grab').addEventListener('click', () => link.send({ t: 'cam', action: 'grab' }));
  $('#btn-cam-live').addEventListener('click', () => link.send({ t: 'cam', action: 'live', on: !state.cam.live }));
  $('#btn-cam-rotate').addEventListener('click', (e) => { state.cam.rotate = !state.cam.rotate; e.currentTarget.setAttribute('aria-pressed', String(state.cam.rotate)); drawCamera(); });
  $('#btn-cam-baud').addEventListener('click', () => link.send({ t: 'cam', action: 'baud', baud: Number($('#sel-baud').value) }));
  document.addEventListener('keydown', (e) => {
    const typing = /^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName) && e.target.type !== 'range' && e.target.type !== 'checkbox';
    if (e.key === 'Escape' || (e.key === ' ' && !typing)) { e.preventDefault(); stopAll(false); }
  });
  window.addEventListener('beforeunload', () => { if (state.enabled) link.send({ t: 'stop', brake: false }); });
  // animation: plots, staleness, camera verdict ageing
  function frame() {
    const h = state.hist;
    plotAcc.draw(h.t, h.acc, 'g'); plotGyro.draw(h.t, h.gyro, '°/s');
    const senStale = state.senAt && performance.now() - state.senAt > 1500;
    $('#sen-missing').hidden = !(state.alive && !state.demo && !state.senEverSeen);
    $('#sen-stale').hidden = !(state.alive && state.senEverSeen && senStale);
    if (state.telAt && performance.now() - state.telAt > 1500 && state.alive) { state.alive = false; renderTower(); setPill('#pill-link', 'crit', 'STALE'); }
    if (state.cam.row && performance.now() - state.cam.rowAt > 2000 && performance.now() - state.cam.rowAt < 2300) renderCameraSide();
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
  link.connect();
}

if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', wire); else wire();
