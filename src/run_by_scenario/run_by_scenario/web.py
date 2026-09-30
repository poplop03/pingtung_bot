"""HTTP server for the run-by-scenario web UI: WASD or joystick teleop, PWM, record and play.

    GET  /              the page
    GET  /api/state     controller state as JSON (plus the saved scenario names)
    POST /api/cmd       {"cmd": ..., ...} -> {"ok": bool, "message": str}

The HTTP threads never touch the controller. A command is handed to the ROS
thread through `submit`, which returns its (ok, message) result.
"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Tuple

INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Run by Scenario</title>
<style>
  :root { --bg:#111; --panel:#1c1c1c; --line:#333; --fg:#ddd; --dim:#888;
          --go:#2e7d32; --stop:#c62828; --acc:#1565c0; --rec:#d84315; }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font-family: system-ui, sans-serif; font-size:14px; }
  main { max-width:560px; margin:auto; padding:12px 16px; }
  section { background:var(--panel); border:1px solid var(--line);
            border-radius:6px; padding:10px 12px; margin-bottom:12px; }
  h2 { font-size:13px; text-transform:uppercase; letter-spacing:.05em;
       color:var(--dim); margin:0 0 8px; }
  button { background:#2a2a2a; color:var(--fg); border:1px solid #444;
           border-radius:5px; padding:8px 10px; font-size:14px; cursor:pointer;
           touch-action:none; user-select:none; }
  button:active, button.held { background:var(--acc); }
  button:disabled { opacity:.35; cursor:default; }
  .big { font-size:20px; font-weight:600; padding:14px; flex:1; }
  #stop { background:var(--stop); }
  #record { background:var(--rec); }
  #start { background:var(--go); }
  .row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
  .seg { display:flex; margin-bottom:10px; }
  .seg button { flex:1; border-radius:0; }
  .seg button:first-child { border-radius:5px 0 0 5px; }
  .seg button:last-child { border-radius:0 5px 5px 0; }
  .seg button.on { background:var(--acc); border-color:var(--acc); }
  .pad { display:grid; grid-template-columns: repeat(3, 64px); gap:6px; }
  .pad button { height:52px; padding:0; }
  #joy { position:relative; width:220px; height:220px; border-radius:50%;
         margin:4px auto; background:#161616; border:2px solid #444;
         touch-action:none; user-select:none; -webkit-user-select:none; cursor:grab; }
  #joy::before, #joy::after { content:''; position:absolute; background:#2c2c2c; }
  #joy::before { left:50%; top:8%; bottom:8%; width:1px; }
  #joy::after { top:50%; left:8%; right:8%; height:1px; }
  #knob { position:absolute; width:72px; height:72px; left:74px; top:74px;
          border-radius:50%; background:#3a3a3a; border:2px solid #666;
          pointer-events:none; z-index:1; }
  #joy.held #knob { background:var(--acc); border-color:var(--acc); }
  .kv { display:grid; grid-template-columns: auto 1fr; gap:3px 12px; }
  .kv span:nth-child(odd) { color:var(--dim); }
  .num { font-variant-numeric: tabular-nums; }
  .dot { display:inline-block; width:9px; height:9px; border-radius:50%;
         margin-right:4px; background:var(--stop); }
  .dot.ok { background:var(--go); }
  #msg { margin-top:8px; padding:6px 8px; border-radius:4px; background:#222;
         min-height:1.6em; }
  #msg.err { background:#4a1515; }
  select, input[type=number], input[type=text] { background:#222; color:var(--fg);
         border:1px solid #444; border-radius:4px; padding:7px; font-size:14px; }
  input[type=text], select { flex:1; min-width:0; }
  input[type=range] { flex:1; min-width:0; }
  .pwmrow { display:grid; grid-template-columns: 64px 1fr 70px; gap:8px;
            align-items:center; margin-bottom:6px; }
  progress { width:100%; height:10px; }
  .hint { color:var(--dim); font-size:12px; margin-top:6px; }
  .locked { opacity:.45; pointer-events:none; }
  .hidden { display:none; }
  #mode.recording { color:var(--rec); font-weight:600; }
  #mode.playing { color:#66bb6a; font-weight:600; }
</style>
</head>
<body>
<main>
  <section>
    <div class="row"><button id="stop" class="big">STOP</button></div>
    <div id="msg"></div>
    <div class="kv" style="margin-top:8px">
      <span>mode</span><span id="mode">-</span>
      <span>time</span><span id="elapsed" class="num">-</span>
      <span>command</span><span id="command" class="num">-</span>
      <span>wheel PWM</span><span id="wheelpwm" class="num">-</span>
      <span>health</span><span><span id="mega" class="dot"></span>mega
        <span id="failsafe"></span></span>
    </div>
    <progress id="progress" value="0" max="1" style="margin-top:8px"></progress>
  </section>

  <section class="manual">
    <h2>PWM</h2>
    <div class="pwmrow"><span>linear</span>
      <input id="lin" type="range" min="0" max="255" step="1" value="0">
      <input id="linn" type="number" min="0" max="255" step="1" value="0"></div>
    <div class="pwmrow"><span>angular</span>
      <input id="ang" type="range" min="0" max="255" step="1" value="0">
      <input id="angn" type="number" min="0" max="255" step="1" value="0"></div>
    <div class="row">
      <button id="pwmset">Set</button>
      <span class="hint" style="margin:0">applied: <span id="pwmcur" class="num">-</span></span>
    </div>
    <div class="hint">Linear: base PWM at full forward/back. Angular: turn PWM at a
      full left/right (wheel_control's feedforward; its yaw-rate loop still holds the
      turn). Also used while recording; playback uses the recorded PWM.</div>
  </section>

  <section class="manual">
    <h2>Teleop</h2>
    <div class="seg">
      <button id="m_wasd" data-mode="wasd">W A S D</button>
      <button id="m_joystick" data-mode="joystick">Joystick</button>
    </div>
    <div id="wasd">
      <div class="pad">
        <span></span><button data-drive="1,0">W &uarr;</button><span></span>
        <button data-drive="0,1">A &larr;</button>
        <button data-drive="-1,0">S &darr;</button>
        <button data-drive="0,-1">D &rarr;</button>
      </div>
      <div class="hint">Hold W/A/S/D (keys or buttons); W+A etc. combine. Space = STOP</div>
    </div>
    <div id="joystick" class="hidden">
      <div id="joy"><div id="knob"></div></div>
      <div class="hint" style="text-align:center">Drag: up/down = linear,
        left/right = turn, proportional. Let go = stop. Space = STOP</div>
      <div class="hint" style="text-align:center">Gamepad: <span id="pad">none - plug one
        into this computer/phone and press a button</span></div>
    </div>
  </section>

  <section>
    <h2>Record</h2>
    <div class="row">
      <input id="recname" type="text" placeholder="scenario name" maxlength="64">
      <button id="record">&#9679; Record</button>
    </div>
    <div class="hint">Record, drive, then STOP: the scenario is saved and loaded.
      Idle time before the first command and after the last one is cut.
      You can change the PWM while recording.</div>
  </section>

  <section>
    <h2>Play</h2>
    <div class="row">
      <select id="list"></select>
      <button id="load">Load</button>
      <button id="delete">Delete</button>
    </div>
    <div class="kv" style="margin-top:8px">
      <span>loaded</span><span id="loaded">none</span>
    </div>
    <div class="row" style="margin-top:8px">
      <button id="start" class="big">&#9654; START</button>
    </div>
  </section>
</main>
<script>
const $ = (id) => document.getElementById(id);
let state = null, pwmLoaded = false, listKey = '';

async function cmd(body) {
  try {
    const r = await fetch('/api/cmd', {method: 'POST',
      headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    const res = await r.json();
    if (res.message) showMsg(res.message, !res.ok);
    poll();
    return res;
  } catch (e) { showMsg('connection lost', true); }
}
function showMsg(text, err) {
  $('msg').textContent = text; $('msg').className = err ? 'err' : '';
}

async function poll() {
  try { state = await (await fetch('/api/state')).json(); }
  catch (e) { state = null; }
  render();
}
function render() {
  const s = state;
  if (!s) { $('mode').textContent = 'no connection'; return; }
  $('mode').textContent = s.mode + (s.recording ? ' "' + s.recording + '"' : '');
  $('mode').className = s.mode;
  $('elapsed').textContent = s.elapsed === null ? '-' : s.elapsed.toFixed(1) + ' s'
    + (s.mode === 'playing' && s.loaded ? ' / ' + s.loaded.duration.toFixed(1) + ' s' : '');
  $('command').textContent = 'linear ' + s.command.linear.toFixed(0)
    + ', angular ' + s.command.angular.toFixed(0);
  $('wheelpwm').textContent = s.wheel_pwm ? s.wheel_pwm.join(', ') : '-';
  $('mega').className = 'dot' + (s.mega_ok ? ' ok' : '');
  $('failsafe').textContent = s.failsafe ? ' WHEEL FAILSAFE' : '';
  $('msg').textContent = s.message; $('msg').className = s.error ? 'err' : '';
  $('progress').value = (s.mode === 'playing' && s.loaded)
    ? s.elapsed / Math.max(s.loaded.duration, 1e-3) : 0;
  $('loaded').textContent = s.loaded
    ? s.loaded.name + ' (' + s.loaded.duration.toFixed(1) + ' s, ' + s.loaded.steps + ' steps)'
    : 'none';

  const idle = s.mode === 'idle';
  document.querySelectorAll('.manual').forEach(
    (el) => el.classList.toggle('locked', s.mode === 'playing'));
  $('record').disabled = !idle;
  $('start').disabled = !idle || !s.loaded;
  $('load').disabled = $('delete').disabled = !idle || !s.scenarios.length;

  showControl(s.control_mode);
  const key = s.scenarios.join('\n');
  if (key !== listKey) {         // rebuild only on change, keep the selection
    listKey = key;
    const keep = $('list').value || (s.loaded && s.loaded.name);
    $('list').innerHTML = '';
    s.scenarios.forEach((n) => $('list').add(new Option(n, n, false, n === keep)));
  }
  $('pwmcur').textContent = 'linear ' + s.linear_pwm.toFixed(0)
    + ', angular ' + s.angular_pwm.toFixed(0);
  $('lin').max = $('linn').max = s.max_linear_pwm;
  $('ang').max = $('angn').max = s.max_angular_pwm;
  if (!pwmLoaded) {              // fill the pickers once; after that they are the user's
    pwmLoaded = true; showPwm(s.linear_pwm, s.angular_pwm);
  }
}
function showPwm(l, a) {
  $('lin').value = l; $('linn').value = Math.round(l);
  $('ang').value = a; $('angn').value = Math.round(a);
}
setInterval(poll, 250); poll();

$('stop').onclick = () => { stopDrive(); cmd({cmd: 'stop'}); };

// ---- PWM ----
[['lin', 'linn'], ['ang', 'angn']].forEach(([r, n]) => {
  $(r).oninput = () => { $(n).value = $(r).value; };
  $(r).onchange = () => $(r).blur();       // a focused slider eats key presses
  $(n).oninput = () => { $(r).value = $(n).value; };
  $(n).onkeydown = (e) => { if (e.key === 'Enter') $('pwmset').click(); };
});
$('pwmset').onclick = () => {
  const clamp = (v, m) => Math.max(0, Math.min(m, +v || 0));
  const l = clamp($('linn').value, state ? state.max_linear_pwm : 255);
  const a = clamp($('angn').value, state ? state.max_angular_pwm : 255);
  showPwm(l, a);
  document.activeElement.blur();           // give W/A/S/D back to the page
  cmd({cmd: 'pwm', linear: l, angular: a});
};

// ---- control mode ----
let control = null;                        // set from the server's control_mode
function showControl(mode) {
  if (mode === control) return;
  control = mode; stopDrive(); held.clear();
  $('m_wasd').classList.toggle('on', mode === 'wasd');
  $('m_joystick').classList.toggle('on', mode === 'joystick');
  $('wasd').classList.toggle('hidden', mode !== 'wasd');
  $('joystick').classList.toggle('hidden', mode !== 'joystick');
}
document.querySelectorAll('.seg button').forEach((b) => b.onclick = () => {
  showControl(b.dataset.mode); cmd({cmd: 'control', mode: b.dataset.mode});
});

// ---- record / play ----
function defaultName() {
  const d = new Date(), p = (n) => String(n).padStart(2, '0');
  return 'scenario_' + d.getFullYear() + p(d.getMonth() + 1) + p(d.getDate())
    + '_' + p(d.getHours()) + p(d.getMinutes()) + p(d.getSeconds());
}
$('recname').onkeydown = (e) => { if (e.key === 'Enter') $('record').click(); };
$('record').onclick = () => {
  const name = $('recname').value.trim() || defaultName();
  $('recname').value = name;
  document.activeElement.blur();
  let overwrite = false;
  if (state && state.scenarios.includes(name)) {
    if (!confirm('"' + name + '" exists. Overwrite it when the recording is saved?')) return;
    overwrite = true;
  }
  cmd({cmd: 'record', name, overwrite});
};
$('load').onclick = () => cmd({cmd: 'load', name: $('list').value});
$('delete').onclick = () => {
  const name = $('list').value;
  if (name && confirm('Delete scenario "' + name + '"?')) cmd({cmd: 'delete', name});
};
$('start').onclick = () => cmd({cmd: 'start'});

// ---- teleop: resend the command every 100 ms while held (server deadman 0.3 s) ----
let drive = null, driveTimer = null;
function sendDrive() {
  if (drive) fetch('/api/cmd', {method: 'POST', body: JSON.stringify(
    {cmd: 'drive', linear: drive[0], angular: drive[1]})}).catch(() => {});
}
function startDrive(v) {
  const changed = !drive || drive[0] !== v[0] || drive[1] !== v[1];
  drive = v;
  if (changed) sendDrive();
  if (!driveTimer) driveTimer = setInterval(sendDrive, 100);
}
function stopDrive() {
  if (!drive) return;
  drive = null; clearInterval(driveTimer); driveTimer = null;
  document.querySelectorAll('.held').forEach((b) => b.classList.remove('held'));
  centerKnob();
  fetch('/api/cmd', {method: 'POST', body: JSON.stringify(
    {cmd: 'drive', linear: 0, angular: 0})}).catch(() => {});
}
window.addEventListener('blur', stopDrive);
document.addEventListener('visibilitychange', stopDrive);

// WASD: buttons and keys
document.querySelectorAll('[data-drive]').forEach((b) => {
  const v = b.dataset.drive.split(',').map(Number);
  b.addEventListener('pointerdown', (e) => {
    b.setPointerCapture(e.pointerId); b.classList.add('held'); startDrive(v);
  });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(
    (ev) => b.addEventListener(ev, stopDrive));
});
const DRIVE_KEYS = {w: [1, 0], s: [-1, 0], a: [0, 1], d: [0, -1]};
const held = new Set();
function keyDrive() {
  let lin = 0, ang = 0;
  held.forEach((k) => { lin += DRIVE_KEYS[k][0]; ang += DRIVE_KEYS[k][1]; });
  if (lin || ang) startDrive([lin, ang]); else stopDrive();
}
document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (k === ' ') { e.preventDefault(); $('stop').click(); return; }
  if (e.repeat || control !== 'wasd') return;
  if (k in DRIVE_KEYS) { held.add(k); keyDrive(); }
});
document.addEventListener('keyup', (e) => {
  const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (held.delete(k)) keyDrive();
});

// Joystick: drag the knob; up = forward, left = turn left, proportional
const joy = $('joy'), knob = $('knob');
const DEAD = 0.08;                          // ignore tiny offsets around the center
function centerKnob() { knob.style.transform = ''; joy.classList.remove('held'); }
function joyMove(e) {
  const r = joy.getBoundingClientRect();
  const R = r.width / 2 - knob.offsetWidth / 2;          // knob travel radius
  let dx = e.clientX - (r.left + r.width / 2), dy = e.clientY - (r.top + r.height / 2);
  const d = Math.hypot(dx, dy);
  if (d > R) { dx *= R / d; dy *= R / d; }
  knob.style.transform = 'translate(' + dx + 'px,' + dy + 'px)';
  const q = (v) => Math.abs(v) < DEAD ? 0 : Math.round(v * 100) / 100;
  const lin = q(-dy / R), ang = q(-dx / R);
  if (lin || ang) startDrive([lin, ang]);
  else if (drive) { drive = [0, 0]; sendDrive(); }       // held at center: stay alive at 0
}
joy.addEventListener('pointerdown', (e) => {
  joy.setPointerCapture(e.pointerId); joy.classList.add('held'); joyMove(e);
});
joy.addEventListener('pointermove', (e) => { if (joy.classList.contains('held')) joyMove(e); });
['pointerup', 'pointercancel', 'lostpointercapture'].forEach((ev) =>
  joy.addEventListener(ev, () => { centerKnob(); stopDrive(); }));
joy.addEventListener('contextmenu', (e) => e.preventDefault());

// Physical gamepad (USB/Bluetooth on the device running this page), joystick mode only.
// Left stick: up/down = linear, left/right = turn. B / circle (button 1) = STOP.
let padActive = false, padStop = false;
function knobAt(lin, ang) {
  const R = joy.offsetWidth / 2 - knob.offsetWidth / 2;
  knob.style.transform = 'translate(' + (-ang * R) + 'px,' + (-lin * R) + 'px)';
}
function pollPad() {
  const gp = [...(navigator.getGamepads ? navigator.getGamepads() : [])].find((g) => g);
  $('pad').textContent = gp ? gp.id.slice(0, 40) : 'none - plug one in and press a button';
  if (!gp) { if (padActive) { padActive = false; stopDrive(); } return; }
  const stopBtn = gp.buttons[1] && gp.buttons[1].pressed;
  if (stopBtn && !padStop) $('stop').click();
  padStop = stopBtn;
  if (control !== 'joystick' || joy.classList.contains('held')) return;  // finger/mouse wins
  const q = (v) => Math.abs(v) < 0.12 ? 0 : Math.round(v * 100) / 100;
  const lin = q(-gp.axes[1] || 0), ang = q(-gp.axes[0] || 0);
  if (lin || ang) { padActive = true; knobAt(lin, ang); startDrive([lin, ang]); }
  else if (padActive) { padActive = false; stopDrive(); }
}
setInterval(pollPad, 50);
</script>
</body>
</html>
"""


def make_handler(get_state: Callable[[], dict],
                 submit: Callable[[dict], Tuple[bool, str]]):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 (http.server API)
            if self.path in ('/', '/index.html'):
                self._send(200, 'text/html; charset=utf-8', INDEX_HTML.encode())
            elif self.path == '/api/state':
                self._json(200, get_state())
            else:
                self._send(404, 'text/plain', b'not found')

        def do_POST(self) -> None:  # noqa: N802
            if self.path != '/api/cmd':
                self._send(404, 'text/plain', b'not found')
                return
            try:
                length = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(body, dict):
                    raise ValueError('body must be a JSON object')
            except ValueError as exc:
                self._json(400, {'ok': False, 'message': f'bad request: {exc}'})
                return
            ok, message = submit(body)
            self._json(200, {'ok': ok, 'message': message})

        def _json(self, status: int, data: dict) -> None:
            self._send(status, 'application/json', json.dumps(data).encode())

        def _send(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format, *args) -> None:  # noqa: A002
            pass

    return Handler


class WebServer:
    def __init__(self, host: str, port: int,
                 get_state: Callable[[], dict],
                 submit: Callable[[dict], Tuple[bool, str]]) -> None:
        self._server = ThreadingHTTPServer((host, port), make_handler(get_state, submit))
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._server.shutdown()
        self._server.server_close()
