"""HTTP server for the pick-fruit web UI: camera stream, state, and commands.

    GET  /              the page
    GET  /stream.mjpg   annotated camera view (MJPEG)
    GET  /snapshot.jpg  latest frame
    GET  /api/state     controller state as JSON
    POST /api/cmd       {"cmd": ..., ...} -> {"ok": bool, "message": str}

The HTTP threads never touch the controller. A command is handed to the ROS
thread through `submit`, which returns its (ok, message) result.
"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional, Tuple

INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pick Fruit</title>
<style>
  :root { --bg:#111; --panel:#1c1c1c; --line:#333; --fg:#ddd; --dim:#888;
          --go:#2e7d32; --stop:#c62828; --acc:#1565c0; --warn:#f9a825; }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font-family: system-ui, sans-serif; font-size:14px; }
  main { display:grid; grid-template-columns: minmax(0,1.5fr) minmax(300px,1fr);
         gap:12px; padding:12px; max-width:1400px; margin:auto; }
  @media (max-width: 860px) { main { grid-template-columns: 1fr; padding:8px 16px; } }
  .cam img { width:100%; height:auto; display:block; background:#000;
             border:1px solid var(--line); border-radius:6px; }
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
  #start { background:var(--go); } #stop { background:var(--stop); }
  .row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
  .pad { display:grid; grid-template-columns: repeat(3, 56px); gap:6px; }
  .pad button { height:48px; padding:0; }
  .kv { display:grid; grid-template-columns: auto 1fr; gap:3px 12px; }
  .kv span:nth-child(odd) { color:var(--dim); }
  .dot { display:inline-block; width:9px; height:9px; border-radius:50%;
         margin-right:4px; background:var(--stop); }
  .dot.ok { background:var(--go); }
  #msg { margin-top:8px; padding:6px 8px; border-radius:4px; background:#222;
         min-height:1.6em; }
  #msg.err { background:#4a1515; }
  table { width:100%; border-collapse:collapse; }
  td { padding:4px 2px; }
  td.num { font-variant-numeric: tabular-nums; }
  select, input[type=number] { background:#222; color:var(--fg);
         border:1px solid #444; border-radius:4px; padding:6px; }
  input[type=range] { flex:1; }
  .hint { color:var(--dim); font-size:12px; margin-top:6px; }
  .manual-only.locked { opacity:.45; pointer-events:none; }
</style>
</head>
<body>
<main>
  <div class="cam">
    <img src="/stream.mjpg" alt="waiting for camera...">
    <div class="hint">Keys: W/A/S/D drive &middot; arrows jog gantry
      &middot; O/C gripper open/close &middot; Space = STOP</div>
  </div>
  <div>
    <section>
      <div class="row">
        <button id="start" class="big">START</button>
        <button id="stop" class="big">STOP</button>
      </div>
      <div id="msg"></div>
      <div class="kv" style="margin-top:8px">
        <span>mode</span><span id="mode">-</span>
        <span>gantry</span><span id="target">-</span>
        <span>driving PWM</span><span id="drivepwm" class="num">-</span>
        <span>fruits hit</span><span id="hits">0</span>
        <span>health</span><span><span id="mega" class="dot"></span>mega
          &nbsp;<span id="vision" class="dot"></span>vision
          &nbsp;<span id="hit" class="dot"></span>line hit
          <span id="failsafe"></span></span>
        <span>wheel PWM</span><span id="wheelpwm" class="num">-</span>
      </div>
    </section>

    <section>
      <h2>Drive PWM (W/S and autonomous)</h2>
      <div class="row">
        <input id="speed" type="range" min="0" max="255" step="1" value="0">
        <input id="speedn" type="number" min="0" max="255" step="1" value="0" style="width:70px">
        <button id="speedset">Set speed</button>
      </div>
      <div class="kv" style="margin-top:6px">
        <span>applied</span><span id="speedcur" class="num">-</span>
      </div>
      <div class="hint">Find the PWM by hand: Set speed, hold W/S, adjust, repeat.
        START then drives forward at exactly the applied PWM. It applies at once,
        also while running. Open loop: no wheel feedback.</div>
    </section>

    <section class="manual-only">
      <h2>Teleop (hold)</h2>
      <div class="pad">
        <span></span><button data-drive="1,0">W &uarr;</button><span></span>
        <button data-drive="0,1">A &larr;</button>
        <button data-drive="-1,0">S &darr;</button>
        <button data-drive="0,-1">D &rarr;</button>
      </div>
    </section>

    <section class="manual-only">
      <h2>Gantry</h2>
      <div class="row" style="align-items:flex-start">
        <div class="pad">
          <span></span><button data-jog="0,1">Y+</button><span></span>
          <button data-jog="-1,0">X&minus;</button><span></span>
          <button data-jog="1,0">X+</button>
          <span></span><button data-jog="0,-1">Y&minus;</button><span></span>
        </div>
        <div>
          <div class="row">jog
            <select id="jogsize">
              <option>10</option><option>50</option><option selected>200</option>
              <option>800</option><option>1600</option>
            </select> steps</div>
          <div class="kv" style="margin-top:8px">
            <span>position</span><span id="pos" class="num">-</span>
            <span>steps left</span><span id="rem" class="num">-</span>
            <span>gripper</span><span id="servo" class="num">-</span>
          </div>
        </div>
      </div>
      <div class="row" style="margin-top:10px">
        <span>gripper</span>
        <button id="gopen">Open</button>
        <button id="gclose">Close</button>
        <input id="gdeg" type="number" min="0" max="180" value="90" style="width:70px">
        <button id="gset">Set &deg;</button>
      </div>
      <table style="margin-top:10px">
        <tr><td>home</td><td class="num">0, 0</td><td></td>
          <td><button data-goto="home">Go</button></td></tr>
        <tr><td>ready</td><td class="num" id="p_ready">-</td>
          <td><button data-save="ready">Save current</button></td>
          <td><button data-goto="ready">Go</button></td></tr>
        <tr><td>hit</td><td class="num" id="p_hit">-</td>
          <td><button data-save="hit">Save current</button></td>
          <td><button data-goto="hit">Go</button></td></tr>
      </table>
      <div class="row" style="margin-top:10px">
        <button id="sethome">Set home here</button>
      </div>
      <div class="hint"><b>ready</b>: where the gantry waits while driving.
        <b>hit</b>: where it reaches out to while a fruit is on the yellow hit
        line. Jog there, then "Save current"; "Go" tests it.</div>
    </section>
  </div>
</main>
<script>
const $ = (id) => document.getElementById(id);
let state = null;

async function cmd(body) {
  try {
    const r = await fetch('/api/cmd', {method: 'POST',
      headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    const res = await r.json();
    if (res.message) showMsg(res.message, !res.ok);
    return res;
  } catch (e) { showMsg('connection lost', true); }
}
function showMsg(text, err) {
  $('msg').textContent = text; $('msg').className = err ? 'err' : '';
}
const fmt = (p) => p ? p[0] + ', ' + p[1] : 'not set';

async function poll() {
  try {
    const r = await fetch('/api/state');
    state = await r.json();
  } catch (e) { state = null; }
  render();
}
function render() {
  const s = state;
  if (!s) { $('mode').textContent = 'no connection'; return; }
  $('mode').textContent = s.mode + (s.running ? ' (running)' : '');
  $('target').textContent = s.gantry_target || '-';
  $('drivepwm').textContent = s.running ? s.drive_pwm.toFixed(0) : '-';
  $('hits').textContent = s.hits;
  $('mega').className = 'dot' + (s.mega_ok ? ' ok' : '');
  $('vision').className = 'dot' + (s.vision_ok ? ' ok' : '');
  $('hit').className = 'dot' + (s.line_hit ? ' ok' : '');
  $('failsafe').textContent = s.failsafe ? ' WHEEL FAILSAFE' : '';
  $('wheelpwm').textContent = s.wheel_pwm ? s.wheel_pwm.join(', ') : '-';
  $('pos').textContent = fmt(s.pos);
  $('rem').textContent = fmt(s.rem);
  $('servo').textContent = s.servo === null ? '-' : s.servo + '°';
  $('p_ready').textContent = fmt(s.positions.ready);
  $('p_hit').textContent = fmt(s.positions.hit);
  $('msg').textContent = s.message; $('msg').className = s.error ? 'err' : '';
  const idle = s.mode === 'idle';
  $('start').disabled = !idle;
  document.querySelectorAll('.manual-only').forEach(
    (el) => el.classList.toggle('locked', !idle));
  $('speedcur').textContent = s.forward_pwm.toFixed(0);
  $('speed').max = $('speedn').max = s.max_forward_pwm;
  if (!speedLoaded) {           // fill the picker once; after that it is the user's
    speedLoaded = true; showSpeed(s.forward_pwm);
  }
}
let speedLoaded = false;
function showSpeed(v) { $('speed').value = v; $('speedn').value = Math.round(v); }
setInterval(poll, 250); poll();

$('start').onclick = () => cmd({cmd: 'start'});
$('stop').onclick = () => { stopDrive(); cmd({cmd: 'stop'}); };
$('sethome').onclick = () => {
  if (confirm('Make the current gantry position home (0, 0)? Saved positions keep their numbers.'))
    cmd({cmd: 'set_home'});
};
$('speed').oninput = (e) => { $('speedn').value = e.target.value; };
$('speed').onchange = (e) => e.target.blur();   // a focused slider eats key presses
$('speedn').oninput = (e) => { $('speed').value = e.target.value; };
$('speedn').onkeydown = (e) => { if (e.key === 'Enter') $('speedset').click(); };
$('speedset').onclick = async () => {
  const max = state ? state.max_forward_pwm : 255;
  const v = Math.max(0, Math.min(max, +$('speedn').value || 0));
  showSpeed(v);                  // show the clamped value the robot will use
  document.activeElement.blur(); // give W/A/S/D back to the page
  cmd({cmd: 'speed', value: v});
};
$('gopen').onclick = () => cmd({cmd: 'gripper', deg: state ? state.gripper_open : 90});
$('gclose').onclick = () => cmd({cmd: 'gripper', deg: state ? state.gripper_closed : 20});
$('gset').onclick = () => cmd({cmd: 'gripper', deg: +$('gdeg').value});
document.querySelectorAll('[data-save]').forEach((b) =>
  b.onclick = () => cmd({cmd: 'save', name: b.dataset.save}));
document.querySelectorAll('[data-goto]').forEach((b) =>
  b.onclick = () => cmd({cmd: 'goto', name: b.dataset.goto}));

function jog(dx, dy) {
  const n = +$('jogsize').value;
  cmd({cmd: 'jog', steps: [dx * n, dy * n]});
}
document.querySelectorAll('[data-jog]').forEach((b) => {
  const [dx, dy] = b.dataset.jog.split(',').map(Number);
  b.onclick = () => jog(dx, dy);
});

// ---- teleop: resend the held direction every 100 ms (server deadman 0.3 s) ----
let drive = null, driveTimer = null;
function sendDrive() {
  if (drive) fetch('/api/cmd', {method: 'POST', body: JSON.stringify(
    {cmd: 'drive', linear: drive[0], angular: drive[1]})}).catch(() => {});
}
function startDrive(v) {
  drive = v; sendDrive();
  if (!driveTimer) driveTimer = setInterval(sendDrive, 100);
}
function stopDrive() {
  if (!drive) return;
  drive = null; clearInterval(driveTimer); driveTimer = null;
  document.querySelectorAll('.held').forEach((b) => b.classList.remove('held'));
  fetch('/api/cmd', {method: 'POST', body: JSON.stringify(
    {cmd: 'drive', linear: 0, angular: 0})}).catch(() => {});
}
document.querySelectorAll('[data-drive]').forEach((b) => {
  const v = b.dataset.drive.split(',').map(Number);
  b.addEventListener('pointerdown', (e) => {
    b.setPointerCapture(e.pointerId); b.classList.add('held'); startDrive(v);
  });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(
    (ev) => b.addEventListener(ev, stopDrive));
});
window.addEventListener('blur', stopDrive);
document.addEventListener('visibilitychange', stopDrive);

const DRIVE_KEYS = {w: [1, 0], s: [-1, 0], a: [0, 1], d: [0, -1]};
const JOG_KEYS = {ArrowUp: [0, 1], ArrowDown: [0, -1], ArrowLeft: [-1, 0], ArrowRight: [1, 0]};
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
  if (e.repeat) { if (k in JOG_KEYS) e.preventDefault(); return; }
  if (k in DRIVE_KEYS) { held.add(k); keyDrive(); }
  else if (k in JOG_KEYS) { e.preventDefault(); jog(...JOG_KEYS[k]); }
  else if (k === 'o') $('gopen').click();
  else if (k === 'c') $('gclose').click();
});
document.addEventListener('keyup', (e) => {
  const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (held.delete(k)) keyDrive();
});
</script>
</body>
</html>
"""


class FrameStore:
    """Latest JPEG frame shared between the ROS thread and HTTP threads."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._sequence = 0

    def put(self, jpeg: bytes) -> None:
        with self._condition:
            self._jpeg = jpeg
            self._sequence += 1
            self._condition.notify_all()

    def latest(self) -> Optional[bytes]:
        with self._condition:
            return self._jpeg

    def wait_next(self, last_sequence: int, timeout: float):
        with self._condition:
            self._condition.wait_for(lambda: self._sequence != last_sequence, timeout)
            return self._sequence, self._jpeg


def make_handler(frames: FrameStore,
                 get_state: Callable[[], dict],
                 submit: Callable[[dict], Tuple[bool, str]]):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 (http.server API)
            if self.path in ('/', '/index.html'):
                self._send(200, 'text/html; charset=utf-8', INDEX_HTML.encode())
            elif self.path == '/api/state':
                self._json(200, get_state())
            elif self.path == '/snapshot.jpg':
                jpeg = frames.latest()
                if jpeg is None:
                    self._send(503, 'text/plain', b'no frame yet')
                else:
                    self._send(200, 'image/jpeg', jpeg)
            elif self.path == '/stream.mjpg':
                self._stream()
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

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            sequence = -1
            try:
                while True:
                    sequence, jpeg = frames.wait_next(sequence, timeout=1.0)
                    if jpeg is None:
                        continue
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n'
                        + f'Content-Length: {len(jpeg)}\r\n\r\n'.encode()
                        + jpeg + b'\r\n'
                    )
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, format, *args) -> None:  # noqa: A002
            pass

    return Handler


class WebServer:
    def __init__(self, host: str, port: int, frames: FrameStore,
                 get_state: Callable[[], dict],
                 submit: Callable[[dict], Tuple[bool, str]]) -> None:
        self._server = ThreadingHTTPServer((host, port), make_handler(frames, get_state, submit))
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._server.shutdown()
        self._server.server_close()
