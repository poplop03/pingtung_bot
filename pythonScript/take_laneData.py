"""Record lane-data videos from a camera with a keyboard toggle.

Controls:
    SPACE or R: start/stop recording
    Q or ESC:   quit
    Web page:   open http://<jetson-ip>:8080/ to watch the live preview and
                start/stop recording from the browser
    Drive:      hold the on-page forward/backward/left/right buttons to drive
                the robot; they publish geometry_msgs/Twist on /mega/cmd_vel
                (needs a sourced ROS 2 environment). Releasing them, or losing
                the connection, stops. Forward/backward can be combined with a
                turn. The sliders below set the linear and angular speed live.
    Ctrl+C:     quit (useful when running headless with --no-window)

The preview contains status text, but the saved video contains clean camera
frames only.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record camera video for lane-data collection"
    )
    parser.add_argument("--camera", type=int, default=4, help="Camera index")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("lane_videos"),
        help="Directory for recorded videos (default: lane_videos)",
    )
    # 640x480 keeps the loop well inside 30 FPS on the Jetson; 1280x720 takes
    # ~36 ms/frame while recording (mp4v encode + preview) and starts to lag.
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--preview-width",
        type=int,
        default=640,
        help="Web/window preview is scaled down to this width; the recording "
        "keeps the full camera size (default: 640)",
    )
    parser.add_argument(
        "--preview-fps",
        type=float,
        default=15.0,
        help="Max web preview FPS, to spare CPU and Wi-Fi (default: 15)",
    )
    parser.add_argument(
        "--preview-quality",
        type=int,
        default=60,
        help="Web preview JPEG quality 1-100 (default: 60)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=0.0,
        help="Output FPS; 0 uses camera FPS with a 30 FPS fallback",
    )
    parser.add_argument(
        "--codec",
        default="mp4v",
        help="FourCC codec with exactly four characters (default: mp4v)",
    )
    parser.add_argument(
        "--port", type=int, default=8080, help="Web preview port (default: 8080)"
    )
    parser.add_argument(
        "--no-window",
        action="store_true",
        help="Do not open a local preview window (auto when DISPLAY is unset)",
    )
    parser.add_argument(
        "--cmd-vel-topic",
        default="/mega/cmd_vel",
        help="Twist topic for joystick teleop (default: /mega/cmd_vel)",
    )
    parser.add_argument(
        "--max-linear",
        type=float,
        default=0.2,
        help="linear.x at full joystick deflection (default: 0.2)",
    )
    parser.add_argument(
        "--max-angular",
        type=float,
        default=1.0,
        help="angular.z in rad/s at full joystick deflection (default: 1.0)",
    )
    parser.add_argument(
        "--linear-limit",
        type=float,
        default=1.0,
        help="Upper end of the web max-linear slider (default: 1.0)",
    )
    parser.add_argument(
        "--angular-limit",
        type=float,
        default=3.0,
        help="Upper end of the web max-angular slider in rad/s (default: 3.0)",
    )
    parser.add_argument(
        "--no-teleop",
        action="store_true",
        help="Do not start the ROS 2 joystick publisher",
    )
    return parser.parse_args()


PAGE = b"""<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Lane data recorder</title>
<style>
  body { margin: 0; background: #111; color: #eee; font-family: sans-serif;
         text-align: center; }
  img { max-width: 100%; height: auto; display: block; margin: 0 auto; }
  button { margin: 12px; padding: 14px 32px; font-size: 20px;
           border: 0; border-radius: 8px; background: #c33; color: #fff; }
  #pad { display: grid; grid-template-columns: repeat(3, 72px);
         grid-template-rows: repeat(3, 72px); gap: 8px; justify-content: center;
         margin: 8px auto 16px; touch-action: none; user-select: none;
         -webkit-user-select: none; }
  #pad button { margin: 0; padding: 0; font-size: 28px; background: #2a2a2a;
                border: 2px solid #555; touch-action: none; }
  #pad button.on { background: #3a8ee6; border-color: #3a8ee6; }
  #fwd { grid-column: 2; grid-row: 1; }
  #left { grid-column: 1; grid-row: 2; }
  #right { grid-column: 3; grid-row: 2; }
  #back { grid-column: 2; grid-row: 3; }
  #joyval { font-size: 14px; color: #aaa; }
  .speed { width: 280px; max-width: 90%; margin: 12px auto; text-align: left; }
  .speed label { display: flex; justify-content: space-between; font-size: 15px; }
  .speed input { width: 100%; }
</style></head>
<body>
  <img src="/stream" alt="live camera">
  <button onclick="fetch('/toggle', {method: 'POST'})">Start / Stop recording</button>
  <div id="pad">
    <button id="fwd" data-y="1" aria-label="forward">&#9650;</button>
    <button id="left" data-x="-1" aria-label="turn left">&#9664;</button>
    <button id="right" data-x="1" aria-label="turn right">&#9654;</button>
    <button id="back" data-y="-1" aria-label="backward">&#9660;</button>
  </div>
  <div id="joyval">stopped</div>
  <div class="speed">
    <label for="lin"><span>Max linear</span><span id="linval">-</span></label>
    <input id="lin" type="range" min="0" step="0.01" disabled>
  </div>
  <div class="speed">
    <label for="ang"><span>Max angular</span><span id="angval">-</span></label>
    <input id="ang" type="range" min="0" step="0.05" disabled>
  </div>
<script>
  const label = document.getElementById('joyval');
  const held = new Map();  // pointerId -> button
  let x = 0, y = 0, timer = null;

  function send() {
    fetch('/joy', {method: 'POST', headers: {'Content-Type': 'application/json'},
                   body: JSON.stringify({x: x, y: y})}).catch(() => {});
  }
  function update() {
    x = 0; y = 0;
    const down = new Set(held.values());
    for (const b of document.querySelectorAll('#pad button')) {
      const on = down.has(b);
      b.classList.toggle('on', on);
      if (on) { x += Number(b.dataset.x || 0); y += Number(b.dataset.y || 0); }
    }
    label.textContent = held.size
      ? [...down].map((b) => b.getAttribute('aria-label')).join(' + ')
      : 'stopped';
    send();
    // Keep streaming while held; the server stops the robot if this stops.
    if (held.size && !timer) timer = setInterval(send, 100);
    if (!held.size && timer) { clearInterval(timer); timer = null; }
  }
  function release(e) {
    if (held.delete(e.pointerId)) update();
  }
  for (const b of document.querySelectorAll('#pad button')) {
    b.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      b.setPointerCapture(e.pointerId);
      held.set(e.pointerId, b);
      update();
    });
    b.addEventListener('pointerup', release);
    b.addEventListener('pointercancel', release);
    b.addEventListener('contextmenu', (e) => e.preventDefault());
  }
  window.addEventListener('blur', () => { if (held.size) { held.clear(); update(); } });

  const lin = document.getElementById('lin');
  const ang = document.getElementById('ang');
  const linval = document.getElementById('linval');
  const angval = document.getElementById('angval');
  function showSpeed() {
    linval.textContent = Number(lin.value).toFixed(2) + ' m/s';
    angval.textContent = Number(ang.value).toFixed(2) + ' rad/s';
  }
  function sendSpeed() {
    showSpeed();
    fetch('/speed', {method: 'POST', headers: {'Content-Type': 'application/json'},
                     body: JSON.stringify({linear: Number(lin.value),
                                           angular: Number(ang.value)})})
      .catch(() => {});
  }
  lin.addEventListener('input', sendSpeed);
  ang.addEventListener('input', sendSpeed);
  // Start from the server's values so a reload or second client stays in sync.
  fetch('/speed').then((r) => r.json()).then((s) => {
    lin.max = s.linear_limit; ang.max = s.angular_limit;
    lin.value = s.linear; ang.value = s.angular;
    lin.disabled = false; ang.disabled = false;
    showSpeed();
  }).catch(() => {});
</script>
</body></html>
"""


class FrameHub:
    """Latest JPEG preview shared with web clients, plus web toggle requests."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._seq = 0
        self.toggle_requested = threading.Event()

    def publish(self, jpeg: bytes) -> None:
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    def wait_next(self, last_seq: int) -> tuple[int, Optional[bytes]]:
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq, timeout=5.0)
            return self._seq, self._jpeg


class SpeedLimits:
    """Full-deflection joystick speeds, adjustable from the web sliders."""

    def __init__(
        self,
        max_linear: float,
        max_angular: float,
        linear_limit: float,
        angular_limit: float,
    ) -> None:
        self.linear_limit = max(linear_limit, max_linear)
        self.angular_limit = max(angular_limit, max_angular)
        self._lock = threading.Lock()
        self._linear = 0.0
        self._angular = 0.0
        self.set(max_linear, max_angular)

    def set(self, linear: float, angular: float) -> None:
        with self._lock:
            self._linear = max(0.0, min(self.linear_limit, linear))
            self._angular = max(0.0, min(self.angular_limit, angular))

    def get(self) -> tuple[float, float]:
        with self._lock:
            return self._linear, self._angular


class Teleop:
    """Publishes the web drive buttons as a Twist on a ROS 2 topic at 20 Hz.

    The browser streams the button state at 10 Hz while one is held. If updates stop
    for JOY_TIMEOUT (released, tab closed, Wi-Fi dropped), one zero Twist is
    sent and publishing stops, so wheel_control's own timeout also applies.
    """

    JOY_TIMEOUT = 0.3
    DEADZONE = 0.1

    def __init__(self, topic: str, speeds: SpeedLimits) -> None:
        import rclpy
        from geometry_msgs.msg import Twist

        self._rclpy = rclpy
        self._twist_type = Twist
        self._speeds = speeds
        self._lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._last_update = 0.0
        self._was_active = False

        rclpy.init()
        self._node = rclpy.create_node("lane_recorder_teleop")
        self._pub = self._node.create_publisher(Twist, topic, 10)
        self._node.create_timer(0.05, self._tick)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        try:
            self._rclpy.spin(self._node)
        except Exception:
            pass  # raised when rclpy shuts down during quit

    def set_joystick(self, x: float, y: float) -> None:
        with self._lock:
            self._x = max(-1.0, min(1.0, x))
            self._y = max(-1.0, min(1.0, y))
            self._last_update = time.monotonic()

    def _tick(self) -> None:
        with self._lock:
            x, y = self._x, self._y
            active = time.monotonic() - self._last_update < self.JOY_TIMEOUT
        if not active:
            if self._was_active:
                self._pub.publish(self._twist_type())
                self._was_active = False
            return
        self._was_active = True
        max_linear, max_angular = self._speeds.get()
        msg = self._twist_type()
        if abs(y) > self.DEADZONE:
            msg.linear.x = max_linear * y
        if abs(x) > self.DEADZONE:
            # Right button = turn right = negative yaw rate (+z is left).
            msg.angular.z = -max_angular * x
        self._pub.publish(msg)

    def shutdown(self) -> None:
        self._pub.publish(self._twist_type())
        # Stop spinning before destroying the node, or rclpy aborts.
        self._rclpy.try_shutdown()
        self._thread.join(timeout=1.0)
        self._node.destroy_node()


def start_teleop(
    args: argparse.Namespace, speeds: SpeedLimits
) -> Optional[Teleop]:
    if args.no_teleop:
        return None
    try:
        teleop = Teleop(args.cmd_vel_topic, speeds)
    except ImportError:
        print("rclpy not found (source ROS 2 first); joystick teleop disabled")
        return None
    print(f"Joystick teleop publishing on {args.cmd_vel_topic}")
    return teleop


def start_web_server(
    hub: FrameHub, port: int, teleop: Optional[Teleop], speeds: SpeedLimits
) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def read_json(self):
            length = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(length))

        def do_GET(self) -> None:
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path == "/speed":
                linear, angular = speeds.get()
                body = json.dumps({
                    "linear": linear,
                    "angular": angular,
                    "linear_limit": speeds.linear_limit,
                    "angular_limit": speeds.angular_limit,
                }).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                seq = 0
                try:
                    while True:
                        seq, jpeg = hub.wait_next(seq)
                        if jpeg is None:
                            continue
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n"
                            b"Content-Length: %d\r\n\r\n" % len(jpeg)
                        )
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def do_POST(self) -> None:
            if self.path == "/toggle":
                hub.toggle_requested.set()
                self.send_response(204)
                self.end_headers()
            elif self.path == "/joy":
                try:
                    data = self.read_json()
                    x, y = float(data["x"]), float(data["y"])
                except (ValueError, KeyError, TypeError):
                    self.send_error(400)
                    return
                if teleop is not None:
                    teleop.set_joystick(x, y)
                self.send_response(204)
                self.end_headers()
            elif self.path == "/speed":
                try:
                    data = self.read_json()
                    linear = float(data["linear"])
                    angular = float(data["angular"])
                except (ValueError, KeyError, TypeError):
                    self.send_error(400)
                    return
                if not (math.isfinite(linear) and math.isfinite(angular)):
                    self.send_error(400)
                    return
                speeds.set(linear, angular)
                self.send_response(204)
                self.end_headers()
            else:
                self.send_error(404)

        def log_message(self, format: str, *args) -> None:
            pass  # keep the terminal for recorder messages

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def local_ip() -> str:
    # No packet is sent; this only asks the OS which interface routes outward.
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "<jetson-ip>"


def video_path(output_dir: Path) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    return output_dir / f"lane_{timestamp}.mp4"


def create_writer(
    path: Path,
    codec: str,
    fps: float,
    frame_width: int,
    frame_height: int,
) -> cv2.VideoWriter:
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*codec),
        fps,
        (frame_width, frame_height),
    )
    if not writer.isOpened():
        writer.release()
        raise RuntimeError(
            f"Cannot create video {path}. Try --codec mp4v or --codec XVID."
        )
    return writer


def draw_preview(
    frame,
    recording: bool,
    recorded_frames: int,
    fps: float,
    current_path: Optional[Path],
):
    preview = frame.copy()
    overlay = preview.copy()
    cv2.rectangle(overlay, (10, 10), (610, 115), (0, 0, 0), cv2.FILLED)
    cv2.addWeighted(overlay, 0.65, preview, 0.35, 0, preview)

    if recording:
        elapsed = recorded_frames / max(fps, 1e-6)
        # Blinking recording dot makes the current state easy to notice.
        if int(time.monotonic() * 2) % 2 == 0:
            cv2.circle(preview, (31, 35), 10, (0, 0, 255), cv2.FILLED)
        status = f"REC  {elapsed:06.1f}s  {recorded_frames} frames"
        colour = (0, 0, 255)
        filename = current_path.name if current_path else ""
    else:
        status = "READY - not recording"
        colour = (0, 255, 255)
        filename = "Press SPACE/R or the web button to start"

    cv2.putText(
        preview, status, (50, 43), cv2.FONT_HERSHEY_SIMPLEX,
        0.78, colour, 2, cv2.LINE_AA,
    )
    cv2.putText(
        preview, filename, (22, 76), cv2.FONT_HERSHEY_SIMPLEX,
        0.55, (235, 235, 235), 1, cv2.LINE_AA,
    )
    cv2.putText(
        preview, "SPACE/R: start-stop    Q/ESC: quit", (22, 101),
        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (235, 235, 235), 1, cv2.LINE_AA,
    )
    return preview


def run(args: argparse.Namespace) -> int:
    if len(args.codec) != 4:
        raise ValueError("--codec must contain exactly four characters")
    if args.width <= 0 or args.height <= 0:
        raise ValueError("--width and --height must be positive")
    if args.fps < 0:
        raise ValueError("--fps cannot be negative")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    # V4L2 explicitly: OpenCV's default GStreamer backend ignores the size
    # request and delivers 1920x1080, which made the loop lag.
    camera = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not camera.isOpened():
        print(f"Cannot open camera {args.camera}")
        return 2

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    # Keep only the newest frame: if the loop is ever slow, show and record what
    # the camera sees now instead of working through a queue of old frames.
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    camera_fps = float(camera.get(cv2.CAP_PROP_FPS))
    output_fps = args.fps if args.fps > 0 else camera_fps
    if output_fps <= 1.0 or output_fps > 240.0:
        output_fps = 30.0

    writer: Optional[cv2.VideoWriter] = None
    current_path: Optional[Path] = None
    recorded_frames = 0
    show_window = not args.no_window and bool(os.environ.get("DISPLAY"))

    hub = FrameHub()
    speeds = SpeedLimits(
        args.max_linear, args.max_angular, args.linear_limit, args.angular_limit
    )
    teleop = start_teleop(args, speeds)
    server = start_web_server(hub, args.port, teleop, speeds)

    preview_period = 1.0 / args.preview_fps if args.preview_fps > 0 else 0.0
    last_preview = 0.0

    print(f"Camera: {args.camera} at "
          f"{int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    print(f"Recording FPS: {output_fps:.2f}")
    print(f"Output directory: {args.output_dir.resolve()}")
    print(f"Live view: http://{local_ip()}:{args.port}/")
    if show_window:
        print("SPACE/R = start-stop, Q/ESC = quit")
    else:
        print("No local window; use the web page to start-stop, Ctrl+C to quit")

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                print("Cannot read frame from camera")
                break

            if writer is not None:
                writer.write(frame)
                recorded_frames += 1

            # The preview is only for watching: build it at a reduced size and
            # rate. Every full-size frame above still goes into the recording.
            key = -1
            now = time.monotonic()
            if now - last_preview >= preview_period:
                last_preview = now
                small = frame
                if 0 < args.preview_width < frame.shape[1]:
                    scale = args.preview_width / frame.shape[1]
                    small = cv2.resize(frame, None, fx=scale, fy=scale,
                                       interpolation=cv2.INTER_AREA)
                preview = draw_preview(
                    small,
                    writer is not None,
                    recorded_frames,
                    output_fps,
                    current_path,
                )
                ok, jpeg = cv2.imencode(
                    ".jpg", preview,
                    [cv2.IMWRITE_JPEG_QUALITY, args.preview_quality],
                )
                if ok:
                    hub.publish(jpeg.tobytes())
                if show_window:
                    cv2.imshow("Lane data recorder", preview)
            if show_window:
                key = cv2.pollKey() & 0xFF

            if key in (27, ord("q"), ord("Q")):
                break
            toggle = key in (32, ord("r"), ord("R"))
            if hub.toggle_requested.is_set():
                hub.toggle_requested.clear()
                toggle = True
            if toggle:
                if writer is None:
                    height, width = frame.shape[:2]
                    current_path = video_path(args.output_dir)
                    try:
                        writer = create_writer(
                            current_path, args.codec, output_fps, width, height
                        )
                    except RuntimeError as error:
                        print(error)
                        current_path = None
                        continue
                    recorded_frames = 0
                    print(f"Recording started: {current_path}")
                else:
                    writer.release()
                    writer = None
                    print(
                        f"Recording stopped: {current_path} "
                        f"({recorded_frames} frames)"
                    )
                    current_path = None
                    recorded_frames = 0
    except KeyboardInterrupt:
        print()
    finally:
        server.shutdown()
        if teleop is not None:
            teleop.shutdown()
        if writer is not None:
            writer.release()
            print(
                f"Recording saved before exit: {current_path} "
                f"({recorded_frames} frames)"
            )
        camera.release()
        cv2.destroyAllWindows()
    return 0


def main() -> int:
    try:
        return run(parse_args())
    except (ValueError, RuntimeError, cv2.error) as error:
        print(f"Error: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
