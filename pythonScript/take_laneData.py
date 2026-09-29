"""Record lane-data videos from a camera with a keyboard toggle.

Controls:
    SPACE or R: start/stop recording
    Q or ESC:   quit
    Web page:   open http://<jetson-ip>:8080/ to watch the live preview and
                start/stop recording from the browser
    Ctrl+C:     quit (useful when running headless with --no-window)

The preview contains status text, but the saved video contains clean camera
frames only.
"""

from __future__ import annotations

import argparse
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
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("lane_videos"),
        help="Directory for recorded videos (default: lane_videos)",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
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
</style></head>
<body>
  <img src="/stream" alt="live camera">
  <button onclick="fetch('/toggle', {method: 'POST'})">Start / Stop recording</button>
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


def start_web_server(hub: FrameHub, port: int) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
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
    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        print(f"Cannot open camera {args.camera}")
        return 2

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    camera_fps = float(camera.get(cv2.CAP_PROP_FPS))
    output_fps = args.fps if args.fps > 0 else camera_fps
    if output_fps <= 1.0 or output_fps > 240.0:
        output_fps = 30.0

    writer: Optional[cv2.VideoWriter] = None
    current_path: Optional[Path] = None
    recorded_frames = 0
    show_window = not args.no_window and bool(os.environ.get("DISPLAY"))

    hub = FrameHub()
    server = start_web_server(hub, args.port)

    print(f"Camera: {args.camera}")
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

            preview = draw_preview(
                frame,
                writer is not None,
                recorded_frames,
                output_fps,
                current_path,
            )
            ok, jpeg = cv2.imencode(
                ".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 75]
            )
            if ok:
                hub.publish(jpeg.tobytes())

            key = -1
            if show_window:
                cv2.imshow("Lane data recorder", preview)
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
