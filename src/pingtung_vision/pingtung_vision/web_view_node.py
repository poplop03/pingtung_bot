"""Serve the annotated vision image to a browser as an MJPEG stream."""

from __future__ import annotations

import os
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String


INDEX_HTML = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pingtung Vision</title>
<style>
  body { margin: 0; background: #111; color: #ddd; font-family: sans-serif;
         text-align: center; }
  h1 { font-size: 18px; margin: 12px; }
  img { max-width: 100%; height: auto; border: 1px solid #333; }
</style>
</head>
<body>
<h1>Pingtung Vision &mdash; mode: <span id="mode">?</span></h1>
<img src="/stream.mjpg" alt="waiting for /vision/debug_image ...">
<script>
  async function poll() {
    try {
      const r = await fetch('/mode');
      document.getElementById('mode').textContent = await r.text();
    } catch (e) {}
  }
  poll();
  setInterval(poll, 2000);
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
        self.mode = 'unknown'

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
            self._condition.wait_for(
                lambda: self._sequence != last_sequence, timeout
            )
            return self._sequence, self._jpeg


def make_handler(store: FrameStore):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 (http.server API)
            if self.path in ('/', '/index.html'):
                self._send(200, 'text/html; charset=utf-8', INDEX_HTML.encode())
            elif self.path == '/mode':
                self._send(200, 'text/plain; charset=utf-8', store.mode.encode())
            elif self.path == '/snapshot.jpg':
                jpeg = store.latest()
                if jpeg is None:
                    self._send(503, 'text/plain', b'no frame yet')
                else:
                    self._send(200, 'image/jpeg', jpeg)
            elif self.path == '/stream.mjpg':
                self._stream()
            else:
                self._send(404, 'text/plain', b'not found')

        def _send(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header(
                'Content-Type', 'multipart/x-mixed-replace; boundary=frame'
            )
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            sequence = -1
            try:
                while True:
                    sequence, jpeg = store.wait_next(sequence, timeout=1.0)
                    if jpeg is None:
                        continue
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n'
                        + f'Content-Length: {len(jpeg)}\r\n\r\n'.encode()
                        + jpeg
                        + b'\r\n'
                    )
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, format, *args) -> None:  # noqa: A002
            pass

    return Handler


class WebViewNode(Node):
    """Subscribe to an image topic and publish it over HTTP."""

    def __init__(self) -> None:
        super().__init__('vision_web_view')
        self.declare_parameter('image_topic', '/vision/debug_image')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('jpeg_quality', 80)

        self._bridge = CvBridge()
        self._store = FrameStore()
        self._quality = int(self.get_parameter('jpeg_quality').value)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        mode_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        image_topic = str(self.get_parameter('image_topic').value)
        self.create_subscription(Image, image_topic, self._on_image, image_qos)
        self.create_subscription(
            String, '/vision/active_mode', self._on_mode, mode_qos
        )

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        self._server = ThreadingHTTPServer((host, port), make_handler(self._store))
        self._server.daemon_threads = True
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True
        )
        self._thread.start()
        self.get_logger().info(
            f'Streaming {image_topic} at http://{host}:{port}/'
        )

    def _on_image(self, message: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        ok, encoded = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality]
        )
        if ok:
            self._store.put(encoded.tobytes())

    def _on_mode(self, message: String) -> None:
        self._store.mode = message.data

    def shutdown(self) -> None:
        self._server.shutdown()
        self._server.server_close()


def _hold_off_signals(deadline_s: float = 3.0) -> None:
    """Let the cleanup finish, but never let the process outlive Ctrl+C.

    Under ros2 launch, Ctrl+C reaches the node twice (terminal and launch), and
    the second KeyboardInterrupt would cut destroy_node() short. Ignore it,
    make SIGTERM kill at once, and exit hard if the cleanup hangs.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    watchdog = threading.Timer(deadline_s, os._exit, (1,))
    watchdog.daemon = True
    watchdog.start()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebViewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        _hold_off_signals()
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
