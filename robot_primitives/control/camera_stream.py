from __future__ import annotations

import argparse
import html
import platform
import signal
import socket
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

try:
    import cv2
except ImportError:  # pragma: no cover - exercised by users without OpenCV installed.
    cv2 = None


def local_ip() -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
        except OSError:
            return "127.0.0.1"


class Camera:
    def __init__(
        self,
        index: int | str,
        width: int,
        height: int,
        fps: int,
        quality: int,
    ) -> None:
        if cv2 is None:
            raise RuntimeError(
                "OpenCV is not installed. Run: pip install -r requirements.txt"
            )

        self.index = resolve_camera_index(index)
        self.quality = quality
        self._lock = threading.Lock()
        self._latest: Optional[bytes] = None
        self._running = threading.Event()
        self._running.set()

        self.capture = open_capture(self.index)
        if not self.capture.isOpened():
            raise RuntimeError(f"Could not open camera index {self.index}")

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)

        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self) -> None:
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        while self._running.is_set():
            ok, frame = self.capture.read()
            if not ok:
                time.sleep(0.1)
                continue

            ok, encoded = cv2.imencode(".jpg", frame, encode_params)
            if ok:
                with self._lock:
                    self._latest = encoded.tobytes()

    def frame(self) -> Optional[bytes]:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._running.clear()
        self.thread.join(timeout=1)
        self.capture.release()


def camera_backend() -> int:
    if cv2 is None:
        return 0
    if platform.system() == "Darwin":
        return cv2.CAP_AVFOUNDATION
    return cv2.CAP_ANY


def open_capture(index: int):
    backend = camera_backend()
    if backend:
        return cv2.VideoCapture(index, backend)
    return cv2.VideoCapture(index)


def default_camera_scan_limit() -> int:
    if platform.system() == "Darwin":
        return 2
    return 8


def available_camera_indices(limit: int | None = None) -> list[int]:
    if limit is None:
        limit = default_camera_scan_limit()

    found: list[int] = []
    for index in range(limit):
        capture = open_capture(index)
        try:
            if capture.isOpened():
                ok, _frame = capture.read()
                if ok:
                    found.append(index)
        finally:
            capture.release()
    return found


def resolve_camera_index(value: int | str) -> int:
    if isinstance(value, int):
        return value

    raw = value.strip().lower()
    if raw.isdigit():
        return int(raw)
    if raw not in {"auto", "external", "usb"}:
        raise ValueError("--camera must be a number, 'auto', or 'external'")

    cameras = available_camera_indices()
    if not cameras:
        raise RuntimeError("No camera found")

    if raw in {"external", "usb"}:
        for index in cameras:
            if index != 0:
                return index
        raise RuntimeError(
            "Only camera index 0 was found. Check the external webcam connection, "
            "then run with --list-cameras."
        )

    return cameras[0]


def page_html(title: str) -> bytes:
    safe_title = html.escape(title)
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{safe_title}</title>
  <style>
    html, body {{
      margin: 0;
      min-height: 100%;
      background: #101412;
      color: #f5f7f2;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }}
    body {{
      display: grid;
      grid-template-rows: auto 1fr;
    }}
    header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      padding: 12px 14px;
      background: #17201b;
      border-bottom: 1px solid #2a352f;
    }}
    h1 {{
      margin: 0;
      font-size: 17px;
      font-weight: 650;
    }}
    a {{
      color: #9ee493;
      font-size: 14px;
      text-decoration: none;
    }}
    main {{
      display: grid;
      place-items: center;
      min-height: 0;
      padding: 10px;
    }}
    img {{
      display: block;
      width: min(100%, 960px);
      max-height: calc(100vh - 72px);
      object-fit: contain;
      background: #000;
      border: 1px solid #2a352f;
    }}
  </style>
</head>
<body>
  <header>
    <h1>{safe_title}</h1>
    <a href="/stream">MJPEG</a>
  </header>
  <main>
    <img src="/stream" alt="Live camera stream">
  </main>
</body>
</html>
""".encode("utf-8")


class StreamHandler(BaseHTTPRequestHandler):
    server: "CameraServer"

    def do_GET(self) -> None:
        if self.path in {"/", "/index.html"}:
            body = page_html(self.server.title)
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/health":
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"ok\n")
            return

        if self.path == "/stream":
            self._stream()
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def _stream(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=robot-camera-frame",
        )
        self.end_headers()

        delay = 1 / max(self.server.fps, 1)
        while True:
            frame = self.server.camera.frame()
            if frame is None:
                time.sleep(delay)
                continue

            try:
                self.wfile.write(b"--robot-camera-frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(delay)
            except (BrokenPipeError, ConnectionResetError):
                return

    def log_message(self, fmt: str, *args: object) -> None:
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))


class CameraServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        handler_class: type[BaseHTTPRequestHandler],
        camera: Camera,
        fps: int,
        title: str,
    ) -> None:
        super().__init__(server_address, handler_class)
        self.camera = camera
        self.fps = fps
        self.title = title


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Serve a USB/webcam live view for phones on the same network."
    )
    parser.add_argument(
        "--camera",
        default="external",
        help="Camera index, 'external' for the first non-built-in camera, or 'auto'",
    )
    parser.add_argument(
        "--list-cameras",
        action="store_true",
        help="Print usable OpenCV camera indices and exit",
    )
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port")
    parser.add_argument("--width", type=int, default=1280, help="Capture width")
    parser.add_argument("--height", type=int, default=720, help="Capture height")
    parser.add_argument("--fps", type=int, default=15, help="Stream frame rate")
    parser.add_argument("--quality", type=int, default=80, help="JPEG quality 1-100")
    parser.add_argument("--title", default="Robot Arm Camera", help="Page title")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.list_cameras:
        cameras = available_camera_indices()
        if not cameras:
            print("No cameras found.")
            return 1
        print("Usable camera indices:")
        for index in cameras:
            label = "built-in/default" if index == 0 else "external candidate"
            print(f"  {index} ({label})")
        return 0

    camera = Camera(
        index=args.camera,
        width=args.width,
        height=args.height,
        fps=args.fps,
        quality=args.quality,
    )
    server = CameraServer(
        (args.host, args.port),
        StreamHandler,
        camera=camera,
        fps=args.fps,
        title=args.title,
    )

    def stop(_signum: int, _frame: object) -> None:
        server.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    address = local_ip() if args.host in {"0.0.0.0", "::"} else args.host
    print(f"Open this on your phone: http://{address}:{args.port}")
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    finally:
        camera.close()
        server.server_close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
