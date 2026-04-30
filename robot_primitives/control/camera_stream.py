from __future__ import annotations

import argparse
import html
import json
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
        self._latest_frame = None
        self._running = threading.Event()
        self._running.set()

        self.capture = open_capture(self.index)
        if not self.capture.isOpened():
            raise RuntimeError(f"Could not open camera index {self.index}")

        configure_capture(self.capture, width=width, height=height, fps=fps)

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
                    self._latest_frame = frame.copy()

    def frame(self) -> Optional[bytes]:
        with self._lock:
            return self._latest

    def frame_image(self):
        with self._lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()

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


def configure_capture(capture, *, width: int, height: int, fps: int) -> None:
    if cv2 is None:
        return

    # Some USB webcams expose YUV-like formats that OpenCV decodes with a pink
    # cast on macOS. MJPG is usually the same stable path OBS chooses.
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_CONVERT_RGB, 1)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)

    auto_wb = getattr(cv2, "CAP_PROP_AUTO_WB", None)
    if auto_wb is not None:
        capture.set(auto_wb, 1)


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


def page_html(title: str, detector_enabled: bool) -> bytes:
    safe_title = html.escape(title)
    annotated_link = '<a href="/annotated_stream">YOLO</a>' if detector_enabled else ""
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
    .actions {{
      display: flex;
      align-items: center;
      gap: 10px;
      flex-wrap: wrap;
      justify-content: flex-end;
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
    <div class="actions">
      <a href="/stream">MJPEG</a>
      {annotated_link}
    </div>
  </header>
  <main>
    <img src="/stream" alt="Live camera stream">
  </main>
</body>
</html>
""".encode("utf-8")


class YoloFrameDetector:
    def __init__(self, model_path: str, conf: float) -> None:
        self.model_path = model_path
        self.conf = conf
        self._model = None
        self._lock = threading.Lock()

    def _load_model(self):
        if self._model is None:
            from ultralytics import YOLO

            self._model = YOLO(self.model_path)
        return self._model

    def _predict(self, frame):
        model = self._load_model()
        results = model(frame, conf=self.conf, verbose=False)
        return model, results

    def detect(self, frame) -> dict[str, object]:
        with self._lock:
            model, results = self._predict(frame)

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return {"ok": False, "best": None}

        best = max(boxes, key=lambda box: float(box.conf[0]))
        x1, y1, x2, y2 = [int(value) for value in best.xyxy[0].tolist()]
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        confidence = float(best.conf[0])
        label = model.names[int(best.cls[0])]
        return {
            "ok": True,
            "best": {
                "label": label,
                "confidence": round(confidence, 4),
                "box": [x1, y1, x2, y2],
                "center": [cx, cy],
            },
        }

    def annotated_jpeg(self, frame, quality: int) -> bytes:
        with self._lock:
            _model, results = self._predict(frame)
            annotated = results[0].plot()

        ok, encoded = cv2.imencode(
            ".jpg",
            annotated,
            [int(cv2.IMWRITE_JPEG_QUALITY), quality],
        )
        if not ok:
            raise RuntimeError("Could not encode annotated frame")
        return encoded.tobytes()


class StreamHandler(BaseHTTPRequestHandler):
    server: "CameraServer"

    def do_GET(self) -> None:
        if self.path in {"/", "/index.html"}:
            body = page_html(self.server.title, self.server.detector is not None)
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

        if self.path == "/annotated_stream":
            self._annotated_stream()
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        if self.path == "/detect":
            self._detect()
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def _send_json(self, status: HTTPStatus, payload: dict[str, object]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _detect(self) -> None:
        if self.server.detector is None:
            self._send_json(
                HTTPStatus.BAD_REQUEST,
                {"ok": False, "error": "Detection model is not configured"},
            )
            return

        frame = self.server.camera.frame_image()
        if frame is None:
            self._send_json(
                HTTPStatus.SERVICE_UNAVAILABLE,
                {"ok": False, "error": "No camera frame is available yet"},
            )
            return

        try:
            payload = self.server.detector.detect(frame)
        except Exception as exc:
            self._send_json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {"ok": False, "error": str(exc)},
            )
            return

        payload["action"] = "detect"
        payload["camera"] = self.server.camera.index
        self._send_json(HTTPStatus.OK, payload)

    def _start_mjpeg_response(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=robot-camera-frame",
        )
        self.end_headers()

    def _write_mjpeg_frame(self, frame: bytes) -> None:
        self.wfile.write(b"--robot-camera-frame\r\n")
        self.wfile.write(b"Content-Type: image/jpeg\r\n")
        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
        self.wfile.write(frame)
        self.wfile.write(b"\r\n")

    def _stream(self) -> None:
        self._start_mjpeg_response()
        delay = 1 / max(self.server.fps, 1)
        while True:
            frame = self.server.camera.frame()
            if frame is None:
                time.sleep(delay)
                continue

            try:
                self._write_mjpeg_frame(frame)
                time.sleep(delay)
            except (BrokenPipeError, ConnectionResetError):
                return

    def _annotated_stream(self) -> None:
        if self.server.detector is None:
            self.send_error(HTTPStatus.BAD_REQUEST, "Detection model is not configured")
            return

        self._start_mjpeg_response()
        delay = 1 / max(self.server.fps, 1)
        while True:
            frame = self.server.camera.frame_image()
            if frame is None:
                time.sleep(delay)
                continue

            try:
                annotated = self.server.detector.annotated_jpeg(
                    frame,
                    quality=self.server.camera.quality,
                )
                self._write_mjpeg_frame(annotated)
                time.sleep(delay)
            except (BrokenPipeError, ConnectionResetError):
                return
            except Exception as exc:
                sys.stderr.write(f"annotated stream error: {exc}\n")
                time.sleep(delay)

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
        detector: YoloFrameDetector | None = None,
    ) -> None:
        super().__init__(server_address, handler_class)
        self.camera = camera
        self.fps = fps
        self.title = title
        self.detector = detector


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
    parser.add_argument(
        "--model",
        default="",
        help="Optional YOLO model path. Enables the /detect API for Telegram.",
    )
    parser.add_argument("--conf", type=float, default=0.4, help="YOLO confidence threshold")
    return parser


def run_camera_stream(
    *,
    camera_index_or_name: int | str = "external",
    host: str = "0.0.0.0",
    port: int = 8080,
    width: int = 1280,
    height: int = 720,
    fps: int = 15,
    quality: int = 80,
    title: str = "Robot Arm Camera",
    model: str = "",
    conf: float = 0.4,
) -> int:
    camera = Camera(
        index=camera_index_or_name,
        width=width,
        height=height,
        fps=fps,
        quality=quality,
    )
    detector = YoloFrameDetector(model, conf) if model else None
    server = CameraServer(
        (host, port),
        StreamHandler,
        camera=camera,
        fps=fps,
        title=title,
        detector=detector,
    )

    def stop(_signum: int, _frame: object) -> None:
        server.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    address = local_ip() if host in {"0.0.0.0", "::"} else host
    print(f"Open this on your phone: http://{address}:{port}")
    if detector is not None:
        print(f"Detection API is enabled for Telegram: http://127.0.0.1:{port}/detect")
        print(f"YOLO overlay stream: http://{address}:{port}/annotated_stream")
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    finally:
        camera.close()
        server.server_close()

    return 0


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

    return run_camera_stream(
        camera_index_or_name=args.camera,
        host=args.host,
        port=args.port,
        width=args.width,
        height=args.height,
        fps=args.fps,
        quality=args.quality,
        title=args.title,
        model=args.model,
        conf=args.conf,
    )


if __name__ == "__main__":
    raise SystemExit(main())
