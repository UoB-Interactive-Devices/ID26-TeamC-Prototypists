from __future__ import annotations

import argparse
import time
from pathlib import Path

try:
    from .camera_stream import configure_capture, open_capture, resolve_camera_index
except ImportError:  # Allows running this file directly: python control/yolo_webcam_detect.py
    from camera_stream import configure_capture, open_capture, resolve_camera_index

try:
    import cv2
except ImportError as exc:
    raise SystemExit("OpenCV is not installed. Run: pip install opencv-python") from exc

try:
    from ultralytics import YOLO
except ImportError as exc:
    raise SystemExit("Ultralytics is not installed. Run: pip install ultralytics") from exc


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run YOLO detection on a webcam.")
    parser.add_argument(
        "--model",
        default="runs/detect/train/weights/best.pt",
        help="Path to trained YOLO model",
    )
    parser.add_argument(
        "--camera",
        default="external",
        help="Camera index, 'external' for non-built-in camera, or 'auto'",
    )
    parser.add_argument("--conf", type=float, default=0.4, help="Confidence threshold")
    parser.add_argument("--width", type=int, default=1280, help="Camera width")
    parser.add_argument("--height", type=int, default=720, help="Camera height")
    parser.add_argument("--fps", type=int, default=30, help="Camera FPS")
    parser.add_argument("--duration", type=float, default=0.0, help="Run time in seconds, 0 for forever")
    parser.add_argument("--once", action="store_true", help="Exit after the first detection")
    parser.add_argument("--no-show", action="store_true", help="Do not open the preview window")
    parser.add_argument("--verbose", action="store_true", help="Print every detection")
    return parser


def draw_center(frame, x1: int, y1: int, x2: int, y2: int) -> tuple[int, int]:
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
    cv2.putText(
        frame,
        f"center=({cx}, {cy})",
        (x1, max(24, y1 - 28)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    return cx, cy


def run_yolo_webcam_detection(
    *,
    model: str,
    camera: int | str = "external",
    conf: float = 0.4,
    width: int = 1280,
    height: int = 720,
    fps: int = 30,
    duration_s: float = 0.0,
    once: bool = False,
    show: bool = True,
    verbose: bool = False,
) -> dict[str, object]:
    model_path = Path(model)
    if not model_path.exists():
        raise SystemExit(f"Model not found: {model_path}")

    camera_index = resolve_camera_index(camera)
    model = YOLO(str(model_path))

    capture = open_capture(camera_index)
    if not capture.isOpened():
        raise SystemExit(f"Could not open camera index {camera_index}")

    configure_capture(capture, width=width, height=height, fps=fps)

    print(f"Using camera {camera_index}")
    if show:
        print("Press q or Esc to quit.")

    best_detection: dict[str, object] | None = None
    detections = 0
    start = time.monotonic()
    try:
        while True:
            if duration_s > 0 and time.monotonic() - start >= duration_s:
                break

            ok, frame = capture.read()
            if not ok:
                print("Could not read frame from camera.")
                break

            results = model(frame, conf=conf, verbose=False)
            annotated = results[0].plot()

            boxes = results[0].boxes
            if boxes is not None and len(boxes) > 0:
                best = max(boxes, key=lambda box: float(box.conf[0]))
                x1, y1, x2, y2 = [int(value) for value in best.xyxy[0].tolist()]
                confidence = float(best.conf[0])
                cx, cy = draw_center(annotated, x1, y1, x2, y2)
                detection = {
                    "label": model.names[int(best.cls[0])],
                    "confidence": round(confidence, 4),
                    "box": [x1, y1, x2, y2],
                    "center": [cx, cy],
                    "camera": camera_index,
                }
                best_detection = detection
                detections += 1
                if verbose or show:
                    print(
                        f"detected {detection['label']} conf={confidence:.2f} "
                        f"box=({x1},{y1},{x2},{y2}) center=({cx},{cy})",
                        end="\r" if show else "\n",
                        flush=True,
                    )
                if once:
                    break

            if show:
                cv2.imshow("YOLO Webcam Detection", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
    finally:
        capture.release()
        if show:
            cv2.destroyAllWindows()
        print()

    return {
        "ok": best_detection is not None,
        "action": "detect",
        "model": str(model_path),
        "camera": camera_index,
        "detections": detections,
        "best": best_detection,
    }


def main() -> int:
    args = build_parser().parse_args()
    result = run_yolo_webcam_detection(
        model=args.model,
        camera=args.camera,
        conf=args.conf,
        width=args.width,
        height=args.height,
        fps=args.fps,
        duration_s=args.duration,
        once=args.once,
        show=not args.no_show,
        verbose=args.verbose,
    )
    print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
