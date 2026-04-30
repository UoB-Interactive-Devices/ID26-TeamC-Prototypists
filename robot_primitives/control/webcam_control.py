from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from typing import Iterable

try:
    import cv2
except ImportError:  # pragma: no cover - exercised by users without OpenCV installed.
    cv2 = None

import numpy as np

from .camera_stream import open_capture, resolve_camera_index
from .primitives import ArmState, run_primitive
from .serial_client import RobotArmSerialClient


COLOR_RANGES: dict[str, tuple[tuple[int, int, int], ...]] = {
    "blue": ((95, 80, 50), (130, 255, 255)),
    "green": ((40, 70, 50), (85, 255, 255)),
    "yellow": ((18, 80, 70), (36, 255, 255)),
    "red": ((0, 90, 60), (10, 255, 255), (170, 90, 60), (180, 255, 255)),
}

PALETTE_ACTIONS = {
    "blue": "pickup",
    "green": "place",
    "yellow": "home",
    "red": "stop",
}


@dataclass(frozen=True)
class Detection:
    color: str
    x: int
    y: int
    width: int
    height: int
    area: float
    frame_width: int
    frame_height: int

    @property
    def center_error_x(self) -> float:
        return (self.x - (self.frame_width / 2)) / max(self.frame_width / 2, 1)

    @property
    def area_ratio(self) -> float:
        return self.area / max(self.frame_width * self.frame_height, 1)


def _mask_for_color(hsv_frame, color: str):
    ranges = COLOR_RANGES[color]
    mask = None
    for index in range(0, len(ranges), 2):
        lower = np.array(ranges[index], dtype=np.uint8)
        upper = np.array(ranges[index + 1], dtype=np.uint8)
        current = cv2.inRange(hsv_frame, lower, upper)
        mask = current if mask is None else cv2.bitwise_or(mask, current)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_largest_color(frame, colors: Iterable[str], min_area: int) -> Detection | None:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_height, frame_width = frame.shape[:2]
    best: Detection | None = None

    for color in colors:
        if color not in COLOR_RANGES:
            raise ValueError(f"Unsupported color: {color}")

        contours, _hierarchy = cv2.findContours(
            _mask_for_color(hsv, color),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            detection = Detection(
                color=color,
                x=x + width // 2,
                y=y + height // 2,
                width=width,
                height=height,
                area=area,
                frame_width=frame_width,
                frame_height=frame_height,
            )
            if best is None or detection.area > best.area:
                best = detection

    return best


def _send_track_step(
    client: RobotArmSerialClient,
    detection: Detection,
    deadzone: float,
    close_ratio: float,
    dry_run: bool,
) -> list[str]:
    commands: list[str] = []
    if detection.center_error_x < -deadzone:
        commands.append("run rotate_left_small")
    elif detection.center_error_x > deadzone:
        commands.append("run rotate_right_small")
    else:
        commands.append("run rotate_stop")

    if detection.area_ratio < close_ratio:
        commands.append("run arm_down_small")
    else:
        commands.append("pickup")

    if dry_run:
        return commands

    state = ArmState.from_mapping(client.get_state())
    for command in commands:
        if command.startswith("run "):
            state = run_primitive(command.split(maxsplit=1)[1], client, state=state)
        elif command == "pickup":
            client.pickup()
    return commands


def _send_palette_action(
    client: RobotArmSerialClient,
    color: str,
    dry_run: bool,
) -> str:
    action = PALETTE_ACTIONS[color]
    if dry_run:
        return action
    if action == "pickup":
        return client.pickup()
    if action == "place":
        return client.place()
    if action == "home":
        return client.home()
    if action == "stop":
        return client.stop()
    raise ValueError(f"Unsupported palette action: {action}")


def run_webcam_control(
    *,
    port: str,
    baudrate: int,
    camera: int | str = "external",
    mode: str = "palette",
    colors: Iterable[str] = ("blue", "green", "yellow", "red"),
    duration_s: float = 30.0,
    width: int = 640,
    height: int = 480,
    min_area: int = 900,
    interval_s: float = 0.35,
    trigger_hold_s: float = 1.0,
    deadzone: float = 0.18,
    close_ratio: float = 0.08,
    dry_run: bool = False,
    verbose: bool = False,
) -> dict[str, object]:
    if cv2 is None:
        raise RuntimeError("OpenCV is not installed. Run: pip install -r requirements.txt")

    selected_colors = tuple(color.strip().lower() for color in colors if color.strip())
    if not selected_colors:
        raise ValueError("At least one color is required")
    for color in selected_colors:
        if color not in COLOR_RANGES:
            raise ValueError(f"Unsupported color: {color}")

    capture = open_capture(resolve_camera_index(camera))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open camera {camera}")

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    client = None if dry_run else RobotArmSerialClient(port=port, baudrate=baudrate)
    start = time.monotonic()
    last_command_at = 0.0
    first_seen: dict[str, float] = {}
    command_log: list[dict[str, object]] = []

    try:
        while time.monotonic() - start < duration_s:
            ok, frame = capture.read()
            if not ok:
                time.sleep(0.05)
                continue

            detection = detect_largest_color(frame, selected_colors, min_area=min_area)
            now = time.monotonic()
            if detection is None:
                first_seen.clear()
                time.sleep(0.04)
                continue

            first_seen.setdefault(detection.color, now)
            if now - first_seen[detection.color] < trigger_hold_s:
                time.sleep(0.04)
                continue
            if now - last_command_at < interval_s:
                time.sleep(0.04)
                continue

            if mode == "palette":
                reply = _send_palette_action(client, detection.color, dry_run=dry_run)  # type: ignore[arg-type]
                entry: dict[str, object] = {
                    "mode": mode,
                    "color": detection.color,
                    "action": PALETTE_ACTIONS[detection.color],
                    "reply": reply,
                }
            elif mode == "track":
                if len(selected_colors) != 1:
                    raise ValueError("track mode expects exactly one color")
                commands = _send_track_step(
                    client,  # type: ignore[arg-type]
                    detection,
                    deadzone=deadzone,
                    close_ratio=close_ratio,
                    dry_run=dry_run,
                )
                entry = {
                    "mode": mode,
                    "color": detection.color,
                    "commands": commands,
                    "center_error_x": round(detection.center_error_x, 3),
                    "area_ratio": round(detection.area_ratio, 3),
                }
            else:
                raise ValueError("mode must be 'palette' or 'track'")

            command_log.append(entry)
            if verbose:
                print(json.dumps(entry, sort_keys=True))
            last_command_at = now
            first_seen.clear()
    finally:
        capture.release()
        if client is not None:
            client.close()

    return {
        "ok": True,
        "action": "webcam_control",
        "mode": mode,
        "colors": selected_colors,
        "commands": command_log,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control OpenClaw from webcam color detections")
    parser.add_argument("--port", default="/dev/cu.usbmodem1101", help="Serial port")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--camera", default="external", help="Camera index, 'external', or 'auto'")
    parser.add_argument("--mode", choices=("palette", "track"), default="palette")
    parser.add_argument(
        "--colors",
        default="blue,green,yellow,red",
        help="Comma-separated colors: blue, green, yellow, red",
    )
    parser.add_argument("--duration", type=float, default=30.0, help="Run time in seconds")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--min-area", type=int, default=900)
    parser.add_argument("--interval", type=float, default=0.35, help="Minimum command gap in seconds")
    parser.add_argument("--trigger-hold", type=float, default=1.0)
    parser.add_argument("--deadzone", type=float, default=0.18)
    parser.add_argument("--close-ratio", type=float, default=0.08)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    result = run_webcam_control(
        port=args.port,
        baudrate=args.baudrate,
        camera=args.camera,
        mode=args.mode,
        colors=args.colors.split(","),
        duration_s=args.duration,
        width=args.width,
        height=args.height,
        min_area=args.min_area,
        interval_s=args.interval,
        trigger_hold_s=args.trigger_hold,
        deadzone=args.deadzone,
        close_ratio=args.close_ratio,
        dry_run=args.dry_run,
        verbose=args.verbose,
    )
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
