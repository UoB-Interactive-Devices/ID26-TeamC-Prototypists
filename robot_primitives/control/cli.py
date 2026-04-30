from __future__ import annotations

import argparse
import json
import sys

from .primitives import ArmState, list_primitives, run_primitive
from .serial_client import RobotArmSerialClient, available_serial_ports
from .webcam_control import run_webcam_control


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot arm primitive calibration CLI")
    parser.add_argument("--port", default="auto", help="Serial port, or 'auto'")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="List available primitives")
    subparsers.add_parser("list-ports", help="List available serial ports")
    subparsers.add_parser("state", help="Read joint state from firmware")
    subparsers.add_parser("channels", help="Read firmware joint-to-channel mapping")
    subparsers.add_parser("home", help="Send HOME")
    subparsers.add_parser("stop", help="Send STOP")

    move_parser = subparsers.add_parser("move", help="Move one joint to an angle")
    move_parser.add_argument("joint")
    move_parser.add_argument("angle", type=int)

    move_multi_parser = subparsers.add_parser("move-multi", help="Move several joints together")
    move_multi_parser.add_argument(
        "pairs",
        nargs="+",
        help="Joint/angle pairs, for example: shoulder 50 wrist 130 grip 0",
    )

    step_parser = subparsers.add_parser("step", help="Move one joint by delta")
    step_parser.add_argument("joint")
    step_parser.add_argument("delta", type=int)

    spin_parser = subparsers.add_parser("spin", help="Set rotate speed")
    spin_parser.add_argument("joint")
    spin_parser.add_argument("speed", type=int)

    raw_parser = subparsers.add_parser("raw", help="Move one Adafruit PCA9685 channel directly (0-15)")
    raw_parser.add_argument("channel", type=int)
    raw_parser.add_argument("angle", type=int)

    channel_parser = subparsers.add_parser("channel", help="Remap one joint to an Adafruit PCA9685 channel (0-15) until reset")
    channel_parser.add_argument("joint")
    channel_parser.add_argument("channel", type=int)

    stepper_parser = subparsers.add_parser("stepper", help="Move or enable the stepper motor")
    stepper_subparsers = stepper_parser.add_subparsers(dest="stepper_command", required=True)

    stepper_move_parser = stepper_subparsers.add_parser("move", help="Move stepper by signed steps")
    stepper_move_parser.add_argument("steps", type=int)

    stepper_enable_parser = stepper_subparsers.add_parser("enable", help="Enable or disable stepper driver")
    stepper_enable_parser.add_argument("enabled", type=int, choices=(0, 1))

    run_parser = subparsers.add_parser("run", help="Run one named primitive")
    run_parser.add_argument("name")
    run_parser.add_argument("--verbose", action="store_true")

    webcam_parser = subparsers.add_parser("webcam", help="Run webcam-driven control")
    webcam_parser.add_argument("--camera", default="external", help="Camera index, 'external', or 'auto'")
    webcam_parser.add_argument("--mode", choices=("palette", "track"), default="palette")
    webcam_parser.add_argument(
        "--colors",
        default="blue,green,yellow,red",
        help="Comma-separated colors: blue, green, yellow, red",
    )
    webcam_parser.add_argument("--duration", type=float, default=30.0)
    webcam_parser.add_argument("--width", type=int, default=640)
    webcam_parser.add_argument("--height", type=int, default=480)
    webcam_parser.add_argument("--min-area", type=int, default=900)
    webcam_parser.add_argument("--interval", type=float, default=0.35)
    webcam_parser.add_argument("--trigger-hold", type=float, default=1.0)
    webcam_parser.add_argument("--deadzone", type=float, default=0.18)
    webcam_parser.add_argument("--close-ratio", type=float, default=0.08)
    webcam_parser.add_argument("--dry-run", action="store_true")
    webcam_parser.add_argument("--verbose", action="store_true")

    detect_parser = subparsers.add_parser("detect", help="Run YOLO webcam detection")
    detect_parser.add_argument(
        "--model",
        default="runs/detect/train/weights/best.pt",
        help="Path to trained YOLO model",
    )
    detect_parser.add_argument("--camera", default="external", help="Camera index, 'external', or 'auto'")
    detect_parser.add_argument("--conf", type=float, default=0.4, help="Confidence threshold")
    detect_parser.add_argument("--duration", type=float, default=30.0, help="Run time in seconds")
    detect_parser.add_argument("--once", action="store_true", help="Exit after first detection")
    detect_parser.add_argument("--width", type=int, default=1280)
    detect_parser.add_argument("--height", type=int, default=720)
    detect_parser.add_argument("--fps", type=int, default=30)
    detect_parser.add_argument("--no-show", action="store_true", help="Do not open the preview window")
    detect_parser.add_argument("--verbose", action="store_true")

    stream_parser = subparsers.add_parser(
        "stream",
        help="Serve live webcam video to a phone, with optional on-demand YOLO detection",
    )
    stream_parser.add_argument("--camera", default="external", help="Camera index, 'external', or 'auto'")
    stream_parser.add_argument("--host", default="0.0.0.0")
    stream_parser.add_argument("--port", type=int, default=8080)
    stream_parser.add_argument("--width", type=int, default=1280)
    stream_parser.add_argument("--height", type=int, default=720)
    stream_parser.add_argument("--fps", type=int, default=15)
    stream_parser.add_argument("--quality", type=int, default=80)
    stream_parser.add_argument("--title", default="Robot Arm Camera")
    stream_parser.add_argument(
        "--model",
        default="",
        help="Optional YOLO model path. Enables the /detect API for Telegram.",
    )
    stream_parser.add_argument("--conf", type=float, default=0.4)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "list":
        for name in list_primitives():
            print(name)
        return 0

    if args.command == "list-ports":
        ports = available_serial_ports()
        if not ports:
            print("No serial ports found.")
            return 1
        for port in ports:
            print(port)
        return 0

    if args.command == "webcam":
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
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0

    if args.command == "detect":
        from .yolo_webcam_detect import run_yolo_webcam_detection

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
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0

    if args.command == "stream":
        from .camera_stream import run_camera_stream

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

    client = RobotArmSerialClient(port=args.port, baudrate=args.baudrate)
    try:
        if args.command == "state":
            print(json.dumps(client.get_state(), indent=2, sort_keys=True))
            return 0
        if args.command == "channels":
            print(client.channels())
            return 0
        if args.command == "home":
            print(client.home())
            return 0
        if args.command == "stop":
            print(client.stop())
            return 0
        if args.command == "move":
            print(client.move(args.joint, args.angle))
            return 0
        if args.command == "move-multi":
            if len(args.pairs) % 2 != 0:
                parser.error("move-multi expects joint/angle pairs")
            moves = {
                str(args.pairs[i]): int(args.pairs[i + 1])
                for i in range(0, len(args.pairs), 2)
            }
            print(client.move_multi(moves))
            return 0
        if args.command == "step":
            print(client.step(args.joint, args.delta))
            return 0
        if args.command == "spin":
            print(client.spin(args.joint, args.speed))
            return 0
        if args.command == "raw":
            print(client.raw(args.channel, args.angle))
            return 0
        if args.command == "channel":
            print(client.channel(args.joint, args.channel))
            return 0
        if args.command == "stepper":
            if args.stepper_command == "move":
                print(client.stepper_move(args.steps))
                return 0
            if args.stepper_command == "enable":
                print(client.stepper_enable(bool(args.enabled)))
                return 0
        if args.command == "run":
            start_state = ArmState.from_mapping(client.get_state())
            end_state = run_primitive(args.name, client, state=start_state, verbose=args.verbose)
            print(json.dumps(end_state.as_dict(), indent=2, sort_keys=True))
            return 0
    finally:
        client.close()

    parser.error(f"Unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
