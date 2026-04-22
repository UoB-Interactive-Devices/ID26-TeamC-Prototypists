from __future__ import annotations

import argparse
import json
import sys

from .primitives import ArmState, list_primitives, run_primitive
from .serial_client import RobotArmSerialClient


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot arm primitive calibration CLI")
    parser.add_argument("--port", default="/dev/cu.usbmodem1101", help="Serial port")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="List available primitives")
    subparsers.add_parser("state", help="Read joint state from firmware")
    subparsers.add_parser("home", help="Send HOME")
    subparsers.add_parser("stop", help="Send STOP")

    move_parser = subparsers.add_parser("move", help="Move one joint to an angle")
    move_parser.add_argument("joint")
    move_parser.add_argument("angle", type=int)

    step_parser = subparsers.add_parser("step", help="Move one joint by delta")
    step_parser.add_argument("joint")
    step_parser.add_argument("delta", type=int)

    spin_parser = subparsers.add_parser("spin", help="Set rotate speed")
    spin_parser.add_argument("joint")
    spin_parser.add_argument("speed", type=int)

    stepper_parser = subparsers.add_parser("stepper", help="Move or enable the stepper motor")
    stepper_subparsers = stepper_parser.add_subparsers(dest="stepper_command", required=True)

    stepper_move_parser = stepper_subparsers.add_parser("move", help="Move stepper by signed steps")
    stepper_move_parser.add_argument("steps", type=int)

    stepper_enable_parser = stepper_subparsers.add_parser("enable", help="Enable or disable stepper driver")
    stepper_enable_parser.add_argument("enabled", type=int, choices=(0, 1))

    run_parser = subparsers.add_parser("run", help="Run one named primitive")
    run_parser.add_argument("name")
    run_parser.add_argument("--verbose", action="store_true")

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "list":
        for name in list_primitives():
            print(name)
        return 0

    client = RobotArmSerialClient(port=args.port, baudrate=args.baudrate)
    try:
        if args.command == "state":
            print(json.dumps(client.get_state(), indent=2, sort_keys=True))
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
        if args.command == "step":
            print(client.step(args.joint, args.delta))
            return 0
        if args.command == "spin":
            print(client.spin(args.joint, args.speed))
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
