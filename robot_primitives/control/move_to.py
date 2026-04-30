from __future__ import annotations

import argparse
import json
import sys

from .kinematics import (
    DEFAULT_SERVO_CALIBRATION,
    IKError,
    solve_inverse_kinematics,
    within_joint_limits,
)
from .serial_client import RobotArmSerialClient


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Move robot arm gripper tip to an XYZ coordinate")
    parser.add_argument("x", type=float, help="Target x in millimeters")
    parser.add_argument("y", type=float, help="Target y in millimeters")
    parser.add_argument("z", type=float, help="Target z in millimeters")
    parser.add_argument("--port", default="/dev/cu.usbmodem1101", help="Serial port")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument(
        "--wrist-pitch",
        type=float,
        default=0.0,
        help="Desired gripper pitch in degrees within the radial plane",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only print the solved servo angles without sending them",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        solution = solve_inverse_kinematics(
            args.x,
            args.y,
            args.z,
            wrist_pitch_deg=args.wrist_pitch,
            elbow_up=False,
        )
        servo_angles = solution.servo_angles(DEFAULT_SERVO_CALIBRATION)
        ok, reason = within_joint_limits(servo_angles)
        if not ok:
            raise IKError(reason or "Servo angles exceed configured limits")
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, sort_keys=True))
        return 1

    payload = {
        "ok": True,
        "target_mm": {"x": args.x, "y": args.y, "z": args.z},
        "ik_degrees": {
            "rotate": round(solution.rotate, 2),
            "shoulder": round(solution.shoulder, 2),
            "wrist": round(solution.wrist, 2),
        },
        "servo_angles": {
            joint: angle
            for joint, angle in servo_angles.items()
            if joint != "elbow"
        },
        "note": "Elbow servo is disabled/removed; move_to sends rotate, shoulder, and wrist only.",
    }

    if args.dry_run:
        print(json.dumps(payload, sort_keys=True))
        return 0

    client = RobotArmSerialClient(port=args.port, baudrate=args.baudrate)
    try:
        # Send base rotation first, then the planar joints.
        payload["reply"] = {
            "rotate": client.move("rotate", servo_angles["rotate"]),
            "shoulder": client.move("shoulder", servo_angles["shoulder"]),
            "wrist": client.move("wrist", servo_angles["wrist"]),
        }
    finally:
        client.close()

    print(json.dumps(payload, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
