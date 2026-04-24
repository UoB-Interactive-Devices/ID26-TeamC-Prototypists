from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from typing import Any

from .kinematics import (
    DEFAULT_SERVO_CALIBRATION,
    IKError,
    solve_inverse_kinematics,
    within_joint_limits,
)
from .primitives import ArmState, list_primitives, run_primitive
from .serial_client import RobotArmSerialClient


def load_env_file(path: str = ".env") -> None:
    if not os.path.exists(path):
        return

    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            raw = line.strip()
            if not raw or raw.startswith("#") or "=" not in raw:
                continue
            key, value = raw.split("=", 1)
            key = key.strip()
            value = value.strip()
            if key and key not in os.environ:
                os.environ[key] = value


def normalize_serial_port(raw: str) -> str:
    raw = raw.strip()
    if not raw:
        return raw
    first = raw.split()[0]
    if first.startswith("/dev/"):
        return first
    return raw


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def normalize_joint_name(raw: str) -> str:
    joint = raw.strip().lower().replace(" ", "_")
    if joint == "wrist_rotate":
        return "wrist_rotation"
    return joint


@dataclass(frozen=True)
class Config:
    serial_port: str
    baudrate: int

    @staticmethod
    def from_env() -> "Config":
        serial_port = normalize_serial_port(os.environ.get("ROBOT_ARM_SERIAL_PORT", ""))
        if not serial_port:
            raise ValueError("ROBOT_ARM_SERIAL_PORT is missing")

        baudrate = int(os.environ.get("ROBOT_ARM_BAUDRATE", "115200"))
        return Config(serial_port=serial_port, baudrate=baudrate)


class AgentRobotController:
    def __init__(self, port: str, baudrate: int) -> None:
        self.client = RobotArmSerialClient(port=port, baudrate=baudrate)

    def close(self) -> None:
        self.client.close()

    def move_to_xyz(self, args: dict[str, Any]) -> dict[str, Any]:
        x = float(args["x"])
        y = float(args["y"])
        z = float(args["z"])
        wrist_pitch = float(args.get("wrist_pitch", 0.0))
        elbow_up = bool(args.get("elbow_up", False))

        solution = solve_inverse_kinematics(
            x,
            y,
            z,
            wrist_pitch_deg=wrist_pitch,
            elbow_up=elbow_up,
        )
        servo_angles = solution.servo_angles(DEFAULT_SERVO_CALIBRATION)
        ok, reason = within_joint_limits(servo_angles)
        if not ok:
            raise IKError(reason or "Servo angles exceed configured limits")

        result: dict[str, Any] = {
            "ok": True,
            "action": "move_to",
            "target_mm": {"x": x, "y": y, "z": z},
            "wrist_pitch": wrist_pitch,
            "elbow_up": elbow_up,
            "ik_degrees": {
                "rotate": round(solution.rotate, 2),
                "shoulder": round(solution.shoulder, 2),
                "elbow": round(solution.elbow, 2),
                "wrist": round(solution.wrist, 2),
            },
            "servo_angles": servo_angles,
        }

        if bool(args.get("dry_run", False)):
            result["dry_run"] = True
            return result

        result["reply"] = {
            "rotate": self.client.move("rotate", servo_angles["rotate"]),
            "shoulder": self.client.move("shoulder", servo_angles["shoulder"]),
            "elbow": self.client.move("elbow", servo_angles["elbow"]),
            "wrist": self.client.move("wrist", servo_angles["wrist"]),
        }
        return result

    def pick_object(self, args: dict[str, Any]) -> dict[str, Any]:
        color = str(args.get("color", "")).strip().lower()
        if color == "blue":
            return {
                "ok": True,
                "action": "pick_object",
                "color": "blue",
                "mode": "pixy2_signature_1_auto_blue",
                "reply": self.client.auto_blue_on(),
            }

        raise NotImplementedError(
            "pick_object currently supports only blue via Pixy2 signature 1. "
            "Train/add more Pixy signatures before using other colors."
        )

    def move_object_to(self, args: dict[str, Any]) -> dict[str, Any]:
        object_color = str(args.get("object_color", "")).strip().lower()
        destination_color = str(args.get("destination_color", "")).strip().lower()
        destination_type = str(args.get("destination_type", "target")).strip().lower()
        raise NotImplementedError(
            "move_object_to needs camera-to-robot coordinate calibration plus "
            f"place-at-target logic. Requested: {object_color} object to "
            f"{destination_color} {destination_type}."
        )

    def dispatch(self, action: str, args: dict[str, Any]) -> dict[str, Any]:
        action = action.strip().lower()

        if action == "list":
            return {"ok": True, "action": action, "primitives": list_primitives()}
        if action == "ping":
            return {"ok": True, "action": action, "reply": self.client.ping()}
        if action == "i2c_scan":
            return {"ok": True, "action": action, "reply": self.client.i2c_scan()}
        if action == "i2c_probe":
            return {"ok": True, "action": action, "reply": self.client.i2c_probe()}
        if action == "servo_test":
            joint = normalize_joint_name(str(args["joint"]))
            return {
                "ok": True,
                "action": action,
                "joint": joint,
                "reply": self.client.servo_test(joint),
            }
        if action == "state":
            return {"ok": True, "action": action, "state": self.client.get_state()}
        if action == "home":
            return {"ok": True, "action": action, "reply": self.client.home()}
        if action == "stop":
            return {"ok": True, "action": action, "reply": self.client.stop()}
        if action == "pickup":
            return {"ok": True, "action": action, "reply": self.client.pickup()}
        if action == "place":
            return {"ok": True, "action": action, "reply": self.client.place()}
        if action == "auto_blue_on":
            return {"ok": True, "action": action, "reply": self.client.auto_blue_on()}
        if action == "auto_blue_off":
            return {"ok": True, "action": action, "reply": self.client.auto_blue_off()}
        if action == "auto_blue_status":
            return {"ok": True, "action": action, "reply": self.client.auto_blue_status()}
        if action == "blue_nod_on":
            return {"ok": True, "action": action, "reply": self.client.blue_nod_on()}
        if action == "blue_nod_off":
            return {"ok": True, "action": action, "reply": self.client.blue_nod_off()}
        if action == "move_to":
            return self.move_to_xyz(args)
        if action == "pick_object":
            return self.pick_object(args)
        if action == "move_object_to":
            return self.move_object_to(args)
        if action == "move":
            joint = normalize_joint_name(str(args["joint"]))
            angle = int(args["angle"])
            return {
                "ok": True,
                "action": action,
                "joint": joint,
                "angle": angle,
                "reply": self.client.move(joint, angle),
            }
        if action == "step":
            joint = normalize_joint_name(str(args["joint"]))
            delta = int(args["delta"])
            return {
                "ok": True,
                "action": action,
                "joint": joint,
                "delta": delta,
                "reply": self.client.step(joint, delta),
            }
        if action == "spin":
            joint = str(args.get("joint", "rotate")).strip().lower()
            speed = int(args["speed"])
            return {
                "ok": True,
                "action": action,
                "joint": joint,
                "speed": speed,
                "reply": self.client.spin(joint, speed),
            }
        if action == "stepper_move":
            steps = int(args["steps"])
            return {
                "ok": True,
                "action": action,
                "steps": steps,
                "reply": self.client.stepper_move(steps),
            }
        if action == "stepper_enable":
            enabled = as_bool(args["enabled"])
            return {
                "ok": True,
                "action": action,
                "enabled": enabled,
                "reply": self.client.stepper_enable(enabled),
            }
        if action == "run":
            primitive = str(args["name"]).strip()
            start_state = ArmState.from_mapping(self.client.get_state())
            end_state = run_primitive(primitive, self.client, state=start_state, verbose=False)
            return {
                "ok": True,
                "action": action,
                "name": primitive,
                "state": end_state.as_dict(),
            }

        raise ValueError(f"Unsupported action: {action}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Agent-friendly JSON API for robot_primitives"
    )
    parser.add_argument(
        "--input",
        help="JSON payload like '{\"action\":\"state\"}'. If omitted, stdin is used.",
    )
    parser.add_argument("--port", help="Override serial port from .env")
    parser.add_argument("--baudrate", type=int, help="Override baudrate from .env")
    return parser


def load_payload(raw: str) -> dict[str, Any]:
    payload = json.loads(raw)
    if not isinstance(payload, dict):
        raise ValueError("Input JSON must be an object")
    if "action" not in payload:
        raise ValueError("Input JSON must include 'action'")
    return payload


def main() -> int:
    load_env_file()
    parser = build_parser()
    args = parser.parse_args()

    raw = args.input if args.input is not None else sys.stdin.read().strip()
    if not raw:
        parser.error("Provide JSON via --input or stdin.")

    try:
        payload = load_payload(raw)
        cfg = Config.from_env()
        port = normalize_serial_port(args.port) if args.port else cfg.serial_port
        baudrate = args.baudrate if args.baudrate else cfg.baudrate

        controller = AgentRobotController(port=port, baudrate=baudrate)
        try:
            result = controller.dispatch(str(payload["action"]), payload)
        finally:
            controller.close()
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, sort_keys=True))
        return 1

    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
