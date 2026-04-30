from __future__ import annotations

import asyncio
import json
import logging
import os
import re
import shlex
import subprocess
import urllib.error
import urllib.request
from dataclasses import dataclass

from telegram import Update
from telegram.ext import Application, CommandHandler, ContextTypes, MessageHandler, filters

from .kinematics import (
    DEFAULT_SERVO_CALIBRATION,
    IKError,
    solve_inverse_kinematics,
    within_joint_limits,
)
from .primitives import ArmState, list_primitives, run_primitive
from .serial_client import RobotArmSerialClient


logging.basicConfig(
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    level=logging.INFO,
)
logger = logging.getLogger("robot-primitives-telegram")

JOINT_WORDS = {"shoulder", "wrist", "wrist_rotation", "grip", "rotate"}
JOINT_PATTERN = r"shoulder|wrist rotation|wrist rotate|wrist_rotation|wrist_rotate|wrist|grip|rotate"
LLM_ACTIONS = {
    "help",
    "detect",
    "state",
    "home",
    "stop",
    "pickup",
    "pickup_1",
    "pickup_2",
    "place",
    "place_1",
    "place_2",
    "auto_blue_on",
    "auto_blue_off",
    "auto_blue_status",
    "blue_nod_on",
    "blue_nod_off",
    "move_to",
    "pick_object",
    "move_object_to",
    "run",
    "move",
    "step",
}


@dataclass(frozen=True)
class NaturalLanguageAction:
    action: str
    args: tuple[str | int, ...] = ()
    description: str = ""


@dataclass(frozen=True)
class NaturalLanguagePlan:
    actions: tuple[NaturalLanguageAction, ...]

    @property
    def description(self) -> str:
        return " -> ".join(action.description or action.action for action in self.actions)


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


@dataclass(frozen=True)
class Config:
    telegram_bot_token: str
    allowed_chat_id: int
    serial_port: str
    baudrate: int
    detect_url: str

    @staticmethod
    def from_env() -> "Config":
        token = os.environ.get("TELEGRAM_BOT_TOKEN", "").strip()
        if not token:
            raise ValueError("TELEGRAM_BOT_TOKEN is missing")

        chat_id_raw = os.environ.get("TELEGRAM_ALLOWED_CHAT_ID", "").strip()
        if not chat_id_raw:
            raise ValueError("TELEGRAM_ALLOWED_CHAT_ID is missing")

        serial_port = normalize_serial_port(os.environ.get("ROBOT_ARM_SERIAL_PORT", ""))
        if not serial_port:
            raise ValueError("ROBOT_ARM_SERIAL_PORT is missing")

        baudrate = int(os.environ.get("ROBOT_ARM_BAUDRATE", "115200"))
        detect_url = os.environ.get("ROBOT_ARM_DETECT_URL", "http://127.0.0.1:8080/detect").strip()

        return Config(
            telegram_bot_token=token,
            allowed_chat_id=int(chat_id_raw),
            serial_port=serial_port,
            baudrate=baudrate,
            detect_url=detect_url,
        )


class PrimitiveRobotController:
    def __init__(self, port: str, baudrate: int) -> None:
        self.client = RobotArmSerialClient(port=port, baudrate=baudrate)

    def close(self) -> None:
        self.client.close()

    def list_primitives_text(self) -> str:
        return "\n".join(list_primitives())

    def state_text(self) -> str:
        payload = self.client.get_state()
        return json.dumps(payload, indent=2, sort_keys=True)

    def detect_text(self, detect_url: str) -> str:
        request = urllib.request.Request(detect_url, method="POST")
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except urllib.error.URLError as exc:
            raise RuntimeError(
                "Detection server is not reachable. Start the stream server with --model first."
            ) from exc

        if not payload.get("ok"):
            error = payload.get("error")
            if error:
                return f"No detection: {error}"
            return "No object detected."

        best = payload.get("best") or {}
        label = best.get("label", "object")
        confidence = float(best.get("confidence", 0.0))
        center = best.get("center", ["?", "?"])
        box = best.get("box", ["?", "?", "?", "?"])
        return (
            f"Detected {label} ({confidence * 100:.1f}%)\n"
            f"center: ({center[0]}, {center[1]})\n"
            f"box: ({box[0]}, {box[1]}, {box[2]}, {box[3]})"
        )

    def home(self) -> str:
        return self.client.home()

    def stop(self) -> str:
        return self.client.stop()

    def pickup(self) -> str:
        return self.client.pickup()

    def pickup_1(self) -> str:
        return self.client.pickup_1()

    def pickup_2(self) -> str:
        return self.client.pickup_2()

    def place(self) -> str:
        return self.client.place()

    def place_1(self) -> str:
        return self.client.place_1()

    def place_2(self) -> str:
        return self.client.place_2()

    def auto_blue_on(self) -> str:
        return self.client.auto_blue_on()

    def auto_blue_off(self) -> str:
        return self.client.auto_blue_off()

    def auto_blue_status(self) -> str:
        return self.client.auto_blue_status()

    def blue_nod_on(self) -> str:
        return self.client.blue_nod_on()

    def blue_nod_off(self) -> str:
        return self.client.blue_nod_off()

    def move(self, joint: str, angle: int) -> str:
        return self.client.move(normalize_joint(joint), angle)

    def step(self, joint: str, delta: int) -> str:
        return self.client.step(normalize_joint(joint), delta)

    def spin(self, joint: str, speed: int) -> str:
        return self.client.spin(joint, speed)

    def move_to_xyz(
        self,
        x: float,
        y: float,
        z: float,
        wrist_pitch: float = 0.0,
    ) -> str:
        solution = solve_inverse_kinematics(
            x,
            y,
            z,
            wrist_pitch_deg=wrist_pitch,
            elbow_up=False,
        )
        servo_angles = solution.servo_angles(DEFAULT_SERVO_CALIBRATION)
        ok, reason = within_joint_limits(servo_angles)
        if not ok:
            raise IKError(reason or "Servo angles exceed configured limits")

        replies = {
            "rotate": self.client.move("rotate", servo_angles["rotate"]),
            "shoulder": self.client.move("shoulder", servo_angles["shoulder"]),
            "wrist": self.client.move("wrist", servo_angles["wrist"]),
        }
        return json.dumps(
            {
                "target_mm": {"x": x, "y": y, "z": z},
                "servo_angles": {
                    joint: angle
                    for joint, angle in servo_angles.items()
                    if joint != "elbow"
                },
                "note": "Elbow servo is disabled/removed; move_to sends rotate, shoulder, and wrist only.",
                "reply": replies,
            },
            indent=2,
            sort_keys=True,
        )

    def pick_object(self, color: str) -> str:
        color = color.strip().lower()
        if color == "blue":
            return self.client.auto_blue_on()
        raise NotImplementedError(
            "Only blue object pickup is wired right now via Pixy2 signature 1. "
            "Other colors need Pixy signature + coordinate calibration first."
        )

    def move_object_to(
        self,
        object_color: str,
        destination_color: str,
        destination_type: str = "target",
    ) -> str:
        raise NotImplementedError(
            "Moving one detected object to another detected target needs "
            "camera-to-robot coordinate calibration and place-at-target logic first. "
            f"Requested: {object_color} object to {destination_color} {destination_type}."
        )

    def run(self, primitive_name: str, verbose: bool = False) -> str:
        state = ArmState.from_mapping(self.client.get_state())
        end_state = run_primitive(primitive_name, self.client, state=state, verbose=verbose)
        return json.dumps(end_state.as_dict(), indent=2, sort_keys=True)


def normalize_text(text: str) -> str:
    normalized = text.strip().lower()
    normalized = re.sub(r"_+", " ", normalized)
    normalized = re.sub(r"\s+", " ", normalized)
    normalized = re.sub(r"^(?:please|do|can you|could you|would you)\s+", "", normalized)
    return normalized


def normalize_joint(raw: str) -> str:
    joint = raw.strip().lower().replace(" ", "_")
    if joint == "wrist_rotate":
        return "wrist_rotation"
    return joint


def parse_single_natural_language(text: str) -> NaturalLanguageAction | None:
    text = normalize_text(text)

    if text in {"help", "commands", "what can you do"}:
        return NaturalLanguageAction("help", description="show help")
    if text in {"detect", "detection", "find object", "find cat"}:
        return NaturalLanguageAction("detect", description="detect object")
    if text in {"state", "status"}:
        return NaturalLanguageAction("state", description="read state")

    move_to_match = re.search(
        r"(?:move to|go to)\s*x\s*(-?\d+(?:\.\d+)?)\s*y\s*(-?\d+(?:\.\d+)?)\s*z\s*(-?\d+(?:\.\d+)?)",
        text,
    )
    if move_to_match:
        return NaturalLanguageAction(
            "move_to",
            (
                float(move_to_match.group(1)),
                float(move_to_match.group(2)),
                float(move_to_match.group(3)),
                0.0,
                False,
            ),
            "move to coordinate",
        )

    blue_words = ("blue",)
    red_words = ("red",)
    if any(word in text for word in blue_words) and any(word in text for word in red_words):
        if "flag" in text and "move" in text:
            return NaturalLanguageAction(
                "move_object_to",
                ("blue", "red", "flag"),
                "move blue object to red flag",
            )

    if any(word in text for word in blue_words):
        if any(phrase in text for phrase in ("nod", "detect", "find")):
            if any(phrase in text for phrase in ("off", "stop")):
                return NaturalLanguageAction("blue_nod_off", description="turn blue nod off")
            return NaturalLanguageAction("blue_nod_on", description="turn blue nod on")
        if any(phrase in text for phrase in ("auto", "pick", "grab")):
            if any(phrase in text for phrase in ("off", "stop")):
                return NaturalLanguageAction("auto_blue_off", description="turn auto blue off")
            return NaturalLanguageAction("auto_blue_on", description="turn auto blue on")
        if "status" in text:
            return NaturalLanguageAction("auto_blue_status", description="read auto blue status")

    if any(phrase in text for phrase in ("go home", "home")):
        return NaturalLanguageAction("home", description="go home")
    if any(phrase in text for phrase in ("stop", "emergency")):
        return NaturalLanguageAction("stop", description="stop")
    if any(phrase in text for phrase in ("pick up 1", "pickup 1", "pick up one", "pickup one")):
        return NaturalLanguageAction("pickup_1", description="pickup_1")
    if any(phrase in text for phrase in ("pick up 2", "pickup 2", "pick up two", "pickup two")):
        return NaturalLanguageAction("pickup_2", description="pickup_2")
    if any(phrase in text for phrase in ("place 1", "drop 1", "release 1", "place one")):
        return NaturalLanguageAction("place_1", description="place_1")
    if any(phrase in text for phrase in ("place 2", "drop 2", "release 2", "place two")):
        return NaturalLanguageAction("place_2", description="place_2")
    if any(phrase in text for phrase in ("pick up", "pickup", "grab")):
        return NaturalLanguageAction("pickup", description="pickup")
    if any(phrase in text for phrase in ("place", "drop", "release")):
        return NaturalLanguageAction("place", description="place")

    run_match = re.search(r"(?:run|primitive)\s+([a-zA-Z0-9_]+)", text)
    if run_match:
        return NaturalLanguageAction("run", (run_match.group(1),), "run primitive")

    move_match = re.search(
        rf"(?:move|set)\s+({JOINT_PATTERN})\s+(?:to\s+)?(-?\d+)",
        text,
    )
    if move_match:
        joint = normalize_joint(move_match.group(1))
        angle = int(move_match.group(2))
        return NaturalLanguageAction("move", (joint, angle), "move joint")

    step_match = re.search(
        rf"(?:step|nudge)\s+({JOINT_PATTERN})\s+(-?\d+)",
        text,
    )
    if step_match:
        joint = normalize_joint(step_match.group(1))
        delta = int(step_match.group(2))
        return NaturalLanguageAction("step", (joint, delta), "step joint")

    simple_move_match = re.search(rf"\b({JOINT_PATTERN})\s+(-?\d+)\b", text)
    if simple_move_match:
        joint = normalize_joint(simple_move_match.group(1))
        angle = int(simple_move_match.group(2))
        if joint in JOINT_WORDS:
            return NaturalLanguageAction("move", (joint, angle), "move joint")

    if text in list_primitives():
        return NaturalLanguageAction("run", (text,), "run primitive")

    return None


def parse_natural_language(text: str) -> NaturalLanguagePlan | None:
    normalized = normalize_text(text)
    single_action = parse_single_natural_language(normalized)

    split_candidates = re.split(
        r"\s*(?:,|;|\band then\b|\bthen\b|\bafter that\b|\bnext\b|\band\b)\s*",
        normalized,
    )
    split_candidates = [part.strip() for part in split_candidates if part.strip()]

    if len(split_candidates) > 1:
        actions: list[NaturalLanguageAction] = []
        for part in split_candidates:
            parsed = parse_single_natural_language(part)
            if parsed is None:
                actions = []
                break
            actions.append(parsed)
        if actions:
            return NaturalLanguagePlan(tuple(actions))

    if single_action is None:
        return None
    return NaturalLanguagePlan((single_action,))


def build_openclaw_prompt(text: str) -> str:
    primitives = ", ".join(list_primitives())
    return (
        "You are translating a Telegram natural-language robot-arm command into a safe JSON action.\n"
        "Return ONLY one JSON object. Do not include Markdown or explanation.\n\n"
        "Allowed JSON forms:\n"
        '{"action":"help"}\n'
        '{"action":"detect"}\n'
        '{"action":"state"}\n'
        '{"action":"home"}\n'
        '{"action":"stop"}\n'
        '{"action":"pickup"}\n'
        '{"action":"pickup_1"}\n'
        '{"action":"pickup_2"}\n'
        '{"action":"place"}\n'
        '{"action":"place_1"}\n'
        '{"action":"place_2"}\n'
        '{"action":"auto_blue_on"}\n'
        '{"action":"auto_blue_off"}\n'
        '{"action":"auto_blue_status"}\n'
        '{"action":"blue_nod_on"}\n'
        '{"action":"blue_nod_off"}\n'
        '{"action":"move_to","x":530,"y":0,"z":240,"wrist_pitch":20}\n'
        '{"action":"pick_object","color":"blue"}\n'
        '{"action":"move_object_to","object_color":"blue","destination_color":"red","destination_type":"flag"}\n'
        '{"action":"run","name":"ready"}\n'
        '{"action":"move","joint":"shoulder","angle":95}\n'
        '{"action":"move","joint":"wrist_rotation","angle":95}\n'
        '{"action":"step","joint":"wrist","delta":-4}\n\n'
        "Allowed joints: shoulder, wrist, wrist_rotation, grip, rotate. Elbow is not available.\n"
        f"Allowed primitives: {primitives}.\n"
        "Choose exactly one action for each user message.\n"
        "For commands like 'pick up the blue object', use pick_object with color blue.\n"
        "For commands like 'move the blue object to the red flag', use move_object_to.\n"
        "If the user asks for multiple actions in one sentence, such as 'pick up and place', "
        'return {"action":"help"} instead of guessing one step.\n'
        "Use move_to only when the user gives explicit x/y/z millimeter coordinates.\n"
        "Use detect when the user asks to run object detection once.\n"
        "Prefer high-level safe actions like home, pickup, place, pick_object, blue_nod_on, auto_blue_on, or run.\n"
        "Use move/step only when the user explicitly gives a joint and numeric value.\n"
        "If the command is unsafe or unclear, return {\"action\":\"help\"}.\n\n"
        f"User command: {text}\n"
    )


def extract_json_object(text: str) -> dict:
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end < start:
        raise ValueError("OpenClaw output did not contain a JSON object")
    payload = json.loads(text[start : end + 1])
    if not isinstance(payload, dict):
        raise ValueError("OpenClaw JSON output must be an object")
    return payload


def action_from_payload(payload: dict) -> NaturalLanguageAction:
    action = str(payload.get("action", "")).strip().lower()
    if action not in LLM_ACTIONS:
        raise ValueError(f"OpenClaw returned unsupported action: {action}")

    if action == "run":
        name = str(payload.get("name", "")).strip()
        if not name:
            raise ValueError("OpenClaw run action is missing name")
        if name not in list_primitives():
            raise ValueError(f"OpenClaw returned unknown primitive: {name}")
        return NaturalLanguageAction("run", (name,), f"OpenClaw chose run {name}")

    if action == "move":
        joint = normalize_joint(str(payload.get("joint", "")))
        if joint not in JOINT_WORDS:
            raise ValueError(f"OpenClaw returned unknown joint: {joint}")
        angle = int(payload["angle"])
        return NaturalLanguageAction("move", (joint, angle), f"OpenClaw chose move {joint} {angle}")

    if action == "step":
        joint = normalize_joint(str(payload.get("joint", "")))
        if joint not in JOINT_WORDS:
            raise ValueError(f"OpenClaw returned unknown joint: {joint}")
        delta = int(payload["delta"])
        return NaturalLanguageAction("step", (joint, delta), f"OpenClaw chose step {joint} {delta}")

    if action == "move_to":
        x = float(payload["x"])
        y = float(payload["y"])
        z = float(payload["z"])
        wrist_pitch = float(payload.get("wrist_pitch", 0.0))
        return NaturalLanguageAction(
            "move_to",
            (x, y, z, wrist_pitch),
            f"OpenClaw chose move_to x={x} y={y} z={z}",
        )

    if action == "pick_object":
        color = str(payload.get("color", "")).strip().lower()
        if not color:
            raise ValueError("OpenClaw pick_object action is missing color")
        return NaturalLanguageAction(
            "pick_object",
            (color,),
            f"OpenClaw chose pick_object {color}",
        )

    if action == "move_object_to":
        object_color = str(payload.get("object_color", "")).strip().lower()
        destination_color = str(payload.get("destination_color", "")).strip().lower()
        destination_type = str(payload.get("destination_type", "target")).strip().lower()
        if not object_color or not destination_color:
            raise ValueError("OpenClaw move_object_to action needs object_color and destination_color")
        return NaturalLanguageAction(
            "move_object_to",
            (object_color, destination_color, destination_type),
            f"OpenClaw chose move_object_to {object_color} to {destination_color} {destination_type}",
        )

    descriptions = {
        "help": "OpenClaw chose help",
        "detect": "OpenClaw chose detect",
        "state": "OpenClaw chose state",
        "home": "OpenClaw chose home",
        "stop": "OpenClaw chose stop",
        "pickup": "OpenClaw chose pickup",
        "pickup_1": "OpenClaw chose pickup_1",
        "pickup_2": "OpenClaw chose pickup_2",
        "place": "OpenClaw chose place",
        "place_1": "OpenClaw chose place_1",
        "place_2": "OpenClaw chose place_2",
        "auto_blue_on": "OpenClaw chose auto blue on",
        "auto_blue_off": "OpenClaw chose auto blue off",
        "auto_blue_status": "OpenClaw chose auto blue status",
        "blue_nod_on": "OpenClaw chose blue nod on",
        "blue_nod_off": "OpenClaw chose blue nod off",
        "move_to": "OpenClaw chose move to",
        "pick_object": "OpenClaw chose pick object",
        "move_object_to": "OpenClaw chose move object to target",
    }
    return NaturalLanguageAction(action, description=descriptions[action])


def interpret_with_openclaw(text: str) -> NaturalLanguagePlan:
    command = os.environ.get("OPENCLAW_INTENT_COMMAND", "").strip()
    if not command:
        raise ValueError("OPENCLAW_INTENT_COMMAND is not set")

    timeout = float(os.environ.get("OPENCLAW_INTENT_TIMEOUT", "20"))
    completed = subprocess.run(
        shlex.split(command),
        input=build_openclaw_prompt(text),
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )
    if completed.returncode != 0:
        stderr = completed.stderr.strip()
        raise ValueError(f"OpenClaw intent command failed: {stderr or completed.returncode}")

    return NaturalLanguagePlan((action_from_payload(extract_json_object(completed.stdout)),))


def interpret_natural_language(text: str) -> NaturalLanguagePlan | None:
    mode = os.environ.get("ROBOT_ARM_NL_INTERPRETER", "rules").strip().lower()
    rule_action = parse_natural_language(text)
    if rule_action is not None:
        return rule_action

    if mode in {"openclaw", "llm"}:
        try:
            return interpret_with_openclaw(text)
        except Exception as exc:
            logger.warning("OpenClaw intent interpreter failed; using rules fallback: %s", exc)

    return None


async def execute_natural_language_action(
    action: NaturalLanguageAction,
    controller: PrimitiveRobotController,
    detect_url: str,
) -> str:
    if action.action == "help":
        return help_text()
    if action.action == "detect":
        return await asyncio.to_thread(controller.detect_text, detect_url)
    if action.action == "state":
        return await asyncio.to_thread(controller.state_text)
    if action.action == "home":
        return await asyncio.to_thread(controller.home)
    if action.action == "stop":
        return await asyncio.to_thread(controller.stop)
    if action.action == "pickup":
        return await asyncio.to_thread(controller.pickup)
    if action.action == "pickup_1":
        return await asyncio.to_thread(controller.pickup_1)
    if action.action == "pickup_2":
        return await asyncio.to_thread(controller.pickup_2)
    if action.action == "place":
        return await asyncio.to_thread(controller.place)
    if action.action == "place_1":
        return await asyncio.to_thread(controller.place_1)
    if action.action == "place_2":
        return await asyncio.to_thread(controller.place_2)
    if action.action == "auto_blue_on":
        return await asyncio.to_thread(controller.auto_blue_on)
    if action.action == "auto_blue_off":
        return await asyncio.to_thread(controller.auto_blue_off)
    if action.action == "auto_blue_status":
        return await asyncio.to_thread(controller.auto_blue_status)
    if action.action == "blue_nod_on":
        return await asyncio.to_thread(controller.blue_nod_on)
    if action.action == "blue_nod_off":
        return await asyncio.to_thread(controller.blue_nod_off)
    if action.action == "run":
        return await asyncio.to_thread(controller.run, str(action.args[0]), False)
    if action.action == "move":
        return await asyncio.to_thread(
            controller.move,
            str(action.args[0]),
            int(action.args[1]),
        )
    if action.action == "step":
        return await asyncio.to_thread(
            controller.step,
            str(action.args[0]),
            int(action.args[1]),
        )
    if action.action == "move_to":
        return await asyncio.to_thread(
            controller.move_to_xyz,
            float(action.args[0]),
            float(action.args[1]),
            float(action.args[2]),
            float(action.args[3]),
        )
    if action.action == "pick_object":
        return await asyncio.to_thread(controller.pick_object, str(action.args[0]))
    if action.action == "move_object_to":
        return await asyncio.to_thread(
            controller.move_object_to,
            str(action.args[0]),
            str(action.args[1]),
            str(action.args[2]),
        )
    raise ValueError(f"Unsupported natural-language action: {action.action}")


async def execute_natural_language_plan(
    plan: NaturalLanguagePlan,
    controller: PrimitiveRobotController,
    detect_url: str,
) -> str:
    results: list[str] = []
    for action in plan.actions:
        result = await execute_natural_language_action(action, controller, detect_url)
        results.append(f"{action.description or action.action}: {result}")
    return "\n".join(results)


def help_text() -> str:
    return (
        "Robot arm primitive bot\n\n"
        "Slash commands:\n"
        "/detect\n"
        "/list\n"
        "/state\n"
        "/home\n"
        "/stop\n"
        "/pickup\n"
        "/run pickup_1\n"
        "/run pickup_2\n"
        "/place\n"
        "/run place_1\n"
        "/run place_2\n"
        "/auto_blue_on\n"
        "/auto_blue_off\n"
        "/auto_blue_status\n"
        "/blue_nod_on\n"
        "/blue_nod_off\n"
        "/move_to <x_mm> <y_mm> <z_mm> [wrist_pitch]\n"
        "/run <primitive>\n"
        "/move <joint> <angle>\n"
        "/step <joint> <delta>\n"
        "/spin rotate <speed>\n\n"
        "Natural language examples:\n"
        "detect\n"
        "go home\n"
        "pick up\n"
        "pick up 1\n"
        "pick up 2\n"
        "place it\n"
        "place 1\n"
        "place 2\n"
        "pick up 1 and place 2\n"
        "find blue and nod\n"
        "stop blue nod\n"
        "auto pick blue\n"
        "pick up the blue object\n"
        "move the blue object to the red flag\n"
        "move to x 530 y 0 z 240\n"
        "move shoulder to 95\n"
        "move wrist rotation to 95\n"
        "step wrist -4"
    )


def is_authorized(update: Update, allowed_chat_id: int) -> bool:
    if not update.effective_chat:
        return False
    return update.effective_chat.id == allowed_chat_id


def fast_ack_enabled() -> bool:
    raw = os.environ.get("ROBOT_ARM_FAST_ACK", "1")
    return raw.strip().lower() in {"1", "true", "yes", "on"}


async def reply_if_authorized(
    update: Update,
    context: ContextTypes.DEFAULT_TYPE,
    text: str,
) -> None:
    cfg: Config = context.bot_data["config"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    if update.message:
        await update.message.reply_text(text)


async def acknowledge_if_authorized(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    if fast_ack_enabled():
        await reply_if_authorized(update, context, "Received. Processing...")


async def start_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    await reply_if_authorized(update, context, help_text())


async def list_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    text = await asyncio.to_thread(controller.list_primitives_text)
    await reply_if_authorized(update, context, text)


async def state_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    text = await asyncio.to_thread(controller.state_text)
    await reply_if_authorized(update, context, text)


async def detect_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    await acknowledge_if_authorized(update, context)
    try:
        text = await asyncio.to_thread(controller.detect_text, cfg.detect_url)
    except Exception as exc:
        await reply_if_authorized(update, context, f"Error: {exc}")
        return
    await reply_if_authorized(update, context, text)


async def home_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    await acknowledge_if_authorized(update, context)
    text = await asyncio.to_thread(controller.home)
    await reply_if_authorized(update, context, text)


async def stop_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    await acknowledge_if_authorized(update, context)
    text = await asyncio.to_thread(controller.stop)
    await reply_if_authorized(update, context, text)


async def pickup_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    await acknowledge_if_authorized(update, context)
    text = await asyncio.to_thread(controller.pickup)
    await reply_if_authorized(update, context, text)


async def place_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    await acknowledge_if_authorized(update, context)
    text = await asyncio.to_thread(controller.place)
    await reply_if_authorized(update, context, text)


async def auto_blue_on_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    await acknowledge_if_authorized(update, context)
    text = await asyncio.to_thread(controller.auto_blue_on)
    await reply_if_authorized(update, context, text)


async def auto_blue_off_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    await acknowledge_if_authorized(update, context)
    text = await asyncio.to_thread(controller.auto_blue_off)
    await reply_if_authorized(update, context, text)


async def auto_blue_status_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    text = await asyncio.to_thread(controller.auto_blue_status)
    await reply_if_authorized(update, context, text)


async def blue_nod_on_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    await acknowledge_if_authorized(update, context)
    text = await asyncio.to_thread(controller.blue_nod_on)
    await reply_if_authorized(update, context, text)


async def blue_nod_off_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    await acknowledge_if_authorized(update, context)
    text = await asyncio.to_thread(controller.blue_nod_off)
    await reply_if_authorized(update, context, text)


async def run_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    if len(context.args) != 1:
        await reply_if_authorized(update, context, "Usage: /run <primitive>")
        return
    await acknowledge_if_authorized(update, context)
    try:
        text = await asyncio.to_thread(controller.run, context.args[0].strip(), False)
    except Exception as exc:
        await reply_if_authorized(update, context, f"Error: {exc}")
        return
    await reply_if_authorized(update, context, text)


async def move_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    if len(context.args) != 2:
        await reply_if_authorized(update, context, "Usage: /move <joint> <angle>")
        return
    await acknowledge_if_authorized(update, context)
    try:
        angle = int(context.args[1].strip())
        text = await asyncio.to_thread(controller.move, context.args[0].strip().lower(), angle)
    except Exception as exc:
        await reply_if_authorized(update, context, f"Error: {exc}")
        return
    await reply_if_authorized(update, context, text)


async def step_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    if len(context.args) != 2:
        await reply_if_authorized(update, context, "Usage: /step <joint> <delta>")
        return
    await acknowledge_if_authorized(update, context)
    try:
        delta = int(context.args[1].strip())
        text = await asyncio.to_thread(controller.step, context.args[0].strip().lower(), delta)
    except Exception as exc:
        await reply_if_authorized(update, context, f"Error: {exc}")
        return
    await reply_if_authorized(update, context, text)


async def spin_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    if len(context.args) != 2:
        await reply_if_authorized(update, context, "Usage: /spin rotate <speed>")
        return
    await acknowledge_if_authorized(update, context)
    try:
        speed = int(context.args[1].strip())
        text = await asyncio.to_thread(controller.spin, context.args[0].strip().lower(), speed)
    except Exception as exc:
        await reply_if_authorized(update, context, f"Error: {exc}")
        return
    await reply_if_authorized(update, context, text)


async def move_to_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return
    if len(context.args) not in {3, 4}:
        await reply_if_authorized(update, context, "Usage: /move_to <x_mm> <y_mm> <z_mm> [wrist_pitch]")
        return
    await acknowledge_if_authorized(update, context)
    try:
        x = float(context.args[0].strip())
        y = float(context.args[1].strip())
        z = float(context.args[2].strip())
        wrist_pitch = float(context.args[3].strip()) if len(context.args) == 4 else 0.0
        text = await asyncio.to_thread(controller.move_to_xyz, x, y, z, wrist_pitch)
    except Exception as exc:
        await reply_if_authorized(update, context, f"Error: {exc}")
        return
    await reply_if_authorized(update, context, text)


async def natural_language_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    cfg: Config = context.bot_data["config"]
    controller: PrimitiveRobotController = context.bot_data["controller"]
    if not is_authorized(update, cfg.allowed_chat_id):
        return

    message = update.effective_message
    if not message or not message.text:
        return

    if fast_ack_enabled():
        await message.reply_text("Received. Processing...")

    plan = interpret_natural_language(message.text)
    if plan is None:
        await message.reply_text(
            "I did not understand that. Try: 'go home', 'pick up', "
            "'find blue and nod', 'auto pick blue', or /start."
        )
        return

    try:
        result = await execute_natural_language_plan(plan, controller, cfg.detect_url)
    except Exception as exc:
        await message.reply_text(f"Error: {exc}")
        return

    await message.reply_text(result)


async def error_handler(update: object, context: ContextTypes.DEFAULT_TYPE) -> None:
    logger.exception("Unhandled telegram bot exception", exc_info=context.error)


async def post_init(app: Application) -> None:
    await app.bot.delete_webhook(drop_pending_updates=True)
    logger.info("Webhook deleted and pending updates dropped.")


def main() -> None:
    load_env_file()
    cfg = Config.from_env()
    controller = PrimitiveRobotController(port=cfg.serial_port, baudrate=cfg.baudrate)

    app = (
        Application.builder()
        .token(cfg.telegram_bot_token)
        .post_init(post_init)
        .build()
    )
    app.bot_data["config"] = cfg
    app.bot_data["controller"] = controller

    app.add_handler(CommandHandler("start", start_handler))
    app.add_handler(CommandHandler("detect", detect_handler))
    app.add_handler(CommandHandler("list", list_handler))
    app.add_handler(CommandHandler("state", state_handler))
    app.add_handler(CommandHandler("home", home_handler))
    app.add_handler(CommandHandler("stop", stop_handler))
    app.add_handler(CommandHandler("pickup", pickup_handler))
    app.add_handler(CommandHandler("place", place_handler))
    app.add_handler(CommandHandler("auto_blue_on", auto_blue_on_handler))
    app.add_handler(CommandHandler("auto_blue_off", auto_blue_off_handler))
    app.add_handler(CommandHandler("auto_blue_status", auto_blue_status_handler))
    app.add_handler(CommandHandler("blue_nod_on", blue_nod_on_handler))
    app.add_handler(CommandHandler("blue_nod_off", blue_nod_off_handler))
    app.add_handler(CommandHandler("run", run_handler))
    app.add_handler(CommandHandler("move", move_handler))
    app.add_handler(CommandHandler("move_to", move_to_handler))
    app.add_handler(CommandHandler("step", step_handler))
    app.add_handler(CommandHandler("spin", spin_handler))
    app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, natural_language_handler))
    app.add_error_handler(error_handler)

    logger.info("Starting robot primitives Telegram bot")
    try:
        app.run_polling(allowed_updates=Update.ALL_TYPES)
    finally:
        controller.close()


if __name__ == "__main__":
    main()
