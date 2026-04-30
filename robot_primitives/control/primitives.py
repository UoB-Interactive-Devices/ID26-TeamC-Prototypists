from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from .config import (
    DEFAULT_STATE,
    GRIP_CLOSED_ANGLE,
    GRIP_OPEN_ANGLE,
    JOINT_LIMITS,
    POSES,
    ROTATE_NUDGE_SPEED,
    STANDARD_JOINTS,
    STEP_SIZES,
)


def clamp(joint: str, value: int) -> int:
    limit = JOINT_LIMITS[joint]
    return max(limit.minimum, min(limit.maximum, value))


@dataclass
class ArmState:
    shoulder: int = DEFAULT_STATE["shoulder"]
    wrist: int = DEFAULT_STATE["wrist"]
    wrist_rotation: int = DEFAULT_STATE["wrist_rotation"]
    grip: int = DEFAULT_STATE["grip"]
    rotate: int = DEFAULT_STATE["rotate"]

    @classmethod
    def from_mapping(cls, payload: dict[str, int]) -> "ArmState":
        return cls(
            shoulder=int(payload.get("shoulder", DEFAULT_STATE["shoulder"])),
            wrist=int(payload.get("wrist", DEFAULT_STATE["wrist"])),
            wrist_rotation=int(
                payload.get("wrist_rotation", DEFAULT_STATE["wrist_rotation"])
            ),
            grip=int(payload.get("grip", DEFAULT_STATE["grip"])),
            rotate=int(payload.get("rotate", DEFAULT_STATE["rotate"])),
        )

    def as_dict(self) -> dict[str, int]:
        return {
            "shoulder": self.shoulder,
            "wrist": self.wrist,
            "wrist_rotation": self.wrist_rotation,
            "grip": self.grip,
            "rotate": self.rotate,
        }


@dataclass(frozen=True)
class Command:
    kind: str
    joint: str = ""
    value: int = 0
    pause_s: float = 0.25


PrimitiveBuilder = Callable[[ArmState], list[Command]]


def pose_commands(name: str) -> PrimitiveBuilder:
    target = POSES[name]

    def build(_: ArmState) -> list[Command]:
        return [Command("move", joint, angle) for joint, angle in target.items()]

    return build


def _single_step(joint: str, delta: int) -> PrimitiveBuilder:
    def build(_: ArmState) -> list[Command]:
        return [Command("step", joint, delta)]

    return build


def open_grip(_: ArmState) -> list[Command]:
    return [Command("move", "grip", GRIP_OPEN_ANGLE)]


def close_grip(_: ArmState) -> list[Command]:
    return [Command("move", "grip", GRIP_CLOSED_ANGLE)]


def arm_down_small(state: ArmState) -> list[Command]:
    return [
        Command("move", "shoulder", clamp("shoulder", state.shoulder + STEP_SIZES["small"])),
        Command("move", "wrist", clamp("wrist", state.wrist - STEP_SIZES["small"])),
    ]


def arm_up_small(state: ArmState) -> list[Command]:
    return [
        Command("move", "shoulder", clamp("shoulder", state.shoulder - STEP_SIZES["small"])),
        Command("move", "wrist", clamp("wrist", state.wrist + STEP_SIZES["small"])),
    ]


def rotate_left_small(_: ArmState) -> list[Command]:
    return [
        Command("spin", "rotate", -ROTATE_NUDGE_SPEED, pause_s=0.35),
        Command("spin", "rotate", 0, pause_s=0.1),
    ]


def rotate_right_small(_: ArmState) -> list[Command]:
    return [
        Command("spin", "rotate", ROTATE_NUDGE_SPEED, pause_s=0.35),
        Command("spin", "rotate", 0, pause_s=0.1),
    ]


def rotate_stop(_: ArmState) -> list[Command]:
    return [Command("spin", "rotate", 0, pause_s=0.05)]


def pickup(_: ArmState) -> list[Command]:
    return [Command("macro", "pickup", 0, pause_s=0.1)]


def pickup_1(_: ArmState) -> list[Command]:
    return [Command("macro", "pickup_1", 0, pause_s=0.1)]


def pickup_2(_: ArmState) -> list[Command]:
    return [Command("macro", "pickup_2", 0, pause_s=0.1)]


def play(_: ArmState) -> list[Command]:
    return [Command("macro", "play", 0, pause_s=0.1)]


def place(_: ArmState) -> list[Command]:
    return [Command("macro", "place", 0, pause_s=0.1)]


def place_1(_: ArmState) -> list[Command]:
    return [Command("macro", "place_1", 0, pause_s=0.1)]


def place_2(_: ArmState) -> list[Command]:
    return [Command("macro", "place_2", 0, pause_s=0.1)]


def feed(_: ArmState) -> list[Command]:
    return [Command("macro", "feed", 0, pause_s=0.1)]


PRIMITIVES: dict[str, PrimitiveBuilder] = {
    "home": pose_commands("home"),
    "ready": pose_commands("ready"),
    "floor_pick": pose_commands("floor_pick"),
    "carry": pose_commands("carry"),
    "open_grip": open_grip,
    "close_grip": close_grip,
    "shoulder_up_small": _single_step("shoulder", -STEP_SIZES["small"]),
    "shoulder_down_small": _single_step("shoulder", STEP_SIZES["small"]),
    "wrist_up_small": _single_step("wrist", STEP_SIZES["small"]),
    "wrist_down_small": _single_step("wrist", -STEP_SIZES["small"]),
    "wrist_rotate_left_small": _single_step("wrist_rotation", -STEP_SIZES["small"]),
    "wrist_rotate_right_small": _single_step("wrist_rotation", STEP_SIZES["small"]),
    "grip_open_small": _single_step("grip", STEP_SIZES["small"]),
    "grip_close_small": _single_step("grip", -STEP_SIZES["small"]),
    "arm_down_small": arm_down_small,
    "arm_up_small": arm_up_small,
    "pickup": pickup,
    "pickup_1": pickup_1,
    "pickup_2": pickup_2,
    "play": play,
    "place": place,
    "place_1": place_1,
    "place_2": place_2,
    "feed": feed,
    "rotate_left_small": rotate_left_small,
    "rotate_right_small": rotate_right_small,
    "rotate_stop": rotate_stop,
}


def list_primitives() -> list[str]:
    return sorted(PRIMITIVES)


def run_primitive(name: str, client, state: ArmState | None = None, verbose: bool = False) -> ArmState:
    if name not in PRIMITIVES:
        raise KeyError(f"Unknown primitive: {name}")

    if state is None:
        state = ArmState.from_mapping(client.get_state())

    commands = PRIMITIVES[name](state)

    for command in commands:
        if command.kind == "move":
            client.move(command.joint, command.value)
            if command.joint in STANDARD_JOINTS:
                setattr(state, command.joint, command.value)
            else:
                setattr(state, command.joint, clamp("rotate", command.value))
        elif command.kind == "step":
            current = getattr(state, command.joint)
            target = clamp(command.joint, current + command.value)
            client.step(command.joint, command.value)
            setattr(state, command.joint, target)
        elif command.kind == "spin":
            client.spin(command.joint, command.value)
            if verbose:
                print(f"spin {command.joint} {command.value}")
        elif command.kind == "macro":
            if command.joint == "pickup":
                client.pickup()
            elif command.joint == "pickup_1":
                client.pickup_1()
            elif command.joint == "pickup_2":
                client.pickup_2()
            elif command.joint == "play":
                client.play()
            elif command.joint == "place":
                client.place()
            elif command.joint == "place_1":
                client.place_1()
            elif command.joint == "place_2":
                client.place_2()
            elif command.joint == "feed":
                client.feed()
            else:
                raise ValueError(f"Unsupported macro command: {command.joint}")
        else:
            raise ValueError(f"Unsupported command kind: {command.kind}")

        if verbose:
            if command.kind in {"move", "step", "spin"}:
                print(f"{command.kind} {command.joint} {command.value}")
            else:
                print(f"{command.kind} {command.joint}")
        time.sleep(command.pause_s)

    return state
