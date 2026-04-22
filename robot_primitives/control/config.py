from dataclasses import dataclass


STANDARD_JOINTS = ("shoulder", "elbow", "wrist", "wrist_rotation", "grip")
ALL_JOINTS = STANDARD_JOINTS + ("rotate",)


@dataclass(frozen=True)
class JointLimit:
    minimum: int
    maximum: int


JOINT_LIMITS = {
    "shoulder": JointLimit(45, 135),
    "elbow": JointLimit(0, 180),
    "wrist": JointLimit(0, 180),
    "wrist_rotation": JointLimit(0, 180),
    "grip": JointLimit(20, 120),
    "rotate": JointLimit(60, 120),
}


DEFAULT_STATE = {
    "shoulder": 90,
    "elbow": 90,
    "wrist": 90,
    "wrist_rotation": 90,
    "grip": 90,
    "rotate": 90,
}


POSES = {
    "home": {
        "shoulder": 90,
        "elbow": 90,
        "wrist": 90,
        "wrist_rotation": 90,
        "grip": 90,
    },
    "ready": {
        "shoulder": 82,
        "elbow": 98,
        "wrist": 92,
        "wrist_rotation": 90,
        "grip": 95,
    },
    "floor_pick": {
        "shoulder": 110,
        "elbow": 120,
        "wrist": 80,
        "wrist_rotation": 90,
        "grip": 110,
    },
    "carry": {
        "shoulder": 75,
        "elbow": 85,
        "wrist": 100,
        "wrist_rotation": 90,
        "grip": 55,
    },
}


STEP_SIZES = {
    "small": 4,
    "medium": 8,
}


GRIP_OPEN_ANGLE = 110
GRIP_CLOSED_ANGLE = 55
ROTATE_STOP_ANGLE = 90
ROTATE_NUDGE_SPEED = 14
