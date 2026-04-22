from __future__ import annotations

import math
from dataclasses import dataclass

from .config import JOINT_LIMITS


class IKError(ValueError):
    """Raised when a target is outside the modeled workspace."""


@dataclass(frozen=True)
class ArmGeometry:
    # Shoulder joint height above ground in millimeters.
    shoulder_height_mm: float
    # Shoulder to elbow length.
    upper_arm_mm: float
    # Elbow to wrist length.
    forearm_mm: float
    # Wrist servo to gripper tip length.
    tool_length_mm: float
    # Base rotation platform diameter, useful for docs / future collision checks.
    base_diameter_mm: float


@dataclass(frozen=True)
class ServoCalibration:
    rotate_center_deg: float = 90.0
    shoulder_center_deg: float = 90.0
    elbow_center_deg: float = 90.0
    wrist_center_deg: float = 90.0
    rotate_direction: int = 1
    shoulder_direction: int = 1
    elbow_direction: int = 1
    wrist_direction: int = 1
    # Forearm absolute pitch that corresponds to the elbow servo center.
    # User-provided calibration: servo 90 deg when the forearm is vertical.
    elbow_vertical_pitch_deg: float = 90.0


@dataclass(frozen=True)
class PoseSolution:
    rotate: float
    shoulder: float
    elbow: float
    wrist: float
    forearm_pitch: float
    radial_mm: float
    wrist_r_mm: float
    wrist_z_mm: float

    def servo_angles(self, calibration: ServoCalibration) -> dict[str, int]:
        servo_rotate = calibration.rotate_center_deg + calibration.rotate_direction * self.rotate
        servo_shoulder = (
            calibration.shoulder_center_deg
            + calibration.shoulder_direction * self.shoulder
        )
        # Map elbow servo against the *absolute forearm direction* rather than the
        # raw inner bend angle. This matches the real arm calibration where
        # elbow servo 90 deg means the forearm is vertical.
        servo_elbow = (
            calibration.elbow_center_deg
            + calibration.elbow_direction
            * (self.forearm_pitch - calibration.elbow_vertical_pitch_deg)
        )
        servo_wrist = calibration.wrist_center_deg + calibration.wrist_direction * self.wrist

        return {
            "rotate": round(servo_rotate),
            "shoulder": round(servo_shoulder),
            "elbow": round(servo_elbow),
            "wrist": round(servo_wrist),
        }


DEFAULT_GEOMETRY = ArmGeometry(
    # URDF-derived model:
    # base_joint z 74 mm + shoulder_joint z 24 mm = shoulder at 98 mm.
    shoulder_height_mm=98.0,
    upper_arm_mm=200.0,
    forearm_mm=120.0,
    tool_length_mm=240.0,
    base_diameter_mm=150.0,
)

# Earlier hand measurements kept here so we can compare against the URDF model.
MEASURED_GEOMETRY = ArmGeometry(
    shoulder_height_mm=140.0,
    upper_arm_mm=185.0,
    forearm_mm=85.0,
    tool_length_mm=240.0,
    base_diameter_mm=150.0,
)

DEFAULT_SERVO_CALIBRATION = ServoCalibration()


def clamp_cosine(value: float) -> float:
    return max(-1.0, min(1.0, value))


def within_joint_limits(angles: dict[str, int]) -> tuple[bool, str | None]:
    for joint, angle in angles.items():
        if joint not in JOINT_LIMITS:
            continue
        limit = JOINT_LIMITS[joint]
        if not (limit.minimum <= angle <= limit.maximum):
            return False, f"{joint} angle {angle} outside [{limit.minimum}, {limit.maximum}]"
    return True, None


def solve_inverse_kinematics(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    *,
    wrist_pitch_deg: float = 0.0,
    elbow_up: bool = False,
    geometry: ArmGeometry = DEFAULT_GEOMETRY,
) -> PoseSolution:
    """Solve a simple 4-DOF arm model for the gripper tip position.

    Coordinate system assumptions:
    - origin is at the center of the turntable on the ground plane
    - +x / +y lie on the ground plane
    - +z points upward
    - wrist_pitch_deg is the gripper pitch in the radial vertical plane
    """

    radial_mm = math.hypot(x_mm, y_mm)
    rotate_deg = math.degrees(math.atan2(y_mm, x_mm)) if radial_mm > 1e-6 else 0.0

    shoulder_z_mm = z_mm - geometry.shoulder_height_mm
    wrist_pitch_rad = math.radians(wrist_pitch_deg)

    wrist_r_mm = radial_mm - geometry.tool_length_mm * math.cos(wrist_pitch_rad)
    wrist_z_mm = shoulder_z_mm - geometry.tool_length_mm * math.sin(wrist_pitch_rad)

    l1 = geometry.upper_arm_mm
    l2 = geometry.forearm_mm
    wrist_distance_sq = wrist_r_mm * wrist_r_mm + wrist_z_mm * wrist_z_mm
    wrist_distance = math.sqrt(wrist_distance_sq)

    if wrist_distance > (l1 + l2):
        raise IKError(
            f"Target is outside reach: wrist center distance {wrist_distance:.1f} mm > "
            f"{l1 + l2:.1f} mm"
        )
    if wrist_distance < abs(l1 - l2):
        raise IKError(
            f"Target is inside the arm dead-zone: wrist center distance {wrist_distance:.1f} mm < "
            f"{abs(l1 - l2):.1f} mm"
        )

    cos_elbow = clamp_cosine((wrist_distance_sq - l1 * l1 - l2 * l2) / (2 * l1 * l2))
    elbow_inner_rad = math.acos(cos_elbow)
    if elbow_up:
        elbow_inner_rad = -elbow_inner_rad

    shoulder_to_target_rad = math.atan2(wrist_z_mm, wrist_r_mm)
    shoulder_offset_rad = math.atan2(l2 * math.sin(elbow_inner_rad), l1 + l2 * math.cos(elbow_inner_rad))
    shoulder_rad = shoulder_to_target_rad - shoulder_offset_rad

    # Use the "arm bend away from straight" convention in degrees.
    shoulder_deg = math.degrees(shoulder_rad)
    elbow_deg = math.degrees(elbow_inner_rad)
    wrist_deg = wrist_pitch_deg - shoulder_deg - elbow_deg
    forearm_pitch_deg = shoulder_deg + elbow_deg

    return PoseSolution(
        rotate=rotate_deg,
        shoulder=shoulder_deg,
        elbow=elbow_deg,
        wrist=wrist_deg,
        forearm_pitch=forearm_pitch_deg,
        radial_mm=radial_mm,
        wrist_r_mm=wrist_r_mm,
        wrist_z_mm=wrist_z_mm,
    )
