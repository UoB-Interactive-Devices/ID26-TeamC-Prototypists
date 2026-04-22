#!/usr/bin/env python3
"""
IK-based position control for robot arm.
Given 3D coordinates (x, y, z) in meters, compute IK and move joints.
"""
import ikpy.chain
import math
import sys
import numpy as np

# Load calibration offsets
try:
    from ik_calibration import apply_offset, calibrate, get_offset
    OFFSET = get_offset()
    print(f"Calibration loaded: offset = {OFFSET}", file=sys.stderr)
except Exception:
    OFFSET = np.array([0.0, 0.0, 0.0])
    print("No calibration data, using zero offset", file=sys.stderr)

# Load chain
my_chain = ikpy.chain.Chain.from_urdf_file(
    "/Users/kanghyeon/Documents/Arduino/robot_primitives/arm_urdf.urdf",
    active_links_mask=[False, False, True, True, True, True, False]
)

# Servo angle ranges (from config)
SERVO_HOME = 90
SERVO_LIMITS = {
    'shoulder': (45, 135),
    'elbow': (0, 180),
    'wrist': (0, 180),
    'rotate': (60, 120),
}

def urdf_angle_to_servo(angle_deg):
    """Convert URDF joint angle (relative to home) to servo angle (0-180)."""
    return round(SERVO_HOME + angle_deg, 1)

def servo_angle_ok(joint, angle):
    lo, hi = SERVO_LIMITS[joint]
    return lo <= angle <= hi

def ik_to_angles(ik):
    """Extract active joint angles from IK result."""
    return {
        'shoulder': urdf_angle_to_servo(math.degrees(ik[2])),
        'elbow': urdf_angle_to_servo(math.degrees(ik[3])),
        'wrist': urdf_angle_to_servo(math.degrees(ik[4])),
        'rotate': urdf_angle_to_servo(math.degrees(ik[5])),
    }

def move_to_position(x, y, z, verbose=True):
    target = np.array([x, y, z])
    
    # Apply calibration offset
    adjusted = apply_offset(target)
    if verbose:
        print(f"Target: {target}, Adjusted: {adjusted}", file=sys.stderr)
    
    ik = my_chain.inverse_kinematics(adjusted.tolist())
    angles = ik_to_angles(ik)
    
    if verbose:
        print(f"Target: ({x}, {y}, {z})")
        print(f"Adjusted: ({adjusted[0]:.3f}, {adjusted[1]:.3f}, {adjusted[2]:.3f})")
        print(f"IK angles: {[round(math.degrees(r), 2) for r in ik.tolist()]}")
        print(f"Servo angles: {angles}")
    
    # Validate
    for joint, angle in angles.items():
        if not servo_angle_ok(joint, angle):
            print(f"WARNING: {joint} angle {angle}° out of range {SERVO_LIMITS[joint]}", file=sys.stderr)
    
    return angles

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='IK position control')
    parser.add_argument('x', type=float, help='X coordinate (meters)')
    parser.add_argument('y', type=float, help='Y coordinate (meters)')
    parser.add_argument('z', type=float, help='Z coordinate (meters)')
    args = parser.parse_args()
    
    result = move_to_position(args.x, args.y, args.z)
    print(result)