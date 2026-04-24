"""
IK calibration offsets — updated from real measurements.
"""
import numpy as np

OFFSET = np.array([0.03, 0.0, -0.25])  # x, y, z

def apply_offset(target):
    return target + OFFSET

def calibrate(target, actual):
    global OFFSET
    OFFSET = actual - target
    return OFFSET

def get_offset():
    return OFFSET