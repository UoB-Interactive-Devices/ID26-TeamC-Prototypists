# Overview

This project splits robot-arm control into two parts:

- Arduino firmware on the board
- Python control tools on the computer

## How It Works

1. Upload the Arduino sketch:
   - `firmware/openclaw_primitive_bridge/openclaw_primitive_bridge.ino`
2. Connect the board by USB.
3. Run the Python CLI from the terminal.
4. The CLI sends serial commands such as `MOVE shoulder 95`.
5. The Arduino firmware receives those commands and moves the servos with safety limits.

## Main Files

- `firmware/openclaw_primitive_bridge/openclaw_primitive_bridge.ino`
  - Safe serial command parser for the robot arm
- `control/config.py`
  - Joint limits, default poses, and guessed calibration values
- `control/serial_client.py`
  - Python serial transport layer
- `control/primitives.py`
  - Named motion primitives like `ready`, `open_grip`, and `arm_down_small`
- `control/cli.py`
  - Terminal interface for testing and running primitives
- `control/kinematics.py`
  - Inverse kinematics solver for gripper-tip coordinates
- `control/move_to.py`
  - Command-line coordinate runner built on the IK solver

## What A Primitive Is

A primitive is a small robot action with a clear name and behavior.

Examples:

- `ready`
- `open_grip`
- `close_grip`
- `shoulder_up_small`
- `arm_down_small`
- `rotate_left_small`

These are easier to debug and safer to use than raw servo control from an agent.

## Intended Progression

1. Calibrate joint directions and limits
2. Tune a few reliable poses
3. Build small safe primitives
4. Compose those primitives into larger actions
5. Later let an agent choose which primitive to run
