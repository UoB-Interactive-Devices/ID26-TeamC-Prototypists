# Calibration Guide

The values in this project are only initial guesses. Calibrate on the real arm before relying on the primitives.

## Goals

You want to discover:

- safe minimum and maximum angle for each joint
- whether the guessed direction is correct
- gripper open and closed values
- a few useful poses like `home` and `ready`
- the stop value and nudge speed for the continuous rotation servo

## Recommended Order

1. Upload the Arduino sketch
2. Confirm the serial port
3. Test one joint at a time
4. Update `control/config.py`
5. Re-test the named primitives

## Joint Testing

Start with single-joint commands, not full primitives.

Examples:

```bash
python -m control.cli --port /dev/cu.usbmodem1101 state
python -m control.cli --port /dev/cu.usbmodem1101 move shoulder 90
python -m control.cli --port /dev/cu.usbmodem1101 step shoulder 4
python -m control.cli --port /dev/cu.usbmodem1101 step shoulder -4
```

Do the same for:

- `elbow`
- `wrist`
- `grip`
- `rotate`

## What To Watch For

Stop immediately if:

- a joint is pushing into a hard stop
- the arm is colliding with the table or base
- the mirrored shoulder movement is backwards
- the continuous rotation servo does not stop at the expected value

## Files To Tune

Update [control/config.py](/Users/kanghyeon/Documents/Arduino/robot_primitives/control/config.py):

- `JOINT_LIMITS`
- `DEFAULT_STATE`
- `POSES`
- `GRIP_OPEN_ANGLE`
- `GRIP_CLOSED_ANGLE`
- `ROTATE_STOP_ANGLE`
- `ROTATE_NUDGE_SPEED`

If physical joint direction is wrong in firmware, update:

- [openclaw_primitive_bridge.ino](/Users/kanghyeon/Documents/Arduino/robot_primitives/firmware/openclaw_primitive_bridge/openclaw_primitive_bridge.ino)

## After Calibration

Once single-joint tests are safe:

1. Run `ready`
2. Run `open_grip`
3. Run `close_grip`
4. Run `arm_down_small`
5. Run `arm_up_small`

Then start building larger task scripts from those primitives.
