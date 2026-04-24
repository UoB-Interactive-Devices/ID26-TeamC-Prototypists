# Kinematics

This project now includes a first-pass inverse kinematics layer for gripper-tip coordinates.

## Coordinate Frame

Assumptions:

- origin is the center of the turntable on the ground plane
- `x` and `y` lie on the ground plane
- `z` is height above the ground
- target coordinates describe the **gripper tip**

## Measurements Used

Current defaults in [kinematics.py](/Users/kanghyeon/Documents/Arduino/robot_primitives/control/kinematics.py):

- shoulder height: `98 mm`
- upper arm `L1` shoulder to elbow: `200 mm`
- forearm `L2` elbow to wrist: `120 mm`
- wrist to gripper tip tool length: `240 mm`
- base diameter: `150 mm`

These values come from the URDF:

- base joint height: `74 mm`
- shoulder joint offset above turntable: `24 mm`
- arm1 length: `200 mm`
- arm2 length: `120 mm`
- wrist plus gripper length: `85 + 155 = 240 mm`

Earlier hand measurements are preserved in the file for comparison:

- shoulder height: `140 mm`
- `L1 = 185 mm`
- `L2 = 85 mm`

## Solver Model

The solver uses:

1. base rotation from `atan2(y, x)`
2. a 2-link planar solve for shoulder and elbow
3. wrist compensation so the gripper tip lands at the requested coordinate

This is a practical first model, not a fully calibrated ground-truth mechanical model.

## Important Assumptions

- servo center angles are currently treated as `90 deg`
- joint directions are assumed positive with increasing mathematical angles
- wrist pitch defaults to `0 deg` in the radial plane
- no collision checking is implemented yet

These assumptions will almost certainly need calibration on the real arm.

## Dry-Run Example

```bash
cd /Users/kanghyeon/Documents/Arduino/robot_primitives
source .venv/bin/activate
python -m control.move_to 530 0 240 --wrist-pitch 20 --dry-run
```

This prints solved servo angles without moving the arm.

## Move Example

```bash
python -m control.move_to 530 0 240 --wrist-pitch 20 --port /dev/cu.usbmodem1101 --baudrate 115200
```

## Agent API Example

OpenClaw can request the same coordinate move through the JSON API:

```bash
python -m control.agent_api --input '{"action":"move_to","x":530,"y":0,"z":240,"wrist_pitch":20}'
```

## Next Calibration Steps

1. Verify that increasing `x` really extends forward.
2. Verify that positive `y` rotates the base the expected way.
3. Compare solved positions against a ruler on a few known points.
4. Adjust servo center offsets and direction signs in the kinematics module.
5. Compare the `185/85` and `200/120` link variants against reality.
