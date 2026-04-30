# OpenClaw Integration

The safest way to connect OpenClaw to this project is to have OpenClaw call the local JSON command runner instead of scraping Telegram messages.

## Entry Point

Use:

```bash
python -m control.agent_api --input '{"action":"state"}'
```

The script reads `.env` for:

- `ROBOT_ARM_SERIAL_PORT`
- `ROBOT_ARM_BAUDRATE`
- `ROBOT_ARM_DETECT_URL`

It returns JSON on stdout.

## Telegram Natural Language Through OpenClaw

Telegram can route plain text through an OpenClaw/LLM command before sending anything to Arduino.

Set these in `.env`:

```text
ROBOT_ARM_NL_INTERPRETER=openclaw
OPENCLAW_INTENT_COMMAND=your-openclaw-command-here
OPENCLAW_INTENT_TIMEOUT=20
```

The command must:

- read the prompt from stdin
- return one JSON object on stdout
- choose only one allowed robot action
- return `{"action":"help"}` for multi-step requests like `pick up and place`

Example JSON output:

```json
{"action":"home"}
```

or:

```json
{"action":"move","joint":"shoulder","angle":95}
```

or:

```json
{"action":"run","name":"ready"}
```

For coordinate moves, OpenClaw can output:

```json
{"action":"move_to","x":530,"y":0,"z":240,"wrist_pitch":20}
```

For object-level commands, OpenClaw can output:

```json
{"action":"pick_object","color":"blue"}
```

or:

```json
{"action":"move_object_to","object_color":"blue","destination_color":"red","destination_type":"flag"}
```

For webcam-driven commands, OpenClaw can output:

```json
{"action":"detect"}
```

This runs YOLO once on the latest frame from the live camera stream.
Start the stream server first with `--model` so `/detect` is available.
The same server also exposes the raw live view at `/stream` and the YOLO
overlay live view at `/annotated_stream`, so OpenClaw does not need to open
the physical webcam itself for detection.

```bash
python -m control.cli stream \
  --camera 0 \
  --model /Users/kanghyeon/Documents/Arduino/runs/detect/train/weights/best.pt \
  --conf 0.7 \
  --width 640 \
  --height 480 \
  --fps 15
```

```json
{"action":"webcam_control","mode":"palette","duration":30}
```

or:

```json
{"action":"webcam_control","mode":"track","colors":["blue"],"duration":30}
```

If OpenClaw fails, the Telegram bot falls back to the local rule parser.

## Supported Actions

State and primitive discovery:

```bash
python -m control.agent_api --input '{"action":"ping"}'
python -m control.agent_api --input '{"action":"state"}'
python -m control.agent_api --input '{"action":"list"}'
```

High-level actions:

```bash
python -m control.agent_api --input '{"action":"home"}'
python -m control.agent_api --input '{"action":"stop"}'
python -m control.agent_api --input '{"action":"pickup"}'
python -m control.agent_api --input '{"action":"pickup_1"}'
python -m control.agent_api --input '{"action":"pickup_2"}'
python -m control.agent_api --input '{"action":"place"}'
python -m control.agent_api --input '{"action":"place_1"}'
python -m control.agent_api --input '{"action":"place_2"}'
python -m control.agent_api --input '{"action":"auto_blue_on"}'
python -m control.agent_api --input '{"action":"auto_blue_off"}'
python -m control.agent_api --input '{"action":"auto_blue_status"}'
python -m control.agent_api --input '{"action":"blue_nod_on"}'
python -m control.agent_api --input '{"action":"blue_nod_off"}'
```

Webcam actions:

```bash
python -m control.agent_api --input '{"action":"detect"}'
python -m control.agent_api --input '{"action":"webcam_control","mode":"palette","duration":30}'
python -m control.agent_api --input '{"action":"webcam_control","mode":"track","colors":["blue"],"duration":30}'
```

`palette` mode uses a USB/webcam on the PC and maps colors to high-level commands:

- blue: pickup
- green: place
- yellow: home
- red: stop

`track` mode expects one color and nudges rotate/arm primitives until the target is centered and close enough to pick up.

Coordinate actions:

```bash
python -m control.agent_api --input '{"action":"move_to","x":530,"y":0,"z":240,"wrist_pitch":20}'
python -m control.agent_api --input '{"action":"move_to","x":530,"y":0,"z":240,"wrist_pitch":20,"dry_run":true}'
```

Object actions:

```bash
python -m control.agent_api --input '{"action":"pick_object","color":"blue"}'
python -m control.agent_api --input '{"action":"move_object_to","object_color":"blue","destination_color":"red","destination_type":"flag"}'
```

`pick_object` currently only executes blue pickup through Pixy2 signature 1. `move_object_to` is intentionally a scaffold until camera-to-robot coordinate calibration and place-at-target behavior are added.

Joint actions:

```bash
python -m control.agent_api --input '{"action":"move","joint":"shoulder","angle":95}'
python -m control.agent_api --input '{"action":"step","joint":"elbow","delta":-4}'
python -m control.agent_api --input '{"action":"spin","joint":"rotate","speed":15}'
```

Primitive execution:

```bash
python -m control.agent_api --input '{"action":"run","name":"ready"}'
python -m control.agent_api --input '{"action":"run","name":"arm_down_small"}'
python -m control.agent_api --input '{"action":"run","name":"pickup"}'
```

## Recommended OpenClaw Policy

Give OpenClaw these rules:

1. Prefer `state`, `run`, `pickup`, `place`, and `home` over raw `move`.
2. Use `move` and `step` only for calibration or recovery.
3. Use `auto_blue_on` only after Pixy2 has been trained on the blue target as signature 1.
4. Use `move_to` only when explicit millimeter coordinates are known.
5. Use `detect` when the user asks to run object detection once on the live camera stream.
6. Use `webcam_control` when the user explicitly asks the robot to use color-based webcam control.
7. Treat object-to-object tasks as unavailable until color signatures and camera-to-robot calibration are completed.
8. Always read `state` before making repeated low-level adjustments.
9. If the user asks for multiple actions in one sentence, return `{"action":"help"}` instead of choosing one.
10. Fall back to `home` or `stop` if the arm behaves unexpectedly.

## Why This Interface

This keeps OpenClaw on a small, structured action API:

- easier to prompt
- easier to log
- safer than direct servo math
- easier to swap between manual, Telegram, and agent control
