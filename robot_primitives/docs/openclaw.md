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
python -m control.agent_api --input '{"action":"place"}'
python -m control.agent_api --input '{"action":"auto_blue_on"}'
python -m control.agent_api --input '{"action":"auto_blue_off"}'
python -m control.agent_api --input '{"action":"auto_blue_status"}'
python -m control.agent_api --input '{"action":"blue_nod_on"}'
python -m control.agent_api --input '{"action":"blue_nod_off"}'
```

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
5. Treat object-to-object tasks as unavailable until color signatures and camera-to-robot calibration are completed.
6. Always read `state` before making repeated low-level adjustments.
7. Fall back to `home` or `stop` if the arm behaves unexpectedly.

## Why This Interface

This keeps OpenClaw on a small, structured action API:

- easier to prompt
- easier to log
- safer than direct servo math
- easier to swap between manual, Telegram, and agent control
