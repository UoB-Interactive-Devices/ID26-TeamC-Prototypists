# Robot Primitive Calibration Starter

This folder is the first step toward a calibration-friendly control stack for the robot arm.

It gives you:

- Arduino firmware for safe serial control with joint limits
- Python tools for sending commands and running named primitives
- A simple workflow for discovering safe angles and useful poses on the real arm

## Project Layout

- `firmware/openclaw_primitive_bridge/openclaw_primitive_bridge.ino`
- `control/config.py`
- `control/serial_client.py`
- `control/primitives.py`
- `control/cli.py`
- `docs/overview.md`
- `docs/cli.md`
- `docs/calibration.md`

## Start Here

1. Read [docs/overview.md](./docs/overview.md)
2. Set up Python dependencies
3. Upload the Arduino sketch
4. Follow [docs/calibration.md](./docs/calibration.md)

## Quick Setup

```bash
cd robot_primitives
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Telegram Setup

To control the arm from Telegram:

```bash
cp .env.example .env
python -m control.telegram_bot
```

Then fill in `.env` with:

- `TELEGRAM_BOT_TOKEN`
- `TELEGRAM_ALLOWED_CHAT_ID`
- `ROBOT_ARM_SERIAL_PORT`
- `ROBOT_ARM_BAUDRATE`

Telegram commands now include:

- `/pickup`
- `/place`
- `/auto_blue_on`
- `/auto_blue_off`
- `/auto_blue_status`
- `/blue_nod_on`
- `/blue_nod_off`
- `/run pickup`
- `/run place`

The Telegram bot also accepts simple natural language messages, for example:

- `go home`
- `pick up`
- `place it`
- `find blue and nod`
- `stop blue nod`
- `auto pick blue`
- `move shoulder to 95`
- `step elbow -4`

To route Telegram natural language through OpenClaw/LLM first, set:

```text
ROBOT_ARM_NL_INTERPRETER=openclaw
OPENCLAW_INTENT_COMMAND=your-openclaw-command-here
```

## OpenClaw / Agent Setup

For agent control, use the JSON command runner:

```bash
python -m control.agent_api --input '{"action":"state"}'
python -m control.agent_api --input '{"action":"home"}'
python -m control.agent_api --input '{"action":"run","name":"ready"}'
python -m control.agent_api --input '{"action":"move","joint":"shoulder","angle":95}'
python -m control.agent_api --input '{"action":"auto_blue_on"}'
python -m control.agent_api --input '{"action":"detect"}'
```

This is the recommended interface for OpenClaw because it returns structured JSON.

## Camera And YOLO Detection

The PC can use a USB webcam as OpenClaw's eyes. Start the shared camera
stream with the trained YOLO model:

```bash
python -m control.cli stream \
  --camera external \
  --model yolo11n.pt \
  --conf 0.4 \
  --width 640 \
  --height 480 \
  --fps 15
```

Open these endpoints from a browser or agent:

- `/stream`: raw live camera feed
- `/annotated_stream`: live camera feed with YOLO boxes
- `/detect`: one-shot JSON detection result

You can also run detection directly from the command line:

```bash
python -m control.yolo_webcam_detect --model yolo11n.pt --camera external --conf 0.4
python -m control.yolo_webcam_detect --model yolo11n.pt --camera external --once --no-show
```

Use the YOLO detector and `/detect` endpoint when OpenClaw needs vision input.

## Important Note

The angles and poses in this project are safe guesses, not hardware-verified values.

Expect to tune:

- joint directions
- joint min and max values
- gripper open and closed values
- shoulder/elbow/wrist coordination
- rotate stop value for the continuous servo
