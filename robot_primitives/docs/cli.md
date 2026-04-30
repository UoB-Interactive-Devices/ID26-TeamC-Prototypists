# CLI Guide

The Python CLI is the easiest way to control the arm from your computer or from a higher-level agent.

## Setup

```bash
cd robot_primitives
source .venv/bin/activate
```

If you have not installed dependencies yet:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Command Pattern

```bash
python -m control.cli --port /dev/cu.usbmodem1101 <command>
```

Replace the port with the board's actual serial device.

## Useful Commands

List all primitives:

```bash
python -m control.cli list
```

Read current state:

```bash
python -m control.cli --port /dev/cu.usbmodem1101 state
```

Move one joint to an angle:

```bash
python -m control.cli --port /dev/cu.usbmodem1101 move shoulder 95
```

Step one joint by a delta:

```bash
python -m control.cli --port /dev/cu.usbmodem1101 step elbow -4
```

Control the continuous rotation servo:

```bash
python -m control.cli --port /dev/cu.usbmodem1101 spin rotate 15
python -m control.cli --port /dev/cu.usbmodem1101 spin rotate 0
```

Run a primitive:

```bash
python -m control.cli --port /dev/cu.usbmodem1101 run ready
python -m control.cli --port /dev/cu.usbmodem1101 run arm_down_small
python -m control.cli --port /dev/cu.usbmodem1101 run open_grip
```

Use the webcam as a color-command controller:

```bash
python -m control.cli --port /dev/cu.usbmodem1101 webcam --mode palette
```

In `palette` mode, hold a colored object in view for about a second:

- blue: pickup
- green: place
- yellow: home
- red: stop

Track and approach one color:

```bash
python -m control.cli --port /dev/cu.usbmodem1101 webcam --mode track --colors blue
```

Preview detections without moving the robot:

```bash
python -m control.cli webcam --mode palette --dry-run --verbose
```

Send safety actions:

```bash
python -m control.cli --port /dev/cu.usbmodem1101 home
python -m control.cli --port /dev/cu.usbmodem1101 stop
```

Serve one shared camera for browser/Telegram/OpenClaw detection:

```bash
python -m control.cli stream \
  --camera 0 \
  --model /Users/kanghyeon/Documents/Arduino/runs/detect/train/weights/best.pt \
  --conf 0.7 \
  --width 640 \
  --height 480 \
  --fps 15
```

Open `/stream` for the raw camera, `/annotated_stream` for YOLO boxes, and use
`/detect` for OpenClaw or Telegram one-shot detection.

## How An Agent Uses It

An agent can use the CLI in two ways:

- call the CLI as a shell command
- import the Python modules directly

Shell example:

```bash
python -m control.cli --port /dev/cu.usbmodem1101 run rotate_left_small
```

Python example:

```python
from control.primitives import ArmState, run_primitive
from control.serial_client import RobotArmSerialClient

client = RobotArmSerialClient("/dev/cu.usbmodem1101")
state = ArmState.from_mapping(client.get_state())
state = run_primitive("ready", client, state=state, verbose=True)
client.close()
```

## Telegram Control

You can also control the arm from Telegram by running:

```bash
python -m control.telegram_bot
```

Setup steps:

1. Copy `.env.example` to `.env`
2. Fill in your bot token, allowed chat ID, and serial port
3. Make sure the Arduino sketch is uploaded and the serial port is free

Example `.env` values:

```text
TELEGRAM_BOT_TOKEN=123456:example
TELEGRAM_ALLOWED_CHAT_ID=123456789
ROBOT_ARM_SERIAL_PORT=/dev/cu.usbmodem1101
ROBOT_ARM_BAUDRATE=115200
```

Supported Telegram commands:

- `/start`
- `/list`
- `/state`
- `/home`
- `/stop`
- `/pickup`
- `/place`
- `/auto_blue_on`
- `/auto_blue_off`
- `/auto_blue_status`
- `/blue_nod_on`
- `/blue_nod_off`
- `/run <primitive>`
- `/move <joint> <angle>`
- `/step <joint> <delta>`
- `/spin rotate <speed>`

Natural language examples:

- `go home`
- `pick up`
- `place it`
- `find blue and nod`
- `stop blue nod`
- `auto pick blue`
- `move shoulder to 95`
- `step elbow -4`

## Agent Control

For OpenClaw or another local agent, prefer:

```bash
python -m control.agent_api --input '{"action":"state"}'
```

See [openclaw.md](./openclaw.md) for the full command set.
