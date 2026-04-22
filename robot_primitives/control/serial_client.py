from __future__ import annotations

import json
import os
import time
from typing import Any

import serial


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


class RobotArmSerialClient:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 1.0,
        reset_before_command: bool | None = None,
    ) -> None:
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.reset_before_command = (
            _env_bool("ROBOT_ARM_RESET_BEFORE_COMMAND", False)
            if reset_before_command is None
            else reset_before_command
        )
        time.sleep(0.5)
        self._wait_for_board_ready(timeout_s=6.0)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def close(self) -> None:
        self.serial.close()

    def _readline(self) -> str:
        raw = self.serial.readline().decode("utf-8", errors="replace").strip()
        return raw

    def _is_background_line(self, line: str) -> bool:
        if line in {"OK RESET", "OpenClaw primitive bridge ready"}:
            return True
        if line.startswith("BOOT "):
            return True
        if line == "error: no response":
            return True
        if line.startswith("AUTO ") and not line.startswith("AUTO_BLUE "):
            return True
        if line == "BLUE detected":
            return True
        return False

    def _read_until_line(self, predicate, timeout_s: float) -> str:
        deadline = time.time() + timeout_s
        last_line = ""
        while time.time() < deadline:
            line = self._readline()
            if not line:
                continue
            last_line = line
            if self._is_background_line(line):
                continue
            if predicate(line):
                return line
        raise TimeoutError(f"Timed out waiting for serial reply. Last line: {last_line!r}")

    def _wait_for_board_ready(self, timeout_s: float) -> None:
        deadline = time.time() + timeout_s
        last_line = ""
        while time.time() < deadline:
            line = self._readline()
            if not line:
                continue
            last_line = line
            if line == "OpenClaw primitive bridge ready":
                return

        # Some boards do not reset on serial open. In that case, continue and let
        # the actual command path prove whether the board is responsive.
        if last_line:
            return

    def _try_read_reset_ack(self, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            line = self._readline()
            if not line:
                continue
            if line == "OK RESET":
                return True
            # If the board sends some other startup/status text instead of a RESET ack,
            # don't fail the whole command path. We'll still send the real command next.
        return False

    def send(
        self,
        command: str,
        expect_reply: bool = True,
        reply_timeout_s: float = 3.0,
    ) -> str:
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        if self.reset_before_command and command.strip().upper() != "PING":
            self.serial.write(b"RESET\n")
            self.serial.flush()
            self._try_read_reset_ack(timeout_s=1.0)
            time.sleep(0.05)

        self.serial.write((command.strip() + "\n").encode("utf-8"))
        self.serial.flush()

        if not expect_reply:
            return ""

        return self._read_until_line(
            lambda line: line.startswith(("OK ", "ERR ", "STATE ", "AUTO_BLUE ", "PONG", "I2C ")),
            timeout_s=reply_timeout_s,
        )

    def move(self, joint: str, angle: int) -> str:
        return self.send(f"MOVE {joint} {angle}")

    def ping(self) -> str:
        return self.send("PING")

    def i2c_scan(self) -> str:
        return self.send("I2C_SCAN", reply_timeout_s=10.0)

    def i2c_probe(self) -> str:
        return self.send("I2C_PROBE", reply_timeout_s=5.0)

    def servo_test(self, joint: str) -> str:
        return self.send(f"SERVO_TEST {joint} 0", reply_timeout_s=8.0)

    def step(self, joint: str, delta: int) -> str:
        return self.send(f"STEP {joint} {delta}")

    def spin(self, joint: str, speed: int) -> str:
        return self.send(f"SPIN {joint} {speed}")

    def stepper_move(self, steps: int) -> str:
        return self.send(f"STEPPER move {steps}", reply_timeout_s=10.0)

    def stepper_enable(self, enabled: bool) -> str:
        return self.send(f"STEPPER enable {1 if enabled else 0}")

    def home(self) -> str:
        return self.send("HOME", reply_timeout_s=20.0)

    def stop(self) -> str:
        return self.send("STOP", reply_timeout_s=20.0)

    def pickup(self) -> str:
        return self.send("PICKUP", reply_timeout_s=20.0)

    def place(self) -> str:
        return self.send("PLACE", reply_timeout_s=20.0)

    def auto_blue_on(self) -> str:
        return self.send("AUTO_BLUE_ON")

    def auto_blue_off(self) -> str:
        return self.send("AUTO_BLUE_OFF")

    def auto_blue_status(self) -> str:
        return self.send("AUTO_BLUE_STATUS")

    def blue_nod_on(self) -> str:
        return self.send("BLUE_NOD_ON")

    def blue_nod_off(self) -> str:
        return self.send("BLUE_NOD_OFF")

    def get_state(self) -> dict[str, Any]:
        reply = self.send("GET_STATE")
        if not reply.startswith("STATE "):
            raise ValueError(f"Unexpected state reply: {reply}")
        return json.loads(reply[6:])
