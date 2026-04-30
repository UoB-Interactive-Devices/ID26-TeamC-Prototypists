#!/usr/bin/env python3
"""
Faster robot control - keeps serial connection open between commands.
Usage: python fast_control.py cmd1 cmd2 cmd3 ...
"""
import sys
import time
from control.serial_client import RobotArmSerialClient

PORT = "/dev/cu.usbmodem101"
BAUD = 115200

def run():
    client = RobotArmSerialClient(port=PORT, baudrate=BAUD)
    args = sys.argv[1:]
    
    for arg in args:
        parts = arg.split(":", 1)
        action = parts[0]
        
        if action == "pickup":
            r = client.pickup()
        elif action == "pickup_1":
            r = client.pickup_1()
        elif action == "pickup_2":
            r = client.pickup_2()
        elif action == "place":
            r = client.place()
        elif action == "place_1":
            r = client.place_1()
        elif action == "place_2":
            r = client.place_2()
        elif action == "home":
            r = client.home()
        elif action == "stop":
            r = client.stop()
        elif action == "state":
            r = client.get_state()
        elif action.startswith("move_"):
            joint = action.replace("move_", "")
            angle = int(parts[1]) if len(parts) > 1 else 90
            r = client.move(joint, angle)
        elif action.startswith("step_"):
            parts2 = action.split("_")
            joint = parts2[1]
            delta = int(parts2[2]) if len(parts2) > 2 else 0
            r = client.step(joint, delta)
        elif action == "auto_blue_on":
            r = client.auto_blue_on()
        elif action == "auto_blue_off":
            r = client.auto_blue_off()
        elif action == "blue_nod_on":
            r = client.blue_nod_on()
        elif action == "blue_nod_off":
            r = client.blue_nod_off()
        else:
            print(f"Unknown: {action}")
            continue
        print(f"OK {action}: {r}")
    
    client.close()

if __name__ == "__main__":
    run()