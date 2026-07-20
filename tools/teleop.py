#!/usr/bin/env python3
"""Numpad teleop client for the ESP32's USB serial link.

Streams numpad 1-9 (translate, 8 compass directions around 5=stop) and o/p
(rotate CCW/CW) at a fixed rate while held, matching
MrKrabs::handleTeleopChar / computeTeleopVelocity on the firmware side
(esp-32/src/mr_krabs.cpp), which infers "key held" from the character
re-arriving within TELEOP_KEY_TIMEOUT_US (300 ms). This talks directly to the
ESP32 over USB serial, bypassing the RPi/AI entirely.

Numpad layout:
    7  8  9      forward-left  forward  forward-right
    4  5  6      left          stop     right
    1  2  3      back-left     back     back-right

Usage:
    python3 tools/teleop.py --port /dev/ttyUSB0
    python3 tools/teleop.py --list          # show available serial ports

Controls: numpad 1-9 translate (5/space = stop), o/p rotate CCW/CW, Esc = quit.

Note (Linux): pynput's keyboard listener needs an active X11 session (or
root access to /dev/input) to capture global key events — it will not work
over a headless SSH-only terminal. On macOS, grant your terminal
Accessibility permission when prompted. Numpad digits are read via key.char,
so NumLock must be on (same behavior as the top-row digit keys).
"""

import argparse
import sys
import threading
import time

import serial
from pynput import keyboard

BAUD = 115200
SEND_INTERVAL_S = 0.1  # well under the firmware's 300 ms per-key timeout

TELEOP_HOLD_KEYS = {"1", "2", "3", "4", "6", "7", "8", "9", "o", "p"}
STOP_KEYS = {"5", " "}


def list_ports():
    from serial.tools import list_ports as lp

    ports = list(lp.comports())
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        print(f"{p.device}  {p.description}")


class Teleop:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, BAUD, timeout=0)
        time.sleep(2)  # let the ESP32 finish its boot reset over USB CDC
        self.held = set()
        self.lock = threading.Lock()
        self.running = True

    def on_press(self, key):
        try:
            c = key.char.lower()
        except AttributeError:
            c = None

        if key == keyboard.Key.esc:
            self.running = False
            return False  # stop listener

        if key == keyboard.Key.space or c in STOP_KEYS:
            with self.lock:
                self.held.clear()
            self.ser.write(b"5")
            return

        if c in TELEOP_HOLD_KEYS:
            with self.lock:
                self.held.add(c)

    def on_release(self, key):
        try:
            c = key.char.lower()
        except AttributeError:
            return
        if c in TELEOP_HOLD_KEYS:
            with self.lock:
                self.held.discard(c)

    def sender_loop(self):
        while self.running:
            with self.lock:
                keys = bytes("".join(self.held), "ascii")
            if keys:
                self.ser.write(keys)
            time.sleep(SEND_INTERVAL_S)

    def run(self):
        sender = threading.Thread(target=self.sender_loop, daemon=True)
        sender.start()
        print("Teleop active — numpad 1-9 translate, o/p rotate, 5/space=stop, Esc=quit")
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()
        self.running = False
        self.ser.write(b"5")  # stop the robot on exit
        self.ser.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="Serial port the ESP32 is on, e.g. /dev/ttyUSB0 or COM3")
    parser.add_argument("--list", action="store_true", help="List available serial ports and exit")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if not args.port:
        parser.error("--port is required (use --list to see available ports)")

    Teleop(args.port).run()


if __name__ == "__main__":
    main()
