#!/usr/bin/env python3
"""Emulates the AgroBot motor-controller board on a virtual serial port.

Lets you run the complete real-hardware stack (hardware:=real) without the
board: the serial protocol, calibration, watchdog and e-stop handling are the
same as with firmware/agrobot_mcu.

    ros2 run aibomech_agrobot_hardware mcu_emulator.py --link /tmp/agrobot_mcu
    ros2 launch aibomech_agrobot_bringup robot.launch.py hardware:=real serial_port:=/tmp/agrobot_mcu

Press Ctrl+C to stop. Send SIGUSR1 to toggle the emulated e-stop:
    pkill -USR1 -f mcu_emulator.py
"""
import argparse
import os
import pty
import select
import signal
import time
import tty

CHANNELS = 6
LIMIT_LO = [-1.09, -2.10, -2.00, -1.00, -0.017, 0.0]
LIMIT_HI = [2.10, 1.11, 1.10, 2.30, 0.010, 3.0]
MAX_SPEED = [2.0, 2.0, 2.0, 3.0, 0.05, 0.30]
WATCHDOG_S = 0.25
STATE_PERIOD_S = 0.02
ENABLED, ESTOP, WATCHDOG, FAULT, RAIL_NOT_HOMED = 1, 2, 4, 8, 16


def checksum(payload):
    value = 0
    for ch in payload.encode():
        value ^= ch
    return value


def frame(payload):
    return f'${payload}*{checksum(payload):02X}\n'.encode()


class Board:
    def __init__(self, homing_time):
        self.position = [0.0] * CHANNELS
        self.position[4] = LIMIT_HI[4]
        self.target = list(self.position)
        self.velocity = [0.0] * CHANNELS
        self.enabled = False
        self.estop_pressed = False
        self.estop_latched = False
        self.homing_left = homing_time
        self.last_command = 0.0
        self.sequence = 0

    def handle(self, line):
        if not line.startswith('$') or '*' not in line:
            return None
        payload, _, received = line[1:].rpartition('*')
        try:
            if int(received[:2], 16) != checksum(payload):
                return None
        except ValueError:
            return None
        fields = payload.split(',')
        if fields[0] == 'C' and len(fields) == CHANNELS + 1:
            for i, value in enumerate(fields[1:]):
                self.target[i] = min(max(float(value), LIMIT_LO[i]), LIMIT_HI[i])
            self.last_command = time.monotonic()
            return None
        if fields[0] == 'E' and len(fields) == 2:
            on = fields[1] == '1'
            if on and self.estop_pressed:
                return self.state()
            if on:
                self.estop_latched = False
                self.target = list(self.position)
                self.last_command = time.monotonic()
            self.enabled = on
        return self.state()

    def step(self, dt):
        if self.estop_pressed and not self.estop_latched:
            self.estop_latched = True
            self.enabled = False
        hold = not self.enabled or time.monotonic() - self.last_command > WATCHDOG_S
        if self.enabled and self.homing_left > 0:
            self.homing_left -= dt
        for i in range(CHANNELS):
            goal = self.position[i] if hold or (i == 5 and self.homing_left > 0) else self.target[i]
            delta = max(-MAX_SPEED[i] * dt, min(MAX_SPEED[i] * dt, goal - self.position[i]))
            self.position[i] += delta
            self.velocity[i] = delta / dt

    def state(self):
        status = 0
        if self.enabled:
            status |= ENABLED
        if self.estop_latched:
            status |= ESTOP
        if self.enabled and time.monotonic() - self.last_command > WATCHDOG_S:
            status |= WATCHDOG
        if self.homing_left > 0:
            status |= RAIL_NOT_HOMED
        values = ','.join(f'{v:.4f}' for v in self.position + self.velocity)
        self.sequence += 1
        return frame(f'S,{self.sequence},{status},{values}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--link', default='/tmp/agrobot_mcu', help='symlink to create for the virtual port')
    parser.add_argument('--homing-time', type=float, default=2.0, help='seconds the rail needs to home')
    args, _ = parser.parse_known_args()

    master, slave = pty.openpty()
    tty.setraw(slave)
    if os.path.islink(args.link):
        os.unlink(args.link)
    os.symlink(os.ttyname(slave), args.link)
    print(f'AgroBot MCU emulator on {os.ttyname(slave)} (link {args.link})', flush=True)

    board = Board(args.homing_time)

    def toggle_estop(*_):
        board.estop_pressed = not board.estop_pressed
        print(f'e-stop {"PRESSED" if board.estop_pressed else "released"}', flush=True)
    signal.signal(signal.SIGUSR1, toggle_estop)

    buffer = b''
    last_step = last_state = time.monotonic()
    try:
        while True:
            ready, _, _ = select.select([master], [], [], 0.002)
            if ready:
                buffer += os.read(master, 1024)
                *lines, buffer = buffer.split(b'\n')
                for raw in lines:
                    reply = board.handle(raw.decode(errors='ignore').strip())
                    if reply:
                        os.write(master, reply)
            now = time.monotonic()
            if now - last_step >= 0.002:
                board.step(now - last_step)
                last_step = now
            if now - last_state >= STATE_PERIOD_S:
                os.write(master, board.state())
                last_state = now
    except KeyboardInterrupt:
        pass
    finally:
        if os.path.islink(args.link):
            os.unlink(args.link)


if __name__ == '__main__':
    main()
