# aibomech_agrobot_hardware

Everything needed to drive the real robot:

| Part | File | Content |
|---|---|---|
| ros2_control driver | `src/agrobot_system.cpp` | `SystemInterface` plugin: joint setpoints out, positions, velocities and status in; calibration, watchdog, e-stop handling |
| Serial protocol | `include/.../protocol.hpp`, `src/protocol.cpp` | ASCII frames with XOR checksums, readable in any serial monitor |
| Firmware | `firmware/agrobot_mcu/agrobot_mcu.ino` | motor-controller board (Arduino Mega, ESP32 or Teensy): servos, rail stepper, homing, e-stop input |
| Board emulator | `scripts/mcu_emulator.py` | a virtual board on a pseudo-terminal, to test the full real-hardware stack without motors |
| Tests | `test/test_protocol.cpp` | protocol unit tests (`colcon test`) |

Try the real-hardware path without hardware:

```bash
ros2 run aibomech_agrobot_hardware mcu_emulator.py --link /tmp/agrobot_mcu
ros2 launch aibomech_agrobot_bringup robot.launch.py hardware:=real serial_port:=/tmp/agrobot_mcu
```

Wiring, flashing, calibration and first-motion checklists: [docs/real_robot.md](../../../docs/real_robot.md). Data path and protocol diagrams: [docs/architecture.md](../../../docs/architecture.md#real-robot-data-path).
