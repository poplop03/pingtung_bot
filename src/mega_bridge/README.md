# mega_bridge

The Jetson side of the link to the Arduino Mega (`Arduino/mega_bridge/mega_bridge.ino`),
which drives every actuator: the two DC wheels (JZ2407DB), the two gantry
steppers and the gripper servo. This node is the only process that opens the
Mega's serial port.

```
/mega/cmd_vel ─► wheel_control (IMU heading loop) ─► /mega/wheel_pwm ─┐
/mega/step ────────────────────────────────────────────────────────────┼─► mega_bridge ◄─► USB ◄─► Mega
/mega/gripper ─────────────────────────────────────────────────────────┘        │
                                                                                └─► /mega/status
```

## Topics

| topic | type | meaning |
|---|---|---|
| `/mega/cmd_vel` | `geometry_msgs/Twist` | base speed. Handled by `wheel_control` with `output: topic`, not by this node |
| `/mega/step` | `std_msgs/Int32MultiArray` `[step1, step2]` | **relative** steps. They are added to the current target, so `[1600, 0]` sent twice moves 3200 |
| `/mega/gripper` | `std_msgs/Float32` | servo angle in degrees, clamped to 0–180 and eased at 1°/15 ms |
| `/mega/wheel_pwm` | `std_msgs/Int16MultiArray` `[m1, m2]` | wheel PWM −255..255, from `wheel_control` |
| `/mega/status` (out) | `std_msgs/Int32MultiArray` | 10 Hz: `[steps_left1, steps_left2, servo_deg, wheel_failsafe]` |

A gantry move is finished when both `steps_left` read 0. That is the signal to
close the gripper.

```bash
ros2 topic pub --once /mega/step std_msgs/Int32MultiArray "{data: [1600, -800]}"
ros2 topic pub --once /mega/gripper std_msgs/Float32 "{data: 45.0}"
ros2 topic echo /mega/status
```

## Safety

| condition | action |
|---|---|
| no `/mega/wheel_pwm` for `pwm_timeout_s` (0.5 s) | bridge sends wheel PWM 0, and the firmware brakes on a ramp |
| no DRIVE frame for 300 ms (bridge dead, cable out) | **firmware** ramps the wheels to a stop and sets `wheel_failsafe` |
| no `/mega/status` for 1 s | bridge logs a warning (wrong port, or old firmware) |

The steppers and the servo are not stopped by the failsafe: they finish the
move they were given.

The bridge does not reopen the port. After unplugging the Mega, restart the node.

## Serial protocol

`0xAA 0xA5 | type | len | payload | crc8(type, len, payload)`, little-endian,
with the same crc8 as the ESP32 link. The frame types are listed in
`mega_bridge/protocol.py` and at the top of the sketch, and the two must match.

## Bring-up

1. Install **AccelStepper** from the Arduino Library Manager, then flash
   `Arduino/mega_bridge/mega_bridge.ino` to the Mega.
2. Wiring is listed at the top of the sketch. Wheels: IN1/IN2/EN on D22/D23/D11
   and D24/D25/D12. Steppers: PUL/DIR on D2/D3 and D5/D6. Servo on D8. D9/D10
   stay free for the pump. **Tie all grounds together.**
3. The JZ2407DB used to run from the ESP32's 3.3 V logic. Check that it
   accepts 5 V on IN/EN.
4. The Mega resets when the port opens, so the 2 s pause at startup is expected.
5. With the wheels off the ground:
   ```bash
   ros2 launch mega_bridge mega_bridge.launch.py port:=/dev/ttyACM0
   ros2 topic pub -r 20 /mega/wheel_pwm std_msgs/Int16MultiArray "{data: [60, 60]}"
   ```
   Stop the publisher and the wheels should brake within about 0.5 s. Do not
   run `wheel_control` during this test, because it also publishes
   `/mega/wheel_pwm`.
6. Full stack: `ros2 launch pingtung_bot_bringup bringup.launch.py`, then drive
   with `/mega/cmd_vel`.

PWM on the Mega runs at ≈3.9 kHz instead of the ESP32's 5 kHz. Re-check
`min_pwm_left`/`min_pwm_right` in `wheel_control.yaml` after switching.

## Test

```bash
colcon build --packages-select wheel_control mega_bridge
colcon test --packages-select mega_bridge && colcon test-result --verbose
```
