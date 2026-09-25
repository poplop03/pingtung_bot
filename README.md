# pingtung_bot

ROS 2 Humble workspace for a two-wheel mobile robot with a two-axis gantry and a
servo gripper. The **Jetson Orin NX** runs ROS 2 and the higher-level logic. An
**Arduino Mega** is the low-level controller: it takes commands from the Jetson
over USB serial and drives every actuator.

## Hardware

| part | connected to | role |
|---|---|---|
| Jetson Orin NX | — | ROS 2, control loops, task logic |
| BNO055 IMU | Jetson I²C bus 7, address `0x28` | yaw rate for the wheel heading loop |
| Arduino Mega 2560 | Jetson USB (`/dev/ttyACM0`) | real-time actuator driver |
| 2 × DC wheel motors | Mega → JZ2407DB dual driver | differential base |
| 2 × stepper motors | Mega → step/dir drivers | gantry axes |
| 1 × MG996R servo | Mega D8 | gripper |

Pinout and wiring notes are at the top of
[Arduino/mega_bridge/mega_bridge.ino](Arduino/mega_bridge/mega_bridge.ino).

## Architecture

```
 JETSON ORIN NX (ROS 2)                                                  │  ARDUINO MEGA
                                                                         │
 bno055 ──/bno055/imu──┐                                                 │
                       ▼                                                 │
 /mega/cmd_vel ──► wheel_control ──/mega/wheel_pwm──┐                    │  wheels   ─► JZ2407DB
   (Twist)         heading hold + yaw-rate PI       ▼                    │  (ramped brake, failsafe)
                                                                  USB    │
 /mega/step ──────────────────────────────────► mega_bridge ◄───────────►│  steppers ─► gantry
 /mega/gripper ───────────────────────────────►  (owns the               │  (AccelStepper)
                                                 serial port)            │
 /mega/status ◄─────────────────────────────────────┘                    │  servo    ─► gripper
```

The work is split three ways:

- **Mega:** timing-critical work only. It generates step pulses, eases the
  servo, runs the brake ramps and enforces the wheel failsafe. It holds no
  robot-level logic, so control can be retuned without reflashing.
- **wheel_control:** the wheel control loop. It needs the IMU, so it runs on
  the Jetson. It turns a `Twist` into left/right PWM and holds a straight
  heading using the gyro.
- **mega_bridge:** the only process that opens the Mega's serial port. It
  converts ROS messages to and from the framed binary protocol. It contains no
  control logic.

## Topics

This is the interface the rest of the software uses:

| topic | type | direction | meaning |
|---|---|---|---|
| `/mega/cmd_vel` | `geometry_msgs/Twist` | in | base motion. Only the **sign** of `linear.x` is used (see *Limits*). `angular.z` is in rad/s, + = left |
| `/mega/step` | `std_msgs/Int32MultiArray` `[step1, step2]` | in | **relative** steps for each stepper, added to the pending move. 1600 steps = 1 rev at 1/8 microstep |
| `/mega/gripper` | `std_msgs/Float32` | in | servo angle in degrees, clamped to 0–180 |
| `/mega/status` | `std_msgs/Int32MultiArray` | out, 10 Hz | `[steps_left1, steps_left2, servo_deg, wheel_failsafe]` |

Internal topics:

| topic | type | meaning |
|---|---|---|
| `/bno055/imu` | `sensor_msgs/Imu` | IMU data for `wheel_control` |
| `/mega/wheel_pwm` | `std_msgs/Int16MultiArray` `[m1, m2]` | wheel PWM −255..255, from `wheel_control` to `mega_bridge` |
| `/wheel_control/debug` | `std_msgs/Float32MultiArray` | loop internals for tuning (see the wheel_control README) |

A typical pick sequence: publish `/mega/step`, wait until both `steps_left` in
`/mega/status` are 0, then publish `/mega/gripper`.

## Serial link (Jetson ↔ Mega)

The link is USB serial at 115200 baud, carrying binary frames:

```
0xAA 0xA5 | type | len | payload | crc8(type, len, payload)      little-endian, crc8 poly 0x07
```

| type | direction | payload |
|---|---|---|
| `0x01` DRIVE | → Mega, streamed at 50 Hz | int16 m1, int16 m2 |
| `0x02` STEP | → Mega, once per message | int32 s1, int32 s2 |
| `0x03` SERVO | → Mega, once per message | uint8 degrees |
| `0x81` STATUS | ← Mega, 10 Hz | int32 rem1, int32 rem2, uint8 servo, uint8 flags |

Frames with a bad CRC are dropped, never acted on. The two definitions,
[protocol.py](src/mega_bridge/mega_bridge/protocol.py) and the sketch, **must
match**. Change them together.

## Safety: where the wheels stop

| layer | trigger | action |
|---|---|---|
| wheel_control | no `/mega/cmd_vel` for 0.5 s | commands 0, or holds heading if `idle_hold` is on |
| wheel_control | no IMU for 0.3 s, or gyro not yet calibrated | commands 0 (never drives without feedback) |
| mega_bridge | no `/mega/wheel_pwm` for 0.5 s | sends PWM 0 |
| Mega firmware | no DRIVE frame for 300 ms (bridge dead, cable out) | ramped brake, sets `wheel_failsafe` |

The steppers and servo are not covered by these: they finish the move they were
given. At startup `wheel_control` calibrates gyro bias from 200 samples, so
**keep the robot still for the first few seconds**.

## Repository layout

```
pingtung_bot/
├── Arduino/
│   ├── mega_bridge/          Mega firmware: wheels + steppers + servo   ← flash this
│   ├── mega_motor_control/   old keyboard test sketch for steppers/servo (legacy)
│   └── pingtung_contest/     ESP32 wheel firmware from before the Mega (legacy)
└── src/                      ROS 2 packages (colcon workspace)
    ├── pingtung_bot_bringup/ launch file that starts the whole robot
    ├── mega_bridge/          serial bridge to the Mega, /mega/* topics
    ├── wheel_control/        closed-loop base controller (IMU heading hold)
    ├── wheel_open_loop/      open-loop baseline for comparison (ESP32 only)
    └── bno055/               BNO055 IMU driver (upstream 0.5.0, vendored)
```

Each package has its own README for details:
[mega_bridge](src/mega_bridge/README.md),
[wheel_control](src/wheel_control/README.md) (tuning, the reasoning behind the
two loops, idle hold) and
[wheel_open_loop](src/wheel_open_loop/README.md).

`wheel_control` can still drive the old ESP32 directly (`output: serial`, its
default). The bringup launch sets `output: topic` so the Mega path is used.

## Build and run

```bash
# 1. firmware: install AccelStepper (Arduino Library Manager), then flash
#    Arduino/mega_bridge/mega_bridge.ino to the Mega

# 2. workspace
cd ~/pingtung_bot
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash

# 3. whole robot (IMU + wheel_control + mega_bridge) - keep it still while the gyro calibrates
ros2 launch pingtung_bot_bringup bringup.launch.py

# 4. command it
ros2 topic pub -r 10 /mega/cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"
ros2 topic pub --once /mega/step std_msgs/Int32MultiArray "{data: [1600, -800]}"
ros2 topic pub --once /mega/gripper std_msgs/Float32 "{data: 45.0}"
ros2 topic echo /mega/status
```

To test the actuators without the IMU or control loop, run only the bridge:
`ros2 launch mega_bridge mega_bridge.launch.py`. Then publish `/mega/wheel_pwm`
directly, with the wheels off the ground.

Tests: `colcon test --packages-select mega_bridge && colcon test-result --verbose`

## Limits

- **No linear speed control.** The wheels have no encoders, so forward motion
  is a fixed `base_pwm` and only the direction of `linear.x` counts. Distance
  varies with battery, load and floor.
- **Heading drifts slowly.** It is integrated from the gyro because the
  magnetometer is unreliable next to the motors. Bias is re-estimated whenever
  the robot is parked.
- **Steppers are open loop.** With no limit switches or homing, position is
  counted from wherever the gantry was at power-up.
- **No automatic reconnect.** If the Mega is unplugged, restart `mega_bridge`.
- **Retune after moving from the ESP32.** Mega PWM runs at ≈3.9 kHz rather
  than 5 kHz, so re-measure `min_pwm_left` and `min_pwm_right`.
