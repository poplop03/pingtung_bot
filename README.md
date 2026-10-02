# pingtung_bot

Software for an agricultural field robot, presented at the **2026 Conference on
Agricultural Machinery and Biomechatronics Engineering** (October 01–02, 2026).

The robot is a small two-wheel mobile base that carries a two-axis gantry with
a servo gripper, a USB colour camera and an Intel RealSense D415 depth camera.
It runs several independent missions:

| mission | what the robot does | package |
|---|---|---|
| Fruit picking | drives along a row and strikes every red or yellow fruit with the gantry, without stopping | [`pick_fruit`](src/pick_fruit/README.md) |
| Lane course | drives a U-shaped course (lane, U-turn, lane) and steers around obstacles ("pigs") in the lanes | [`lane_runner`](src/lane_runner/README.md) |
| All-terrain race | follows the centre line of a road lined with posts and reflectors, over gravel, grass and ramps, through a hairpin | [`race_terain`](src/race_terain/README.md) |
| Scripted drive | replays a drive that was recorded by hand | [`run_by_scenario`](src/run_by_scenario/README.md) |
| Vision tasks | recognises animals, fruit colour, and pigs vs. manure from the camera | [`pingtung_vision`](src/pingtung_vision/README.md) |

Everything is written for **ROS 2 Humble** in Python. Each mission starts with
one launch command and has a phone-friendly web page for START/STOP, live
camera views and tuning, so no laptop or terminal is needed in the field.

> **New to ROS 2?** A *node* is one running program. Nodes exchange data by
> publishing and subscribing to named *topics*, such as `/mega/cmd_vel`. A
> *package* is a folder holding one or more nodes plus their launch and config
> files. A *launch file* starts a set of nodes together.

---

## Contents

1. [Hardware](#hardware)
2. [System architecture](#system-architecture)
3. [How the missions are wired](#how-the-missions-are-wired)
4. [The packages, one by one](#the-packages-one-by-one)
5. [Topic reference](#topic-reference)
6. [Serial link (Jetson ↔ Mega)](#serial-link-jetson--mega)
7. [Safety: where the wheels stop](#safety-where-the-wheels-stop)
8. [Build and run](#build-and-run)
9. [Limits](#limits)
10. [Repository layout](#repository-layout)
11. [License](#license)

---

## Hardware

The **Jetson Orin NX** runs ROS 2, perception and all mission logic. The
**Arduino Mega** is the low-level controller: it receives commands from the
Jetson over USB serial and drives every actuator with exact timing.

| part | connected to | role |
|---|---|---|
| Jetson Orin NX | — | ROS 2, control loops, perception, mission logic |
| BNO055 IMU | Jetson I²C bus 7, address `0x28` | yaw rate for the wheel heading loop |
| Intel RealSense D415 | Jetson USB | depth images for `lane_runner` and `race_terain` |
| USB colour camera | Jetson USB (`/dev/video4`) | images for `pingtung_vision` / `pick_fruit` |
| Arduino Mega 2560 | Jetson USB (`/dev/ttyACM0`) | real-time actuator driver |
| 2 × DC wheel motors | Mega → JZ2407DB dual driver | differential base |
| 2 × stepper motors | Mega → step/dir drivers | gantry axes |
| 1 × MG996R servo | Mega D8 | gripper |

The wheels have **no encoders**. Forward speed is open loop and set as a motor
PWM value; turning is closed loop on the IMU gyro. This choice shapes much of
the software (see [Limits](#limits)).

### Arduino Mega pinout

| Mega pin | connects to | signal | notes |
|---|---|---|---|
| D0 / D1 | USB serial | RX / TX to the Jetson | used by the USB link, leave unconnected |
| D2 | stepper driver 1 `PUL+` | step pulse | gantry axis 1 |
| D3 | stepper driver 1 `DIR+` | direction | |
| D5 | stepper driver 2 `PUL+` | step pulse | gantry axis 2 |
| D6 | stepper driver 2 `DIR+` | direction | |
| D8 | MG996R signal (orange) | servo PWM | gripper; servo power comes from a separate 5–6 V supply |
| D9 / D10 | — | — | **reserved** for the water pump PWM |
| D11 | JZ2407DB motor 1 `EN` | wheel PWM | Timer1, ≈3.9 kHz |
| D12 | JZ2407DB motor 2 `EN` | wheel PWM | Timer1, ≈3.9 kHz |
| D22 | JZ2407DB motor 1 `IN1` | direction A | |
| D23 | JZ2407DB motor 1 `IN2` | direction B | |
| D24 | JZ2407DB motor 2 `IN1` | direction A | |
| D25 | JZ2407DB motor 2 `IN2` | direction B | |
| GND | driver GND, stepper `PUL−`/`DIR−`, servo supply − | common ground | **mandatory**: every supply shares this ground |

- **Stepper drivers:** `PUL−` and `DIR−` go to GND. The drivers are wired
  common-cathode, so an output driven HIGH is active.
- **JZ2407DB:** it was previously driven by an ESP32's 3.3 V logic. Check the
  datasheet to confirm its IN/EN inputs accept the Mega's 5 V.
- **Spare pins:** the sketch speeds up Timer1 for the wheel EN pins, so don't
  add a library that also uses Timer1. The Servo library occupies Timer5, so
  D44–D46 have no PWM.
- **Changing a pin:** edit the constants at the top of
  [mega_bridge.ino](Arduino/mega_bridge/mega_bridge.ino) and update this table
  too.

---

## System architecture

The software is built in four layers. Each layer only talks to the one below
it through ROS topics, so a mission never touches the serial port or the
motors directly.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ 4. MISSIONS        pick_fruit   lane_runner   race_terain   run_by_scenario  │
│    (one at a time, each with a web UI on port 8080)                          │
├──────────────────────────────────────────────────────────────────────────────┤
│ 3. PERCEPTION      pingtung_vision (USB camera)     RealSense D415 driver    │
│                    + pingtung_vision_interfaces     (depth, read directly by │
│                      (custom result messages)        lane_runner/race_terain)│
├──────────────────────────────────────────────────────────────────────────────┤
│ 2. MOTION CONTROL  wheel_control: Twist → left/right PWM, IMU heading hold   │
│                    bno055: IMU driver                                        │
├──────────────────────────────────────────────────────────────────────────────┤
│ 1. HARDWARE LINK   mega_bridge: the only process that opens the Mega's port  │
│                    Arduino/mega_bridge firmware: wheels, steppers, servo     │
└──────────────────────────────────────────────────────────────────────────────┘
        pingtung_bot_bringup starts layers 1 + 2 with one command
```

A drawn version of layers 1 and 2 is in
[docs/architecture.svg](docs/architecture.svg).

### Layers 1 and 2: the robot base (always running)

Every mission includes `pingtung_bot_bringup`, which starts these three nodes:

```
 JETSON ORIN NX (ROS 2)                                                  │  ARDUINO MEGA
                                                                         │
 bno055 ──/bno055/imu──┐                                                 │
                       ▼                                                 │
 /mega/cmd_vel ──► wheel_control ──/mega/wheel_pwm──┐                    │  wheels   ─► JZ2407DB
   (Twist)         heading hold + yaw-rate PI       ▼                    │  (ramped brake, failsafe)
                                                                  USB    │
 /mega/step ──────────────────────────────────► mega_bridge ◄───────────►│  steppers ─► gantry
 /mega/goto ──────────────────────────────────►  (owns the               │  (AccelStepper)
 /mega/gripper ───────────────────────────────►   serial port)           │
 /mega/status ◄─────────────────────────────────────┘                    │  servo    ─► gripper
 /mega/set_home (service) ─────────────────────────►                     │
```

The work is split three ways on purpose:

- **Arduino Mega:** timing-critical work only. It generates step pulses, eases
  the servo, ramps the wheel brakes and enforces the wheel failsafe. It holds
  no robot-level logic, so control can be retuned without reflashing.
- **wheel_control:** the wheel control loop. It needs the IMU, so it runs on
  the Jetson. It turns a velocity command (`Twist`) into left/right PWM and
  holds a straight heading using the gyro.
- **mega_bridge:** the only process that opens the Mega's serial port. It
  converts ROS messages to and from the binary serial protocol. It contains no
  control logic.

The result is a small, stable interface: **any mission drives the robot by
publishing `/mega/cmd_vel`, and moves the gantry with `/mega/goto` or
`/mega/step`.** That interface is the same for every mission.

---

## How the missions are wired

Each mission has its own launch file that starts the base (bringup), the
sensor it needs, and its own node. Only **one mission runs at a time**: they
all publish `/mega/cmd_vel`, and they share the camera and web port 8080.

### Fruit picking — `ros2 launch pick_fruit pick_fruit.launch.py`

```
 USB camera ─► v4l2_camera ─/camera/image_raw─► vision (fruit_color mode)
                                                  │            │
                               /vision/fruit/line_hit   /vision/debug_image
                               (FruitLineHit: 0 or 1)          │
                                                  ▼            ▼
                                          pick_fruit_node ── web UI :8080
                                           │   │   │   ▲
                         /mega/cmd_vel ◄───┘   │   │   └── /mega/status
                         /mega/goto    ◄───────┘   │
                         /mega/step, /mega/gripper ◄┘ (manual jog only)
                                           ▼
                                   base (bringup)
```

The robot drives straight. When the vision node reports a red/yellow fruit on
the hit line, the gantry moves to the saved `hit` position and back to
`ready`, while the robot keeps driving at reduced speed.

### Lane course — `ros2 launch lane_runner lane_runner.launch.py`

```
 RealSense D415 ─/camera/camera/depth/image_rect_raw─┐
                                                     ▼
 bno055 ───────────────/bno055/imu──────────► lane_runner_node ── web UI :8080
                                                     │
                                       /mega/cmd_vel ▼
                                              base (bringup)
```

States: `IDLE → CALIBRATE → LANE1 → PIVOT1 → CROSS → PIVOT2 → LANE2 → DONE`.
The depth image is turned into 3D points, the floor plane is fitted, and
anything 4–50 cm above the floor counts as an obstacle. The robot steers for
the centre of the free gap it fits through.

### All-terrain race — `ros2 launch race_terain race_terain.launch.py`

```
 RealSense D415 ─/camera/camera/depth/image_rect_raw─┐
                                                     ▼
 bno055 ───────────────/bno055/imu──────────► race_terain_node ── web UI :8080
                                                     │
                                       /mega/cmd_vel ▼
                                              base (bringup)
```

The ground plane is refitted every frame because the robot pitches on rough
ground. Objects beside the road go into a top-down map. The node traces the
centre line between the two rows and follows it with pure pursuit.

### Scripted drive — `ros2 launch run_by_scenario run_by_scenario.launch.py`

```
 browser (keys / joystick / gamepad) ─► run_by_scenario_node ─/mega/cmd_vel─► base (bringup)
                                         records to ~/.ros/scenarios/<name>.json
                                         and plays it back on START
```

### Vision only — `ros2 launch pingtung_vision vision.launch.py vision_mode:=<mode>`

```
 USB camera ─► v4l2_camera ─/camera/image_raw─► vision ─┬─ /vision/animal/result      (animal)
                                                        ├─ /vision/fruit/line_hit     (fruit_color)
                                                        ├─ /vision/pig_shit/spatial   (pig_shit)
                                                        ├─ /vision/active_mode
                                                        └─ /vision/debug_image ─► web_view_node :8080
```

---

## The packages, one by one

All ROS 2 packages live in [`src/`](src). Every package has its own README with
parameters, tuning steps and tests; this section gives the short version.

### Base and hardware

#### [`pingtung_bot_bringup`](src/pingtung_bot_bringup)
*Launch only, no nodes of its own.* Starts the whole robot base with one
command: `bno055`, `wheel_control` (in `output: topic` mode) and `mega_bridge`.
Every mission launch file includes it.

| launch argument | default | meaning |
|---|---|---|
| `port` | `/dev/ttyACM0` | USB serial port of the Arduino Mega |
| `base` | `true` | `false` starts only `mega_bridge` (gantry and gripper on a bench, no IMU or wheels) |
| `gantry_test` | `false` | also run the gantry/gripper self-test once |

#### [`mega_bridge`](src/mega_bridge/README.md)
**Node `mega_bridge_node`** — the Jetson side of the USB link to the Arduino
Mega. It is the only node that opens the serial port.

- Subscribes: `/mega/wheel_pwm`, `/mega/step` (relative gantry move),
  `/mega/goto` (absolute gantry move), `/mega/gripper` (servo angle).
- Publishes: `/mega/status` at 10 Hz (steps left, servo angle, wheel
  failsafe flag, gantry position).
- Service: `/mega/set_home` makes the current gantry position the zero point.
  There are no limit switches, so the node counts steps itself and saves the
  position to `~/.ros/mega_bridge_home.txt`.

**Node `gantry_test`** — moves each gantry axis out and back and cycles the
gripper, then exits with PASS or FAIL.

#### [`wheel_control`](src/wheel_control/README.md)
**Node `wheel_control_node`** — the closed-loop controller for the
differential base.

- Subscribes: `cmd_vel` (remapped to `/mega/cmd_vel`) and `/bno055/imu`.
- Publishes: `/mega/wheel_pwm` (left/right PWM, −255..255) and
  `/wheel_control/debug` for tuning.
- How: two loops. An outer **heading hold** keeps the robot pointing where it
  was pointing, and an inner **yaw-rate PI loop** with feedforward makes the
  robot turn at the commanded rate. The README explains why a rate loop alone
  still lets the robot drift.
- Forward speed is open loop: `base PWM = k_lin × linear.x`. Missions therefore
  set speed as a PWM value and send `linear.x = pwm / k_lin`.

#### [`wheel_open_loop`](src/wheel_open_loop/README.md)
**Node `wheel_open_loop_node`** — the same base with the feedback removed:
`cmd_vel` maps straight to two PWM values. It is used as a **baseline
experiment** to measure what the IMU loop gains. It drives the older ESP32
board directly, not the Mega.

#### [`bno055`](src/bno055/README.md)
**Node `bno055`** — driver for the Bosch BNO055 IMU over I²C. Publishes
`/bno055/imu` (`sensor_msgs/Imu`) at 100 Hz. This is the upstream
[flynneva/bno055](https://github.com/flynneva/bno055) driver (0.5.0), vendored
here unchanged apart from configuration, under its own BSD license.

### Perception

#### [`pingtung_vision`](src/pingtung_vision/README.md)
**Node `vision`** — one node that runs exactly one camera algorithm at a time,
chosen by the `algorithm` parameter. The mode can be switched at runtime
(`ros2 param set /vision algorithm fruit_color`).

| mode | method | publishes |
|---|---|---|
| `animal` | MobileNetV3 classifier (PyTorch) in [`models/`](src/pingtung_vision/models) | `/vision/animal/result`: dog, monkey, rabbit, turtle or unknown, with probabilities |
| `fruit_color` | OpenCV colour segmentation | `/vision/fruit/line_hit`: 1 when a stable red/yellow fruit is on the hit line |
| `pig_shit` | OpenCV segmentation, left/right halves | `/vision/pig_shit/spatial`: share of pig, manure and empty on each side |
| `off` | — | nothing |

Every result has a `valid` flag. It is `false` at start-up and after a camera
timeout, so missions can reject stale data. Results are smoothed over several
frames so one noisy frame does not trigger an action.

**Node `web_view_node`** — streams `/vision/debug_image` (the annotated camera
image) to a browser at port 8080.

The launch file also starts `v4l2_camera` for the USB camera.

#### [`pingtung_vision_interfaces`](src/pingtung_vision_interfaces/msg)
*Message definitions only, no nodes.* The custom ROS messages that
`pingtung_vision` publishes and the missions read:

| message | fields |
|---|---|
| `AnimalResult` | `valid`, `animal_id` (0 unknown, 1 dog, 2 monkey, 3 rabbit, 4 turtle), four probabilities |
| `FruitLineHit` | `valid`, `line_hit` (0 or 1) |
| `PigShitSpatial` | `valid`, `left_pig`, `left_shit`, `left_empty`, `right_pig`, `right_shit`, `right_empty` |

It is a separate CMake package because ROS 2 builds message types that way.

### Missions

All four mission nodes share the same design: a state machine driven by a
fixed-rate control timer (20–50 Hz), a small built-in web server (no extra install) for the UI, and
pure-Python logic kept apart from the ROS code so it can be unit-tested and
simulated without a robot. Settings changed in the UI are saved under
`~/.ros/` and reloaded at the next start.

#### [`pick_fruit`](src/pick_fruit/README.md)
**Node `pick_fruit_node`** — drives forward and strikes each red or yellow
fruit with the gantry as it passes the camera's hit line.

- Reads: `/vision/fruit/line_hit`, `/vision/debug_image`, `/mega/status`.
- Sends: `/mega/cmd_vel`, `/mega/goto`; `/mega/step` and `/mega/gripper`
  only for manual jogging in the UI.
- The UI has teleop (W/A/S/D), gantry jog, and buttons to save the `ready`
  and `hit` gantry positions.
- Code: `controller.py` (state machine), `web.py` (UI), `pick_fruit_node.py`
  (ROS glue).

#### [`lane_runner`](src/lane_runner/README.md)
**Node `lane_runner_node`** — drives the U-shaped lane course on its own,
using only the IMU and the RealSense depth image: no map, no Nav2, no wheel
encoders.

- Reads: RealSense depth image and camera info, `/bno055/imu`.
- Sends: `/mega/cmd_vel`. Publishes `/lane_runner/points` (coloured point
  cloud) for RViz.
- Code: `perception.py` (depth → points, floor fit, obstacle grid),
  `mission.py` (states, gap following, U-turn), `lane_runner_node.py` (ROS
  glue and UI).
- Tested with a ray-cast depth simulator ([`test/sim.py`](src/lane_runner/test/sim.py))
  that drives the whole course with obstacles, wrong speed estimates and gyro
  drift.

#### [`race_terain`](src/race_terain/README.md)
**Node `race_terain_node`** — the all-terrain race: follows the centre line
between the two rows of objects lining the road, to the finish.

- Reads: RealSense depth image and camera info, `/bno055/imu`.
- Sends: `/mega/cmd_vel`.
- Code: `perception.py` (ground fit with RANSAC, obstacle points),
  `tracker.py` (memory map, 2D ICP motion correction, centre-line tracing,
  pure pursuit), `race_terain_node.py` (ROS glue and UI).
- The package name keeps its original spelling ("terain").

#### [`run_by_scenario`](src/run_by_scenario/README.md)
**Node `run_by_scenario_node`** — record a hand-driven run, then replay it.

- Sends: `/mega/cmd_vel`. Reads `/mega/status` and `/mega/wheel_pwm`.
- Drive with keys, an on-screen joystick or a gamepad; press Record, drive,
  press STOP. The run is saved as a readable JSON list of timed PWM commands
  in `~/.ros/scenarios/`, and START replays it.
- Replay is timed, not position-controlled, because there are no wheel
  encoders. The heading hold keeps straight parts straight.

### Outside `src/`

| folder | contents |
|---|---|
| [`Arduino/mega_bridge`](Arduino/mega_bridge/mega_bridge.ino) | **current firmware** for the Arduino Mega: wheels, steppers, servo. Flash this one |
| `Arduino/mega_motor_control` | old keyboard test sketch for steppers and servo (legacy) |
| `Arduino/pingtung_contest` | ESP32 wheel firmware from before the Mega (legacy, used by `wheel_open_loop`) |
| [`pythonScript/fruit_color_opencv.py`](pythonScript/fruit_color_opencv.py) | stand-alone prototype of the fruit-colour algorithm, works on images or a live camera |
| [`pythonScript/take_laneData.py`](pythonScript/take_laneData.py) | records camera videos of the lanes for development, with a web page to drive the robot while recording |
| [`docs/`](docs) | architecture diagram |

---

## Topic reference

Interface used by the missions:

| topic | type | direction | meaning |
|---|---|---|---|
| `/mega/cmd_vel` | `geometry_msgs/Twist` | in | base motion. `linear.x = pwm / k_lin` (forward PWM, open loop). `angular.z` in rad/s, + = left |
| `/mega/step` | `std_msgs/Int32MultiArray` `[step1, step2]` | in | **relative** gantry steps, added to the pending move. 1600 steps = 1 rev at 1/8 microstep |
| `/mega/goto` | `std_msgs/Int32MultiArray` `[x, y]` | in | **absolute** gantry position in steps from home. `[0, 0]` goes home |
| `/mega/gripper` | `std_msgs/Float32` | in | servo angle in degrees, clamped to 0–180 |
| `/mega/status` | `std_msgs/Int32MultiArray` | out, 10 Hz | `[steps_left1, steps_left2, servo_deg, wheel_failsafe, pos1, pos2]` |
| `/mega/set_home` | `std_srvs/Trigger` (service) | — | current gantry position becomes home |
| `/vision/fruit/line_hit` | `pingtung_vision_interfaces/FruitLineHit` | out | fruit on the hit line |
| `/vision/animal/result` | `pingtung_vision_interfaces/AnimalResult` | out | recognised animal |
| `/vision/pig_shit/spatial` | `pingtung_vision_interfaces/PigShitSpatial` | out | pig / manure / empty, left and right |
| `/vision/debug_image` | `sensor_msgs/Image` | out | annotated camera image |

Internal topics:

| topic | type | meaning |
|---|---|---|
| `/bno055/imu` | `sensor_msgs/Imu` | IMU data |
| `/mega/wheel_pwm` | `std_msgs/Int16MultiArray` `[m1, m2]` | wheel PWM −255..255, from `wheel_control` to `mega_bridge` |
| `/wheel_control/debug` | `std_msgs/Float32MultiArray` | loop internals for tuning (see the wheel_control README) |
| `/camera/image_raw` | `sensor_msgs/Image` | USB camera, from `v4l2_camera` |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | RealSense depth, from `realsense2_camera` |

A gantry move is finished when both `steps_left` in `/mega/status` read 0.
That is the signal to move on, for example to close the gripper.

---

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

---

## Safety: where the wheels stop

The wheels are stopped independently at four levels, so a crash in any one
program still stops the robot:

| layer | trigger | action |
|---|---|---|
| mission node | STOP button, Space key, Ctrl+C, lost sensor | sends zero velocity |
| wheel_control | no `/mega/cmd_vel` for 0.5 s | commands 0, or holds heading if `idle_hold` is on |
| wheel_control | no IMU for 0.3 s, or gyro not yet calibrated | commands 0 (never drives without feedback) |
| mega_bridge | no `/mega/wheel_pwm` for 0.5 s | sends PWM 0 |
| Mega firmware | no DRIVE frame for 300 ms (bridge dead, cable out) | ramped brake, sets `wheel_failsafe` |

The steppers and servo are not covered by these: they finish the move they were
given. At startup `wheel_control` calibrates gyro bias from 200 samples, so
**keep the robot still for the first few seconds**.

---

## Build and run

Requirements: Ubuntu 22.04, ROS 2 Humble, Arduino IDE. The camera drivers come
from apt:

```bash
sudo apt install ros-humble-v4l2-camera ros-humble-realsense2-camera
```

The `animal` vision mode also needs PyTorch and TorchVision built for your
JetPack version; they are not installed automatically.

```bash
# 1. firmware: install AccelStepper (Arduino Library Manager), then flash
#    Arduino/mega_bridge/mega_bridge.ino to the Mega

# 2. workspace
git clone <this repository> ~/pingtung_bot
cd ~/pingtung_bot
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash

# 3a. base only (IMU + wheel_control + mega_bridge) - keep it still while the gyro calibrates
ros2 launch pingtung_bot_bringup bringup.launch.py
#     args: port:=/dev/ttyACM0   base:=false (gantry + gripper only, no IMU/wheel_control)

# 3b. or one mission (each one starts the base too), then open http://<robot-ip>:8080/
ros2 launch pick_fruit pick_fruit.launch.py
ros2 launch lane_runner lane_runner.launch.py
ros2 launch race_terain race_terain.launch.py
ros2 launch run_by_scenario run_by_scenario.launch.py

# 4. command the base by hand
ros2 topic pub -r 10 /mega/cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"
ros2 topic pub --once /mega/goto std_msgs/Int32MultiArray "{data: [1600, 800]}"
ros2 topic pub --once /mega/gripper std_msgs/Float32 "{data: 45.0}"
ros2 topic echo /mega/status
```

To test the actuators without the IMU or control loop, run only the bridge:
`ros2 launch mega_bridge mega_bridge.launch.py`. Then publish `/mega/wheel_pwm`
directly, with the wheels off the ground.

Tests (no robot needed):

```bash
colcon test && colcon test-result --verbose
# or one package: cd src/lane_runner && python3 -m pytest test
```

---

## Limits

- **No linear speed control.** The wheels have no encoders, so forward motion
  is open-loop: base PWM = `k_lin * linear.x`, assumed proportional. Distance
  varies with battery, load and floor.
- **Heading drifts slowly.** It is integrated from the gyro because the
  magnetometer is unreliable next to the motors. Bias is re-estimated whenever
  the robot is parked.
- **Steppers are open loop.** With no limit switches or homing switch, gantry
  position is counted from the point set with `/mega/set_home`.
- **No automatic reconnect.** If the Mega is unplugged, restart `mega_bridge`.
- **One mission at a time.** Missions share `/mega/cmd_vel`, the cameras and
  web port 8080.
- **Retune after moving from the ESP32.** Mega PWM runs at ≈3.9 kHz rather
  than 5 kHz, so re-measure `min_pwm_left` and `min_pwm_right`.

---

## Repository layout

```
pingtung_bot/
├── Arduino/
│   ├── mega_bridge/          Mega firmware: wheels + steppers + servo   ← flash this
│   ├── mega_motor_control/   old keyboard test sketch for steppers/servo (legacy)
│   └── pingtung_contest/     ESP32 wheel firmware from before the Mega (legacy)
├── docs/                     architecture diagram
├── pythonScript/             stand-alone prototypes and data-collection tools
└── src/                      ROS 2 packages (colcon workspace)
    ├── pingtung_bot_bringup/       launch file that starts the robot base
    ├── mega_bridge/                serial bridge to the Mega, /mega/* topics
    ├── wheel_control/              closed-loop base controller (IMU heading hold)
    ├── wheel_open_loop/            open-loop baseline for comparison (ESP32 only)
    ├── bno055/                     BNO055 IMU driver (upstream 0.5.0, vendored)
    ├── pingtung_vision/            camera algorithms: animal, fruit colour, pig/manure
    ├── pingtung_vision_interfaces/ custom result messages for pingtung_vision
    ├── pick_fruit/                 mission: drive and strike red/yellow fruit with the gantry
    ├── lane_runner/                mission: U-shaped lane course, avoids obstacles
    ├── race_terain/                mission: all-terrain race along the road centre line
    └── run_by_scenario/            mission: record a teleop drive and play it back
```

---

## License

The packages written for this robot are released under the MIT license, as
declared in each `package.xml`. The vendored `bno055` driver keeps its own BSD
license ([src/bno055/LICENSE](src/bno055/LICENSE)).
