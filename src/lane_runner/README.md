# lane_runner

Drive the U-shaped course on its own: down lane 1, U-turn at the end, back up
lane 2, stop at the goal, and steer past the pigs on the way. It uses only the
BNO055 IMU (heading) and the RealSense D415 depth camera. No map, no Nav2, no
wheel encoders.

```
bringup.launch.py   ── bno055, wheel_control, mega_bridge                (hardware)
realsense2_camera   ── D415 depth only, 424x240 @ 30 fps
lane_runner_node    ── course logic + web UI :8080                        ─► /mega/cmd_vel
```

```
 IDLE ─START─► CALIBRATE ─► LANE1 ─end wall─► PIVOT1 ─► CROSS ─► PIVOT2 ─► LANE2 ─end wall─► DONE
               (1.5 s still)    follow the free gap        90°    ~1 lane   90°
```

## Run

```bash
colcon build --packages-select lane_runner && source install/setup.bash
ros2 launch lane_runner lane_runner.launch.py
#   args: port:=/dev/ttyACM0  web_port:=8080  dry_run:=true  hardware:=false
# open http://<robot-ip>:8080/
```

1. Put the robot at the start of lane 1, roughly centred, pointing down the lane.
2. Keep it still for the first few seconds (wheel_control calibrates its gyro).
3. Pick the U-turn direction in the page (**left** if lane 2 is on the robot's
   left), press **START**, and keep the robot still for ~1.5 s while it
   calibrates. Then it drives the course and stops at the end of lane 2.
   Set the **linear** and **angular PWM** in the page before or during the
   run (see below).
4. **STOP** (or Space) at any time.

Do not run it together with `pick_fruit` / `pingtung_vision`: they use the same
camera and the same port. The RealSense driver is `ros-humble-realsense2-camera`
(apt).

## How it works

- **Obstacles:** each depth frame becomes 3D points. At START the floor plane
  is fitted from the depth image, so the camera's height and tilt never need
  measuring; they are logged and shown in the page. Everything 4–50 cm above
  the floor is an obstacle: the lane walls and the pigs. The floor's slots read
  as below the floor and are ignored.
- **Heading:** the gyro, integrated from START (bias averaged while still).
  Lane 1 = 0°, cross-over = ±90°, lane 2 = ±180°.
- **Steering:** obstacles go into a top-down grid along the lane. The robot's
  position in it is dead-reckoned from the heading and the commanded speed
  (`pwm / k_lin × speed_scale`). That is rough, but it only has to hold for the
  metre or so that a pig is beside or behind the camera, out of its view. Each
  cycle it takes the free gap the robot fits through (its width plus `margin`
  each side), from one robot length behind the camera to `look_ahead` in
  front, and steers for its centre. With no pig that is the middle of the lane;
  with a pig it is the free side. A "gap" on the far side of a wall never counts.
- **End of lane:** something closer than `end_dist` that spans the whole lane
  width. A pig never does that, since it sits on one side.
- **U-turn:** pivot 90° on the gyro, drive `lane_width + divider` across
  (dead-reckoned, or less if a wall comes within `cross_front_stop`), pivot
  another 90°. It does not have to land exactly in the middle of lane 2: the
  lane-following re-centres the robot between the walls.

wheel_control still does the low-level work: `linear.x = pwm / k_lin` as
everywhere else in this repo, and `angular.z` is a yaw rate its loop tracks.
`k_lin` and `imu_yaw_sign` are read from `wheel_control.yaml` by the launch file.

## Set up on the real course

Measure and set these in [config/lane_runner.yaml](config/lane_runner.yaml)
first: `robot_width`, `robot_length` (camera to the robot's rear),
`lane_width`, `divider`. Then:

1. **Mount the camera** with the page open (no need to START): check the
   camera view and the live height/tilt. Then **watch a pass:** `dry_run:=true`,
   and push the robot down the lane by hand with the page open. The top-down view should show both walls and the pigs
   in the right places (white). START is disabled in dry run.
2. **Speed estimate:** time the robot over 1 m at your linear PWM, which
   gives v m/s. Then set `speed_scale = v × k_lin / linear_pwm`. It matters most
   for how long a passed pig is remembered and for the cross-over distance.
   The tests pass with the real speed anywhere from 0.6× to 1.5× the estimate.
3. **Pivots:** if a U-turn pivot stalls, raise the angular PWM (or
   `pivot_min_pwm`); if it overshoots, lower the angular PWM.
4. **End distance:** the robot needs room to pivot at the end of lane 1. If it
   scrapes the end wall while turning, raise `end_dist`. Keep it above
   ~0.3 m: the D415 cannot see closer than about 0.18 m at 424x240.

## Web page

**PWM**, set any time, also mid-run, and kept in `~/.ros/lane_runner.json`
for the next start:

| setting | what it does | sent as |
|---|---|---|
| **linear** | forward speed in the lanes and the U-turn cross-over | `linear.x = pwm / k_lin` |
| **angular** | the fastest turn: dodging pigs and the U-turn pivots | `angular.z` up to `pwm / k_ff` |

Each value has a number box with −5 / −1 / +1 / +5 buttons, and nothing changes
until you press **Set**: an edited box is shown in orange until then. There are
no sliders on purpose, because on a phone a slider moves when you scroll across
it. The angular PWM is capped at `max_angular_pwm` (100 ≈ 3 rad/s); steering
never needs more.

There are no wheel encoders, so these are the honest units. `k_lin` and
`k_ff` come from `wheel_control.yaml`, and wheel_control's yaw-rate loop still
tracks the turn. If you change the linear PWM, re-check `speed_scale`: it
converts PWM to a speed guess for the pig memory.

**Obstacle height band** (bottom / top, in cm above the floor, default 4–50),
set any time and kept with the PWMs: only points in this band count as
obstacles. Everything below is floor, everything above is ignored. Tune it
while watching the camera view: raise the bottom if floor noise or the floor's
slots turn red, and lower the top if things behind or above the walls (people,
the tent) turn red. Keep the bottom below the pigs' height (~9 cm), or the
robot will not see them.

**Views, always live, also when not driving**, so you can mount and aim the
camera while watching:

- **Camera:** the depth image as the camera sees it. Green = floor, red =
  obstacle (walls, pigs), grey = too high or farther than `max_range`, black =
  no depth, brighter = closer. Aim so that the lane floor and both walls are
  in view, and a pig on the floor ahead turns red.
- **Camera height and tilt:** while idle, the floor is refitted about once a
  second, so the numbers follow the camera as you move it ("live"). START fits
  it once more and keeps it for the run ("fixed at START"). "no floor found"
  means the camera does not see enough floor: tilt it down.
- **Top-down:** this frame's points (red = obstacles, dim blue = floor) on top
  of what the run remembers.

For RViz, `/lane_runner/points` (PointCloud2, in the camera's
`camera_depth_optical_frame`, coloured the same way) is published whenever
something subscribes. Set the colour transformer to RGB8.

Also START / STOP, the U-turn direction, state, heading, distance travelled, what
is straight ahead, the command sent, the fitted camera height/tilt and IMU and
depth health, plus a live top-down view:

| colour | meaning |
|---|---|
| white | walls / pigs remembered |
| green | the free gap the robot fits through |
| yellow | the direction it steers |
| orange | the robot (camera at the front edge) |

`GET /api/state`, `GET /view.jpg`, `GET /camera.jpg`, `POST /api/cmd` with `{"cmd": "start" | "stop"}`,
`{"cmd": "turn", "left": true|false}`, `{"cmd": "pwm", "linear": 40, "angular": 30}` or
`{"cmd": "heights", "min": 0.04, "max": 0.5}` (metres).

## Safety

| condition | action |
|---|---|
| STOP, Space, Ctrl+C | base stops |
| no IMU for `imu_timeout_s` | run stopped, shown in red |
| no depth for `depth_timeout_s` | paused (base 0) until depth is back |
| something in the robot's path closer than `emergency_dist` | no forward motion, turns in place towards the free side |
| a pivot or the cross-over takes too long | run stopped, shown in red |

## Limits

- The dead-reckoned position drifts. It only has to be good for a few metres
  per leg (the grid restarts on every leg), and the walls keep re-centring the
  robot.
- A pig inside the U-turn space itself (past the end of the divider) can block
  the cross-over. The robot then stops the cross-over early and may end in the
  wrong lane. Pigs in the lanes are fine.

## Test

A small ray-cast simulator ([test/sim.py](test/sim.py)) renders D415-like depth
images of the course. The tests drive the whole course with pigs through it,
including a speed estimate that is wrong by ×0.6 and ×1.5, an off-centre and
skewed start, a drifting gyro, and a different camera height and tilt. They
check that the robot never touches a pig or a wall and ends at the goal.

```bash
cd src/lane_runner && python3 -m pytest test      # ~1.5 min
```
