# wheel_control

Two-wheel differential base. Open loop in linear speed (fixed base PWM), closed
loop in yaw rate **and heading** from a BNO055.

```
/cmd_vel (Twist) ──┐
                   ├─> [heading hold] ─> [yaw-rate PI] ─> mix ─> ESP32 ─> JZ2407DB
/imu/data (Imu)  ──┘
```

## Why two loops

The block diagram as drawn — `ω_d → controller → PWM → base → IMU → ω` — is a
**rate** loop. Holding ω = 0 means "do not rotate *right now*". Any error the
loop has not corrected yet has already been integrated into heading and is
never recovered: the robot stops curving but keeps the heading it drifted to.

Simulated against a first-order plant with a 14-count motor mismatch, 12 s of
driving straight:

| controller | final heading error |
|---|---|
| yaw-rate loop only | **+10.5°** and still growing |
| yaw-rate + heading hold | **−2.9°**, bounded |

That residual −2.9° is exactly the leftover gyro bias (0.004 rad/s × 12 s)
integrating. Heading hold removes everything that is removable; gyro bias sets
the floor. See "Drift" below.

## PI, not PID

- **P and I are both needed.** The I term is what cancels the constant motor
  mismatch — it is the reason "straight" becomes straight rather than a slow
  arc.
- **D is 0 by default.** Differentiating a BNO055 gyro signal mostly amplifies
  noise, and the plant is roughly first-order, so D buys little. If you do
  enable it, `kd_tau` filtering is already wired in.
- **Feedforward does most of the work.** `k_ff · ω_sp` puts the output in the
  right neighbourhood immediately; the PI only trims. Tune `k_ff` first.
- **Anti-windup is not optional.** With `base_pwm = 80` the differential
  saturates at ±175. A wound-up integrator is the classic "overshoots the turn,
  then wanders back".

## Tuning order

1. **Measure `min_pwm_left` and `min_pwm_right`.** Raise both wheels off the
   ground, command increasing PWM, and note where each wheel reliably starts
   turning. The two motors usually differ, so measure them separately. "Left"
   means the robot's left wheel, after `swap_motors` has been applied.
2. **Measure `k_ff`.** With the loop disabled (`kp=ki=0`, `heading_hold=false`),
   command a few fixed differential values and record steady-state ω from
   `/wheel_control/debug`. `k_ff = u / ω`. This is the single most valuable
   number here.
3. **`kp`** — raise until the response to a step ω_d is quick with a small
   overshoot, then back off ~30%.
4. **`ki`** — raise until steady-state error vanishes within ~1 s. Too high
   shows as a slow hunting oscillation.
5. **`k_heading`** last, with `heading_hold: true`. Drive forward, nudge the
   robot sideways, watch it return to the original line. Too high makes it
   snake.

`/wheel_control/debug` is a `Float32MultiArray`:
`[ω_d, ω_setpoint, ω_measured, u, P, I, D, heading_err, pwm_l, pwm_r]`

```bash
ros2 topic echo /wheel_control/debug
ros2 run plotjuggler plotjuggler     # much easier than reading numbers
```

## Drift, and what this cannot do

Heading is integrated from the gyro, deliberately — the BNO055's fused yaw uses
the magnetometer, and a magnetometer sitting near two motors drawing up to 7 A
each is not a heading reference. The cost is that integrated yaw drifts with
gyro bias. Mitigations in the node:

- bias is estimated from 200 stationary samples at startup (**keep the robot
  still**, it refuses to drive until this finishes)
- bias is slowly re-estimated whenever the robot has been commanded to stop for
  more than 1.5 s, since BNO055 bias walks with temperature

For runs longer than a minute or two you need an absolute heading reference —
wheel odometry, a magnetometer away from the motors, or vision. Without one,
"straight" degrades at roughly the residual bias rate.

Also note: with a fixed `base_pwm` there is **no linear velocity control**.
`cmd_vel.linear.x` is used only for its sign. Distance travelled will vary with
battery voltage, payload and floor surface, and nav stacks that assume the
commanded velocity was achieved will not work well against this. That is a fine
trade for now, but it is the next thing to fix — wheel encoders would close it.

## Idle hold

With `idle_hold: true` the controller keeps running when nobody is driving,
whether `/cmd_vel` is zero or has timed out. The robot parks on the heading it
had when the last command ended. If it is knocked round by more than
`hold_tolerance_deg`, it spins in place back to that heading and switches the
motors off again once inside `hold_settle_deg`.

**Heading only, not position.** Nothing on this robot measures translation: the
IMU gives yaw rate and the wheels have no encoders. A push that slides the
robot is invisible to it, and it will not drive back. Holding a full pose needs
wheel odometry, the same missing sensor as in "Drift" above.

| param | default | meaning |
|---|---|---|
| `idle_hold` | `false` in code, `true` in `config/wheel_control.yaml` | on/off switch |
| `hold_tolerance_deg` | 3.0 | error that starts a correction |
| `hold_settle_deg` | 1.0 | error at which the motors stop again |
| `k_hold` | 2.0 | rad/s of spin per rad of error |
| `hold_max_rate` | 0.5 | rad/s cap on the recovery spin |

Why two thresholds: any non-zero output is lifted to the wheel's `min_pwm_left`/`min_pwm_right`, so with a
single deadband the correction overshoots its edge and the robot buzzes. If it
still hunts, widen the gap between them. If a recovery stalls short of the
settle band, raise `k_hold`.

To tune, put the robot on the floor, rotate it about 20° by hand, let go, and
watch `heading_err` (index 7) on `/wheel_control/debug`.

While parked, gyro bias is re-estimated only when the gyro reads under about
1°/s, so being pushed is not learned as bias.

**With `idle_hold` on, the robot moves without a command.** Turn it off, or lift
the wheels, before working on the base.

## Safety

Three independent stops, which is the right number:

| condition | action |
|---|---|
| no `/cmd_vel` for `cmd_timeout_s` (0.5 s) | node commands 0; with `idle_hold`, holds heading instead (rotation only, `v` is forced to 0) |
| **no `/imu/data` for `imu_timeout_s` (0.3 s)** | node commands 0 |
| no serial packet for 300 ms | **firmware** ramps to a stop on its own |

The IMU gate matters most: no feedback means no closed loop, and driving
open-loop on a base PWM is how a robot walks into a wall sideways.

## Build and run

```bash
cd ~/ros2_ws/src && cp -r wheel_control .
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select wheel_control
source install/setup.bash

ros2 launch wheel_control wheel_control.launch.py port:=/dev/ttyUSB0
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You need a BNO055 driver publishing `sensor_msgs/Imu` on `/imu/data`
(`ros2 run bno055 bno055` from the `bno055` package, or your own). Reading the
IMU over I²C inside this node would couple sensor timing to control timing —
keep them separate.

**First bring-up, wheels off the ground:**

1. `ros2 topic echo /imu/data` — confirm `angular_velocity.z` is positive when
   you rotate the robot counter-clockwise. If not, set `imu_yaw_sign: -1.0`.
2. Command `linear.x: 0.2, angular.z: 0.0` and confirm both wheels turn
   forward. If one is backwards, use `invert_motor1` / `invert_motor2`.
3. Command `angular.z: 0.5` and confirm the robot tries to turn **left**. If it
   turns right, `swap_motors: true`.

Fix the IMU sign before the motor signs. Flipping both hides the error instead
of correcting it, and the loop will be positive-feedback unstable the moment
you enable it.
