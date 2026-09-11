# wheel_open_loop

The same two-wheel base as `wheel_control`, with the feedback removed. It maps
`/cmd_vel` straight to two PWM values and sends them to the ESP32. No IMU, no
PID, no heading hold. Use it as the baseline to show what the closed loop buys.

```
/cmd_vel (Twist) ─> mix ─> ESP32 ─> JZ2407DB
```

```
base      = base_pwm · sign(v)      # 0 when v == 0
u         = k_turn · ω              # same as k_ff in wheel_control
pwm_left  = base − u
pwm_right = base + u
```

That is exactly `wheel_control`'s feedforward path, so with matching parameters
the only difference between the two nodes is feedback.

Kept, because they are wiring or safety rather than control: `swap_motors`,
`invert_motor1/2`, the `max_pwm` clamp and the `cmd_timeout_s` stop. Left out on
purpose: `min_pwm` lifting and slew limiting. A turn command whose `k_turn · ω`
is below a wheel's stiction threshold will not move that wheel.

## Fair comparison

Copy these from `wheel_control/config/wheel_control.yaml` into
`config/wheel_open_loop.yaml` whenever you retune:

| wheel_open_loop | wheel_control |
|---|---|
| `base_pwm` | `base_pwm` |
| `k_turn` | `k_ff` |
| `port`, `swap_motors`, `invert_motor1/2` | same names |

## Demo

Both nodes open the same serial port, so **run only one at a time**. Keep the
BNO055 running for both so the heading can be recorded.

```bash
colcon build --packages-select wheel_control wheel_open_loop
source install/setup.bash

ros2 run bno055 bno055 --ros-args --params-file \
  install/bno055/share/bno055/config/bno055_params_i2c.yaml

# run A: open loop
ros2 launch wheel_open_loop wheel_open_loop.launch.py
# run B: closed loop (stop run A first)
ros2 launch wheel_control wheel_control.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

For each run, start from the same mark, hold `i` (straight forward) for the same
time, and measure how far the robot ends up off the line. Watch
`/bno055/imu` in PlotJuggler for the yaw. Then nudge the robot sideways
mid-run: the open-loop base keeps the new heading, `wheel_control` turns back.

`/wheel_open_loop/debug` is `[pwm_l, pwm_r]`, the same as indices 8 and 9 of
`/wheel_control/debug`, so the two can be plotted together.
