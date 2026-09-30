# pick_fruit

Drive straight forward and hit every red/yellow fruit that reaches the camera's
hit line with the gantry, without stopping. A web UI provides START/STOP, the camera view, teleop,
gantry jog and the saved gantry positions.

```
bringup.launch.py ── bno055, wheel_control, mega_bridge      (hardware)
vision.launch.py  ── camera + vision (fruit_color)           ─► /vision/fruit/line_hit, /vision/debug_image
pick_fruit_node   ── state machine + web UI :8080            ─► /mega/cmd_vel, /mega/goto, /mega/step, /mega/gripper
```

## Run

```bash
colcon build --packages-select pick_fruit && source install/setup.bash
ros2 launch pick_fruit pick_fruit.launch.py
#   args: port:=/dev/ttyACM0  video_device:=/dev/video4  web_port:=8080
# open http://<robot-ip>:8080/
```

Keep the robot still for the first few seconds while wheel_control calibrates
the gyro. The launch starts the camera itself, and `pingtung_vision`'s own web
view is turned off so port 8080 belongs to this node.

## Workflow

1. **First time only: set the positions.** With the robot stopped, jog the
   gantry (arrow keys or the X/Y pad) until the gripper waits where it should
   while driving, then click **Save current** on `ready`. Put a fruit on the
   yellow hit line in the camera view, jog the gantry to where it hits the
   fruit, and click **Save current** on `hit`. **Go** moves to a saved
   position to check it. The positions and forward PWM are saved to
   `~/.ros/pick_fruit.json` and loaded again at the next start.
2. Find the drive PWM by hand: pick a value with the slider or number box,
   press **Set speed**, then hold **W** / **S** and watch the robot. Adjust and
   repeat until it moves the way you want. W/S and the autonomous run use the
   **same** PWM, so START drives forward at exactly what you tried. It applies
   at once, also while running, and is saved. It is a wheel **PWM** (0–255),
   not m/s: there are no wheel encoders, so the node only asks
   `wheel_control` for a base PWM (sent as `linear.x = pwm / k_lin`, with
   `k_lin` read from `wheel_control.yaml`). If the robot does not move,
   raise it. Anything below `min_pwm_left/right` is lifted to that value.
3. **START**: the gantry goes to `ready`, then the robot drives straight
   forward at the drive PWM (wheel_control's heading hold keeps it straight).
4. The robot never stops for a fruit:

   | `line_hit` | base | gantry |
   |---|---|---|
   | 1 | keeps driving, `hit_slowdown_pct` slower (20% → 80% of the PWM) | goes to `hit` |
   | 0 for `ready_delay_s` (0.3 s) **and** the gantry has reached `hit` | back to the full drive PWM | goes back to `ready` |

   The gantry always completes the reach: even if the fruit leaves the line
   before the gantry arrives, it goes back only once `/mega/status` confirms
   it stopped at `hit` (two fresh statuses, 0 steps left, position = `hit`),
   and it keeps driving at the slower PWM until then. The delay stops a
   one-frame flicker of `line_hit` from pulling the gantry in; set it to 0
   to retract as soon as the gantry arrives. The page shows the gantry target, the PWM being
   driven and how many fruits were hit.
5. **STOP** (or Space) at any time: the base stops and a moving gantry is
   halted.

Teleop, jog, gripper and saving positions only work while stopped (`idle`),
so manual commands never fight the automatic run.

| key | action |
|---|---|
| W / S | drive forward / back while held |
| A / D | turn left / right while held |
| arrows | jog gantry by the selected step size |
| O / C | gripper open / close |
| Space | STOP |

## Safety

| condition | action |
|---|---|
| no valid `/vision/fruit/line_hit` for `vision_timeout_s` | driving pauses (base 0) and the gantry retracts to `ready` until vision is back |
| no `/mega/status` for `status_timeout_s` | driving pauses |
| a teleop key held but the browser stops refreshing (closed tab, lost Wi-Fi) | base stops after `teleop_timeout_s` |
| after START, the gantry has not reached `ready` within `move_timeout_s` | run aborted, shown in red in the UI |
| the gantry has not reached `hit` within `move_timeout_s` of being sent there | run aborted, shown in red in the UI |

The gantry counts as at `ready` only once `/mega/status` confirms it, after at
least two fresh status messages, the same rule `mega_bridge` uses.

## Notes

- Positions are gantry steps from `mega_bridge`'s home (`pos1`, `pos2`).
  **Set home here** calls `/mega/set_home`. The saved numbers are not
  changed, so re-home only at the same physical home point, or save the
  positions again afterwards.
- The hit position is fixed relative to the robot, so it works as long as
  the fruit is at a similar distance from the robot each time. The hit line
  is set in `pingtung_vision` (`fruit.roi_height`, `fruit.line_tolerance_px`).
- There are no gantry soft limits. The jog buttons send relative steps, so
  start with a small step size.
- The gripper is not used by the run; its buttons are for manual use.
- Settings files from the earlier pick version load their `pick` position as `hit`.
- All parameters are in [config/pick_fruit.yaml](config/pick_fruit.yaml).

## HTTP API

`GET /api/state` returns the state as JSON. `POST /api/cmd` takes a JSON body
`{"cmd": ...}` with one of: `start`, `stop`,
`drive {linear, angular}` (−1..1), `jog {steps: [s1, s2]}`,
`goto {name}`, `gripper {deg}`, `speed {value}` (PWM), `save {name}`, `set_home`.
The camera stream is at `/stream.mjpg`.

## Test

```bash
colcon test --packages-select pick_fruit && colcon test-result --verbose
# or: cd src/pick_fruit && python3 -m pytest test
```
