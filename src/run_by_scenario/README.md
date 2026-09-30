# run_by_scenario

Record a teleop drive of the base as a **scenario** script, then play it back.
The web UI has only the teleop (**W/A/S/D** or an on-screen **joystick**), the
**linear and angular PWM**, **Record / STOP**, and a scenario list with
**Load / START**.

```
bringup.launch.py     ── bno055, wheel_control, mega_bridge      (hardware)
run_by_scenario_node  ── record / playback + web UI :8080        ─► /mega/cmd_vel
```

## Run

```bash
colcon build --packages-select run_by_scenario && source install/setup.bash
ros2 launch run_by_scenario run_by_scenario.launch.py
#   args: port:=/dev/ttyACM0  web_port:=8080
# open http://<robot-ip>:8080/
```

Keep the robot still for the first few seconds while wheel_control calibrates
the gyro. Do **not** run it together with `pick_fruit`: both drive
`/mega/cmd_vel` and both use port 8080 by default.

## Workflow

1. **Pick the control.** The switch above the pad selects:
   - **W A S D**: hold keys or buttons, full PWM while held; W+A etc. combine.
   - **Joystick**: drag the knob with a mouse or finger. Up/down = linear,
     left/right = turn, proportional to how far you drag; let go = stop.
     A **gamepad** (USB/Bluetooth, plugged into the device running the browser)
     works in this mode too: left stick drives the same way, **B / circle** =
     STOP. Press any button once so the browser detects it; its name then shows
     under the joystick.

   The choice is saved and comes back after a restart.
2. **Set the PWM.** Two sliders, then **Set**:
   - **linear**: base PWM (0–255) at full W/S or full joystick up/down,
     sent as `linear.x = pwm / k_lin`.
   - **angular**: turn PWM at full A/D or full joystick left/right, sent as
     `angular.z = pwm / k_ff`. That makes it `wheel_control`'s turn
     feedforward; its yaw-rate loop still holds the turn, so the actual
     wheel difference can differ a little.

   `k_lin` and `k_ff` are read from `wheel_control.yaml` by the launch file.
3. **Record.** Type a name (letters, digits, `_`, `-`; empty = a timestamp) and
   press **Record**. Drive the robot. You may change the PWM while recording,
   and the change is recorded too.
4. **STOP** (or Space). The scenario is saved to
   `~/.ros/scenarios/<name>.json` and loaded, ready to play.
5. **Play.** Later, pick a scenario from the list, press **Load**, put the robot
   at the same start pose and press **START**. The robot is driven exactly as
   it was commanded while recording. It stops by itself at the end. STOP aborts.

While a scenario plays, teleop, the PWM and the control switch are locked.
The PWMs, the control mode and the last loaded scenario are kept in
`~/.ros/run_by_scenario.json` across restarts.

| key | action |
|---|---|
| W / S | drive forward / back while held (W A S D mode) |
| A / D | turn left / right while held (W A S D mode) |
| Space | STOP (both modes) |

## What is recorded

The **command** the base received, not the key presses or joystick
positions: a list of change points in PWM, each held until the next one. A
W/A/S/D and a joystick recording are the same kind of file.

```json
{
 "version": 2, "name": "row1", "created": "2026-09-30T11:07:21", "duration_s": 3.1,
 "steps": [
  {"t": 0.0,  "linear": 60.0, "angular": 0.0},
  {"t": 2.35, "linear": 60.0, "angular": 20.0},
  {"t": 3.1,  "linear": 0.0,  "angular": 0.0}
 ]
}
```

- `linear` is the signed base PWM (− = backwards), `angular` the signed turn
  PWM (+ = left). Playback sends them exactly as recorded, whatever the PWM
  sliders are set to now.
- Commands are rounded to whole PWM counts, so joystick jitter does not add a
  step every 20 ms.
- Version 1 files (`pwm` plus `angular` in rad/s) still load: their angular is
  converted with `angular × k_ff`.
- The last step must be `0, 0`: it marks the end (`duration_s` is only informative).
- Idle time before the first key press and after the last release is cut, so
  playback starts moving at once.
- A deadman stop while recording (the browser stopped refreshing a held key) is
  recorded as a stop, because that is what the robot did.
- Times are sampled at `control_rate_hz` (50 Hz → 20 ms).
- The file is plain JSON and can be edited by hand. The file name is the
  scenario name.

Playback is **timed**, not closed-loop on position: there are no wheel
encoders. `wheel_control`'s heading hold still keeps straight segments
straight, but distance depends on battery, floor and load. Expect some
drift over long scenarios, and record at the PWM you will play back on.

## Safety

| condition | action |
|---|---|
| no `/mega/status` for `status_timeout_s` | START refused; a playback is aborted (base 0), shown in red |
| a teleop key held but the browser stops refreshing | base stops after `teleop_timeout_s` |
| end of scenario, STOP, or node shutdown | base 0 (sent for `zero_burst_s`) |

## HTTP API

`GET /api/state` returns the state as JSON. `POST /api/cmd` takes a JSON body
`{"cmd": ...}` with one of: `drive {linear, angular}` (−1..1, + = forward / left),
`pwm {linear, angular}`, `control {mode}` (`wasd` | `joystick`), `record {name, overwrite}`, `stop`, `load {name}`, `start`,
`delete {name}`.

## Test

```bash
colcon test --packages-select run_by_scenario && colcon test-result --verbose
# or: cd src/run_by_scenario && python3 -m pytest test
```
