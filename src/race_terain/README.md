# race_terain

Drive the all-terrain course to the finish by following the centre line between
the two rows of things that line the road (posts, barriers, reflectors), using
the RealSense point cloud and the BNO055 IMU.

```
bringup.launch.py   ── bno055, wheel_control, mega_bridge          (hardware)
realsense2_camera   ── D415 point cloud, 424x240 @ 30 fps
race_terain_node    ── centre-line following + web UI :8080        ─► /mega/cmd_vel
```

## Run

```bash
colcon build --packages-select race_terain && source install/setup.bash
ros2 launch race_terain race_terain.launch.py
#   args: port:=/dev/ttyACM0  web_port:=8080  dry_run:=true  hardware:=false
# open http://<robot-ip>:8080/
```

1. Put the robot on the road between the two rows, pointing along it.
2. Press **START** and keep it still ~1 s (gyro bias). It drives, and stops by
   itself when there is nothing beside the road for `lost_hold_s` (the end).
3. **STOP** (or Space) any time.

Check `ros2 topic list` shows `/camera/camera/depth/color/points`; if your
realsense2_camera names it differently, set `cloud_topic`.

## How it works

- **Ground:** refitted on every cloud (RANSAC on the points below the camera
  axis), because the robot pitches on gravel, grass and the ramps. A fit far
  from the one at START is ignored. Points `min_height`–`max_height` above the
  ground are the things beside the road.
- **Centre line:** those points, plus remembered ones, go into a top-down map
  of the distance to the nearest object. The centre line is traced forward
  step by step towards the most clearance (up to half `road_width`), so it
  is the middle between the two rows and bends with them, also around the
  hairpin U-turn, where a straight line per side cannot work.
- **Memory:** points that left the view (the U-turn's inner posts) are kept
  for `memory_s` / `memory_range`. The robot's motion comes from the IMU yaw
  and the commanded speed, corrected each frame by matching the new points to
  the remembered ones (2D ICP), so a wrong speed guess or gyro drift does not
  smear the map.
- **Steering:** pure pursuit to a point `pursuit_dist` along the centre line,
  brought closer when the straight line to it would cut a bend. Slower while
  turning hard. If the way is blocked right ahead, it turns in place.

## Set up

Measure `road_width` (inner face to inner face) and `robot_width` in
[config/race_terain.yaml](config/race_terain.yaml). In the web page:

- **Height band:** watch the camera view. The posts and reflectors should be
  red and the ground green. Raise the bottom if gravel or the grass mat's
  rim turns red; keep it below the reflectors.
- **PWM:** linear = forward speed, angular = fastest turn. If it cuts the
  hairpin, lower linear or `pursuit_dist`; if it is sluggish in the U-turn,
  raise angular.

## Test

```bash
cd src/race_terain && python3 -m pytest test
```
A closed-loop simulation drives straight → hairpin → straight with both turn
directions, a wrong speed estimate (×0.6, ×1.5) and a drifting gyro.
