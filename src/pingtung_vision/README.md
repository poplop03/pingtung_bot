# pingtung_vision

One ROS 2 node runs exactly one camera algorithm selected by the `algorithm`
parameter: `off`, `animal`, `fruit_color`, or `pig_shit`.

The launch file starts `v4l2_camera` on `/dev/video4` and feeds its
`sensor_msgs/Image` output to `/camera/image_raw` by default. Set
`start_camera:=false` when another camera driver already publishes the topic.

## Install camera driver

```bash
sudo apt update
sudo apt install ros-humble-v4l2-camera
```

## Run

```bash
ros2 launch pingtung_vision vision.launch.py vision_mode:=animal
ros2 launch pingtung_vision vision.launch.py vision_mode:=fruit_color
ros2 launch pingtung_vision vision.launch.py vision_mode:=pig_shit
```

Select another V4L2 device with `video_device`, for example:

```bash
ros2 launch pingtung_vision vision.launch.py \
  vision_mode:=fruit_color video_device:=/dev/video4
```

The mode can be changed without stopping the robot:

```bash
ros2 param set /vision algorithm fruit_color
```

Changing mode clears the pending camera frame and resets every temporal filter.
The PyTorch model is loaded only in animal mode.

## Results

| mode | topic | type |
|---|---|---|
| animal | `/vision/animal/result` | `pingtung_vision_interfaces/AnimalResult` |
| fruit_color | `/vision/fruit/line_hit` | `pingtung_vision_interfaces/FruitLineHit` |
| pig_shit | `/vision/pig_shit/spatial` | `pingtung_vision_interfaces/PigShitSpatial` |

Only the active result topic is updated. Each mode first publishes one message
with `valid=false`; normal camera results have `valid=true`. A camera timeout
also publishes `valid=false` once, allowing robot logic to reject stale output.

### Message contract

- `AnimalResult`: `animal_id` is 0=unknown, 1=dog, 2=monkey, 3=rabbit,
  4=turtle. The four named probabilities are in that fixed class order and are
  the same 0.75/0.25 EMA values used by the original camera display. ID becomes
  unknown when their maximum is below 0.65.
- `FruitLineHit`: `line_hit` is 1 only after a red/yellow candidate remains
  stable for four frames and its smoothed centroid is within 10 pixels (in y) of
  the horizontal image centre line. Green, empty, unstable, and off-centre
  frames publish 0.
- `PigShitSpatial`: the six fields reproduce the original left/right panels.
  They describe the share of detected object pixels on each side, not class
  probability or literal image occupancy. Unknown publishes Empty=100 on both
  sides. Values use the original 0.75/0.25 EMA and reset on label changes.

Inspect results with:

```bash
ros2 topic echo /vision/animal/result
ros2 topic echo /vision/fruit/line_hit
ros2 topic echo /vision/pig_shit/spatial
ros2 topic echo /vision/active_mode
```

Set `publish_debug_image:=true` to publish annotated images on
`/vision/debug_image`. No `cv2.imshow()` window is opened.

## Web view

The launch file also starts `web_view_node` (default `web_view:=true`), which
streams `/vision/debug_image` to a browser, replacing the `cv2.imshow()` window
of `pythonScript/fruit_color_opencv.py --camera`:

```bash
ros2 launch pingtung_vision vision.launch.py vision_mode:=fruit_color
# open http://<robot-ip>:8080/
```

Endpoints: `/` (page), `/stream.mjpg` (MJPEG), `/snapshot.jpg` (latest frame).
Use `web_port:=9000` for another port, or `web_view:=false` to disable the
server and the debug image.

Animal mode additionally requires a PyTorch and TorchVision build compatible
with the deployment machine (for Jetson, use the versions compatible with its
JetPack release). They are intentionally not installed by `setup.py`.
