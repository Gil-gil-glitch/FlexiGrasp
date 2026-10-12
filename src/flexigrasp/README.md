# FlexiGrasp

Two-finger FlexiTac (GD510, 16x32 taxels) tactile sensing for ROS 2: one node reads both pads and
publishes them, and a live viewer shows both side by side. Standalone for now; it is meant to be
dropped into the Open Manipulator X6 workspace later.

```
flexigrasp_ws/src/
  flexigrasp_msgs/   TactileFrame.msg            (ament_cmake)
  flexigrasp/        node, viewer, launch, config (ament_python)
```

## Build

```bash
sudo apt install python3-serial python3-tk python3-matplotlib python3-numpy   # once
sudo usermod -aG dialout $USER                                               # once, then log out/in
cd flexigrasp_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch flexigrasp flexigrasp.launch.py              # node + viewer, pads on /dev/ttyUSB0 and /dev/ttyUSB1
ros2 launch flexigrasp flexigrasp.launch.py gui:=false   # node only
ros2 launch flexigrasp flexigrasp.launch.py demo:=true   # simulated pads, no hardware
ros2 run flexigrasp flexigrasp_node --ros-args -p left.port:=/dev/ttyUSB1 -p right.port:=/dev/ttyUSB0
```

Keep both pads **unloaded** (gripper open, nothing touching) for the first ~0.3 s after start: the
node takes a per-taxel median baseline from the first `baseline_frames` frames. Re-zero any time
with the viewer button, `r`, or the services below.

## Topics, services

| Name | Type | Notes |
|---|---|---|
| `/flexigrasp/left/frame`, `/flexigrasp/right/frame` | `flexigrasp_msgs/TactileFrame` | every frame (~98 Hz), best-effort sensor QoS |
| `/flexigrasp/left/image`, `/flexigrasp/right/image` | `sensor_msgs/Image` (mono8, 32x16) | delta clipped to 0..255, for `rqt_image_view` |
| `/flexigrasp/left/rezero`, `/flexigrasp/right/rezero` | `std_srvs/Trigger` | new baseline for one finger |
| `/flexigrasp/rezero_all` | `std_srvs/Trigger` | new baseline for both |

`TactileFrame` carries `raw` (uint8[512]), `delta` (float32[512], raw - baseline), and the features
`peak`, `total` (sum of delta above threshold, the number to use as a contact/grip signal),
`contact_taxels`, `in_contact`, `centroid_row/col`, plus link health (`rate_hz`, `bad_frames`,
`dropped_bytes`). Row-major, index = row*cols + col. `header.stamp` is the host receive time from
the system clock; with `use_sim_time` the stamps will not match sim time.

## Parameters (`config/flexigrasp.yaml`)

`finger_names`, `baud` (2000000), `baseline_frames` (30), `contact_threshold` (15 counts),
`contact_min_taxels` (2), `publish_image`, `topic_prefix`, `demo`, and per finger `<name>.port`,
`<name>.frame_id`, `<name>.mask` (taxels to ignore, `"0,10;0,11;1,30"`, useful for the drifting
top-edge taxels on the first-build pad).

## Recording (sync with the arm)

```bash
ros2 bag record /flexigrasp/left/frame /flexigrasp/right/frame /joint_states
```

Recording the pads and the arm's joint topics in one bag gives you a shared time base for the
arm tests, so a separate logger is not needed. For plots, read the bag and use `delta`/`total`.

## Stable port names

`/dev/ttyUSB0/1` can swap after a reboot or re-plug. If left/right matter, see
`udev/99-flexigrasp.rules.example`. Arduino Nano clones (CH340) usually have no serial number, so
match on the physical USB port rather than `/dev/serial/by-id`.

## Troubleshooting

- *"cannot open ... Permission denied"*: not in the `dialout` group (see Build).
- *"port open but no frames"*: wrong baud, no power, or firmware not streaming.
- *Rejected frames > 0*: a byte was lost on the wire; the frame was dropped, nothing is shifted.
  A steady count means a flaky cable.
- *Viewer says NO DATA*: the node is not running, or the topic prefix differs.
- Viewer needs a display and `python3-tk`.

## Tests (no ROS needed)

`cd src/flexigrasp && python3 -m pytest test` runs the parser, features, serial retry, a real
pyserial pty test, the node logic against stand-in ROS classes, and a headless render of the viewer.
