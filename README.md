# FlexiGrasp

Tactile sensing for a two-finger gripper. Two FlexiTac pads (GD510 film, 16x32 taxels each, one per
finger) are read over USB serial by a ROS 2 node that publishes each pad as a topic, and a live
viewer shows both pads side by side. It runs on its own for now and is meant to be merged into the
Open Manipulator X6 workspace later.

```
src/flexigrasp_msgs/   TactileFrame message (raw values, baseline-subtracted values, contact features)
src/flexigrasp/        flexigrasp_node (serial -> topics), flexigrasp_gui (live view), launch, config
```

## One-time setup

```bash
sudo apt install python3-serial python3-tk python3-matplotlib python3-numpy
sudo usermod -aG dialout $USER        # then log out and back in
```

## Build

```bash
cd ~/FlexiGrasp
rm -rf build install log              # only if you built before without --symlink-install
colcon build --symlink-install
source install/setup.bash             # in every new terminal
```

## Run

Left pad on `/dev/ttyUSB0`, right pad on `/dev/ttyUSB1` (change in `src/flexigrasp/config/flexigrasp.yaml`).

```bash
ros2 launch flexigrasp flexigrasp.launch.py              # node + live viewer
ros2 launch flexigrasp flexigrasp.launch.py demo:=true   # simulated pads, no hardware
ros2 launch flexigrasp flexigrasp.launch.py gui:=false   # node only
```

Keep both pads unloaded (gripper open, nothing touching) for the first second after launch while the
baselines are taken. Re-zero later with the viewer buttons, `r`, or
`ros2 service call /flexigrasp/rezero_all std_srvs/srv/Trigger`.

Viewer keys: `r` re-zero both, `v` raw/delta, `+`/`-` color range, `space` pause, `q` quit.

## Topics

| Topic | Type |
|---|---|
| `/flexigrasp/left/frame`, `/flexigrasp/right/frame` | `flexigrasp_msgs/TactileFrame` (~98 Hz) |
| `/flexigrasp/left/image`, `/flexigrasp/right/image` | `sensor_msgs/Image` (mono8, for `rqt_image_view`) |

`TactileFrame.total` (sum of delta above the contact threshold) is the contact/grip signal;
`in_contact`, `contact_taxels`, `centroid_row/col` describe where the contact is.

## Record

```bash
ros2 bag record /flexigrasp/left/frame /flexigrasp/right/frame /joint_states
```

## If something goes wrong

- `Permission denied` on the port: not in the `dialout` group (see setup).
- Left and right swapped: the USB numbers can change; see `src/flexigrasp/udev/99-flexigrasp.rules.example`.
- Viewer says NO DATA: the node is not running or the ports are wrong; read the node's log lines.
- Build error about a symbolic link: delete `build install log` and rebuild with `--symlink-install`.