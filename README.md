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
For force estimates, prefer the sum of positive baseline-subtracted values (see Calibration below):
`total` has a dead zone at light loads.

## Record

```bash
ros2 bag record /flexigrasp/left/frame /flexigrasp/right/frame /joint_states
```

## Calibration

### Pad → load with coins (13 Nov 2026)

![Coin calibration: three repeats, loading and unloading](docs/coin_calibration.png)

**Setup.** Left pad on the X6 finger, facing up and level. 10-yen coins (4.5 g each, 23.5 mm across),
stacked centred with no plate, 0 → 45 g → 0 in 4.5 g steps, about 5 s per step, 3 repeats without
re-zeroing between them. The load was labelled in the bag with `coin_marker.py` (`/calib_load_g`).

**Key findings**

| | Sum of positive taxels | Published `total` |
|---|---|---|
| Response to load | Rises steadily to at least 45 g; slight bend above ~25 g | Rises to at least 45 g, but unevenly |
| Sensitivity | **~17 counts/g** (15–19 across repeats, R² 0.95–0.97), 23.5 mm contact | Not linear enough to quote one value |
| Smallest load seen | 4.5 g (first coin) | ~5–9 g (dead zone below that) |
| Spread between repeats | ±5–18% | ±20–40% |

- **No saturation up to 45 g.** The earlier "pads flatten out around 19 g" was wrong. The light weights
  in the 9 Oct test barely registered because of the dead zone at low load, not saturation.
- **Use the sum of positive taxels as the force signal** (`sum(max(delta, 0))` over the pad). `total`
  ignores taxels below the contact threshold, so it misses light touches and jumps when the contact
  shifts.
- **Placement matters.** In repeat 1, `total` stayed flat between 18 and 36 g and then jumped when the
  contact centre moved 2–3 taxels. The positive sum was much less affected.
- **Loading vs unloading:** no consistent hysteresis. The 10–40% differences, mostly at light loads,
  go both ways and come from handling the coins.
- **Settling:** readings still drifted during many 5 s holds (up to 20–40 counts/s). Hold calibration
  steps 8–10 s and use the last few seconds.
- **Zero offset after unloading:** about 10–50 counts (1–3 g). Re-zero before each grasp.

**Limits.** One pad, one contact size (23.5 mm disc) and loads up to 45 g only. The counts per gram
change with contact area, so check them against the load cell between the jaws before using them as
grip force.

### Next: load cell between the jaws

Squeeze a load cell (HX711) between the pads at stepped gripper current caps, holding each step
8–10 s. This gives current → force for the X6 gripper and pad → force at higher loads and grasp-sized
contacts.

## If something goes wrong

- `Permission denied` on the port: not in the `dialout` group (see setup).
- Left and right swapped: the USB numbers can change; see `src/flexigrasp/udev/99-flexigrasp.rules.example`.
- Viewer says NO DATA: the node is not running or the ports are wrong; read the node's log lines.
- Build error about a symbolic link: delete `build install log` and rebuild with `--symlink-install`.