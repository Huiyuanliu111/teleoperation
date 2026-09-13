# Teleoperation Real Robot Setup

This repository contains the real-robot teleoperation workflow for two Franka
Emika Panda robots.

- Franka 3: leader
- Franka 4: follower
- Goal: the follower robot copies the leader robot motion.

The active executable is in `vla_finetune`. The `remote_controller` folder is a
separate single-robot controller and is not used for the current two-robot
teleoperation test.

## Network Layout

Each robot controller PC is directly connected to its own Franka robot. The two
controller PCs communicate with each other through the normal external network.

- Leader controller PC: `panda@10.157.175.16`
- Follower controller PC: `truphysics@10.157.175.22`
- Franka robot IP on each direct robot link: `192.168.3.100`
- Teleoperation UDP port: `5001`

The robot IP passed to the executable is always the local Franka robot IP,
usually `192.168.3.100`.

## Config Files

The current executable reads:

- Leader side: `vla_finetune/src/leader_config.json`
- Follower side: `vla_finetune/src/follower_config.json`

The `remote_ip` field must point to the other controller PC, not to the robot.

Leader config:

```json
"remote_ip": "10.157.175.22"
```

Follower config:

```json
"remote_ip": "10.157.175.16"
```

The code currently uses UDP port `5001`. Keep `local_port` and `remote_port`
set to `5001` for consistency.

Follower data and camera recording are enabled for collection:

```json
"record_data": true,
"record_camera": true,
"maze_cycle": true,
"automatic_lift_m": 0.02,
"leader_initialization_speed_factor": 0.05,
"lift_tolerance_m": 0.001,
"lift_velocity_tolerance_mps": 0.005,
"lift_hold_cycles": 200,
"camera_serials": {
  "cam1": "233722072293",
  "cam2": "233622071984",
  "cam3": "233522077069"
},
"camera_enabled": {
  "cam1": true,
  "cam2": false,
  "cam3": true
},
"gripper_grasp_force": 70.0
```

Only the follower writes recorded files. `gripper_grasp_force` controls its
grasp force in newtons and must be greater than `0` and no more than `70`.
With `maze_cycle` enabled, the follower publishes its current seven-joint pose
at startup and holds that pose. The leader reads it over UDP and automatically
moves to the same joint pose at the configured speed factor. Closing the leader
gripper then closes the follower gripper, establishes a fresh motion baseline,
and makes both robots rise 2 cm. Recording starts only after both TCPs have
settled at their new heights. That follower TCP z is held for the episode and
saved as `fixed_tcp_z_m`.

## Controller PC Requirements

Install the required packages on both controller PCs:

```bash
sudo apt update
sudo apt install cmake build-essential libopencv-dev nlohmann-json3-dev
```

The Franka stack must also be installed on both PCs:

- `libfranka`
- Franka CMake package discoverable by CMake
- Real-time capable kernel or verified real-time scheduling permissions

ROS is not used by this collector and must not be installed just for data
collection. The build links directly against the follower's existing standalone
`libfranka`, RealSense SDK, OpenCV, and Poco installations.

Check real-time status:

```bash
uname -a
grep -E '^CONFIG_PREEMPT|^CONFIG_HZ_' /boot/config-$(uname -r)
ulimit -r
id -nG
chrt -f 80 true && echo "Realtime permission OK"
```

The user should be in the `realtime` group and `chrt` should succeed.

If using a XanMod PREEMPT_RT kernel, `libfranka` may fail to detect real-time
support through `/sys/kernel/realtime`. The current code uses
`franka::RealtimeConfig::kIgnore` after real-time scheduling was verified
manually.

## Sync Project To Controller PCs

From the development machine:

```bash
rsync -av --progress \
  --exclude='build/' \
  --exclude='data/' \
  --exclude='.venv/' \
  --exclude='__pycache__/' \
  /home/huiyuan/teleoperation/ \
  panda@10.157.175.16:/home/panda/teleoperation/
```

```bash
rsync -av --progress \
  --exclude='build/' \
  --exclude='data/' \
  --exclude='.venv/' \
  --exclude='__pycache__/' \
  /home/huiyuan/teleoperation/ \
  truphysics@10.157.175.22:/home/truphysics/teleoperation/
```

These commands do not use `--delete`, so existing remote build folders and data
folders are preserved.

## Build

Run on both controller PCs:

```bash
cd ~/teleoperation/vla_finetune
cmake -S . -B build
cmake --build build -j
```

If CMake reports that `nlohmann_json` is missing:

```bash
sudo apt update
sudo apt install nlohmann-json3-dev
```

If CMake reports that OpenCV is missing:

```bash
sudo apt install libopencv-dev
```

## Run Teleoperation

Start the follower first:

```bash
cd ~/teleoperation/vla_finetune/build
./TelePandaTDPA2010AsWhole 192.168.3.100 f test_session 1
```

Then start the leader:

```bash
cd ~/teleoperation/vla_finetune/build
./TelePandaTDPA2010AsWhole 192.168.3.100 l test_session 1
```

Use the same `trial_name` and `episode_idx` on both sides for one episode.

The arguments are:

```text
./TelePandaTDPA2010AsWhole <local_robot_ip> <l_or_f> <trial_name> <episode_idx>
```

- `<local_robot_ip>`: the robot connected to this controller PC, usually
  `192.168.3.100`
- `<l_or_f>`: `l` for leader, `f` for follower
- `<trial_name>`: name of the complete collection, for example `wipe_whiteboard`
- `<episode_idx>`: integer episode number

Recordings use the following directory structure:

```text
vla_finetune/data/<trial_name>/episode_001/
  DATA_follower.m
  recording_manifest.json
  cam1.mp4
  cam1_depth.z16.zst
  cam1_timestamps.csv
  cam1_metadata.json
  cam3.mp4
  cam3_depth.z16.zst
  cam3_timestamps.csv
  cam3_metadata.json
```

The camera mapping is `cam1=sideview`, `cam2=wrist`, and `cam3=frontview`.
The current Threading configuration disables `cam2`, so only the side and front
views are connected and recorded. Camera serial numbers and enabled states are
configured in `follower_config.json`. Verify the mapping physically on the
follower before collection; enumeration alone cannot tell which viewpoint a
camera occupies:

```bash
cd ~/teleoperation/vla_finetune/build
./TestRealSense --list
./TestRealSense --preview <serial>
```

`--list` is headless and does not require ROS. `--preview` opens RGB and aligned
depth windows; run it once per serial and assign the observed role in
`src/follower_config.json`.

Each camera runs in its own capture thread. Depth is aligned to color and each
640x480 little-endian Z16 frame is compressed independently with lossless Zstd
level 1. The timestamp CSV stores its byte offset and compressed length, so the
converter retains random access and ignores a trailing partial write after an
error. Camera CSVs and the first column of the 30-column follower matrix use the
same `std::chrono::steady_clock` nanosecond clock. RealSense sensor timestamps
and frame numbers are retained as additional diagnostics. Legacy uncompressed
`camN_depth.z16` episodes remain supported.

The current two-camera configuration avoids the wrist stream's recording and
compression overhead. On the earlier smoke episodes, Zstd reduced one
representative depth stream from 407 MB to about 61 MB; the exact ratio depends
on the scene. Record to the follower's local SSD. Do not run
`trim_invisible_prefix.py` on RGB-D recordings; filter timestamped episodes
during conversion instead.

One pair of processes records repeated episodes. Start the follower process
first, then the leader process, using the same trial name and starting episode:

```bash
# Follower computer
cd ~/teleoperation/vla_finetune
./build/TelePandaTDPA2010AsWhole 192.168.3.100 f maze_data 1
```

```bash
# Leader computer
cd ~/teleoperation/vla_finetune
./build/TelePandaTDPA2010AsWhole 192.168.3.100 l maze_data 1
```

The fourth argument is the first episode number. Opening the leader gripper
ends and flushes the current episode. Keep the processes running, move the
leader manually to reset both robots, then close the gripper to start the next
2 cm lift and episode. Episode directories increment automatically. The
follower refuses to overwrite an existing episode directory.

Do not use Franka Desk guide mode while `robot.control(...)` is running. Guide
mode or the user stop button aborts the libfranka control command and causes:

```text
Move command aborted: User Stop pressed!
```

The leader should be moved by physically pushing it while the teleoperation
program is running.

## Expected Startup Logs

Follower:

```text
[Gripper Init] Homing succeeded.
[Follower Gripper] Homed. Current width ... m; initial grasp skipped.
[Maze Init] Holding the current follower pose while the leader initializes.
TDPA initialize done
[Recording] Waiting for leader gripper close.
```

Leader:

```text
[Gripper Init] Homing succeeded.
[Leader Gripper] Homed and left open at width ... m.
[Maze Init] Waiting for the follower's current joint pose.
[Maze Init] Moving leader to follower q=[...]
[Maze Init] Leader reached the follower pose. Close the leader gripper to start the first lift.
TDPA initialize done
```

After startup, the program first confirms that the leader gripper is open. A
confirmed close starts the synchronized lift. Expected transition logs include
`[Maze] Close confirmed`, `[Recording] Started episode ... after both robots
reached z=...`, and, after opening, `[Maze] Episode ended ... close again for
next episode`.

Both sides should also print UDP receive diagnostics:

```text
[UDP recv] packets=... from=... q_delta_norm=... dq_norm=...
```

There is no predefined numeric initial joint posture. Before the first close,
the follower holds the pose it had when the process started and the leader
automatically moves to that follower joint pose. Every close re-zeros the
leader/follower motion mapping.

Before collecting a full dataset, record two or three episodes and validate
all committed RGB-D frames and timestamp joins. Raw acquisition stays on the
follower. Run the Python validator on a machine that already has the `pushbox`
environment (the follower currently does not), either after copying those test
episodes or through a mounted follower data directory:

```bash
cd /home/huiyuan/teleoperation
conda run -n pushbox python validate_vla_rgbd.py \
  <path-to-follower-data>/<trial_name> \
  --output data/<trial_name>_rgbd_validation.json
```

The default limits are 25 ms between cameras and 5 ms between a reference
camera frame and the nearest robot row. Conversion uses the same limits:

```bash
conda run -n pushbox python convert_vla_to_lerobot_v3.py \
  vla_finetune/data/<trial_name> data/<dataset_name> \
  --repo-id local/<dataset_name> --task "insert the grasped block through the needle" \
  --image-size 224
```

For timestamped recordings this creates standard LeRobot RGB videos plus a
portable `rgbd/episode_XXXXXX/` sidecar containing original-resolution MP4s,
lossless time-aligned Z16 depth, source-frame indices, and robot/camera host
timestamps.
Passing `--skip-depth` is explicit opt-out and is not appropriate for point
cloud training. Legacy 29-column RGB recordings are still converted with the
old approximate normalized-progress mapping.

Build the point-cloud training sidecar directly from the new follower recording:

```bash
conda run -n pushbox python build_vla_pointcloud_dataset.py \
  vla_finetune/data/<trial_name> \
  data/<trial_name>_pointcloud_stride5.h5 \
  --calibration threading_real/calibration/block_grasp_spatial.json \
  --stride 5 --num-points 8192
```

This applies the same geometry used by `arp/real-robot`: aligned depth is
deprojected with the recorded intrinsics, transformed into the Franka base frame,
cropped to the workspace, and sampled to a fixed point count. The output stores
base-frame XYZ, RGB, camera provenance, synchronized robot state, and the next
stride-5 state. The default base-frame crop is
`[0.15, -0.40, -0.15]` to `[0.75, 0.30, 0.50]` metres and can be changed with
`--bounds XMIN YMIN ZMIN XMAX YMAX ZMAX` after inspecting a trial.

Only `sideview` and `frontview` are recorded and fused because their base
extrinsics are already calibrated. The wrist camera is disabled and would
require a separate hand-eye calibration before its depth could be transformed
into the base frame.

On the follower, packets should come from `10.157.175.16`. On the leader,
packets should come from `10.157.175.22`.

After the first UDP packet, a `100 ms` communication watchdog is active. If no
new packet arrives within that interval, the current episode stops instead of
continuing with stale motion commands. The follower tracks the leader's joint
position delta directly. Application-level joint soft limits are not enabled;
joint-limit protection is provided by libfranka and the robot controller.

## UDP Debugging

If the follower does not move, first check whether UDP packets arrive.

On the follower PC:

```bash
sudo tcpdump -ni any 'udp port 5001 and host 10.157.175.16'
```

On the leader PC:

```bash
sudo tcpdump -ni any 'udp port 5001 and host 10.157.175.22'
```

Check whether the program is bound to UDP port `5001`:

```bash
sudo ss -uapn | grep 5001
```

`Connection refused` for `localhost:6666` is not the inter-PC teleoperation
link. It is only the optional local visualization publisher. The current code
disables that publisher by default.

## Common Errors

### `nlohmann_json` not found

Install the system package:

```bash
sudo apt install nlohmann-json3-dev
```

### `OpenCVConfig.cmake` not found

Install OpenCV development files:

```bash
sudo apt install libopencv-dev
```

### `realsense2Config.cmake` or `librealsense2/rs.hpp` not found

The RGB-D recorder requires the RealSense development package, not only the
runtime tools:

```bash
sudo apt install librealsense2-dev
```

### `Running kernel does not have realtime capabilities`

Boot into a real-time kernel and verify scheduling:

```bash
uname -a
grep -E '^CONFIG_PREEMPT|^CONFIG_HZ_' /boot/config-$(uname -r)
ulimit -r
id -nG
chrt -f 80 true && echo "Realtime permission OK"
```

### `motion aborted by reflex ["communication_constraints_violation"]`

This means the Franka 1 kHz control loop missed communication timing
constraints. For debugging:

- Disable camera and data recording with `record_camera=false` and
  `record_data=false`.
- Avoid excessive terminal logging.
- Make sure the PC is running with real-time scheduling permissions.
- Do not run other heavy processes during control.

### `Move command aborted: User Stop pressed!`

The user stop button or Franka Desk guide mode interrupted the active libfranka
control command. Do not use guide mode during teleoperation.
