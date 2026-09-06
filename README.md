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
"gripper_grasp_force": 70.0
```

Only the follower writes the recorded files. `gripper_grasp_force` controls the
follower grasp force in newtons and must be greater than `0` and no more than
`70`.

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
  cam1.mp4
  cam2.mp4
```

For repeated recording, run the loop script on both computers with the same
trial name and episode count. Start the follower script first, then the leader
script:

```bash
# Follower computer
cd ~/teleoperation/vla_finetune
./run_teleoperation_episodes.sh f wipe_whiteboard 10
```

```bash
# Leader computer
cd ~/teleoperation/vla_finetune
./run_teleoperation_episodes.sh l wipe_whiteboard 10
```

After an episode ends, the next episode starts immediately and performs robot
and gripper initialization again. Pass a fourth argument to start from another
episode number, for example `... 10 11` starts ten episodes at
`episode_011`. An episode count of `0` repeats until `Ctrl+C`. The follower
script refuses to overwrite an existing episode directory.

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
Finished moving to initial joint configuration.
TDPA initialize done
```

Leader:

```text
[Gripper Init] Homing succeeded.
[Leader Gripper] Homed and left open at width ... m.
Finished moving to initial joint configuration.
TDPA initialize done
```

After startup, the follower ignores closed or stale initial values. It first
confirms that the leader gripper is open, then treats a confirmed open-to-close
transition as one grasp command. A failed grasp is attempted only once; the
leader must reopen and close again before another attempt.

Both sides should also print UDP receive diagnostics:

```text
[UDP recv] packets=... from=... q_delta_norm=... dq_norm=...
```

Both robots move to the low collection posture
`[0.307272, 0.323924, -0.112529, -2.501686, -0.012559, 2.764401, 0.833281]`
rad before teleoperation starts. This is the same default start posture used by
the `threading_real` deployment scripts.

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
