#!/usr/bin/env bash

set -uo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./run_teleoperation_episodes.sh <l|f> <trial_name> <episode_count> [start_episode] [robot_ip]

Arguments:
  l|f             l = leader, f = follower
  trial_name      Parent directory name under data/
  episode_count   Number of episodes to run; 0 means run until Ctrl+C
  start_episode   First episode number (default: 1)
  robot_ip        Local Franka IP (default: 192.168.3.100)

Example:
  ./run_teleoperation_episodes.sh f wipe_whiteboard 10
  ./run_teleoperation_episodes.sh l wipe_whiteboard 10
EOF
}

if (( $# < 3 || $# > 5 )); then
  usage >&2
  exit 2
fi

role=$1
trial_name=$2
episode_count=$3
start_episode=${4:-1}
robot_ip=${5:-192.168.3.100}

if [[ $role != "l" && $role != "f" ]]; then
  echo "Error: role must be 'l' or 'f'." >&2
  exit 2
fi

if [[ ! $trial_name =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "Error: trial_name may contain only letters, numbers, '.', '_' and '-', and must not start with '.'." >&2
  exit 2
fi

if [[ ! $episode_count =~ ^[0-9]+$ ]]; then
  echo "Error: episode_count must be a non-negative integer." >&2
  exit 2
fi

if [[ ! $start_episode =~ ^[1-9][0-9]*$ ]]; then
  echo "Error: start_episode must be a positive integer." >&2
  exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
build_dir="$script_dir/build"
executable="$build_dir/TelePandaTDPA2010AsWhole"
follower_config="$script_dir/src/follower_config.json"

if [[ ! -x $executable ]]; then
  echo "Error: executable not found: $executable" >&2
  echo "Build it first with: cmake -S '$script_dir' -B '$build_dir' && cmake --build '$build_dir' -j" >&2
  exit 1
fi

if [[ $role == "f" ]]; then
  if ! grep -Eq '"record_data"[[:space:]]*:[[:space:]]*true' "$follower_config" ||
     ! grep -Eq '"record_camera"[[:space:]]*:[[:space:]]*true' "$follower_config"; then
    echo "Error: follower recording requires record_data=true and record_camera=true in:" >&2
    echo "  $follower_config" >&2
    exit 1
  fi
fi

stop_requested=false
active_pid=""

request_stop() {
  stop_requested=true
  echo
  echo "Stop requested; terminating the current episode..."
  if [[ -n $active_pid ]]; then
    kill -INT "$active_pid" 2>/dev/null || true
  fi
}

trap request_stop INT TERM

episode=$start_episode
completed=0

echo "Role:          $role"
echo "Trial name:    $trial_name"
echo "First episode: $(printf '%03d' "$start_episode")"
if (( episode_count == 0 )); then
  echo "Episode count: unlimited (stop with Ctrl+C)"
else
  echo "Episode count: $episode_count"
fi
echo

while (( episode_count == 0 || completed < episode_count )); do
  episode_name=$(printf 'episode_%03d' "$episode")
  episode_dir="$script_dir/data/$trial_name/$episode_name"

  if [[ $role == "f" && -e $episode_dir ]]; then
    echo "Error: refusing to overwrite existing episode directory:" >&2
    echo "  $episode_dir" >&2
    exit 1
  fi

  echo "============================================================"
  echo "Starting $trial_name/$episode_name"
  echo "============================================================"

  (
    cd "$build_dir"
    exec "$executable" "$robot_ip" "$role" "$trial_name" "$episode"
  ) &
  active_pid=$!
  wait "$active_pid"
  exit_code=$?
  active_pid=""

  if [[ $stop_requested == true ]]; then
    echo "Teleoperation loop stopped by user."
    exit 130
  fi

  if (( exit_code != 0 )); then
    echo "Error: $episode_name exited with status $exit_code; stopping the loop." >&2
    exit "$exit_code"
  fi

  echo "Finished $trial_name/$episode_name"
  completed=$((completed + 1))
  episode=$((episode + 1))
done

echo
echo "Completed $completed episode(s) for trial '$trial_name'."
