#!/usr/bin/env bash
# Launch helper for path_follower with Tilix panes.
# Requirements: tilix, ros2 workspace already built (install/setup.bash exists).
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SETUP_SCRIPT="$ROOT_DIR/install/setup.bash"

if ! command -v tilix >/dev/null 2>&1; then
  echo "Tilix가 설치되어 있지 않습니다. 'sudo apt install tilix' 후 다시 실행하세요." >&2
  exit 1
fi

if [ ! -f "$SETUP_SCRIPT" ]; then
  echo "install/setup.bash 를 찾을 수 없습니다. colcon build 후 다시 실행하세요." >&2
  exit 1
fi

BASE_CMD="bash -lc 'cd \"$ROOT_DIR\" && source \"$SETUP_SCRIPT\" && %s; exec bash'"

# Start Static TF
tilix --title "Static TF" -e "$(printf "$BASE_CMD" "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom")" &

# Start Fake Robot
tilix --title "Fake Robot" -e "$(printf "$BASE_CMD" "ros2 run path_follower_pkg fake_robot")" &

# Start Path Follower + GUI
tilix --title "Follower + GUI" -e "$(printf "$BASE_CMD" "ros2 launch path_follower_pkg path_follower.launch.py")" &

# Start RViz
tilix --title "RViz" -e "$(printf "$BASE_CMD" "rviz2 -d rviz_config/path_follower.rviz")" &

wait
