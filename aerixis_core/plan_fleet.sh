#!/usr/bin/env bash
set -e
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$task_root/install/setup.bash"
source "$task_root/rescue_ws/install/setup.bash"
exec ros2 run as2_rescue_planner plan_fleet "$@"
