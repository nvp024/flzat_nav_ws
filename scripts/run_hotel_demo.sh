#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
install_setup="$workspace_dir/install/setup.bash"

source "$workspace_dir/scripts/clean_snap_gui_environment.bash"

if [[ "${AMENT_PREFIX_PATH:-}" == *"/home/hai/sim-workspace"* ]]; then
  echo "ERROR: legacy sim-workspace overlay is active; use a fresh terminal." >&2
  exit 2
fi

if [[ ! -f "$install_setup" ]]; then
  "$workspace_dir/scripts/build_workspace.sh"
fi

source /opt/ros/jazzy/setup.bash
source "$install_setup"
set -u
exec ros2 launch openarm_skeleton_v1_2_gazebo hotel_demo.launch.py "$@"
