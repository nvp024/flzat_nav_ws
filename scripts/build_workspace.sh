#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy" >&2
  exit 1
fi

for overlay_var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH; do
  if [[ "${!overlay_var:-}" == *"/home/hai/sim-workspace"* ]]; then
    echo "ERROR: legacy sim-workspace is sourced in $overlay_var." >&2
    echo "Open a fresh terminal before building the official workspace." >&2
    exit 2
  fi
done

cd "$workspace_dir"
source /opt/ros/jazzy/setup.bash
set -u
colcon build --symlink-install --event-handlers console_direct+
