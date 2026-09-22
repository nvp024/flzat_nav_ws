#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
install_setup="$workspace_dir/install/setup.bash"
isaac_sim_path="${ISAAC_SIM_PATH:-}"

source "$workspace_dir/scripts/clean_snap_gui_environment.bash"

if [[ ! -f "$install_setup" ]] || \
   [[ ! -x "$workspace_dir/install/openarm_skeleton_v1_2_isaac/lib/openarm_skeleton_v1_2_isaac/openarm_isaac_sim.py" ]]; then
  "$workspace_dir/scripts/build_workspace.sh"
fi

if [[ -z "$isaac_sim_path" ]]; then
  for candidate in \
    "$HOME/isaacsim" \
    "$HOME/isaac-sim-5.0.0" \
    "$HOME/Downloads/isaac-sim-5.0.0" \
    "$HOME/.local/share/ov/pkg/isaac-sim-5.0.0"; do
    if [[ -f "$candidate/python.sh" ]]; then
      isaac_sim_path="$candidate"
      break
    fi
  done
fi

if [[ ! -f "$isaac_sim_path/python.sh" ]]; then
  echo "ERROR: Isaac Sim 5.0 python.sh was not found." >&2
  echo "Extract Isaac Sim, then run:" >&2
  echo "  export ISAAC_SIM_PATH=/absolute/path/to/isaac-sim-5.0.0" >&2
  echo "  ./scripts/run_isaac_nav2.sh" >&2
  exit 2
fi

export ISAAC_SIM_PATH="$isaac_sim_path"
"$workspace_dir/scripts/check_isaac_host.sh"

source /opt/ros/jazzy/setup.bash
source "$install_setup"
set -u
exec ros2 launch \
  openarm_skeleton_v1_2_isaac isaac_nav2.launch.py \
  "isaac_sim_path:=$isaac_sim_path" "$@"
