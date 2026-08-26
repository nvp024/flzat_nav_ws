#!/usr/bin/env bash
set -Eeuo pipefail

# One-file bootstrap and launcher for OpenArm Skeleton v1.2.
# It can run either from inside the repository or as a standalone downloaded
# file. A standalone copy clones the official main branch before continuing.

readonly REPOSITORY_URL="https://github.com/DinhPhuHai/openarm_skeleton_v1.2.git"
readonly REPOSITORY_BRANCH="main"
readonly DEFAULT_WORKSPACE_DIR="${HOME}/openarm_skeleton_v1.2_ws"

show_help() {
  cat <<'EOF'
OpenArm Skeleton v1.2 one-file launcher

Usage:
  ./START_OPENARM.sh [launch_argument:=value ...]

Examples:
  ./START_OPENARM.sh
  ./START_OPENARM.sh headless:=true use_rviz:=false
  OPENARM_WORKSPACE_DIR=/path/to/workspace ./START_OPENARM.sh

The launcher automatically:
  1. clones or fast-forward updates the official GitHub main branch;
  2. installs ROS 2 Jazzy dependencies when they are missing;
  3. resolves rosdep dependencies and builds the workspace;
  4. sources ROS 2 plus the workspace and starts Gazebo, SLAM, Nav2 and RViz.

Local source changes are never overwritten. If the repository is dirty, the
update step is skipped and the current local source is built and launched.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  show_help
  exit 0
fi

script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"

# File managers do not always attach a terminal to shell scripts. Re-open this
# same standalone file in an available desktop terminal when double-clicked.
if [[ ! -t 1 && "${OPENARM_TERMINAL_RELAUNCHED:-0}" != "1" ]]; then
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal -- env OPENARM_TERMINAL_RELAUNCHED=1 \
      bash "$script_path" "$@"
    exit 0
  elif command -v konsole >/dev/null 2>&1; then
    konsole -e env OPENARM_TERMINAL_RELAUNCHED=1 \
      bash "$script_path" "$@"
    exit 0
  elif command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -e env OPENARM_TERMINAL_RELAUNCHED=1 \
      bash "$script_path" "$@"
    exit 0
  fi
fi

on_error() {
  local exit_code=$?
  echo
  echo "ERROR: OpenArm startup stopped at line ${BASH_LINENO[0]}." >&2
  echo "Fix the message above, then run START_OPENARM.sh again." >&2
  if [[ "${OPENARM_TERMINAL_RELAUNCHED:-0}" == "1" && -t 0 ]]; then
    read -r -p "Press Enter to close this terminal..." _
  fi
  exit "$exit_code"
}
trap on_error ERR

if [[ "$EUID" -eq 0 ]]; then
  echo "ERROR: Run START_OPENARM.sh as the normal desktop user, not with sudo." >&2
  exit 2
fi

install_git_if_needed() {
  if command -v git >/dev/null 2>&1; then
    return
  fi

  if ! command -v apt-get >/dev/null 2>&1; then
    echo "ERROR: git is missing and apt-get is unavailable." >&2
    return 1
  fi

  echo "[1/6] Installing git"
  sudo apt-get update
  sudo apt-get install -y git
}

is_openarm_workspace() {
  local candidate=$1
  [[ -d "$candidate/.git" \
    && -f "$candidate/AGENTS.md" \
    && -f "$candidate/scripts/run_nav2_sim.sh" \
    && -f "$candidate/src/openarm_skeleton_v1_2_navigation/package.xml" ]]
}

install_git_if_needed

script_dir="$(cd -- "$(dirname -- "$script_path")" && pwd)"
workspace_dir="${OPENARM_WORKSPACE_DIR:-$DEFAULT_WORKSPACE_DIR}"

if is_openarm_workspace "$script_dir"; then
  workspace_dir="$script_dir"
elif [[ -e "$workspace_dir" ]] && ! is_openarm_workspace "$workspace_dir"; then
  echo "ERROR: $workspace_dir already exists but is not an OpenArm workspace." >&2
  echo "Move it or set OPENARM_WORKSPACE_DIR to another location." >&2
  exit 2
elif [[ ! -e "$workspace_dir" ]]; then
  echo "[1/6] Downloading OpenArm source from GitHub"
  mkdir -p -- "$(dirname -- "$workspace_dir")"
  git clone --branch "$REPOSITORY_BRANCH" --single-branch \
    "$REPOSITORY_URL" "$workspace_dir"
fi

echo "[2/6] Checking the official GitHub main branch"
if git -C "$workspace_dir" fetch origin "$REPOSITORY_BRANCH"; then
  current_branch="$(git -C "$workspace_dir" branch --show-current)"
  local_changes="$(git -C "$workspace_dir" status --porcelain)"

  if [[ "$current_branch" != "$REPOSITORY_BRANCH" ]]; then
    echo "WARNING: Current branch is '$current_branch', not main; update skipped."
  elif [[ -n "$local_changes" ]]; then
    echo "WARNING: Local source changes detected; update skipped to protect them."
  else
    git -C "$workspace_dir" merge --ff-only "origin/$REPOSITORY_BRANCH"
  fi
else
  echo "WARNING: GitHub is unavailable; continuing with the downloaded source."
fi

echo "[3/6] Checking ROS 2 Jazzy and system dependencies"
needs_install=false
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  needs_install=true
else
  # ros2 and most Jazzy commands enter PATH only after the base setup is sourced.
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  for required_command in ros2 colcon rosdep gz; do
    if ! command -v "$required_command" >/dev/null 2>&1; then
      needs_install=true
      break
    fi
  done
fi

if [[ "$needs_install" == "true" ]]; then
  "$workspace_dir/scripts/install_jazzy_dependencies.sh"
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "[4/6] Resolving package dependencies"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
if [[ ! -d "${HOME}/.ros/rosdep/sources.cache" ]]; then
  rosdep update
fi
rosdep install --from-paths "$workspace_dir/src" --ignore-src -r -y \
  --rosdistro jazzy

echo "[5/6] Building the workspace"
"$workspace_dir/scripts/build_workspace.sh"

echo "[6/6] Starting Gazebo + robot + SLAM + Nav2 + RViz"
echo "Workspace: $workspace_dir"
echo "Stop everything with Ctrl+C."
cd "$workspace_dir"
set +u
source "$workspace_dir/install/setup.bash"
set -u

set +e
"$workspace_dir/scripts/run_nav2_sim.sh" "$@"
launch_exit_code=$?
set -e

if [[ "${OPENARM_TERMINAL_RELAUNCHED:-0}" == "1" && -t 0 ]]; then
  echo
  read -r -p "OpenArm stopped. Press Enter to close this terminal..." _
fi
exit "$launch_exit_code"
