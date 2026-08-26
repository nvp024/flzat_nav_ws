#!/usr/bin/env bash
set -euo pipefail

# Install the system dependencies for this workspace on Ubuntu 24.04.
# Run this script as your normal desktop user. It invokes sudo only for the
# package-manager and rosdep initialization operations that require root.

if [[ "${EUID}" -eq 0 ]]; then
  echo "ERROR: Run this script as your normal user, not with 'sudo'." >&2
  echo "The script will request sudo itself when needed." >&2
  exit 2
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "ERROR: ROS 2 Jazzy deb packages require Ubuntu 24.04 (noble)." >&2
  echo "Detected: ${PRETTY_NAME:-unknown operating system}" >&2
  exit 2
fi

echo "[1/6] Authenticating for system package installation"
sudo -v

echo "[2/6] Enabling Ubuntu Universe and installing repository tools"
sudo apt-get update
sudo apt-get install -y software-properties-common curl ca-certificates
sudo add-apt-repository -y universe

echo "[3/6] Configuring the official ROS 2 apt repository"
ros_apt_source_version="$({
  curl --fail --silent --show-error \
    https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest
} | awk -F'"' '/"tag_name"/ {print $4; exit}')"
if [[ -z "$ros_apt_source_version" ]]; then
  echo "ERROR: Could not determine the latest ros-apt-source release." >&2
  exit 1
fi

ros_source_deb="$(mktemp --suffix=.deb)"
trap 'rm -f "$ros_source_deb"' EXIT
curl --fail --location --show-error \
  --output "$ros_source_deb" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${VERSION_CODENAME}_all.deb"
sudo dpkg -i "$ros_source_deb"

echo "[4/6] Installing ROS 2 Jazzy, Gazebo, Nav2, SLAM and build tools"
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-pytest \
  python3-rosdep

echo "[5/6] Initializing rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
else
  echo "rosdep is already initialized; keeping the existing configuration."
fi
rosdep update

echo "[6/6] Verifying the installation"
source /opt/ros/jazzy/setup.bash
ros2 --help >/dev/null
colcon version-check >/dev/null 2>&1 || true
gz sim --versions

echo
echo "ROS 2 Jazzy dependencies are installed successfully."
echo "Return to Codex so it can build and test the workspace."
