#!/usr/bin/env bash
set -o pipefail

echo "OpenArm Isaac Sim host check"
echo "OS: $(. /etc/os-release && echo "$PRETTY_NAME")"
echo "Kernel: $(uname -r)"
echo "Memory: $(free -h | awk '/^Mem:/ {print $2 " total, " $7 " available"}')"
echo "Swap: $(free -h | awk '/^Swap:/ {print $2 " total, " $3 " used"}')"
echo "NVIDIA module: $(cat /proc/driver/nvidia/version 2>/dev/null | head -1 || echo unavailable)"

driver_version=""
driver_compatible=true
if command -v nvidia-smi >/dev/null 2>&1; then
  if nvidia-smi --query-gpu=name,driver_version,memory.total \
      --format=csv,noheader; then
    driver_version="$(
      nvidia-smi --query-gpu=driver_version --format=csv,noheader |
        head -1 | tr -d '[:space:]'
    )"
  else
    echo "WARN: nvidia-smi cannot access the GPU from this environment." >&2
  fi
else
  echo "WARN: nvidia-smi is not installed." >&2
fi

isaac_sim_path="${ISAAC_SIM_PATH:-}"
if [[ -n "$isaac_sim_path" && -f "$isaac_sim_path/python.sh" ]]; then
  isaac_version="$(head -1 "$isaac_sim_path/VERSION" 2>/dev/null || true)"
  echo "Isaac Sim: $isaac_sim_path (${isaac_version:-unknown version})"
  driver_major="${driver_version%%.*}"
  if [[ "$isaac_version" == 5.* ]] &&
     [[ "$driver_major" =~ ^[0-9]+$ ]] &&
     (( driver_major >= 595 )); then
    echo "ERROR: Isaac Sim 5.x crashes in the RTX renderer with NVIDIA" >&2
    echo "driver $driver_version on this host. Install the validated 580" >&2
    echo "driver branch, reboot, and run this check again." >&2
    driver_compatible=false
  fi
  if [[ "$driver_compatible" == true ]]; then
    exit 0
  fi
  exit 3
fi

echo "Isaac Sim: not found; set ISAAC_SIM_PATH after extracting 5.0." >&2
exit 2
