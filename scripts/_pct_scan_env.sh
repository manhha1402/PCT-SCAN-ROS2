#!/usr/bin/env bash
# Shared runtime environment. Source this file; do not execute it directly.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This helper must be sourced by a PCT-SCAN-ROS2 launch script." >&2
  exit 2
fi

PCT_SCAN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PCT_SCAN_PROJECT_ROOT="$(cd "$PCT_SCAN_SCRIPT_DIR/.." && pwd -P)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[pct-scan] ROS 2 Humble was not found at /opt/ros/humble." >&2
  return 2
fi

# Standalone colcon root is this repo. When it is built inside a parent
# workspace (robot_ws), install/setup.bash lives above the repo.
PCT_SCAN_INSTALL_SETUP="$PCT_SCAN_PROJECT_ROOT/install/setup.bash"
if [[ ! -f "$PCT_SCAN_INSTALL_SETUP" ]]; then
  _pct_scan_search="$PCT_SCAN_PROJECT_ROOT"
  while [[ "$_pct_scan_search" != "/" ]]; do
    _pct_scan_search="$(dirname "$_pct_scan_search")"
    if [[ -f "$_pct_scan_search/install/setup.bash" ]]; then
      PCT_SCAN_INSTALL_SETUP="$_pct_scan_search/install/setup.bash"
      break
    fi
  done
fi
if [[ ! -f "$PCT_SCAN_INSTALL_SETUP" ]]; then
  echo "[pct-scan] Workspace is not built. Run ./scripts/build_workspace.sh." >&2
  return 2
fi

# Generated ROS setup files legitimately inspect optional unset variables.
set +u
source /opt/ros/humble/setup.bash

# ~/.local numpy 2.x breaks Ubuntu's scipy, which pct_route_smoke imports.
export PYTHONNOUSERSITE=1

PCT_DEPS_ROOT="${PCT_DEPS_ROOT:-$PCT_SCAN_PROJECT_ROOT/.deps/install}"
if [[ -f "$PCT_DEPS_ROOT/gtsam-4.2/lib/libgtsam.so.4" ]]; then
  export LD_LIBRARY_PATH="$PCT_DEPS_ROOT/gtsam-4.2/lib:$PCT_DEPS_ROOT/osqp-1.0.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
elif [[ ! -e /usr/local/lib/libgtsam.so.4 && ! -e /usr/lib/libgtsam.so.4 ]]; then
  echo "[pct-scan] PCT dependencies are missing. Run ./scripts/setup_pct_dependencies.sh." >&2
  return 2
fi

source "$PCT_SCAN_INSTALL_SETUP"
set -u

export PCT_SCAN_ROS2_ROOT="$PCT_SCAN_PROJECT_ROOT"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PCT_SCAN_PROJECT_ROOT/.ros/log}"
mkdir -p "$ROS_LOG_DIR"
