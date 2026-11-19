#!/usr/bin/env bash
# Launch the RViz PCT + SCAN demo on MuJoCo scene 11_three_storey_house_F1.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"
PROJECT_ROOT="$PCT_SCAN_PROJECT_ROOT"
TOMOGRAM="$PROJECT_ROOT/assets/house_f1/tomogram/house_f1.pickle"
PCD_MAP="$PROJECT_ROOT/assets/house_f1/pcd/house_f1.pcd"

for required_path in "$TOMOGRAM" "$PCD_MAP"; do
  if [[ ! -e "$required_path" ]]; then
    echo "[pct-scan-house-f1] Missing required path: $required_path" >&2
    exit 2
  fi
done

exec ros2 launch pct_scan_bridge paper_scene_pct_scan.launch.py \
  scene_name:=custom tomogram_path:="$TOMOGRAM" pcd_map_file:="$PCD_MAP" \
  scene_start_x:=2.9 scene_start_y:=0.8 scene_start_z:=0.0 \
  scene_goal_x:=24.5 scene_goal_y:=18.0 scene_goal_z:=0.0 \
  init_x:=2.9 init_y:=0.8 init_z:=0.4 \
  map_size_x:=65.0 map_size_y:=65.0 map_size_z:=8.0 \
  planning_horizon:=0.8 max_vel:=0.6 "$@"
