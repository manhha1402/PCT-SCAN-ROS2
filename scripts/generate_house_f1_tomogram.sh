#!/usr/bin/env bash
# Build the PCT tomogram for MuJoCo scene 11_three_storey_house_F1 straight from
# the FAST-LIO prior map (PLY). 0.20 m resolution: at 0.10 m ~15% of floor cells
# have no points and become barriers. Margins are small enough to keep doorways open.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
SOURCE_PLY="$PROJECT_ROOT/../fastlio_localization/map/11_three_storey_house_F1.ply"

mkdir -p "$PROJECT_ROOT/assets/house_f1/tomogram"
exec "$SCRIPT_DIR/generate_tomogram.sh" \
  --input "$SOURCE_PLY" \
  --output "$PROJECT_ROOT/assets/house_f1/tomogram/house_f1.pickle" \
  --resolution 0.20 --ground-height -0.10 --slice-height 0.50 \
  --kernel-size 5 --interval-min 0.50 --interval-free 0.65 \
  --slope-max 0.40 --step-max 0.20 --standable-ratio 0.30 \
  --cost-barrier 50.0 --safe-margin 0.20 --inflation 0.10 \
  --backend cpu "$@"
