#!/usr/bin/env bash
# Headless single-floor PCT A* smoke test for 11_three_storey_house_F1.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"
PROJECT_ROOT="$PCT_SCAN_PROJECT_ROOT"
TOMOGRAM="$PROJECT_ROOT/assets/house_f1/tomogram/house_f1.pickle"
ROUTE_OUTPUT="$PROJECT_ROOT/assets/house_f1/routes/f1_demo.csv"

if [[ ! -e "$TOMOGRAM" ]]; then
  echo "[house-f1-astar] Missing $TOMOGRAM; run ./scripts/generate_house_f1_tomogram.sh" >&2
  exit 2
fi
mkdir -p "$(dirname "$ROUTE_OUTPUT")"

exec ros2 run pct_scan_bridge pct_route_smoke \
  --tomogram "$TOMOGRAM" \
  --output "$ROUTE_OUTPUT" \
  --start 2.9 0.8 0.0 \
  --goal 24.5 18.0 0.0 \
  --snap-radius 12 \
  "$@"
