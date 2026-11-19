# Phase 4: Connect PCT global planning to SCAN local planning

This phase finishes the algorithm interface of the full navigation chain. PCT searches a multi-floor global route on the tomogram and publishes it as `nav_msgs/msg/Path` on `/pct_path`. SCAN Mode 3 receives that route, builds a global reference that stays on the PCT polyline, and then rolls out local B-splines.

```text
tomogram pickle
    ↓
PCT native A* (across tomogram layers)
    ↓  /pct_path: nav_msgs/Path
3D downsample + RDP + piecewise-linear global reference
    ↓
SCAN rolling local B-spline
    ↓  planning/bspline
3D open-loop demo executor
```

## 1. Why `pct_scan_bridge` is needed

Different PCT scenes have different numbers of tomogram layers. A third-party ROS 2 wrapper hardcoded the native planner's layer count, so the C++ A* could build the map with the wrong memory layout. `pct_scan_bridge` reads `self.n_slice` from the pickle and passes the real layer count to the native planner. That fix applies to Building, Plaza, and Spiral.

## 2. Check A* without a ROS GUI first

Build the bridge package from the repository root:

```bash
source /opt/ros/humble/setup.bash
./scripts/build_workspace.sh --packages-select pct_scan_bridge scan_planner
```

Then run one scene:

```bash
./scripts/test_pct_building_astar.sh --force
./scripts/test_pct_plaza_astar.sh --force
./scripts/test_pct_spiral_astar.sh --force
```

The script prints the number of path points, the layers visited, the height gain, and the route length. If A* fails, first check that the endpoints snap to traversable cells, then check that the two endpoints are connected through a cross-floor gateway.

## 3. Why not hand every grid point to SCAN

PCT's output is a sequence of A* cells on the tomogram grid. Keeping every cell is unnecessary and adds cost to later projection, display, and local-goal sampling.

The combined launch first removes duplicates by 3D distance, then runs 3D Ramer-Douglas-Peucker (RDP) simplification. RDP keeps stairs, ramps, and clear corners, and it limits the geometric error to the configured value.

The related launch parameters are:

```text
reference_path_min_distance:=0.15
reference_path_simplify_tolerance:=0.1
```

Do not fit the whole multi-floor route with one global minimum-snap pass. The curve can leave the PCT corridor at bends and floor connections. The current global reference is piecewise linear. Local smoothing is still done by SCAN's B-spline.

## 4. Start PCT + SCAN with one command

```bash
./scripts/launch_pct_scan_building.sh
```

The script does the following:

- sources ROS 2 Humble and this project's overlay;
- adds the GTSAM and OSQP runtime library paths;
- loads the Building PCD and tomogram;
- starts the PCT interactive waypoints, SCAN Mode 3, the kinematic robot, and RViz.

In RViz, select `Interact` and drag the `Start` and `Goal` spheres. Right-click either sphere and choose `Plan route`. PCT publishes a new path, and SCAN updates the global reference immediately.

To check fixed endpoints without opening RViz:

```bash
./scripts/launch_pct_scan_building.sh \
  use_rviz:=false \
  plan_on_startup:=true \
  publish_tomogram:=false \
  enable_interactive_markers:=false
```

The important success logs are:

```text
Trajectory with 874 poses published on /pct_path
Reference path reduced from 874 poses to 94 trajectory waypoints
Reference path accepted
Received trajectory 1
Received trajectory 2
Received trajectory 3
```

Both ends of `/pct_path` use `reliable + transient_local + depth 1`. Even if SCAN finishes initializing a little later, it still gets the latest path PCT stored, without running A* again.

## 5. How 3D tracking works

`closed_loop` `go2_kinematic_sim` takes a planar `cmd_vel` and integrates only x, y, and yaw, so it cannot be used for a multi-floor video. The paper-scene launcher uses `open_loop` by default: the executor samples the 3D B-spline from SCAN directly, and body z rises with the ramp.

Each local SCAN B-spline covers only one planning horizon. When a local segment ends, the state machine must plan the next segment until the global PCT goal. Otherwise the robot walks about one meter and returns to `WAIT_TARGET` by mistake. That fix and the multi-floor check are in [`tutorial_phase5_route_tracking.md`](tutorial_phase5_route_tracking.md). This executor is only for the RViz demo. It does not represent real dynamics or a low-level MoE policy.
