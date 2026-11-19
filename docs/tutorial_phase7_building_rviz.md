# Phase 7: RViz-only PCT + SCAN navigation on the Building map

This phase switches the main demo to Building, which is easier to record and to explain. The system does not start Gazebo and does not compute contact dynamics. PCT searches a cross-floor route on the tomogram. SCAN turns the global polyline into a continuous local B-spline. The open-loop controller updates the Go2 pose along that trajectory. RViz shows the map, the route, the local trajectory, and the robot model at the same time.

## 1. Data flow and responsibilities

```text
building2_9.pickle
        │
        ▼
PCT A* ─────── /pct_path (orange global route)
                         │
                         ▼
                 SCAN Mode 3
                         │
                         ▼
             /planning/bspline (local trajectory)
                         │
                         ▼
             open_loop_controller
                         │
                         ▼
              /quad_0/body_pose ──► RViz Go2

building2_9.pcd ──────────────────► RViz Global Map
```

The PCD and the pickle come from the same Building point cloud and both use the `world` frame, so no extra rotation, translation, or scale is applied. `z` on the PCT path is the ground height. SCAN generates the body height from `grid_map.body_height=0.4 m`.

## 2. Why simulated local sensing is off by default

SCAN's original local sensor is built for free-space 3D point clouds and inflates obstacle points with a double cylinder. Building stairs are a dense step cloud. PCT's terrain model treats them as traversable, but SCAN's default `0.25 m` radius inflates the narrow stair corridor shut. Rendering the simulated lidar cloud also uses a lot of CPU while a video is being recorded.

The default is therefore `enable_local_sensing:=false`. The PCD is still shown as a clear RViz scene. Traversability comes from the PCT tomogram. SCAN still samples the global route, builds the local B-spline, and replans continuously. That matches the goal of a video demo without Gazebo. It is not a configuration for checking local obstacle-avoidance safety on a real robot.

Do not add `enable_local_sensing:=true` for the current video demo. With it on, the dense steps enter SCAN's collision inflation. After local A* or B-spline refinement fails repeatedly, the state machine enters `EMERGENCY_STOP/WAIT_TARGET`, and the dog stops while walking. The launcher prints a warning about that risk.

Turn it on only when you are studying SCAN's original point-cloud collision layer:

```bash
./scripts/launch_pct_scan_building.sh enable_local_sensing:=true
```

After you turn it on, the PCT terrain footprint and the SCAN collision model still need to be aligned. Do not use this option as the acceptance check for the current demo, and do not use it to judge whether the PCT global route is correct.

## 3. Build and asset check

```bash
cd /mnt/sage_data/workspace/PCT-SCAN-ROS2

./scripts/build_workspace.sh --packages-select pct_scan_bridge scan_planner
```

Check Building PCT A* without starting a ROS GUI:

```bash
./scripts/test_pct_building_astar.sh --force
```

A passing run should show:

```text
shape=(8, 285, 150) resolution=0.100 m
layers=[0, 1, 2, 3, 4, 5, 6]
height_gain=13.50 m
clearance: p05=0.20 m median=0.41 m below_0.20m=38/872
```

## 4. Launch and plan interactively

```bash
./scripts/launch_pct_scan_building.sh
```

The main RViz displays are:

- `Global Map`: the original Building PCD, in gray;
- `PCT Full Cost Tomogram`: the full PCT cost layers, colored over the paper's range;
- `PCT Global Path`: the orange cross-floor route published by PCT;
- `Robot Path`: the green path the Go2 has already executed;
- `PCT Start and Goal`: interactive start and goal markers.

How to operate it:

1. Select the `Interact` tool at the top of RViz.
2. Drag Start and Goal. For a cross-floor plan, change `x/y` and also move the goal along the blue Z axis onto the target floor.
3. Right-click Start or Goal and click `Plan route`.
4. The orange PCT route appears first. SCAN then publishes local trajectories segment by segment, and the Go2 moves along the route.

The default Start is `(5, 5, 0)` and the default Goal is `(-6, -1, 14)`. PCT snaps them to valid cells on layer 0 and layer 6. That pair is a stable cross-floor demo.

To run the default endpoints automatically:

```bash
./scripts/launch_pct_scan_building.sh plan_on_startup:=true
```

Headless acceptance command:

```bash
./scripts/launch_pct_scan_building.sh \
  use_rviz:=false \
  plan_on_startup:=true \
  publish_tomogram:=false \
  enable_interactive_markers:=false
```

## 5. How to tell the chain is working

The terminal should print these logs in order:

```text
Trajectory with 874 poses published on /pct_path
Reference path reduced from 874 poses to 94 trajectory waypoints
Reference path accepted
Received trajectory 1
Local reference window complete (...); continuing along the global path
Received trajectory 2
```

A rising `Received trajectory N` means SCAN is rolling replans. It is not playing the PCT polyline once. After the centerline bias was added, the headless check kept generating local trajectories through number 26. Body height rose from `z=0.40 m` to `z=4.90 m` and never entered `EMERGENCY_STOP`. The test then ended on its timer. The planner did not stop itself.

## 6. Why the key parameters are set this way

### Why PCT needs a centerline bias

The original PCT A* uses an 8-neighborhood and treats tomogram traversability as the cell cost. The original code also zeroes the extra cost when `step_cost_weight * cell_cost < 5`. With the default `step_cost_weight=0.2` and a traversable threshold of `20`, every ordinary traversable cell satisfies `0.2 * cell_cost <= 4`. A* then mostly minimizes path length, and when both sides are traversable it can walk along the stair edge.

The bridge now runs a 2D distance transform on each tomogram layer and adds a soft cost near non-traversable cells. It does not delete cells that were already traversable, so a narrow stair stays connected. Among several feasible routes, the one with more clearance becomes cheaper. Building defaults are:

- `centerline_bias_enabled=true`;
- `preferred_clearance=0.5 m`;
- `clearance_cost=20.0`;
- `astar_step_cost_weight=1.0`.

On the baseline route, A* cells closer than `0.2 m` to a non-traversable region dropped from `386/769` to `38/872`, and the median clearance rose from about `0.14 m` to `0.41 m`. The remaining low-clearance points are mostly on cross-floor gateways, which a floor change has to pass through. This is a soft "prefer the centerline" constraint. It does not force every point onto the geometric axis of the stair.

If an interactive endpoint still produces a route that hugs the edge, raise the target clearance a little:

```bash
./scripts/launch_pct_scan_building.sh preferred_clearance:=0.6
```

Do not start with a large value. A strong bias makes A* detour toward open areas. To turn this bridge feature off when reproducing an experiment, pass `centerline_bias_enabled:=false`.

### SCAN tracking parameters

- `reference_path_simplify_tolerance=0.1 m`: matches the tomogram resolution, removes grid-path jaggedness, and keeps the maximum deviation inside one voxel;
- `planning_horizon=0.8 m`: a short local window on stairs and corners;
- `max_vel=0.5 m/s`: smaller acceleration when the polyline changes direction, so the motion looks smoother;
- `controller_mode=open_loop`: executes the 3D B-spline directly and can change height; the 2D `closed_loop` mode tracks only `x/y/yaw` and is not suitable for a cross-floor demo;
- `use_sim_time=false`: there is no Gazebo `/clock`, so every node uses system time.

The Building pickle was written by NumPy 2 in the PCT virtual environment. The launch script puts that environment's `site-packages` at the front of `PYTHONPATH`, so loading system NumPy 1.x does not raise `ModuleNotFoundError: numpy._core`.

### Why the paper's cost map is blue–green–yellow–red

The Building figure in the paper has three different renderings, and their colors are not comparable. The original 3D model uses material colors. The point-cloud figure is colored by Z height. Only the PCT tomogram figure is colored by travel cost. This project publishes the pickle's full original `traversability` on `/pct_tomogram`, with values in `0--50`. The paper-style coloring does not use `0--50`. It uses the fixed `-20--60` intensity range and the Rainbow colormap from the author's RViz config. The extra margin maps a raw cost of `0` into the blue region, rather than the magenta endpoint of the Rainbow colormap.

Read the cost colors this way:

| Raw cost | RViz color trend | Meaning |
| ---: | --- | --- |
| near `0` | dark blue | flat, enough clearance, preferred |
| medium cost | cyan / green / yellow | near an edge, or worse slope, step, or clearance |
| near `50` | orange / red | inflated boundary or barrier, not traversable |

The display range and the search threshold are different:

- The point cloud is still published as `0 <= cost <= 50`, so the full warm boundary from the paper figure stays visible.
- RViz normalizes color over `-20--60` to match the author's blue–cyan–green–yellow–orange palette.
- A* still admits only cells with `cost <= 20`.
- `20 < cost <= 50` shows structure and risk boundaries. Those cells do not become traversable.

The bridge used to apply the A* threshold of `20` when publishing the cloud, which filtered out the yellow and red cost rings that stand out in the paper. Changing only the RViz color scale cannot restore those cells. The publisher and the planner are now separate. The current Building data publishes `95,501` visible tomogram cells, of which `48,054` have cost above `20` and `29,254` have the barrier cost `50`.

To capture a view close to figure (b3) in the paper, temporarily hide `Global Map`, the robot, and the trajectories in RViz, leave only `PCT Full Cost Tomogram`, and switch to a suitable top or oblique view.

## 7. When the view is hard to read

The dedicated `building.rviz` uses `0.10 m` tomogram cubes, matching the Building voxel resolution. If the full cost field looks crowded:

1. Temporarily hide `Global Map` and look only at the PCT layers and the route.
2. Lower `PCT Full Cost Tomogram/Alpha` to `0.5`.
3. Move the focus to the target floor with the middle mouse button, then zoom with the wheel.
4. For a wide shot, keep every layer. For a close-up of the robot, keep only the PCD, the global route, and Robot Path.

The project now has a complete video-demo chain: interactive cross-floor endpoints, a PCT global search, continuous SCAN trajectories, and a quadruped animation in RViz.
