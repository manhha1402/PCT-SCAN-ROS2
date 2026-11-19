# Phase 5: Make SCAN follow the PCT route up the stairs

## 1. This is not a single controller-tuning problem

In RViz, the orange `/pct_path` is PCT's global route and the green `/quad_0/path` is the robot's actual trajectory. In the old implementation the green trajectory cut away from the orange route, and it could stop after the first local segment. Three separate problems were involved:

1. A global minimum-snap fit of the whole multi-floor route overshot badly.
2. The state machine treated "the local B-spline ended" as "global navigation ended".
3. The planar closed-loop simulation updates only x, y, and yaw, so it cannot change z.

## 2. The global reference holds the corridor; the local trajectory does the smoothing

The old implementation fit the whole multi-floor route once. On a sharp bend, or where floors connect closely, the fitted curve could leave the original polyline. The robot looked as if it were crossing a wall or walking around the stairs. The local planner had already received a global reference that had left the PCT corridor.

The current implementation splits those jobs:

```text
PCT polyline ──RDP(0.10 m)──> 3D keypoints
                              │
                              ├─ piecewise-linear global reference: no shortcuts
                              │
                              └─ local window: SCAN B-spline smoothing and avoidance
```

Piecewise linear does not mean the robot moves in a polyline. It only defines the centerline of the global navigation corridor. What is actually published to the executor is still the cubic B-spline optimized by SCAN. Building uses a short `0.8 m` window by default, which reduces local shortcuts on stairs and corners.

## 3. Finishing a local segment is not finishing the global route

Each SCAN B-spline covers about one planning horizon. The state machine now checks `last_progress_time_` on the global reference:

- Not yet at the global end: switch from `EXEC_TRAJ` to `REPLAN_TRAJ` and generate the next window.
- At the global end: clear the goal and enter `WAIT_TARGET`.

A headless run keeps receiving increasing trajectory numbers. If the number stays at 1, the state machine usually did not leave the local-complete state and enter `REPLAN_TRAJ`.

## 4. Why the multi-floor demo uses open-loop

`open_loop` here is a 3D trajectory executor for the demo. It samples `planning/bspline` directly and publishes x, y, z, and a yaw taken from the velocity direction. `closed_loop` drives a simplified kinematic model with a planar `cmd_vel` and is only suitable for a 2D demo.

This does not mean a real quadruped should be controlled open-loop. On a real robot the chain should still be: SCAN desired motion → velocity or foot commands → MoE low-level policy → joint execution and state feedback. The current goal of this project is a video, so the paper-scene launcher uses 3D `open_loop` by default.

## 5. Check Building multi-floor tracking

Run the PCT A* test first:

```bash
./scripts/test_pct_building_astar.sh --force
```

After it passes, play the default route automatically:

```bash
./scripts/launch_pct_scan_building.sh plan_on_startup:=true
```

Watch the body height in another terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /quad_0/body_pose --field pose.pose.position.z
```

The log should keep printing an increasing `Received trajectory N`, rather than entering `WAIT_TARGET` after trajectory 1. In RViz, orange is the PCT global route and green is the robot's history. After the robot enters the stairs, z of `/quad_0/body_pose` should follow the local trajectory.
