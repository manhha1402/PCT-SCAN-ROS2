# Phase 1: Feed the PCT global path into SCAN

## 1. Goal of this phase

This phase only checks the navigation message chain. Gazebo is not started:

```text
RViz Start/Goal
       │ right-click Plan route
       ▼
PCT native 3D A* ──/pct_path──▶ SCAN local B-spline
                                      │ /planning/bspline
                                      ▼
                              closed-loop controller
                                      │ /quad_0/cmd_vel
                                      ▼
                               Go2 kinematic model
```

The check passes when PCT produces a multi-floor `nav_msgs/Path`, SCAN accepts the reference path and publishes `scan_planner_msgs/Bspline`, the controller outputs velocity, and the Go2 planar position changes.

## 2. Why SCAN is still needed after PCT

A PCT tomogram is not an ordinary 2D occupancy grid. It organizes traversability cost, ground height, and cross-floor gateways from height slices into a 3D search space. PCT's A* answers which floor to start on, which stair to take, and where to end. Its output is a global ground polyline.

SCAN works at another scale. Using the local point cloud and the robot state, it turns the forward piece of that polyline into a continuous, trackable B-spline that respects velocity and acceleration limits, and it replans when a local obstacle appears. The split is:

- PCT runs only the native 3D A* and keeps the global topology and floor choice.
- SCAN runs local B-spline optimization and closed-loop control.
- PCT's own GPMP optimizer stays off, so two optimizers do not smooth the path twice and move the endpoints or the stair height.

The A* mentioned in the README is the discrete search stage inside PCT. It is not a traditional 2D A* standing in for PCT.

## 3. Three interface contracts

### 3.1 Frames

RViz, the PCD, and SCAN all use `world`. The current multifloor tomogram uses PCT coordinates, which differ from the visible scene by a 180° rotation about Z:

```text
x_pct = -x_world
y_pct = -y_world
z_pct =  z_world
```

The conversion happens only at the boundary of the PCT ROS node. Interactive markers and the published `/pct_path` stay in `world`, so SCAN does not need to know PCT's internal frame.

### 3.2 Height

`z` on the PCT path is the supporting-surface / ground height. When SCAN receives the path it adds `grid_map.body_height=0.4` once, which gives the body-center height. Do not add it again in the PCT node, or the robot hangs about one body height above the ground.

### 3.3 QoS

Both ends of `/pct_path` use `reliable + transient_local + depth 1`. SCAN can still receive the latest global path if it starts after PCT. A default volatile subscription does not guarantee that.

## 4. Build

After the dependencies are ready, build PCT, the bridge, and SCAN from this repository:

```bash
# from the PCT-SCAN-ROS2 repository root
./scripts/setup_pct_dependencies.sh
./scripts/build_workspace.sh
```

## 5. Launch and interaction

The script sources ROS and this repository's overlay in the right order, and it adds the dynamic-library path for the PCT native libraries under `.deps`:

```bash
./scripts/launch_pct_scan_phase1.sh
```

This early Multifloor example reads assets only from `assets/phase1` in this repository. It does not look at an external `pct_scan` folder. The current main path uses Building, launched with `./scripts/launch_pct_scan_building.sh`.

After RViz opens:

1. Confirm that Fixed Frame is `world`.
2. Select `Interact` in the toolbar.
3. Drag the green Start and the red Goal. Drag the Z axis to pick another floor.
4. When you release a marker, it snaps to a nearby traversable tomogram cell.
5. Right-click either marker and click `Plan route`.

Planning does not run while you drag. That is intentional: multi-floor A* should not rerun on every mouse update, and an explicit Plan makes the demo's action clearer.

`PCT Traversable Tomogram` in RViz is the main reference for choosing start and goal. Multifloor mode does not show all 26 slices. It shows only the main connected components of F1 layer 8 and F2 layer 15 that contain the verified route anchors, and it keeps cells within 0.4 m of each floor height. Marker snapping uses the same mask, so markers do not land on isolated noise islands. Blue means a lower traversal cost. Warm colors mean a higher cost. The light-gray `Global Map` only outlines the building. `Sensor Cloud`, `Occupancy`, `Inflated Occupancy`, and `Sliding Map Bounds` are off by default. Turn them on only when you are debugging SCAN's local sensing.

The default multi-floor example can also be tested without a GUI:

```bash
./scripts/launch_pct_scan_phase1.sh use_rviz:=false plan_on_startup:=true
```

## 6. How to tell that it worked

The terminal should print logs like these, in order:

```text
Trajectory with 100 poses published on /pct_path
Reference path accepted
[FSM]: from WAIT_TARGET to GEN_NEW_TRAJ
Received trajectory 1, duration ...
```

In another terminal, inspect the interfaces:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /pct_path --once
ros2 topic echo /planning/bspline --once
ros2 topic hz /quad_0/cmd_vel
```

## 7. Known limits of this phase

The current Go2 kinematic model integrates only planar `vx/vy/yaw`. Collision and terrain do not solve the body `z`. It can show that the PCT → SCAN → controller loop is closed, and it can move on the ground floor, but a multi-floor demo becomes inconsistent in height once the robot reaches the stairs. Phase 2 will add a kinematic update of body height along the SCAN trajectory / supporting surface, then a scripted gait. That does not need Gazebo physics.

## 8. How to describe it

"I did not treat PCT and SCAN as two competing planners. PCT solves the global 3D topology on the tomogram, especially the cross-floor gateways. SCAN consumes PCT's ground reference and builds a dynamically feasible B-spline from the local point cloud. The easy engineering mistakes are the frame transform, applying the height offset twice, and ROS 2 QoS, so I made those explicit interface contracts and checked them step by step with a layered smoke test of `/pct_path → bspline → cmd_vel`."

## 9. Where to start reading

- PCT interaction and publishing: `pct_planner/pct_planner/planner_node.py`
- PCT native A* path conversion: `pct_planner/pct_planner/planner_wrapper.py`
- SCAN receiving the reference path: `src/planner/plan_manage/src/scan_replan_fsm.cpp`
- Combined launch: `src/planner/plan_manage/launch/pct_scan_phase1.launch.py`
- Closed-loop control: `src/planner/plan_manage/src/closed_loop_controller.cpp`
- Kinematic model: `src/planner/plan_manage/src/go2_kinematic_sim.cpp`
