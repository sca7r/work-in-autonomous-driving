# Trajectory Planner (`adapt_trajp`)

## Overview

The **Trajectory Planner** sits between the Route Computer and the Control module in the ADAPT (Autonomous Driving And Parking Technology) pipeline. It transforms the raw discrete waypoints produced by the Route Computer into smooth, physically drivable trajectories, and computes the precise geometric path for the final parking manoeuvre.

The package provides three distinct trajectory outputs, each triggered by a separate Boolean control signal from Behaviour Planning:

- **Drive trajectory** (`/drive` → `true`) — cubic spline interpolation over the route waypoints, producing 50 densely-spaced points at constant velocity 1.0 m/s for general navigation.
- **Stop trajectory** (`/drive` → `false`) — a single zero-velocity point to halt the vehicle.
- **Forward parking trajectory** (`/park` → `true`) — geometric arc-and-line path covering the first three phases of the parking manoeuvre (approach and forward arc).
- **Reverse parking trajectory** (`/park_reverse` → `true`) — geometric arc-and-line path for the final two phases (reverse arc into the spot and straight alignment).

All trajectories are published as `trajectory_msgs/JointTrajectory` messages carrying `[x, y, velocity]` per point, and are simultaneously published as `nav_msgs/Path` messages on visualisation topics for RViz.

The package also ships two standalone development utilities  `dummy_path.py` (publishes a hardcoded test route) and `dummy_state.py` (publishes configurable Boolean state signals), enabling the trajectory planner to be tested in isolation without the full ADAPT stack running.

---

## Table of Contents

- [Architecture](#architecture)
- [ROS 2 Interface](#ros-2-interface)
- [Trajectory Modes](#trajectory-modes)
  - [Drive Trajectory — Cubic Spline Interpolation](#drive-trajectory--cubic-spline-interpolation)
  - [Stop Trajectory](#stop-trajectory)
  - [Parking Trajectory — Geometric Path](#parking-trajectory--geometric-path)
- [Parking Manoeuvre Geometry](#parking-manoeuvre-geometry)
  - [Key Points](#key-points)
  - [Five Phases](#five-phases)
  - [Arc and Line Generation](#arc-and-line-generation)
- [Development Utilities](#development-utilities)
  - [dummy\_path.py](#dummy_pathpy)
  - [dummy\_state.py](#dummy_statepy)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Running the Node](#running-the-node)
- [Testing](#testing)
- [Integration with ADAPT](#integration-with-adapt)

---

## Architecture

```
/route          (geometry_msgs/PoseArray)  ──────────────────────────┐
  from Route Computer                                                 │
                                                                      │
/selected_spot  (geometry_msgs/PoseStamped) ─────────────────────────┤
  from Behaviour Planning                                             │
                                                                      ▼
/loc_pose       (geometry_msgs/PoseStamped) ──────────▶   TrajectoryPlanner
  from Localisation                                                   │
                                                                      │
/euler_angles   (geometry_msgs/Vector3)    ───────────┘    Triggered by:
  from Localisation                                                   │
                                                    ┌─────────────────┼──────────────────────┐
                                                    │                 │                      │
                                              /drive=true       /park=true          /park_reverse=true
                                                    │                 │                      │
                                                    ▼                 ▼                      ▼
                                              /trajd             /trajpf               /trajpr
                                         (JointTrajectory)  (JointTrajectory)    (JointTrajectory)
                                         + /visualize        + /visualize_        + /visualize_
                                         (Path)               forward_park         reverse_park
                                                              (Path)               (Path)
                                                    │
                                              /drive=false
                                                    │
                                                    ▼
                                              /trajs
                                         (JointTrajectory)
                                         zero-velocity stop
```

---

## ROS 2 Interface

### Subscriptions

| Topic | Message Type | Description |
|---|---|---|
| `/route` | `geometry_msgs/PoseArray` | Ordered waypoint sequence from the Route Computer. Stored as `current_path`. Minimum 3 poses required for spline interpolation. |
| `/drive` | `std_msgs/Bool` | `true` triggers smooth drive trajectory generation and publishes to `/trajd`. `false` triggers stop and publishes to `/trajs`. |
| `/park` | `std_msgs/Bool` | `true` triggers forward parking trajectory generation. Publishes to `/trajpf`. |
| `/park_reverse` | `std_msgs/Bool` | `true` triggers reverse parking trajectory generation. Publishes to `/trajpr`. |
| `/euler_angles` | `geometry_msgs/Vector3` | Euler angles from Localisation. The Z component (yaw) is stored as `current_yaw`. |
| `/loc_pose` | `geometry_msgs/PoseStamped` | Full 6DOF pose of the ego vehicle from Localisation. Stored as `current_loc`. |
| `/selected_spot` | `geometry_msgs/PoseStamped` | Target parking spot coordinates from Behaviour Planning. X and Y stored as `selected_spot` tuple. |

### Publications

| Topic | Message Type | Triggered by | Description |
|---|---|---|---|
| `/trajd` | `trajectory_msgs/JointTrajectory` | `/drive=true` | 50-point cubic spline smoothed drive trajectory. Joint names: `['x', 'y', 'velocity']`. Velocity: 1.0 m/s constant. |
| `/trajs` | `trajectory_msgs/JointTrajectory` | `/drive=false` | Single-point stop trajectory. Position `[0.0, 0.0]`, velocity `[0.0, 0.0]`. |
| `/trajpf` | `trajectory_msgs/JointTrajectory` | `/park=true` | Forward parking trajectory (phases 1–3): straight approach + forward arc + straight to cross-point. |
| `/trajpr` | `trajectory_msgs/JointTrajectory` | `/park_reverse=true` | Reverse parking trajectory (phases 4–5): reverse arc into spot + reverse straight alignment. |
| `/visualize` | `nav_msgs/Path` | `/drive=true` | RViz visualisation of the drive trajectory in the `map` frame. |
| `/visualize_forward_park` | `nav_msgs/Path` | `/park=true` | RViz visualisation of the forward parking trajectory. |
| `/visualize_reverse_park` | `nav_msgs/Path` | `/park_reverse=true` | RViz visualisation of the reverse parking trajectory. |

> **One-shot publishing:** Each trajectory type is published only once per session. Gate flags (`sent_drive_message`, `sent_park_for_message`, `sent_park_rev_message`) prevent re-publication on repeated Boolean triggers.

---

## Trajectory Modes

### Drive Trajectory — Cubic Spline Interpolation

**Triggered by:** `/drive` = `true`
**Published to:** `/trajd`, `/visualize`

Raw routes from the Route Computer are sequences of discrete graph nodes that would produce jerky motion if followed directly. The `smooth_trajectory` method applies **cubic spline interpolation** using `scipy.interpolate.CubicSpline`:

1. Extract (x, y) arrays from the `PoseArray` waypoints.
2. Compute cumulative arc-length distance along the route as the parametric variable.
3. Fit independent cubic splines `cs_x(t)` and `cs_y(t)` over the arc-length parameter.
4. Resample to **50 evenly-spaced points** along the full arc length using `np.linspace`.
5. Assign a constant velocity of **1.0 m/s** to all points.
6. Pack into a `JointTrajectory` with joint names `['x', 'y', 'velocity']`.

**Minimum waypoints:** 3. If fewer than 3 poses are in `/route`, a warning is logged and no trajectory is published.

### Stop Trajectory

**Triggered by:** `/drive` = `false`
**Published to:** `/trajs`

The `stop_car` method publishes a single `JointTrajectoryPoint` with `positions = [0.0, 0.0]` and `velocities = [0.0, 0.0]`. The Control module interprets this as an immediate stop command.

### Parking Trajectory — Geometric Path

**Triggered by:** `/park` = `true` (forward) and `/park_reverse` = `true` (reverse)
**Published to:** `/trajpf`, `/visualize_forward_park` and `/trajpr`, `/visualize_reverse_park`

The parking trajectory is computed by `generate_parking_trajectory` using geometric construction (described fully in the next section), then split into forward and reverse halves. Each phase uses either `generate_straight_line` (10 uniformly-spaced points) or `generate_circular_arc` (10 arc points). Velocity is `+1.0 m/s` for forward phases and `−1.0 m/s` for reverse phases, the sign is used by the Control module to determine direction.

---

## Parking Manoeuvre Geometry

The parking manoeuvre uses two circular arcs of equal radius and three straight line segments to guide the vehicle from the end of its driving route into the target parking spot. The design is based on tangent-circle geometry, where both circles share a common tangent line drawn at angle `θ = 30°`.

### Key Points

| Point | Description |
|---|---|
| `initp` | Starting point — last waypoint of the driving route (end of `/route`). |
| `interp` | Intermediate point — same X as `initp`, Y aligned with the parking spot. |
| `interp1` | Tangent point 1 — offset from `interp` downward by `δ = r·tan(θ/2)`. Entry point of arc 1. |
| `interp2` | Tangent point 2 — offset from `interp` along the 60° line by `δ`. Exit point of arc 1 / entry of straight segment. |
| `crossp` | Cross-point — further along the 60° line from `interp` by `Δs = r·tan((π/2 − θ)/2)`. Entry point of arc 2. |
| `destp` | Destination point — horizontally offset from the map reference X by `Δs`, at parking spot Y. Exit point of arc 2. |
| `goalp` | Goal point — exact parking spot coordinates from `/selected_spot`. |

**Geometry constants (hardcoded):**

| Parameter | Value | Description |
|---|---|---|
| `θ` (theta) | 30° (π/6 rad) | Tangent line angle, controls sharpness of the manoeuvre. |
| `r` (radius) | 1.2 m | Radius of both turning circles. |
| `δ` (delta_theta) | `r·tan(θ/2)` ≈ 0.321 m | Offset from `interp` to `interp1`/`interp2`. |
| `Δs` (delta_s) | `r·tan((π/2−θ)/2)` ≈ 1.073 m | Arc entry distance for the reverse circle. |

### Five Phases

```
initp ──────────── straight ──────────▶ interp1
                                              │
                                          arc (r=1.2m, fwd)
                                              │
                                              ▼
interp2 ◀────────────────────────────── interp2
    │
    straight (along 60° tangent line)
    │
    ▼
crossp
    │
arc (r=1.2m, reverse)
    │
    ▼
destp ──────────── straight (reverse) ─▶ goalp
```

| Phase | Type | From | To | Velocity |
|---|---|---|---|---|
| 1 | Straight line | `initp` | `interp1` | +1.0 m/s |
| 2 | Circular arc | `interp1` | `interp2` | +1.0 m/s |
| 3 | Straight line | `interp2` | `crossp` | +1.0 m/s |
| 4 | Circular arc | `crossp` | `destp` | −1.0 m/s |
| 5 | Straight line | `destp` | `goalp` | −1.0 m/s |

Phases 1–3 are combined into `trajectory_for` (published on `/trajpf`).
Phases 4–5 are combined into `trajectory_rev` (published on `/trajpr`).

### Arc and Line Generation

**Straight lines** (`generate_straight_line`): 10 uniformly-spaced points via `np.linspace` between start and end. Velocity is uniform across all points.

**Circular arcs** (`generate_circular_arc`): The centre of the arc is computed geometrically from the perpendicular bisector of the chord connecting start and end, at the given `radius`. The method:
1. Finds the midpoint of the chord.
2. Computes the perpendicular unit vector.
3. Offsets from the midpoint by `sqrt(r² − |chord/2|²)` to find two candidate centres.
4. Selects the centre producing the smaller angular sweep.
5. Samples 10 points uniformly between the start and end angles.

**`calculate_point_on_line`**: Given a reference point, a slope, and a distance, computes the (x, y) coordinates of a point that lies on the line at that distance, used to place `interp2` and `crossp` on the 60° tangent.

---

## Development Utilities

### `dummy_path.py`

**Executable:** `dum_path`
**Node name:** `path_publisher`

Publishes a hardcoded 14-point `PoseArray` on `/route` once at startup. The path represents a typical Model City route from (0.5, 0.7) to (3.5, 4.5), travelling first along the X axis then up the Y axis, matching the structure of the real Route Computer output.

```bash
ros2 run adapt_trajp dum_path
```

Use this to test the trajectory planner's drive trajectory mode without running the Route Computer or the rest of the ADAPT stack.

### `dummy_state.py`

**Executable:** `dum_state`
**Node name:** `dummy_state_publisher`

A configurable Boolean publisher that sends a `std_msgs/Bool` at 1 Hz to any topic. Topic name and state are passed as command-line arguments. Used to simulate Behaviour Planning's control signals during isolated testing.

```bash
# Trigger drive trajectory
ros2 run adapt_trajp dum_state --topic /drive --state true

# Trigger stop
ros2 run adapt_trajp dum_state --topic /drive --state false

# Trigger forward parking
ros2 run adapt_trajp dum_state --topic /park --state true

# Trigger reverse parking
ros2 run adapt_trajp dum_state --topic /park_reverse --state true
```

---

## Package Structure

```
trajectory/
├── adapt_trajp/
│   ├── __init__.py
│   ├── trajectory_planner.py    # Main TrajectoryPlanner node
│   ├── dummy_path.py            # Development utility: hardcoded route publisher
│   └── dummy_state.py           # Development utility: configurable Bool publisher
├── images/
│   ├── diagram.png              # Parking manoeuvre geometry diagram
│   └── traj_plan_bd.png         # Trajectory planner block diagram
├── resource/
│   └── adapt_trajp              # Ament resource index marker
├── test/
│   ├── trajectory_planner_test.py   # Integration test
│   ├── test_copyright.py            # Ament copyright linting
│   ├── test_flake8.py               # Flake8 style checks
│   └── test_pep257.py               # PEP 257 docstring checks
├── package.xml
├── setup.cfg
└── setup.py
```

---

## Dependencies

### ROS 2

| Dependency | Purpose |
|---|---|
| `rclpy` | ROS 2 Python client library |
| `geometry_msgs` | `PoseStamped`, `PoseArray`, `Vector3` |
| `nav_msgs` | `Path` — used for RViz visualisation topics |
| `std_msgs` | `Bool` — drive and park control signals |
| `trajectory_msgs` | `JointTrajectory`, `JointTrajectoryPoint` — primary output format |
| `ros2launch` | Launch system support |

### Python

| Package | Purpose |
|---|---|
| `numpy` | Array operations, linspace, cumsum, arctan2, linalg |
| `scipy` (`CubicSpline`) | Cubic spline fitting for drive trajectory smoothing |
| `math` | `radians`, `tan`, `sqrt`, `pi` for parking geometry |

### Build System

- **ROS 2 Foxy** (or compatible)
- **ament_python** build type

---

## Installation

1. Place the package in your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   # copy or clone adapt_trajp here
   ```

2. Install Python dependencies:

   ```bash
   pip install scipy numpy --break-system-packages
   ```

3. Install ROS dependencies:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build:

   ```bash
   colcon build --packages-select adapt_trajp
   source install/setup.bash
   ```

---

## Running the Node

### Start the trajectory planner

```bash
ros2 run adapt_trajp traj
```

### Full isolated test using development utilities

Open three terminals (with the workspace sourced in each):

```bash
# Terminal 1: Start the trajectory planner
ros2 run adapt_trajp traj

# Terminal 2: Publish a test route
ros2 run adapt_trajp dum_path

# Terminal 3: Trigger drive trajectory
ros2 run adapt_trajp dum_state --topic /drive --state true
```

To test parking, also publish a `/selected_spot` and trigger the parking states:

```bash
# Publish a mock selected spot (adjust coordinates to match the test route)
ros2 topic pub /selected_spot geometry_msgs/PoseStamped \
  "{pose: {position: {x: 4.75, y: 5.25, z: 0.0}}}" --once

# Trigger forward parking
ros2 run adapt_trajp dum_state --topic /park --state true

# Trigger reverse parking (after forward completes)
ros2 run adapt_trajp dum_state --topic /park_reverse --state true
```

### Verifying output

```bash
# Watch the drive trajectory
ros2 topic echo /trajd

# Watch the forward parking trajectory
ros2 topic echo /trajpf

# Watch the reverse parking trajectory
ros2 topic echo /trajpr
```

Open **RViz** and add `Path` displays on `/visualize`, `/visualize_forward_park`, and `/visualize_reverse_park` (frame: `map`) to see all three trajectories rendered.

---

## Testing

### Integration Test (`trajectory_planner_test.py`)

Uses `pytest` with `SingleThreadedExecutor`. The test creates a `TrajectoryPlanner` node and an `IntegrationHelperNode` (subscribes to `/route`), then injects messages by calling callbacks directly and spinning the executor.

The test exercises all six callback methods that were present at the time of writing:

| Callback invoked | Input injected | Purpose |
|---|---|---|
| `path_callback` | 3-pose `PoseArray` (0,0)→(1,1)→(2,2) | Populates `current_path` |
| `drive_callback` | `JointTrajectory` with one point at (1,1) | Triggers drive trajectory |
| `stop_drive_callback` | `JointTrajectory` at (2,2) | Triggers stop |
| `stop_park_callback` | `JointTrajectory` at (2,2) | Triggers park stop |
| `park_forward_callback` | `JointTrajectory` at (2,2) | Triggers forward parking |
| `park_reverse_callback` | `JointTrajectory` at (2,2) | Triggers reverse parking |
| `emergency_stop_callback` | `JointTrajectory` at (2,2) | Triggers emergency stop |

**Assertions:** The helper node checks that at least one `PoseArray` is received on `/route` with non-zero poses.

> **Note:** The test calls several callback methods (`stop_drive_callback`, `stop_park_callback`, `emergency_stop_callback`) that do not exist in the current `trajectory_planner.py`, these would raise `AttributeError` at runtime. See [Known Issues](#known-issues--notes).

### Linting Tests

| Test | Tool |
|---|---|
| `test_copyright.py` | ament_copyright |
| `test_flake8.py` | flake8 |
| `test_pep257.py` | pep257 |

### Running all tests

```bash
cd ~/ros2_ws
colcon test --packages-select adapt_trajp
colcon test-result --verbose
```

---


## Integration with ADAPT

| Component | Relationship |
|---|---|
| **Route Computer (`adapt_roucomp`)** | Upstream provider of `/route`. Publishes the raw `PoseArray` of waypoints that the trajectory planner smooths with cubic spline interpolation. |
| **Localisation (`adapt_loc`)** | Provides `/loc_pose` (vehicle position) and `/euler_angles` (yaw), stored for use in parking geometry calculations. |
| **Behaviour Planning** | Orchestrates all trajectory modes by publishing Boolean signals to `/drive`, `/park`, and `/park_reverse` based on FSM state transitions. Also provides `/selected_spot`. |
| **Control** | Downstream consumer of `/trajd`, `/trajpf`, `/trajpr`, and `/trajs`. Interprets the `JointTrajectory` messages to command the vehicle's lateral and longitudinal actuators. Negative velocities in the parking trajectory signal reverse motion. |


---


## License

Apache License 2.0 — see [LICENSE](LICENSE) for details.