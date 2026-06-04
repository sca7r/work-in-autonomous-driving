# Route Computer (`adapt_roucomp`)

## Overview

The **Route Computer** is the path planning component of the ADAPT (Autonomous Driving And Parking Technology) system. Given the ego vehicle's current position and its target parking spot, it computes an optimal, directionally-aware route through the Model City map using the **A\* (A-star) algorithm**, and publishes the result as a sequence of waypoints for downstream execution.

The package contains two node implementations that evolved over development:

- **`RouteComputer`** (`routemodule5.py`) — the original node, using a direction-agnostic A\* over a simple node/edge graph.
- **`RouteComputerCardinal`** (`route_cardinal.py`) — the current production node, extending A\* with **cardinal direction constraints** (N/S/E/W), **turning penalties**, vehicle yaw awareness, and **performance metrics logging**.

Both nodes subscribe to the same topics and publish to the same output topics, but the cardinal node is the recommended implementation.

---

## Table of Contents

- [Architecture](#architecture)
- [ROS 2 Interface](#ros-2-interface)
- [Nodes](#nodes)
  - [RouteComputerCardinal (route\_cardinal.py)](#routecomputercardinal-route_cardinalpy)
  - [RouteComputer (routemodule5.py)](#routecomputer-routemodule5py)
- [A\* Algorithm](#a-algorithm)
  - [Direction-Aware Pathfinding](#direction-aware-pathfinding)
  - [Turning Penalties](#turning-penalties)
  - [Heuristic](#heuristic)
  - [Performance Metrics Logging](#performance-metrics-logging)
- [Map Format](#map-format)
  - [map\_cardinal.txt](#map_cardinaltxt)
  - [map.txt](#maptxt)
  - [Map Visualisation](#map-visualisation)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Running the Node](#running-the-node)
- [Parameters & Configuration](#parameters--configuration)
- [Testing](#testing)
- [Rosbag Data](#rosbag-data)
- [Integration with ADAPT](#integration-with-adapt)

---

## Architecture

```
/selected_spot  (geometry_msgs/PoseStamped)  ─────────────────┐
  from Behaviour Planning / Spot Selector                      │
                                                               ▼
/loc_pose       (geometry_msgs/PoseStamped)  ──────▶  RouteComputerCardinal
  from Localisation                                            │
                                                               │  A* over directed
/euler_angles   (geometry_msgs/Vector3)      ──────┘  graph with cardinal
  from Localisation (yaw only)                                 │  direction constraints
                                                               │
                                             ┌─────────────────┴──────────────────┐
                                             ▼                                    ▼
                                       /route                             /route_state
                               (geometry_msgs/PoseArray)              (std_msgs/Bool)
                               Ordered waypoint sequence              true = route ready
                               frame: map                             to Behaviour Planning
                                             │
                                             ▼
                                    Behaviour Planning /
                                    Trajectory Planner
```

On every timer tick (10 Hz), if vehicle location, target coordinate, and vehicle orientation are all available and a route has not yet been sent, the node snaps both positions to their nearest graph nodes, determines the vehicle's initial cardinal direction from its yaw, runs A\*, and publishes the result.

---

## ROS 2 Interface

### Subscriptions

| Topic | Message Type | Description |
|---|---|---|
| `/selected_spot` | `geometry_msgs/PoseStamped` | Target parking spot coordinates from Behaviour Planning / Spot Selector. The node applies a fixed offset of (−1.0 m, −1.5 m) to align the target with the nearest navigable graph node. |
| `/loc_pose` | `geometry_msgs/PoseStamped` | Full 6DOF pose of the ego vehicle from the Localisation component. Only X and Y position are used. |
| `/euler_angles` | `geometry_msgs/Vector3` | Euler angles from Localisation. Only the Z component (yaw) is used to determine the vehicle's initial cardinal heading. |

### Publications

| Topic | Message Type | QoS | Description |
|---|---|---|---|
| `/route` | `geometry_msgs/PoseArray` | Default (depth 10) | Ordered sequence of (x, y) waypoint poses in the `map` frame. Published once when a valid path is found. |
| `/route_state` | `std_msgs/Bool` | Default (depth 10) | `true` immediately after a route is successfully published; used by Behaviour Planning to trigger the next FSM state transition. |

> **Note:** The route is published **once** per planning request. `send_route` is set to `True` after publishing to prevent re-computation on subsequent timer ticks.

---

## Nodes

### RouteComputerCardinal (`route_cardinal.py`)

**Node name:** `route_computer_cardinal`
**Executable:** `route_cardinal`

The current production implementation. Extends basic A\* with:

- **Cardinal direction awareness** — each edge in the graph carries a direction label (`N`, `S`, `E`, `W`). The vehicle's initial heading is derived from the yaw angle received on `/euler_angles`.
- **Turning penalties** — costs are added when the path requires a turn, discouraging unnecessary direction changes.
- **Metrics logging** — every successful path computation writes a row to `route_planning_metrics.csv` in the working directory.
- **Structured map loading** — the map parser (`load_map`) is split into `process_node_line`, `process_edge_line`, and `process_parking_spot_line`, each handling comma-separated fields.

**Key internal state:**

| Variable | Type | Description |
|---|---|---|
| `nodes` | `list[(x, y, id)]` | All graph nodes loaded from map. |
| `edges` | `dict{node_id: [(neighbor_id, direction)]}` | Directed adjacency list with cardinal labels. |
| `parking_spots` | `list[dict]` | Parking spot coordinates with their closest assigned node. |
| `vehicle_location` | `tuple(x, y)` | Updated by `/loc_pose` callback. |
| `target_coordinate` | `tuple(x, y)` | Updated by `/selected_spot` callback (with offset applied). |
| `vehicle_orientation` | `float` | Yaw in radians, updated by `/euler_angles` callback. |
| `send_route` | `bool` | Gate flag; prevents re-running A\* once a route is published. |

### RouteComputer (`routemodule5.py`)

**Node name:** `route_computer`
**Executable:** `route`

The original implementation. Uses the same A\* algorithm but without cardinal direction constraints or turning penalties. Edges are stored as plain integer neighbor lists (no direction labels). It does not subscribe to `/euler_angles` and therefore has no yaw awareness.

It loads `map.txt` (space-delimited) instead of `map_cardinal.txt` (comma-delimited). The launch file currently starts this node via the `route` executable.

---

## A\* Algorithm

Both nodes implement A\* (`heapq`-based priority queue) over a preloaded directed graph. The cardinal node adds direction-awareness on top.

### Direction-Aware Pathfinding

At each expansion step, the current travel direction is tracked. When a neighbour is evaluated, the direction of the edge leading to it is compared with the current direction. The vehicle's initial direction is seeded from its yaw angle via `yaw_to_direction`:

| Yaw range | Cardinal direction |
|---|---|
| −π/4 to +π/4 | East (`E`) |
| +π/4 to +3π/4 | North (`N`) |
| −3π/4 to −π/4 | South (`S`) |
| ±3π/4 to ±π | West (`W`) |

### Turning Penalties

The cost function is:

```
tentative_g = g_score[current] + euclidean_distance(current, neighbour) + direction_penalty
```

Penalties applied per turn type:

| Turn type | Penalty |
|---|---|
| Straight (same direction) | 0 |
| Right-angle turn (90°) | 5 |
| U-turn (180°) | 10 |

This discourages unnecessary turns while still allowing them when the geometry requires it.

### Heuristic

Both nodes use **Euclidean distance** to the goal as the admissible heuristic:

```
h(n) = sqrt((n.x − goal.x)² + (n.y − goal.y)²)
```

### Performance Metrics Logging

`RouteComputerCardinal` logs the following fields to `route_planning_metrics.csv` on every successful path computation:

| Column | Description |
|---|---|
| `timestamp` | ISO 8601 datetime of the planning call. |
| `execution_time` | Wall-clock time taken by A\* in seconds. |
| `path_length` | Total Euclidean length of the found path in metres. |
| `explored_nodes` | Number of unique nodes expanded during search. |
| `iterations` | Total loop iterations of the A\* main loop. |
| `path_cost` | Final `g_score` of the goal node (distance + penalties). |

---

## Map Format

The Model City environment is represented as a directed graph stored in plain text files. Both map files define the same 33-node, 75-edge graph and 4 parking spots — they differ only in format and the addition of cardinal direction labels.

### `map_cardinal.txt`

Used by `RouteComputerCardinal`. Comma-delimited. Edges carry explicit cardinal direction labels.

**Structure:**
```
NODES
x, y, node_id
...

EDGES
from_node_id, to_node_id, direction
...

PARKING_SPOTS
x, y
...
```

**Example entries:**
```
NODES
0.5, 0.7, 1
1.5, 0.7, 2

EDGES
1, 2, W        ← node 1 → node 2, travelling West
2, 1, E        ← node 2 → node 1, travelling East

PARKING_SPOTS
4.75, 5.25
```

### `map.txt`

Used by `RouteComputer`. Space-delimited. Edges are plain integer neighbor lists with no direction labels.

**Structure:**
```
NODES
x y node_id

EDGES
node_id neighbor1 neighbor2 ...

PARKING_SPOTS
x y
```

### Map Statistics

| Property | Value |
|---|---|
| Total nodes | 33 |
| Total directed edges | 75 (cardinal map) |
| Parking spots | 4 (at x=4.75, y=5.25/5.75/6.25/6.75) |
| Map coordinate space | Metres, origin at (0, 0) |
| Approximate extent | 7.5 m × 7.5 m (Model City dimensions) |

### Map Visualisation

`config/map_plot.py` is a standalone utility script (requires `networkx` and `matplotlib`) that renders the directed graph with colour-coded edges per cardinal direction:

| Edge colour | Direction |
|---|---|
| Red | East |
| Blue | West |
| Green | North |
| Purple | South |

Run with:
```bash
python3 config/map_plot.py
```

> Update the `filename` path inside `map_plot.py` to point to your local `map_cardinal.txt` before running.

---

## Package Structure

```
route_computer/
├── adapt_roucomp/
│   ├── __init__.py
│   ├── route_cardinal.py       # RouteComputerCardinal node (current, cardinal-aware A*)
│   └── routemodule5.py         # RouteComputer node (original, direction-agnostic A*)
├── config/
│   ├── map_cardinal.txt        # Directed graph map with cardinal edge labels (comma-delimited)
│   ├── map.txt                 # Directed graph map without direction labels (space-delimited)
│   └── map_plot.py             # Utility: visualise the map graph with networkx/matplotlib
├── images/                     # Documentation and development screenshots
│   ├── A_algo.png              # A* algorithm diagram
│   ├── Block Diagram.jpg       # System block diagram
│   ├── Figure_1.png            # Map plot figure
│   ├── Method_Call_Flow.png    # Method call flow diagram
│   ├── code_components.png     # Code component diagram
│   ├── diagram.png             # Architecture diagram
│   ├── map_cardinal.png        # Rendered cardinal direction map
│   ├── orientatio_E_1.png      # Vehicle orientation East example
│   ├── orientatio_w_1.png      # Vehicle orientation West example
│   ├── relative_path.png       # Relative path illustration
│   ├── route_rviz2.png         # Route visualised in RViz
│   ├── routecomputer_output.png# Route computer console output
│   ├── rqt_module6.png         # rqt_graph screenshot
│   ├── rviz.png                # Full RViz session screenshot
│   ├── south.png               # Vehicle heading South example
│   ├── updated_map.jpg         # Updated map image
│   └── west.png                # Vehicle heading West example
├── launch/
│   └── roucomp_launch.py       # Launches the RouteComputer (routemodule5) node
├── resource/
│   └── adapt_roucomp           # Ament resource index marker
├── ros2_bags/
│   ├── cross-l/                # Rosbag: left-crossing trajectory (~7.8 s, 235 msgs)
│   ├── cross-r/                # Rosbag: right-crossing trajectory (~9.2 s, 277 msgs)
│   ├── diagnal/                # Rosbag: diagonal trajectory (~12.8 s, 384 msgs)
│   ├── origin-1/               # Rosbag: origin start scenario 1 (~8.5 s, 256 msgs)
│   ├── origin-2/               # Rosbag: origin start scenario 2 (~13.3 s, 399 msgs)
│   └── reverse/                # Rosbag: reverse manoeuvre (~6.2 s, 188 msgs)
├── test/
│   ├── unit_test.py            # Unit tests for RouteComputer (routemodule5)
│   ├── unit_test_cardinal.py   # Unit tests for RouteComputerCardinal
│   ├── integration_test.py     # Integration test for RouteComputer
│   ├── integration_test_cardinal.py  # Integration test for RouteComputerCardinal
│   ├── test_copyright.py       # Ament copyright linting
│   ├── test_flake8.py          # Flake8 style checks
│   └── test_pep257.py          # PEP 257 docstring checks
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
| `geometry_msgs` | `PoseStamped`, `PoseArray`, `Vector3` message types |
| `std_msgs` | `Bool` message type for `/route_state` |
| `nav_msgs` | `OccupancyGrid` (imported in `routemodule5` but not actively used) |
| `ros2launch` | Launch system support |

### Python Standard Library

| Module | Purpose |
|---|---|
| `heapq` | Priority queue for A\* open set |
| `math` | Euclidean distance, yaw-to-direction conversion |
| `csv`, `datetime` | Metrics logging to `route_planning_metrics.csv` |
| `time` | Execution time measurement |
| `os` | Map file path resolution via `ament_index_python` |

### Build System

- **ROS 2 Foxy** (or compatible)
- **ament_python** build type

---

## Installation

1. Place the package in your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   # copy or clone adapt_roucomp here
   ```

2. Install dependencies:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build:

   ```bash
   colcon build --packages-select adapt_roucomp
   source install/setup.bash
   ```

---

## Running the Node

### Using the launch file

```bash
ros2 launch adapt_roucomp roucomp_launch.py
```

> **Note:** The launch file currently starts the `route` executable, which runs `RouteComputer` (routemodule5). To run the cardinal node instead, use the direct command below or update the launch file.

### Running the cardinal node directly (recommended)

```bash
ros2 run adapt_roucomp route_cardinal
```

### Running the original node directly

```bash
ros2 run adapt_roucomp route
```

### Verifying output

```bash
# Watch the computed route waypoints
ros2 topic echo /route

# Watch the route state signal
ros2 topic echo /route_state
```

### Replaying a rosbag for testing

```bash
ros2 bag play src/route_computer/ros2_bags/origin-1/
```

This replays `/pose_modelcars` MoCap data, which can be fed through the Localisation node to simulate the full pipeline.

---

## Parameters & Configuration

There are no ROS 2 runtime parameters declared. The following values are hardcoded and would need to be changed in source if required:

| Value | Location | Description |
|---|---|---|
| `map_cardinal.txt` | `route_cardinal.py` → `__init__` | Map file used by the cardinal node. Resolved via `ament_index_python`. |
| `map.txt` | `routemodule5.py` → `__init__` | Map file used by the original node. |
| `(−1.0, −1.5)` offset | Both nodes → `spot_location_callback` | Fixed coordinate offset applied to the received spot location to align it with the nearest navigable graph node. |
| `0.1 s` timer period | Both nodes → `__init__` | Route planning check runs at 10 Hz. |
| Penalty: 90° turn = 5 | `route_cardinal.py` → `direction_change_penalty` | Cost added for a right-angle direction change. |
| Penalty: 180° turn = 10 | `route_cardinal.py` → `direction_change_penalty` | Cost added for a U-turn. |
| `route_planning_metrics.csv` | `route_cardinal.py` → `initialize_metrics_logging` | Output file for performance metrics. Written to the working directory. |

---

## Testing

### Unit Tests — RouteComputer (`unit_test.py`)

Tests the original `RouteComputer` node using `unittest`.

| Test | Description |
|---|---|
| `test_initialization` | Verifies publisher and subscribers are created, and `publisher_count` starts at 0. |
| `test_a_star_pathfinding` | Calls `a_star` with synthetic start/goal nodes and asserts a non-None path is returned. |
| `test_publisher_count_increment` | Injects vehicle and spot location messages and spins the node to trigger route computation. |

### Unit Tests — RouteComputerCardinal (`unit_test_cardinal.py`)

Tests the cardinal node with direct method-level assertions.

| Test | Description |
|---|---|
| `test_yaw_to_direction` | Verifies cardinal direction assignment for yaw = 0 (E), π/2 (N), −π/2 (S), ±π (W). |
| `test_process_node_line` | Parses a CSV node line and asserts the tuple is added to `self.nodes`. |
| `test_process_edge_line` | Parses a CSV edge line and asserts the `(neighbor_id, direction)` tuple is added to `self.edges`. |
| `test_a_star_no_nodes` | Clears nodes and edges, calls A\*, asserts `None` is returned. |
| `test_a_star_path_found` | Sets a minimal 2-node graph and asserts A\* returns the correct index path `[0, 1]`. |

### Integration Test — RouteComputer (`integration_test.py`)

Runs `RouteComputer` and a helper subscriber node in a `SingleThreadedExecutor`. Injects `/selected_spot` and `/loc_pose` messages and asserts a route is published on `/route`.

### Integration Test — RouteComputerCardinal (`integration_test_cardinal.py`)

The most complete test. Uses `unittest.mock.patch` to bypass map file loading, then manually injects a 6-node graph. Injects all three required inputs (`/loc_pose`, `/selected_spot`, `/euler_angles`), manually triggers `timer_callback`, and spins the executor. Asserts both `/route` and `/route_state` are received.

| Test | Description |
|---|---|
| `test_route_computation_and_publishing` | Full end-to-end: inject all inputs, trigger timer, assert `/route` received and `/route_state` is `true`. |

### Linting Tests

| Test | Tool |
|---|---|
| `test_copyright.py` | ament_copyright |
| `test_flake8.py` | flake8 |
| `test_pep257.py` | pep257 |

### Running All Tests

```bash
cd ~/ros2_ws
colcon test --packages-select adapt_roucomp
colcon test-result --verbose
```

---

## Rosbag Data

Six rosbag recordings captured during development cover distinct vehicle trajectory scenarios. All bags record only `/pose_modelcars` (MoCap rigid body stream) and were recorded on 2024-07-04.

| Folder | Scenario | Duration | Messages |
|---|---|---|---|
| `origin-1/` | Vehicle starting from origin, scenario 1 | ~8.5 s | 256 |
| `origin-2/` | Vehicle starting from origin, scenario 2 | ~13.3 s | 399 |
| `cross-l/` | Left-crossing manoeuvre at intersection | ~7.8 s | 235 |
| `cross-r/` | Right-crossing manoeuvre at intersection | ~9.2 s | 277 |
| `diagnal/` | Diagonal path through the map | ~12.8 s | 384 |
| `reverse/` | Reverse manoeuvre | ~6.2 s | 188 |

To replay:
```bash
ros2 bag play src/route_computer/ros2_bags/diagnal/
```

---


## Integration with ADAPT

| Component | Relationship |
|---|---|
| **Localisation (`adapt_loc`)** | Provides `/loc_pose` (vehicle position) and `/euler_angles` (yaw) — both required inputs before A\* can run. |
| **Behaviour Planning** | Provides `/selected_spot` (the destination) via its FSM state transitions. Also consumes `/route_state` to know when a valid route has been computed and execution can begin. |
| **Trajectory Planner** | Downstream consumer of `/route`. Receives the ordered `PoseArray` of waypoints and converts them into a smooth, executable trajectory for the actuation layer. |

---


## License

Apache License 2.0 — see [LICENSE](LICENSE) for details.