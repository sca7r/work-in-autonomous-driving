# Environment Model (`adapt_envmod`)

## Overview

The **Environment Model** module is the perception backbone of the ADAPT (Autonomous Driving And Parking Technology) system. It is responsible for processing raw sensor data from a **2D LiDAR scanner**, anchoring it to the ego vehicle's pose received from the **Localisation** component, and producing two key outputs:

1. **`/scans`** — a stream of transformed, filtered `DetectedObjects` (obstacle positions in the vehicle frame) consumed by Behaviour Planning and Route Computer.
2. **`/stop`** — a binary stop signal that triggers an immediate halt when any obstacle falls within a configurable safety range.

A second node — the **Visualization** node — runs alongside the environment model and publishes TF transforms and 3D mesh markers for the ego vehicle and all other vehicles in the scene, making the full multi-vehicle environment visible in RViz.

---

## Table of Contents

- [Architecture](#architecture)
- [Nodes](#nodes)
  - [EnvModel (`env.py`)](#envmodel-envpy)
  - [Environment / Visualization (`visualization.py`)](#environment--visualization-visualizationpy)
- [ROS 2 Interface](#ros-2-interface)
- [Processing Pipeline (EnvModel)](#processing-pipeline-envmodel)
  - [Scan Filtering by Angle](#scan-filtering-by-angle)
  - [TF Coordinate Transform](#tf-coordinate-transform)
  - [Stop Detection](#stop-detection)
  - [QoS Profiles](#qos-profiles)
- [Visualization Node Detail](#visualization-node-detail)
  - [Vehicle Colours](#vehicle-colours)
  - [3D Mesh Marker](#3d-mesh-marker)
- [Static Map Resource](#static-map-resource)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Running the Nodes](#running-the-nodes)
- [Parameters](#parameters)
- [Testing](#testing)
- [Rosbag Data](#rosbag-data)
- [Integration with ADAPT](#integration-with-adapt)

---

## Architecture

```
                          /scan  (sensor_msgs/LaserScan)
                          │  BEST_EFFORT QoS, depth 1
                          ▼
              ┌───────────────────────────┐
/loc_pose ───▶│        EnvModel Node       │──▶ /scans   (adapt_msgs/DetectedObjects)
(PoseStamped) │  - angle filter [-0.5, 0.5]│──▶ /stop    (std_msgs/Bool)
              │  - TF lookup (laser→base)  │
              │  - stop range check        │
              │  - 10 Hz timer publish     │
              └───────────────────────────┘

/loc_pose ───▶ ┌──────────────────────────┐──▶ /tf_9          (TransformStamped)
(PoseStamped)  │   Environment (Viz) Node  │──▶ /car_marker_9  (Marker)
               │                           │
/ev_location ─▶│  - TF broadcast per car   │──▶ /tf_7          (TransformStamped)
(VehData)      │  - Mesh marker per car    │──▶ /car_marker_7  (Marker)
               └──────────────────────────┘──▶ /tf_10 / /car_marker_10
```

---

## Nodes

The launch file starts **both** nodes together under the `adapt_envmod` package.

### EnvModel (`env.py`)

| Property | Value |
|---|---|
| Node name | `env_model` |
| Executable | `envmod` |
| Launch name | `env` |
| Timer period | 100 ms (10 Hz) |

Subscribes to raw LiDAR scans and the ego vehicle pose. On each timer tick it processes the most recent stored scan, filters it to a forward-facing angular window, transforms valid detections into the vehicle frame via TF2, and publishes detected objects plus a stop flag.

### Environment / Visualization (`visualization.py`)

| Property | Value |
|---|---|
| Node name | `Environment` |
| Executable | `map` |
| Launch name | `map` |

Subscribes to the ego vehicle's localisation pose and to V2X `VehData` messages from other vehicles. For each vehicle it publishes a `TransformStamped` (for RViz TF tree) and a `Marker` (3D `.obj` mesh) so the full multi-vehicle scene is visible in RViz.

---

## ROS 2 Interface

### EnvModel — Subscriptions

| Topic | Message Type | QoS | Description |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | BEST_EFFORT, KEEP_LAST 1 | Raw 2D LiDAR scan from the on-board laser. |
| `/loc_pose` | `geometry_msgs/PoseStamped` | BEST_EFFORT, KEEP_LAST 1 | Ego vehicle pose from the Localisation component. |

### EnvModel — Publications

| Topic | Message Type | QoS | Description |
|---|---|---|---|
| `/scans` | `adapt_msgs/DetectedObjects` | Default (depth 10) | List of detected obstacles in the `9/base_link` frame; each object has `distance` (m) and `angle` (rad). |
| `/stop` | `std_msgs/Bool` | RELIABLE, TRANSIENT_LOCAL, KEEP_LAST 1 | `true` when any detected obstacle is within `stop_range`; `false` otherwise. |

### Visualization — Subscriptions

| Topic | Message Type | Description |
|---|---|---|
| `/loc_pose` | `geometry_msgs/PoseStamped` | Ego vehicle (car ID 9) pose from Localisation. |
| `/ev_location` | `adapt_msgs/VehData` | Position and orientation of other vehicles (IDs 7, 10) from V2X / CAM messages. |

### Visualization — Publications

| Topic | Message Type | Description |
|---|---|---|
| `/tf_9` | `geometry_msgs/TransformStamped` | TF transform from `map` to `car_9` (ego vehicle). |
| `/car_marker_9` | `visualization_msgs/Marker` | 3D mesh marker for the ego vehicle. |
| `/tf_7` | `geometry_msgs/TransformStamped` | TF transform from `map` to `car_7`. |
| `/car_marker_7` | `visualization_msgs/Marker` | 3D mesh marker for car 7 (yellow). |
| `/tf_10` | `geometry_msgs/TransformStamped` | TF transform from `map` to `car_10`. |
| `/car_marker_10` | `visualization_msgs/Marker` | 3D mesh marker for car 10 (blue). |

---

## Processing Pipeline (EnvModel)

### Scan Filtering by Angle

The LiDAR produces a full 360° (or wide-angle) scan. The node restricts processing to a **forward-facing window** of approximately ±0.5 radians (~±28.6°) around the vehicle's heading.

The effective angle for each ray is calculated as:

```
angle = angle_start + scan.angle_min + i * scan.angle_increment
```

Only rays where `-0.5 < angle < 0.5` are processed. Rays returning `inf`, `nan`, or `0.0` are also discarded as invalid readings.

### TF Coordinate Transform

For each valid ray, the polar reading `(distance, angle)` is first converted to Cartesian coordinates in the **laser frame**:

```
x_laser = distance * cos(angle)
y_laser = distance * sin(angle)
```

The node then queries TF2 for the transform between `9/laser_frame` and `9/base_link` and applies it:

```
x_base = (x_laser * cos(yaw) − y_laser * sin(yaw)) + translation.x
y_base = (x_laser * sin(yaw) + y_laser * cos(yaw)) + translation.y
```

The quaternion from the TF transform is converted to Euler angles internally to extract the yaw component for the 2D rotation. If the TF lookup fails (e.g., transform not yet available), the ray is silently skipped and a warning is logged.

The transformed distance and angle are packed into a `DetectedObject` and appended to the `DetectedObjects` message.

### Stop Detection

After all rays have been processed, the node checks whether any object's transformed distance falls within `stop_range`:

```python
obstacle_detected = any(obj.distance <= self.stop_range for obj in detected_objects_msg.objects)
```

A `Bool` message is published to `/stop` on every timer tick regardless of whether an obstacle is present, ensuring downstream subscribers always have a current value (including `false` when the path is clear).

### QoS Profiles

Two QoS profiles are used deliberately:

| Profile | Used for | Rationale |
|---|---|---|
| `BEST_EFFORT`, `KEEP_LAST 1` | `/scan`, `/loc_pose` subscriptions | Sensor data; a dropped message is acceptable, latency is not. |
| `RELIABLE`, `TRANSIENT_LOCAL`, `KEEP_LAST 1` | `/stop` publication | Safety-critical; late-joining subscribers must receive the current stop state immediately. |

---

## Visualization Node Detail

### Vehicle Colours

The mesh marker colour is assigned per car ID:

| Car ID | Role | Colour |
|---|---|---|
| 9 | Ego vehicle | Grey (R=0.5, G=0.5, B=0.5) |
| 7 | Other vehicle | Yellow (R=1.0, G=1.0, B=0.0) |
| 10 | Other vehicle | Blue (R=0.0, G=0.0, B=1.0) |

### 3D Mesh Marker

All vehicles are rendered using the same `.obj` mesh:

```
package://adapt_envmod/resource/adaptcar.obj
```

Scale is set uniformly to `0.1` on all axes. The marker type is `Marker.MESH_RESOURCE`, which requires RViz to be able to resolve the `package://` URI.

---

## Static Map Resource

The package ships with a static 2D map of the Model City environment for reference and potential nav_msgs/Map use:

| File | Description |
|---|---|
| `resource/adapt_map.pgm` | PGM occupancy image (grayscale bitmap) |
| `resource/adapt_map.yaml` | Map metadata |

Map parameters from `adapt_map.yaml`:

| Parameter | Value |
|---|---|
| Resolution | 0.05 m/pixel (5 cm per cell) |
| Origin | [0.0, 0.0, 0.0] |
| Occupied threshold | 0.65 |
| Free threshold | 0.196 |

---

## Package Structure

```
environment_model/
├── adapt_envmod/
│   ├── __init__.py
│   ├── env.py                  # EnvModel node — LiDAR processing & stop detection
│   └── visualization.py        # Environment node — TF & RViz marker publishing
├── images/                     # Screenshots and diagrams for documentation
│   ├── blockdig.jpg
│   ├── interface.jpg
│   ├── scan.png
│   ├── scans1.png / scans2.png
│   ├── stop.png / stop_rviz.png / stop_rviz2.png
│   ├── testenv.png
│   ├── visualisation.jpg
│   ├── viz.png
│   └── viztest.png
├── launch/
│   └── env_launch.py           # Starts both envmod and visualization nodes
├── resource/
│   ├── adapt_envmod            # Ament resource index marker
│   ├── adapt_map.pgm           # Static Model City occupancy map
│   ├── adapt_map.yaml          # Map metadata (resolution, thresholds)
│   └── adaptcar.obj            # 3D mesh model used for RViz markers
├── rosbag/
│   ├── alltopics/              # Full session recording (~61 s, 9580 msgs)
│   ├── locpose/                # /loc_pose only recording (~115 s, 3455 msgs)
│   ├── output/                 # /scans + /loc_pose recording (~47 s, 1396 msgs)
│   ├── posemodelcars/          # /pose_modelcars recording
│   └── scanpose/               # /scan + /pose_modelcars (~37 s, 1502 msgs)
├── test/
│   ├── env_test.py             # Unit tests for EnvModel
│   ├── env_integrationtest.py  # Integration test for EnvModel
│   ├── visualization_test.py   # Unit tests for Environment (viz) node
│   ├── test_copyright.py       # Ament copyright linting
│   ├── test_flake8.py          # Flake8 style checks
│   ├── test_pep257.py          # PEP 257 docstring checks
│   └── test_reports/
│       ├── env_test_report.md
│       ├── Visualization _test_report.md
│       ├── integration_test_report.txt
│       ├── lizard_report.txt
│       └── pylint_report.txt
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

---

## Dependencies

### ROS 2

| Dependency | Purpose |
|---|---|
| `rclpy` | ROS 2 Python client library |
| `sensor_msgs` | `LaserScan` message type |
| `geometry_msgs` | `PoseStamped`, `TransformStamped`, `Vector3`, `Point` |
| `std_msgs` | `Bool` message type |
| `visualization_msgs` | `Marker` message type |
| `tf2_ros` | TF2 transform broadcaster, buffer, and listener |
| `vision_msgs` | Vision-related message types |
| `adapt_msgs` | Custom ADAPT messages: `DetectedObjects`, `DetectedObject`, `VehData` |
| `ros2launch` | Launch system support |

### Build System

- **ROS 2 Foxy** (or compatible)
- **ament_python** build type

---

## Installation

1. Place the package in your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   # copy or clone adapt_envmod here
   ```

2. Ensure `adapt_msgs` is also present in the workspace (it is a direct dependency):

   ```bash
   ls ~/ros2_ws/src/adapt_msgs
   ```

3. Install dependencies:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build:

   ```bash
   colcon build --packages-select adapt_envmod
   ```

5. Source:

   ```bash
   source install/setup.bash
   ```

---

## Running the Nodes

### Using the launch file (recommended)

Starts both the `EnvModel` and `Environment` (visualization) nodes together:

```bash
ros2 launch adapt_envmod env_launch.py
```

### Running nodes individually

```bash
# Environment model (LiDAR processing)
ros2 run adapt_envmod envmod

# Visualization only
ros2 run adapt_envmod map
```

### Verifying output

```bash
# Watch detected objects
ros2 topic echo /scans

# Watch stop signal
ros2 topic echo /stop

# Check topic rates
ros2 topic hz /scans
ros2 topic hz /stop
```

Open **RViz** and add:
- **TF** display to see vehicle transforms under the `map` frame.
- **Marker** displays on `/car_marker_7`, `/car_marker_9`, `/car_marker_10` to see the 3D vehicle meshes.

---

## Parameters

Both parameters are declared with defaults and can be overridden at launch or via `ros2 param set`:

| Parameter | Default | Description |
|---|---|---|
| `angle_start` | `1.0472` rad (~60°) | Angular offset applied to each scan ray to correct for LiDAR mounting orientation relative to the vehicle's forward axis. |
| `stop_range` | `0.95` m | Distance threshold in metres. Any detected object closer than this value causes `true` to be published on `/stop`. |

### Overriding at launch

```bash
ros2 run adapt_envmod envmod --ros-args -p angle_start:=0.0 -p stop_range:=0.75
```

---

## Testing

The package has three test files plus ament linting checks.

### Unit Tests — EnvModel (`env_test.py`)

Uses `unittest` with mocked publishers (`unittest.mock.MagicMock`) to isolate node logic from the ROS middleware.

| Test ID | Test Name | Description |
|---|---|---|
| TC_01 | `test_init_node` | Verifies node is created with name `env_model` and is an instance of `EnvModel`. |
| TC_02 | `test_pose_callback` | Sends a `PoseStamped` to `pose_callback` and asserts `base_link_pose` is updated with correct X, Y, Z values. |
| TC_03 | `test_scan_callback` | Injects a `LaserScan` with mixed ranges, sets up a TF transform in the buffer, and asserts that `detected_objects_publisher.publish` is called with the correct number of detected objects. |
| TC_04 | `test_object_detection_within_stop_range` | Injects scan ranges all below `stop_range` and asserts `stop_publisher.publish` is called with `data=True`. |

All four tests passed. (See `test/test_reports/env_test_report.md`.)

### Unit Tests — Visualization (`visualization_test.py`)

Uses `unittest` with a `MockNode` helper. Tests TF and Marker publishing via `tf2_ros.Buffer` and subscription callbacks.

| Test Name | Description |
|---|---|
| `test_node_initialization` | Confirms `Environment` node name and type. |
| `test_loc_pose_callback` | Sends a `PoseStamped` for car 9, looks up the TF transform `map → car_9`, and asserts translation and rotation values match within 0.01 tolerance. |
| `test_ev_location_callback` | Sends a `VehData` message for car 7, looks up `map → car_7`, and asserts transform values. |
| `test_marker_publishing` | Subscribes to `/car_marker_7` and validates mesh resource path, scale, pose, and colour values. |

All four tests passed. (See `test/test_reports/Visualization _test_report.md`.)

### Integration Test (`env_integrationtest.py`)

Runs `EnvModel` and an `IntegrationHelperNode` in a `SingleThreadedExecutor`. The helper publishes `/loc_pose` and `/scan` messages and subscribes to `/stop`.

| Test Name | Description |
|---|---|
| `test_env_model_integration` | Publishes 32 scan ranges all equal to 1.0 m (above default `stop_range` of 0.95 m) and asserts that `/stop` is **not** received as `true`, confirming no false stop trigger for clear-path conditions. Also validates no messages are emitted for a non-matching body name. |

### Linting Tests

| Test | Tool | Status |
|---|---|---|
| `test_copyright.py` | ament_copyright | Checks source file copyright headers. |
| `test_flake8.py` | flake8 | PEP 8 style compliance. |
| `test_pep257.py` | pep257 | Docstring convention checks. |

### Running All Tests

```bash
cd ~/ros2_ws
colcon test --packages-select adapt_envmod
colcon test-result --verbose
```

Or directly with pytest:

```bash
pytest src/environment_model/test/
```

---

## Rosbag Data

The package ships with five rosbag recordings captured during development for replay-based testing and debugging:

| Folder | Topics | Duration | Messages | Notes |
|---|---|---|---|---|
| `alltopics/` | `/pose_modelcars`, `/scan`, `/tf`, `/tf_static` | ~61 s | 9,580 | Full session; `/scan` recorded 0 messages in this bag. |
| `locpose/` | `/loc_pose` | ~115 s | 3,455 | Ego pose stream only. |
| `output/` | `/loc_pose`, `/scans` | ~47 s | 1,396 | Recorded output from EnvModel (all messages are `/loc_pose`). |
| `posemodelcars/` | `/pose_modelcars` | — | — | Raw MoCap rigid body stream. |
| `scanpose/` | `/pose_modelcars`, `/scan` | ~37 s | 1,502 | Simultaneous LiDAR + MoCap; used for scan processing validation. |

To replay a bag:

```bash
ros2 bag play src/environment_model/rosbag/scanpose/
```

---


## Integration with ADAPT

This module connects to several other ADAPT components:

| Component | Relationship |
|---|---|
| **Localisation (`adapt_loc`)** | Upstream provider of `/loc_pose`. The EnvModel subscribes to this to know the ego vehicle's current position; the Visualization node uses it to render the ego vehicle in RViz. |
| **Behaviour Planning** | Downstream consumer of `/scans` (object list) and `/stop` (immediate halt signal). Uses the detected obstacles to decide whether to proceed, slow down, or stop. |
| **Route Computer** | Downstream consumer of `/scans`. Uses the obstacle map to plan collision-free paths. |
| **V2X / CAM** | Upstream provider of `/ev_location` (`VehData`). Supplies the positions of other vehicles in the scene for the Visualization node. |



---


## License

Apache License 2.0 — see [LICENSE](LICENSE) for details.