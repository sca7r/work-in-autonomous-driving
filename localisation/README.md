# Localisation (`adapt_loc`)

## Overview

The **Localisation** module provides the Ego Vehicle with a precise, real-time understanding of its position and orientation in the world. Within the **ADAPT** (Autonomous Driving And Parking Technology) system, localisation is achieved using an **OptiTrack Motion Capture (MoCap)** system deployed inside a Model City test environment.

The node subscribes to raw rigid body pose data from the OptiTrack system, identifies the ego vehicle by its assigned rigid body ID, converts the raw quaternion orientation into Euler angles (roll, pitch, yaw), and publishes both the full 6DOF pose and the Euler angles for downstream consumption by other ADAPT components.

---

## Table of Contents

- [Architecture](#architecture)
- [ROS 2 Interface](#ros-2-interface)
- [Node Behaviour](#node-behaviour)
  - [Rigid Body Filtering](#rigid-body-filtering)
  - [Quaternion-to-Euler Conversion](#quaternion-to-euler-conversion)
  - [Position Rounding](#position-rounding)
  - [Publishing](#publishing)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Running the Node](#running-the-node)
- [Testing](#testing)
- [Configuration](#configuration)
- [Integration with ADAPT](#integration-with-adapt)
- [Notes & Limitations](#notes--limitations)

---

## Architecture

```
OptiTrack MoCap System
        │
        │  /pose_modelcars  (mocap_msgs/RigidBodies)
        │  — raw pose data for ALL rigid bodies in the scene
        ▼
┌──────────────────────────────────────────┐
│          SimpleLocalization Node          │
│                                          │
│  1. Filter for rigid body name == "9"    │
│  2. Round X, Y, Z to 4 decimal places    │
│  3. Quaternion → Euler conversion        │
│  4. Publish PoseStamped  (/loc_pose)     │
│  5. Publish Vector3      (/euler_angles) │
└──────────────────────────────────────────┘
        │                   │
        ▼                   ▼
  /loc_pose            /euler_angles
  (PoseStamped)        (Vector3)
  X, Y, Z +            Roll, Pitch, Yaw
  Quaternion           (radians)
```

---

## ROS 2 Interface

### Subscriptions

| Topic | Message Type | Description |
|---|---|---|
| `/pose_modelcars` | `mocap_msgs/RigidBodies` | Raw pose data (position + quaternion) for all rigid bodies tracked by the OptiTrack system. |

### Publications

| Topic | Message Type | Description |
|---|---|---|
| `/loc_pose` | `geometry_msgs/PoseStamped` | Full 6DOF pose of the ego vehicle: X, Y, Z position (metres, rounded to 4 d.p.) plus raw quaternion orientation. Frame ID: `base_link_7`. |
| `/euler_angles` | `geometry_msgs/Vector3` | Euler angle representation of the vehicle's orientation. `x` = roll, `y` = pitch, `z` = yaw, all in radians. |

---

## Node Behaviour

### Rigid Body Filtering

The OptiTrack system publishes pose data for **all** tracked rigid bodies in the scene. The node filters this stream and only processes the body whose `rigid_body_name` equals `"9"` — the ROS 2 ID assigned to the yellow model car (ego vehicle). All other rigid bodies are silently ignored.

```python
if body.rigid_body_name == "9":
    self.process_rigid_body(body)
```

> **Note:** If you rename the vehicle's rigid body in the OptiTrack software, you must update this string accordingly.

### Quaternion-to-Euler Conversion

The MoCap system returns orientation as a quaternion `(x, y, z, w)`. The node converts this to intrinsic Euler angles using the standard ZYX (yaw-pitch-roll) decomposition:

| Angle | Axis | Formula |
|---|---|---|
| **Roll** | X | `atan2(2(wx + yz), 1 − 2(x² + y²))` |
| **Pitch** | Y | `asin(2(wy − zx))` clamped to `[−1, +1]` |
| **Yaw** | Z | `atan2(2(wz + xy), 1 − 2(y² + z²))` |

All angles are returned and published in **radians**.

### Position Rounding

Raw floating-point position values from the MoCap system are rounded to **4 decimal places** before publishing:

```python
rounded_x = round(body.pose.position.x, 4)
```

This provides sub-millimetre precision (0.1 mm) while avoiding unnecessary floating-point noise in downstream subscribers.

### Publishing

On every callback invocation that matches the ego vehicle, the node publishes two messages simultaneously:

1. **`/loc_pose` (`PoseStamped`)** — includes a timestamp (`get_clock().now()`), frame ID (`base_link_7`), rounded XYZ position, and the raw quaternion orientation.
2. **`/euler_angles` (`Vector3`)** — `x` = roll, `y` = pitch, `z` = yaw in radians.

Both publications are logged at `INFO` level.

---

## Package Structure

```
localisation/
├── adapt_loc/
│   ├── __init__.py
│   └── localization.py        # Main ROS 2 node (SimpleLocalization)
├── images/
│   ├── Localization5.png
│   ├── demo_m5.png
│   ├── loc_bloc_v4.png
│   ├── localization_rqt.png
│   └── rqt_graph_5.png
├── launch/
│   └── loc_launch.py          # ROS 2 launch file
├── resource/
│   └── adapt_loc              # Ament resource index marker
├── test/
│   ├── test_localization.py   # Unit tests
│   └── test_integration.py    # Integration tests
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
| `geometry_msgs` | `PoseStamped`, `Vector3`, `Pose`, `Quaternion` message types |
| `mocap_msgs` | `RigidBodies` / `RigidBody` message types from the OptiTrack driver |
| `std_msgs` | Standard ROS message types |
| `ros2launch` | Launch system support |

### External

| Dependency | Purpose |
|---|---|
| `mocap_optitrack` | ROS driver for the OptiTrack hardware — [ros-drivers/mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack) |
| `numpy` | Numerical utilities (available as a runtime dependency) |

### Hardware

- **OptiTrack Motion Capture system** with the Model City environment set up and calibrated.

### Build System

- **ROS 2 Foxy** (or compatible)
- **ament_python** build type

---

## Installation

1. Clone or copy this package into your ROS 2 workspace `src/` directory:

   ```bash
   cd ~/ros2_ws/src
   # copy or clone the package here
   ```

2. Install the `mocap_msgs` and `mocap_optitrack` dependencies:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:

   ```bash
   colcon build --packages-select adapt_loc
   ```

4. Source the workspace:

   ```bash
   source install/setup.bash
   ```

---

## Running the Node

### Using the launch file (recommended)

```bash
ros2 launch adapt_loc loc_launch.py
```

This starts the `localization` executable from the `adapt_loc` package with the node name `localization`.

### Running directly

```bash
ros2 run adapt_loc localization
```

### Verifying output

Once running, you can monitor the published topics with:

```bash
# Full 6DOF pose
ros2 topic echo /loc_pose

# Euler angles
ros2 topic echo /euler_angles
```

You can also inspect the topic graph with `rqt_graph`.

---

## Testing

The package includes two test suites under `test/`.

### Unit Tests (`test_localization.py`)

Tests individual methods of the `SimpleLocalization` node in isolation:

| Test | Description |
|---|---|
| `test_quaternion_to_euler` | Verifies conversion correctness for an identity quaternion (→ 0, 0, 0) and a 180° yaw rotation around Z (→ 0, 0, π). |
| `test_publish_pose` | Calls `publish_pose` and confirms no exceptions are raised. |
| `test_publish_euler_angles` | Calls `publish_euler_angles` and confirms no exceptions are raised. |
| `test_process_rigid_body` | Creates a synthetic `RigidBody` with name `"9"` and exercises the full processing pipeline. |
| `test_position_callback` | Calls `position_callback` with a `RigidBodies` message containing body `"9"`, and verifies it handles both matching and non-matching body names without error. |

### Integration Tests (`test_integration.py`)

Tests the full publish/subscribe pipeline end-to-end using `SingleThreadedExecutor`:

| Test | Description |
|---|---|
| `test_localization_integration` | Spins both `SimpleLocalization` and a helper subscriber node. Injects a `RigidBodies` message for body `"9"` and asserts that `/loc_pose` and `/euler_angles` messages are received. Then repeats with body `"10"` and asserts that **no** messages are received, confirming correct filtering. |

### Running the tests

```bash
cd ~/ros2_ws
colcon test --packages-select adapt_loc
colcon test-result --verbose
```

Or directly with pytest (with ROS 2 sourced):

```bash
pytest src/localisation/test/
```

---

## Configuration

There are currently no runtime parameters or config files. The following values are hardcoded and would need to be changed in source if required:

| Value | Location | Description |
|---|---|---|
| `"9"` | `localization.py` → `position_callback` | Rigid body name (ROS 2 ID) of the ego vehicle in OptiTrack. |
| `'/pose_modelcars'` | `localization.py` → `__init__` | Input topic name from the MoCap driver. |
| `'base_link_7'` | `localization.py` → `publish_pose` | TF frame ID attached to published `PoseStamped` messages. |
| `4` | `localization.py` → `process_rigid_body` | Decimal places for position rounding. |

---

## Integration with ADAPT

This component sits at the **sensing layer** and feeds several downstream modules:

| Component | How it uses localisation |
|---|---|
| **Environment Model** | Uses `/loc_pose` to anchor the occupancy grid to the vehicle's current position in the Model City map. |
| **Route Computer** | Uses the current position as the starting point for path planning queries. |
| **Behaviour Planning** | Tracks vehicle progress along a planned route by comparing current position against waypoints. |

---

## Notes & Limitations

- **Real-world alternative:** For outdoor or real-world deployment, the localisation source should be replaced with a **GNSS + IMU fusion** system (e.g., using `robot_localization`) or an **HD Map-based** localisation solution.
- **Frame convention:** Positions are published in the ENU (East-North-Up) coordinate frame as provided by the MoCap system.
- **Pitch gimbal lock:** The pitch calculation clamps the intermediate value to `[−1, +1]` to avoid `asin` domain errors, but gimbal lock at ±90° pitch is an inherent limitation of Euler angle representation.


---


## License

Apache License 2.0 — see [LICENSE](LICENSE) for details.