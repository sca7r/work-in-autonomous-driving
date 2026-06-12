# Messages (`adapt_msgs`)

## Overview

The **Messages** package (`adapt_msgs`) is the shared interface layer of the ADAPT (Autonomous Driving And Parking Technology) system. It defines all custom **ROS 2 message types** used for inter-component communication across the entire pipeline, from LiDAR perception and localisation through to behaviour planning, V2X communication, and actuation.

In ROS 2, custom message types must be compiled into their own dedicated package before any other package can import them. Centralising all ADAPT-specific `.msg` definitions here avoids circular dependencies, enforces consistent field naming and types system-wide, and provides a single place to evolve the data contracts between components.

This package uses `ament_cmake` and `rosidl` to generate the C++ and Python bindings that every other ADAPT component imports at runtime.

---

## Table of Contents

- [Why a Dedicated Messages Package?](#why-a-dedicated-messages-package)
- [Message Definitions](#message-definitions)
  - [CarCom.msg](#carcommsg)
  - [DetectedObject.msg](#detectedobjectmsg)
  - [DetectedObjects.msg](#detectedobjectsmsg)
  - [LaneInfo.msg](#laneinfomsg)
  - [LiveTrack.msg](#livetrackmsg)
  - [VehData.msg](#vehdatamsg)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Build System](#build-system)
- [Installation & Building](#installation--building)
- [Using Messages in Other Packages](#using-messages-in-other-packages)
- [Who Uses Each Message](#who-uses-each-message)


---

## Why a Dedicated Messages Package?

ROS 2's `rosidl` interface generation pipeline requires that `.msg` files reside in their own package, compiled independently before any downstream consumer. The benefits of centralising all ADAPT messages here are:

- **No circular dependencies** — any ADAPT component can declare `<depend>adapt_msgs</depend>` without risk of a build cycle.
- **Single source of truth** — field names, types, and units are defined once and shared across all nodes.
- **Easy versioning** — changes to a shared data structure (e.g., adding a field to `VehData`) are made in one file and propagate to all consumers on the next build.
- **Consistent units** — inline comments in each `.msg` file document the expected unit (metres, radians, degrees) directly alongside the field definition.

---

## Message Definitions

### CarCom.msg

**Purpose:** Commands sent to the ego vehicle's actuators. This is the primary control output message — published by Behaviour Planning or a controller node and consumed by the drive-by-wire / actuation layer.

```
# car command for the actuators
float64 linear    # linear velocity
float64 angular   # angular velocity
float64 steering  # steering angle
```

| Field | Type | Description |
|---|---|---|
| `linear` | `float64` | Desired linear (forward) velocity of the vehicle. |
| `angular` | `float64` | Desired angular velocity (yaw rate). |
| `steering` | `float64` | Desired steering angle. |

---

### DetectedObject.msg

**Purpose:** Represents a single obstacle detected by the LiDAR sensor, expressed in polar coordinates relative to the ego vehicle's base link frame. Used as the element type within `DetectedObjects`.

```
float32 distance
float32 angle
```

| Field | Type | Description |
|---|---|---|
| `distance` | `float32` | Radial distance from the vehicle to the detected obstacle, in metres. |
| `angle` | `float32` | Bearing angle to the obstacle, in radians. Positive is counter-clockwise from the vehicle's forward axis. |

---

### DetectedObjects.msg

**Purpose:** A stamped collection of `DetectedObject` entries published by the Environment Model node on `/scans`. The header carries the timestamp and frame ID (`9/base_link`) so downstream consumers know when and in which reference frame the detections were made.

```
std_msgs/Header header
DetectedObject[] objects
```

| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp and frame ID of the detection batch. Frame ID is `9/base_link`. |
| `objects` | `DetectedObject[]` | Variable-length array of detected obstacles. Empty array means no obstacles in the filtered scan window. |

**Dependencies:** `std_msgs` (for `Header`); `DetectedObject` (defined in this same package).

---

### LaneInfo.msg

**Purpose:** Describes a detected lane segment as produced by a lane detection algorithm (camera-based). Intended for consumption by a lane-keeping or path-following controller.

```
float64 confidence_score  # how confident is the algorithm in the detected lane
string  lane_type         # Solid, Dashed, etc.
int32   lane_width        # lane width in pixels
float64 curvature         # in Radians
float64 heading           # in Radians
int32   lane_center_x     # pixel position of lane centre on x
int32   lane_center_y     # pixel position of lane centre on y
string  description
```

| Field | Type | Unit | Description |
|---|---|---|---|
| `confidence_score` | `float64` | — | Algorithm confidence in the lane detection, typically in the range [0.0, 1.0]. |
| `lane_type` | `string` | — | Lane marking type, e.g. `"Solid"`, `"Dashed"`. |
| `lane_width` | `int32` | pixels | Width of the detected lane in image pixels. |
| `curvature` | `float64` | radians | Curvature of the detected lane. |
| `heading` | `float64` | radians | Heading angle of the lane relative to the image/vehicle frame. |
| `lane_center_x` | `int32` | pixels | X pixel coordinate of the lane centre in the camera image. |
| `lane_center_y` | `int32` | pixels | Y pixel coordinate of the lane centre in the camera image. |
| `description` | `string` | — | Free-text description or additional metadata from the detector. |

---

### LiveTrack.msg

**Purpose:** Communicates the live tracking status of a vehicle — its current 6DOF pose and an operational status string. Used for real-time vehicle tracking across the ADAPT system, including potential display in a monitoring interface.

```
# The current pose of the vehicle using ROS geometry_msgs/PoseStamped message type.
# This includes position and orientation of the vehicle.
geometry_msgs/PoseStamped pose

# The status of the vehicle as a string.
# Typical statuses might include "Moving", "Parked", etc.
string status
```

| Field | Type | Description |
|---|---|---|
| `pose` | `geometry_msgs/PoseStamped` | Full 6DOF pose of the vehicle: stamped position (x, y, z in metres) and orientation (quaternion). |
| `status` | `string` | Operational status of the vehicle, e.g. `"Moving"`, `"Parked"`. |

**Dependencies:** `geometry_msgs` (for `PoseStamped`).

---

### VehData.msg

**Purpose:** A comprehensive vehicle state message used for V2X (Vehicle-to-Everything) / CAM (Cooperative Awareness Message) communication between vehicles in the ADAPT system. Carries both GPS coordinates and local Cartesian position, plus full quaternion orientation and a vehicle ID.

```
float64 latitude      # latitude of the vehicle in degrees
float64 longitude     # longitude of the vehicle in degrees
float64 altitude      # altitude of the vehicle in degrees
float64 heading       # heading of the vehicle
float64 x_rotation    # x in quaternion
float64 y_rotation    # y in quaternion
float64 z_rotation    # z in quaternion
float64 w_rotation    # w in quaternion
float64 x             # cartesian value of the vehicle in the x direction (m)
float64 y             # cartesian value of the vehicle in the y direction (m)
float64 z             # cartesian value of the vehicle in the z direction (m)
int32   id            # vehicle id
```

| Field | Type | Unit | Description |
|---|---|---|---|
| `latitude` | `float64` | degrees | WGS-84 latitude of the vehicle. |
| `longitude` | `float64` | degrees | WGS-84 longitude of the vehicle. |
| `altitude` | `float64` | degrees (metres) | Altitude of the vehicle. Note: the inline comment says "degrees" but altitude is conventionally in metres — verify with the publishing node. |
| `heading` | `float64` | — | Heading of the vehicle (unit not specified; verify with publisher). |
| `x_rotation` | `float64` | — | X component of the orientation quaternion. |
| `y_rotation` | `float64` | — | Y component of the orientation quaternion. |
| `z_rotation` | `float64` | — | Z component of the orientation quaternion. |
| `w_rotation` | `float64` | — | W (scalar) component of the orientation quaternion. |
| `x` | `float64` | metres | Cartesian X position in the local coordinate frame. |
| `y` | `float64` | metres | Cartesian Y position in the local coordinate frame. |
| `z` | `float64` | metres | Cartesian Z position in the local coordinate frame. |
| `id` | `int32` | — | Unique identifier for the vehicle. Known IDs in ADAPT: `7` (yellow car), `9` (ego/yellow model car), `10` (blue car). |

---

## Package Structure

```
messages/
├── msg/
│   ├── CarCom.msg            # Actuator command (velocity + steering)
│   ├── DetectedObject.msg    # Single LiDAR-detected obstacle (distance + angle)
│   ├── DetectedObjects.msg   # Stamped array of DetectedObject
│   ├── LaneInfo.msg          # Camera-based lane detection result
│   ├── LiveTrack.msg         # Vehicle live tracking (pose + status)
│   └── VehData.msg           # V2X vehicle state (GPS + Cartesian + quaternion + ID)
├── CMakeLists.txt            # ament_cmake build + rosidl interface generation
├── package.xml               # Package metadata and dependencies
└── README.md
```

---

## Dependencies

### Build-time

| Dependency | Purpose |
|---|---|
| `ament_cmake` | CMake build system for ROS 2 packages. |
| `rosidl_default_generators` | Generates C++ and Python bindings from `.msg` files. |
| `geometry_msgs` | Provides `geometry_msgs/PoseStamped` used in `LiveTrack.msg`. |
| `std_msgs` | Provides `std_msgs/Header` used in `DetectedObjects.msg`. |
| `vision_msgs` | Included in the `rosidl_generate_interfaces` dependency list for potential future vision-related messages. |

### Runtime

| Dependency | Purpose |
|---|---|
| `rosidl_default_runtime` | Runtime library required to deserialise generated message types. |
| `geometry_msgs` | Runtime dependency for `LiveTrack.msg`. |
| `std_msgs` | Runtime dependency for `DetectedObjects.msg`. |
| `vision_msgs` | Exported as a runtime dependency. |

### Build System

- **ROS 2 Foxy** (or compatible)
- **ament_cmake** build type (note: this is a CMake package, not `ament_python`)

---

## Build System

The package uses `ament_cmake` with `rosidl_generate_interfaces` to compile all six `.msg` files into both C++ headers and Python modules. The `CMakeLists.txt` registers the following interfaces:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneInfo.msg"
  "msg/CarCom.msg"
  "msg/VehData.msg"
  "msg/LiveTrack.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs
)
```

All dependencies are exported to downstream packages via `ament_export_dependencies`, so consumers of `adapt_msgs` automatically get transitive access to `std_msgs`, `geometry_msgs`, and `vision_msgs`.

---

## Installation & Building

This package **must be built before** any other ADAPT component that depends on it. Build it first in isolation:

```bash
cd ~/ros2_ws
colcon build --packages-select adapt_msgs
source install/setup.bash
```

Then build the rest of the workspace:

```bash
colcon build
source install/setup.bash
```

To verify that the messages were generated correctly:

```bash
# List all adapt_msgs message types
ros2 interface list | grep adapt_msgs

# Inspect a specific message definition
ros2 interface show adapt_msgs/msg/VehData
ros2 interface show adapt_msgs/msg/DetectedObjects
```

---

## Using Messages in Other Packages

### Declaring the dependency

In `package.xml` of the consuming package:

```xml
<depend>adapt_msgs</depend>
```

### Python import

```python
from adapt_msgs.msg import CarCom
from adapt_msgs.msg import DetectedObject
from adapt_msgs.msg import DetectedObjects
from adapt_msgs.msg import LaneInfo
from adapt_msgs.msg import LiveTrack
from adapt_msgs.msg import VehData
```

### C++ import

```cpp
#include "adapt_msgs/msg/car_com.hpp"
#include "adapt_msgs/msg/detected_object.hpp"
#include "adapt_msgs/msg/detected_objects.hpp"
#include "adapt_msgs/msg/lane_info.hpp"
#include "adapt_msgs/msg/live_track.hpp"
#include "adapt_msgs/msg/veh_data.hpp"
```

### Example: publishing a CarCom command

```python
from adapt_msgs.msg import CarCom

publisher = self.create_publisher(CarCom, '/car_com', 10)

msg = CarCom()
msg.linear = 0.5      # m/s forward
msg.angular = 0.0
msg.steering = 0.1    # radians
publisher.publish(msg)
```

### Example: publishing DetectedObjects

```python
from adapt_msgs.msg import DetectedObjects, DetectedObject

pub = self.create_publisher(DetectedObjects, '/scans', 10)

msg = DetectedObjects()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = '9/base_link'

obj = DetectedObject()
obj.distance = 0.75   # metres
obj.angle = 0.1       # radians
msg.objects.append(obj)

pub.publish(msg)
```

---

## Who Uses Each Message

| Message | Published by | Subscribed by |
|---|---|---|
| `CarCom` | Behaviour Planning / Controller | Actuation / Drive-by-wire node |
| `DetectedObject` | *(element of DetectedObjects; not published standalone)* | — |
| `DetectedObjects` | Environment Model (`/scans`) | Behaviour Planning, Route Computer |
| `LaneInfo` | Lane Detection node | Lane-keeping / Path-following controller |
| `LiveTrack` | Localisation / Tracking node | Monitoring interface, Behaviour Planning |
| `VehData` | V2X / CAM communication node | Environment Model Visualization (`/ev_location`) |

---


## License

Apache License 2.0 — see [LICENSE](LICENSE) for details.