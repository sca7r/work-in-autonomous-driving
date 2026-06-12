# Transceiver (`adapt_transceiver`)

## Overview

The **Transceiver** is the V2X (Vehicle-to-Everything) communication layer of the ADAPT (Autonomous Driving Platform and Test) system. It serves as a bidirectional protocol bridge between the internal ROS 2 message bus and the ETSI ITS-compliant V2X network, enabling the ego vehicle to announce its own presence, share its perception of the environment, and receive position data from other vehicles participating in the network.

The package contains two nodes with distinct responsibilities:

- **`Transceiver`** (`transceiver.py`) — handles **CAM (Cooperative Awareness Message)** transmission and reception. Publishes the ego vehicle's own CAM at 2 Hz, subscribes to CAMs from other vehicles on the shared `/cam_msgs` topic, converts received CAM positions from geodetic (WGS-84) to local ENU (East-North-Up) coordinates, and re-publishes the results as `VehData` messages on `/ev_location` for use by the Environment Model's Visualization node.

- **`CpmPublisher`** (`CPM.py`) — handles **CPM (Collective Perception Message)** transmission. Subscribes to DetectNet camera detections, maps object class IDs to ETSI CPM class codes, and publishes `CPM` messages on `/detected_objects` to share the ego vehicle's perceived environment with surrounding infrastructure and vehicles.

---

## Table of Contents

- [Architecture](#architecture)
- [V2X Message Types](#v2x-message-types)
- [ROS 2 Interface](#ros-2-interface)
- [Nodes](#nodes)
  - [Transceiver (transceiver.py)](#transceiver-transceiverpy)
  - [CpmPublisher (CPM.py)](#cpmpublisher-cpmpy)
- [Coordinate System Conversions](#coordinate-system-conversions)
  - [ENU to Geodetic (Outbound CAM)](#enu-to-geodetic-outbound-cam)
  - [Geodetic to ENU (Inbound CAM)](#geodetic-to-enu-inbound-cam)
  - [Yaw to WGS-84 Heading (Outbound CAM)](#yaw-to-wgs-84-heading-outbound-cam)
  - [Heading to Quaternion (Inbound CAM)](#heading-to-quaternion-inbound-cam)
- [Reference Coordinates](#reference-coordinates)
- [Object Class Mapping](#object-class-mapping)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Running the Nodes](#running-the-nodes)
- [Testing with Dummy Data](#testing-with-dummy-data)
- [Integration with ADAPT](#integration-with-adapt)

---

## Architecture

```
                          ┌──────────────────────────────────────────────┐
                          │              Transceiver Node                │
                          │                                              │
/loc_pose ───────────────▶│  ENU → Geodetic (pymap3d)                    │
(PoseStamped)             │  Yaw → WGS-84 heading                        │──▶ /cam_msgs (CAM)  [2 Hz]
                          │  Encode → CAM (ETSI ITS)                     │
/euler_angles ───────────▶│                                              │
(Vector3)                 │                                              │
                          │  /cam_msgs (CAM) ─────────────────────────── │◀── other vehicles
                          │  Filter out own station_id = 9               │
                          │  Geodetic → ENU (pymap3d)                    │
                          │  Heading → Quaternion                        │──▶ /ev_location (VehData) [1 Hz]
                          │  Decode → VehData                            │
                          └──────────────────────────────────────────────┘

                          ┌──────────────────────────────────────────────┐
                          │             CpmPublisher Node                │
                          │                                              │
/detectnet/detections ───▶│  Map class IDs → ETSI CPM object codes       │──▶ /detected_objects (CPM)
(Detection2DArray)        │  Encode → CPM (ETSI ITS)                     │
                          └──────────────────────────────────────────────┘
```

---

## V2X Message Types

The package implements ETSI ITS G5 standardised message types via the `v2x` ROS 2 message package.

| Message | Standard | Direction | Purpose |
|---|---|---|---|
| **CAM** | ETSI EN 302 637-2 | Outbound + Inbound | Cooperative Awareness Message, broadcasts vehicle position (geodetic), speed, heading, and station ID to all participants. Published at 2 Hz. |
| **CPM** | ETSI TR 103 562 | Outbound | Collective Perception Message, shares the ego vehicle's detected objects (from DetectNet) with surrounding infrastructure and vehicles. Published on new detections. |


---

## ROS 2 Interface

### Transceiver Node — Subscriptions

| Topic | Message Type | Description |
|---|---|---|
| `/loc_pose` | `geometry_msgs/PoseStamped` | Ego vehicle pose from Localisation. X, Y, Z are treated as ENU (East, North, Up) coordinates and converted to geodetic for inclusion in outbound CAM. |
| `/euler_angles` | `geometry_msgs/Vector3` | Euler angles from Localisation. The Z component (yaw, radians) is converted to WGS-84 heading (degrees, 0–360) for the CAM heading field. |
| `/cam_msgs` | `v2x/CAM` | Shared CAM topic. The node publishes its own CAM here and simultaneously subscribes to receive CAMs from other vehicles. Messages with `station_id = 9` (the ego vehicle itself) are filtered out. |

### Transceiver Node — Publications

| Topic | Message Type | Rate | Description |
|---|---|---|---|
| `/cam_msgs` | `v2x/CAM` | 2 Hz | Ego vehicle CAM. Station ID = 9. Contains geodetic position (lat × 10⁷, lon × 10⁷, alt × 100), WGS-84 heading × 10, station type = 5 (passenger car). |
| `/ev_location` | `adapt_msgs/VehData` | 1 Hz | Decoded position data for each other vehicle seen on the CAM network. Contains geodetic coordinates, local ENU Cartesian coordinates, quaternion orientation, heading, and vehicle ID. |

### CpmPublisher Node — Subscriptions

| Topic | Message Type | Description |
|---|---|---|
| `/detectnet/detections` | `vision_msgs/Detection2DArray` | Object detection results from DetectNet (NVIDIA camera-based inference). Each detection includes a class ID and confidence score. |

### CpmPublisher Node — Publications

| Topic | Message Type | Description |
|---|---|---|
| `/detected_objects` | `v2x/CPM` | ETSI CPM message containing all currently detected objects, their ETSI class codes, and confidence scores. |

---

## Nodes

### Transceiver (`transceiver.py`)

**Node name:** `transceiver_data`
**Executable:** `transceiver_node`

Handles both outbound CAM transmission and inbound CAM reception in a single node. Operates on two timer-driven loops:

**CAM Publisher (2 Hz, `cam_callback`):**
- Fills the full ETSI CAM structure: `ItsPduHeader` → `CoopAwareness` → `CamParameters` → `BasicContainer` → `ReferencePosition`.
- Position fields are populated from the most recent `/loc_pose` callback (converted to integer-scaled geodetic).
- Heading is populated from the most recent `/euler_angles` callback (converted to WGS-84 degrees × 10).
- Fixed fields: `protocol_version = 2`, `message_id = 2`, `station_id = 9`, `station_type = 5`, `generation_deltatime = 121`.

**EV Location Publisher (1 Hz, `evlocation_callback`):**
- Iterates over `cam_data` (the in-memory list of all known remote vehicles).
- Publishes one `VehData` message per known vehicle, carrying full position, orientation, and ID.

**CAM Subscriber (`cam_subscriber`):**
- Receives all messages on `/cam_msgs`.
- Ignores messages where `station_id = 9` (self-filtering).
- For each new `station_id`, creates a new entry in `cam_data` and appends the `station_id` to `seen_ids`.
- For already-known vehicles, finds the existing entry in `cam_data` by `vehicle_id` and updates all fields in-place.
- Converts geodetic position to ENU via `pm.geodetic2enu` and WGS-84 heading to quaternion via `quaternion_from_euler`.

### CpmPublisher (`CPM.py`)

**Node name:** `cpm_publisher`
**Executable:** `cpm`

Translates DetectNet detection results into ETSI CPM format. On each `/detectnet/detections` callback:

1. Creates a `CPM` message with `message_id = 14`.
2. Sets segmentation info: `total_msg_no = 1`, `this_msg_no = 1` (single-segment CPM).
3. Sets originating vehicle container fields (orientation/pitch/roll all set to `3601` = unavailable, confidence = `127` = unavailable), these are placeholder values.
4. For each detection in the `Detection2DArray`:
   - Creates a `PerceivedObject` with an incremental `object_id`.
   - For each result in the detection, converts DetectNet class ID to CPM class code (`ord(id) + 1`), looks up the human-readable name from `object_list_dict`, and sets confidence as an integer percentage (`score × 100`).
5. Publishes the assembled CPM on `/detected_objects`.

---

## Coordinate System Conversions

The transceiver performs four coordinate-related transformations, all tied to a fixed geodetic reference point for the ADAPT Model City.

### ENU to Geodetic (Outbound CAM)

The `/loc_pose` position is in local ENU metres (as provided by the OptiTrack MoCap system). For transmission in a CAM, it must be expressed in geodetic (WGS-84) degrees.

Conversion using `pymap3d.enu2geodetic`:

```python
lat, lon, alt = pm.enu2geodetic(east, north, up, LAT0, LON0, ALT0)
```

CAM integer encoding (ETSI standard scaling):

```
latitude  → int(lat × 10⁷)   [1/10 microdegree resolution]
longitude → int(lon × 10⁷)
altitude  → int(alt × 100)    [cm resolution]
```

### Geodetic to ENU (Inbound CAM)

Received CAM positions are in integer-scaled geodetic. To make them usable within the ADAPT coordinate system:

```python
# Decode from ETSI integer format
latitude  = cam_lat  × 1e-7
longitude = cam_lon  × 1e-7
altitude  = cam_alt  × 1e-2

# Convert to local ENU
East, North, Up = pm.geodetic2enu(lat, lon, alt, LAT0, LON0, ALT0)
```

The ENU values are stored in `VehData.x`, `.y`, `.z` respectively.

### Yaw to WGS-84 Heading (Outbound CAM)

The ego vehicle's yaw from `/euler_angles` is in radians (ROS convention: counter-clockwise from East). ETSI CAM heading is in degrees clockwise from North (WGS-84 convention, 0–3600 × 0.1°).

The conversion logic handles four quadrants:

| Yaw range (after 360° wrap) | WGS-84 heading formula |
|---|---|
| 0° – 180° | `(180 − yaw) + 90` |
| 180° – 270° | `(360 − yaw) − 90` |
| 270° – 360° | `(360 − yaw) + 270` |

The result is multiplied by 10 before insertion into the CAM heading field (`heading_value = int(wgs84_yaw × 10)`).

### Heading to Quaternion (Inbound CAM)

Received WGS-84 heading (degrees) is converted back to a ROS quaternion for use in `VehData`:

```python
yaw_rad = math.radians(heading_degrees / 10)
x, y, z, w = quaternion_from_euler(yaw_rad)
# roll = 0, pitch = 0 (2D planar motion assumed)
```

---

## Reference Coordinates

The ENU ↔ Geodetic conversion is anchored to a fixed reference point representing the origin of the ADAPT Model City test environment:

| Parameter | Value | Description |
|---|---|---|
| `LAT0` | 50.24132213367954° | Reference latitude (Coburg, Germany area) |
| `LON0` | 11.321265180951718° | Reference longitude |
| `ALT0` | 0.0 m | Reference altitude |

These values are hardcoded in `transceiver.py`. Any change to the physical installation position of the MoCap system would require updating these constants.

---

## Object Class Mapping

`CPM.py` maps DetectNet internal class IDs to ETSI CPM object class codes using a lookup dictionary. The mapping also matches the classes used by the ADAPT Object Detection model:

| ETSI CPM Class ID | Class Name | DetectNet ID |
|---|---|---|
| 1 | Background | 0 |
| 2 | Car | 1 |
| 3 | Traffic Light | 2 |
| 4 | Potted Plant | 3 |
| 5 | Person | 4 |

The mapping is also documented in `adapt_transceiver/object_classes.txt`.

The conversion from DetectNet result ID to CPM class ID is:

```python
object_class_id = ord(msg.detections[i].results[j].id) + 1
```

> **Note:** This uses `ord()` on the DetectNet result ID string, implying the ID is a single ASCII character. Class 0 (Background) maps to CPM class 1; unknown IDs are assigned `vehicle_sub_class = 0`.

---

## Package Structure

```
transceiver/
├── adapt_transceiver/
│   ├── __init__.py
│   ├── transceiver.py          # Transceiver node: CAM TX/RX + EV location publisher
│   ├── CPM.py                  # CpmPublisher node: DetectNet → CPM encoder
│   └── object_classes.txt      # Human-readable object class index reference
├── dummy_data                  # Manual test commands for ros2 topic pub
├── images/
│   ├── tr.png                  # Transceiver architecture diagram
│   └── transmitter.png         # Transmitter block diagram
├── launch/
│   └── transmitter_launch.py   # Launch file (references outdated package/executable names)
├── resource/
│   └── adapt_transceiver       # Ament resource index marker
├── test/
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
| `geometry_msgs` | `PoseStamped`, `Vector3` |
| `std_msgs` | `String` (imported but unused in current code) |
| `adapt_msgs` | `VehData` — custom ADAPT message for vehicle data on `/ev_location` |
| `v2x` | ETSI ITS V2X message definitions: `CAM`, `CPM`, `ItsPduHeader`, `CoopAwareness`, `CamParameters`, `BasicContainer`, `ReferencePosition`, `Altitude`, `PerceivedObject`, `ObjectClassWithConfidence` |
| `vision_msgs` | `Detection2DArray` — DetectNet detection results for CPM encoding |
| `ros2launch` | Launch system support |

### Python

| Package | Purpose |
|---|---|
| `pymap3d` | ENU ↔ Geodetic coordinate conversion (`enu2geodetic`, `geodetic2enu`) |
| `numpy` | Quaternion-from-Euler computation |
| `math` | Degree/radian conversion, heading quadrant logic |

### External

| Dependency | Source | Purpose |
|---|---|---|
| `v2x` ROS 2 package | [git.hs-coburg.de/Autonomous_Driving/v2x](https://git.hs-coburg.de/Autonomous_Driving/v2x.git) | ETSI ITS message type definitions for CAM, CPM, and related sub-messages. Must be built before `adapt_transceiver`. |

### Build System

- **ROS 2 Foxy** (or compatible)
- **ament_python** build type

---

## Installation

1. Clone and build the `v2x` messages package first (it is a required dependency):

   ```bash
   cd ~/ros2_ws/src
   git clone https://git.hs-coburg.de/Autonomous_Driving/v2x.git
   cd ~/ros2_ws
   colcon build --packages-select v2x
   source install/setup.bash
   ```

2. Place `adapt_transceiver` in the workspace:

   ```bash
   cd ~/ros2_ws/src
   # copy or clone adapt_transceiver here
   ```

3. Install Python dependencies:

   ```bash
   pip install pymap3d numpy --break-system-packages
   ```

4. Install ROS dependencies:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. Build:

   ```bash
   colcon build --packages-select adapt_transceiver
   source install/setup.bash
   ```

---

## Running the Nodes

### Start the Transceiver node (CAM TX/RX + EV location)

```bash
ros2 run adapt_transceiver transceiver_node
```

### Start the CPM publisher (DetectNet → CPM)

```bash
ros2 run adapt_transceiver cpm
```

### Verify output

```bash
# Watch outbound CAM messages
ros2 topic echo /cam_msgs

# Watch decoded EV locations from other vehicles
ros2 topic echo /ev_location

# Watch outbound CPM messages
ros2 topic echo /detected_objects
```

---

## Testing with Dummy Data

The `dummy_data` file contains `ros2 topic pub` commands for injecting test inputs without the full ADAPT stack:

**Simulate ego vehicle location:**
```bash
ros2 topic pub /loc_pose geometry_msgs/PoseStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, \
    pose: {position: {x: 25.0, y: 249.0, z: 50.0}, \
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

**Simulate selected parking spot:**
```bash
ros2 topic pub /spot_location geometry_msgs/PoseStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, \
    pose: {position: {x: 64.5, y: 58.20, z: 61.78}, \
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

**Simulate user info string:**
```bash
ros2 topic pub /user_info std_msgs/String "data: 'Juno Muller; AUDI R8; 4XYZ28'"
```

With `/loc_pose` injected, the transceiver will begin publishing CAMs at 2 Hz on `/cam_msgs`. Any other ROS 2 node (or a second terminal with `ros2 topic echo /cam_msgs`) can receive them to verify the encoding.

---


## Integration with ADAPT

| Component | Relationship |
|---|---|
| **Localisation (`adapt_loc`)** | Provides `/loc_pose` (ENU position) and `/euler_angles` (yaw) used to encode outbound CAM messages with the ego vehicle's current geodetic position and WGS-84 heading. |
| **Object Detection (`adapt_obj`)** | The `CpmPublisher` subscribes to `/detectnet/detections` from DetectNet and encodes them as CPM messages for broadcast. |
| **Environment Model (`adapt_envmod`)** | The `Environment` (Visualization) node subscribes to `/ev_location` to render other vehicles in RViz. The `VehData` messages published by the transceiver are the input for other-vehicle TF transforms and 3D mesh markers. |
| **V2X Network / Infrastructure Transceiver** | The counterpart infrastructure transceiver encodes and transmits EVCSNs (parking spot availability). The ADAPT transceiver is designed to receive these, though EVCSN decoding is not yet implemented. |
| **Spot Selector / Behaviour Planning** | Would consume the decoded EVCSN parking spot data once the EVCSN inbound path is implemented. |

---


## License

Apache License 2.0 — see [LICENSE](LICENSE) for details.