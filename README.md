# Work in Autonomous Driving

> A curated collection of modules and components developed as part of the **ADAPT** system, an infrastructure-based, end-to-end autonomous parking solution built at Hochschule Coburg (HS Coburg).

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Foxy-brightgreen)](https://docs.ros.org/en/foxy/Installation.html)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04-orange)](https://ubuntu.com/)
[![Build](https://img.shields.io/badge/Build-colcon-yellow)](https://colcon.readthedocs.io/)
[![V2X](https://img.shields.io/badge/V2X-ETSI%20ITS-red)]()
[![Deep Learning](https://img.shields.io/badge/Deep%20Learning-DetectNet-76b900)](https://github.com/dusty-nv/jetson-inference)
[![OptiTrack](https://img.shields.io/badge/Localisation-OptiTrack%20MoCap-lightblue)]()
[![LiDAR](https://img.shields.io/badge/Sensor-LiDAR-informational)]()
[![RealSense](https://img.shields.io/badge/Camera-Intel%20RealSense-0071C5)](https://github.com/IntelRealSense/realsense-ros)
[![A*](https://img.shields.io/badge/Planning-A%2A%20Algorithm-9cf)]()
[![Pure Pursuit](https://img.shields.io/badge/Control-Pure%20Pursuit-ff69b4)]()
[![CAN Bus](https://img.shields.io/badge/Actuation-CAN%20Bus-critical)]()
[![Jetson](https://img.shields.io/badge/Hardware-NVIDIA%20Jetson-76b900)](https://developer.nvidia.com/embedded/jetson-modules)
[![Roboflow](https://img.shields.io/badge/Dataset-Roboflow-purple)](https://roboflow.com)
[![YASMIN](https://img.shields.io/badge/FSM-YASMIN-blueviolet)](https://github.com/uleroboticsgroup/yasmin)
[![SciPy](https://img.shields.io/badge/Math-SciPy-8CAAE6)](https://scipy.org)
[![NumPy](https://img.shields.io/badge/Math-NumPy-013243)](https://numpy.org)


---

## Table of Contents

- [About This Repository](#about-this-repository)
- [The Core Question](#the-core-question)
- [What is ADAPT?](#what-is-adapt)
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Modules in This Repository](#modules-in-this-repository)
  - [Localisation](#-localisation)
  - [Environment Model](#-environment-model)
  - [Object Detection](#-object-detection)
  - [Messages](#-messages)
  - [Route Computer](#-route-computer)
  - [Trajectory Planner](#-trajectory-planner)
  - [Control](#-control)
  - [Transceiver](#-transceiver)
  - [Main Project](#-main-project)
- [Full ROS 2 Topic Map](#full-ros-2-topic-map)
- [Tech Stack](#tech-stack)
- [Software Dependencies](#software-dependencies)
- [Hardware Dependencies](#hardware-dependencies)
- [Installation](#installation)
- [Running the System](#running-the-system)
- [UML Diagrams](#uml-diagrams)
- [License](#license)
- [Get in Touch](#get-in-touch)

---

## About This Repository

This repository showcases the modules I built or contributed to as part of the **ADAPT** autonomous parking project. ADAPT is a full-stack system where a model-scale ego vehicle communicates with smart infrastructure over a V2X network to autonomously locate, navigate to, and park at the nearest available parking spot — with zero human input after the initial preferences are set.

> **Note:** This is a curated subset of the full ADAPT system. Some components (Behaviour Planning, Spot Selector, Vehicle Interface, Mobile Interface, and Infrastructure components) are proprietary or hosted separately. The modules here represent the core perception, planning, control, and communication stack.

---

## The Core Question

> *How can we design an infrastructure-based end-to-end parking solution for all people arriving with an Autonomous Vehicle into a covered urban area, to eliminate the distance travelled and effort required for parking, and reduce emissions, by integrating leading-edge technologies in communication, sensing infrastructure, and autonomous driving?*

---

## What is ADAPT?

**ADAPT (Autonomous Driving Platform and Test)** is a complete autonomous parking system deployed on a model-scale vehicle in a physical **Model City** test environment at HS Coburg. The system consists of two independently deployable halves:

**Ego Vehicle** — the autonomous vehicle stack running on an NVIDIA Jetson platform, covering Sense → Plan → Act.

**Infrastructure** — a fixed server installation monitoring parking spot occupancy and broadcasting availability over V2X.

Both sides communicate using **ETSI ITS-compliant V2X messages** (CAM, CPM, EVCSN). Ground-truth localisation is provided by an **OptiTrack Motion Capture** system, eliminating the need for GPS in the indoor Model City environment.

### Use Case

A user arrives at their destination and enters their name, licence plate, and parking zone preference on the **Vehicle Interface**. The ego vehicle then:

1. Requests available parking spots from the infrastructure via V2X (EVCSN).
2. Selects the optimal spot based on user preferences.
3. Computes an A\*-based route through the Model City map.
4. Smooths the route using cubic spline interpolation.
5. Drives autonomously via Pure Pursuit path tracking.
6. Executes a five-phase geometric parking manoeuvre (two arcs + three straight segments).
7. Parks in the selected spot — all without further user input.

---

## System Architecture

```
┌───────────────────────────────────────────────────────────────────────┐
│                           EGO VEHICLE                                  │
│                                                                        │
│  SENSE                    PLAN                          ACT            │
│  ┌──────────────┐  ┌──────────────────────────┐  ┌──────────────────┐ │
│  │ Localisation │  │ Spot Selector            │  │ Lat/Long Control │ │
│  │ (MoCap)      │─▶│ Environment Model        │─▶│ (Pure Pursuit)   │ │
│  │ Object Det.  │  │ Route Computer (A*)      │  │                  │ │
│  │ (DetectNet   │  │ Trajectory Planner       │  │ Transceiver      │ │
│  │ + LiDAR)     │  │ Behaviour Planning (FSM) │  │ (CAM / CPM)      │ │
│  └──────────────┘  └──────────────────────────┘  └──────────────────┘ │
└────────────────────────────────┬──────────────────────────────────────┘
                                 │  V2X  (CAM, CPM, EVCSN — ETSI ITS G5)
┌────────────────────────────────┴──────────────────────────────────────┐
│                         INFRASTRUCTURE                                  │
│                                                                        │
│  SENSE                    PLAN                          ACT            │
│  ┌──────────────┐  ┌──────────────────────────┐  ┌──────────────────┐ │
│  │ Parking Spot │  │ Spot Updater             │  │ Infra Transceiver│ │
│  │ OD (inf model│─▶│ Spot Filter              │─▶│ (EVCSN TX)       │ │
│  │ free/occupied│  │                          │  │                  │ │
│  └──────────────┘  └──────────────────────────┘  └──────────────────┘ │
└───────────────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
work-in-autonomous-driving/
│
├── localisation/         # Real-time MoCap-based ego vehicle localisation
├── environment_model/    # LiDAR scan processing, stop detection, V2X visualization
├── object_detection/     # DetectNet camera models + 5,000-image annotated dataset
├── messages/             # Custom ROS 2 message definitions (adapt_msgs)
├── route_computer/       # A* pathfinding over the Model City directed graph
├── trajectory/           # Cubic spline smoothing + geometric parking manoeuvre
├── control/              # Pure Pursuit lateral/longitudinal controller
├── transceiver/          # V2X communication node (CAM TX/RX, CPM TX)
└── main_project/         # Top-level launch files + full system integration
```

---

## Modules in This Repository

---

### 📍 Localisation

**Package:** `adapt_loc` | **Path:** `localisation/`

Provides precise real-time 6DOF pose of the ego vehicle from the **OptiTrack Motion Capture** system. Filters the rigid body stream for the ego vehicle (ID `"9"`), converts quaternion orientation to Euler angles, and publishes both.

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/pose_modelcars` | Input | `mocap_msgs/RigidBodies` | Raw MoCap rigid body stream for all tracked objects. |
| `/loc_pose` | Output | `geometry_msgs/PoseStamped` | Ego vehicle 6DOF pose, frame `base_link_7`. |
| `/euler_angles` | Output | `geometry_msgs/Vector3` | Roll, pitch, yaw in radians. Z (yaw) used by Route Computer and Transceiver. |

**Key features:** Quaternion-to-Euler conversion (ZYX decomposition), rigid body filtering by name, position rounded to 4 decimal places (0.1 mm precision).

📁 [Detailed README →](localisation/README.md)

---

### 🌍 Environment Model

**Package:** `adapt_envmod` | **Path:** `environment_model/`

Two-node package fusing LiDAR perception and V2X data. The **EnvModel** node processes LiDAR scans with TF2 coordinate transforms to detect nearby obstacles and publish a stop signal. The **Environment** (Visualization) node broadcasts TF transforms and 3D mesh markers for all V2X-connected vehicles in RViz.

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/scan` | Input | `sensor_msgs/LaserScan` | 2D LiDAR scan (BEST_EFFORT QoS). |
| `/loc_pose` | Input | `geometry_msgs/PoseStamped` | Ego vehicle pose. |
| `/ev_location` | Input | `adapt_msgs/VehData` | Other vehicles' positions from V2X. |
| `/scans` | Output | `adapt_msgs/DetectedObjects` | Obstacles in the `9/base_link` frame (distance + angle). |
| `/stop` | Output | `std_msgs/Bool` | `true` when an obstacle is within `stop_range` (default 0.95 m). RELIABLE + TRANSIENT_LOCAL. |
| `/tf_{id}` | Output | `geometry_msgs/TransformStamped` | Per-vehicle TF from `map` to `car_{id}`. |
| `/car_marker_{id}` | Output | `visualization_msgs/Marker` | 3D mesh marker for each vehicle. |

**Key features:** Angular filtering (±0.5 rad forward window), TF2 laser→base_link transform, configurable `stop_range` and `angle_start` parameters, 5 rosbag recordings included.

📁 [Detailed README →](environment_model/README.md)

---

### 👁️ Object Detection

**Package:** `adapt_obj` | **Path:** `object_detection/`

Camera-based object detection using **NVIDIA DetectNet** on an **Intel RealSense** camera. Contains two trained models and a 5,000-image annotated dataset.

**Vehicle Detection Model** — detects obstacles on the road:

| Class | Represents |
|---|---|
| `car` | Model cars |
| `person` | Pedestrian figures |
| `potted plant` | Roadside vegetation props (COCO label for tree stand-ins) |
| `traffic light` | Road furniture |
| `building` | Roadside structures |

**Infrastructure (inf) Model** — parking spot occupancy classification:

| Class | Meaning |
|---|---|
| `free` | Parking spot is available |
| `occupied` | Parking spot is taken |

**Dataset:** 5,000 annotated images in Pascal VOC XML format, managed via Roboflow (Roboflow project: `Adapt_detectnet`, v4, 640×640).

| Split | Images |
|---|---|
| Train | 3,500 |
| Valid | 1,000 |
| Test | 500 |

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/camera/color/image_raw` | Input | `sensor_msgs/Image` | Intel RealSense RGB frames. |
| `/detectnet/detections` | Output | `vision_msgs/Detection2DArray` | Bounding boxes, class IDs, confidence scores. |

📁 [Detailed README →](object_detection/README.md)

---

### 📨 Messages

**Package:** `adapt_msgs` | **Path:** `messages/`

Central repository for all custom ROS 2 message types shared across ADAPT components. Must be built before any other package.

| Message | Used between | Description |
|---|---|---|
| `VehData` | Transceiver → Environment Model | Full vehicle state: geodetic coordinates, ENU Cartesian position, quaternion orientation, vehicle ID. |
| `CarCom` | Behaviour Planning → Control | Actuator command: linear velocity, angular velocity, steering angle. |
| `DetectedObject` | (element of `DetectedObjects`) | Single obstacle: distance (m) and angle (rad). |
| `DetectedObjects` | Environment Model → Behaviour Planning | Stamped array of detected obstacles. |
| `LiveTrack` | Live Tracker → Mobile Interface | Live pose + status string (`Moving`/`Parked`). |
| `LaneInfo` | Lane detection → Controller | Lane geometry: confidence, type, curvature, heading, pixel centre. |

📁 [Detailed README →](messages/README.md)

---

### 🗺️ Route Computer

**Package:** `adapt_roucomp` | **Path:** `route_computer/`

Computes the optimal path from the ego vehicle's current position to the selected parking spot using a **cardinal-direction-aware A\*** algorithm over a preloaded directed graph of the Model City (33 nodes, 75 directed edges, 4 parking spots).

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/loc_pose` | Input | `geometry_msgs/PoseStamped` | Current vehicle position. |
| `/selected_spot` | Input | `geometry_msgs/PoseStamped` | Target parking spot (with −1.0, −1.5 m calibration offset applied). |
| `/euler_angles` | Input | `geometry_msgs/Vector3` | Vehicle yaw for initial heading direction. |
| `/route` | Output | `geometry_msgs/PoseArray` | Ordered waypoints in the `map` frame. Published once per request. |
| `/route_state` | Output | `std_msgs/Bool` | `true` once a route has been computed. |

**Algorithm highlights:**
- Directed graph with N/S/E/W edge labels, loaded from `config/map_cardinal.txt`.
- Turning penalties: straight = 0, 90° turn = 5, U-turn = 10.
- Euclidean distance heuristic.
- Yaw-to-direction seeding (4 quadrant mapping).
- Performance metrics logged to `route_planning_metrics.csv` on every run.
- 6 rosbag recordings covering origin, crossing, diagonal, and reverse scenarios.

📁 [Detailed README →](route_computer/README.md)

---

### 🛤️ Trajectory Planner

**Package:** `adapt_trajp` | **Path:** `trajectory/`

Transforms raw route waypoints into smooth, physically drivable trajectories and computes the geometric parking manoeuvre path. Three trajectory output modes, each triggered by a Boolean control signal from Behaviour Planning:

| Mode | Trigger | Output Topic | Description |
|---|---|---|---|
| Drive | `/drive=true` | `/trajd` | 50-point cubic spline at constant 1.0 m/s. |
| Stop | `/drive=false` | `/trajs` | Zero-velocity single point. |
| Forward Park | `/park=true` | `/trajpf` | Phases 1–3 of the parking manoeuvre (+1.0 m/s). |
| Reverse Park | `/park_reverse=true` | `/trajpr` | Phases 4–5 of the parking manoeuvre (−1.0 m/s). |

**Drive trajectory:** cubic spline via `scipy.interpolate.CubicSpline`, arc-length parametrised, resampled to 50 points.

**Parking manoeuvre geometry** (two tangent circles of radius 1.2 m, tangent line at θ = 30°):

| Phase | Type | Direction |
|---|---|---|
| 1 — Straight approach | Line | Forward |
| 2 — Alignment arc | Arc (r = 1.2 m) | Forward |
| 3 — Tangent line | Line | Forward |
| 4 — Reverse arc into spot | Arc (r = 1.2 m) | Reverse |
| 5 — Final straight | Line | Reverse |

Also includes `dummy_path.py` (publishes a hardcoded 14-point test route) and `dummy_state.py` (configurable Bool publisher) for isolated testing without the full stack.

📁 [Detailed README →](trajectory/README.md)

---

### 🎮 Control

**Package:** `adapt_latlongcon` | **Path:** `control/`

Implements a **Pure Pursuit path-tracking controller** operating at 100 Hz. Converts `JointTrajectory` messages from the Trajectory Planner into `Twist` actuator commands, managing all three driving modes and emergency stop.

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/trajd` | Input | `trajectory_msgs/JointTrajectory` | Drive trajectory. |
| `/trajpf` | Input | `trajectory_msgs/JointTrajectory` | Forward parking trajectory. |
| `/trajpr` | Input | `trajectory_msgs/JointTrajectory` | Reverse parking trajectory. |
| `/stop` | Input | `std_msgs/Bool` | Emergency stop — overrides all other modes. |
| `/loc_pose` | Input | `geometry_msgs/PoseStamped` | Current vehicle pose. |
| `/cmd_vel` | Output | `geometry_msgs/Twist` | Linear and angular velocity commands to actuators. |
| `/reach_goal` | Output | `std_msgs/Bool` | `true` when within 0.3 m of the drive goal. |
| `/park_reverse` | Output | `std_msgs/Bool` | `true` when forward parking alignment is complete. |

**Pure Pursuit algorithm:**
1. Find nearest waypoint (vectorised NumPy distance computation).
2. Advance to lookahead waypoint (`look_ahead_distance` = 1.2 m, configurable).
3. Compute heading error α = `atan2(Δy, Δx) − yaw`.
4. Steering angle δ = `atan2(2 · L · sin(α), lookahead)` where L = 0.5 m (wheelbase).
5. Clip to [−30°, 30°].

**Velocity per mode:** 0.5 m/s (drive), 0.6 m/s (forward park), −0.8 m/s (reverse park).

**Parameter:** `look_ahead_distance` (default 1.2 m) — override with `--ros-args -p look_ahead_distance:=1.5`.

📁 [Detailed README →](control/README.md)

---

### 📡 Transceiver

**Package:** `adapt_transceiver` | **Path:** `transceiver/`

The V2X communication bridge for the ego vehicle. Two nodes handle distinct responsibilities:

**`Transceiver` node** — CAM TX/RX + EV location:
- Publishes ego vehicle CAM at 2 Hz (station ID = 9), with ENU→Geodetic conversion via `pymap3d.enu2geodetic` and yaw→WGS-84 heading conversion.
- Subscribes to all `/cam_msgs`, filters self (station ID = 9), converts received CAMs from geodetic to ENU via `pymap3d.geodetic2enu`, and re-publishes as `VehData` on `/ev_location` at 1 Hz.

**`CpmPublisher` node** — DetectNet → CPM:
- Subscribes to `/detectnet/detections` and encodes each detection as an ETSI CPM perceived object with ETSI class codes and confidence scores.

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/loc_pose` | Input | `geometry_msgs/PoseStamped` | ENU pose for CAM encoding. |
| `/euler_angles` | Input | `geometry_msgs/Vector3` | Yaw for WGS-84 heading. |
| `/detectnet/detections` | Input | `vision_msgs/Detection2DArray` | Object detections for CPM. |
| `/cam_msgs` | Input + Output | `v2x/CAM` | Shared V2X CAM topic (self-filtered). |
| `/ev_location` | Output | `adapt_msgs/VehData` | Other vehicles' positions decoded from CAM. |
| `/detected_objects` | Output | `v2x/CPM` | Detected objects encoded as ETSI CPM. |

**Reference coordinates** (Model City, Coburg): `LAT0 = 50.24132213367954°`, `LON0 = 11.321265180951718°`.

📁 [Detailed README →](transceiver/README.md)

---

### 🏗️ Main Project

**Path:** `main_project/`

Top-level integration repository containing all launch files and the `vcstool` repository manifest for cloning the full ADAPT workspace.

**Launch files:**

| File | Nodes started | Purpose |
|---|---|---|
| `adapt_launch.py` | 12 + car model | Full EV autonomous driving stack |
| `infra_launch.py` | 3 | Infrastructure parking management stack |
| `backend_components.py` | 6 | Sensor, localisation, and comms backend |

**`adapt_launch.py` starts:**

| Node | Package | Role |
|---|---|---|
| `vi` | `adapt_vi` | Vehicle Interface |
| `spotsl` | `adapt_spotsl` | Spot Selector |
| `localization` | `adapt_loc` | Localisation |
| `routemodule5` | `adapt_roucomp` | Route Computer |
| `trajectory_planner` | `adapt_trajp` | Trajectory Planner |
| `envmod` + `mapping` | `adapt_envmod` | Environment Model + Visualisation |
| `path_tracking` | `adapt_latlongcon` | Lateral/Longitudinal Control |
| `beh` | `adapt_behplan` | Behaviour Planning (FSM) |
| `adaptmi` | `adapt_mobint` | Mobile Interface |
| `transceiver` + `CPM` | `adapt_transceiver` | V2X Transceiver |

📁 [Detailed README →](main_project/README.md)

---

## Full ROS 2 Topic Map

Key data flows across all included components:

| Topic | Type | Producer | Consumer(s) |
|---|---|---|---|
| `/pose_modelcars` | `mocap_msgs/RigidBodies` | OptiTrack driver | Localisation |
| `/loc_pose` | `geometry_msgs/PoseStamped` | Localisation | Route Computer, Trajectory, Control, Transceiver, Env. Model |
| `/euler_angles` | `geometry_msgs/Vector3` | Localisation | Route Computer, Trajectory, Transceiver |
| `/scan` | `sensor_msgs/LaserScan` | YDLidar driver | Environment Model |
| `/detectnet/detections` | `vision_msgs/Detection2DArray` | DetectNet | Transceiver (CPM) |
| `/selected_spot` | `geometry_msgs/PoseStamped` | Spot Selector | Route Computer, Trajectory Planner |
| `/cam_msgs` | `v2x/CAM` | Transceiver | Transceiver (self-filtered) |
| `/ev_location` | `adapt_msgs/VehData` | Transceiver | Environment Model (Visualisation) |
| `/scans` | `adapt_msgs/DetectedObjects` | Environment Model | Behaviour Planning |
| `/stop` | `std_msgs/Bool` | Environment Model | Behaviour Planning, Control |
| `/route` | `geometry_msgs/PoseArray` | Route Computer | Trajectory Planner, Behaviour Planning |
| `/route_state` | `std_msgs/Bool` | Route Computer | Behaviour Planning |
| `/drive` | `std_msgs/Bool` | Behaviour Planning | Trajectory Planner |
| `/park` | `std_msgs/Bool` | Behaviour Planning | Trajectory Planner |
| `/park_reverse` | `std_msgs/Bool` | Control / Behaviour Planning | Trajectory Planner |
| `/trajd` | `trajectory_msgs/JointTrajectory` | Trajectory Planner | Control |
| `/trajpf` | `trajectory_msgs/JointTrajectory` | Trajectory Planner | Control |
| `/trajpr` | `trajectory_msgs/JointTrajectory` | Trajectory Planner | Control |
| `/cmd_vel` | `geometry_msgs/Twist` | Control | Vehicle actuators (via CAN bus) |
| `/reach_goal` | `std_msgs/Bool` | Control | Behaviour Planning |

---

## Tech Stack

| Category | Technology |
|---|---|
| Language | Python 3.8+, CMake |
| Middleware | ROS 2 Foxy |
| OS | Ubuntu 20.04 LTS |
| Localisation | OptiTrack MoCap (via `mocap_optitrack`) |
| Camera | Intel RealSense D series |
| LiDAR | YDLidar (via `ydlidar_ros2`) |
| V2X Communication | ETSI ITS G5 — CAM, CPM, EVCSN (via `v2x` ROS msgs) |
| Deep Learning | NVIDIA DetectNet (`ros_deep_learning`) on NVIDIA Jetson |
| Path Planning | A\* with cardinal direction constraints + turning penalties |
| Path Smoothing | Cubic spline interpolation (`scipy.interpolate.CubicSpline`) |
| Path Tracking | Pure Pursuit controller |
| Actuation | CAN bus via `ros2_pcan` |
| FSM | YASMIN (`uleroboticsgroup/yasmin`) |
| Annotation | Roboflow (Pascal VOC XML) |
| Visualisation | RViz, `rqt_graph` |

---

## Software Dependencies

| Dependency | Purpose | Source |
|---|---|---|
| ROS 2 Foxy | Middleware | [Install Guide](https://docs.ros.org/en/foxy/Installation.html) |
| `mocap_optitrack` | OptiTrack MoCap ROS driver | [GitHub](https://github.com/ros-drivers/mocap_optitrack) |
| `v2x` | ETSI ITS V2X message definitions | [HS Coburg Git](https://git.hs-coburg.de/Autonomous_Driving/v2x.git) |
| `ros2_pcan` | CAN bus interface for actuators | [HS Coburg Git](https://git.hs-coburg.de/Autonomous_Driving/ros2_pcan.git) |
| `ros_deep_learning` | NVIDIA DetectNet ROS 2 node | [HS Coburg Git](https://git.hs-coburg.de/Autonomous_Driving/ros_deep_learning) |
| `realsense2_camera` | Intel RealSense ROS 2 driver | [GitHub](https://github.com/IntelRealSense/realsense-ros) |
| `ydlidar_ros2` | YDLidar ROS 2 driver | [HS Coburg Git](https://git.hs-coburg.de/Autonomous_Driving/ydlidar_ros2.git) |
| `yasmin` | FSM library for Behaviour Planning | [GitHub](https://github.com/uleroboticsgroup/yasmin) |
| `nav2_bringup` | Navigation 2 stack | [GitHub](https://github.com/open-navigation/navigation2) |
| `car_description` | Vehicle URDF/mesh for RViz | [HS Coburg Git](https://git.hs-coburg.de/Autonomous_Driving/car_description.git) |
| `scipy` | Cubic spline interpolation | `pip install scipy` |
| `pymap3d` | ENU ↔ Geodetic coordinate conversion | `pip install pymap3d` |
| `numpy` | Numerical operations | `pip install numpy` |

---

## Hardware Dependencies

| Hardware | Purpose |
|---|---|
| **OptiTrack Motion Capture System** | Sub-millimetre ground-truth localisation of all vehicles in the Model City. Provides pose data via `/pose_modelcars`. |
| **Intel RealSense Camera** | RGB camera on the ego vehicle for DetectNet-based obstacle detection. Also used on the infrastructure for parking spot occupancy classification. |
| **YDLidar** | 2D LiDAR on the ego vehicle providing `/scan` for the Environment Model's stop detection. |
| **NVIDIA Jetson** | On-board GPU platform running the full EV stack including real-time DetectNet inference. |
| **PCAN / CAN Bus Interface** | Hardware bridge between the ROS 2 control stack and the vehicle's drive motor and steering actuators. |

---

## Installation

### 1. Clone this repository

```bash
git clone https://github.com/sca7r/work-in-autonomous-driving.git
cd work-in-autonomous-driving
```

### 2. Set up a ROS 2 workspace and import all dependencies

If you want to build the full ADAPT system using the included `adapt_repos.repo` file:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
cp -r /path/to/work-in-autonomous-driving/* .
# Import all ADAPT repositories
vcs import < main_project/adapt_repos.repo
```

### 3. Build dependency packages first

`adapt_msgs` and `v2x` must be built before all other packages:

```bash
cd ~/ros2_ws
colcon build --packages-select adapt_msgs v2x
source install/setup.bash
```

### 4. Install ROS dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Install Python dependencies

```bash
pip install scipy pymap3d numpy --break-system-packages
```

### 6. Build the full workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## Running the System

### Prerequisites — start hardware drivers first

```bash
# 1. OptiTrack MoCap driver (provides /pose_modelcars)
ros2 launch mocap_optitrack mocap.launch.py

# 2. Intel RealSense camera (provides /camera/color/image_raw)
ros2 launch realsense2_camera rs_launch.py

# 3. NVIDIA DetectNet (provides /detectnet/detections)
ros2 launch ros_deep_learning detectnet.ros2.launch

# 4. YDLidar (provides /scan)
ros2 launch ydlidar_ros2 ydlidar_launch.py
```

### Launch the Ego Vehicle stack

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch adapt_launch.py
```

### Launch the Infrastructure stack (on the infrastructure server)

```bash
ros2 launch infra_launch.py
```

### Running individual modules for development/testing

```bash
# Localisation
ros2 launch adapt_loc loc_launch.py

# Environment Model
ros2 launch adapt_envmod env_launch.py

# Route Computer (cardinal A* node)
ros2 run adapt_roucomp route_cardinal

# Trajectory Planner
ros2 run adapt_trajp traj

# Control
ros2 run adapt_latlongcon pp

# Transceiver (CAM TX/RX + EV location)
ros2 run adapt_transceiver transceiver_node

# CPM publisher (DetectNet → CPM)
ros2 run adapt_transceiver cpm
```

### Isolated trajectory testing (no full stack required)

```bash
# Terminal 1: Start trajectory planner
ros2 run adapt_trajp traj

# Terminal 2: Publish a test route
ros2 run adapt_trajp dum_path

# Terminal 3: Trigger drive trajectory
ros2 run adapt_trajp dum_state --topic /drive --state true
```

---

## UML Diagrams

All diagrams are in `main_project/images/`.

| Diagram | File | Description |
|---|---|---|
| Activity Diagram | `activity_diagram.png` | EV–Infrastructure interaction from user command to parking. |
| State Diagram | `State_Diagram.jpg` | Ego vehicle FSM: Idle → Drive → Obstacle Check → Park → Parked. |
| Sequence Diagram | `sequence_diagram.png` | Full message exchange timeline during parking. |
| EV Architecture | `ego-veihcle-v3.png` | Detailed ego vehicle component block diagram. |
| Infrastructure Architecture | `infra-archi.png` | Infrastructure component block diagram. |
| rqt Graph | `rqt_full.png` | Full ROS 2 node/topic graph. |

---

## License

This project is licensed under the **Apache License 2.0**, see the [LICENSE](LICENSE) file for details.

---

## Get in Touch

This repository is a curated subset of the full ADAPT system. If you're interested in:

- The complete codebase (including Behaviour Planning, Spot Selector, Vehicle Interface, Mobile Interface, and all Infrastructure components)
- Technical deep-dives on any module
- Collaboration or research discussions

Feel free to **open an issue** on [GitHub](https://github.com/sca7r/work-in-autonomous-driving) or reach out directly. Always open for discussions!