# ADAPT — Autonomous Driving And Parking Technoogy

> **How can we design an infrastructure-based end-to-end parking solution for all people arriving with an autonomous vehicle into a covered urban area, to eliminate the distance travelled and effort required for parking, and reduce emissions, by integrating leading-edge technologies in communication, sensing infrastructure, and autonomous driving?**

---

## Table of Contents

- [Overview](#overview)
- [Use Case](#use-case)
- [System Architecture](#system-architecture)
  - [Ego Vehicle](#ego-vehicle)
  - [Infrastructure](#infrastructure)
  - [V2X Communication Bridge](#v2x-communication-bridge)
- [Ego Vehicle Components](#ego-vehicle-components)
  - [Sense Layer](#sense-layer)
  - [Plan Layer](#plan-layer)
  - [Act Layer](#act-layer)
- [Infrastructure Components](#infrastructure-components)
  - [Sense Layer](#sense-layer-1)
  - [Plan Layer](#plan-layer-1)
  - [Act Layer](#act-layer-1)
- [Shared / Supporting Packages](#shared--supporting-packages)
- [Full ROS 2 Topic Map](#full-ros-2-topic-map)
- [UML Diagrams](#uml-diagrams)
- [Repository Structure](#repository-structure)
- [All Repositories](#all-repositories)
- [Software Dependencies](#software-dependencies)
- [Hardware Dependencies](#hardware-dependencies)
- [Installation](#installation)
- [Running ADAPT](#running-adapt)
- [Launch Files](#launch-files)

---

## Overview

**ADAPT** is a complete end-to-end autonomous parking system developed at Hochschule Coburg (HS Coburg). It enables a model-scale autonomous electric vehicle (EV) to navigate a physical **Model City** test environment, communicate with a smart parking infrastructure over a **V2X (Vehicle-to-Everything)** network, select an available parking spot based on user preferences, drive autonomously to that spot, and park itself.

The system is split into two independently deployable halves:

- **Ego Vehicle (EV)** — the autonomous vehicle stack running on an NVIDIA Jetson platform, responsible for perception, planning, and actuation.
- **Infrastructure** — the fixed-installation side, responsible for monitoring parking spot occupancy, publishing availability data over V2X, and updating the spot database as vehicles arrive and depart.

Both halves communicate over a V2X network using **ETSI ITS standardised messages** (CAM, CPM, EVCSN), bridged into the ROS 2 message bus by the respective transceiver components.

The Model City is instrumented with an **OptiTrack Motion Capture (MoCap)** system providing ground-truth localisation for the ego vehicle, eliminating the need for GPS in the indoor test environment.

---

## Use Case

**Scenario:** Parking Autonomously to the Nearest Feasible Parking Spot

| Property | Detail |
|---|---|
| **Main Actors** | Ego Vehicle, Infrastructure |
| **Preconditions** | Infrastructure is monitoring parking spots; user has arrived at their destination. |
| **Success Guarantee** | EV parked at the selected spot according to user preferences. |

**Steps:**
1. User enters their name, licence plate, and parking zone preference on the **Vehicle Interface (VI)**.
2. EV initiates V2X communication with the infrastructure requesting available parking spots.
3. Infrastructure responds with available spot data via an **EVCSN** message.
4. **Spot Selector** picks the optimal spot based on user preference and publishes the selection.
5. Infrastructure marks the chosen spot as unavailable.
6. **Route Computer** calculates the optimal path; **Trajectory Planner** smooths it.
7. **Behaviour Planning** drives the FSM transitions; **Lateral/Longitudinal Control** executes motion.
8. EV drives autonomously to the spot and parks using a geometric arc-and-line manoeuvre.
9. User monitors the vehicle in real time via the **Mobile Interface (MI)**.

---

## System Architecture

ADAPT is structured around the classic autonomous driving **Sense → Plan → Act** pipeline, applied independently to both the EV and the Infrastructure, with V2X as the inter-system communication channel.

```
┌─────────────────────────────────────────────────────────────────────┐
│                         EGO VEHICLE                                  │
│                                                                      │
│  SENSE                   PLAN                       ACT             │
│  ┌──────────┐   ┌──────────────────────────┐   ┌──────────────┐    │
│  │Localisa- │   │  Spot Selector           │   │ Lat/Long     │    │
│  │tion      │──▶│  Environment Model       │──▶│ Control      │    │
│  │(MoCap)   │   │  Route Computer          │   │              │    │
│  │Object    │   │  Trajectory Planner      │   │ Transceiver  │    │
│  │Detection │   │  Behaviour Planning      │   │ (CAM/CPM)    │    │
│  │(DetectNet│   │  Live Tracker            │   └──────────────┘    │
│  │+ LiDAR)  │   └──────────────────────────┘                       │
│  └──────────┘                                                       │
└─────────────────────────────────────┬───────────────────────────────┘
                                      │  V2X (CAM, CPM, EVCSN)
                                      │  ETSI ITS G5
┌─────────────────────────────────────┴───────────────────────────────┐
│                        INFRASTRUCTURE                                │
│                                                                      │
│  SENSE                   PLAN                       ACT             │
│  ┌──────────┐   ┌──────────────────────────┐   ┌──────────────┐    │
│  │Parking   │   │  Spot Updater            │   │ Infra        │    │
│  │Spot OD   │──▶│  Spot Filter             │──▶│ Transceiver  │    │
│  │(Inf OD)  │   │                          │   │ (EVCSN)      │    │
│  └──────────┘   └──────────────────────────┘   └──────────────┘    │
└─────────────────────────────────────────────────────────────────────┘
```

### Ego Vehicle

Runs the full autonomous driving stack. Deployed on the model-scale vehicle. Communicates inward via ROS 2 topics and outward via the V2X transceiver.

### Infrastructure

Runs the parking management stack. Deployed on a fixed server in the Model City environment. Monitors parking spot occupancy using its own object detection pipeline and publishes availability over V2X.

### V2X Communication Bridge

| Message | Standard | Direction | Purpose |
|---|---|---|---|
| **CAM** | ETSI EN 302 637-2 | EV → All | EV broadcasts its geodetic position, heading, and speed at 2 Hz. Infrastructure and other vehicles receive this to track the EV. |
| **CPM** | ETSI TR 103 562 | EV → All | EV shares its DetectNet-detected objects (cars, persons, traffic lights, vegetation) encoded as ETSI CPM perceived objects. |
| **EVCSN** | ETSI TS 101 556-1 | Infra → EV | Infrastructure broadcasts available parking spot list, including location, availability, and pricing zone. |

---

## Ego Vehicle Components

### Sense Layer

#### Vehicle Interface (`adapt_vi`)

The entry point for the user. Provides a two-frame graphical interface:
- **Frame 1:** ADAPT introduction with a `GO` button.
- **Frame 2:** Input form for name, licence plate, and parking zone preference (`Zone 1` — Paid/E-charging, `Zone 2` — Free, `Park Anywhere`). `START` begins the process; `CANCEL` aborts it.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Output | `/vi_start` | `String` | User ID, parking preference, and start command. |
| Output | `/vi_cancel` | `String` | Cancel command to abort the parking process. |

---

#### Mobile Interface (`adapt_mobint`)

A mobile application providing real-time live tracking and control of the ego vehicle. Lets the user monitor vehicle status and location after initiating autonomous parking.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/live_loc` | `adapt_msgs/LiveTrack` | Real-time vehicle position and status (Moving / Parked). |

---

#### Localisation (`adapt_loc`)

Provides precise real-time position and orientation of the ego vehicle using the **OptiTrack Motion Capture system**. Subscribes to the MoCap rigid body stream, filters for the ego vehicle (rigid body ID `"9"`), converts the quaternion orientation to Euler angles, and publishes both the full pose and Euler angles.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/pose_modelcars` | `mocap_msgs/RigidBodies` | Raw pose data for all rigid bodies from the OptiTrack system. |
| Output | `/loc_pose` | `geometry_msgs/PoseStamped` | Ego vehicle 6DOF pose in the `base_link_7` frame. |
| Output | `/euler_angles` | `geometry_msgs/Vector3` | Roll, pitch, yaw (radians). Z = yaw used by Route Computer and Transceiver. |

---

#### Object Detection (`adapt_obj`)

Provides camera-based object detection using **NVIDIA DetectNet** on an **Intel RealSense** camera. Trains and runs two separate DetectNet models:

- **Vehicle detection model** — detects cars, persons, potted plants (vegetation props), traffic lights, and buildings.
- **Infrastructure (inf) model** — classifies parking spots as `free` or `occupied` from a fixed overhead camera.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/camera/color/image_raw` | `sensor_msgs/Image` | RGB frames from the Intel RealSense camera. |
| Output | `/detectnet/detections` | `vision_msgs/Detection2DArray` | Bounding boxes, class IDs, and confidence scores. |
| Output | `/parking_status` | custom | Per-spot occupancy state for the infrastructure model. |

---

### Plan Layer

#### Spot Selector (`adapt_spotsl`)

Receives the available parking spot list from the infrastructure (via EVCSN decoded by the Transceiver) and the user's parking preference from the Vehicle Interface. Selects the optimal spot and publishes its coordinates.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/vi_start` | `String` | User preferences (free/paid parking zone). |
| Input | `/evcsn_msg` | `ItsEVCSNData` | Available parking spots with location and pricing from infrastructure. |
| Output | `/selected_spot` | `geometry_msgs/PoseStamped` | Location of the selected parking spot. |

---

#### Environment Model (`adapt_envmod`)

Fuses LiDAR scan data and the vehicle pose to build a real-time view of the surroundings. Publishes a stop signal when obstacles are detected within a configurable range, and publishes TF transforms and 3D mesh markers for all vehicles visible on the V2X network.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR scan. |
| Input | `/loc_pose` | `geometry_msgs/PoseStamped` | Ego vehicle position. |
| Input | `/ev_location` | `adapt_msgs/VehData` | Position and orientation of other V2X-connected vehicles. |
| Output | `/scans` | `adapt_msgs/DetectedObjects` | Transformed obstacle detections in the vehicle frame. |
| Output | `/stop` | `std_msgs/Bool` | `true` when an obstacle is within the stop range. |
| Output | `/tf_{id}` | `geometry_msgs/TransformStamped` | TF transform from `map` to each vehicle. |
| Output | `/car_marker_{id}` | `visualization_msgs/Marker` | 3D mesh marker for RViz visualisation per vehicle. |

---

#### Route Computer (`adapt_roucomp`)

Computes the optimal path from the ego vehicle's current position to the selected parking spot using the **A\* algorithm** over a preloaded directed graph of the Model City. The cardinal-direction-aware version (`route_cardinal`) applies turning penalties and uses the vehicle's yaw to seed the initial heading.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/loc_pose` | `geometry_msgs/PoseStamped` | Current vehicle position. |
| Input | `/selected_spot` | `geometry_msgs/PoseStamped` | Target parking spot coordinates. |
| Input | `/euler_angles` | `geometry_msgs/Vector3` | Vehicle yaw for initial heading direction. |
| Output | `/route` | `geometry_msgs/PoseArray` | Ordered waypoint sequence in the `map` frame. |
| Output | `/route_state` | `std_msgs/Bool` | `true` once a route has been computed successfully. |

---

#### Trajectory Planner (`adapt_trajp`)

Transforms the raw route waypoints into smooth, physically drivable trajectories using **cubic spline interpolation**, and computes the precise geometric parking manoeuvre path using circular arcs and line segments.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/route` | `geometry_msgs/PoseArray` | Raw waypoints from Route Computer. |
| Input | `/selected_spot` | `geometry_msgs/PoseStamped` | Parking spot for manoeuvre geometry. |
| Input | `/loc_pose` | `geometry_msgs/PoseStamped` | Current vehicle position. |
| Input | `/euler_angles` | `geometry_msgs/Vector3` | Vehicle yaw. |
| Input | `/drive` | `std_msgs/Bool` | `true` = publish smooth drive trajectory; `false` = publish stop. |
| Input | `/park` | `std_msgs/Bool` | `true` = publish forward parking trajectory. |
| Input | `/park_reverse` | `std_msgs/Bool` | `true` = publish reverse parking trajectory. |
| Output | `/trajd` | `trajectory_msgs/JointTrajectory` | 50-point cubic spline drive trajectory at 1.0 m/s. |
| Output | `/trajs` | `trajectory_msgs/JointTrajectory` | Zero-velocity stop command. |
| Output | `/trajpf` | `trajectory_msgs/JointTrajectory` | Forward parking trajectory (phases 1–3). |
| Output | `/trajpr` | `trajectory_msgs/JointTrajectory` | Reverse parking trajectory (phases 4–5). |
| Output | `/visualize` | `nav_msgs/Path` | RViz visualisation of the drive trajectory. |
| Output | `/visualize_forward_park` | `nav_msgs/Path` | RViz visualisation of forward parking path. |
| Output | `/visualize_reverse_park` | `nav_msgs/Path` | RViz visualisation of reverse parking path. |

---

#### Behaviour Planning (`adapt_behplan`)

The central decision-making component of the ego vehicle. Implemented as a **Finite State Machine (FSM)** using the `yasmin` library. Monitors topics from the environment model, route computer, and trajectory planner, and transitions between states (Idle, Drive, Park Forward, Park Reverse, Stop, etc.) by publishing Boolean control signals to the trajectory planner.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/route_state` | `std_msgs/Bool` | Route ready signal from Route Computer. |
| Input | `/stop` | `std_msgs/Bool` | Obstacle stop signal from Environment Model. |
| Input | `/loc_pose` | `geometry_msgs/PoseStamped` | Current vehicle position for waypoint progress tracking. |
| Input | `/route` | `geometry_msgs/PoseArray` | Route for progress monitoring. |
| Output | `/drive` | `std_msgs/Bool` | Drive/stop command to Trajectory Planner. |
| Output | `/park` | `std_msgs/Bool` | Forward park command to Trajectory Planner. |
| Output | `/park_reverse` | `std_msgs/Bool` | Reverse park command to Trajectory Planner. |
| Output | `/act_cmd` | `geometry_msgs/Twist` | Linear and angular velocity commands to Lateral/Longitudinal Control. |

---

#### Live Tracker (`adapt_livtrac`)

Publishes real-time vehicle position and operational status to the Mobile Interface. Allows the user to track their autonomous vehicle during the parking process.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/loc_pose` | `geometry_msgs/PoseStamped` | Current vehicle pose. |
| Input | `/route` | `geometry_msgs/PoseArray` | Route data for navigation guidance display. |
| Output | `/live_loc` | `adapt_msgs/LiveTrack` | Live position and status (`Moving` / `Parked`). |

---

### Act Layer

#### Lateral and Longitudinal Control (`adapt_latlongcon`)

Executes the motion commands computed by Behaviour Planning. Implements **pure pursuit** lateral control for path tracking and longitudinal speed control. Sends the final actuator commands over the **CAN bus** via `ros2_pcan`.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/act_cmd` | `geometry_msgs/Twist` | Linear and angular velocity commands from Behaviour Planning. |
| Input | `/trajd` / `/trajpf` / `/trajpr` | `trajectory_msgs/JointTrajectory` | Trajectory points with position and velocity from Trajectory Planner. |
| Output | `/cmd_vel` | `adapt_msgs/CarCom` | Speed and steering angle commands to the vehicle actuators via CAN bus. |

---

#### Transceiver (`adapt_transceiver`)

The V2X communication bridge for the ego vehicle. Handles both outbound CAM/CPM transmission and inbound CAM reception. Converts between the internal ENU coordinate system and geodetic WGS-84 using `pymap3d`.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/loc_pose` | `geometry_msgs/PoseStamped` | Ego vehicle ENU position for CAM encoding. |
| Input | `/euler_angles` | `geometry_msgs/Vector3` | Ego vehicle yaw for WGS-84 heading conversion. |
| Input | `/detectnet/detections` | `vision_msgs/Detection2DArray` | Object detections for CPM encoding. |
| Input | `/cam_msgs` | `v2x/CAM` | Incoming CAMs from all vehicles (self-filtered by station ID = 9). |
| Output | `/cam_msgs` | `v2x/CAM` | Ego vehicle CAM broadcast at 2 Hz (station ID = 9). |
| Output | `/detected_objects` | `v2x/CPM` | Detected objects encoded as ETSI CPM at detection rate. |
| Output | `/ev_location` | `adapt_msgs/VehData` | Decoded positions of all other vehicles at 1 Hz. |

---

## Infrastructure Components

### Sense Layer

#### Parking Spot Object Detection (`adapt_inf_od`)

Monitors the parking area using the infrastructure's camera. Detects which parking spots are occupied and which are free. Uses the infrastructure (inf) DetectNet model trained for `free`/`occupied` classification.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/selected_spot` | `geometry_msgs/PoseStamped` | Spot selected by the ego vehicle's Spot Selector. |
| Input | `/ev_location` | `geometry_msgs/PoseStamped` | Location of the ego vehicle. |
| Input | `/user_info` | `String` | User information from the Vehicle Interface. |
| Output | `/updated_parking_spots` | `ItsChargingStationData` | Updated parking spot list. |

---

### Plan Layer

#### Spot Updater (`adapt_inf_spotupd`)

Maintains the infrastructure's parking spot database. Updates spot availability based on object detection results and handles spot reservation/cancellation from the Spot Selector.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/selected_spot` | `String` | Spot confirmed by the ego vehicle's Spot Selector. |
| Input | `/vi_cancel` | `String` | Cancellation signal to release a reserved spot. |
| Input | `/occupancy_status` | `vision_msgs/Detection2DArray` | Detected occupied/free spots from the parking camera. |
| Output | `/spot_list` | `String` | Current list of unoccupied, available parking spots. |

---

#### Spot Filter (`adapt_spot_filter`)

Filters and refines the raw spot list from the Spot Updater before it is passed to the Infrastructure Transceiver for V2X broadcast.

---

### Act Layer

#### Infrastructure Transceiver (`adapt_inf_trans`)

The V2X communication bridge for the infrastructure side. Encodes the available parking spot list as an **EVCSN (Electric Vehicle Charging Spot Notification)** message and broadcasts it over the V2X network for the ego vehicle's Transceiver to receive and decode.

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/spot_list` | `String` | Available parking spot list from the Spot Updater. |
| Output | `/evcsn_msg` | `v2x/EVCSN` | Standardised ETSI EVCSN broadcast with parking spot availability. |

---

## Shared / Supporting Packages

#### Custom Messages (`adapt_msgs`)

Central repository for all custom ADAPT ROS 2 message types. Must be built before any other ADAPT package.

| Message | Used by | Description |
|---|---|---|
| `VehData` | Transceiver → Environment Model | Vehicle state: geodetic coordinates, ENU position, quaternion orientation, vehicle ID. |
| `CarCom` | Behaviour Planning → Lat/Long Control | Actuator command: linear velocity, angular velocity, steering angle. |
| `LiveTrack` | Live Tracker → Mobile Interface | Live vehicle pose and operational status string. |
| `DetectedObjects` | Environment Model → Behaviour Planning | Array of detected obstacles with distance and angle. |
| `DetectedObject` | (element of DetectedObjects) | Single detected obstacle: distance (m) and angle (rad). |
| `LaneInfo` | Lane detection → Controller | Lane geometry: confidence, type, curvature, heading, centre pixel position. |

#### V2X Messages (`v2x`)

ETSI ITS message definitions for CAM, CPM, EVCSN, and their sub-structures. Provided by the HS Coburg Autonomous Driving group. Must be built first as it is a dependency of `adapt_transceiver` and `adapt_inf_trans`.

#### Car Description (`car_description`)

URDF/mesh description of the model car. Used by the `adapt_launch.py` launch file via `publish_model.launch.py` to broadcast the vehicle's TF tree and 3D model to RViz.

#### YDLidar ROS 2 (`ydlidar_ros2`)

ROS 2 driver for the YDLidar sensor mounted on the ego vehicle. Publishes `/scan` (`sensor_msgs/LaserScan`) consumed by the Environment Model.

---

## Full ROS 2 Topic Map

The following summarises all major inter-component topics across the full system:

| Topic | Type | Producer | Consumer(s) |
|---|---|---|---|
| `/pose_modelcars` | `mocap_msgs/RigidBodies` | OptiTrack MoCap driver | Localisation |
| `/loc_pose` | `geometry_msgs/PoseStamped` | Localisation | Route Computer, Trajectory Planner, Behaviour Planning, Transceiver, Environment Model, Live Tracker |
| `/euler_angles` | `geometry_msgs/Vector3` | Localisation | Route Computer, Trajectory Planner, Transceiver |
| `/scan` | `sensor_msgs/LaserScan` | YDLidar driver | Environment Model |
| `/detectnet/detections` | `vision_msgs/Detection2DArray` | DetectNet | Transceiver (CPM), Environment Model |
| `/vi_start` | `std_msgs/String` | Vehicle Interface | Spot Selector |
| `/vi_cancel` | `std_msgs/String` | Vehicle Interface | Spot Updater (Infra) |
| `/evcsn_msg` | `v2x/EVCSN` | Infra Transceiver | Spot Selector |
| `/selected_spot` | `geometry_msgs/PoseStamped` | Spot Selector | Route Computer, Trajectory Planner, Infra Spot Updater |
| `/cam_msgs` | `v2x/CAM` | Transceiver (shared) | Transceiver (self + others) |
| `/ev_location` | `adapt_msgs/VehData` | Transceiver | Environment Model (Visualization) |
| `/detected_objects` | `v2x/CPM` | Transceiver (CPM) | V2X network |
| `/scans` | `adapt_msgs/DetectedObjects` | Environment Model | Behaviour Planning |
| `/stop` | `std_msgs/Bool` | Environment Model | Behaviour Planning |
| `/tf_{id}` | `geometry_msgs/TransformStamped` | Environment Model | RViz TF tree |
| `/car_marker_{id}` | `visualization_msgs/Marker` | Environment Model | RViz |
| `/route` | `geometry_msgs/PoseArray` | Route Computer | Trajectory Planner, Behaviour Planning, Live Tracker |
| `/route_state` | `std_msgs/Bool` | Route Computer | Behaviour Planning |
| `/drive` | `std_msgs/Bool` | Behaviour Planning | Trajectory Planner |
| `/park` | `std_msgs/Bool` | Behaviour Planning | Trajectory Planner |
| `/park_reverse` | `std_msgs/Bool` | Behaviour Planning | Trajectory Planner |
| `/trajd` | `trajectory_msgs/JointTrajectory` | Trajectory Planner | Lateral/Longitudinal Control |
| `/trajpf` | `trajectory_msgs/JointTrajectory` | Trajectory Planner | Lateral/Longitudinal Control |
| `/trajpr` | `trajectory_msgs/JointTrajectory` | Trajectory Planner | Lateral/Longitudinal Control |
| `/trajs` | `trajectory_msgs/JointTrajectory` | Trajectory Planner | Lateral/Longitudinal Control |
| `/act_cmd` | `geometry_msgs/Twist` | Behaviour Planning | Lateral/Longitudinal Control |
| `/cmd_vel` | `adapt_msgs/CarCom` | Lateral/Longitudinal Control | Vehicle actuators (via CAN bus) |
| `/live_loc` | `adapt_msgs/LiveTrack` | Live Tracker | Mobile Interface |
| `/spot_list` | `String` | Spot Updater (Infra) | Spot Filter, Infra Transceiver |

---

## UML Diagrams

All diagrams are stored in the `images/` directory.

| Diagram | File | Description |
|---|---|---|
| **Activity Diagram** | `activity_diagram.png` | EV–Infrastructure interaction flow from user parking command through obstacle-aware navigation to final parking. |
| **State Diagram** | `State_Diagram.jpg` | FSM states of the ego vehicle: Idle → Drive → Obstacle Check → Park → Parked. |
| **Sequence Diagram** | `sequence_diagram.png` | Message exchange timeline between EV, Infrastructure, Localisation, and user during full parking sequence. |
| **EV Architecture v3** | `ego-veihcle-v3.png` | Detailed ego vehicle component block diagram. |
| **Infrastructure Architecture** | `infra-archi.png` | Infrastructure component block diagram. |
| **Full rqt Graph** | `rqt_full.png` | Complete ROS 2 topic graph showing all node connections. |
| **Story Map** | `Story Map.png` | Agile user story map used in project planning. |

---

## Repository Structure

```
adapt_main/
├── README.md                     # This file
├── # Architecture v3.0.txt       # Detailed architecture notes and component interface tables
├── adapt_repos.repo              # vcstool repository list for all ADAPT packages
├── images/                       # Architecture diagrams, UML diagrams, photos
│   ├── State_Diagram.jpg
│   ├── activity_diagram.png
│   ├── sequence_diagram.png
│   ├── ego-veihcle-v3.png
│   ├── infra-archi.png
│   ├── rqt_full.png
│   └── ...
├── launch/
│   ├── adapt_launch.py           # Full EV stack launch (12 nodes)
│   ├── infra_launch.py           # Infrastructure stack launch (3 nodes)
│   └── backend_components.py     # Backend/sensor components launch (6 nodes)
└── src/
    └── placeholder               # Submodule source packages cloned here by vcs
```

---

## All Repositories

The `adapt_repos.repo` file defines all repositories imported by `vcs`. The full list:

| Repository | URL | Branch | Description |
|---|---|---|---|
| `adapt_vi` | `…/ADAPT/adapt_vi.git` | main | Vehicle Interface (GUI) |
| `adapt_mobint` | `…/ADAPT/adapt_mobint.git` | main | Mobile Interface |
| `adapt_obj` | `…/ADAPT/adapt_obj.git` | main | Object Detection (DetectNet + dataset) |
| `adapt_trajp` | `…/ADAPT/adapt_trajp.git` | main | Trajectory Planner |
| `adapt_loc` | `…/ADAPT/adapt_loc.git` | main | Localisation (MoCap) |
| `adapt_roucomp` | `…/ADAPT/adapt_roucomp.git` | main | Route Computer (A*) |
| `adapt_latlongcon` | `…/ADAPT/adapt_latlongcon.git` | main | Lateral/Longitudinal Control |
| `adapt_behplan` | `…/ADAPT/adapt_behplan.git` | main | Behaviour Planning (FSM) |
| `adapt_envmod` | `…/ADAPT/adapt_envmod.git` | main | Environment Model |
| `adapt_spotsl` | `…/ADAPT/adapt_spotsl.git` | main | Spot Selector |
| `adapt_transceiver` | `…/ADAPT/adapt_transceiver.git` | main | EV Transceiver (CAM/CPM) |
| `adapt_inf_spotupd` | `…/ADAPT/adapt_inf_spotupd.git` | master | Infrastructure Spot Updater |
| `adapt_inf_trans` | `…/ADAPT/adapt_inf_trans.git` | master | Infrastructure Transceiver (EVCSN) |
| `adapt_inf_od` | `…/ADAPT/adapt_inf_od.git` | main | Infrastructure Object Detection |
| `adapt_spot_filter` | `…/ADAPT/adapt_spot_filter.git` | master | Parking Spot Filter |
| `adapt_msgs` | `…/ADAPT/adapt_msgs.git` | main | Custom ROS 2 message definitions |
| `v2x` | `…/Autonomous_Driving/v2x.git` | students_msgs | ETSI ITS V2X message definitions |
| `yasmin` | `github.com/uleroboticsgroup/yasmin.git` | main | FSM library for Behaviour Planning |
| `car_description` | `…/Autonomous_Driving/car_description.git` | master | Vehicle URDF/mesh for RViz |
| `ydlidar_ros2` | `…/Autonomous_Driving/ydlidar_ros2.git` | master | YDLidar ROS 2 driver |

---

## Software Dependencies

| Dependency | Version / Source | Purpose |
|---|---|---|
| **Ubuntu** | 20.04 LTS | Operating system |
| **ROS 2** | Foxy | Robot middleware framework |
| **mocap_msgs** | [ros-drivers/mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack) | OptiTrack MoCap message types (`RigidBodies`) |
| **v2x msgs** | [git.hs-coburg.de/Autonomous_Driving/v2x](https://git.hs-coburg.de/Autonomous_Driving/v2x.git) | ETSI ITS V2X message definitions (CAM, CPM, EVCSN) |
| **ros2_pcan** | [git.hs-coburg.de/Autonomous_Driving/ros2_pcan](https://git.hs-coburg.de/Autonomous_Driving/ros2_pcan.git) | CAN bus interface for vehicle actuator commands |
| **ros_deep_learning** | [git.hs-coburg.de/Autonomous_Driving/ros_deep_learning](https://git.hs-coburg.de/Autonomous_Driving/ros_deep_learning) | NVIDIA DetectNet ROS 2 node for object detection |
| **realsense2_camera** | [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) | Intel RealSense camera ROS 2 driver |
| **nav2_bringup** | [open-navigation/navigation2](https://github.com/open-navigation/navigation2/blob/main/nav2_bringup/README.md) | Navigation 2 stack |
| **yasmin** | [uleroboticsgroup/yasmin](https://github.com/uleroboticsgroup/yasmin) | FSM library used by Behaviour Planning |
| **scipy** | pip | Cubic spline interpolation in Trajectory Planner |
| **pymap3d** | pip | ENU ↔ Geodetic coordinate conversion in Transceiver |
| **numpy** | pip | Numerical operations across multiple components |

---

## Hardware Dependencies

| Hardware | Purpose |
|---|---|
| **OptiTrack Motion Capture System** | Ground-truth localisation of all vehicles in the Model City. Provides sub-millimetre precision pose data via the `/pose_modelcars` topic. |
| **Intel RealSense Camera** | RGB camera mounted on the ego vehicle for DetectNet-based object detection. Also used by the infrastructure for parking spot occupancy detection. |
| **YDLidar** | 2D LiDAR mounted on the ego vehicle. Provides `/scan` for the Environment Model's stop detection. |
| **NVIDIA Jetson** | On-board GPU compute platform running the full EV software stack including DetectNet inference. |
| **CAN Bus Interface (PCAN)** | Hardware interface between the ROS 2 control stack and the vehicle's drive motor and steering actuators. |

---

## Installation

All ADAPT repositories are managed with `vcstool`. The following steps clone and build the complete workspace.

1. **Clone this repository:**
   ```bash
   git clone https://git.hs-coburg.de/ADAPT/adapt_main.git
   cd adapt_main
   ```

2. **Import all component repositories:**
   ```bash
   vcs import src < adapt_repos.repo
   ```
   This clones all 20 repositories listed in `adapt_repos.repo` into `src/`.

3. **Install ROS dependencies:**
   ```bash
   cd ..
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Install Python dependencies:**
   ```bash
   pip install scipy pymap3d numpy --break-system-packages
   ```

5. **Build the workspace** (build order matters — `adapt_msgs` and `v2x` must be built first):
   ```bash
   colcon build --packages-select adapt_msgs v2x
   source install/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## Running ADAPT

### Prerequisites

Before launching, ensure all hardware is connected and the following background services are running:

1. **Start the OptiTrack MoCap driver** (publishes `/pose_modelcars`):
   ```bash
   ros2 launch mocap_optitrack mocap.launch.py
   ```

2. **Start the Intel RealSense camera** (publishes `/camera/color/image_raw`):
   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```

3. **Start DetectNet** (publishes `/detectnet/detections`):
   ```bash
   ros2 launch ros_deep_learning detectnet.ros2.launch
   ```

4. **Start the YDLidar driver** (publishes `/scan`):
   ```bash
   ros2 launch ydlidar_ros2 ydlidar_launch.py
   ```

### Launch the Ego Vehicle Stack

From the `launch/` directory:

```bash
cd adapt_main/launch
ros2 launch adapt_launch.py
```

This starts the following 12 nodes simultaneously:

| Node name | Package | Executable | Role |
|---|---|---|---|
| `vi` | `adapt_vi` | `gvi_node` | Vehicle Interface |
| `spotsl` | `adapt_spotsl` | `spotsl_node` | Spot Selector |
| `localization` | `adapt_loc` | `localization` | Localisation |
| `routemodule5` | `adapt_roucomp` | `route` | Route Computer |
| `trajectory_planner` | `adapt_trajp` | `traj` | Trajectory Planner |
| `envmod` | `adapt_envmod` | `env_mod` | Environment Model |
| `mapping` | `adapt_envmod` | `map` | Visualisation / TF publisher |
| `path_tracking` | `adapt_latlongcon` | `pp` | Lateral/Longitudinal Control |
| `beh` | `adapt_behplan` | `behave` | Behaviour Planning |
| `adaptmi` | `adapt_mobint` | `minode` | Mobile Interface |
| `transceiver` | `adapt_transceiver` | `transceiver_node` | EV Transceiver (CAM/CPM TX+RX) |
| `CPM` | `adapt_transceiver` | `cpm` | CPM Publisher (DetectNet → CPM) |
| *(car model)* | `car_description` | *(via include)* | Vehicle URDF publisher |

### Launch the Infrastructure Stack

On the infrastructure server, from the `launch/` directory:

```bash
ros2 launch infra_launch.py
```

This starts 3 nodes:

| Node name | Package | Executable | Role |
|---|---|---|---|
| `inf_trans` | `adapt_inf_trans` | `inf_trans` | Infrastructure Transceiver (EVCSN) |
| `inf_trans` | `adapt_inf_spotupd` | `spot_upd` | Spot Updater |
| `filter_node` | `adapt_spot_filter` | `filter_node` | Spot Filter |

### Backend Components Launch (Alternative)

`backend_components.py` provides an alternative launch grouping for sensor and communication-heavy components:

```bash
ros2 launch backend_components.py
```

Starts: `adapt_envmod` (envmod), `adapt_trajp` (traj), `adapt_loc` (localization), `adapt_transceiver` (transceiver_node + cpm), `ros2_pcan` (ros2pcan_node).

---

## Launch Files

| File | Purpose | Nodes started |
|---|---|---|
| `adapt_launch.py` | Full EV autonomous driving stack | 12 nodes + car_description |
| `infra_launch.py` | Full infrastructure parking management stack | 3 nodes |
| `backend_components.py` | Sensor, localisation, and communication backend | 6 nodes |