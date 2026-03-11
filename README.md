#  Work in Autonomous Driving

> A curated collection of modules and components developed or contributed for Autonomous Driving, centered around the **ADAPT** system: an infrastructure-based, end-to-end autonomous parking solution.

---

##  About This Repository

This repository showcases individual components I built or contributed to as part of the **ADAPT (Autonomous Driving & Parking Technology)** project. ADAPT is a full-stack autonomous parking system where an Ego Vehicle communicates with smart infrastructure to locate, navigate to, and park at the nearest feasible parking spot, without any human input.

> **Note:** This isn't the complete project. Some components are proprietary or hosted elsewhere. Feel free to reach out for discussions or deeper dives into any module.

---

##  The Core Question

> *How can we design an infrastructure-based End-to-End parking solution for all people arriving with an Autonomous Vehicle into a covered urban area, to eliminate the distance traveled and effort required for parking, reduce emissions, by integrating leading-edge technologies in communication, sensing infrastructure, and autonomous driving?*

---

##  Repository Structure

```
work-in-autonomous-driving/
│
├── control/              # Lateral & longitudinal vehicle control
├── environment_model/    # Occupancy grid & environment perception
├── localisation/         # Real-time GNSS & mocap-based localization
├── main_project/         # Top-level integration & launch configuration
├── messages/             # Custom ROS2 message definitions
├── object_detection/     # LiDAR & camera-based object detection
├── route_computer/       # Optimal route computation to parking spot
├── trajectory/           # Trajectory planning & smooth path generation
└── transceiver/          # V2X communication (CAM, CPM, EVCSN messages)
```

---

##  System Components

###  Transceiver
Handles Vehicle-to-Infrastructure (V2X) communication. Transmits and receives **CAM**, **CPM**, and **EVCSN** messages between the Ego Vehicle, Infrastructure, and other traffic participants.

###  Localisation
Provides precise real-time positioning of the Ego Vehicle. Processes data from a **Motion Capture (MoCap)** system, publishing X/Y/Z position and Euler orientation angles for downstream navigation.

###  Route Computer
Determines the optimum path from the vehicle's current position to the selected parking spot. Outputs an efficient route to the **Trajectory Planner** and publishes route state to **Behaviour Planning**.

###  Environment Model
Fuses object detections from **LiDAR** and **DetectNet** with the vehicle's localization to publish a live **OccupancyGrid**, the vehicle's perception of its surroundings.

###  Trajectory
Smooths the raw route using **cubic spline interpolation** and computes the parking maneuver geometry using circle arcs and line segments.

###  Control
Translates speed manoeuvres from Behaviour Planning into **CAN BUS** commands to the drive motor, managing both lateral (steering) and longitudinal (speed) control.

###  Messages
Custom ROS2 message type definitions shared across system components.

###  Object Detection
Detects static and dynamic obstacles using **LiDAR** and an **Intel RealSense** camera with deep learning inference via DetectNet.

---

##  System Flow

```
User Input (Preferences)
        │
        ▼
Vehicle Interface → Spot Selector → Route Computer
                                          │
                                          ▼
                                   Trajectory Planner
                                          │
                                          ▼
                              Behaviour Planning (FSM)
                                          │
                                  ┌───────┴───────┐
                                  ▼               ▼
                           Lateral Control   Longitudinal Control
                                          │
                                          ▼
                                   EV Parks 
```

---

##  Tech Stack

| Category | Technology |
|---|---|
| Language | Python 3, CMake |
| Middleware | ROS 2 Foxy |
| OS | Ubuntu 20.04 |
| Sensing | Intel RealSense, LiDAR, OptiTrack MoCap |
| Communication | V2X (CAM / CPM / EVCSN) |
| Navigation | Nav2, Cubic Spline, CAN BUS |
| Deep Learning | NVIDIA DetectNet (ros_deep_learning) |

---

##  Dependencies

### Software
| Dependency | Link |
|---|---|
| Ubuntu 20.04 | [Install Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop) |
| ROS 2 Foxy | [Install Guide](https://docs.ros.org/en/foxy/Installation.html) |
| mocap_optitrack | [GitHub](https://github.com/ros-drivers/mocap_optitrack) |
| Intel RealSense ROS | [GitHub](https://github.com/IntelRealSense/realsense-ros) |
| nav2_bringup | [GitHub](https://github.com/open-navigation/navigation2) |
| yasmin (FSM) | [GitHub](https://github.com/uleroboticsgroup/yasmin) |

### Hardware
- OptiTrack Motion Capture System
- Intel RealSense Camera
- LiDAR Sensor

---

##  Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/sca7r/work-in-autonomous-driving.git
cd work-in-autonomous-driving
```

### 2. Install ROS 2 Dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace
```bash
colcon build --symlink-install
```

### 4. Source the Workspace
```bash
source install/setup.bash
```

---

##  Running the System

**Launch the Ego Vehicle stack:**
```bash
ros2 launch adapt_launch.py
```

**Launch the Infrastructure stack:**
```bash
ros2 launch infra_launch.py
```

**Start RealSense Camera:**
```bash
ros2 launch realsense2_camera rs_launch.py
```

**Start Object Detection (DetectNet):**
```bash
ros2 launch ros_deep_learning detectnet.ros2.launch
```

---

##  System Architecture

The ADAPT system consists of two main subsystems:

**Ego Vehicle** — Onboard modules for perception, planning, control, and V2X communication.

**Infrastructure** — Smart sensors that monitor parking spot availability and relay updates via EVCSN messages.

Both communicate over a V2X link, enabling the vehicle to receive live parking spot data and autonomously navigate to the selected spot.

---

##  License

This project is licensed under the **Apache 2.0 License**, see the [LICENSE](LICENSE) file for details.

---

##  Get in Touch

This repository only contains a subset of the full ADAPT system. If you're interested in:
- The complete codebase
- Technical deep-dives on any module
- Collaboration or research discussions

Feel free to **open an issue** or reach out directly. Always open for discussions! 🚀
