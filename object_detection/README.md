# Object Detection (`adapt_obj`)

## Overview

The **Object Detection** module is the vision perception layer of the ADAPT (Autonomous Driving And Parking Technology) system. It provides real-time identification and localisation of obstacles in the ego vehicle's surroundings using two complementary sensing modalities: **LiDAR** point cloud processing and **camera-based deep learning inference** via NVIDIA's DetectNet running on an Intel RealSense RGB camera.

This package covers two trained DetectNet models serving distinct purposes within ADAPT: a **vehicle detection model** for identifying cars, persons, and roadside obstacles in the ego vehicle's surroundings, and an **infrastructure (inf) model** for classifying parking spot occupancy, determining whether each parking space is free or occupied.

Detection results from the vehicle model are forwarded to the **Environment Model**, which integrates them into the vehicle's occupancy grid for downstream planning and control. The infrastructure model feeds parking state information to the **Behaviour Planning** component.

A significant portion of this package is the **Adapt_dataset**, a custom annotated image dataset of 5,000 images captured specifically within the ADAPT Model City environment and used to train the DetectNet model. The dataset is managed and exported via Roboflow.

---

## Table of Contents

- [Architecture](#architecture)
- [Detection Classes](#detection-classes)
  - [Vehicle Detection Model](#vehicle-detection-model)
  - [Infrastructure (inf) Model](#infrastructure-inf-model)
- [ROS 2 Interface](#ros-2-interface)
- [Package Structure](#package-structure)
- [Dataset](#dataset)
  - [Dataset v4 — annotated data](#dataset-v4--annotated-data)
  - [Annotation Format](#annotation-format)
  - [Raw Capture Folders](#raw-capture-folders)
  - [Inference Sample Images](#inference-sample-images)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Running Object Detection](#running-object-detection)
- [Testing](#testing)
- [Integration with ADAPT](#integration-with-adapt)

---

## Architecture

```
                                        ┌─────────────────────────────────┐
Intel RealSense Camera                  │     Vehicle Detection Model      │
        │  /camera/color/image_raw      │  DetectNet (ros_deep_learning)   │
        ├──────────────────────────────▶│  car, person, potted plant,      │──▶ /detected_objects
        │                               │  traffic_light, building         │
        │                               └─────────────────────────────────┘
        │
        │                               ┌─────────────────────────────────┐
        │                               │  Infrastructure (inf) Model      │
        └──────────────────────────────▶│  DetectNet (ros_deep_learning)   │──▶ /parking_status
                                        │  free / occupied per spot        │
                                        └─────────────────────────────────┘
        ▲
        │  /lidar/points  (sensor_msgs/PointCloud2)
LiDAR ──┘  (vehicle detection only)
```

Two separate DetectNet models run in ADAPT:

**Vehicle Detection Model** — subscribes to the camera stream and LiDAR point cloud to detect nearby obstacles and classify them by type. Output feeds the Environment Model's occupancy grid.

**Infrastructure (inf) Model** — subscribes to the camera stream pointed at the parking area and classifies each parking spot as free or occupied. Output feeds Behaviour Planning to enable autonomous parking decisions.

---

## Detection Classes

### Vehicle Detection Model

The vehicle model is trained to detect and classify five object categories, each assigned a distinct colour for visualisation:

| Colour | Class Name | Hex Code | Represents in ADAPT |
|---|---|---|---|
| 🟩 Olive | `car` | `#808000` | Model cars in the test environment |
| 🟥 Red | `person` | `#FF0000` | Pedestrian figures |
| 🟪 Purple | `potted plant` | `#800080` | Roadside vegetation / tree stand-ins |
| 🟨 Yellow | `traffic_light` | `#FFFF00` | Traffic signals and road furniture |
| ⬛ Dark | `building` | — | Roadside structures |

> **Note:** The class label `potted plant` is the COCO label used by DetectNet for the vegetation-like obstacles present in the Model City. It does not mean literal potted plants  it maps to the roadside tree/shrub props used in the test environment.

### Infrastructure (inf) Model

The infrastructure model is dedicated to **parking spot occupancy detection**. A fixed camera observes the parking area and the model classifies each visible parking spot as either:

| Class | Description |
|---|---|
| `free` | The parking spot is unoccupied and available. |
| `occupied` | The parking spot is taken by a vehicle. |

This enables the ego vehicle to autonomously identify available parking spaces without relying on external sensors or infrastructure signals.

---

## ROS 2 Interface

### Vehicle Detection Model — Subscriptions

| Topic | Message Type | Description |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB frames from the Intel RealSense camera, passed to DetectNet for inference. |
| `/lidar/points` | `sensor_msgs/PointCloud2` | Raw LiDAR point cloud used for distance measurement and obstacle proximity. |

### Vehicle Detection Model — Publications

| Topic | Message Type | Description |
|---|---|---|
| `/detected_objects` | custom | Detected obstacle positions, bounding boxes, and class labels for consumption by the Environment Model. |

### Infrastructure (inf) Model — Subscriptions

| Topic | Message Type | Description |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB frames from the camera observing the parking area. |

### Infrastructure (inf) Model — Publications

| Topic | Message Type | Description |
|---|---|---|
| `/parking_status` | custom | Per-spot occupancy state (`free` or `occupied`) for consumption by Behaviour Planning. |


---

## Package Structure

```
object_detection/
├── adapt_obj/
│   └── __init__.py               # ROS 2 Python package init
├── Adapt_dataset/
│   ├── annotated data/           # Dataset v4 — Adapt_detectnet (5000 images, 640×640)
│   │   ├── train/                # 3500 image/annotation pairs
│   │   ├── valid/                # 1000 image/annotation pairs
│   │   ├── test/                 # 500 image/annotation pairs
│   │   ├── README.dataset.txt    # Roboflow dataset metadata
│   │   └── README.roboflow.txt   # Export provenance
│   ├── blue car/                 # Raw captures: blue model cars
│   ├── human01/                  # Raw captures: pedestrian figure type 1
│   ├── human02/                  # Raw captures: pedestrian figure type 2
│   ├── lane1/ … lane4a/          # Raw captures: lane scenes (8 sub-categories)
│   ├── latest_dataset/           # Most recent raw capture batch
│   ├── parking spots/            # Raw captures: parking area scenes
│   ├── parking spots 01/02/      # Raw captures: parking area variants
│   ├── redcar01/                 # Raw captures: red model car
│   ├── square1/ square2/ square3/# Raw captures: intersection/square scenes
│   ├── test/                     # Raw captures: test/misc scenes
│   ├── tree01/ tree02/           # Raw captures: vegetation obstacles
│   └── (other capture folders)
├── git/                          # Reference/development screenshots (30 images)
├── images/                       # Documentation screenshots (map.png, object.png)
├── inf_dataset/                  # Infrastructure model: parking spot inference sample images (28 images)
├── resource/
│   └── adapt_obj                 # Ament resource index marker
├── test/
│   ├── test_copyright.py         # Ament copyright linting
│   ├── test_flake8.py            # Flake8 style checks
│   └── test_pep257.py            # PEP 257 docstring checks
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

---

## Dataset

The `Adapt_dataset` directory contains all training data for both DetectNet models used in ADAPT. The annotated dataset covers the vehicle detection model classes. The raw capture folders include scene types used across both models, including dedicated parking spot captures for the infrastructure model.

### Dataset v4 — `annotated data` (Vehicle Detection Model)

| Property | Value |
|---|---|
| Roboflow project | `Adapt_detectnet` |
| Version | v4 (exported 2024-04-25) |
| Roboflow URL | https://universe.roboflow.com/rah-oz5oq/adapt_detectnet |
| License | CC BY 4.0 |
| Total images | 5,000 images across splits |
| Image resolution | 640 × 640 (stretched) |
| Annotation format | Pascal VOC (XML) |
| Pre-processing | Auto-orientation (EXIF strip), resize to 640×640 |
| Augmentation | None |

**Split sizes:**

| Split | Images |
|---|---|
| train | 3,500 |
| valid | 1,000 |
| test | 500 |

**Class distribution (training split):**

| Class | Annotation count |
|---|---|
| `potted plant` | 5,956 |
| `car` | 4,550 |
| `traffic light` | 4,397 |
| `person` | 242 |
| `building` | 95 |

### Annotation Format

All annotations use **Pascal VOC XML** format. Each image has a paired `.xml` file containing:

```xml
<annotation>
  <filename>…</filename>
  <size>
    <width>640</width><height>640</height><depth>3</depth>
  </size>
  <object>
    <name>car</name>           <!-- class label -->
    <bndbox>
      <xmin>121</xmin><xmax>492</xmax>
      <ymin>304</ymin><ymax>492</ymax>
    </bndbox>
  </object>
  …
</annotation>
```

Some annotations in v4 also include a `<polygon>` element with vertex coordinates for more precise contour labelling alongside the bounding box.

File naming follows Roboflow's convention: `{original_name}_jpg.rf.{hash}.jpg` / `.xml`.

### Raw Capture Folders

The original, unannotated (or pre-annotation) image captures are stored in named scene folders directly under `Adapt_dataset/`:

| Folder | Scene type |
|---|---|
| `blue car/` | Blue model car in the test environment |
| `human01/`, `human02/` | Two pedestrian figure types |
| `lane1/`, `lane1a/`, `lane2/`, `lane2a/`, `lane3/`, `lane4/`, `lane4a/` | Various lane and road section views |
| `parking spots/`, `parking spots 01/`, `parking spots 02/` | Parking area scenes |
| `redcar01/` | Red model car captures |
| `square1/`, `square2/`, `square3/` | Intersection / town square scenes |
| `tree01/`, `tree02/` | Vegetation obstacle captures |
| `test/` | Miscellaneous test captures |
| `latest_dataset/` | Most recent batch of raw captures |

### Infrastructure Model — Inference Sample Images

The `inf_dataset/` directory contains 28 PNG images used to validate the **infrastructure (inf) model**, sample frames of the parking area passed through the trained occupancy classifier to verify that free and occupied spots are correctly identified. These images represent the fixed-camera view of the parking zone used during inference.

---

## Dependencies

### ROS 2

| Dependency | Purpose |
|---|---|
| `rclpy` | ROS 2 Python client library |
| `sensor_msgs` | `Image` and `PointCloud2` message types |

### External / Hardware

| Dependency | Purpose |
|---|---|
| `ros_deep_learning` | NVIDIA DetectNet ROS 2 node for camera-based inference. |
| `realsense2_camera` | Intel RealSense ROS 2 driver, providing `/camera/color/image_raw`. |
| LiDAR driver | Hardware-dependent; provides `/lidar/points`. |
| Intel RealSense camera | RGB camera hardware for DetectNet input. |
| LiDAR sensor | 2D/3D range sensor hardware. |
| NVIDIA GPU | Required by DetectNet for real-time deep learning inference. |

### Build System

- **ROS 2 Foxy** (or compatible)
- **ament_python** build type

---

## Installation

1. Place the package in your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   # copy or clone adapt_obj here
   ```

2. Install ROS dependencies:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Install `ros_deep_learning` following NVIDIA's Jetson documentation:
   [https://github.com/dusty-nv/ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning)

4. Install the RealSense ROS 2 driver:

   ```bash
   sudo apt install ros-foxy-realsense2-camera
   ```

5. Build the package:

   ```bash
   colcon build --packages-select adapt_obj
   source install/setup.bash
   ```

---

## Running Object Detection

### Vehicle Detection Model

**Step 1 — Start the Intel RealSense camera:**

```bash
ros2 launch realsense2_camera rs_launch.py
```

**Step 2 — Start DetectNet inference (vehicle model):**

```bash
ros2 launch ros_deep_learning detectnet.ros2.launch
```

DetectNet subscribes to the camera topic and publishes detected objects with class labels and bounding boxes.

**Step 3 — Start the LiDAR driver** for your hardware. It should publish to `/lidar/points`. Refer to your sensor manufacturer's ROS 2 driver documentation.

**Verify detections:**

```bash
ros2 topic echo /detected_objects
```

### Infrastructure (inf) Model

**Step 1 — Start the camera observing the parking area:**

```bash
ros2 launch realsense2_camera rs_launch.py
```

**Step 2 — Start DetectNet inference (infrastructure model):**

```bash
ros2 launch ros_deep_learning detectnet.ros2.launch
```

Point the model at the parking area camera feed. The model classifies each visible parking spot and publishes occupancy state.

**Verify parking status:**

```bash
ros2 topic echo /parking_status
```

---

## Testing

The test suite consists only of ament linting checks, there are no unit or integration tests in this package for the detection logic itself, as the core inference is delegated to the external `ros_deep_learning` package.

| Test | Tool | Description |
|---|---|---|
| `test_copyright.py` | ament_copyright | Checks source file copyright headers. |
| `test_flake8.py` | flake8 | PEP 8 style compliance. |
| `test_pep257.py` | pep257 | Docstring convention checks. |

Run all tests:

```bash
cd ~/ros2_ws
colcon test --packages-select adapt_obj
colcon test-result --verbose
```


---

## Integration with ADAPT

| Component | Relationship |
|---|---|
| **Environment Model (`adapt_envmod`)** | Primary downstream consumer of `/detected_objects` from the vehicle detection model. Integrates camera-based detections alongside LiDAR scan data to populate the occupancy grid used by planners. |
| **Behaviour Planning** | Consumes `/parking_status` from the infrastructure model to identify free parking spots and execute autonomous parking manoeuvres. Also uses the environment model's occupancy grid (fed by vehicle detections) for stop/go decisions. |
| **Intel RealSense** | Provides raw RGB frames as input to both DetectNet models. |
| **LiDAR** | Provides complementary range data for the vehicle detection pipeline. LiDAR processing is handled by the Environment Model node directly alongside camera-based detections. |

---


## License

Apache License 2.0 — see [LICENSE](LICENSE) for details.