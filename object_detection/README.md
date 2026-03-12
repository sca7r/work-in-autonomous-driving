#  Object Detection

## Overview

The **Object Detection** module is responsible for identifying and localizing obstacles in the Ego Vehicle's surroundings. It combines two complementary sensing modalities — **LiDAR** point cloud data and **camera-based deep learning inference** via NVIDIA's DetectNet — to provide robust, real-time detection of static and dynamic objects.

The detected objects are forwarded to the **Environment Model**, which integrates them into the vehicle's occupancy grid.

---

## Responsibilities

- Process raw **LiDAR** point cloud data to detect nearby objects
- Run **DetectNet** inference on **Intel RealSense** camera frames
- Publish detected object positions and classes for the Environment Model

---

## How It Works

```
Intel RealSense Camera
        │  RGB frames
        ▼
   DetectNet (ros_deep_learning)
        │  detected objects + bounding boxes
        ▼
  Object Detection Node  ──────────────────▶  /detected_objects
        ▲
        │  point cloud
LiDAR ──┘
```

LiDAR provides precise 3D distance measurements for nearby obstacles, while DetectNet adds semantic classification (e.g. pedestrian, vehicle, cone) from camera data. Together they produce richer detections than either sensor alone.

---

## ROS 2 Interface

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribed | `/camera/color/image_raw` | `sensor_msgs/Image` | RGB frames from RealSense |
| Subscribed | `/lidar/points` | `sensor_msgs/PointCloud2` | LiDAR point cloud |
| Published | `/detected_objects` | custom | Detected obstacle positions & classes |

> Topic names may vary — refer to source code for exact topic strings.

---

## Running Object Detection

**Start the Intel RealSense camera:**
```bash
ros2 launch realsense2_camera rs_launch.py
```

**Start DetectNet inference:**
```bash
ros2 launch ros_deep_learning detectnet.ros2.launch
```

---

## Dependencies

- ROS 2 Foxy
- `ros_deep_learning` — NVIDIA DetectNet for ROS 2
- `realsense2_camera` — Intel RealSense ROS 2 driver
- LiDAR driver (hardware dependent)
- Intel RealSense Camera (hardware)
- LiDAR sensor (hardware)

---

## Related Components

- **Environment Model** — consumes detection output to build the occupancy grid
- **Behaviour Planning** — reacts to detected obstacles (e.g. stopping before parking)


The following color codes are assigned to each class for visual differentiation during the detection process:

| Color       | Class Name    | Hex Code |
|-------------|---------------|----------|
| 🟩| car       | `#808000` |
| 🟥| person    | `#FF0000` |
| 🟪 | potted plant | `#800080` |
| 🟨 | traffic_light    | `#FFFF00` |




