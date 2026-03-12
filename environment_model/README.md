# Environment Model

## Overview

The **Environment Model** module is the perception backbone of the ADAPT system. It fuses data from multiple sensing sources — **LiDAR** and **camera-based deep learning detection** — along with the vehicle's position from the **Localisation** component, and builds a unified spatial map of the environment as an **OccupancyGrid**.

This grid is consumed by other components (e.g., Behaviour Planning, Route Computer) to make safe, informed driving decisions.

---

## Responsibilities

- Receive detected objects from LiDAR point cloud processing
- Receive detected objects from DetectNet (camera-based deep learning)
- Receive vehicle position from the Localisation component
- Fuse all inputs into a coherent **OccupancyGrid**
- Publish the OccupancyGrid for downstream use

---

## How It Works

```
LiDAR Detections  ──────┐
                         │
DetectNet Detections ────┼──▶  Environment Model Node  ──▶  /occupancy_grid
                         │
Vehicle Position ────────┘
  (from Localisation)
```

Each detected object is placed into the grid relative to the vehicle's current position. The resulting occupancy grid marks cells as free, occupied, or unknown — giving the rest of the system a real-time view of the surroundings.

---

## ROS 2 Interface

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribed | `/detected_objects_lidar` | custom | LiDAR-detected objects |
| Subscribed | `/detected_objects_camera` | custom | DetectNet-detected objects |
| Subscribed | `/vehicle_position` | custom | Ego vehicle position from Localisation |
| Published | `/occupancy_grid` | `nav_msgs/OccupancyGrid` | Fused environment map |



---

## Dependencies

- ROS 2 Foxy
- `nav_msgs` (OccupancyGrid)
- ADAPT Localisation (`adapt_loc`)
- `ros_deep_learning` (DetectNet)
- LiDAR driver / point cloud processor

---

## Related Components

- **Localisation** — provides vehicle pose used to anchor the grid
- **Object Detection** — upstream source of detected obstacle data
- **Behaviour Planning** — consumes the occupancy grid for decision making
- **Route Computer** — uses grid to plan obstacle-free routes
