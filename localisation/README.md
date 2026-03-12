# Localisation

## Overview

The **Localisation** module provides the Ego Vehicle with a precise, real-time understanding of where it is in the world. In the ADAPT system, this is achieved using an **OptiTrack Motion Capture (MoCap)** system deployed within the Model City environment.

The component processes incoming MoCap data, converts raw quaternion orientation into intuitive Euler angles, and publishes the vehicle's full 6DOF pose for other components to consume.

---

## Responsibilities

- Listen to incoming data from the OptiTrack MoCap system
- Extract X, Y, Z position coordinates
- Convert quaternion orientation to **Euler angles** (roll, pitch, yaw)
- Publish the vehicle's position and orientation in real time

---

## How It Works

```
OptiTrack MoCap System
        │
        │  raw pose (position + quaternion)
        ▼
  Localisation Node
  ┌──────────────────────────────┐
  │  Quaternion → Euler convert  │
  │  Position extraction         │
  └──────────────────────────────┘
        │
        ▼
  /vehicle_position  (X, Y, Z + Roll, Pitch, Yaw)
```

Quaternion-to-Euler conversion is performed internally to provide human-readable and controller-friendly angle representations, making downstream use in control and planning simpler.

---

## ROS 2 Interface

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribed | MoCap system topic | `mocap_msgs` | Raw position & quaternion from OptiTrack |
| Published | `/vehicle_position` | custom | X, Y, Z + Euler angles of the vehicle |


---

## Dependencies

- ROS 2 Foxy
- `mocap_optitrack` — [ros-drivers/mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack)
- OptiTrack Motion Capture hardware

---

## Notes

- This component is designed specifically for use within a **Model City** test environment equipped with an OptiTrack setup.
- For real-world deployment, the localisation source would typically be replaced with a **GNSS + IMU** fusion system or an HD Map-based localisation solution.

---

## Related Components

- **Environment Model** — uses vehicle position to anchor the occupancy grid
- **Route Computer** — uses current position as the starting point for path planning
- **Behaviour Planning** — uses position to track progress along the route
