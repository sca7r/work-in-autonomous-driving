#  Control

## Overview

The **Control** module is responsible for the physical actuation of the Ego Vehicle. It translates high-level driving commands from the **Behaviour Planning** component into low-level motor signals sent over the **CAN BUS**, governing both how the vehicle steers and how fast it moves.

This is the final step in the autonomous driving pipeline before commands reach the hardware.

---

## Responsibilities

- Receive speed and steering manoeuvres from the Behaviour Planning FSM
- Execute **lateral control** (steering direction)
- Execute **longitudinal control** (acceleration/braking/speed)
- Send actuation commands to the drive motor via **CAN BUS**

---

## How It Works

```
Behaviour Planning
       │
       │  speed manoeuvre commands
       ▼
  Control Node
  ┌─────────────────────────┐
  │  Lateral Controller     │  →  Steering angle
  │  Longitudinal Controller│  →  Throttle / Brake
  └─────────────────────────┘
       │
       │  CAN BUS
       ▼
   Drive Motor / Vehicle Hardware
```

The control module decouples lateral and longitudinal axes, processing each independently before issuing a unified CAN BUS output to the vehicle's drive system.

---

## ROS 2 Interface

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribed | `/speed_manoeuvre` | custom | Speed & steering commands from Behaviour Planning |
| Published | — | CAN BUS | Direct hardware commands to the drive motor |

---

## Dependencies

- ROS 2 Foxy
- `ros2_pcan` — CAN BUS interface for ROS 2
- ADAPT Behaviour Planning (`adapt_behplan`)

---

## Related Components

- **Behaviour Planning** — upstream source of manoeuvre commands
- **Trajectory Planner** — provides the path that behaviour planning interprets
