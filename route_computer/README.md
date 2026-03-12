#  Route Computer

## Overview

The **Route Computer** determines the optimal path from the Ego Vehicle's current position to the selected parking spot. It acts as the bridge between spot selection and physical navigation — taking a destination and computing a drivable, efficient route through the environment.

The computed route is passed downstream to the **Trajectory Planner** for smoothing, and the route state is published to **Behaviour Planning** to drive FSM transitions.

---

## Responsibilities

- Receive the selected parking spot location (from Spot Selector)
- Receive the vehicle's current position (from Localisation)
- Compute an optimal, collision-aware route to the destination
- Publish the route to the Trajectory Planner
- Publish route state updates to Behaviour Planning

---

## How It Works

```
Spot Selector  ──── selected spot location ───┐
                                               │
Localisation   ──── current vehicle position ─┼──▶  Route Computer
                                               │         │
                                               │         ├──▶  /route        (to Trajectory Planner)
                                               │         └──▶  /route_state  (to Behaviour Planning)
```

The route computer takes both the start (current vehicle pose) and the goal (parking spot coordinates) and calculates a feasible path — typically using a graph-based or grid-based planning algorithm over the occupancy grid or a known map of the environment.

---

## ROS 2 Interface

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribed | `/selected_spot` | custom | Target parking spot from Spot Selector |
| Subscribed | `/vehicle_position` | custom | Current ego vehicle pose from Localisation |
| Published | `/route` | custom | Computed path waypoints to Trajectory Planner |
| Published | `/route_state` | custom | Current routing status to Behaviour Planning |

> Topic names may vary — refer to source code for exact topic strings.

---

## Dependencies

- ROS 2 Foxy
- `nav2_bringup` — Navigation 2 stack for path planning primitives
- ADAPT Localisation (`adapt_loc`)
- ADAPT Messages (`messages`)

---

## Related Components

- **Spot Selector** — provides the destination parking spot
- **Localisation** — provides the vehicle's starting position
- **Trajectory Planner** — receives and smooths the computed route
- **Behaviour Planning** — monitors route state to trigger FSM transitions
