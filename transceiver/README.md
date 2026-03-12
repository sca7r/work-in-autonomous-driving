#  Transceiver

## Overview

The **Transceiver** module is the communication hub of the Ego Vehicle. It handles all **Vehicle-to-Everything (V2X)** messaging, transmitting and receiving standardized messages between the Ego Vehicle, the smart infrastructure, and other traffic participants.

Without this module, the vehicle has no awareness of available parking spots and cannot coordinate with the infrastructure.

---

## Responsibilities

- Transmit **CAM** (Cooperative Awareness Messages) to announce the vehicle's presence
- Transmit **CPM** (Collective Perception Messages) to share perceived objects with others
- Receive **EVCSN** (Electric Vehicle Charging Spot Notification) messages from infrastructure containing available parking spot data
- Bridge V2X communication into ROS 2 topics for internal system use

---

## Message Types

| Message | Direction | Description |
|---|---|---|
| **CAM** | Outbound | Broadcasts ego vehicle position, speed, and heading to infrastructure and other participants |
| **CPM** | Outbound | Shares locally detected objects with surrounding infrastructure/vehicles |
| **EVCSN** | Inbound | Receives available parking spot list from infrastructure |

These message formats follow **ETSI ITS** (Intelligent Transport Systems) standards for V2X communication.

---

## How It Works

```
                    ┌─────────────────────────────┐
ROS 2 Topics ──────▶│                             │──▶  V2X Network (CAM, CPM)
                    │       Transceiver Node      │
V2X Network ───────▶│                             │──▶  ROS 2 Topics (EVCSN data)
                    └─────────────────────────────┘
```

The transceiver acts as a protocol translator, converting internal ROS 2 messages into V2X-compliant packets for transmission, and decoding incoming V2X packets back into ROS 2 messages for the rest of the system.

---

## ROS 2 Interface

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribed | `/vehicle_position` | custom | Ego vehicle pose for CAM generation |
| Subscribed | `/detected_objects` | custom | Object detections for CPM generation |
| Published | `/parking_spots` | custom | Decoded EVCSN parking spot list |

> Topic names may vary — refer to source code for exact topic strings.

---

## Dependencies

- ROS 2 Foxy
- `v2x_msgs` — V2X message definitions ([V2X msg repo](https://git.hs-coburg.de/Autonomous_Driving/v2x.git))
- V2X-capable communication hardware/network interface

---

## Related Components

- **Spot Selector** — consumes the decoded parking spot list from EVCSN messages
- **Localisation** — provides vehicle position for outbound CAM messages
- **Object Detection** — provides detected objects for outbound CPM messages
- **Infrastructure Transceiver** — the counterpart on the infrastructure side that encodes/sends EVCSN
