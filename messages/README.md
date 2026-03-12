#  Messages

## Overview

The **Messages** module defines all custom **ROS 2 message types** used across the ADAPT system. Rather than each component defining its own data structures independently, all shared message interfaces are centralized here — ensuring consistency and reusability across the full pipeline.

---

## Responsibilities

- Define custom `.msg` files for inter-component communication
- Define custom `.srv` files for service-based interactions (if any)
- Provide a single source of truth for all ADAPT-specific data types

---

## Why a Dedicated Messages Package?

In ROS 2, custom message types must be compiled into their own package before they can be imported by other packages. Centralizing them here means:

- Any component can depend on `messages` without circular dependencies
- Changes to message structures are made in one place
- Consistent field naming and types across the entire system

---

## Structure

```
messages/
├── msg/
│   ├── *.msg          # Custom message definitions
├── CMakeLists.txt
└── package.xml
```

---

## How to Use in Another Package

In your `package.xml`:
```xml
<depend>messages</depend>
```

In your Python node:
```python
from messages.msg import YourCustomMessage
```

---

## Building

This package must be built **before** any other ADAPT component that depends on it:

```bash
colcon build --packages-select messages
source install/setup.bash
```

---

## Dependencies

- ROS 2 Foxy
- `rosidl_default_generators`
- `rosidl_default_runtime`

---

## Related Components

All components in the ADAPT system depend on this package for shared message types.
