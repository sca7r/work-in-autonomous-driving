
# Test Report: Visualization Node (Environment Class)

## Overview

This report provides an overview of the tests conducted for the `Environment` class in the ADAPT project. The tests validate the functionality of TF transforms and marker publishing implemented in the `Environment` class.

- **Tested Component**: `Environment` class in `adapt_envmod.visualization`
- **Test Framework**: `unittest`
- **ROS Client Library**: `rclpy`

## Test Results

<div align="center">
<img src="https://git.hs-coburg.de/ADAPT/adapt_envmod/raw/branch/main/images/viztest.png" width="700" height="500">
</div>



The following tests were executed:

### 1. Node Initialization Test (`test_node_initialization`)

- **Description**: Verifies the initialization of the ROS node.
- **Status**: Passed

### 2. Localization Pose Callback Test (`test_loc_pose_callback`)

- **Description**: Simulates the localization pose callback and checks the TF transform and marker publishing for a specific car ID (`9`).
- **Status**: Passed

### 3. EV Location Callback Test (`test_ev_location_callback`)

- **Description**: Simulates the EV location callback and checks the TF transform and marker publishing for dynamic car IDs (`7` in this case).
- **Status**: Passed

### 4. Marker Publishing Test (`test_marker_publishing`)

- **Description**: Verifies the content of the marker message published after an EV location callback.
- **Status**: Passed

## Detailed Test Results

### 1. Node Initialization Test (`test_node_initialization`)

- **Test Outcome**: Passed
- **Details**:
  - The `Environment` node was initialized successfully.
  - Node name is correctly set to `Environment`.
  - Instance of `Environment` class is of the expected type.

### 2. Localization Pose Callback Test (`test_loc_pose_callback`)

- **Test Outcome**: Passed
- **Details**:
  - Simulated a localization pose message (`PoseStamped`) for car ID `9`.
  - Verified the TF transform broadcast (`TransformStamped`) from `map` to `car_9`.
  - Verified the marker message (`Marker`) published for car ID `9` with expected pose, scale, and color.

### 3. EV Location Callback Test (`test_ev_location_callback`)

- **Test Outcome**: Passed
- **Details**:
  - Simulated an EV location message (`VehData`) for car ID `7`.
  - Verified the TF transform broadcast (`TransformStamped`) from `map` to `car_7`.
  - Verified the marker message (`Marker`) published for car ID `7` with expected pose, scale, and color.

### 4. Marker Publishing Test (`test_marker_publishing`)

- **Test Outcome**: Passed
- **Details**:
  - Subscribed to the marker message published for car ID `7`.
  - Verified the contents of the marker message against expected values.

## Conclusion

All tests for the `Environment` class in the ADAPT project have passed successfully. The TF transforms and marker publishing mechanisms were validated to be working correctly based on the provided test cases.



