

# Test Report: EnvModel Class

## Overview

This report documents the results of the unit tests conducted for the `EnvModel` class in the ADAPT project. The tests evaluate the functionality of pose callbacks, scan callbacks, and object detection mechanisms implemented within the `EnvModel` class.

- **Tested Component**: `EnvModel` class in `adapt_envmod.env`
- **Test Framework**: `unittest`
- **ROS Client Library**: `rclpy`

## Test Results


<div align="center">
<img src="https://git.hs-coburg.de/ADAPT/adapt_envmod/raw/branch/main/images/testenv.png" width="700" height="500">
</div>

The following tests were executed:

### 1. Node Initialization Test (`test_init_node`)

- **Description**: Verifies the initialization of the ROS node.
- **Status**: Passed

### 2. Pose Callback Test (`test_pose_callback`)

- **Description**: Validates the handling of pose messages.
- **Status**: Passed

### 3. Scan Callback Test (`test_scan_callback`)

- **Description**: Checks the processing of laser scan messages.
- **Status**: Passed

### 4. Object Detection Test (`test_object_detection_within_stop_range`)

- **Description**: Tests if object detection within stop range triggers the stop signal.
- **Status**: Passed

## Detailed Test Results

### 1. Node Initialization Test (`test_init_node`)

- **Test Outcome**: Passed
- **Details**:
  - The `EnvModel` node was initialized successfully.
  - Node name is correctly set to `env_model`.
  - Instance of `EnvModel` class is of the expected type.

### 2. Pose Callback Test (`test_pose_callback`)

- **Test Outcome**: Passed
- **Details**:
  - Simulated a pose message (`PoseStamped`) and verified the update of `base_link_pose`.
  - Asserted that `base_link_pose` was correctly updated with expected position and orientation.

### 3. Scan Callback Test (`test_scan_callback`)

- **Test Outcome**: Passed
- **Details**:
  - Simulated a laser scan message (`LaserScan`) and ensured transformation setup.
  - Checked that the `detected_objects_publisher` correctly published a message with detected objects.

### 4. Object Detection Test (`test_object_detection_within_stop_range`)

- **Test Outcome**: Passed
- **Details**:
  - Simulated a laser scan message with object distances within the stop range.
  - Confirmed that the `stop_publisher` correctly published a stop signal message.

## Conclusion

All tests for the `EnvModel` class in the ADAPT project have passed successfully. The pose callback, scan callback, and object detection mechanisms were validated to be functioning correctly based on the provided test cases.
