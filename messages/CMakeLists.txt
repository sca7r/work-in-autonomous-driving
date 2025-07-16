cmake_minimum_required(VERSION 3.5)
project(adapt_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # This package is needed for std_msgs/String which is used in LiveTrack.msg
find_package(vision_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneInfo.msg"
  "msg/CarCom.msg"
  "msg/VehData.msg"
  "msg/LiveTrack.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs  # Add dependencies needed by your messages here
)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs vision_msgs)  # Ensure all dependencies are exported

ament_package()
