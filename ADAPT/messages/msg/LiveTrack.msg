# LiveTrack Message
#
# This message type is used to communicate the live tracking status of a vehicle,
# including its current position (pose) and its operational status (e.g., moving or parked).

# The current pose of the vehicle using ROS geometry_msgs/PoseStamped message type.
# This includes position and orientation of the vehicle.
geometry_msgs/PoseStamped pose

# The status of the vehicle as a string.
# Typical statuses might include "Moving", "Parked", etc., describing the vehicle's current action state.
string status
