

## Component Description
The localization component provides a precise location of the Ego-Vehicle with respect to its environment. It is designed to handle real-time localization data from motion capture systems used in the Model City. This node operates by subscribing to real-time pose data of the model car, as provided by the motion capture system, and processing this data to both refine its accuracy and convert it into useful formats.

> :memo: **Note:** For more details regarding functions development, Criteria, and implementation of Module 5 please cheack the folder: **M5_Functions**. 

### Component Interfaces
| In/Out | Topic Name          | Message Type                 | Description                                  |
|--------|---------------------|------------------------------|----------------------------------------------|
| Input  | `/pose_modelcars`     | `mocap_msgs/msg/RigidBodies` | Receives data from motion capture systems where each RigidBody has an ID.   |
| Output | `/loc_pose`         | `geometry_msgs/msg/PoseStamped` | Publishes position and orientation ( as ENU coordinates) for the correct ID     |
| Output | `/euler_angels`         | `geometry_msgs/msg/Vector3` | Publishes Euler angles representing orientation.     |

### Localization Block Diagram
 
<div align="center">
    <img src="https://git.hs-coburg.de/ADAPT/adapt_loc/raw/branch/main/images/loc_bloc_v4.png" width="700" height="450">
</div>



## Node Overview
Upon initialization, the node sets up subscriptions to receive RigidBodies messages containing positional and orientational data of various objects (including ADAPT model car) from a topic `/pose_modelcars`. It specifically filters for data corresponding to the model car, identified by a unique `ID=9`. Once the car's data is received, the node processes it by converting the orientation from quaternion to Euler Pitch,Yaw, and Roll angles that describe the car’s orientation in space. 

The node also rounds the position coordinates to 4 decimals to maintain a consistent precision level and then publishes two types of messages: `PoseStamped` messages containing the refined pose (position and orientation) data, and `Vector3` messages containing the Euler angles. These published messages can then be utilized by other components to make informed navigation decisions.

Through systematic logging, the node keeps a trace of its operations, which aids in debugging and real-time monitoring of the localization process. Overall, the code serves as a critical link between the raw data from the motion capture system and the higher-level navigation and control algorithms that drive the autonomous model car.


### Euler Angles Calculation

In the `SimpleLocalization` node, Euler angles (roll, pitch, yaw) are derived from the quaternion representation of the orientation, which is part of the incoming pose data from OptiTrack. This conversion is for other components requiring angular orientations in a more intuitive format than quaternions. The mathematical transformation used is as follows:

1. **Roll** (rotation around the x-axis) is calculated by:
   \[
   \text{roll} = \text{atan2}(2 \cdot (w \cdot x + y \cdot z), 1 - 2 \cdot (x^2 + y^2))
   \]
2. **Pitch** (rotation around the y-axis) is computed using:
   \[
   \text{pitch} = \text{asin}(2 \cdot (w \cdot y - z \cdot x))
   \]


3. **Yaw** (rotation around the z-axis) is determined by:
   \[
   \text{yaw} = \text{atan2}(2 \cdot (w \cdot z + x \cdot y), 1 - 2 \cdot (y^2 + z^2))
   \]

These calculations convert the quaternion orientation data into Euler angles (in radians), which are then published to ´/euler_angels´ topic from the message type ´Vector3´

### Localization RQT Graph 

| Interaction     | Topic             | Destination Node(s)                    |
|-----------------|-------------------|----------------------------------------|
| Subscribes to   | `/pose_modelcars` | `/localization`                        |
| Publishes to    | `/loc_pose`       | `/transceiver_data` ,`/lateral`, `/live_tracker`,  `/beh_plan`, `/route_computer`,  `/Environment`         |
| Publishes to    | `/euler_angles`   | `/transceiver_data`                    |


<div align="center">
    <img src="https://git.hs-coburg.de/ADAPT/adapt_loc/raw/branch/main/images/rqt_graph_5.png" width="700" height="350">
</div>


### ROS2 Node Demo
The screenshot illustrates the console output of localization node as it processes and publishes data from an OptiTrack system, focusing on a rigid body labeled "9" which is the EV assinged ID in the model city OptiTrack System. The node receives this data, which includes the position and orientation, and it performs the following operations:
This data is being published in two primary formats:
* PoseStamped: This includes the 3D position (In meters) and orientation of the rigid body.  
* Euler Angles: Simultaneously, the node publishes Euler angles (Roll, Pitch, Yaw) in Radians.


<div align="center">
    <img src="https://git.hs-coburg.de/ADAPT/adapt_loc/raw/branch/main/images/demo_m5.png" width="700" height="450">
</div>

## Installation Instructions

To install the Localization package, do follow the steps below:

### Step 1: Clone the Repository
Navigate to 'src' and follow the given command to clone the repository.
```shell
git clone https://git.hs-coburg.de/ADAPT/adapt_loc.git
```
### Step 2: Build the Package
After cloning the repository, navigate back to the workspace
```shell
cd ..
```
Now, build the package using **`colcon`**:
```shell
colcon build --symlink-install
```
### Step 3: Source the setup file
Once the build is complete, you'll need to source the workspace to make it available to ROS2:
```shell
source install/setup.bash
```
### Step 5: Now Run the Node
You can now launch the Localization node:
The entry point for Localization is **`localization`**.
```shell
ros2 run adapt_loc localization
```


