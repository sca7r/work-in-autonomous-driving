## Component Description

`Maintainer - Harshawardhan`


This component is responsible for the environment perception for ADAPT. It recieves detected objects from LiDAR and the vehicle position through Localization component and publishes them in the form of custom message(DetectedObjects) for the rest of the system.




| In/Out | Topic Name| Message Type | Description | 
| --------- | ---------- | ---------- | ----------- |
| Input | `/loc_pose`| `PoseStamped` | The current position of our vehicle |
| Input | `/scan` | `LaserScan `| The LiDAR detections |
| Input | `/ev_location`| `VehData` | The pose and orientation of other cars|
| Output | `/car_marker_{id}`| `Marker` | These are visualization messages of the poses and orientations of our car and other cars that are transmitting CAM messgae|
| Output | `/scans`| `DetectedObjects` | Outputs the distance and angles ofthe objects detected|
| Output | `/stop`| `Bool` | This message is published when the node detects obstacle in its close range(forward) |
## The RQT Graph
- The RQT graph below is for the env.py node which is publishing the detected objects

<div align="center">
<img src="https://git.hs-coburg.de/ADAPT/adapt_envmod/raw/branch/main/images/scan.png"  width="800" height="350">
</div>



- The RQT graph below is for the visualization.py node which is publishing the car markers for the sake of visualization of all the cars publishing CAM messages  

<div align="center">
<img src="https://git.hs-coburg.de/ADAPT/adapt_envmod/raw/branch/main/images/viz.png" width="800" height="350">
</div>

## Component Interface
<div align="center">
 <img src="https://git.hs-coburg.de/ADAPT/adapt_envmod/raw/branch/main/images/interface.jpg"  width="800" height="350">
 </div>

 ## Dependencies
- adapt model city map 
- nav2_bringup
- adapt_msgs
- car_description
- LiDAR

# Functionality

## env.py


### Overview

The node processes laser scan data and pose information to detect objects in the environment. The node transforms the detected objects' coordinates from the laser frame to the base link frame, publishes the detected objects, and issues stop messages if objects are within a predefined stop range.

### Algorithms

### 1. Euler to Quaternion Conversion

#### Function: `euler_to_quaternion(self, roll, pitch, yaw)`

**Purpose:** Converts Euler angles (roll, pitch, yaw) to a quaternion representation used for rotation transformations in 3D space.

**Algorithm:**
1. Compute the quaternion components using trigonometric functions:
   - \( qx = \sin(\text{roll}/2) \cdot \cos(\text{pitch}/2) \cdot \cos(\text{yaw}/2) - \cos(\text{roll}/2) \cdot \sin(\text{pitch}/2) \cdot \cos(\text{yaw}/2) \)
   - \( qy = \cos(\text{roll}/2) \cdot \sin(\text{pitch}/2) \cdot \cos(\text{yaw}/2) + \sin(\text{roll}/2) \cdot \cos(\text{pitch}/2) \cdot \cos(\text{yaw}/2) \)
   - \( qz = \cos(\text{roll}/2) \cdot \cos(\text{pitch}/2) \cdot \sin(\text{yaw}/2) - \sin(\text{roll}/2) \cdot \sin(\text{pitch}/2) \cdot \cos(\text{yaw}/2) \)
   - \( qw = \cos(\text{roll}/2) \cdot \cos(\text{pitch}/2) \cdot \cos(\text{yaw}/2) + \sin(\text{roll}/2) \cdot \sin(\text{pitch}/2) \cdot \sin(\text{yaw}/2) \)

**Output:** A list containing the quaternion components \([qx, qy, qz, qw]\).

### 2. Quaternion to Euler Conversion

#### Function: `quaternion_to_euler(self, x, y, z, w)`

**Purpose:** Converts a quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).

**Algorithm:**
1. Compute intermediate variables:
   - \( t0 = +2.0 \cdot (w \cdot x + y \cdot z) \)
   - \( t1 = +1.0 - 2.0 \cdot (x \cdot x + y \cdot y) \)
   - \( t2 = +2.0 \cdot (w \cdot y - z \cdot x) \)
   - \( t3 = +2.0 \cdot (w \cdot z + x \cdot y) \)
   - \( t4 = +1.0 - 2.0 \cdot (y \cdot y + z \cdot z) \)
2. Compute Euler angles:
   - \( \text{roll} = \text{atan2}(t0, t1) \)
   - \( t2 = \text{clamp}(t2, -1.0, 1.0) \)
   - \( \text{pitch} = \text{asin}(t2) \)
   - \( \text{yaw} = \text{atan2}(t3, t4) \)

**Output:** A tuple containing the roll, pitch, and yaw angles in radians.

### 3. Transform Coordinates to Base Link Frame

#### Function: `transform_coordinates_to_base_link(self, x, y, transform)`

**Purpose:** Transforms coordinates from the laser frame to the base link frame using a given transform (translation and rotation).

**Algorithm:**
1. Extract the translation and rotation from the `transform` object.
2. Convert the quaternion rotation to Euler angles using the `quaternion_to_euler` method.
3. Apply the translation and rotation to the coordinates:
   - \( x_{\text{base}} = (x \cdot \cos(\text{yaw}) - y \cdot \sin(\text{yaw})) + \text{translation}.x \)
   - \( y_{\text{base}} = (x \cdot \sin(\text{yaw}) + y \cdot \cos(\text{yaw})) + \text{translation}.y \)

**Output:** A tuple containing the transformed x and y coordinates in the base link frame.

### 4. Timer Callback for Processing and Publishing Data

#### Function: `timer_callback(self)`

**Purpose:** Periodically processes the latest `LaserScan` message to detect objects, transforms their coordinates, and publishes detected objects and stop messages.

#### Processing Steps:

1. **Data Retrieval:**
   - Check if a scan message is available.
   - Warn if no scan data is available.
2. **Object Detection:**
   - For each range in the scan message:
     - Calculate the angle: \( \text{angle} = \text{angle\_start} + \text{angle\_min} + i \cdot \text{angle\_increment} \)
     - Filter by angle range: \(-0.5 < \text{angle} < 0.5\) (this enables the car to only consider the region of 30 degrees ahead of it)
     - Compute coordinates in the laser frame:
       - \( x_{\text{laser}} = \text{distance} \cdot \cos(\text{angle}) \)
       - \( y_{\text{laser}} = \text{distance} \cdot \sin(\text{angle}) \)
     - Transform coordinates to the base link frame using the `transform_coordinates_to_base_link` method.
     - Check if the object is within the stop range:
       - \( \text{transformed\_distance} = \sqrt{x_{\text{base}}^2 + y_{\text{base}}^2} \)
       - \( \text{transformed\_angle} = \text{atan2}(y_{\text{base}}, x_{\text{base}}) \)
     - Create and append `DetectedObject` messages.
3. **Publishing:**
   - Publish the `DetectedObjects` message.
   - Check for obstacles within the stop range and publish a `Bool` stop message.



## Gallery
#### The visualization of scan from LiDAR 

<div align="center">
 <img src="https://git.hs-coburg.de/ADAPT/adapt_envmod/raw/branch/main/images/scans1.png"  width="800" height="350">
 </div>
<div align="center">
 <img src="https://git.hs-coburg.de/ADAPT/adapt_envmod/raw/branch/main/images/scans2.png"  width="800" height="350">
 </div>
  


#### When there is any obstacle in stop range with output

<div align="center">
 <img src="https://git.hs-coburg.de/ADAPT/adapt_envmod/raw/branch/main/images/stop_rviz2.png"  width="800" height="350">
 </div>
<div align="center">
 <img src="https://git.hs-coburg.de/ADAPT/adapt_envmod/raw/branch/main/images/stop_rviz.png"  width="800" height="350">
 </div><div align="center">
 <img src="https://git.hs-coburg.de/ADAPT/adapt_envmod/raw/branch/main/images/stop.png"  width="800" height="350">
 </div>

---


## visualization.py 

### Overview

The node processes vehicle pose and location data to publish transforms and visualization markers for ADAPT and other vehicles. The node uses specific algorithms to handle incoming data and publish the required transformations and markers.

### Algorithms

### 1. Location Pose Callback

**Function:** `loc_pose_callback(self, msg)`

**Purpose:** Processes the localization pose of the car with ID 9 and publishes the corresponding transform and marker.

**Algorithm:**
1. Extract car ID and pose data from the received `PoseStamped` message.
2. Create and publish a `TransformStamped` message:
   - Set header, child frame ID, translation, and rotation based on the pose data.
   - Set the frame ID to "map".
3. Create and publish a `Marker` message:
   - Set header, namespace, ID, type, mesh resource, action, pose, scale, and color based on the car's information.

**Output:** Publishes the transform and marker for car ID 9.

### 2. Vehicle Location Callback

**Function:** `ev_location_callback(self, msg)`

**Purpose:** Processes the location data of other vehicles and publishes the corresponding transforms and markers.

**Algorithm:**
1. Extract car ID and location data from the received `VehData` message.
2. Create and publish a `TransformStamped` message:
   - Set header stamp to current time.
   - Set frame ID to "map" and child frame ID based on the car ID.
   - Set translation and rotation based on the vehicle's position and orientation data.
3. Create and publish a `Marker` message:
   - Set header, namespace, ID, type, mesh resource, action, pose, scale, and color based on the car's information.
   - Use specific colors for cars with IDs 7 (yellow) and 10 (blue).

**Output:** Publishes the transform and marker for the specified car ID.

## Gallery
#### The visualization Cars publishing CAM 
 </div><div align="center">
 <img src="https://git.hs-coburg.de/ADAPT/adapt_envmod/raw/branch/main/images/visualisation.jpg"  width="800" height="350">
 </div>


---

## Setup

## Installation Instructions

To install the Environment Model package, do follow the steps below:

### Step 1: Create a worksapce

Open the terminal, navigate to the worksapce and create a 'src' directory where you want to clone the repository. 

```shell
mkdir src
```
### Step 2: Clone the Repository
Navigate to 'src' and follow the given command to clone the repository.
```shell
git clone https://git.hs-coburg.de/ADAPT/adapt_envmod.git
```
### Step 3: Build the Package
After cloning the repository, navigate back to the workspace
```shell
cd ..
```
Now, build the package using **`colcon`**:
```shell
colcon build --symlink-install
```
### Step 4: Source the setup file
Once the build is complete, you'll need to source the workspace to make it available to ROS2:
```shell
source install/setup.bash
```

## To run the visualization node:
This node publishes the location of all the cars transmitting CAM messages on the model city map.


To publish the model city map, make sure to source ros2 and nav2 and run the following command: 

```shell
ros2 launch nav2_bringup localization_launch.py map:='/home/af/adapt_main/src/adapt_envmod/adapt_map.yaml'
```

Few things to consider
- change the .yaml file path based on your system 
- If nav2 is not installed, please follow the steps as described in [this website](https://navigation.ros.org/getting_started/index.html). 

To visualise the model city map on the Rviz2, run rviz2 in another terminal. Click on Add > By Topic > Map , to add map to the Displays. The map would be published only once, hence, end the command in terminal used to publish the modelcity map and rerun it.


you can now launch the mapping node

the executable is **`map`**
```shell
ros2 run adapt_envmod map
```
after running the node add and adjust the markers with corresponding topics in order to visualise them on the map.

## To run the env node:


#### Note
Run the following command to clone the car_description repository.
```shell
https://git.hs-coburg.de/Autonomous_Driving/car_description.git
```
after cloning, colcon build and source the termianl


If you are looking to visualize the model in RVIZ2 you can simply use the provided launch file by running the following command:
```shell
ros2 launch car_description visualize_model.launch.py
```



This will start the robot_state_publisher node to publish the URDF file and also RVIZ2 with a predefined configuration file.

If you want to use the model without RVIZ2, you only need to publish it by using the robot_state_publisher. To do this, simply use the provided launch file by running the following command:
```shell
ros2 launch car_description publish_model.launch.py
```

Run the following command to start the LiDAR.
```shell
ros2 launch ydlidar_launch.py
```

Lastly run the below command to run the env node

This node publishes the Detected objects.
the executable is **`envmod`**
```shell
ros2 run adapt_envmod envmod
```



