## Component Description


`Maintainer - Harshawardhan & Anish`

This component is responsible for transmiting and receiving data between Ego-vehicle, Infrastructure and other traffic participants.

It is also responsible for converting and  publishing CAM and CPM messages  


| In/Out | Topic Name| Message Type | Description | 
| --------- | ---------- | ---------- | ----------- |
| Input | `/loc_pose`| `PoseStamped` | The location of the EV.|
| Input | `/detectnet/detctions`| `Detection2DArray` | The objection detection list to Infrastructure |
| Input | `/euler_angles`| `RPY` | The Euler angles of the EV |
| Input | `/selected_spot_location`| `PoseStamped` | The objection detection list to Infrastructure |
| Output | `/ev_location`| `VehData` | locations of all vehicles |
| Output | `/cam_msgs` | `CAM` |It is responsible for publishing the CAM messages of ego vehicle for V2X application|
| Output | `/detected_objects` | `CPM` |It is responsible for publishing the CPM messages for V2X application|

## transceiver Block Diagram
 <img src="https://git.hs-coburg.de/ADAPT/adapt_transceiver/raw/branch/main/images/tr.png" alt="Transceiver Block Diagram" title="Transceiver" width="800" height="400">

 ## Installation Instructions

To install the Transceiver package, do follow the steps below:

### Step 1: Create a worksapce

Open the terminal, navigate to the worksapce and create a 'src' directory where you want to clone the repository. 

```shell
mkdir src
```
### Step 2: Clone the Repository
Navigate to 'src' and follow the given command to clone the repository.
```shell
git clone https://git.hs-coburg.de/ADAPT/adapt_transceiver.git
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
### To publish the CAM message
You can now launch the Transceiver node:
The entry point is **`transceiver_node`**.
```shell
ros2 run adapt_transceiver transceiver_node
```
### To publish CPM message
you can now launch the cpm node
the entry point is **`cpm`**
```shell
ros2 run adapt_transceiver cpm
```