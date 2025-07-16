
## Component Description
The RouteComputer node is responsible for calculating the optimal route from the vehicle's current location to a selected parking spot within the predefined model city map. It subscribes to the vehicle's location and the selected parking spot coordinates and publishes a series of waypoints that define the route to the target parking spot. The node uses the A* algorithm to compute the shortest path between nodes on the map. Additionally, the RouteComputer accounts for the initial orientation of the vehicle in path planning and applies a turning penalty to encourage good traffic behaviour and find the optimum path. The component also utilizes a cardinal directional edges map, enabling the A* algorithm to track orientation changes as well. 

 We've referred [SMEC Path Planning](https://git.hs-coburg.de/SMEC/smec_path_planning.git) and [Path Planning](https://git.hs-coburg.de/Autonomous_Driving/path_planning.git) for the basic A* algorithm and basic map.

 <div align="center">

| In/Out | Topic Name| Message Type | Description | 
| --------- | ---------- | ---------- | ----------- |
| Input | /loc_pose | PoseStamped | The current position of our vehicle |
| Input | /euler_angels | Vector3 | The current orientation of our vehicle |
| Input | /selected_spot | PoseStamped | The selected parking spot's location |
| Output | /route | PoseArray | A optimum route from the vehicle's location to the parking spot|
| Output | /route_state | Bool | Route state if it is generated or not|

</div>

## Route Computer Block Diagram

 <img src="https://git.hs-coburg.de/ADAPT/adapt_roucomp/raw/branch/main/images/Block%20Diagram.jpg" alt="Route Computer Block Diagram" title="Route Computer" width="1200" height="550">

## A* Algorithm

This section describes the enhanced A* algorithm used in the Route Computer component, which incorporates turning penalties and direction mapping to optimize vehicle pathfinding. The flowchart below outlines the main steps of the algorithm:

1. **Start A* Algorithm**: The algorithm initialization begins.

2. **Initialize Open Set with Start Node**: The open set, containing nodes to be evaluated, starts with the initial node.

3. **Initialize g_score and f_score**: 
   - `g_score`: Cost from the start node to the current node.
   - `f_score`: Estimated total cost from the start node to the goal, passing through the current node.

4. **Open Set is Not Empty**: The main loop of the algorithm continues as long as the open set has nodes to evaluate. If empty, no path is found.

5. **Select Node with Lowest f_score**: The node in the open set with the lowest `f_score` is selected as the current node for evaluation.

6. **Check if Current Node is Goal Node**: If the current node is the goal, reconstruct the path and terminate.

7. **Evaluate Neighbors of Current Node**: If the current node is not the goal, proceed to evaluate each of its neighbors.

8. **Calculate Tentative g_score**: For each neighbor, calculate the tentative `g_score` from the start node to this neighbor.

9. **Calculate Direction Penalty**: Apply a direction penalty to account for vehicle orientation changes and to encourage optimal routing behavior.

10. **Update Scores and Path if Better**: If the new path to the neighbor is better (lower cost) than previously known paths, update the `g_score`, `f_score`, and the path.

11. **Add Neighbor to Open Set**: If the neighbor provides a valid path, add it to the open set for future evaluation.

12. **Reconstruct and Return Path**: Upon reaching the goal node, reconstruct the optimal path from the goal back to the start node by following recorded best paths.

<div align="center">
 <img src="https://git.hs-coburg.de/ADAPT/adapt_roucomp/raw/branch/main/images/A_algo.png" alt="Route Computer Block Diagram" title="Route Computer" width="500" height="900">
</div>

This enhanced A* algorithm ensures efficient and realistic vehicle route planning within the model city map by considering turning penalties and tracking direction changes.

###  A* Algorithm Sequence Diagram

This sequence diagram illustrates the interactions between different components involved in the enhanced A* algorithm within the code. It details the process of pathfinding with turning penalties and direction mapping to ensure optimal route calculation.

1. **RouteComputer**
   - Initiates the A* Algorithm.

2. **AStarAlgorithm**
   - **Initialize Open Set, g_score, and f_score**: Sets up the initial parameters for the algorithm.
   - **Open Set is Not Empty**: Checks if there are nodes left to evaluate.
   - **Select Node with Lowest f_score**: Chooses the node with the lowest `f_score` for evaluation.
   - **Is This Node the Goal?**: Determines if the current node is the goal node.
     - If yes, triggers **PathReconstruction** to return the path.
     - If no, continues to evaluate neighbors.

3. **NodeEvaluation**
   - **Get Neighbors of Current Node**: Retrieves the neighboring nodes for evaluation.

4. **NeighborProcessing**
   - For each neighbor:
     - **Calculate Tentative g_score with Turning Penalty and Initial Orientation**: Computes the tentative `g_score` considering the turning penalty and initial vehicle orientation.
     - **direction_change_penalty(from_direction, to_direction)**:
       - 0: Same direction
       - 5: Right-angle turn
       - 10: U-turn
     - **Is Tentative g_score < g_score[neighbor]?**: Checks if the newly calculated `g_score` is less than the previously known `g_score` for that neighbor.
       - If yes, updates `came_from`, `g_score`, and `f_score` for the neighbor.

5. **NodeEvaluation**
   - **Add Neighbor to Open Set**: Adds the evaluated neighbor to the open set for further evaluation.

6. **AStarAlgorithm**
   - **No Path Found**: If no path can be found (open set is empty without reaching the goal), the process ends without a valid route.

7. **PathReconstruction**
   - **Reconstruct and Return Path**: If the goal node is reached, reconstructs the optimal path from the goal back to the start node.

<div align="center">
 <img src="https://git.hs-coburg.de/ADAPT/adapt_roucomp/raw/branch/main/images/diagram.png" alt="Route Computer Block Diagram" title="Route Computer" width="700" height="700">
</div>

This sequence diagram showcases the flow and interactions of the enhanced A* algorithm, detailing how turning penalties and direction mapping are incorporated to find the most efficient and realistic route.

## Code Components and Interactions
This mind map outlines the main components of the Route Computer code:

- **RouteComputerCardinal:**  Handles node initialization, map loading, and A* algorithm execution, and applying turning penalties for optimal pathfinding.
- **Metrics Logging:** Manages CSV initialization and logging of performance metrics.
- **Map Processing:** Processes nodes, edges, and parking spots, and assigns parking nodes.
- **Direction Handling:** Converts yaw to cardinal directions, applies direction penalties, and maps direction changes.
These components work together to ensure accurate route planning, incorporating turning penalties and direction mapping.

<div align="center">
 <img src="https://git.hs-coburg.de/ADAPT/adapt_roucomp/raw/branch/main/images/code_components.png" alt="Route Computer Block Diagram" title="Route Computer" width="600" height="800">
</div>

### Method Call Flow

This section explains the method call flow in the Route Computer component, as depicted in the provided flowchart. The flowchart outlines the sequence of method calls and their interactions, starting from program initialization to the final route publication and logging.

1. **Program Start**
   - The program begins with the initialization (`__init__`) method, setting up the environment for route planning.

2. **Initialize Components**
   - **Create Publisher**: Initializes the publishers for routing and state information.
   - **Create Subscription**: Sets up subscriptions for:
     - `spot_location_callback`: Handles updates for selected parking spot locations.
     - `location_pose_callback`: Handles updates for the vehicle's current location and orientation.
     - `euler_angles_callback`: Manages updates for vehicle's orientation in Euler angles.
   - **Create Timer**: Sets up a timer to trigger periodic route calculations via the `timer_callback`.

3. **Load Map**
   - **initialize_metrics_logging**: Sets up metrics logging, preparing for data collection.
     - **process_node_line**: Processes each node in the map data.
     - **process_edge_line**: Processes each edge in the map data.
     - **process_parking_spot_line**: Processes each parking spot in the map data.

4. **Timer Callback**
   - The timer triggers the `timer_callback` method, which initiates the following processes:
     - **A* Algorithm (`a_star`)**: Executes the enhanced A* algorithm with turning penalties and direction mapping to compute the optimal path.
     - **Assign Parking Nodes**: Assigns nodes to parking spots for accurate route calculation.
     - **Log Metrics (`log_metrics`)**: Records metrics related to the route planning process.

5. **Route Publishing**
   - **Publish Route (`publish_route`)**: Publishes the calculated route.
   - **Publish Route State (`publish_route_state`)**: Publishes the state of the route, including any relevant metadata.

<div align="center">
<img src="https://git.hs-coburg.de/ADAPT/adapt_roucomp/raw/branch/main/images/Method_Call_Flow.png" alt="Route Computer Block Diagram" title="Route Computer" width="800" height="700">
</div>

## Map.txt Explanation

 **Section defining nodes in the map**
 Format: x-coordinate y-coordinate node_id

         NODES

         0.5 0.7 1        
         1.5 0.7 2
         2.3 0.4 3
         3.5 1.0 4
         3.50 1.5 5
         3.50 2.7 6

 **Section defining edges (connections) between nodes**
 Format: node_id neighbor_id [neighbor_id ...]

         EDGES

         1 2 20
         2 1 3
         3 2 4
         4 3 5 21
         5 4 6
         6 5 7 30 29

 **Section defining parking spots**
 Format: x-coordinate y-coordinate

         PARKING_SPOTS

         4.75 5.25
         4.75 5.75
         4.75 6.25
         4.75 6.75        

## Map_cardinal.txt

 **Section defining nodes in the map**
 Format:(x, y) and a unique node ID

         NODES

         0.5, 0.7, 1
         1.5, 0.7, 2
         2.3, 0.4, 3
         3.5, 1.0, 4
         3.50, 1.5, 5
         3.50, 2.7, 6  

 **Section defining edges (connections) between nodes**
 Format: start node ID, end node ID, and the cardinal direction from start to end

         Edges

         1, 2, W
         1, 20, S
         2, 1, E
         2, 3, W
         3, 2, E
         3, 4, W      

 **Section defining parking spots**
 Format: coordinates (x, y)

         Parking Spots

         4.75, 5.25
         4.75, 5.75
         4.75, 6.25
         4.75, 6.75      
               
## Route Computer Rqt
 The RQT graph shows how the system works together to help the ego vehicle find a parking spot. The localization node tells the system where the vehicle is and its orientation. The spot_finder node tells the system where the chosen parking spot is. This information goes to the route_computer_cardinal node, which calculates the best route to the parking spot. It shares this route with the trajectory_planner node, which plans how the vehicle will move along that route. The route_computer_cardinal node also updates the behaviour planning on whether the route is ready. 
 
 
 <div align="center">
 <img src="https://git.hs-coburg.de/ADAPT/adapt_roucomp/raw/branch/main/images/rqt_module6.png" alt="Route Computer Block Diagram" title="Route Computer" width="1800" height="270">
</div>


## Functionality
1. The Route Computer is subscribing to the /loc_pose and /selected_spot which contains the EV's location, orientation and the selected parking spot's location.
2. And using the modified A* algorithm it calculates and gives a route considering EV's orientation at start point to the parking spot using the nodes and edges from the map_cardinal.txt in the form of way points (PoseArray) and publishes on the topic '/route'.

<div align="center">
<img src="https://git.hs-coburg.de/ADAPT/adapt_roucomp/raw/branch/main/images/relative_path.png" alt="Route Computer Block Diagram" title="Route Computer" width="810" height="810">
</div>

This is the representation of the nodes in the map.txt

## Dependencies
- localization
- Spot Selector

## Demo & Results
We tested the route_cardinal by placing the EV at origin of the model city and with different orientations (car heading)

1. So in the first case the car is placed at the origin of the model city and the orientation of the vehicle is at south.

<div align="center">
<img src="https://git.hs-coburg.de/ADAPT/adapt_roucomp/raw/branch/main/images/south.png" alt="Route Computer Block Diagram" title="Route Computer" width="400" height="400">
</div>

2. In the second case the car is placed at the origin of the model city but the orientation of the vehicle is at west.

<div align="center">
<img src="https://git.hs-coburg.de/ADAPT/adapt_roucomp/raw/branch/main/images/west.png" alt="Route Computer Block Diagram" title="Route Computer" width="400" height="400">
</div>

you can see the car is getting the waypoints of the route considering the orientation of the car even though the starting position of the EV is same for both cases.

 ## Installation Instructions

To install the Route Computer package, do follow the steps below:

### Step 1: Create a worksapce

Open the terminal, navigate to the worksapce and create a 'src' directory where you want to clone the repository. 

```shell
mkdir src
```
### Step 2: Clone the Repository
Navigate to 'src' and follow the given command to clone the repository.
```shell
git clone https://git.hs-coburg.de/ADAPT/adapt_roucomp.git
```
### Step 3: Build the Package
After cloning the repository, navigate back to the workspace
```shell
cd ..
```
Now, build the package using **`colcon`**:
```shell
colcon build --symlink-install --package-select adapt_roucomp
```
### Step 4: Source the setup file
Once the build is complete, you'll need to source the workspace to make it available to ROS2:
```shell
source install/setup.bash
```
### Step 5: Now Run the Node
You can now launch the Route Computer node:
The executable for Route Computer  is **`route`** for basic a* algorithm, and ***route_cardinal** for our enhanced a* algorithm.
```shell
ros2 run adapt_roucomp route
ros2 run adapt_roucomp route_cardinal
```

