# Object Detection Component

This repository hosts the trained DetectNet model and ROS2 components for object detection in a model city environment. The model has been fine-tuned on a dataset representative of a controlled urban landscape, with the goal of accurately identifying key elements that are crucial for autonomous navigation.

## Overview

Our DetectNet model has been trained to recognize several classes of objects which are integral to the autonomous driving context. These classes, annotated in the dataset, include:

- **Person**: Individuals dummies in the model city.
- **traffic_light**: Signalling devices positioned at road intersections, pedestrian crossings, and other locations to control flows of traffic.
- **potted plant**: Trees, shrubs, and other plant within the model city.
- **car**: Cars, trucks, and other modes of transportation moving or stationary within the model city.

Each class has been annotated with a unique color code for visual differentiation in the detection process.

## Process

The development of our object detection model involved several steps:

1. **Data Capture**: We collected images from our model city using the Intel Realsense Camera.
2. **Annotation**: Utilizing the Roboflow tool, we annotated the dataset with bounding boxes around objects of interest across the aforementioned classes.
3. **Model Training**: We trained our model using NVIDIA Jetson and DetecNet architectures to optimize for prediction precision based on the pre-trained model.
4. **ROS2 Integration**: The trained model was then implemented within a ROS2 environment to serve as a real-time object detection component.

## Classes and Color Codes


We have assigned the following color codes to each class for visual differentiation during the detection process:

| Color       | Class Name    | Hex Code |
|-------------|---------------|----------|
| 🟩| car       | `#808000` |
| 🟥| person    | `#FF0000` |
| 🟪 | potted plant | `#800080` |
| 🟨 | traffic_light    | `#FFFF00` |

## ROS2 Topics for Object Detection

| In/Out | Topic Name| Message Type | Description | 
| --------- | ---------- | ---------- | ----------- |
| Input | /raw_images|  image_in| Raw input image | |
| Output | /detectnet/detections| vision_msgs/Detection2DArray  | Detection results (Bounding boxes, class IDs, confidences) |

## object detection node
Below are the mAP scores of our trained model.


<img src="https://git.hs-coburg.de/ADAPT/adapt_obj/raw/branch/main/images/map.png" alt="mAP Score" title="mAP Score" width="600" height="400">



In order to run the object detection the realsense camera needs to be initialized, to do so run the following command
```shell
ros2 launch realsense2_camera rs_launch.py
``` 
### Note -
- To run the trained model we need to change the path of the model in `detctnet.ros2.launch` file to the the path of trained model.
 
 Now start the detectnet to run the trained model by running the following command 
```shell
ros2 launch ros_deep_learning detectnet.ros2.launch
``` 


