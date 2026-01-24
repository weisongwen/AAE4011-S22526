# AAE4011 AI for Unmanned Autonomous Systems - ROS Package

This ROS package contains AI algorithms and examples for unmanned autonomous systems, developed for the AAE4011 course at The Hong Kong Polytechnic University.

## Package Overview

**Package Name:** `aae4011_ai_uas`  
**Version:** 1.0.0  
**ROS Version:** ROS 1 (Noetic)  
**Maintainer:** Dr. Weisong Wen (welson.wen@polyu.edu.hk)

## Features

- **Object Detection**: YOLOv5-based object detection for autonomous systems
- **Trajectory Visualization**: Visualize trajectory data from CSV files in RViz
- **Logistic Regression**: Binary classification using logistic regression
- **Utility Functions**: Common functions for quaternion conversion, sigmoid, etc.

## Installation

### Prerequisites

1. **ROS Noetic** (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04)
2. **Python 3** with required packages
3. **catkin workspace**

### Install Dependencies

```bash
# Install ROS dependencies
sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport

# Install Python dependencies
pip3 install torch torchvision numpy pandas scipy matplotlib
pip3 install ultralytics  # For YOLOv5
```

### Build the Package

```bash
# Navigate to your catkin workspace
cd ~/catkin_ws/src

# Clone or copy the package
# Option 1: Clone entire repository
git clone https://github.com/weisongwen/AAE4011-S22526.git
ln -s AAE4011-S22526/ros_package/aae4011_ai_uas .

# Option 2: Copy package directly
cp -r /path/to/aae4011_ai_uas .

# Build
cd ~/catkin_ws
catkin_make

# Source the workspace
source devel/setup.bash
```

## Nodes

### 1. trajectory_visualizer.py

Visualizes trajectory data from CSV files in RViz.

**Published Topics:**
- `/trajectory` (nav_msgs/Path): Trajectory path
- `/trajectory_markers` (visualization_msgs/MarkerArray): Visualization markers

**Parameters:**
- `~csv_file`: Path to CSV file with trajectory data
- `~frame_id`: Frame ID for visualization (default: "world")
- `~publish_rate`: Publishing rate in Hz (default: 10)

**Usage:**
```bash
roslaunch aae4011_ai_uas trajectory_visualizer.launch csv_file:=/path/to/data.csv
```

### 2. object_detection_node.py

Performs object detection using YOLOv5.

**Subscribed Topics:**
- `/camera/image_raw` (sensor_msgs/Image): Input image

**Published Topics:**
- `/detections` (std_msgs/String): JSON formatted detection results
- `/detection_image` (sensor_msgs/Image): Image with bounding boxes

**Parameters:**
- `~image_topic`: Input image topic
- `~detection_topic`: Output detection topic
- `~model_name`: YOLOv5 model (yolov5n, yolov5s, yolov5m, yolov5l, yolov5x)
- `~confidence_threshold`: Detection threshold (default: 0.5)

**Usage:**
```bash
roslaunch aae4011_ai_uas object_detection.launch image_topic:=/camera/image_raw
```

### 3. logistic_regression_node.py

Demonstrates logistic regression for binary classification.

**Subscribed Topics:**
- `/input_features` (std_msgs/Float64MultiArray): Input feature vector

**Published Topics:**
- `/classification_result` (std_msgs/String): JSON classification result
- `/probability` (std_msgs/Float64): Probability of class 1

**Parameters:**
- `~slope`: Slope parameter for logistic function
- `~intercept`: Intercept parameter
- `~threshold`: Classification threshold (default: 0.5)

**Usage:**
```bash
rosrun aae4011_ai_uas logistic_regression_node.py _slope:=1.0 _intercept:=0.0
```

## Launch Files

### trajectory_visualizer.launch
```bash
roslaunch aae4011_ai_uas trajectory_visualizer.launch
```

### object_detection.launch
```bash
roslaunch aae4011_ai_uas object_detection.launch
```

### demo.launch
```bash
roslaunch aae4011_ai_uas demo.launch
```

## CSV File Format

For trajectory visualization, CSV files should contain:

| Column | Description |
|--------|-------------|
| Time | Timestamp (nanoseconds) |
| PosX | X position |
| PosY | Y position |
| PosZ | Z position |
| VelX | X velocity (optional) |
| VelY | Y velocity (optional) |
| VelZ | Z velocity (optional) |
| QuatW | Quaternion W (optional) |
| QuatX | Quaternion X (optional) |
| QuatY | Quaternion Y (optional) |
| QuatZ | Quaternion Z (optional) |

## Examples

### Visualize Trajectory
```bash
# Start roscore
roscore

# In another terminal, run trajectory visualizer
roslaunch aae4011_ai_uas trajectory_visualizer.launch \
    csv_file:=$(rospack find aae4011_ai_uas)/config/sample_trajectory.csv

# In another terminal, start RViz
rviz
# Add Path display, set topic to /trajectory
```

### Run Object Detection
```bash
# Start roscore
roscore

# Run a camera node (example with USB camera)
rosrun usb_cam usb_cam_node

# Run object detection
roslaunch aae4011_ai_uas object_detection.launch image_topic:=/usb_cam/image_raw

# View detection results
rostopic echo /detections
```

### Test Logistic Regression
```bash
# Start roscore
roscore

# Run logistic regression node
rosrun aae4011_ai_uas logistic_regression_node.py

# In another terminal, publish test data
rostopic pub /input_features std_msgs/Float64MultiArray "data: [2.5]"
```

## Directory Structure

```
aae4011_ai_uas/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package manifest
├── setup.py                # Python setup
├── README.md               # This file
├── launch/                 # Launch files
│   ├── demo.launch
│   ├── object_detection.launch
│   └── trajectory_visualizer.launch
├── scripts/                # Python ROS nodes
│   ├── logistic_regression_node.py
│   ├── object_detection_node.py
│   └── trajectory_visualizer.py
├── src/                    # Python modules
│   └── aae4011_ai_uas/
│       ├── __init__.py
│       └── utils.py
├── msg/                    # Custom messages (if any)
└── config/                 # Configuration files
```

## Related Resources

- **Standalone Python Examples**: See `lecture_slide_code/` in the main repository
- **Course Materials**: [AAE4011 GitHub Page](https://github.com/weisongwen/AAE4011-S22526)
- **YOLOv5 Documentation**: [Ultralytics YOLOv5](https://github.com/ultralytics/yolov5)

## Course Information

- **Course**: AAE4011 - Artificial Intelligence for Unmanned Autonomous Systems
- **Semester**: 2, 2025-2026
- **Lecturer**: Dr. Weisong Wen
- **Email**: welson.wen@polyu.edu.hk
- **Department**: Aeronautical and Aviation Engineering (AAE), The Hong Kong Polytechnic University

## License

Copyright (c) 2025-2026  
This code is provided for educational purposes as part of the AAE4011 course.  
All rights reserved.
