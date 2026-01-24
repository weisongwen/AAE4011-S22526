# AAE4011 AI for Unmanned Autonomous Systems - ROS Package

This ROS package contains a point cloud analyzer for 3D LiDAR data analysis, developed for the AAE4011 course at The Hong Kong Polytechnic University.

## Package Overview

**Package Name:** `aae4011_ai_uas`  
**Version:** 1.0.0  
**ROS Version:** ROS 1 (Noetic)  
**Maintainer:** Dr. Weisong Wen (welson.wen@polyu.edu.hk)

## Features

- **Point Cloud Analysis**: Analyze 3D point cloud data from LiDAR sensors
- **Point Count**: Count the number of points in each point cloud message
- **Frequency Analysis**: Calculate the frequency of incoming point cloud messages
- **Statistics**: Compute min, max, mean, and standard deviation for X, Y, Z coordinates
- **Bounding Box**: Calculate the spatial extent of the point cloud

## Installation

### Prerequisites

1. **ROS Noetic** (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04)
2. **Python 3** with numpy
3. **catkin workspace**

### Install Dependencies

```bash
# Install ROS dependencies
sudo apt-get install ros-noetic-sensor-msgs ros-noetic-std-msgs

# Install Python dependencies
pip3 install numpy
```

### Build the Package

```bash
# Navigate to your catkin workspace
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/weisongwen/AAE4011-S22526.git

# Create symlink to the ROS package
ln -s AAE4011-S22526/ros_package/aae4011_ai_uas .

# Build
cd ~/catkin_ws
catkin_make

# Source the workspace
source devel/setup.bash
```

## Node: pointcloud_analyzer.py

Analyzes 3D point cloud data and publishes statistics.

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/points_raw` (configurable) | sensor_msgs/PointCloud2 | Input 3D point cloud |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/pointcloud_stats` | std_msgs/String | JSON formatted statistics |
| `/pointcloud_num_points` | std_msgs/Int32 | Number of points in the point cloud |
| `/pointcloud_frequency` | std_msgs/Float64 | Message frequency in Hz |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~pointcloud_topic` | string | `/points_raw` | Input point cloud topic |
| `~window_size` | int | 10 | Number of messages for frequency calculation |

### Output Statistics (JSON)

The `/pointcloud_stats` topic publishes JSON with the following fields:

```json
{
  "timestamp": 1234567890.123,
  "message_count": 100,
  "num_points": 65536,
  "frequency_hz": 10.0,
  "total_points_received": 6553600,
  "x": {
    "min": -50.0,
    "max": 50.0,
    "mean": 0.5,
    "std": 15.2
  },
  "y": {
    "min": -50.0,
    "max": 50.0,
    "mean": -0.3,
    "std": 14.8
  },
  "z": {
    "min": -2.0,
    "max": 10.0,
    "mean": 1.2,
    "std": 2.5
  },
  "bounding_box": {
    "x_range": 100.0,
    "y_range": 100.0,
    "z_range": 12.0
  }
}
```

## Usage

### Using Launch File

The launch file starts the point cloud analyzer **and** RViz to visualize the LiDAR point cloud (fixed frame: `velodyne`, topic: `/points_raw`).

```bash
# Default: analyzer + RViz (subscribes to /points_raw, matches campus_small_dataset.bag)
roslaunch aae4011_ai_uas pointcloud_analyzer.launch

# With custom topic
roslaunch aae4011_ai_uas pointcloud_analyzer.launch pointcloud_topic:=/lidar/points

# Without RViz
roslaunch aae4011_ai_uas pointcloud_analyzer.launch launch_rviz:=false

# With custom topic and window size
roslaunch aae4011_ai_uas pointcloud_analyzer.launch pointcloud_topic:=/camera/depth/points window_size:=20
```

### Using rosrun

```bash
# Start roscore
roscore

# Run the node
rosrun aae4011_ai_uas pointcloud_analyzer.py _pointcloud_topic:=/points_raw
```

### View Output

```bash
# View statistics (JSON)
rostopic echo /pointcloud_stats

# View number of points
rostopic echo /pointcloud_num_points

# View frequency
rostopic echo /pointcloud_frequency
```

## Example with Rosbag

A test rosbag file is available for testing:

**Test Rosbag:**
- **Filename:** `campus_small_dataset.bag`
- **Download:** [Google Drive](https://drive.google.com/file/d/160zAWgFxbWGUAKMiI8tJcmSgSmBex5ZC/view?usp=drive_link)

**Download using gdown:**
```bash
pip3 install gdown
gdown --id 160zAWgFxbWGUAKMiI8tJcmSgSmBex5ZC -O campus_small_dataset.bag
```

**Run the example:**
```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Play the rosbag
rosbag play campus_small_dataset.bag

# Terminal 3: Run analyzer + RViz (visualizes LiDAR; frame_id: velodyne)
roslaunch aae4011_ai_uas pointcloud_analyzer.launch

# Terminal 4: View results
rostopic echo /pointcloud_stats
```
RViz shows the point cloud from `/points_raw` in the `velodyne` frame.

**Note:** `campus_small_dataset.bag` publishes point clouds on `/points_raw`. Check topic names with:
```bash
rosbag info campus_small_dataset.bag
```

## Common Point Cloud Topics

| Sensor / Dataset | Common Topic |
|------------------|-------------|
| campus_small_dataset.bag | `/points_raw` |
| Velodyne LiDAR | `/velodyne_points` |
| Ouster LiDAR | `/ouster/points` |
| Livox LiDAR | `/livox/lidar` |
| Intel RealSense | `/camera/depth/points` |
| Generic | `/points`, `/lidar/points` |

## Directory Structure

```
aae4011_ai_uas/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package manifest
├── setup.py                # Python setup
├── README.md               # This file
├── launch/
│   └── pointcloud_analyzer.launch
├── scripts/
│   └── pointcloud_analyzer.py
├── src/
│   └── aae4011_ai_uas/
│       ├── __init__.py
│       └── utils.py
├── msg/                    # Custom messages (if any)
└── config/                 # Configuration files
```

## Troubleshooting

### No data received
- Check if the point cloud topic exists: `rostopic list | grep points`
- Verify topic name: `rostopic info /your_topic`
- Check message type: Should be `sensor_msgs/PointCloud2`

### Low frequency reported
- Ensure the LiDAR is running at expected rate
- Check for network delays if using remote sensors
- Verify no other nodes are consuming too much bandwidth

### Import errors
- Ensure numpy is installed: `pip3 install numpy`
- Source your workspace: `source ~/catkin_ws/devel/setup.bash`

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
