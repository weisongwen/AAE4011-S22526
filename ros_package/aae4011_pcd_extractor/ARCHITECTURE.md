# System Architecture - PCD Extractor

## Visual Learning Guide

This document provides visual representations of the ROS package architecture to help students understand how components interact.

---

## 1. Package Structure

```

aae4011_pcd_extractor/          ← Package root directory
│
├── package.xml                  ← Package metadata & dependencies
│   └── Defines: name, version, dependencies, license
│
├── CMakeLists.txt              ← Build configuration
│   └── Defines: how to compile, what to link, where to install
│
├── src/                        ← Source code directory
│   └── pcd_extractor_node.cpp  ← Main C++ node implementation
│
└── launch/                     ← Launch files directory
    └── extract_pcd.launch      ← Launch configuration


```

---

## 2. ROS Node Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  pcd_extractor_node (C++ ROS Node)                          │
│                                                               │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  PCDExtractor Class                                   │   │
│  │                                                        │   │
│  │  Member Variables:                                    │   │
│  │  • ros::NodeHandle nh_                               │   │
│  │  • ros::Subscriber cloud_sub_                        │   │
│  │  • std::string output_dir_                           │   │
│  │  • std::string topic_name_                           │   │
│  │  • int cloud_count_                                   │   │
│  │                                                        │   │
│  │  Methods:                                             │   │
│  │  • Constructor()        → Setup & initialization     │   │
│  │  • cloudCallback()      → Process incoming clouds    │   │
│  │  • Destructor()         → Cleanup                     │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                               │
│  Subscribes to: /velodyne_points (sensor_msgs/PointCloud2)  │
│  Publishes to:  (none in base version)                      │
│  Services:      (none in base version)                      │
│  Parameters:    output_directory, pointcloud_topic          │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. Data Flow Diagram

```
┌──────────────┐         ┌─────────────┐         ┌──────────────────┐
│   Rosbag     │ ──────> │  ROS Topic  │ ──────> │ pcd_extractor_   │
│   Player     │ publish │ /velodyne_  │subscribe│     node         │
│              │         │   points    │         │                  │
└──────────────┘         └─────────────┘         └──────────────────┘
                                                           │
                         sensor_msgs::PointCloud2          │
                                                           │
                                                           ▼
                                                  ┌─────────────────┐
                                                  │  cloudCallback  │
                                                  │    Function     │
                                                  └─────────────────┘
                                                           │
                                                           ▼
                                                  ┌─────────────────┐
                                                  │  Convert ROS    │
                                                  │  to PCL format  │
                                                  │ (fromROSMsg)    │
                                                  └─────────────────┘
                                                           │
                                                           ▼
                                        pcl::PointCloud<pcl::PointXYZI>
                                                           │
                                                           ▼
                                                  ┌─────────────────┐
                                                  │  Generate       │
                                                  │  Filename       │
                                                  │ (timestamp)     │
                                                  └─────────────────┘
                                                           │
                                                           ▼
                                                  ┌─────────────────┐
                                                  │  Save to PCD    │
                                                  │     File        │
                                                  │ (savePCDFile)   │
                                                  └─────────────────┘
                                                           │
                                                           ▼
                                                  ┌─────────────────┐
                                                  │  Disk Storage   │
                                                  │  /tmp/pcd_      │
                                                  │    output/      │
                                                  └─────────────────┘
```

---

## 4. Message Flow Timeline

```
Time ─────────────────────────────────────────────────────────────>

t=0.0s    Rosbag publishes PointCloud2 #1
          │
          ├─> ROS Topic receives message
          │
          ├─> Subscriber calls cloudCallback()
          │   │
          │   ├─> Convert to PCL format
          │   ├─> Save as pointcloud_1234567890.123.pcd
          │   └─> Log success
          │
t=0.1s    Rosbag publishes PointCloud2 #2
          │
          ├─> ROS Topic receives message
          │
          ├─> Subscriber calls cloudCallback()
          │   │
          │   ├─> Convert to PCL format
          │   ├─> Save as pointcloud_1234567890.223.pcd
          │   └─> Log success
          │
t=0.2s    Rosbag publishes PointCloud2 #3
          │
          └─> ... (continues until bag ends)
```

---

## 5. Build Process Flow

```
┌─────────────────┐
│  Run catkin_make│
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────┐
│  CMake reads CMakeLists.txt     │
│  • Finds required packages      │
│  • Configures build settings    │
└────────┬────────────────────────┘
         │
         ▼
┌─────────────────────────────────┐
│  CMake generates Makefiles      │
└────────┬────────────────────────┘
         │
         ▼
┌─────────────────────────────────┐
│  Make compiles source files     │
│  • Preprocesses headers         │
│  • Compiles .cpp to .o objects  │
└────────┬────────────────────────┘
         │
         ▼
┌─────────────────────────────────┐
│  Make links object files        │
│  • Links ROS libraries          │
│  • Links PCL libraries          │
│  • Creates executable           │
└────────┬────────────────────────┘
         │
         ▼
┌─────────────────────────────────┐
│  Executable created at:         │
│  devel/lib/aae4011_pcd_         │
│     extractor/pcd_extractor_    │
│     node                         │
└─────────────────────────────────┘
```

---

## 6. Launch File Execution Flow

```
┌────────────────────────────────────────────────────────┐
│  roslaunch extract_pcd.launch bag_file:=/path/to.bag   │
└───────────────────────┬────────────────────────────────┘
                        │
                        ▼
        ┌───────────────────────────────┐
        │  Parse launch file XML        │
        │  • Load arguments             │
        │  • Resolve parameters         │
        └───────────┬───────────────────┘
                    │
                    ▼
        ┌───────────────────────────────┐
        │  Start ROS Master (roscore)   │
        │  if not already running       │
        └───────────┬───────────────────┘
                    │
                    ▼
        ┌───────────────────────────────┐
        │  Set ROS parameters           │
        │  • output_directory           │
        │  • pointcloud_topic           │
        └───────────┬───────────────────┘
                    │
        ┌───────────┴───────────┐
        │                       │
        ▼                       ▼
┌──────────────────┐   ┌──────────────────┐
│  Start Node 1:   │   │  Start Node 2:   │
│  pcd_extractor_  │   │  rosbag play     │
│  node            │   │                  │
│                  │   │  • Publishes to  │
│  • Loads params  │   │    /velodyne_    │
│  • Creates sub   │   │    points        │
│  • Waits for msg │   │  • Uses --clock  │
└──────────────────┘   └──────────────────┘
        │                       │
        └───────────┬───────────┘
                    │
                    ▼
        ┌───────────────────────────────┐
        │  Both nodes running           │
        │  Messages flowing             │
        │  PCD files being saved        │
        └───────────────────────────────┘
```

---

## 7. ROS Computation Graph

```
                    ┌─────────────┐
                    │  ROS Master │
                    │  (roscore)  │
                    └──────┬──────┘
                           │
                    (registration)
                           │
           ┌───────────────┴───────────────┐
           │                               │
           ▼                               ▼
    ┌─────────────┐                ┌─────────────┐
    │  /player    │                │ /pcd_       │
    │             │                │  extractor_ │
    │ rosbag play │                │  node       │
    └──────┬──────┘                └──────┬──────┘
           │                               │
           │  publishes                    │ subscribes
           │                               │
           └────────────┬──────────────────┘
                        │
                        ▼
              ┌─────────────────┐
              │  /velodyne_     │
              │   points        │
              │                 │
              │ (sensor_msgs/   │
              │  PointCloud2)   │
              └─────────────────┘
```

View this live with:
```bash
rosrun rqt_graph rqt_graph
```

---

## 8. Point Cloud Data Structure

### ROS Message (sensor_msgs::PointCloud2)

```
sensor_msgs::PointCloud2
├── header
│   ├── seq: 0
│   ├── stamp: {sec: 1234567890, nsec: 123456789}
│   └── frame_id: "velodyne"
├── height: 1 (unorganized)
├── width: 100000 (number of points)
├── fields: [x, y, z, intensity]
│   ├── name: "x", offset: 0, datatype: FLOAT32
│   ├── name: "y", offset: 4, datatype: FLOAT32
│   ├── name: "z", offset: 8, datatype: FLOAT32
│   └── name: "intensity", offset: 12, datatype: FLOAT32
├── is_bigendian: false
├── point_step: 16 (bytes per point)
├── row_step: 1600000 (bytes per row)
├── data: [binary data...]
└── is_dense: true
```

### PCL Point Cloud (pcl::PointCloud<pcl::PointXYZI>)

```
pcl::PointCloud<pcl::PointXYZI>
├── header
│   ├── seq: 0
│   ├── stamp: 1234567890123456
│   └── frame_id: "velodyne"
├── points: vector<PointXYZI>
│   ├── [0]: {x: 1.0, y: 2.0, z: 3.0, intensity: 0.5}
│   ├── [1]: {x: 1.1, y: 2.1, z: 3.1, intensity: 0.6}
│   └── ...
├── width: 100000
├── height: 1
└── is_dense: true
```

---

## 9. Callback Execution Model

```
Main Thread                 ROS Spinner Thread
───────────────             ──────────────────

ros::init()
    │
    ▼
NodeHandle nh
    │
    ▼
PCDExtractor(nh)
    │
    ├─> subscribe()  ────────────────┐
    │                                │
    ▼                                │
ros::spin() ─────────────────────┐   │
    │                            │   │
    │                            ▼   ▼
    │                    ┌──────────────────┐
    │                    │  Wait for        │
    │                    │  messages        │
    │                    └────────┬─────────┘
    │                             │
    │                             ▼
    │                    Message arrives
    │                             │
    │                             ▼
    │                    ┌──────────────────┐
    │                    │  Call callback   │
    │                    │  cloudCallback() │
    │                    └────────┬─────────┘
    │                             │
    │                             ▼
    │                    ┌──────────────────┐
    │                    │  Process cloud   │
    │                    │  Save to file    │
    │                    └────────┬─────────┘
    │                             │
    │                             ▼
    │                    ┌──────────────────┐
    │                    │  Return from     │
    │                    │  callback        │
    │                    └────────┬─────────┘
    │                             │
    │                             ▼
    │                    ┌──────────────────┐
    │                    │  Wait for next   │
    │                    │  message         │
    │                    └──────────────────┘
    │                             
    ▼                             
(blocked in spin)
```

---

## 10. File System Interaction

```
┌────────────────────────────────────────────────────────┐
│  Filesystem                                            │
│                                                        │
│  /tmp/pcd_output/                                     │
│  ├── pointcloud_1234567890.123456.pcd    (2.1 MB)    │
│  ├── pointcloud_1234567890.223456.pcd    (2.0 MB)    │
│  ├── pointcloud_1234567890.323456.pcd    (2.2 MB)    │
│  ├── pointcloud_1234567890.423456.pcd    (2.1 MB)    │
│  └── ...                                              │
│                                                        │
│  Each file contains:                                  │
│  ┌──────────────────────────────────────────┐        │
│  │ PCD File Format                          │        │
│  │                                           │        │
│  │ # .PCD v0.7 - Point Cloud Data           │        │
│  │ VERSION 0.7                               │        │
│  │ FIELDS x y z intensity                    │        │
│  │ SIZE 4 4 4 4                              │        │
│  │ TYPE F F F F                              │        │
│  │ COUNT 1 1 1 1                             │        │
│  │ WIDTH 100000                              │        │
│  │ HEIGHT 1                                  │        │
│  │ VIEWPOINT 0 0 0 1 0 0 0                   │        │
│  │ POINTS 100000                             │        │
│  │ DATA binary                               │        │
│  │ [binary point data...]                    │        │
│  └──────────────────────────────────────────┘        │
└────────────────────────────────────────────────────────┘
```

---

## 11. Dependency Graph

```
aae4011_pcd_extractor
        │
        ├─> roscpp (ROS C++ client)
        │     └─> ros_comm (ROS communications)
        │           └─> cpp_common, rostime, etc.
        │
        ├─> sensor_msgs (Message definitions)
        │     └─> std_msgs, geometry_msgs
        │
        ├─> pcl_ros (PCL-ROS bridge)
        │     ├─> PCL (Point Cloud Library)
        │     │     ├─> pcl_common
        │     │     ├─> pcl_io
        │     │     ├─> pcl_filters
        │     │     └─> Eigen (linear algebra)
        │     └─> roscpp, sensor_msgs
        │
        └─> pcl_conversions (ROS<->PCL conversion)
              └─> pcl_ros, sensor_msgs
```

---

## 12. Parameter Flow

```
Launch File                 Parameter Server           Node
───────────                ─────────────────          ────

<param name=               rosparam set               nh_.param<>()
  "output_                   /pcd_extractor_             │
  directory"                 node/output_        ◄───────┘
  value="/tmp/               directory
  pcd_output"/>              = "/tmp/pcd_
       │                     output"
       │                          │
       └──────────────────────────┘
                │
                ▼
        Used by node at
        runtime to determine
        where to save files
```

---

## 13. Error Handling Flow

```
cloudCallback(msg)
    │
    ▼
┌─────────────────┐
│ Validate message│
└────────┬────────┘
         │
         ▼
    Valid? ───No──> Log error, return
         │
         Yes
         │
         ▼
┌─────────────────┐
│ Convert to PCL  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Generate        │
│ filename        │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Save PCD file   │
└────────┬────────┘
         │
         ▼
    Success? ───No──> Log error
         │              │
         Yes            │
         │              │
         ▼              ▼
    Log success    Continue anyway
         │              │
         └──────┬───────┘
                │
                ▼
         Wait for next message
```

---

## 14. Learning Path Visualization

```
Start Here
    │
    ▼
┌─────────────────────────┐
│ 1. Understand ROS       │
│    Package Structure    │
│    (README Part 1-2)    │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 2. Study package.xml    │
│    (Dependencies)       │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 3. Study CMakeLists.txt │
│    (Build System)       │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 4. Study C++ node       │
│    (ROS Programming)    │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 5. Study launch file    │
│    (Integration)        │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 6. Build and test       │
│    (Hands-on Practice)  │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 7. Extend with          │
│    Exercises            │
└─────────────────────────┘
```

---

## Quick Reference: Key Files and Their Roles

| File | Purpose | When to Edit |
|------|---------|--------------|
| `package.xml` | Package metadata & dependencies | Adding new libraries |
| `CMakeLists.txt` | Build configuration | Changing compilation |
| `src/*.cpp` | Implementation code | Adding features |
| `launch/*.launch` | Startup configuration | Changing parameters |

---

## Useful Commands for Visualization

```bash
# View computation graph
rosrun rqt_graph rqt_graph

# Monitor topics
rostopic list
rostopic hz /velodyne_points
rostopic echo /velodyne_points

# Check node info
rosnode info /pcd_extractor_node

# View parameters
rosparam list
rosparam get /pcd_extractor_node/output_directory

# View TF tree (if using transforms)
rosrun rqt_tf_tree rqt_tf_tree

# View logs
rqt_console
```

---

This architecture document should help you visualize how all the components work together!
