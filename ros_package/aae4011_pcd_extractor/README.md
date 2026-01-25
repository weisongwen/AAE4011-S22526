# ROS Point Cloud Extractor Package Tutorial

## Overview
This tutorial will guide you through creating a ROS package from scratch that reads 3D point clouds from a rosbag file and saves them as PCD (Point Cloud Data) files. This is a hands-on introduction to ROS package development for undergraduate students.

## Learning Objectives
By completing this tutorial, you will learn:
1. How to create and structure a ROS package
2. Understanding of `package.xml` and `CMakeLists.txt`
3. How to write a ROS node in C++
4. How to subscribe to ROS topics
5. How to process sensor_msgs/PointCloud2 messages
6. How to use PCL (Point Cloud Library) with ROS
7. How to create and use launch files

---

## Prerequisites
- ROS (Kinetic/Melodic/Noetic) installed
- Basic C++ knowledge
- PCL library installed
- A rosbag file containing PointCloud2 messages

To install required dependencies:
```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions
```

---

## Part 1: Understanding ROS Package Structure

A typical ROS package has the following structure:
```
aae4011_pcd_extractor/
├── CMakeLists.txt          # Build configuration file (tells catkin how to build)
├── package.xml             # Package metadata (dependencies, version, author)
├── launch/                 # Launch files to start nodes
│   └── extract_pcd.launch
├── src/                    # Source code directory
│   └── pcd_extractor_node.cpp
└── README.md               # Documentation
```

---

## Part 2: Step-by-Step Package Creation

### Step 1: Create the Package Directory Structure

Navigate to your catkin workspace source directory and create the package structure:

```bash
# Navigate to your workspace source directory
cd ~/catkin_ws/src  # Replace with your workspace path

# Create the package directory
mkdir -p aae4011_pcd_extractor/src
mkdir -p aae4011_pcd_extractor/launch

# Navigate into the package
cd aae4011_pcd_extractor
```

### Step 2: Create package.xml

The `package.xml` file contains metadata about your package including:
- Package name, version, and description
- Maintainer and license information
- Dependencies (other packages this package needs)

Create the file with detailed annotations (see `package.xml` in this directory).

**Key Concepts:**
- `<buildtool_depend>`: Tools needed to build the package (always catkin for ROS)
- `<build_depend>`: Packages needed at compile time
- `<exec_depend>`: Packages needed at runtime
- `<depend>`: Packages needed at both build and runtime

### Step 3: Create CMakeLists.txt

The `CMakeLists.txt` file tells the build system (catkin) how to compile your code. It includes:
- Finding required packages
- Specifying include directories
- Declaring executables
- Linking libraries

Create the file with detailed annotations (see `CMakeLists.txt` in this directory).

**Key Concepts:**
- `find_package()`: Locate required packages
- `include_directories()`: Tell compiler where to find header files
- `add_executable()`: Define what executable to build
- `target_link_libraries()`: Link necessary libraries to your executable

### Step 4: Write the C++ Node

Create `src/pcd_extractor_node.cpp`. This is the main program that:
1. Initializes a ROS node
2. Subscribes to a PointCloud2 topic
3. Converts ROS messages to PCL format
4. Saves point clouds to PCD files

See the annotated source file for detailed explanations of each section.

**Key Concepts:**
- **ROS Node**: An executable that communicates with other nodes
- **Subscriber**: Receives messages from a topic
- **Callback Function**: Function called when a new message arrives
- **Topic**: Named channel for message passing
- **Message Type**: sensor_msgs::PointCloud2 (ROS format for point clouds)

### Step 5: Create Launch File

Create `launch/extract_pcd.launch`. Launch files allow you to:
- Start multiple nodes with one command
- Set parameters
- Remap topics
- Configure node arguments

See the annotated launch file for detailed explanations.

### Step 6: Build the Package

Now compile your package:

```bash
# Navigate to your catkin workspace root
cd ~/catkin_ws  # Replace with your workspace path

# Build the package
catkin_make

# Or if using catkin build:
# catkin build aae4011_pcd_extractor

# Source the workspace to make your package available
source devel/setup.bash
```

**What happens during build:**
1. CMake reads `CMakeLists.txt` and `package.xml`
2. Checks all dependencies are available
3. Compiles C++ source code
4. Links libraries
5. Creates executable in `devel/lib/aae4011_pcd_extractor/`

---

## Part 3: Running the Package

### Method 1: Using Launch File (Recommended)

The launch file allows you to configure everything in one place:

```bash
# Make sure you've sourced your workspace
source ~/catkin_ws/devel/setup.bash

# Run with a rosbag file
roslaunch aae4011_pcd_extractor extract_pcd.launch \
    bag_file:=/path/to/your/bagfile.bag \
    topic:=/your/pointcloud/topic \
    output_dir:=/path/to/output/directory
```

**Parameters:**
- `bag_file`: Path to your rosbag file
- `topic`: The topic name containing PointCloud2 messages (default: `/velodyne_points`)
- `output_dir`: Where to save PCD files (default: `/tmp/pcd_output`)

### Method 2: Manual Execution (For Understanding)

This method helps you understand what happens step-by-step:

```bash
# Terminal 1: Start ROS master
roscore

# Terminal 2: Run your node
rosrun aae4011_pcd_extractor pcd_extractor_node \
    _pointcloud_topic:=/velodyne_points \
    _output_directory:=/tmp/pcd_output

# Terminal 3: Play the rosbag
rosbag play /path/to/your/bagfile.bag
```

### Verifying Output

Check that PCD files are being created:

```bash
ls -lh /tmp/pcd_output/
# You should see files like:
# pointcloud_1234567890.123456.pcd
# pointcloud_1234567891.234567.pcd
# ...
```

View a PCD file using PCL tools:

```bash
# Install pcl-tools if not already installed
sudo apt-get install pcl-tools

# View a point cloud
pcl_viewer /tmp/pcd_output/pointcloud_1234567890.123456.pcd
```

---

## Part 4: Understanding the Code Flow

### Execution Flow Diagram

```
1. roslaunch starts ROS master and nodes
         ↓
2. pcd_extractor_node initializes
         ↓
3. Subscribes to PointCloud2 topic
         ↓
4. rosbag player publishes messages
         ↓
5. Callback function receives each message
         ↓
6. Convert ROS message → PCL format
         ↓
7. Save to PCD file with timestamp
         ↓
8. Log success message
         ↓
9. Repeat for next message
```

### Message Flow

```
rosbag → /velodyne_points (topic) → Subscriber → Callback
                                                      ↓
                                              PCL Conversion
                                                      ↓
                                              Save to File
```

---

## Part 5: Common Issues and Troubleshooting

### Issue 1: Package Not Found
**Error:** `[roslaunch] ROS path [0]=/opt/ros/noetic/share/ros ... Cannot locate [aae4011_pcd_extractor]`

**Solution:**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Issue 2: No Messages Received
**Error:** Node runs but no PCD files are created

**Diagnosis:**
```bash
# Check available topics in the bag
rosbag info /path/to/your/bagfile.bag

# Check if topic name matches
rostopic list

# Echo the topic to see if messages are being published
rostopic echo /velodyne_points
```

**Solution:** Make sure the topic name in your launch file matches the actual topic in the bag.

### Issue 3: Permission Denied Writing Files
**Error:** `Cannot save PCD file`

**Solution:**
```bash
# Ensure output directory exists and is writable
mkdir -p /tmp/pcd_output
chmod 777 /tmp/pcd_output
```

### Issue 4: Missing Dependencies
**Error:** `fatal error: pcl/point_cloud.h: No such file or directory`

**Solution:**
```bash
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions libpcl-dev
```

---

## Part 6: Exercises for Students

### Exercise 1: Modify Save Frequency
**Task:** Modify the node to save only every 10th point cloud instead of every one.

**Hint:** Add a counter variable and use modulo operator.

### Exercise 2: Add Downsampling
**Task:** Before saving, downsample the point cloud using VoxelGrid filter.

**Hint:** Look up `pcl::VoxelGrid` and add filtering before saving.

### Exercise 3: Add Statistics
**Task:** Print statistics (number of points) for each cloud before saving.

**Hint:** Use `cloud->points.size()` to get point count.

### Exercise 4: Custom File Naming
**Task:** Allow users to specify a custom prefix for saved files via parameter.

**Hint:** Add a new ROS parameter in the constructor.

### Exercise 5: Add Service
**Task:** Create a service to start/stop saving dynamically.

**Hint:** Look up `ros::ServiceServer` and create a std_srvs::SetBool service.

---

## Part 7: Key Takeaways

### What You Learned

1. **ROS Package Structure:**
   - `package.xml`: Defines package metadata and dependencies
   - `CMakeLists.txt`: Configures the build process
   - `src/`: Contains source code
   - `launch/`: Contains launch files

2. **ROS Concepts:**
   - **Nodes**: Executable programs that perform computation
   - **Topics**: Named buses for message passing
   - **Messages**: Data structures for inter-node communication
   - **Subscribers**: Receive messages from topics
   - **Callbacks**: Functions triggered when messages arrive

3. **Build System:**
   - `catkin_make`: Builds all packages in workspace
   - Dependency resolution
   - Header file inclusion
   - Library linking

4. **Point Cloud Processing:**
   - ROS ↔ PCL conversion
   - File I/O with PCL
   - Timestamp handling

### Best Practices Demonstrated

- ✅ Clear code organization
- ✅ Comprehensive comments
- ✅ Error handling
- ✅ Parameterization (using ROS parameters)
- ✅ Logging (ROS_INFO, ROS_WARN, ROS_ERROR)
- ✅ Resource management (directory creation)

---

## Part 8: Further Reading and Resources

### Official Documentation
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
- [Writing a Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
- [PCL-ROS Integration](http://wiki.ros.org/pcl_ros)

### Additional Topics to Explore
- ROS Publishers (publishing processed data)
- ROS Services (request-response communication)
- ROS Actions (long-running tasks with feedback)
- Dynamic Reconfigure (changing parameters at runtime)
- Nodelets (running multiple nodes in one process)
- TF (coordinate frame transformations)

### Next Steps
1. Try the exercises above
2. Modify the code to publish processed clouds back to ROS
3. Add visualization using RViz
4. Create a more complex pipeline with multiple nodes
5. Learn about ROS 2 (the next generation)

---

## Part 9: Quick Reference

### Common Commands

```bash
# Build package
catkin_make

# Source workspace
source devel/setup.bash

# Run node directly
rosrun aae4011_pcd_extractor pcd_extractor_node

# Run with launch file
roslaunch aae4011_pcd_extractor extract_pcd.launch

# Check if node is running
rosnode list

# Check published topics
rostopic list

# Get info about a topic
rostopic info /velodyne_points

# Examine rosbag contents
rosbag info bagfile.bag

# View PCD file
pcl_viewer pointcloud.pcd
```

### File Locations After Build

- **Executable**: `~/catkin_ws/devel/lib/aae4011_pcd_extractor/pcd_extractor_node`
- **Setup file**: `~/catkin_ws/devel/setup.bash`
- **Build files**: `~/catkin_ws/build/aae4011_pcd_extractor/`

---

## Contact and Support

If you have questions or encounter issues:
1. Check the troubleshooting section above
2. Review ROS wiki documentation
3. Ask on [ROS Answers](https://answers.ros.org/)
4. Check [ROS Discourse](https://discourse.ros.org/)

---

## License

This tutorial package is provided for educational purposes.

---

**Happy Learning!** 🤖🎓
