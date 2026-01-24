# Setting Up AAE4011 ROS Package in WSL2 + Ubuntu 20.04

This guide explains how to set up the `aae4011_ai_uas` ROS package in your WSL2 environment with Ubuntu 20.04 and ROS Noetic.

## Prerequisites

- Windows 10/11 with WSL2 enabled
- Ubuntu 20.04 installed in WSL2
- ROS Noetic installed (instructions below if not installed)

## Step-by-Step Setup

### Step 1: Open WSL2 Terminal

Open Windows Terminal or PowerShell and run:
```bash
wsl
```

### Step 2: Install ROS Noetic (if not already installed)

If you don't have ROS Noetic installed, run these commands:

```bash
# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update and install ROS Noetic (full desktop version)
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# Initialize rosdep
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update

# Add ROS to bashrc (auto-source on terminal open)
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Create Catkin Workspace

```bash
# Create workspace directory
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# Initialize the workspace
catkin_make

# Source the workspace
source devel/setup.bash
```

### Step 4: Copy Package from Windows to WSL2

The Windows filesystem is accessible from WSL2 at `/mnt/c/`, `/mnt/d/`, etc.

```bash
# Navigate to catkin workspace src folder
cd ~/catkin_ws/src

# Copy the package from Windows
# Replace the path with your actual Windows path
# Example: If package is at C:\Users\Administrator\Documents\AAE4011-S22526\ros_package\aae4011_ai_uas

cp -r /mnt/c/Users/Administrator/Documents/AAE4011-S22526/ros_package/aae4011_ai_uas .

# Or create a symbolic link (not recommended for ROS packages)
# ln -s /mnt/c/Users/Administrator/Documents/AAE4011-S22526/ros_package/aae4011_ai_uas .
```

### Step 5: Install Dependencies

```bash
# Install ROS dependencies
sudo apt-get update
sudo apt-get install -y ros-noetic-sensor-msgs ros-noetic-std-msgs python3-pip

# Install Python dependencies
pip3 install numpy
```

### Step 6: Make Scripts Executable

```bash
chmod +x ~/catkin_ws/src/aae4011_ai_uas/scripts/*.py
```

### Step 7: Build the Workspace

```bash
cd ~/catkin_ws
catkin_make
```

### Step 8: Source the Workspace

```bash
source ~/catkin_ws/devel/setup.bash

# Add to bashrc for automatic sourcing
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Using the Package

### Test the Installation

```bash
# Check if package is found
rospack find aae4011_ai_uas

# List available launch files
roslaunch aae4011_ai_uas --files
```

### Run the Point Cloud Analyzer

```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Run the analyzer (default topic: /velodyne_points)
roslaunch aae4011_ai_uas pointcloud_analyzer.launch

# Or with custom topic
roslaunch aae4011_ai_uas pointcloud_analyzer.launch pointcloud_topic:=/your_lidar_topic
```

### Test with a Rosbag

If you have a rosbag file with point cloud data:

```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Play the rosbag
rosbag play /path/to/your/lidar_data.bag

# Terminal 3: Run the analyzer
roslaunch aae4011_ai_uas pointcloud_analyzer.launch pointcloud_topic:=/velodyne_points

# Terminal 4: View the output
rostopic echo /pointcloud_stats
```

## Quick Setup Script

For convenience, you can use the provided setup script:

```bash
# Navigate to the package scripts folder
cd /mnt/c/Users/Administrator/Documents/AAE4011-S22526/ros_package/aae4011_ai_uas/scripts

# Make the script executable
chmod +x setup_wsl2.sh

# Run the setup script
./setup_wsl2.sh
```

## Troubleshooting

### Issue: "rospack find" returns error
```bash
# Make sure workspace is sourced
source ~/catkin_ws/devel/setup.bash

# Rebuild if needed
cd ~/catkin_ws
catkin_make
```

### Issue: Permission denied when running scripts
```bash
chmod +x ~/catkin_ws/src/aae4011_ai_uas/scripts/*.py
```

### Issue: Package not found after build
```bash
# Check if package.xml exists
ls ~/catkin_ws/src/aae4011_ai_uas/package.xml

# Rebuild workspace
cd ~/catkin_ws
catkin_make --force-cmake
source devel/setup.bash
```

### Issue: Import errors in Python scripts
```bash
# Install missing Python packages
pip3 install numpy

# Make sure ROS Python packages are available
source /opt/ros/noetic/setup.bash
```

### Issue: Cannot access Windows files
```bash
# Windows C: drive is at /mnt/c/
ls /mnt/c/Users/

# If you see permission issues, try:
sudo mount -t drvfs C: /mnt/c -o metadata
```

## File Paths Reference

| Windows Path | WSL2 Path |
|--------------|-----------|
| `C:\Users\Administrator\Documents\` | `/mnt/c/Users/Administrator/Documents/` |
| `D:\Data\` | `/mnt/d/Data/` |

## Useful Commands

```bash
# Check ROS version
rosversion -d

# List all ROS packages
rospack list

# Check if roscore is running
rostopic list

# View point cloud topics
rostopic list | grep -i point

# Check message type
rostopic info /velodyne_points
```

## Contact

For questions about this setup, contact:
- **Lecturer**: Dr. Weisong Wen
- **Email**: welson.wen@polyu.edu.hk
