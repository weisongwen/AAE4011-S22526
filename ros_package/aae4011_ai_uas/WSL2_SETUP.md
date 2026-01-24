# Setting Up AAE4011 ROS Package in WSL2 + Ubuntu 20.04

This guide explains how to set up the `aae4011_ai_uas` ROS package in your WSL2 environment with Ubuntu 20.04 and ROS Noetic. This setup assumes you are using Cursor IDE in WSL2 to directly clone and work with the repository.

## Workflow Overview

The setup process follows these steps:

1. **Clone the repository** from GitHub directly in WSL2 using Cursor or git command
2. **Create a catkin workspace** (if not already created)
3. **Link the ROS package** to the catkin workspace
4. **Install dependencies** (ROS packages and Python libraries)
5. **Build the workspace** using `catkin_make`
6. **Run the Python scripts** in WSL2+ROS environment

This approach allows you to:
- Work directly with the repository in WSL2
- Use Cursor IDE for code editing
- Keep the repository in sync with GitHub
- Compile and run ROS nodes seamlessly

## Prerequisites

- Windows 10/11 with WSL2 enabled
- Ubuntu 20.04 installed in WSL2
- ROS Noetic installed (instructions below if not installed)
- Git installed in WSL2
- Cursor IDE (or any code editor) running in WSL2

## Step-by-Step Setup

### Step 1: Open WSL2 Terminal

Open Windows Terminal or PowerShell and run:
```bash
wsl
```

Or open Cursor IDE directly in WSL2 environment.

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

### Step 4: Clone the Repository from GitHub

Clone the entire repository directly in WSL2:

```bash
# Navigate to catkin workspace src folder
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/weisongwen/AAE4011-S22526.git

# Create symbolic link to the ROS package (recommended for ROS packages)
cd ~/catkin_ws/src
ln -s AAE4011-S22526/ros_package/aae4011_ai_uas .

# Verify the package is linked correctly
ls -la ~/catkin_ws/src/aae4011_ai_uas
```

**Note:** Using a symbolic link allows you to work directly with the cloned repository while keeping it in the catkin workspace structure.

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

A test rosbag file is available for testing the point cloud analyzer:

**Rosbag Information:**
- **Filename:** `campus_small_dataset.bag`
- **Download Link:** [Google Drive](https://drive.google.com/file/d/160zAWgFxbWGUAKMiI8tJcmSgSmBex5ZC/view?usp=drive_link)

**Download the Rosbag:**

You can download the rosbag file in WSL2 using one of these methods:

**Method 1: Using gdown (Recommended)**
```bash
# Install gdown if not already installed
pip3 install gdown

# Download the rosbag (extract file ID from Google Drive link)
gdown --id 160zAWgFxbWGUAKMiI8tJcmSgSmBex5ZC -O campus_small_dataset.bag

# Move to a convenient location (optional)
mkdir -p ~/rosbags
mv campus_small_dataset.bag ~/rosbags/
```

**Method 2: Using wget (if direct download link is available)**
```bash
# Create directory for rosbags
mkdir -p ~/rosbags
cd ~/rosbags

# Download using wget (you may need to get the direct download link from Google Drive)
# Note: For Google Drive, you may need to use gdown instead
wget --no-check-certificate 'https://drive.google.com/uc?export=download&id=160zAWgFxbWGUAKMiI8tJcmSgSmBex5ZC' -O campus_small_dataset.bag
```

**Method 3: Manual Download**
1. Open the [Google Drive link](https://drive.google.com/file/d/160zAWgFxbWGUAKMiI8tJcmSgSmBex5ZC/view?usp=drive_link) in your browser
2. Download `campus_small_dataset.bag` to your Windows Downloads folder
3. Copy it to WSL2:
```bash
# Copy from Windows Downloads to WSL2
mkdir -p ~/rosbags
cp /mnt/c/Users/$(whoami)/Downloads/campus_small_dataset.bag ~/rosbags/
```

**Run the Test:**

Once you have the rosbag file, test the point cloud analyzer:

```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Play the rosbag
rosbag play ~/rosbags/campus_small_dataset.bag

# Terminal 3: Run the analyzer
roslaunch aae4011_ai_uas pointcloud_analyzer.launch pointcloud_topic:=/velodyne_points

# Terminal 4: View the output
rostopic echo /pointcloud_stats
```

**Note:** Make sure the point cloud topic name matches. You can check available topics in the rosbag:
```bash
# Check topics in the rosbag
rosbag info ~/rosbags/campus_small_dataset.bag
```

## Quick Setup Script

For convenience, you can use the provided setup script after cloning the repository:

```bash
# Navigate to the package scripts folder
cd ~/catkin_ws/src/aae4011_ai_uas/scripts

# Make the script executable
chmod +x setup_wsl2.sh

# Run the setup script
./setup_wsl2.sh
```

**Note:** The setup script will automatically detect the package location and set up the workspace accordingly.

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

### Issue: Git clone fails
```bash
# Make sure git is installed
sudo apt install git -y

# Check internet connection
ping -c 3 github.com

# If using proxy, configure git
git config --global http.proxy http://proxy.example.com:port
```

### Issue: Symbolic link not working
```bash
# Verify the link exists
ls -la ~/catkin_ws/src/aae4011_ai_uas

# If link is broken, remove and recreate
rm ~/catkin_ws/src/aae4011_ai_uas
cd ~/catkin_ws/src
ln -s AAE4011-S22526/ros_package/aae4011_ai_uas .

# Or copy the package instead (if symlink doesn't work)
cp -r AAE4011-S22526/ros_package/aae4011_ai_uas .
```

## Working with the Repository

### Updating the Code

Since you cloned the repository directly, you can easily update it:

```bash
# Navigate to the repository
cd ~/catkin_ws/src/AAE4011-S22526

# Pull latest changes
git pull origin main

# Rebuild the workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Running Python Scripts Directly

You can also run the Python scripts directly in WSL2 without ROS:

```bash
# Navigate to the script directory
cd ~/catkin_ws/src/AAE4011-S22526/ros_package/aae4011_ai_uas/scripts

# Run the script directly (if it doesn't require ROS)
python3 pointcloud_analyzer.py
```

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

# Check git repository status
cd ~/catkin_ws/src/AAE4011-S22526
git status
```

## Contact

For questions about this setup, contact:
- **Lecturer**: Dr. Weisong Wen
- **Email**: welson.wen@polyu.edu.hk
