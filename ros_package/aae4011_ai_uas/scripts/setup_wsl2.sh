#!/bin/bash
# =============================================================================
# AAE4011 - AI for Unmanned Autonomous Systems
# WSL2 + Ubuntu 20.04 + ROS Noetic Setup Script
# =============================================================================
#
# This script sets up the aae4011_ai_uas ROS package in WSL2 environment.
#
# Prerequisites:
#   - WSL2 with Ubuntu 20.04 installed
#   - ROS Noetic installed (if not, this script will guide you)
#
# Usage:
#   chmod +x setup_wsl2.sh
#   ./setup_wsl2.sh
#
# =============================================================================

set -e  # Exit on error

echo "============================================================"
echo "AAE4011 ROS Package Setup for WSL2 + Ubuntu 20.04"
echo "============================================================"

# =============================================================================
# Step 1: Check if ROS Noetic is installed
# =============================================================================
echo ""
echo "[Step 1] Checking ROS Noetic installation..."

if [ -f "/opt/ros/noetic/setup.bash" ]; then
    echo "✓ ROS Noetic is installed."
    source /opt/ros/noetic/setup.bash
else
    echo "✗ ROS Noetic is NOT installed."
    echo ""
    echo "Please install ROS Noetic first by running these commands:"
    echo ""
    echo "  # Setup sources.list"
    echo "  sudo sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu focal main\" > /etc/apt/sources.list.d/ros-latest.list'"
    echo ""
    echo "  # Setup keys"
    echo "  sudo apt install curl -y"
    echo "  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -"
    echo ""
    echo "  # Install ROS Noetic"
    echo "  sudo apt update"
    echo "  sudo apt install ros-noetic-desktop-full -y"
    echo ""
    echo "  # Initialize rosdep"
    echo "  sudo apt install python3-rosdep -y"
    echo "  sudo rosdep init"
    echo "  rosdep update"
    echo ""
    echo "  # Add ROS to bashrc"
    echo "  echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc"
    echo "  source ~/.bashrc"
    echo ""
    echo "After installing ROS Noetic, run this script again."
    exit 1
fi

# =============================================================================
# Step 2: Create catkin workspace if it doesn't exist
# =============================================================================
echo ""
echo "[Step 2] Setting up catkin workspace..."

CATKIN_WS=~/catkin_ws

if [ ! -d "$CATKIN_WS/src" ]; then
    echo "Creating catkin workspace at $CATKIN_WS..."
    mkdir -p $CATKIN_WS/src
    cd $CATKIN_WS
    catkin_make
    echo "✓ Catkin workspace created."
else
    echo "✓ Catkin workspace already exists at $CATKIN_WS"
fi

# =============================================================================
# Step 3: Copy package to catkin workspace
# =============================================================================
echo ""
echo "[Step 3] Copying package to catkin workspace..."

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
PACKAGE_NAME="aae4011_ai_uas"

# Check if package already exists in workspace
if [ -d "$CATKIN_WS/src/$PACKAGE_NAME" ]; then
    echo "Package already exists in workspace. Updating..."
    rm -rf $CATKIN_WS/src/$PACKAGE_NAME
fi

# Copy the package
cp -r $PACKAGE_DIR $CATKIN_WS/src/
echo "✓ Package copied to $CATKIN_WS/src/$PACKAGE_NAME"

# =============================================================================
# Step 4: Install dependencies
# =============================================================================
echo ""
echo "[Step 4] Installing dependencies..."

sudo apt-get update
sudo apt-get install -y python3-pip python3-numpy ros-noetic-sensor-msgs ros-noetic-std-msgs

# Install Python dependencies
pip3 install numpy

echo "✓ Dependencies installed."

# =============================================================================
# Step 5: Make Python scripts executable
# =============================================================================
echo ""
echo "[Step 5] Making scripts executable..."

chmod +x $CATKIN_WS/src/$PACKAGE_NAME/scripts/*.py
echo "✓ Scripts are now executable."

# =============================================================================
# Step 6: Build the workspace
# =============================================================================
echo ""
echo "[Step 6] Building catkin workspace..."

cd $CATKIN_WS
source /opt/ros/noetic/setup.bash
catkin_make

echo "✓ Workspace built successfully."

# =============================================================================
# Step 7: Add workspace to bashrc
# =============================================================================
echo ""
echo "[Step 7] Configuring environment..."

if ! grep -q "source $CATKIN_WS/devel/setup.bash" ~/.bashrc; then
    echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc
    echo "✓ Added workspace to ~/.bashrc"
else
    echo "✓ Workspace already in ~/.bashrc"
fi

source $CATKIN_WS/devel/setup.bash

# =============================================================================
# Done!
# =============================================================================
echo ""
echo "============================================================"
echo "Setup Complete!"
echo "============================================================"
echo ""
echo "To use the package, run:"
echo "  source ~/.bashrc"
echo ""
echo "To test the package:"
echo "  roslaunch aae4011_ai_uas pointcloud_analyzer.launch"
echo ""
echo "Or with a custom topic:"
echo "  roslaunch aae4011_ai_uas pointcloud_analyzer.launch pointcloud_topic:=/your_topic"
echo ""
echo "============================================================"
