# LIO-SAM Student Quick Start Guide

This guide is for the AAE4011 course and will help you compile and run LIO-SAM in your local catkin_ws.

---

## 1. Clone the LIO-SAM Repository

Open a terminal in the `catkin_ws/src` directory and run:

```bash
cd /home/mint/dev/code/catkin_ws/src
# Clone the official LIO-SAM repository
git clone https://github.com/TixiaoShan/LIO-SAM.git
```

---

## 2. Build the Catkin Workspace

In the `catkin_ws` directory, run:

```bash
cd /home/mint/dev/code/catkin_ws
catkin_make
# After building, source the environment
source devel/setup.bash
```

If you encounter missing dependencies, install them with:

```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 3. Run LIO-SAM

Go to the LIO-SAM launch directory and run the main launch file:

```bash
roslaunch LIO-SAM run.launch
```

To play a rosbag file:

```bash
rosbag play your_data.bag
```

---

## 4. RViz Visualization

LIO-SAM will automatically start RViz. You can also open it manually:

```bash
rviz
```

In RViz, add the following topics for visualization:
- `/lio_sam/mapping/odometry`
- `/lio_sam/mapping/path`
- `/lio_sam/mapping/map_global`

---

## 5. Common Issues

- **Package/command not found**: Make sure you have run `source devel/setup.bash`.
- **Missing dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y`.
- **Build errors**: Check CMakeLists.txt and package.xml, or ask the TA for help.
- **No trajectory in RViz**: Make sure the rosbag is playing and the launch file parameters are correct.

---

## 6. References
- LIO-SAM official documentation: https://github.com/TixiaoShan/LIO-SAM
- ROS tutorials: http://wiki.ros.org/ROS/Tutorials

---

If you have questions, please contact the TA or ask in the course group.
