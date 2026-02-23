# AAE4011 Assignment 1 — Question 3: ROS-Based Vehicle Detection from Rosbag

> **Student Name:** [Your Name]
> **Student ID:** [Your Student ID]
> **Date:** [Submission Date]

---

## Table of Contents

- [1. Overview](#1-overview)
- [2. Detection Method](#2-detection-method)
- [3. Repository Structure](#3-repository-structure)
- [4. Prerequisites](#4-prerequisites)
- [5. Installation](#5-installation)
- [6. How to Run](#6-how-to-run)
- [7. Sample Results](#7-sample-results)
- [8. Video Demonstration](#8-video-demonstration)
- [9. Reflection & Critical Analysis](#9-reflection--critical-analysis)
- [10. References](#10-references)

---

## 1. Overview

This project implements a ROS-based vehicle detection pipeline that processes images extracted from a rosbag file. The pipeline reads camera frames from the provided `.bag` file, applies a deep-learning-based object detector to identify vehicles (cars, trucks, buses, etc.), and visualises the results through an interactive UI with bounding boxes, class labels, and confidence scores.

---

## 2. Detection Method

<!-- Q3.1 README – Method description [2 marks] -->
<!-- Explains detection method (architecture, why selected) -->

### Architecture

This project uses **YOLOv5** (You Only Look Once, version 5) as the vehicle detection model. YOLOv5 is a single-stage object detector that divides the input image into a grid and predicts bounding boxes and class probabilities in a single forward pass.

**Key components of the pipeline:**

1. **Image Extraction Node** — A ROS node subscribes to the image topic from the rosbag (e.g., `/camera/image_raw`) using `cv_bridge` to convert ROS `sensor_msgs/Image` messages into OpenCV frames.
2. **Detection Node** — Each extracted frame is passed through the YOLOv5s (small) model, pre-trained on the COCO dataset (which includes vehicle classes such as car, bus, truck, motorcycle). The model outputs bounding box coordinates, class labels, and confidence scores.
3. **Visualisation Node** — Detected vehicles are drawn on the original frames with bounding boxes, labels, and scores. Results are published as annotated images and displayed in a GUI window.

### Why YOLOv5?

- **Real-time performance:** YOLOv5s achieves ~140 FPS on a modern GPU, making it suitable for onboard UAS processing.
- **High accuracy on vehicles:** The COCO-pretrained model already covers common vehicle categories, reducing the need for custom training.
- **Easy integration with ROS:** The PyTorch-based implementation works seamlessly with Python ROS nodes.
- **Lightweight variant available:** The YOLOv5s (small) variant balances speed and accuracy, appropriate for resource-constrained UAS platforms.

---

## 3. Repository Structure

```
├── CMakeLists.txt              # Catkin build configuration
├── package.xml                 # ROS package manifest
├── setup.py                    # Python package setup
├── README.md                   # This file
├── launch/
│   └── vehicle_detection.launch   # Launch file for the full pipeline
├── scripts/
│   ├── image_extractor.py         # Extracts images from rosbag
│   ├── vehicle_detector.py        # YOLOv5 detection node
│   └── detection_ui.py            # Visualisation / UI node
├── src/
│   └── vehicle_detection/
│       ├── __init__.py
│       ├── detector.py            # Detection model wrapper
│       └── utils.py               # Utility functions (drawing, stats)
├── config/
│   ├── detection_params.yaml      # Detection thresholds and settings
│   └── rviz_config.rviz           # RViz visualisation config
├── msg/
│   └── DetectionResult.msg        # Custom message for detection output
├── results/
│   └── sample_output.png          # Example detection result
└── bag/
    └── (place your .bag file here)
```

---

## 4. Prerequisites

- **OS:** Ubuntu 20.04
- **ROS:** ROS Noetic
- **Python:** 3.8+
- **GPU (optional):** NVIDIA GPU with CUDA 11.x for faster inference

### Python Dependencies

```
torch>=1.9.0
torchvision>=0.10.0
opencv-python>=4.5.0
numpy>=1.21.0
cv_bridge
rospy
Pillow>=8.0.0
```

---

## 5. Installation

<!-- Q3.1 README – How to run [2 marks] -->
<!-- Clear, reproducible step-by-step instructions -->

### Step 1: Clone the Repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/<your-username>/<your-repo-name>.git
```

### Step 2: Install Python Dependencies

```bash
cd ~/catkin_ws/src/<your-repo-name>
pip install torch torchvision opencv-python numpy Pillow
```

### Step 3: Download YOLOv5 Weights (if not bundled)

```bash
# The script will auto-download yolov5s.pt on first run,
# or you can manually download it:
wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt \
     -O ~/catkin_ws/src/<your-repo-name>/weights/yolov5s.pt
```

### Step 4: Build the ROS Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Step 5: Place the Rosbag File

```bash
# Copy the provided .bag file into the bag/ directory
cp /path/to/provided_rosbag.bag ~/catkin_ws/src/<your-repo-name>/bag/
```

---

## 6. How to Run

### Option A: Run the Full Pipeline with the Launch File

```bash
# Terminal 1 — Start ROS core
roscore

# Terminal 2 — Launch the detection pipeline
roslaunch <your-package-name> vehicle_detection.launch \
    bag_file:=$(rospack find <your-package-name>)/bag/provided_rosbag.bag
```

### Option B: Run Nodes Individually

```bash
# Terminal 1 — Start ROS core
roscore

# Terminal 2 — Play the rosbag
rosbag play ~/catkin_ws/src/<your-repo-name>/bag/provided_rosbag.bag

# Terminal 3 — Run the image extractor
rosrun <your-package-name> image_extractor.py

# Terminal 4 — Run the vehicle detector
rosrun <your-package-name> vehicle_detector.py

# Terminal 5 — Run the visualisation UI
rosrun <your-package-name> detection_ui.py
```

### Expected Output

- An OpenCV window displays each frame with bounding boxes, class labels, and confidence scores overlaid.
- Terminal output shows detection statistics: total frames processed, vehicles detected per frame, and average confidence.

---

## 7. Sample Results

### Image Extraction Summary

| Property | Value |
|----------|-------|
| Total frames extracted | 523 |
| Image resolution | 1280 × 720 |
| Colour format | BGR (8-bit) |
| Image topic | `/camera/image_raw` |

### Detection Results

| Metric | Value |
|--------|-------|
| Average vehicles per frame | 4.2 |
| Average confidence score | 0.82 |
| Detection classes found | car, bus, truck |
| Processing speed | ~28 FPS (with GPU) |

### Sample Detection Output

> *(Insert a screenshot of the detection output here)*
>
> ![Sample Detection](results/sample_output.png)

---

## 8. Video Demonstration

<!-- Q3.2 Video Demonstration [5 marks] -->

A screen-capture video (approximately 2 minutes) demonstrating the full pipeline is available here:

**Video Link:** [YouTube (Unlisted)](https://youtu.be/your-video-link-here)

The video shows:
- (a) Launching the ROS package and running the detection pipeline
- (b) The UI displaying detection results with bounding boxes on rosbag images
- (c) A brief verbal explanation of the detection results and observations

---

## 9. Reflection & Critical Analysis

<!-- Q3.3 Reflection & Critical Analysis [8 marks] -->
<!-- 300–500 words -->

### (a) What Did You Learn?

<!-- [2 marks] Identify at least two specific technical skills or concepts you gained -->

Through this assignment, I gained two key technical skills. First, I learned how to work with **ROS bag files** — specifically, how to use the `rosbag` Python API and `cv_bridge` to extract and convert image messages into OpenCV-compatible formats. Before this project, I had only worked with pre-saved image datasets, so learning to process time-stamped sensor data from a robotic middleware was a valuable new skill. Second, I developed a practical understanding of **single-stage object detection architectures**, particularly YOLOv5. I learned how anchor boxes, non-maximum suppression (NMS), and confidence thresholds affect detection quality, and I experimented with adjusting these parameters to balance precision and recall for vehicle detection.

### (b) How Did You Use AI Tools?

<!-- [2 marks] Describe how you used AI assistants and discuss both benefits and limitations -->

I used an AI coding assistant (e.g., GitHub Copilot / ChatGPT) in several stages of this project. It was particularly helpful for generating boilerplate ROS node code (publishers, subscribers, message callbacks) and for debugging `cv_bridge` compatibility issues between ROS Noetic and OpenCV 4. The AI also helped me understand YOLOv5's output tensor format and write the post-processing code for extracting bounding boxes.

However, I encountered limitations: the AI occasionally suggested deprecated ROS APIs or incorrect message type imports. I had to cross-reference the official ROS documentation to verify suggestions. Additionally, the AI-generated code sometimes lacked error handling (e.g., for empty frames or missing topics), which I had to add manually. This experience taught me that AI tools are powerful accelerators but require careful validation.

### (c) How to Improve Accuracy?

<!-- [2 marks] Propose two concrete strategies to improve detection accuracy and explain why each would help -->

Two strategies could improve the detection accuracy of this pipeline:

1. **Fine-tuning on a domain-specific dataset:** The current model uses COCO-pretrained weights, which cover general vehicle categories but may not perform optimally on aerial/UAS-perspective images. Fine-tuning YOLOv5 on a drone-view vehicle dataset (e.g., VisDrone or UAVDT) would help the model learn features specific to the top-down or oblique viewpoints common in UAS applications, improving both precision and recall.

2. **Multi-scale detection with test-time augmentation (TTA):** Vehicles in the rosbag images appear at varying scales depending on distance from the camera. Applying TTA — running inference at multiple image resolutions and merging the results — would improve detection of small or distant vehicles that the model might otherwise miss at a single scale.

### (d) Real-World Challenges

<!-- [2 marks] Discuss two challenges of deploying this pipeline on an actual drone in real time -->

Deploying this detection pipeline on an actual drone presents two significant challenges:

1. **Computational constraints:** Drones carry limited onboard computing power (typically embedded GPUs like NVIDIA Jetson). Running YOLOv5 at full resolution in real time may exceed the available compute budget, forcing trade-offs between detection accuracy (model size, input resolution) and frame rate. Optimisations such as TensorRT conversion or model pruning would be necessary to achieve real-time performance.

2. **Environmental variability:** Real-world drone operations involve changing lighting conditions (shadows, glare, night), weather (rain, fog), and camera motion (vibration, rapid attitude changes). These factors degrade image quality and can cause false detections or missed vehicles. Robust deployment would require data augmentation during training, temporal filtering across frames, and potentially sensor fusion with LiDAR or radar to maintain reliable detection.

---

## 10. References

1. Ultralytics YOLOv5: https://github.com/ultralytics/yolov5
2. ROS Noetic Documentation: http://wiki.ros.org/noetic
3. VisDrone Dataset: https://github.com/VisDrone/VisDrone-Dataset
4. cv_bridge Tutorial: http://wiki.ros.org/cv_bridge/Tutorials

---

> **Note:** This README serves as an example template for AAE4011 Assignment 1 Question 3. Students should replace all placeholder content (e.g., `[Your Name]`, `<your-repo-name>`, sample statistics) with their own actual implementation details and results.
