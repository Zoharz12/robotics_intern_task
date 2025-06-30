
# Robotics Intern Task – Situational Awareness PoC

## 1. Overview

This project implements a minimal world mapping system for a robotic dog using a 128-beam LiDAR and Intel RealSense RGB-D camera
The system fuses sensor data and generates a semantic occupancy grid using ROS2 and a pretrained YOLOv8 segmentation model.

## 2. Features

- 128‑beam LiDAR & Intel RealSense RGB-D fusion
- Real-time 2D occupancy grid into a 20 m × 20 m
- Ultralytics YOLOv8-based obstacle detection
- Compact core logic (< 60 lines)
- Optimized for Jetson Orin NX for ≥ 5 FPS


## 3. Usage

### 3.1. Prerequisites

- ROS2 (Humble or newer)
- Python 3.8+, OpenCV, Numpy, `ultralytics`, `cv_bridge`
- RealSense & LiDAR ROS2 drivers
- Linux environment

### 3.2. Build & Run

To run the code, you must prepare auxiliary files,
according to the following structure:

- ~/ros2_ws/
-  ├── src/
-  │   └── minimal_world_mapper/            # ROS2 package
-  │       ├── minimal_world_mapper/        # Python package
-  │       │   ├── fusion_mapper_node.py    # main ROS2 node
-  │       │   ├── yolov8n-seg.pt           # YOLOv8-Seg model
-  │       │   └── __init__.py
-  │       ├── resource/
-  │       │   └── minimal_world_mapper
-  │       ├── package.xml
-  │       ├── setup.cfg
-  │       ├── setup.py
-  │       └── README.md


Then, run the following code on Linux:

```bash

# Go to your workspace root
cd ~/ros2_ws

# Source ROS2 Humble environment
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --symlink-install

# Source the workspace setup script
source install/setup.bash

# Run the node
ros2 run minimal_world_mapper fusion_mapper_node

```


## 4. Missing for Real Deployment

-  **Organized management, divided into a development portfolio and a production portfolio:**

- **Development portfolio capabilities:**
- •	Run on simulations and recorded data (Gazebo / Isaac Sim and .bag / .avi data)
- •	Stabilization and performance testing system
- •	Real-time health control monitoring tool for each sensor
- •	In the event of a malfunction in one sensor - full control capability of the corresponding sensor (only camera or only LiDAR), until repaired, of course including lowering the dog's walking pace due to identification errors (the camera and LiDAR complement each other)
- •	Adding the use of an internal temperature sensor + an additional external sensor to the processor - expensive and sensitive!
- •	Software version management using Git
- •	Using Jetson’s internal temperature sensor to monitor proper operating temperature - an expensive and sensitive component
- •	Mechanical production files, etc.
- •	TF calibration tool for calibrating physical locations of the camera, LiDAR, robot
- •	Adapting to an easy version update, or alternatively replacing the CV mode By replacing a file or config
- •	Optional:
-   o	Real-time unsupervised training capability, labels can be given afterwards
-   o	Remote control capability, for controlling the dog if necessary, including:
-    >	Adding a GPS component
-    >	Supporting application
-    >    Option to insert a SIM into the Internet
-    >    Remote control encryption

- **Production portfolio features:**
- •	Adapting topics from the development portfolio to current use and production
- •	A neat BOM file of the components
- •	Safety sensors for protection against dangerous proximity and automatic shutdown
- •	High-quality battery
- •	Separate configuration (config.yaml file) (restricted access), for easy replacement or upgrade of hardware or important software data and for cleaner code. The file includes:
-   o	Hardware and API data of the camera, LiDAR and other hardware
-   o	Access to replace the CV model
-   o	Resolution setting and other parameters
-   o	Access to train a self-model to adapt to the needs of the robot using a customized dataset
-   And more


## 5. References

- KITTI Dataset: https://www.cvlibs.net/datasets/kitti/raw_data.php  
- Ultralytics YOLOv8 Segmentation: https://docs.ultralytics.com/tasks/segment/  
- COCO Dataset for YOLOv8 Segmentation: https://cocodataset.org/
- rclpy documentation for ROS2 API: https://docs.ros2.org/foxy/api/rclpy/api/node.html
