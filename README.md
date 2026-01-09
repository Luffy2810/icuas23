# ICUAS 2023 UAV Competition Solution

![ROS](https://img.shields.io/badge/ROS-Noetic-blue)
![Python](https://img.shields.io/badge/Python-3.8-brightgreen)
![CUDA](https://img.shields.io/badge/CUDA-11.6-green)
![License](https://img.shields.io/badge/License-GPL--3.0-red)

An autonomous UAV solution for the **ICUAS 2023 International Competition**, featuring real-time crack detection, tile classification, and intelligent exploration using a combination of **EGO-Planner** for trajectory planning, **YOLOv7** for object detection, and **ORB-SLAM2** for visual SLAM.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Key Features](#key-features)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Components](#components)
  - [EGO-Planner](#ego-planner)
  - [YOLOv7 Detection](#yolov7-detection)
  - [ORB-SLAM2 Integration](#orb-slam2-integration)
  - [Trajectory Controller](#trajectory-controller)
- [Configuration](#configuration)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Citation](#citation)
- [License](#license)

---

## Overview

This repository contains our solution for the ICUAS 2023 UAV Competition, where the objective is to autonomously navigate a UAV through an arena to locate and classify textured tiles (detecting cracks vs. non-cracks). The solution integrates:

- **Autonomous Exploration**: Intelligent navigation to Points of Interest (POIs)
- **Real-time Detection**: YOLOv7-based tile detection with a secondary classification network
- **Trajectory Planning**: EGO-Planner for ESDF-free gradient-based local planning
- **Visual SLAM**: ORB-SLAM2 for pose estimation and localization
- **Depth Processing**: RGB-D alignment and depth-to-3D coordinate transformation

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              ICUAS23 Solution Pipeline                          │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────┐    ┌──────────────────┐    ┌─────────────────┐                │
│  │  Gazebo     │───▶│  ArduPilot SITL  │───▶│   MAVROS        │                │
│  │  Simulator  │    │  (ArduCopter)    │    │   Interface     │                │
│  └─────────────┘    └──────────────────┘    └────────┬────────┘                │
│                                                       │                         │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                         ROS Communication Layer                          │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│         │                    │                    │                    │        │
│         ▼                    ▼                    ▼                    ▼        │
│  ┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────────┐   │
│  │ POI Handler │    │  EGO-Planner │    │   YOLOv7    │    │  ORB-SLAM2   │   │
│  │ & Explorer  │    │  Trajectory  │    │  Detection  │    │  Localization│   │
│  └──────┬──────┘    └──────┬───────┘    └──────┬──────┘    └──────────────┘   │
│         │                  │                    │                              │
│         └──────────────────┴────────────────────┴                              │
│                            │                                                    │
│                   ┌────────▼────────┐                                          │
│                   │  Trajectory     │                                          │
│                   │  Controller     │                                          │
│                   │  (traj_node.py) │                                          │
│                   └─────────────────┘                                          │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## Key Features

| Feature | Description |
|---------|-------------|
| 🎯 **Intelligent POI Exploration** | Distance-sorted POI traversal with adaptive height scanning |
| 🔍 **Dual-Stage Detection** | YOLOv7 tile detection + CNN-based crack classification |
| 🛫 **ESDF-Free Planning** | EGO-Planner for ~1ms trajectory computation |
| 📷 **RGB-D Processing** | Depth-to-RGB alignment with 3D coordinate estimation |
| 🎮 **PD Yaw Control** | Smooth yaw trajectory generation with PD controller |
| 🔄 **Auto Return-to-Home** | Automatic return to takeoff point before timeout |
| 🐳 **Docker Support** | Full containerized deployment with NVIDIA GPU support |

---

## Repository Structure

```
icuas23/
├── docker/                          # Docker configuration and launch scripts
│   ├── Dockerfile                   # Main container definition (CUDA 11.6 + ROS)
│   ├── build.sh                     # Build script for Docker image
│   ├── icuas.sh                     # Container orchestration and GPU passthrough
│   └── readme.md                    # Docker usage instructions
│
├── ego-planner/                     # ESDF-free gradient-based local planner
│   ├── src/
│   │   ├── ego_planner/             # Core planner package
│   │   ├── plan_env/                # Planning environment representation
│   │   ├── traj_utils/              # Trajectory utilities and TOPP-RA
│   │   └── uav_simulator/           # Lightweight quadrotor simulator
│   └── README.md                    # EGO-Planner documentation
│
├── icuas23_competition/             # Competition-specific ROS package
│   ├── launch/                      # ROS launch files
│   ├── models/                      # Gazebo world models
│   ├── startup/challenge/           # Challenge startup configuration
│   │   ├── session.yml              # TMux session definition
│   │   ├── traj_node.py             # Main trajectory controller node
│   │   ├── spawn_tiles.py           # Dynamic tile spawning
│   │   └── custom_config/           # PID and TOPP configuration
│   ├── worlds/                      # Gazebo world definitions
│   └── README.md                    # Competition package documentation
│
├── orb_ws/                          # ORB-SLAM2 ROS workspace
│   └── src/orb_slam_2_ros/          # ORB-SLAM2 ROS wrapper
│       ├── orb_slam2/               # Core ORB-SLAM2 library
│       ├── ros/                     # ROS interface nodes
│       └── docker/                  # ORB-SLAM specific Docker files
│
├── orb_utils/                       # SLAM utilities and post-processing
│   ├── d2r.py                       # Depth-to-RGB alignment node
│   ├── broadcast.py                 # TF broadcaster utilities
│   ├── plotter.py                   # Trajectory visualization
│   ├── rmse.py                      # RMSE calculation for evaluation
│   └── icuas/                       # ICUAS-specific SLAM scripts
│       ├── icuas_FBM1.sh            # TMux automation for rosbag processing
│       └── readme.txt               # SLAM pipeline documentation
│
├── yolov7/                          # YOLOv7 object detection
│   ├── detect_class.py              # Detection + classification wrapper
│   ├── models/                      # Network architecture definitions
│   ├── utils/                       # Detection utilities
│   ├── cfg/                         # Model configurations
│   └── readme.md                    # Weight file download instructions
│
└── README.md                        # This file
```

---

## Prerequisites

### Hardware Requirements
- **GPU**: NVIDIA GPU with CUDA 11.6+ support (Compute Capability 6.1+)
- **RAM**: Minimum 16GB recommended
- **Storage**: ~30GB for Docker image and dependencies

### Software Requirements
- **OS**: Ubuntu 20.04 LTS (Focal Fossa)
- **Docker**: 20.10+ with NVIDIA Container Toolkit
- **ROS**: Noetic Ninjemys (installed inside Docker)
- **CUDA**: 11.6+ with cuDNN 8

---

## Installation

### Option 1: Docker (Recommended)

1. **Install Docker with NVIDIA support**:
   ```bash
   curl https://raw.githubusercontent.com/larics/uav_ros_simulation/main/installation/dependencies/docker.sh | bash
   ```

2. **Clone the repository**:
   ```bash
   git clone <repository-url> icuas23
   cd icuas23
   ```

3. **Build the Docker image**:
   ```bash
   cd docker
   chmod +x build.sh
   ./build.sh
   ```

4. **Start the Docker container**:
   ```bash
   chmod +x icuas.sh
   ./icuas.sh
   ```

### Option 2: Native Installation

1. **Install UAV ROS Simulation stack**:
   ```bash
   git clone https://github.com/larics/uav_ros_simulation.git
   ./uav_ros_simulation/installation/install_and_setup_workspace.sh uav_ws
   ```

2. **Install additional dependencies**:
   ```bash
   sudo apt-get install libarmadillo-dev ffmpeg libsm6 libxext6
   pip install torch torchvision gdown
   ```

3. **Build all packages**:
   ```bash
   cd ~/uav_ws
   catkin build
   ```

---

## Quick Start

### Inside Docker Container

```bash
# 1. Start the Docker container (from host)
cd docker
./icuas.sh

# 2. Inside the container - build exploration files
cd /root/
./run.sh

# 3. Navigate to challenge directory
roscd icuas23_competition/startup/challenge

# 4. Start the simulation
./start.sh
```

### What happens when you run `start.sh`:

1. **ROS Core** initializes
2. **ArduPilot SITL** starts with simulated ArduCopter
3. **Gazebo** launches with the competition world
4. **Tiles spawn** at randomized positions around POIs
5. **UAV arms and takes off** automatically
6. **EGO-Planner** provides trajectory planning
7. **YOLOv7** begins real-time tile detection
8. **traj_node.py** orchestrates exploration and classification

---

## Components

### EGO-Planner

A lightweight gradient-based local planner that eliminates ESDF construction for faster computation (~1ms planning time).

**Key parameters** (in `ego_planner/launch/run_in_sim.launch`):
```yaml
max_vel: 2.0          # Maximum velocity (m/s)
max_acc: 3.0          # Maximum acceleration (m/s²)
planning_horizon: 7.5 # Planning horizon distance (m)
```

**Launch independently**:
```bash
# Terminal 1: RViz visualization
roslaunch ego_planner rviz.launch

# Terminal 2: Planner
roslaunch ego_planner run_in_sim.launch
```

### YOLOv7 Detection

Dual-stage detection pipeline:
1. **Stage 1**: YOLOv7 detects tiles in the camera feed
2. **Stage 2**: Classification CNN determines crack vs. non-crack

**Required weight files** (downloaded during Docker build):
```bash
# Detection model
gdown 1rF0Hq7JhosXx8EM1yJu1DXQ6nPctAKtq  # best.pt

# Classification model  
gdown 1k7jzh2hdD4TSVaW5j6WcsRq1OX35xj4P  # classify.pt
```

**Detection class usage**:
```python
from detect_class import mlcv

detector = mlcv()
detections, coords, annotated_img = detector.detect(image)
```

### ORB-SLAM2 Integration

Visual SLAM for localization using RGB-D camera input.

**Run ORB-SLAM2 with rosbag**:
```bash
cd orb_utils/icuas
./icuas_FBM1.sh <bagfile>.bag
```

**Pipeline steps**:
1. Launches `roscore` and `orb_slam3_ros`
2. Plays rosbag for sensor data
3. Saves initial pose from Vicon
4. Exports trajectory in camera frame
5. Transforms to world frame output

### Trajectory Controller

The main control node (`traj_node.py`) handles:

| Function | Description |
|----------|-------------|
| `callback_poi()` | Receives and sorts POIs by distance |
| `detect2()` | Executes vertical scan pattern at POI |
| `Yaw_Traj_Gen()` | Generates smooth yaw trajectories with PD control |
| `delete_poi()` | Removes POIs after successful detection |
| `Return_to_takeoffpt()` | Auto-return before competition timeout |

**ROS Topics**:
```
Published:
  /red/position_hold/trajectory    # Direct trajectory commands
  /red/tracker/input_trajectory    # TOPP-RA trajectory input
  /red/crack_image_annotated       # Annotated detection output

Subscribed:
  /red/poi                         # Points of Interest
  /red/odometry                    # UAV odometry
  /red/camera/color/image_raw      # RGB camera feed
  /red/camera/depth/image_raw      # Depth camera feed
  /planning/pos_cmd                # EGO-Planner position commands
```

---

## Configuration

### Position Control
`startup/challenge/custom_config/position_control_thrust.yaml`:
```yaml
Kp_x: 10.0
Kp_y: 10.0
Kp_z: 8.0
Kd_x: 5.0
Kd_y: 5.0
Kd_z: 4.0
```

### TOPP Trajectory Generation
`startup/challenge/custom_config/topp_config_custom.yaml`:
```yaml
max_velocity: 2.0
max_acceleration: 3.0
sampling_dt: 0.01
```

### Camera Parameters
In `traj_node.py`:
```python
self.focalx = 381.36246688113556
self.focaly = 381.36246688113556
self.cx = 320.5
self.cy = 240.5
```

---

## Usage

### Running the Full Competition

```bash
# Start everything with the session file
roscd icuas23_competition/startup/challenge
./start.sh
```

### Testing Individual Components

**Test EGO-Planner only**:
```bash
roslaunch ego_planner simple_run.launch
```

**Test YOLOv7 detection**:
```python
import cv2
from detect_class import mlcv

detector = mlcv()
img = cv2.imread('test_image.jpg')
detections, coords, result = detector.detect(img)
cv2.imshow('Detection', result)
cv2.waitKey(0)
```

**Test depth alignment**:
```bash
rosrun icuas23_competition d2r.py
```

### Modifying Tile Locations

Edit `spawn_tiles.py` or use the session file to change spawn positions:
```bash
rosrun icuas23_competition spawn_tiles.py __ns:="red"
```

---

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| **Docker GPU not detected** | Ensure NVIDIA Container Toolkit is installed: `nvidia-smi` should work inside container |
| **Gazebo crashes on startup** | Increase shared memory: `docker run --shm-size=4gb ...` |
| **YOLOv7 model not loading** | Verify weight files exist in `/root/yolov7/` |
| **EGO-Planner not responding** | Check if `/planning/pos_cmd` topic is publishing |
| **Camera images black** | Wait for Gazebo to fully load textures |

### Updating Packages

```bash
# Update competition package
cd ~/uav_ws/src/icuas23_competition
git pull origin main --rebase
catkin build

# Rebuild Docker image (if needed)
./docker_build.sh --build-args "--no-cache --pull" --focal
```

### Debug Commands

```bash
# Check active topics
rostopic list | grep red

# Monitor trajectory commands
rostopic echo /red/position_hold/trajectory

# View detection output
rosrun rqt_image_view rqt_image_view /red/crack_image_annotated

# Check POI data
rostopic echo /red/poi
```

---

## Citation

If you use this work in your research, please cite:

```bibtex
@article{Markovic2023,
  doi = {10.1007/s10846-023-01909-z},
  url = {https://doi.org/10.1007/s10846-023-01909-z},
  year = {2023},
  month = jul,
  publisher = {Springer Science and Business Media LLC},
  volume = {108},
  number = {3},
  author = {Lovro Markovic and Frano Petric and Antun Ivanovic and 
            Jurica Goricanec and Marko Car and Matko Orsag and Stjepan Bogdan},
  title = {Towards A Standardized Aerial Platform: ICUAS'22 Firefighting Competition},
  journal = {Journal of Intelligent \& Robotic Systems}
}
```

### Related Publications

**EGO-Planner**:
```bibtex
@article{Zhou2020,
  title={EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors},
  author={Zhou, Xin and Wang, Zhepei and Xu, Chao and Gao, Fei},
  journal={IEEE Robotics and Automation Letters},
  year={2020}
}
```

**YOLOv7**:
```bibtex
@article{Wang2022,
  title={YOLOv7: Trainable bag-of-freebies sets new state-of-the-art for real-time object detectors},
  author={Wang, Chien-Yao and Bochkovskiy, Alexey and Liao, Hong-Yuan Mark},
  journal={arXiv preprint arXiv:2207.02696},
  year={2022}
}
```

---

## License

This project is licensed under the **GPL-3.0 License** - see the [LICENSE](ego-planner/LICENSE) file for details.

### Component Licenses:
- **EGO-Planner**: GPL-3.0
- **YOLOv7**: GPL-3.0  
- **ORB-SLAM2**: GPL-3.0
- **uav_ros_simulation**: BSD-3-Clause

---

## Acknowledgments

- [LARICS Lab](https://github.com/larics) - UAV ROS Simulation framework
- [ZJU-FAST-Lab](https://github.com/ZJU-FAST-Lab) - EGO-Planner
- [WongKinYiu](https://github.com/WongKinYiu/yolov7) - YOLOv7
- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) - Visual SLAM

---

## Contact

For technical questions about this implementation, please open an issue in this repository.

For competition-related inquiries, refer to the [ICUAS 2023 Competition Discussions](https://github.com/larics/icuas23_competition/discussions).
