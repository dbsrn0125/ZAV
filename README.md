# ZAV (Zenith Aerial Vehicle)

Autonomous Drone Platform equipped with Livox Mid-360 LiDAR, Hikrobot MV-CU013 Global Shutter Camera, FAST-LIVO2 LiDAR-Inertial-Visual Odometry & Mapping, and NVIDIA Isaac Sim / Pegasus Simulation Environment.

## Overview
ZAV is an advanced aerial robotics project designed for GPS-denied indoor/outdoor 3D navigation and dense SLAM under a 2kg payload limit.

### Hardware Stack
- **Onboard Computer**: Radxa Embedded ARM SBC
- **LiDAR**: Livox Mid-360 (360° x 59° FOV, 10Hz, Built-in IMU 200Hz)
- **Camera**: Hikrobot MV-CU013-A0UC (Global Shutter, Wide-Angle)
- **Mounting**: LiDAR tilted forward by ~22°, Camera forward-facing

### Software Architecture
- **ROS 2**: Humble Hawksbill
- **Container Environment**: Docker (`zenith_dev`)
- **Odometry & SLAM**: [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) (LiDAR-Inertial-Visual tightly-coupled EKF)
- **Sensors Drivers**:
  - `livox_ros_driver2` (CustomMsg & PointCloud2 streaming)
  - `mvs_ros2_driver` (Hikrobot industrial camera ROS 2 driver)
- **Simulation**:
  - NVIDIA Isaac Sim & [PegasusSimulator](https://github.com/PegasusSimulator/PegasusSimulator)
  - PX4 SITL (v1.14+) with Micro-XRCE-DDS Agent

---

## Workspace Structure
```text
zenith_ws/
├── docker/              # Docker environment & container setup scripts (Radxa Onboard)
├── scans/               # Generated 3D point cloud maps & interactive WebGL viewers
├── scripts/             # Operational automation scripts
│   ├── run_camera_calibration.sh  # Automated camera intrinsic calibration
│   ├── auto_camera_calib.py       # OpenCV corner detection & yaml updater
│   ├── play_bag.sh                # Sensor bag replay with RViz2 & FAST-LIVO2
│   └── view_pcd.sh                # 3D PCD viewer in RViz2
├── simulation/          # [Simulation] Pegasus / Isaac Sim & PX4 SITL
│   ├── setup_sim.sh     # One-click environment setup script
│   ├── scripts/         # All-in-one tmux dashboard launcher (zav_sim)
│   ├── px4/             # Airframe configuration (10020_isaac_zav)
│   └── pegasus/         # Vehicle dynamics (zav_v1.py) & 3D USD Robot asset
└── src/                 # ROS 2 source packages
    ├── FAST-LIVO2/      # Visual-LiDAR-Inertial Odometry & Mapping
    ├── livox_ros_driver2/
    ├── mvs_ros_driver2/
    ├── px4_msgs/        # PX4 ROS 2 interfaces
    ├── zav_control/     # FPV keyboard teleoperation & mission controller
    ├── rpg_vikit/
    └── sophus/
```

---

## Quick Start

### 1. Build Workspace (inside Docker or Native ROS 2)
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Camera Intrinsic Calibration
```bash
./scripts/run_camera_calibration.sh
```

### 3. Replay Sensor Bag & Real-time FAST-LIVO2 Mapping
```bash
./scripts/play_bag.sh <bag_name_or_timestamp> 2
```

### 4. View Completed 3D Point Cloud Map
```bash
./scripts/view_pcd.sh scans/corridor_20260908_101118.pcd
```
Or open `scans/corridor_scan_viewer.html` in any web browser for interactive 3D inspection.

---

### 5. ZAV Simulation (Isaac Sim + Pegasus + PX4 SITL)

#### (1) One-Click Simulation Setup
```bash
cd ~/zenith_ws/simulation
./setup_sim.sh
```

#### (2) Launch Simulation Dashboard
```bash
zav_sim
```

#### (3) FPV Keyboard Teleoperation
In a new terminal:
```bash
source ~/zenith_ws/install/setup.bash
ros2 run zav_control teleop_keyboard
```
- `T`: Auto-takeoff (2.0m hover)
- `L`: Auto-land & disarm
- `W/S/A/D`: Forward / Backward / Left / Right (FPV body frame)
- `Space / C`: Climb / Descend
- `Q / E`: Yaw turn
