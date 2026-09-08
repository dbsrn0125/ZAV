# ZAV (Zenith Aerial Vehicle)

Autonomous Drone Platform equipped with Livox Mid-360 LiDAR, Hikrobot MV-CU013 Global Shutter Camera, and FAST-LIVO2 LiDAR-Inertial-Visual Odometry & Mapping.

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

---

## Workspace Structure
```text
zenith_ws/
├── docker/              # Docker environment & container setup scripts
├── scans/               # Generated 3D point cloud maps & interactive WebGL viewers
├── scripts/             # Operational automation scripts
│   ├── run_camera_calibration.sh  # Automated camera intrinsic calibration
│   ├── auto_camera_calib.py       # OpenCV corner detection & yaml updater
│   ├── play_bag.sh                # Sensor bag replay with RViz2 & FAST-LIVO2
│   └── view_pcd.sh                # 3D PCD viewer in RViz2
└── src/                 # ROS 2 source packages
    ├── FAST-LIVO2/      # Visual-LiDAR-Inertial Odometry & Mapping
    ├── livox_ros_driver2/
    ├── mvs_ros_driver2/
    ├── rpg_vikit/
    └── sophus/
```

---

## Quick Start

### 1. Build Workspace (inside Docker)
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
