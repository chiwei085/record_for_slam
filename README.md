# record_for_slam

One-command launch to start RealSense D455 and record a ROS 2 bag for SLAM.

> [!NOTE]
> Only tested with **RTABMap**. Other SLAM frameworks may require topic or QoS adjustments.

## Environment

|               | Version |
| ------------- | ------- |
| ROS 2         | Humble  |
| realsense-ros | 4.57.6  |
| LibRealSense  | 2.57.6  |

### WSL2

If you are running on WSL2 and encounter issues with RealSense (USB passthrough, device not found, etc.), refer to [WSL2-RealSense-Helper](https://github.com/chiwei085/WSL2-RealSense-Helper).

## Installation

Go to your ROS2 workspace (e.g. `~/ros2_ws`)

```bash
cd ~/ros2_ws/src
git clone https://github.com/chiwei085/record_for_slam.git
git clone https://github.com/IntelRealSense/realsense-ros.git  # tag: 4.57.6
cd ~/ros2_ws
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src --skip-keys=librealsense2 -y
colcon build --symlink-install
source install/setup.bash
```

## Recording

Bags are saved to `src/record_for_slam/bags/`:

```bash
# Basic recording
ros2 launch record_for_slam bag_for_slam.launch.py

# Custom bag name prefix
ros2 launch record_for_slam bag_for_slam.launch.py bag_prefix:=outdoor_run

# Also record point cloud
ros2 launch record_for_slam bag_for_slam.launch.py enable_pointcloud:=true

# Record without RViz2
ros2 launch record_for_slam bag_for_slam.launch.py enable_rviz:=false
```

## Playback

```bash
# Play back with RViz2 preview
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name>

# Half speed
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name> rate:=0.5

# Loop
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name> loop:=true

# Play without RViz2
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name> enable_rviz:=false
```

## Recorded Topics

| Topic                                        | Description                                        |
| -------------------------------------------- | -------------------------------------------------- |
| `/camera/color/image_raw`                    | RGB image                                          |
| `/camera/color/camera_info`                  | RGB camera intrinsics                              |
| `/camera/depth/image_rect_raw`               | Depth image                                        |
| `/camera/depth/camera_info`                  | Depth camera intrinsics                            |
| `/camera/aligned_depth_to_color/image_raw`   | Depth aligned to color frame                       |
| `/camera/aligned_depth_to_color/camera_info` | Aligned depth intrinsics                           |
| `/camera/imu`                                | IMU (gyro + accel merged, linear interpolation)    |
| `/tf`                                        | Dynamic transforms (15 Hz)                         |
| `/tf_static`                                 | Static transforms                                  |
| `/camera/depth/color/points`                 | Point cloud (opt-in via `enable_pointcloud:=true`) |
