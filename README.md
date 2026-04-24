# record_for_slam

ROS 2 Humble tools for RGB-D bag recording, offline RTAB-Map replay, and semantic mapping.

> [!NOTE]
> Only tested with **RTABMap**. Other SLAM frameworks may require topic or QoS adjustments.

> [!NOTE]
> RGB-D topic assumptions are now configured through camera profile YAML files under `config/`.
> The package supports general RGB-D sensor inputs through camera profiles. `bag_for_slam.launch.py` can auto-load a built-in driver bringup, disable driver launch, or include a custom driver launch file.

## Workflow

1. Use `bag_for_slam.launch.py` to gate and record RGB-D topics into a ROS 2 bag.
2. Use `playback.launch.py` to inspect recorded data.
3. Use `offline_rtabmap.launch.py` to build an RTAB-Map database from the bag.
4. Use semantic mapping tools on the exported RTAB-Map results when needed.

## Installation

Base install:

```bash
cd ~/ros2_ws/src
git clone https://github.com/chiwei085/record_for_slam.git
cd ~/ros2_ws
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src --skip-keys=librealsense2 -y
colcon build --symlink-install
source install/setup.bash
```

Optional RealSense bringup for `bag_for_slam.launch.py`:

```bash
cd ~/ros2_ws/src
git clone --branch 4.57.6 --depth 1 \
  https://github.com/IntelRealSense/realsense-ros.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

For other RGB-D sensors, install the vendor driver separately and point this package at the correct topics through `camera_profile` or `camera_config`.

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

# Use a different camera topic profile for record_gate / bag topic selection
ros2 launch record_for_slam bag_for_slam.launch.py camera_profile:=yahboom_astra

# Bring your own driver
ros2 launch record_for_slam bag_for_slam.launch.py driver_bringup:=none

# Use a custom driver launch file
ros2 launch record_for_slam bag_for_slam.launch.py \
  driver_bringup:=/abs/path/to/driver.launch.py

# Use a custom camera profile YAML
ros2 launch record_for_slam bag_for_slam.launch.py \
  camera_config:=/abs/path/to/camera_profile.yaml
```

Default gate conditions: RGB image, depth image, RGB/depth `camera_info`, `/tf_static`, and `/imu/filtered` only when the active profile enables IMU.

> [!WARNING]
> `bag_for_slam.launch.py` is generic only at the recorder/gate layer. Driver bringup is controlled separately by `driver_bringup:=auto|none|/abs/path/to/driver.launch.py`.

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

## Offline RTAB-Map Replay

```bash
# Replay with the default camera profile
ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir

# Replay with the Yahboom Astra topic profile
ros2 launch record_for_slam offline_rtabmap.launch.py \
  bag_path:=/abs/path/to/bag_dir \
  camera_profile:=yahboom_astra

# Replay with a custom camera profile YAML
ros2 launch record_for_slam offline_rtabmap.launch.py \
  bag_path:=/abs/path/to/bag_dir \
  camera_config:=/abs/path/to/camera_profile.yaml
```

## Camera Profiles

Camera input is configured by YAML under `config/`.

`offline_rtabmap.launch.py` is the generic path: it remaps RTAB-Map from the selected profile.
`bag_for_slam.launch.py` uses the same profile for gating and recording, while driver launch is controlled separately by `driver_bringup`.

Built-in profiles:

| Profile         | RGB topic                 | Depth topic                                | Depth aligned to color | IMU required |
| --------------- | ------------------------- | ------------------------------------------ | ---------------------- | ------------ |
| `realsense`     | `/camera/color/image_raw` | `/camera/aligned_depth_to_color/image_raw` | `true`                 | `true`       |
| `yahboom_astra` | `/camera/color/image_raw` | `/camera/depth/image_raw`                  | `false`                | `false`      |

- Yahboom official documentation lists `/camera/depth/image_raw`, not `/camera/aligned_depth_to_color/image_raw`.
- `depth_registered_to_color=false` is the conservative default for `yahboom_astra`.
- Yahboom/Orbbec-style topic names in this repo are assumptions until verified on the target robot.

More detail and validation commands are documented in `docs/camera_profiles.md`.

Minimal custom profile example:

```yaml
rgbd_input:
  ros__parameters:
    rgb_image_topic: /camera/color/image_raw
    rgb_camera_info_topic: /camera/color/camera_info
    depth_image_topic: /camera/depth/image_raw
    depth_camera_info_topic: /camera/depth/camera_info
    depth_registered_to_color: false
    require_registered_depth: false
    require_imu: false
    imu_raw_topic: ""
    imu_topic: ""
    qos_profile: sensor_data
```

Typical RealSense profile topics:

| Topic                                        | Description                                       |
| -------------------------------------------- | ------------------------------------------------- |
| `/camera/color/image_raw`                    | RGB image                                         |
| `/camera/color/camera_info`                  | RGB camera intrinsics                             |
| `/camera/aligned_depth_to_color/image_raw`   | Depth aligned to color frame                      |
| `/camera/aligned_depth_to_color/camera_info` | Aligned depth intrinsics                          |
| `/camera/imu`                                | Raw IMU (gyro + accel merged by RealSense driver) |
| `/imu/filtered`                              | Filtered IMU from `imu_filter_madgwick`           |
| `/tf`                                        | Dynamic transforms                                |
| `/tf_static`                                 | Static transforms                                 |

RealSense driver note:

- `/camera/depth/color/points` is a RealSense driver topic, not a default topic from `camera_realsense.yaml`
- if you want the driver bringup to publish point clouds, set `enable_pointcloud:=true` or a profile with `use_point_cloud: true`

Typical Yahboom Astra profile topics:

| Topic                       | Description             |
| --------------------------- | ----------------------- |
| `/camera/color/image_raw`   | RGB image               |
| `/camera/color/camera_info` | RGB camera intrinsics   |
| `/camera/depth/image_raw`   | Depth image             |
| `/camera/depth/camera_info` | Depth camera intrinsics |
| `/camera/depth/points`      | Point cloud if enabled  |
| `/tf`                       | Dynamic transforms      |
| `/tf_static`                | Static transforms       |

For `yahboom_astra`, IMU topics are disabled by default:

- `imu_raw_topic: ""`
- `imu_topic: ""`
- `require_imu: false`

## ONNX Runtime And YOLO

If you only need bag recording, playback, or offline RTAB-Map replay, you can skip this section.

Semantic mapping builds without ONNX Runtime, but YOLO inference requires it. The helper script is the recommended path:

```bash
python3 src/record_for_slam/scripts/build.py
python3 src/record_for_slam/scripts/build.py --disable-yolo
python3 src/record_for_slam/scripts/build.py --onnxruntime-root /path/to/onnxruntime-root
```

Notes:

- the helper script always configures CMake with `-GNinja`
- CMake does not allow switching generators in the same build directory; if you start with the helper script, keep using it, or pass `-GNinja` yourself on raw `colcon build`
- `find_package(onnxruntime CONFIG QUIET)` is used in CMake
- if ONNX Runtime is not found, the package still builds, but YOLO inference is disabled
