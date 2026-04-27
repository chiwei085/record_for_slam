# record_for_slam

ROS 2 Humble tools for RGB-D bag recording, offline RTAB-Map replay, and semantic mapping.

> [!NOTE]
> Only tested with **RTABMap**. Other SLAM frameworks may require topic or QoS adjustments.

> [!NOTE]
> RGB-D topic assumptions are now configured through camera profile YAML files under `config/`.
> The package supports general RGB-D sensor inputs through camera profiles. `bag_for_slam.launch.py` can auto-load a built-in driver bringup, disable driver launch, or include a custom driver launch file.

> [!WARNING]
> `offline_rtabmap.launch.py` still targets RTAB-Map's registered RGB-D workflow. A bag recorded from an Astra/Orbbec profile with raw `/camera/depth/image_raw` should not be used by itself to conclude that "the recording quality is bad" until aligned-depth and sync assumptions have been ruled out.

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

# Delegate robot control bringup to this launch session
ros2 launch record_for_slam bag_for_slam.launch.py \
  robot_bringup:=/abs/path/to/bringup_yahboom_r2_control.launch.py

# Also record robot/control topics with a YAML list
ros2 launch record_for_slam bag_for_slam.launch.py \
  robot_record_topics:="['/cmd_vel', '/joy', '/odom_raw']"

# Use a custom driver launch file
ros2 launch record_for_slam bag_for_slam.launch.py \
  driver_bringup:=/abs/path/to/driver.launch.py

# Use a custom camera profile YAML
ros2 launch record_for_slam bag_for_slam.launch.py \
  camera_config:=/abs/path/to/camera_profile.yaml
```

Default gate conditions follow the active profile: RGB image, depth image, RGB/depth `camera_info`, plus `/imu/filtered` and `/tf_static` only when that profile requires them.

> [!WARNING]
> `bag_for_slam.launch.py` is generic only at the recorder/gate layer. Driver bringup is controlled separately by `driver_bringup:=auto|none|/abs/path/to/driver.launch.py`.

## Playback

```bash
# Play back with RViz2 preview
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name>

# Play back a bag outside src/record_for_slam/bags/
ros2 launch record_for_slam playback.launch.py bag:=/abs/path/to/bag_dir

# Half speed
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name> rate:=0.5

# Loop
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name> loop:=true

# Play without RViz2
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name> enable_rviz:=false

# Use the Astra/Yahboom camera profile so RViz follows /camera/depth/image_raw
ros2 launch record_for_slam playback.launch.py \
  bag:=/abs/path/to/bag_dir \
  camera_profile:=yahboom_astra

# Use a custom camera profile YAML
ros2 launch record_for_slam playback.launch.py \
  bag:=/abs/path/to/bag_dir \
  camera_config:=/abs/path/to/camera_profile.yaml
```

## Offline RTAB-Map Replay

```bash
# Replay with the default camera profile
ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir

# Replay with the Yahboom Astra topic profile
ros2 launch record_for_slam offline_rtabmap.launch.py \
  bag_path:=/abs/path/to/bag_dir \
  camera_profile:=yahboom_astra

# Override replay sync tolerance if the bag's color/depth timestamps are wider apart
ros2 launch record_for_slam offline_rtabmap.launch.py \
  bag_path:=/abs/path/to/bag_dir \
  camera_config:=/abs/path/to/camera_profile.yaml

# Force a debug replay even when the profile says depth is not aligned to color
ros2 launch record_for_slam offline_rtabmap.launch.py \
  bag_path:=/abs/path/to/bag_dir \
  camera_profile:=yahboom_astra \
  allow_unregistered_depth:=true

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

- `yahboom_astra` is the raw-depth profile that matches the known Yahboom/Orbbec ROS 2 topic layout from the robot-side codebase.
- `yahboom_astra` now uses verified ROSMASTER R2 topics, not guessed topic names.
- RTAB-Map's standard RGB-D launch path expects registered depth. The upstream `rtabmap.launch` examples use registered topics like `depth_registered` / `aligned_depth_to_color`.
- Orbbec SDK ROS 2 documentation says depth-to-color alignment must be explicitly enabled with `depth_registration:=true`; raw `/camera/depth/image_raw` should not be treated as aligned by default.
- For camera profiles other than the validated ROSMASTER R2 Astra path, verify actual topic names on the target robot with `ros2 topic list`, `ros2 topic info`, and `ros2 topic echo`.
- New camera profiles inherit `require_registered_depth: false`, `require_tf_static: false`, and `replay_approx_sync_max_interval: 0.1` by default. Deployment-specific robot topics are configured separately at launch time.

More detail and validation commands are documented in `docs/camera_profiles.md`.

Minimal custom profile example for raw-depth record/playback:

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
    approx_sync_max_interval: 0.02
    replay_approx_sync_max_interval: 0.1
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

For `offline_rtabmap.launch.py`, `yahboom_astra` now follows the validated
Yahboom RTAB-Map topology more closely:

- `use_rgbd_sync: true` starts `rtabmap_sync/rgbd_sync`
- RTAB-Map subscribes to the synchronized `rgbd_image` output instead of directly syncing raw RGB/depth topics itself

For `bag_for_slam.launch.py`, robot/base behavior is separated from the camera profile:

- `driver_bringup:=auto` uses `launch/bringup_<camera_profile>.launch.py` when available
- `robot_bringup:=none` is the default; pass an explicit launch file path if you want this package to start robot control too
- `robot_record_topics:=...` accepts a YAML list of extra robot/control topics, so deployment-specific topics are decoupled from camera profiles
- `launch/bringup_yahboom_r2_control.launch.py` stays in this repo only as a convenience reference path

For Astra/Orbbec on ROS 2 Humble, validate the driver before changing this profile:

- `ros2_astra_camera` README: `https://github.com/orbbec/ros2_astra_camera`
- Orbbec SDK ROS 2 alignment docs: `https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/configuration/align_depth_color.html`
- RTAB-Map launch docs: `https://docs.ros.org/en/rolling/p/rtabmap_launch/__README.html`

Practical interpretation:

- If the robot publishes `/camera/depth/image_raw`, assume it is raw depth unless you have verified otherwise.
- If the driver can be started with depth alignment enabled, record the aligned depth topic and update the profile to match the actual published topic names.
- Keep `approx_sync_max_interval` tight. This repo now defaults it to `0.02` seconds instead of `0.1`.
- Keep `replay_approx_sync_max_interval` looser than live recording when needed. This repo defaults replay to `0.1` seconds to tolerate bag playback skew.

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
