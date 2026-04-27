# record_for_slam 

ROS 2 Humble tools for recording RGB-D bags, replaying them in RViz2, and building offline RTAB-Map databases for later semantic mapping.

## Prerequisites

- ROS 2 Humble
- `colcon` and `rosdep`
- A supported RGB-D camera setup such as Intel RealSense or Yahboom Astra
- RTAB-Map packages available in the ROS environment

> [!NOTE]
> This package is tested around RTAB-Map workflows. Other SLAM frameworks may require different topic, sync, or QoS assumptions.

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

## Quick Workflow

<table>
  <tbody>
    <tr>
      <td width="18%" valign="top">
        <strong>1. Record</strong>
      </td>
      <td valign="top">
        <code>bag_for_slam.launch.py</code><br/>
        Gate the required sensor topics, then start bag recording.<br/>
        <strong>Output</strong>: a timestamped bag directory under <code>src/record_for_slam/bags/</code>.
      </td>
    </tr>
    <tr>
      <td width="18%" valign="top">
        <strong>2. Inspect</strong>
      </td>
      <td valign="top">
        <code>playback.launch.py</code><br/>
        Replay the bag with RViz2 so the recorded streams can be checked quickly.<br/>
        <strong>Output</strong>: RViz2 preview and <code>ros2 bag play</code> replay of the recorded bag.
      </td>
    </tr>
    <tr>
      <td width="18%" valign="top">
        <strong>3. Process</strong>
      </td>
      <td valign="top">
        <code>offline_rtabmap.launch.py</code><br/>
        Run offline RTAB-Map over the recorded bag.<br/>
        <strong>Output</strong>: an <code>rtabmap.db</code> database alongside the bag directory.
      </td>
    </tr>
    <tr>
      <td width="18%" valign="top">
        <strong>4. Map</strong>
      </td>
      <td valign="top">
        semantic mapping tools<br/>
        Run semantic-map export or visualization on the RTAB-Map database when that stage is needed.<br/>
        <strong>Output</strong>: semantic mapping artifacts derived from the RTAB-Map database.
      </td>
    </tr>
  </tbody>
</table>

If recording does not start or replay/mapping behaves unexpectedly, check [Troubleshooting](#troubleshooting).

## Recording

Bags are saved to `src/record_for_slam/bags/`.

### Default

```bash
# Record with the default bag prefix and active camera profile
ros2 launch record_for_slam bag_for_slam.launch.py

# Change the output bag directory prefix
ros2 launch record_for_slam bag_for_slam.launch.py bag_prefix:=outdoor_run

# Disable RViz2 during recording
ros2 launch record_for_slam bag_for_slam.launch.py enable_rviz:=false

# Also record point cloud when the active driver/profile provides it
ros2 launch record_for_slam bag_for_slam.launch.py enable_pointcloud:=true
```

The default flow waits for the active profile's required sensor topics before starting the recorder.

### Driver Control

```bash
# Use the built-in driver helper for the selected camera profile
ros2 launch record_for_slam bag_for_slam.launch.py driver_bringup:=auto

# Start the camera driver yourself
ros2 launch record_for_slam bag_for_slam.launch.py driver_bringup:=none

# Use an explicit driver launch file
ros2 launch record_for_slam bag_for_slam.launch.py \
  driver_bringup:=/abs/path/to/driver.launch.py

# Delegate robot control bringup to this launch session
ros2 launch record_for_slam bag_for_slam.launch.py \
  robot_bringup:=/abs/path/to/bringup_yahboom_r2_control.launch.py
```

> [!NOTE]
> `driver_bringup` is camera-related. `robot_bringup` is deployment-related and is intentionally configured separately.

### Topic Overrides

```bash
# Use the Yahboom Astra camera profile
ros2 launch record_for_slam bag_for_slam.launch.py camera_profile:=yahboom_astra

# Use a custom camera profile YAML
ros2 launch record_for_slam bag_for_slam.launch.py \
  camera_config:=/abs/path/to/camera_profile.yaml

# Record extra deployment-specific robot/control topics
ros2 launch record_for_slam bag_for_slam.launch.py \
  robot_record_topics:="['/cmd_vel', '/joy', '/odom_raw']"
```

## Playback

### Default

```bash
# Replay a bag recorded under src/record_for_slam/bags/
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name>

# Replay a bag from any absolute path
ros2 launch record_for_slam playback.launch.py bag:=/abs/path/to/bag_dir
```

### Playback Controls

```bash
# Slow playback down
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name> rate:=0.5

# Loop playback
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name> loop:=true

# Disable RViz2 during playback
ros2 launch record_for_slam playback.launch.py bag:=<bag_directory_name> enable_rviz:=false
```

### Topic Overrides

```bash
# Replay with the Yahboom Astra camera profile
ros2 launch record_for_slam playback.launch.py \
  bag:=/abs/path/to/bag_dir \
  camera_profile:=yahboom_astra

# Replay with a custom camera profile YAML
ros2 launch record_for_slam playback.launch.py \
  bag:=/abs/path/to/bag_dir \
  camera_config:=/abs/path/to/camera_profile.yaml
```

Use playback to verify that the recorded RGB, depth, TF, and optional robot topics are present before running offline mapping.

## Offline RTAB-Map Replay

```bash
# Replay with the default camera profile
ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir

# Replay with the Yahboom Astra profile
ros2 launch record_for_slam offline_rtabmap.launch.py \
  bag_path:=/abs/path/to/bag_dir \
  camera_profile:=yahboom_astra

# Debug only: force replay even when the profile says depth is not registered to color
ros2 launch record_for_slam offline_rtabmap.launch.py \
  bag_path:=/abs/path/to/bag_dir \
  camera_profile:=yahboom_astra \
  allow_unregistered_depth:=true

# Replay with a custom camera profile YAML
ros2 launch record_for_slam offline_rtabmap.launch.py \
  bag_path:=/abs/path/to/bag_dir \
  camera_config:=/abs/path/to/camera_profile.yaml
```

The Yahboom Astra path uses `rtabmap_sync/rgbd_sync` during offline replay so that replay topology matches the validated on-robot RTAB-Map pipeline more closely.

> [!WARNING]
> `offline_rtabmap.launch.py` still targets RTAB-Map's RGB-D assumptions. For raw-depth profiles such as Yahboom Astra, use the replay result as a debugging signal, not as proof that recording quality is bad by itself.

## Camera Profiles

Built-in profiles:

<table>
  <thead>
    <tr>
      <th width="22%" align="left">Profile</th>
      <th width="78%" align="left">Configuration</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td valign="top">
        <strong><code>realsense</code></strong><br/>
        <sub>aligned-depth profile</sub>
      </td>
      <td valign="top">
        <strong>RGB</strong><br/>
        <code>/camera/color/image_raw</code><br/><br/>
        <strong>Depth</strong><br/>
        <code>/camera/aligned_depth_to_color/image_raw</code><br/><br/>
        <strong>Behavior</strong><br/>
        aligned depth: <code>true</code><br/>
        imu required: <code>true</code><br/>
        replay path: <code>direct</code>
      </td>
    </tr>
    <tr>
      <td valign="top">
        <strong><code>yahboom_astra</code></strong><br/>
        <sub>raw-depth ROSMASTER R2 profile</sub>
      </td>
      <td valign="top">
        <strong>RGB</strong><br/>
        <code>/camera/color/image_raw</code><br/><br/>
        <strong>Depth</strong><br/>
        <code>/camera/depth/image_raw</code><br/><br/>
        <strong>Behavior</strong><br/>
        aligned depth: <code>false</code><br/>
        imu required: <code>false</code><br/>
        replay path: <code>rgbd_sync</code>
      </td>
    </tr>
  </tbody>
</table>

`RGB-D sync path` means how offline replay feeds RTAB-Map:

- `direct`: RTAB-Map subscribes to RGB and depth topics separately
- `rgbd_sync`: `rtabmap_sync/rgbd_sync` packs RGB and depth into `rgbd_image` first

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
    approx_sync_max_interval: 0.02
    replay_approx_sync_max_interval: 0.1
```

More detail, field descriptions, driver notes, and validation references are in [docs/camera_profiles.md](docs/camera_profiles.md).

## Semantic Mapping

If you only need bag recording, playback, or offline RTAB-Map replay, you can skip this section.

Semantic mapping builds without ONNX Runtime, but YOLO inference requires it. The helper script is the recommended path:

```bash
python3 src/record_for_slam/scripts/build.py
python3 src/record_for_slam/scripts/build.py --disable-yolo
python3 src/record_for_slam/scripts/build.py --onnxruntime-root /path/to/onnxruntime-root
```

Notes:

- The helper script always configures CMake with `-GNinja`.
- CMake does not allow switching generators in the same build directory; if you start with the helper script, keep using it, or pass `-GNinja` yourself on raw `colcon build`.
- If ONNX Runtime is not found, the package still builds, but YOLO inference is disabled.

## Troubleshooting

- Recording gate never clears:
  Check `ros2 topic list`, `ros2 topic info`, and the launch logs to confirm that the active profile's RGB, depth, `camera_info`, IMU, and TF requirements are actually being published.
- Depth alignment concerns:
  RealSense uses an aligned depth profile by default. Yahboom Astra uses raw depth by default. Do not compare replay outcomes across those two paths without accounting for that difference.
- Replay drops or sync issues:
  Adjust `replay_approx_sync_max_interval` in the camera profile used for `offline_rtabmap.launch.py`.
- When to use `allow_unregistered_depth:=true`:
  Use it only for debugging when the active profile says depth is not registered to color but you still want to test whether replay can proceed. Do not treat the result as a clean recording-quality verdict.
- Driver mismatch on Astra:
  The built-in Yahboom helper expects `astra_camera` and defaults to `astra_pro.launch.xml`. If the installed driver layout differs, override `driver_package` or `driver_launch_file`.
- Wrong robot/control topics:
  Keep deployment-specific topics in `robot_record_topics:=...` instead of editing the camera profile unless the camera topics themselves changed.
