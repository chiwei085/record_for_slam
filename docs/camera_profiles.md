# Camera Profiles

`record_for_slam` now keeps RGB-D input assumptions in YAML camera profiles instead of hardcoding RealSense topic names in code.

## Profiles

RealSense rosbag profile: `config/camera_realsense.yaml`

- `rgb_image_topic`: `/camera/color/image_raw`
- `rgb_camera_info_topic`: `/camera/color/camera_info`
- `depth_image_topic`: `/camera/aligned_depth_to_color/image_raw`
- `depth_camera_info_topic`: `/camera/aligned_depth_to_color/camera_info`
- `point_cloud_topic`: empty by default
- `imu_raw_topic`: `/camera/imu`
- `imu_topic`: `/imu/filtered`
- `approx_sync_max_interval`: `0.02`
- `replay_approx_sync_max_interval`: `0.1`
- `depth_registered_to_color`: `true`
- `require_registered_depth`: `true`
- `require_imu`: `true`
- `require_tf_static`: `true`

Yahboom Astra profile: `config/camera_yahboom_astra.yaml`

- `rgb_image_topic`: `/camera/color/image_raw`
- `rgb_camera_info_topic`: `/camera/color/camera_info`
- `depth_image_topic`: `/camera/depth/image_raw`
- `depth_camera_info_topic`: `/camera/depth/camera_info`
- `point_cloud_topic`: `/camera/depth/points`
- `imu_raw_topic`: empty
- `imu_topic`: empty
- `use_rgbd_sync`: `true`
- `approx_sync_max_interval`: `0.02`
- `replay_approx_sync_max_interval`: `0.1`
- `depth_registered_to_color`: `false`
- `require_registered_depth`: `false`
- `require_imu`: `false`
- `require_tf_static`: `false`

Yahboom/Orbbec documentation, the on-robot `record_for_slam` reference, and Yahboom navigation/vision nodes consistently use `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`, and `/camera/depth/points`.
Yahboom/Orbbec examples show raw depth topics like `/camera/depth/image_raw`, not a guaranteed RealSense-style aligned-depth topic.
Orbbec SDK ROS 2 documentation says depth-to-color alignment must be explicitly enabled with `depth_registration:=true`.
This profile is meant to describe the robot's raw-depth record/playback behavior, not to silently rewrite it into a RealSense-style aligned-depth assumption.
These topic names should now be treated as validated for Yahboom ROSMASTER R2.

Default behavior for new custom profiles is less strict:

- `depth_registered_to_color` defaults to `false`
- `require_registered_depth` defaults to `false`
- `require_tf_static` defaults to `false`
- `replay_approx_sync_max_interval` defaults to `0.1`

This avoids surprising startup failures for profiles that only override topic names.
If a profile is meant for RTAB-Map's registered RGB-D path, set `require_registered_depth: true` explicitly.
If bag recording must wait for a latched camera static transform, set `require_tf_static: true` explicitly.
If bag replay needs a looser color-depth sync window than live recording, set `replay_approx_sync_max_interval` explicitly.
If a camera/robot combination is known to rely on `rtabmap_sync/rgbd_sync`, set `use_rgbd_sync: true`.

## Launch Usage

Offline RTAB-Map replay:

```bash
ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir
ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir camera_profile:=yahboom_astra
ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir camera_config:=/abs/path/to/custom_camera.yaml
ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir camera_profile:=yahboom_astra allow_unregistered_depth:=true
```

Bag recording gate:

```bash
ros2 launch record_for_slam bag_for_slam.launch.py
ros2 launch record_for_slam bag_for_slam.launch.py camera_profile:=yahboom_astra
ros2 launch record_for_slam bag_for_slam.launch.py driver_bringup:=none
ros2 launch record_for_slam bag_for_slam.launch.py robot_bringup:=/abs/path/to/control.launch.py
ros2 launch record_for_slam bag_for_slam.launch.py robot_record_topics:="['/cmd_vel', '/joy', '/odom_raw']"
ros2 launch record_for_slam bag_for_slam.launch.py driver_bringup:=/abs/path/to/driver.launch.py
ros2 launch record_for_slam bag_for_slam.launch.py camera_config:=/abs/path/to/custom_camera.yaml
```

`bag_for_slam.launch.py` now separates camera profile from driver bringup:

- `driver_bringup:=auto` tries `launch/bringup_<camera_profile>.launch.py`
- `driver_bringup:=none` expects you to start the camera driver yourself
- `driver_bringup:=/abs/path/to/driver.launch.py` includes a custom launch file
- `robot_bringup:=none` is the default and expects you to start robot/base/joystick control yourself
- `robot_bringup:=/abs/path/to/control.launch.py` includes a robot/control launch file explicitly
- `robot_record_topics:=...` adds a YAML list of deployment-specific robot topics to the bag

If `camera_config:=...` is set, `driver_bringup:=auto` falls back to `none` and warns, because a custom profile does not safely imply a driver choice.
`robot_bringup:=auto` is deprecated and intentionally does nothing, because robot control is a deployment concern rather than a camera-profile concern.

## Validation References

- `ros2_astra_camera` README: `https://github.com/orbbec/ros2_astra_camera`
- Orbbec depth-to-color alignment docs: `https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/configuration/align_depth_color.html`
- RTAB-Map launch README: `https://docs.ros.org/en/rolling/p/rtabmap_launch/__README.html`

## Adding A Driver Bringup

If you want `driver_bringup:=auto` to support a new sensor, add a launch file named:

```text
launch/bringup_<camera_profile>.launch.py
```

The minimum expected launch args are:

- `fps`
- `enable_pointcloud`

For `bringup_yahboom_astra.launch.py`, the built-in default expects package share
`astra_camera` and launch file `astra_pro.launch.xml`, matching the validated
on-robot `record_for_slam` reference. Override
`driver_package:=...` or `driver_launch_file:=...` if your Astra driver layout differs.

The simplest path is to copy `launch/bringup_realsense.launch.py` and replace only the vendor-specific driver section.

## On-Robot Validation

```bash
ros2 topic list | grep camera
ros2 topic info -v /camera/color/image_raw
ros2 topic info -v /camera/depth/image_raw
ros2 topic echo --once /camera/color/image_raw --field header.frame_id
ros2 topic echo --once /camera/depth/image_raw --field header.frame_id
ros2 topic echo --once /camera/color/camera_info
ros2 topic echo --once /camera/depth/camera_info
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link camera_link
```
