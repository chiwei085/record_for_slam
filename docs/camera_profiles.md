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
- `depth_registered_to_color`: `true`
- `require_registered_depth`: `true`
- `require_imu`: `true`

Yahboom Astra profile: `config/camera_yahboom_astra.yaml`

- `rgb_image_topic`: `/camera/color/image_raw`
- `rgb_camera_info_topic`: `/camera/color/camera_info`
- `depth_image_topic`: `/camera/depth/image_raw`
- `depth_camera_info_topic`: `/camera/depth/camera_info`
- `point_cloud_topic`: `/camera/depth/points`
- `imu_raw_topic`: empty
- `imu_topic`: empty
- `depth_registered_to_color`: `false`
- `require_registered_depth`: `false`
- `require_imu`: `false`

Yahboom official documentation lists `/camera/depth/image_raw`, not `/camera/aligned_depth_to_color/image_raw`.
If your algorithm requires aligned depth, you must verify on the robot whether the depth image is already registered to the color frame.
The Yahboom Astra topic names in this profile are unverified assumptions until checked on the robot with `ros2 topic list` and `ros2 topic info`.

## Launch Usage

Offline RTAB-Map replay:

```bash
ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir
ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir camera_profile:=yahboom_astra
ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir camera_config:=/abs/path/to/custom_camera.yaml
```

Bag recording gate:

```bash
ros2 launch record_for_slam bag_for_slam.launch.py
ros2 launch record_for_slam bag_for_slam.launch.py camera_profile:=yahboom_astra
ros2 launch record_for_slam bag_for_slam.launch.py driver_bringup:=none
ros2 launch record_for_slam bag_for_slam.launch.py driver_bringup:=/abs/path/to/driver.launch.py
ros2 launch record_for_slam bag_for_slam.launch.py camera_config:=/abs/path/to/custom_camera.yaml
```

`bag_for_slam.launch.py` now separates camera profile from driver bringup:

- `driver_bringup:=auto` tries `launch/bringup_<camera_profile>.launch.py`
- `driver_bringup:=none` expects you to start the camera driver yourself
- `driver_bringup:=/abs/path/to/driver.launch.py` includes a custom launch file

If `camera_config:=...` is set, `driver_bringup:=auto` falls back to `none` and warns, because a custom profile does not safely imply a driver choice.

## Adding A Driver Bringup

If you want `driver_bringup:=auto` to support a new sensor, add a launch file named:

```text
launch/bringup_<camera_profile>.launch.py
```

The minimum expected launch args are:

- `fps`
- `enable_pointcloud`

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
