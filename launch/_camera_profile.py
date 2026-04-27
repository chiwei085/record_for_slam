from pathlib import Path

from launch.actions import LogInfo

import yaml


CAMERA_PROFILE_DEFAULTS = {
    "rgb_image_topic": "/camera/color/image_raw",
    "rgb_camera_info_topic": "/camera/color/camera_info",
    "depth_image_topic": "/camera/depth/image_raw",
    "depth_camera_info_topic": "/camera/depth/camera_info",
    "point_cloud_topic": "",
    "imu_raw_topic": "/camera/imu",
    "imu_topic": "/imu/filtered",
    "use_point_cloud": False,
    "use_rgbd_sync": False,
    "approximate_sync": True,
    "approx_sync_max_interval": 0.02,
    "replay_approx_sync_max_interval": 0.1,
    "depth_registered_to_color": False,
    "require_registered_depth": False,
    "require_imu": True,
    "require_tf_static": False,
    "required_tf_frames": [],
    "qos_profile": "sensor_data",
}


def load_camera_profile(config_path):
    config_path = Path(config_path)
    if not config_path.is_file():
        raise FileNotFoundError(f"Camera profile config not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle) or {}

    warnings = []
    rgbd_input = payload.get("rgbd_input")
    if not isinstance(rgbd_input, dict):
        warnings.append("missing top-level 'rgbd_input' map; falling back to defaults")
        rgbd_input = {}

    ros_params = rgbd_input.get("ros__parameters")
    if not isinstance(ros_params, dict):
        warnings.append("missing 'rgbd_input.ros__parameters' map; falling back to defaults")
        ros_params = {}

    unknown_keys = sorted(set(ros_params.keys()) - set(CAMERA_PROFILE_DEFAULTS.keys()))
    if unknown_keys:
        warnings.append("unknown camera profile keys ignored: " + ", ".join(unknown_keys))

    recognized_keys = {k: ros_params[k] for k in CAMERA_PROFILE_DEFAULTS if k in ros_params}
    if not recognized_keys:
        warnings.append("loaded 0 recognized camera profile keys; all values are defaults")

    profile = dict(CAMERA_PROFILE_DEFAULTS)
    profile.update(recognized_keys)
    return config_path, profile, warnings, len(recognized_keys)


def resolve_camera_profile(context, package_dir, camera_profile_sub, camera_config_sub):
    camera_profile = context.perform_substitution(camera_profile_sub)
    camera_config = context.perform_substitution(camera_config_sub)
    config_path = (
        Path(camera_config)
        if camera_config
        else Path(package_dir) / "config" / f"camera_{camera_profile}.yaml"
    )
    resolved_path, camera, warnings, loaded_key_count = load_camera_profile(config_path)
    return camera_profile, camera_config, resolved_path, camera, warnings, loaded_key_count


def resolve_use_point_cloud(context, enable_pointcloud_sub, camera):
    override = context.perform_substitution(enable_pointcloud_sub).lower()
    if override == "":
        return bool(camera["use_point_cloud"])
    return override == "true"


def build_camera_profile_logs(prefix, config_path, camera, warnings,
                              loaded_key_count, use_point_cloud=None,
                              include_replay_params=False):
    approx_sync = bool(camera["approximate_sync"])
    approx_sync_interval = (
        str(camera["approx_sync_max_interval"])
        if approx_sync
        else "<n/a (approximate_sync=false)>"
    )
    logs = [
        LogInfo(msg=f"[{prefix}] Camera config: {config_path}"),
        LogInfo(msg=f"[{prefix}] loaded_profile_keys: {loaded_key_count}"),
        LogInfo(msg=f"[{prefix}] rgb_image_topic: {camera['rgb_image_topic']}"),
        LogInfo(msg=f"[{prefix}] rgb_camera_info_topic: {camera['rgb_camera_info_topic']}"),
        LogInfo(msg=f"[{prefix}] depth_image_topic: {camera['depth_image_topic']}"),
        LogInfo(msg=f"[{prefix}] depth_camera_info_topic: {camera['depth_camera_info_topic']}"),
        LogInfo(msg=f"[{prefix}] point_cloud_topic: {camera['point_cloud_topic'] or '<disabled>'}"),
        LogInfo(msg=f"[{prefix}] imu_raw_topic: {camera['imu_raw_topic'] or '<disabled>'}"),
        LogInfo(msg=f"[{prefix}] imu_topic: {camera['imu_topic'] or '<disabled>'}"),
        LogInfo(msg=f"[{prefix}] use_rgbd_sync: {str(camera['use_rgbd_sync']).lower()}"),
        LogInfo(msg=f"[{prefix}] require_imu: {str(camera['require_imu']).lower()}"),
        LogInfo(msg=f"[{prefix}] require_tf_static: {str(camera['require_tf_static']).lower()}"),
        LogInfo(msg=f"[{prefix}] required_tf_frames: {camera['required_tf_frames'] or '<none>'}"),
        LogInfo(msg=f"[{prefix}] approximate_sync: {str(approx_sync).lower()}"),
        LogInfo(msg=f"[{prefix}] approx_sync_max_interval: {approx_sync_interval}"),
        LogInfo(msg=f"[{prefix}] depth_registered_to_color: {str(camera['depth_registered_to_color']).lower()}"),
        LogInfo(msg=f"[{prefix}] require_registered_depth: {str(camera['require_registered_depth']).lower()}"),
        LogInfo(msg=f"[{prefix}] qos_profile: {camera['qos_profile']}"),
    ]
    if include_replay_params:
        logs.insert(
            len(logs) - 3,
            LogInfo(
                msg=(
                    f"[{prefix}] replay_approx_sync_max_interval: "
                    f"{camera['replay_approx_sync_max_interval']}"
                )
            ),
        )
    if use_point_cloud is not None:
        logs.insert(
            7, LogInfo(msg=f"[{prefix}] use_point_cloud: {str(use_point_cloud).lower()}")
        )
    logs.extend(LogInfo(msg=f"[{prefix}][WARN] {warning}") for warning in warnings)
    return logs


def build_rgbd_remappings(camera):
    remappings = [
        ("rgb/image", camera["rgb_image_topic"]),
        ("rgb/camera_info", camera["rgb_camera_info_topic"]),
        ("depth/image", camera["depth_image_topic"]),
        ("depth/camera_info", camera["depth_camera_info_topic"]),
    ]
    if camera["require_imu"] and camera["imu_topic"]:
        remappings.append(("imu", camera["imu_topic"]))
    return remappings
