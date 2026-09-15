import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LITE_CHANNEL_CONFIG_FILE = "control_command_odom.yaml"
LITE_ADDITIONAL_ENV = {"FASTDDS_BUILTIN_TRANSPORTS": "UDPv4"}


def _truthy(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _runtime_enabled(value, odin_impl):
    normalized = str(value).strip().lower()
    if normalized == "auto":
        return odin_impl == "lite"
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(
        "enable_pelvis_runtime must be 'auto', true, or false, "
        f"got {value!r}"
    )


def _as_int(value, name):
    try:
        return int(value)
    except ValueError as exc:
        raise RuntimeError(f"{name} must be an integer, got {value!r}") from exc


def _build_relocalization_config(base_config, map_path, init_pose):
    with open(base_config, "r", encoding="utf-8") as f:
        root = yaml.safe_load(f) or {}

    reg = root.setdefault("register_keys", {})
    reg["custom_map_mode"] = 2
    reg["relocalization_map_abs_path"] = map_path
    if init_pose:
        parts = [p for p in init_pose.strip().strip("[]").replace(",", " ").split() if p]
        if len(parts) != 7:
            raise RuntimeError("odin_map_init must contain 7 values: x,y,z,qx,qy,qz,qw")
        reg["custom_init_pos"] = [float(p) for p in parts]

    tmp = tempfile.NamedTemporaryFile(
        mode="w", prefix="pico_odin_control_", suffix=".yaml", delete=False
    )
    with tmp:
        yaml.safe_dump(root, tmp, default_flow_style=False, sort_keys=False)
    return tmp.name


def _setup(context, *args, **kwargs):
    odin_impl = LaunchConfiguration("odin_impl").perform(context)
    if odin_impl not in ("original", "lite"):
        raise RuntimeError("odin_impl must be 'original' or 'lite'")

    image_width = _as_int(LaunchConfiguration("image_width").perform(context), "image_width")
    image_height = _as_int(LaunchConfiguration("image_height").perform(context), "image_height")
    image_fps = _as_int(LaunchConfiguration("image_fps").perform(context), "image_fps")

    if odin_impl == "lite":
        nodes = [Node(
            package="odin_ros_driver_rev1",
            executable="odin_ros_driver_node",
            output="screen",
            additional_env=LITE_ADDITIONAL_ENV,
            parameters=[{
                "operating_mode": "normal",
                "channel_config_file": LITE_CHANNEL_CONFIG_FILE,
                "image_width": image_width,
                "image_height": image_height,
                "image_fps": image_fps,
                "image_format": "mjpeg",
                "topic_prefix": "manifold",
            }],
            remappings=[
                ("/manifold/ODIN2/device0/odometry", "/raw/odom/odin"),
                ("/manifold/ODIN2/device0/odometry_hf", "/raw/odom/odin_highfreq"),
                ("/manifold/ODIN2/device0/imu", "/raw/imu/odin"),
                ("/manifold/ODIN2/device0/cloud/slam", "/raw/cloud/odin_slam"),
            ],
        )]
    else:
        odin_pkg_share = get_package_share_directory("odin_ros_driver")
        config_file = LaunchConfiguration("config_file").perform(context)
        if not config_file:
            config_file = os.path.join(odin_pkg_share, "config", "control_command.yaml")

        odin_map = LaunchConfiguration("odin_map").perform(context)
        odin_map_init = LaunchConfiguration("odin_map_init").perform(context)
        if odin_map:
            if not os.path.exists(odin_map):
                raise RuntimeError(f"odin_map does not exist: {odin_map}")
            config_file = _build_relocalization_config(
                config_file, os.path.abspath(odin_map), odin_map_init
            )

        nodes = [Node(
            package="odin_ros_driver",
            executable="host_sdk_sample",
            name="odin_driver_node",
            output="screen",
            parameters=[{"config_file": config_file}],
            remappings=[
                ("odin1/odometry_highfreq", "/raw/odom/odin_highfreq"),
                ("odin1/odometry", "/raw/odom/odin"),
                ("odin1/imu", "/raw/imu/odin"),
            ],
        )]

        if _truthy(LaunchConfiguration("enable_local_pose").perform(context)):
            nodes.append(Node(
                package="odin_ros_driver",
                executable="local_pose_node",
                name="odin_local_pose_node",
                output="screen",
                parameters=[{
                    "odometry_topic": "/raw/odom/odin",
                    "target_frame": "map",
                    "local_pose_topic": "/raw/pose/odin_local",
                    "local_odometry_topic": "/raw/odom/odin_local",
                }],
            ))

    runtime_setting = LaunchConfiguration("enable_pelvis_runtime").perform(context)
    if _runtime_enabled(runtime_setting, odin_impl):
        nodes.append(Node(
            package="pico_odin",
            executable="odin_pelvis_runtime",
            name="odin_pelvis_runtime",
            output="screen",
            parameters=[{
                "extrinsics_file": LaunchConfiguration(
                    "pelvis_extrinsics_file"
                ).perform(context),
                "raw_odin_topic": LaunchConfiguration(
                    "raw_odin_topic"
                ).perform(context),
                "raw_odin_highfreq_topic": LaunchConfiguration(
                    "raw_odin_highfreq_topic"
                ).perform(context),
                "pico_smpl_topic": LaunchConfiguration(
                    "pico_smpl_topic"
                ).perform(context),
                "pico_smpl_fused_topic": LaunchConfiguration(
                    "pico_smpl_fused_topic"
                ).perform(context),
                "world_reset_topic": LaunchConfiguration(
                    "world_reset_topic"
                ).perform(context),
                "output_frame": LaunchConfiguration(
                    "pelvis_output_frame"
                ).perform(context),
                "expected_skeleton_frame": LaunchConfiguration(
                    "expected_skeleton_frame"
                ).perform(context),
                "pelvis_output_topic": LaunchConfiguration(
                    "pelvis_output_topic"
                ).perform(context),
                "pelvis_highfreq_output_topic": LaunchConfiguration(
                    "pelvis_highfreq_output_topic"
                ).perform(context),
                "smpl_odin_output_topic": LaunchConfiguration(
                    "smpl_odin_output_topic"
                ).perform(context),
                "smpl_fused_odin_output_topic": LaunchConfiguration(
                    "smpl_fused_odin_output_topic"
                ).perform(context),
                "alignment_samples": _as_int(
                    LaunchConfiguration("alignment_samples").perform(context),
                    "alignment_samples",
                ),
                "max_sync_gap_sec": float(
                    LaunchConfiguration("max_sync_gap_sec").perform(context)
                ),
                "max_pending_age_sec": float(
                    LaunchConfiguration("max_pending_age_sec").perform(context)
                ),
                "host_sync_history_sec": float(
                    LaunchConfiguration("host_sync_history_sec").perform(context)
                ),
                "max_receive_lag_sec": float(
                    LaunchConfiguration("max_receive_lag_sec").perform(context)
                ),
                "receive_lag_step_sec": float(
                    LaunchConfiguration("receive_lag_step_sec").perform(context)
                ),
                "min_receive_lag_correlation": float(
                    LaunchConfiguration("min_receive_lag_correlation").perform(context)
                ),
                "min_receive_lag_pairs": _as_int(
                    LaunchConfiguration("min_receive_lag_pairs").perform(context),
                    "min_receive_lag_pairs",
                ),
                "receive_lag_smoothing_alpha": float(
                    LaunchConfiguration("receive_lag_smoothing_alpha").perform(context)
                ),
                "receive_lag_update_period_sec": float(
                    LaunchConfiguration("receive_lag_update_period_sec").perform(context)
                ),
                "odin_restart_gap_sec": float(
                    LaunchConfiguration("odin_restart_gap_sec").perform(context)
                ),
                "odin_translation_jump_m": float(
                    LaunchConfiguration("odin_translation_jump_m").perform(context)
                ),
                "odin_rotation_jump_rad": float(
                    LaunchConfiguration("odin_rotation_jump_rad").perform(context)
                ),
                "stationary_linear_speed_mps": float(
                    LaunchConfiguration("stationary_linear_speed_mps").perform(context)
                ),
                "stationary_angular_speed_radps": float(
                    LaunchConfiguration("stationary_angular_speed_radps").perform(context)
                ),
            }],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "odin_impl",
            default_value="original",
            description="Odin driver implementation: original or lite",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Odin1 control config YAML. Empty uses odin_ros_driver default.",
        ),
        DeclareLaunchArgument(
            "odin_map",
            default_value="",
            description="Odin1 relocalization map file. Empty disables map relocalization override.",
        ),
        DeclareLaunchArgument(
            "odin_map_init",
            default_value="",
            description="Optional Odin1 initial pose: x,y,z,qx,qy,qz,qw",
        ),
        DeclareLaunchArgument(
            "enable_local_pose",
            default_value="true",
            description="For Odin1, publish /raw/pose/odin_local and /raw/odom/odin_local in map frame.",
        ),
        DeclareLaunchArgument(
            "enable_pelvis_runtime",
            default_value="auto",
            description=(
                "Run Odin-to-pelvis correction: auto enables it for Odin Lite; "
                "true/false explicitly override the default."
            ),
        ),
        DeclareLaunchArgument(
            "pelvis_extrinsics_file",
            default_value="",
            description=(
                "Saved T_pelvis_odin YAML. Empty uses the portable XDG default."
            ),
        ),
        DeclareLaunchArgument("raw_odin_topic", default_value="/raw/odom/odin"),
        DeclareLaunchArgument(
            "raw_odin_highfreq_topic",
            default_value="/raw/odom/odin_highfreq",
        ),
        DeclareLaunchArgument("pico_smpl_topic", default_value="/pico/smpl"),
        DeclareLaunchArgument(
            "pico_smpl_fused_topic",
            default_value="/pico/smpl_fused",
        ),
        DeclareLaunchArgument("world_reset_topic", default_value="/pico/world_reset"),
        DeclareLaunchArgument("pelvis_output_frame", default_value="pico_ground"),
        DeclareLaunchArgument(
            "expected_skeleton_frame", default_value="pico_ground"
        ),
        DeclareLaunchArgument(
            "pelvis_output_topic",
            default_value="/calibrated/odom/pelvis",
        ),
        DeclareLaunchArgument(
            "pelvis_highfreq_output_topic",
            default_value="/calibrated/odom/pelvis_highfreq",
        ),
        DeclareLaunchArgument(
            "smpl_odin_output_topic",
            default_value="/pico/smpl_odin",
        ),
        DeclareLaunchArgument(
            "smpl_fused_odin_output_topic",
            default_value="/pico/smpl_fused_odin",
        ),
        DeclareLaunchArgument(
            "alignment_samples",
            default_value="30",
            description="Stationary synchronized samples used for per-run alignment.",
        ),
        DeclareLaunchArgument(
            "max_sync_gap_sec",
            default_value="0.05",
            description="Maximum host-time gap between Odin samples bracketing PICO receipt time.",
        ),
        DeclareLaunchArgument(
            "max_pending_age_sec",
            default_value="0.25",
            description="Maximum host-receipt age of a queued PICO skeleton.",
        ),
        DeclareLaunchArgument(
            "host_sync_history_sec",
            default_value="6.0",
            description="Host-receipt history retained for lag estimation.",
        ),
        DeclareLaunchArgument(
            "max_receive_lag_sec",
            default_value="0.30",
            description="Maximum absolute PICO/Odin receive lag considered.",
        ),
        DeclareLaunchArgument(
            "receive_lag_step_sec",
            default_value="0.002",
            description="Search resolution for per-run receive-lag estimation.",
        ),
        DeclareLaunchArgument(
            "min_receive_lag_correlation",
            default_value="0.25",
            description="Minimum motion correlation required to accept receive lag.",
        ),
        DeclareLaunchArgument(
            "min_receive_lag_pairs",
            default_value="10",
            description="Minimum active motion pairs required to estimate receive lag.",
        ),
        DeclareLaunchArgument(
            "receive_lag_smoothing_alpha",
            default_value="0.20",
            description="Smoothing factor for accepted receive-lag updates.",
        ),
        DeclareLaunchArgument(
            "receive_lag_update_period_sec",
            default_value="0.50",
            description="Minimum host time between receive-lag updates.",
        ),
        DeclareLaunchArgument(
            "odin_restart_gap_sec",
            default_value="1.0",
            description="Receipt gap that forces Odin session realignment.",
        ),
        DeclareLaunchArgument(
            "odin_translation_jump_m",
            default_value="0.35",
            description="Minimum Odin position jump treated as an origin reset.",
        ),
        DeclareLaunchArgument(
            "odin_rotation_jump_rad",
            default_value="0.60",
            description="Minimum Odin orientation jump treated as an origin reset.",
        ),
        DeclareLaunchArgument(
            "stationary_linear_speed_mps",
            default_value="0.04",
            description="Maximum PICO and Odin speed accepted during run alignment.",
        ),
        DeclareLaunchArgument(
            "stationary_angular_speed_radps",
            default_value="0.08",
            description="Maximum PICO and Odin angular speed during run alignment.",
        ),
        DeclareLaunchArgument("image_width", default_value="0"),
        DeclareLaunchArgument("image_height", default_value="0"),
        DeclareLaunchArgument("image_fps", default_value="0"),
        OpaqueFunction(function=_setup),
    ])
