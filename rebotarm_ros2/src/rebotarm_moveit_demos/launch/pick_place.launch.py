from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_model():
    try:
        path = Path(
            get_package_share_directory("rebotarm_bringup")
        ) / "config" / "rebotarm_hardware.yaml"
        if path.exists():
            with open(path, "r", encoding="utf-8") as f:
                return str((yaml.safe_load(f) or {}).get("default_model") or "dm")
    except (PackageNotFoundError, OSError):
        pass
    return "dm"


def generate_launch_description():
    model = LaunchConfiguration("model")
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("rebotarm_moveit_demos"),
            "config",
            PythonExpression(
                [
                    "'pick_place_rs.yaml' if '",
                    model,
                    "'.lower() == 'rs' else 'pick_place.yaml'",
                ]
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=_default_model(),
                description="Robot model used by the active MoveIt demo: dm or rs",
            ),
            Node(
                package="rebotarm_moveit_demos",
                executable="pick_place",
                name="pick_place",
                output="screen",
                parameters=[config_file],
            )
        ]
    )
