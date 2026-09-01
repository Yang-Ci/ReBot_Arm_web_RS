import os
from importlib.machinery import SourceFileLoader

from ament_index_python.packages import get_package_share_directory


def apply_rviz_urdf_compat(moveit_config):
    """Apply the Jazzy RViz multi-material workaround to robot_description."""
    description_key = "robot_description"
    robot_description = moveit_config.robot_description.get(description_key)
    if not isinstance(robot_description, str):
        raise RuntimeError(
            "MoveIt robot_description must be expanded before RViz adaptation"
        )

    compat_path = os.path.join(
        get_package_share_directory("rebotarm_bringup"),
        "launch",
        "rviz_urdf_compat.py",
    )
    compat_module = SourceFileLoader(
        "rebotarm_rviz_urdf_compat", compat_path
    ).load_module()
    moveit_config.robot_description[description_key] = (
        compat_module.make_rviz_compatible(robot_description)
    )
    return moveit_config


def moveit_parameters(moveit_config):
    parameters = moveit_config.to_dict()
    ompl = parameters.setdefault("ompl", {})
    ompl["planning_plugin"] = "ompl_interface/OMPLPlanner"

    if os.environ.get("ROS_DISTRO") == "humble":
        ompl["request_adapters"] = " ".join(
            [
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/ResolveConstraintFrames",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
            ]
        )
        ompl.pop("response_adapters", None)

    return parameters
