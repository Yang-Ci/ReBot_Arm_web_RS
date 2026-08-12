# rebotarm_mujoco_rs

ROS 2 wrapper for the B601-RS MuJoCo model maintained at
`LAN-GER/reBot-B601-RS-for-mujoco_sim`.

The upstream repository currently has no LICENSE file, so its source and assets
are not vendored into this package. `scripts/setup_rs_workspace.sh` checks out
the pinned upstream revision under `rebotarm_ros2/third_party/`, which is ignored
by this repository.

The wrapper does not open SocketCAN or send hardware commands. It subscribes to
ROS `JointState`, so the real arm continues to have a single owner: the
`reBotArmController` node.

The RS package now also includes a physics grasp environment with red, blue,
and yellow objects, an overhead ROS camera, object detections, Cartesian IK,
trajectory actions, and task recording services used by `rebotarm_agent`.

Modes:

- `kinematic`: directly synchronizes ROS joint state into MuJoCo.
- `physics`: tracks ROS targets with conservative PD plus MuJoCo bias forces.

```bash
ros2 launch rebotarm_mujoco_rs mujoco_rs.launch.py \
  arm_namespace:=rebotarm_rs simulation_mode:=physics use_viewer:=true
```

Important topics:

- `/rebotarm_rs/mujoco/object_states`
- `/rebotarm_rs/mujoco/overhead_rgb/image_raw`
- `/rebotarm_rs/vision/color_blocks/detections`

Task endpoints:

- `/rebotarm_rs/move_to_pose_ik`
- `/rebotarm_rs/move_to_pose`
- `/rebotarm_rs/follow_joint_trajectory`
