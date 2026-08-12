# reBot Arm B601-RS 网页控制台

该目录提供 B601-RS 的 Three.js 网页模型、ROS 2 控制面板和可选的
MotorBridge 高级控制界面。它与仓库中的 `rebotarm_ros2` 使用同一份 RS
URDF/STL；目录内的 `description` 只作为网页被单独复制时的离线后备。

## 当前能力

| 能力 | 状态 |
| --- | --- |
| RS Three.js/URDF/STL | 可用 |
| RS ROS 2 Fake Driver | 可用，命名空间 `/rebotarm_rs` |
| RS 真机 ROS 2 Controller | 可用，命名空间 `/rebotarm` |
| 新版 `JointPosVelCmd`/`JointMitCmd` | 已适配 |
| RS 夹爪宽度与电机角度换算 | 已适配，71.5 mm ↔ 5 rad |
| RS MuJoCo 动力学 | 可用，启动脚本默认打开 Viewer 与 physics 模式 |
| 三色物体抓取环境 | 可用，含俯视相机、检测、IK 与物理抬升验证 |
| MCP/Text 抓取 Agent | 可用，MCP 默认 `http://127.0.0.1:8081/mcp` |
| MotorBridge 直连 | 保留；不能替代物理急停和后端安全层 |

## 启动网页

```bash
cd reBotArm_simulator-RS
npm start
```

浏览器访问 `http://localhost:3002`。

## 启动安全仿真

先在仓库根目录构建 ROS 2 工作区：

```bash
./scripts/setup_rs_workspace.sh
```

然后启动 RS Fake Driver、MuJoCo 动力学 Viewer、抓取 Agent 和 rosbridge：

```bash
./scripts/start_rs_sim.sh
```

网页填写 `ws://<Ubuntu-IP>:9090`，选择“RS 仿真（/rebotarm_rs）”。收到
`fake_rs_pos_vel` 状态后，网页会自动允许仿真控制。

网页自然语言助手另开终端启动：

```bash
export DASHSCOPE_API_KEY="你的 Key"
./scripts/start_rs_text_agent.sh
```

## RS 真机

真机使用 SocketCAN `can0`，默认控制模式为 MIT。真机脚本设有显式保护：

```bash
export REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE
./scripts/start_rs_hardware.sh
```

同时启动真机、Fake Driver 和 rosbridge：

```bash
export REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE
./scripts/start_rs_dual.sh
```

执行前必须确认 `can0` 配置正确、机械臂工作区无人、物理急停可用。网页真机
模式不会自动打开控制锁。

## ROS 2 接口约定

- 关节位置速度：`/<namespace>/joints/<joint>/cmd/pos_vel`
- 关节 MIT：`/<namespace>/joints/<joint>/cmd/mit`
- 夹爪位置速度：`/<namespace>/gripper/cmd/pos_vel`
- 状态：`/<namespace>/joint_states`、`arm_status`、`gripper/state`
- 网页内部夹爪单位是米；ROS 2 夹爪电机位置单位是弧度。
