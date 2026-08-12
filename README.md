# reBot Arm B601-RS

这是 B601-RS（RobStride/SocketCAN）专用仓库。ROS 2 核心基于最新统一版
`rebotarm_ros2`，网页控制台针对 RS 模型、命令接口和夹爪单位单独适配。

## 目录

```text
reBot_Arm_Mujoco-RS/
├── rebotarm_ros2/          # ROS 2 驱动、MuJoCo 抓取环境、RS Agent
├── reBotArm_simulator-RS/  # RS Three.js 网页控制台
└── scripts/                # 构建、仿真和真机启动脚本
```

## 最短运行路径

```bash
./scripts/setup_rs_workspace.sh
./scripts/start_rs_sim.sh
```

首次执行 setup 会拉取并固定
[`LAN-GER/reBot-B601-RS-for-mujoco_sim`](https://github.com/LAN-GER/reBot-B601-RS-for-mujoco_sim)
的 RS MJCF、网格和场景资源，同时创建独立 `.venv` 并安装 MuJoCo 与真机 SDK
所需依赖。`start_rs_sim.sh` 默认启动 MuJoCo 窗口、动力学抓取环境、俯视相机、
目标检测、IK/轨迹服务和 MCP 抓取 Agent：

```bash
./scripts/start_rs_sim.sh
```

无桌面的自动化测试可显式关闭窗口并切换到运动学同步：

```bash
REBOTARM_MUJOCO_VIEWER=false REBOTARM_MUJOCO_MODE=kinematic ./scripts/start_rs_sim.sh
```

另开终端：

```bash
cd reBotArm_simulator-RS
npm start
```

打开 `http://localhost:3002`，ROS 目标选择 `/rebotarm_rs`。

## 抓取 Agent

仿真启动后，场景内有红色方块、蓝色长方块和黄色圆柱。MCP 地址为：

```text
http://127.0.0.1:8081/mcp
```

`pick_color` 会执行 RS 专用的俯视抓取，并确认物体在 MuJoCo 中实际升高后才返回
成功。主要 ROS 话题：

- `/rebotarm_rs/mujoco/object_states`
- `/rebotarm_rs/mujoco/overhead_rgb/image_raw`
- `/rebotarm_rs/vision/color_blocks/detections`

如需网页里的自然语言助手，配置兼容 OpenAI 的模型后启动：

```bash
export DASHSCOPE_API_KEY="你的 Key"
export REBOTARM_LLM_MODEL="qwen-plus"
./scripts/start_rs_text_agent.sh
```

Text Agent/Dashboard 默认监听 `http://localhost:8082`。不配置大模型时，MCP 的
检测、抓取、IK 和关节工具仍可直接使用。

真机启动受到显式环境变量保护，参见
[`reBotArm_simulator-RS/README.md`](reBotArm_simulator-RS/README.md)。

## 当前边界

RS 网页、ROS 2 真机驱动、MoveIt 模型、Fake Driver、MuJoCo 物理抓取和 Agent
已经形成闭环。运动学模式适合验证接口和网页控制；动力学参数与抓取高度针对当前 RS
MJCF 验证通过，后续用于真机前仍需重新标定。RS 使用独立 MJCF，不复用 DM 的机械
结构和执行器参数。

## 第三方版本与本仓库覆盖

`rebotarm_ros2/third_party/` 是 setup 自动生成目录，不直接提交嵌套 Git 仓库：

- `patches/rebotarm_control_py_rs.patch` 保存 RS 轨迹时长安全修正；
- `vendor_overrides/reBot-B601-RS-for-mujoco_sim/` 保存已验证的 RS MJCF 外观、碰撞与标牌资源；
- `scripts/setup_rs_workspace.sh` 拉取固定上游提交后自动应用上述内容。

因此从全新 clone 执行 setup，可以复现当前本机使用的 SDK 与 MuJoCo 场景，而不依赖
未提交的 `third_party` 工作树。
