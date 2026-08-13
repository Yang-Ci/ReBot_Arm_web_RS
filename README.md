# reBot Arm B601-RS

B601-RS（RobStride + SocketCAN）完整控制与仿真工程，包含 ROS 2 真机驱动、Three.js 网页、
RS 专用 MuJoCo 动力学场景、视觉检测、轨迹/IK 和 MCP 抓取 Agent。

本工程不把 DM 模型简单换皮为 RS。RS 使用独立机械模型、MIT 控制、125 Hz 在线平滑、
真机反馈动画和安全状态机。具体差异见：

- [完整中文使用手册](USER_MANUAL_ZH.md)
- [DM/RS 控制模式、频率、速度与优化对比](DM_RS_COMPARISON_ZH.md)
- [RS 数据流与处理链](DATA_FLOW_RS_ZH.md)
- [完整开发者手册](DEVELOPER_GUIDE_ZH.md)

## 功能

- RS 真机：SocketCAN `can0`、MIT 125 Hz、关节/TCP/夹爪网页控制；
- 在线运动平滑：60 Hz 网页目标 + 125 Hz 限速度/加速度/jerk 参考；
- 真机反馈显示：自适应低通、死区、插值，实线反馈与目标影子分离；
- 安全控制：非零位失能前回零、回零验证、旧目标清理、命令状态仲裁；
- 重力补偿：状态查询、重复启动保护、非零位置启动和停止后位置保持；
- 轨迹与 TCP：`FollowJointTrajectory`、动作速度保护、自适应 DLS IK；
- MuJoCo：RS 专用模型、动力学、薄桌面、彩色物体、相机和检测；
- Agent：颜色检测、抓取、IK/关节工具和可选自然语言入口。

## 支持环境

- 推荐 Ubuntu 24.04 + ROS 2 Jazzy；
- 支持 Ubuntu 22.04 + ROS 2 Humble（真机需自行回归）；
- Python 3.12/3.10，Node.js 18+；
- 真机 CAN：`can0`，1 Mbps。

## 安装

```bash
cd /home/robot/reBot_Arm_Mujoco-RS
./setup.sh --check       # 只检查，不修改系统
./setup.sh --yes         # 一键安装依赖、资源并构建
./rebotarm doctor        # 安装完成后复查
```

控制 SDK 与 MuJoCo 源码已经作为本仓库的普通文件保存在
`rebotarm_ros2/third_party/`，不是 submodule，也不是嵌套 Git 仓库。一次普通 clone 即可
取得完整源码。它们的上游基线为：

- `vectorBH6/reBotArm_control_py`：`40ab6ce58fec3c58cb603efb3f30240d6f5849e4`
- `LAN-GER/reBot-B601-RS-for-mujoco_sim`：`1249cb6efdf393ba636056fc41df30dc6ba389aa`

安装脚本只检查这些本地源码是否存在，然后创建独立 Python venv、执行 rosdep 和
`colcon build --symlink-install`。它不会联网拉取或重置 SDK/MuJoCo 源码；已有 `.env`
也不会被覆盖。

## 快速启动仿真

终端 1：

```bash
./rebotarm start rs_sim
```

默认会打开 MuJoCo 窗口并启用 `physics` 动力学，同时启动 Fake Driver、场景相机、颜色
检测、MCP Agent 和 rosbridge。

终端 2：

```bash
./rebotarm start web
```

打开 `http://localhost:3002`，ROS 目标选择 `/rebotarm_rs`。

无窗口/运动学运行：

```bash
REBOTARM_MUJOCO_VIEWER=false ./rebotarm start rs_sim
REBOTARM_MUJOCO_MODE=kinematic ./rebotarm start rs_sim
```

仿真场景主要话题：

```text
/rebotarm_rs/mujoco/joint_states
/rebotarm_rs/mujoco/object_states
/rebotarm_rs/mujoco/overhead_rgb/image_raw
/rebotarm_rs/vision/color_blocks/detections
```

MCP 地址：`http://127.0.0.1:8081/mcp`。

## 快速启动真机

先配置 CAN：

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

确认工作区无人、急停可用后启动：

```bash
REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE \
  ./rebotarm start rs
```

另开终端执行 `./rebotarm start web`，打开网页并选择 `/rebotarm`。首次测试建议把关节速度
降到 0.2–0.4 rad/s，一次只动一个关节。

真机驱动使用 `Ctrl+C` 停止，不要用 `Ctrl+Z`。如果上次控制器被暂停或异常残留，再次
运行启动命令会先请求旧控制器安全退出，再清理僵尸 Fast DDS 资源并启动新实例。

## 三层频率

| 层 | 默认频率 | 职责 |
|---|---:|---|
| 网页目标 | 最高 60 Hz | 关节滑块和 TCP IK 连续更新目标 |
| 真机同步反馈查询 | 20 Hz | 刷新电机测量缓存，不阻塞实时发送环 |
| ROS 真机状态 | 60 Hz | 从缓存发布反馈、控制目标和参考 |
| RS 电机 MIT 控制 | 125 Hz | 生成限 jerk 参考、用缓存计算重力前馈并发送到电机 |

网页显示通常按屏幕 60 Hz 绘制，但真机状态经 rosbridge 默认最多 12.5 Hz 到达，浏览器在
测量值间插值。画面刷新率不是控制器频率。

## 关键默认值

```text
网页 vlim             1.2 rad/s（范围 0.05–1.5）
普通真机轨迹速度保护  0.60 rad/s（仅局部超速段延时）
TCP 示教回放保护      1.20 rad/s（匹配实时拖拽）
在线加速度限制        4 rad/s²
在线 jerk 限制        30 rad/s³
J1–J6 滑块阻尼        30 ms
J7 夹爪拖动           独立宽度通道，每绘制帧只发送最新目标
MuJoCo 同步           250 Hz
Fake Driver 状态      100 Hz
```

项目启动脚本默认把 ROS 2 发现范围设为 `LOCALHOST`，避免 Wi-Fi 漫游或 IP 地址变化后，
先后启动的 rosbridge 与控制器落入不同 DDS 网络。网页端口仍监听局域网地址，不影响其他
设备打开网页。只有确实需要另一台计算机直接加入 ROS 图时，才在两个终端都设置
`REBOTARM_ROS_DISCOVERY_RANGE=SUBNET` 后启动。

完整状态表和算法说明见 [DM_RS_COMPARISON_ZH.md](DM_RS_COMPARISON_ZH.md)。

## 抓取 Agent

仿真中默认有红色方块、蓝色长方块和黄色圆柱。`pick_color` 执行俯视抓取，并在物体实际
升高后才报告成功。不配置大模型时，MCP 工具仍可直接使用。

启用自然语言 Agent：

```bash
export DASHSCOPE_API_KEY='你的 Key'
export REBOTARM_LLM_MODEL='qwen-plus'
./scripts/start_rs_text_agent.sh
```

Dashboard 默认地址为 `http://localhost:8082`。

## 常用命令

```bash
./rebotarm doctor
./rebotarm start rs_sim [--force]
./rebotarm start web
REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE ./rebotarm start rs
./rebotarm status
./rebotarm stop
```

日常使用统一通过 `rebotarm` 入口。`scripts/setup_rs_workspace.sh` 是一键安装脚本内部使用的
ROS 工作区构建器；`scripts/start_rs_dual.sh` 仅用于同时启动真机 `/rebotarm` 和 Fake Driver
`/rebotarm_rs` 做接口对照，不包含完整 MuJoCo 场景。

## 开发验证

```bash
source scripts/rs_env.sh
python3 -m pytest rebotarm_ros2/src/rebotarmcontroller/test -q

cd rebotarm_ros2
colcon build --symlink-install
colcon test --packages-select rebotarmcontroller
colcon test-result --verbose
```

Shell 与网页语法检查：

```bash
bash -n setup.sh rebotarm scripts/*.sh
node --check reBotArm_simulator-RS/server.js
node --check reBotArm_simulator-RS/public/js/ros/rebot-ros-ui.js
node --check reBotArm_simulator-RS/public/js/rebot-sim.js
```

## 本地源码维护

`rebotarm_ros2/third_party/` 中的 SDK 和 MuJoCo 源码由本仓库直接跟踪。不要在其中运行
`git init`，也不要再 clone 形成嵌套仓库。更新上游时直接审查普通文件差异并提交到本
仓库，确保其他用户只需 clone 一次即可得到完整工程。来源基线见
[`rebotarm_ros2/third_party/README.md`](rebotarm_ros2/third_party/README.md)。

## 安全与提交约定

- 不提交 `.env`、密钥、ROS bag、`build/`、`install/`、`log/`；
- 真机参数更改必须先仿真、再低速单关节、最后完整回归；
- 普通问题优先查 [数据流文档](DATA_FLOW_RS_ZH.md)
