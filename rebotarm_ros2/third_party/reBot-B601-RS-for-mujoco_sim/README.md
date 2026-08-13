# reBot-B601-RS-for-mujoco_sim

基于 **MuJoCo** 的 reBot Arm B601-RS 机械臂仿真工程。

底层运动学 / 逆运动学 / 动力学 / 重力补偿通过
[reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py.git)
提供，本工程负责将其与 MuJoCo 仿真环境桥接，并提供 Real-to-Sim 接口。

## 功能规划

- [x] MuJoCo 仿真框架搭建
- [x] Conda 环境配置
- [x] 集成 `reBotArm_control_py` SDK
- [x] MuJoCo 中实现 IK（含交互式 Viewer 示例）
- [x] MuJoCo 中实现重力补偿（含交互式 Viewer 示例）
- [x] 浏览器交互式 IK 控制（拖条 + 摇杆速度控制）
- [x] Real-to-Sim 接口（支持真实 B601-RS 硬件与模拟模式）

## 环境要求

- Ubuntu 22.04+（推荐）
- Python 3.10
- [Miniconda](https://docs.conda.io/en/latest/miniconda.html) 或 Anaconda
- 支持 OpenGL 的显卡（运行 MuJoCo Viewer 时需要）

## 安装与使用

### 1. 进入父工程内的本地源码

```bash
cd rebotarm_ros2/third_party/reBot-B601-RS-for-mujoco_sim
```

本目录已经由 `ReBot_Arm_web_RS` 父仓库直接跟踪，不是独立或嵌套 Git 仓库。

### 2. 验证本地底层 SDK

```bash
bash scripts/setup_third_party.sh
```

该命令只检查同级目录 `../reBotArm_control_py`，不会下载或更新源码。

### 3. 创建并激活 Conda 环境

```bash
conda env create -f environment.yml
conda activate rebot-b601-rs-sim
```

环境名称为 `rebot-b601-rs-sim`，包含 MuJoCo 3.10、Pinocchio、NumPy、SciPy、PyYAML、pytest 等依赖。

> **注意**：如果你的系统安装了 ROS 2，ROS 的 `PYTHONPATH` 可能携带旧版 Pinocchio，导致版本冲突。
> `environment.yml` 已配置 `PYTHONPATH=""`，激活环境后会自动隔离。

### 4. 验证环境

```bash
python -c "import mujoco, pinocchio, numpy; print('MuJoCo:', mujoco.__version__, 'Pinocchio:', pinocchio.__version__)"
pytest tests/
```

### 5. 加载 MuJoCo 模型

本工程直接使用用户手动转换的 MuJoCo XML：

- 机器人模型：`assets/00_arm_rs_asm_v3/00_arm_rs_asm_v3.xml`
- 场景模型：`assets/00_arm_rs_asm_v3/scene.xml`（包含机器人、地面、灯光、世界坐标轴、台面、抓取方块）

本工程不再使用 `assets/robot/` 目录；所有 MuJoCo XML 均来自 `assets/00_arm_rs_asm_v3/`。

## 运行示例

### 基础示例（无界面批量运行）

```bash
# 加载模型并做简单物理仿真
python examples/01_load_model.py

# 将真实机器人状态同步到仿真（需先启动 CAN）
sudo ip link set can0 up type can bitrate 500000
python examples/04_real_to_sim.py

# 无硬件时使用模拟模式
python examples/04_real_to_sim.py --mock
```

### 带可视化窗口的基础示例

任意基础示例后加 `--viewer` 即可打开 MuJoCo Viewer：

```bash
python examples/01_load_model.py --viewer
python examples/04_real_to_sim.py --viewer
```

> **注意**：`04_real_to_sim.py` 默认会尝试连接真实机械臂；在无硬件环境中请使用 `--mock` 或 `--headless`。

### 交互式 MuJoCo Viewer 示例（需要图形界面）

```bash
# 终端输入目标位姿，机械臂在 MuJoCo 中实时运动
python examples/06_interactive_ik_mujoco.py

# 在 MuJoCo Viewer 中拖动关节，松手后关节悬浮在当前位置
python examples/07_interactive_gravity_compensation_mujoco.py

# 浏览器拖条 / 摇杆控制 IK + MuJoCo 可视化
python examples/08_interactive_ik_browser.py
```

`06` 的交互命令：

- `x y z` 或 `x y z roll pitch yaw`：目标位姿（米 / 弧度）
- `b` / `home` / `zero`：回归零点
- `q` / `quit` / `exit`：退出

`07` 的交互命令：

- `b` / `home` / `zero`：回归零点
- `o` / `open`：张开夹爪
- `c` / `close`：闭合夹爪
- `q` / `quit` / `exit`：退出

### 浏览器交互式 IK（Web 控制面板）

```bash
python examples/08_interactive_ik_browser.py
```

启动后打开终端输出的地址（默认 `http://localhost:8766`），通过网页拖条设定目标位姿，
或使用页面摇杆进行笛卡尔速度控制；MuJoCo viewer 会实时同步机械臂运动。

> **注意**：浏览器示例依赖 `tornado`，已包含在 `environment.yml` 中。若提示未安装，可执行
> `conda install -c conda-forge tornado` 或 `pip install tornado`。

### 真实机械臂 + MuJoCo 数字孪生（Real-to-Sim）

```bash
# 09：纯重力补偿 + 数字孪生同步（无夹爪力反馈，可自由开合夹爪）
# --no-hold 禁用电机保持力矩，方便手动拖动；真机运行时建议加上
python examples/09_real_to_sim_gravity_comp.py --no-hold

# 无硬件时使用模拟模式
python examples/09_real_to_sim_gravity_comp.py --mock --headless
```

`09` 的关键参数：

- `--kp-arm 0.0` / `--kd-arm 0.2`：默认纯重力补偿，kd 提供少量阻尼。
- `--gravity-scale 1.0,1.0,...`：若某关节下坠，可调大对应系数。
- `--tau-arm-limit 20.0`：臂前馈力矩上限。
- `--kp-gripper 0.0` / `--kd-gripper 0.05`：夹爪纯位置跟随，可自由开合。

> **注意**：`09` 每轮循环只读取一次真实机械臂状态，然后同步到 MuJoCo 并发送 MIT 命令。
> 如果在控制循环中多次触发硬件读取（例如反复调用 `bridge.read_real_state()`），
> 会导致 `_send_zero_mit()` 与重力补偿命令冲突，引起机械臂抖动。

### 运行测试

```bash
pytest tests/
```

## 常用命令速查

| 命令 | 说明 |
| --- | --- |
| `bash scripts/setup_third_party.sh` | 验证父仓库内置 SDK |
| `conda env create -f environment.yml` | 创建 Conda 环境 |
| `conda activate rebot-b601-rs-sim` | 激活环境 |
| `pytest tests/` | 运行单元测试 |
| `python examples/06_interactive_ik_mujoco.py` | 交互式 IK |
| `python examples/07_interactive_gravity_compensation_mujoco.py` | 交互式重力补偿 |
| `python examples/08_interactive_ik_browser.py` | 浏览器交互式 IK |
| `python examples/09_real_to_sim_gravity_comp.py --no-hold` | 真机重力补偿 + 数字孪生同步 |

## 项目结构

```
reBot-B601-RS-for-mujoco_sim/
├── README.md                          # 本文件
├── environment.yml                    # Conda 环境配置
├── pytest.ini                         # pytest 配置
├── scripts/
│   ├── setup_third_party.sh           # 验证同级本地 SDK
│   └── convert_urdf_to_mjcf.py        # 备用简化模型生成脚本
├── ../reBotArm_control_py/             # 父仓库直接跟踪的同级 SDK
├── assets/
│   └── 00_arm_rs_asm_v3/              # 用户手动转换的 MuJoCo 模型
│       ├── 00_arm_rs_asm_v3.xml       # 机器人模型
│       ├── scene.xml                  # 场景（含地面、灯光、坐标轴、台面、方块）
│       └── meshes/                    # STL 网格文件
├── src/rebot_b601_rs_sim/
│   ├── config.py                      # 路径与全局配置
│   ├── robot/                         # MuJoCo 模型/状态封装
│   ├── control/                       # IK、重力补偿、控制器
│   ├── bridge/                        # Real-to-Sim 桥接
│   ├── simulation/                    # 仿真主循环
│   ├── utils/                         # 工具函数
│   └── templates/                     # Web 控制面板 HTML / 工作空间常量 JSON
├── examples/                          # 示例脚本
└── tests/                             # 单元测试
```

## 依赖说明

- Python 3.10
- MuJoCo >= 3.0
- Pinocchio（SDK 依赖）
- NumPy / SciPy / PyYAML
- pytest（测试）
- MeshCat（可选可视化）
- tornado（浏览器交互示例 `08_interactive_ik_browser.py`）

## 注意事项

- `../reBotArm_control_py` 由父仓库直接跟踪，禁止创建嵌套 `.git`。
- `assets/00_arm_rs_asm_v3/` 下的 MuJoCo XML 由用户手动维护，是本仓库的主要模型文件。
- `04_real_to_sim.py` 已接入 `reBotArm_control_py` 的 `RebotArm`：连接真实机械臂前请确认 CAN 接口已启动；无硬件时会自动回退到模拟模式。
- `09_real_to_sim_gravity_comp.py` 中夹爪由真实电机位置直接驱动；`assets/00_arm_rs_asm_v3/00_arm_rs_asm_v3.xml` 中夹爪作动器已注释禁用，避免 MuJoCo 产生虚假伺服力。
- `scripts/convert_urdf_to_mjcf.py` 仅作为备用的简化 capsule 模型生成脚本，主流程不使用。

## 常见问题

### Q: 运行示例时提示 `ModuleNotFoundError: No module named 'rebot_b601_rs_sim'`

确保在工程根目录运行，且 `pytest.ini` 已配置 `pythonpath = src`。
若直接使用 `python -c` 运行，需手动设置 `PYTHONPATH=src`：

```bash
PYTHONPATH=src python -c "import mujoco; from rebot_b601_rs_sim.config import SCENE_PATH; m = mujoco.MjModel.from_xml_path(str(SCENE_PATH)); print('loaded', m.nq, m.nv)"
```

### Q: 导入 Pinocchio 时报错 `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`

通常是系统 ROS 的 Pinocchio 被优先加载。`rebot-b601-rs-sim` 环境已设置 `PYTHONPATH=""` 隔离 ROS，
请确认已执行 `conda activate rebot-b601-rs-sim`。

### Q: 运行 `09_real_to_sim_gravity_comp.py` 时机械臂抖动

常见原因：

1. **未加 `--no-hold`**：电机会保持初始位置，与手动拖动冲突。真机运行时请加 `--no-hold`。
2. **控制循环中多次触发硬件读取**：`09` 每轮循环只读取一次 `arm_group` 状态。
   若自定义代码中反复调用 `bridge.read_real_state()` 或 `bridge.get_arm_q_real()`，
   会导致零力矩 MIT 命令与重力补偿命令冲突，引起抖动。
3. **kd 不合适**：默认 `--kd-arm 0.2` 已提供足够阻尼。若仍抖动，可尝试 `--kd-arm 0.1` 或 `--kd-arm 0.3`。

### Q: MuJoCo Viewer 无法启动

MuJoCo Viewer 需要图形界面。在 SSH 或无显示器环境中，可运行无 viewer 的基础示例，
或使用 X11 转发 / VNC / 本地运行。
