# AGENTS.md

## 项目背景

本工程为 reBot Arm B601-RS 机械臂的 **MuJoCo 仿真** 项目。
底层运动学、逆运动学、动力学、重力补偿由第三方仓库
`reBotArm_control_py` 提供；本仓库负责 MuJoCo 模型、仿真循环、
控制接口以及 Real-to-Sim 桥接。

## 重要约定

### 第三方 SDK 管理

- 本目录已 vendor 到父工程，不再是独立 Git 仓库。
- SDK 路径：同级目录 `../reBotArm_control_py`。
- SDK 与本 MuJoCo 源码都由父工程主仓库直接跟踪；禁止在其中创建嵌套 `.git`。
- `scripts/setup_third_party.sh` 只验证父工程已提供 SDK，不再执行 clone/update。
- `src/rebot_b601_rs_sim/config.py` 会自动将 SDK 目录加入 `sys.path`，
  工程内可直接 `import reBotArm_control_py`。

### 模型与坐标系

- B601-RS URDF：`../reBotArm_control_py/urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf`
- MuJoCo 机器人模型：`assets/00_arm_rs_asm_v3/00_arm_rs_asm_v3.xml`（用户手动从 URDF 转换）
- MuJoCo 场景：`assets/00_arm_rs_asm_v3/scene.xml`（包含机器人、地面、灯光、坐标轴、台面、方块）
- 受控关节名称：`joint1` ~ `joint6`（见 `src/rebot_b601_rs_sim/config.py`）
- 末端执行器帧：`gripper_end`
- 关节单位：弧度（rad）

### 代码组织

- `src/rebot_b601_rs_sim/robot/`：MuJoCo 模型封装与状态读取
- `src/rebot_b601_rs_sim/control/`：IK、重力补偿、控制器
- `src/rebot_b601_rs_sim/bridge/`：Real-to-Sim 桥接
- `src/rebot_b601_rs_sim/simulation/`：仿真主循环
- `src/rebot_b601_rs_sim/utils/`：通用工具函数（如数组裁剪、重力缩放解析、被动循环封装）
- `examples/`：可独立运行的示例脚本
- `tests/`：pytest 单元测试

### MuJoCo 模型

- 主流程直接使用 `assets/00_arm_rs_asm_v3/00_arm_rs_asm_v3.xml` 与
  `assets/00_arm_rs_asm_v3/scene.xml`。
- 这两个 XML 由用户手动从 SDK 的 B601-RS URDF 转换/维护，**不再**在运行时从
  URDF 自动生成。
- `scripts/convert_urdf_to_mjcf.py` 仅作为历史备用脚本保留，当前主流程不使用。

### 命名风格

- Python 代码遵循 PEP 8，类型提示使用 `from __future__ import annotations`。
- MuJoCo 执行器命名约定：`actuator_jointN`。

### 测试

- 运行测试：`pytest tests/`
- 新增模块建议补充基础导入/接口测试。

### 当前示例与已删除模块

- 保留示例：`01_load_model.py`、`04_real_to_sim.py`、`06_interactive_ik_mujoco.py`、
  `07_interactive_gravity_compensation_mujoco.py`、`09_real_to_sim_gravity_comp.py`。
- 已删除示例/模块：`05_sim_to_real.py`、`08_real_to_sim_grasp_feedback.py`、
  `src/rebot_b601_rs_sim/bridge/sim_to_real.py`、
  `src/rebot_b601_rs_sim/bridge/grasp_feedback.py`。
  这些功能实现复杂或当前无法稳定验证，已从工程中移除以保持精简。

### 夹爪与 MuJoCo 映射

- 真实夹爪电机读数范围：0°（闭合）~ 345°（张开）。
- MuJoCo 夹爪直线位移范围：0 ~ 0.05 m。
- 缩放系数：`0.05 / 6.021 ≈ 0.00830`。
- `assets/00_arm_rs_asm_v3/00_arm_rs_asm_v3.xml` 中夹爪作动器已注释禁用：
  real-to-sim 中夹爪由真实电机位置直接驱动，MuJoCo 不应再施加伺服力，
  否则会产生虚假接触力。

### 提交前检查

- 确保本目录及同级 SDK 下没有嵌套 `.git`。
- 确保 `assets/00_arm_rs_asm_v3/` 下的 XML 与当前主流程一致。
- 删除示例或模块时，同步清理 `README.md`、bridge `__init__.py` 及对应测试。
