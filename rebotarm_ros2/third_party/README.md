# Vendored third-party sources

These directories are ordinary files tracked by the parent
`ReBot_Arm_web_RS` repository. They are intentionally **not** Git submodules or
nested repositories.

| Directory | Upstream | Baseline revision |
|---|---|---|
| `reBotArm_control_py` | `vectorBH6/reBotArm_control_py` | `40ab6ce58fec3c58cb603efb3f30240d6f5849e4` |
| `reBot-B601-RS-for-mujoco_sim` | `LAN-GER/reBot-B601-RS-for-mujoco_sim` | `1249cb6efdf393ba636056fc41df30dc6ba389aa` |

The SDK includes the RS Cartesian trajectory duration safety adjustment. The
MuJoCo tree includes the RS gripper, material, and Seeed-badge model updates.
These integrated files are now the single source of truth; no patch or override
copy is required.

Do not run `git init` or clone another repository inside these directories.
Update vendored files through the parent repository so one normal clone contains
the complete build inputs.
