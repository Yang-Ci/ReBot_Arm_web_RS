# Gravity calibration harness (RS build)

The scripts and data behind `docs/gravity_calibration_rs_2026-07-17.md`.
Measured on a RobStride build (rs-06 ×3 + rs-00 ×4, ids 1–7, host 0xFD,
SocketCAN `can0` @ 1 Mbps). Requires `motorbridge`, `pinocchio`, `numpy`.

**Safety model:** the PD loop is always in command during identification
(no open-loop feedforward while measuring); torque feedforwards soft-start
over 2 s; every abort FREEZES with a PD hold (never torque-off — a loaded
arm free-falls); velocity aborts use finite differences of mechPos
(`mechVel` 0x701A is not rad/s on this firmware). Keep a hand on the e-stop
anyway.

## Scripts

| script | what it does |
|---|---|
| `phase_a_sign_probe.py` | NO-torque sign check: streams mechPos while you move joints by hand — anchors the motor→URDF sign convention before any torque is applied. |
| `pd_sweep_id.py` | Per-joint gravity identification: PD-servos the joint slowly across a range (up+down); holding torque comes from the tracking error (`tau ≈ kp·err − kd·v`). `--pre joint2=0.7 --pre joint3=1.1` pre-positions to a clearance pose first. Logs full 6-joint pose per sample. |
| `fit_sweeps.py` | Fits sweep JSONs against the URDF model: reports scale k, zero offset, bias, Coulomb friction, rms. |
| `auto_float_test.py` | Validation: auto-lifts one joint, fades PD to zero and floats it on gravity feedforward alone; drift/integral measure the model error at that pose. |

## Critical lesson: clearance

Sweeps run from poses where anything touches — gripper on the table at the
rest pose, link-to-link contact in folded poses, base-yaw dragging the
gripper — produce **convincing but fake fits** (we got a −0.40 rad phantom
zero offset, a ×1.66 phantom wrist scale and a phantom "cable spring" with
good-looking rms before catching it). Always pre-position to a clearance
pose (elbow-up "L": `--pre joint2=0.7 --pre joint3=1.1`) and/or exclude the
contact region with `fit_sweeps.py --qmin`.

## Reference data (`data/`)

Contact-free sweeps from 2026-07-17 and their fits (`final_clean_fits.json`):
URDF mass model correct to ~5–11% on all load-bearing joints, no zero
offsets, biases below friction. `autofloat_joint3_vendor_*.json`: elbow
float validation, drift −0.0002 rad over 12 s.

Reproduce the table:

```bash
python fit_sweeps.py "data/pdsweep_joint4_clear_*.json" \
                     "data/pdsweep_joint5_clear_*.json"
python fit_sweeps.py "data/pdsweep_joint2_Lpose_*.json" --qmin 0.25  # skip near-table start
python fit_sweeps.py "data/pdsweep_joint3_*.json" --qmin 0.4         # airborne part
```

Torque-scale caveat: everything is derived from the MIT-mode kp (assumed
exact N·m/rad); there is no independent torque sensor, so a firmware kp
error of a few percent shifts all k values equally.
