#!/usr/bin/env python3
"""将 reBotArm B601-RS 的 URDF 简单转换为 MuJoCo MJCF。

该脚本仅用于框架搭建阶段生成一个可运行的 MJCF 模型。
它保留了 URDF 中的关节层级、坐标系与限位，并用 capsule/box 近似几何体。
后续可替换为更精细的网格模型。
"""

from __future__ import annotations

import argparse
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


_ARM_JOINT_RE = re.compile(r"^joint[1-6]$")


def parse_xyz_rpy(elem: ET.Element | None) -> tuple[np.ndarray, np.ndarray]:
    """解析 origin 元素的 xyz 和 rpy 属性。"""
    if elem is None:
        return np.zeros(3), np.zeros(3)
    xyz = np.array([float(x) for x in (elem.get("xyz") or "0 0 0").split()])
    rpy = np.array([float(x) for x in (elem.get("rpy") or "0 0 0").split()])
    return xyz, rpy


def rpy_to_quat(rpy: np.ndarray) -> np.ndarray:
    """将 RPY (XYZ 欧拉角) 转换为 MuJoCo 的 wxyz 四元数。"""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])


def build_link_tree(root: ET.Element) -> dict[str, dict]:
    """构建 link 字典，包含惯性、视觉、碰撞信息。"""
    links: dict[str, dict] = {}
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is not None:
            origin = inertial.find("origin")
            i_xyz, i_rpy = parse_xyz_rpy(origin)
            mass = float(inertial.find("mass").get("value") or "1.0")
            inertia = inertial.find("inertia")
            if inertia is not None:
                diag = np.array([
                    float(inertia.get("ixx") or "1e-4"),
                    float(inertia.get("iyy") or "1e-4"),
                    float(inertia.get("izz") or "1e-4"),
                ])
            else:
                diag = np.array([1e-4, 1e-4, 1e-4])
        else:
            i_xyz, i_rpy = np.zeros(3), np.zeros(3)
            mass = 1.0
            diag = np.array([1e-4, 1e-4, 1e-4])
        links[name] = {
            "name": name,
            "mass": mass,
            "inertia": diag,
            "inertial_origin": i_xyz,
            "inertial_rpy": i_rpy,
        }
    return links


def build_joint_tree(root: ET.Element) -> dict[str, dict]:
    """构建关节字典，key 为 child link 名称。"""
    joints: dict[str, dict] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        jtype = joint.get("type")
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        xyz, rpy = parse_xyz_rpy(origin)
        axis_vec = np.array([float(x) for x in (axis.get("xyz") if axis is not None else "0 0 1").split()])
        lower = float(limit.get("lower")) if limit is not None else -np.pi
        upper = float(limit.get("upper")) if limit is not None else np.pi
        effort = float(limit.get("effort")) if limit is not None and limit.get("effort") else 100.0
        joints[child] = {
            "name": name,
            "type": jtype,
            "parent": parent,
            "child": child,
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis_vec,
            "lower": lower,
            "upper": upper,
            "effort": effort,
        }
    return joints


def find_children(parent_name: str, joints: dict[str, dict]) -> list[dict]:
    """查找某个 link 的所有子关节。"""
    return [j for j in joints.values() if j["parent"] == parent_name]


def body_xml(
    link_name: str,
    links: dict[str, dict],
    joints: dict[str, dict],
    body_pos: np.ndarray | None = None,
    body_quat: np.ndarray | None = None,
    depth: int = 2,
    indent: str = "    ",
) -> list[str]:
    """递归生成 MuJoCo body XML。

    当前 link 的位姿由父 joint 的 origin 决定（即 body_pos / body_quat）。
    在该 body 内部，先放置本 link 的几何体/惯性，再递归放置由子 joint 连接的子 link。
    """
    prefix = indent * depth
    link = links.get(link_name, {})
    mass = link.get("mass", 1.0)
    inertia = link.get("inertia", np.array([1e-4, 1e-4, 1e-4]))
    i_xyz = link.get("inertial_origin", np.zeros(3))

    pos_str = "0 0 0" if body_pos is None else " ".join(f"{x:.6f}" for x in body_pos)
    quat_str = "1 0 0 0" if body_quat is None else " ".join(f"{x:.6f}" for x in body_quat)

    lines: list[str] = []
    lines.append(f'{prefix}<body name="{link_name}" pos="{pos_str}" quat="{quat_str}">')
    lines.append(
        f'{prefix}  <inertial pos="{" ".join(f"{x:.4f}" for x in i_xyz)}" mass="{mass:.4f}" diaginertia="{" ".join(f"{x:.6f}" for x in inertia)}"/>'
    )

    # 本 link 的近似几何体
    geom_size, geom_fromto = _approximate_geometry(link_name)
    geom_attrs = (
        f'type="capsule" size="{geom_size:.4f}" '
        f'fromto="{" ".join(f"{x:.4f}" for x in geom_fromto)}" material="link"'
    )
    lines.append(f'{prefix}  <geom {geom_attrs}/>')

    # 递归处理子 link
    for joint in find_children(link_name, joints):
        child_link = joint["child"]
        jtype = joint["type"]
        quat = rpy_to_quat(joint["rpy"])

        if jtype in ("revolute", "prismatic"):
            # 子 body 的位姿 = joint 的 origin；joint 定义在该 body 内部
            mj_type = "hinge" if jtype == "revolute" else "slide"
            axis = joint["axis"]
            lines.append(
                f'{prefix}  <joint name="{joint["name"]}" type="{mj_type}" axis="{" ".join(f"{x:.6f}" for x in axis)}" '
                f'range="{joint["lower"]:.4f} {joint["upper"]:.4f}" damping="0.5" armature="0.01"/>'
            )
            lines.extend(
                body_xml(
                    child_link,
                    links,
                    joints,
                    body_pos=joint["xyz"],
                    body_quat=quat,
                    depth=depth + 1,
                    indent=indent,
                )
            )
        elif jtype == "fixed":
            # fixed joint：不创建 joint 标签，将 child body 合并到当前 body 的坐标系下
            lines.extend(
                body_xml(
                    child_link,
                    links,
                    joints,
                    body_pos=joint["xyz"],
                    body_quat=quat,
                    depth=depth + 1,
                    indent=indent,
                )
            )

    lines.append(f"{prefix}</body>")
    return lines


def _approximate_geometry(link_name: str) -> tuple[float, np.ndarray]:
    """根据 link 名称返回近似的胶囊几何体参数 (radius, fromto)。"""
    defaults: dict[str, tuple[float, np.ndarray]] = {
        "base_link": (0.060, np.array([0, 0, 0, 0, 0, 0.08])),
        "link1": (0.030, np.array([0, 0, 0, 0, 0, 0.08])),
        "link2": (0.025, np.array([0, 0, 0, -0.12, 0, 0])),
        "link3": (0.025, np.array([0, 0, 0, -0.12, 0, 0])),
        "link4": (0.020, np.array([0, 0, 0, 0, 0, 0.06])),
        "link5": (0.020, np.array([0, 0, 0, 0, 0, 0.05])),
        "link6": (0.015, np.array([0, 0, 0, 0, 0, 0.04])),
    }
    return defaults.get(link_name, (0.02, np.array([0, 0, 0, 0, 0, 0.05])))


def generate_mjcf(urdf_path: Path, output_path: Path) -> None:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    robot_name = root.get("name", "robot")

    links = build_link_tree(root)
    joints = build_joint_tree(root)

    body_lines = body_xml("base_link", links, joints, depth=2)
    actuator_lines: list[str] = []
    for joint in joints.values():
        if joint["type"] not in ("revolute", "prismatic"):
            continue
        if not _ARM_JOINT_RE.match(joint["name"]):
            continue
        actuator_lines.append(
            f'    <position name="actuator_{joint["name"]}" joint="{joint["name"]}" '
            f'kp="100" kv="10" ctrllimited="true" ctrlrange="{joint["lower"]:.4f} {joint["upper"]:.4f}"/>'
        )

    xml = [
        f'<mujoco model="{robot_name}">',
        '  <compiler angle="radian" meshdir="meshes" autolimits="true" balanceinertia="true"/>',
        '  <option timestep="0.002" gravity="0 0 -9.81" iterations="50" solver="Newton" integrator="Euler"/>',
        '  <default>',
        '    <joint damping="0.5" armature="0.01"/>',
        '    <geom friction="0.9 0.2 0.2" rgba="0.7 0.7 0.7 1"/>',
        '  </default>',
        '  <asset>',
        '    <material name="link" rgba="0.75 0.75 0.75 1"/>',
        '    <material name="joint" rgba="0.2 0.6 0.9 1"/>',
        '  </asset>',
        '  <worldbody>',
    ]
    xml.extend(body_lines)
    xml.append('  </worldbody>')
    xml.append('  <actuator>')
    xml.extend(actuator_lines)
    xml.append('  </actuator>')
    xml.append('</mujoco>')

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(xml), encoding="utf-8")
    print(f"[convert] MJCF written to: {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert reBotArm RS URDF to a simplified MJCF.")
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "reBotArm_control_py"
        / "urdf" / "00-arm-rs_asm-v3" / "urdf" / "00-arm-rs_asm-v3.urdf",
        help="Input URDF path.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "assets" / "00_arm_rs_asm_v3" / "rebot_simplified.xml",
        help="Output MJCF path.",
    )
    args = parser.parse_args()
    generate_mjcf(args.urdf, args.output)


if __name__ == "__main__":
    main()
