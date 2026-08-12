#!/usr/bin/env python3
"""Split the upper RS Seeed Studio badge from its black PLA binary STL.

The SolidWorks export stores the long rounded badge as two disconnected shells
(one on each side of the arm).  The counters inside e/d/o are separate islands,
so they must move with the badge as well.  Keeping the original coordinates and
writing those components to their own STL lets URDF/MJCF assign the yellow
finish while the recessed Seeed Studio lettering reveals the black body.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import math
import struct


_HEADER_SIZE = 84
_TRIANGLE_SIZE = 50
_BADGE_EXTENTS_M = (0.05125, 0.01450, 0.00100)
_EXTENT_TOLERANCE_M = 0.0005
_COUNTER_EXTENTS_M = (
    (0.002619, 0.002639, 0.001000),
    (0.002328, 0.000886, 0.001000),
)
_COUNTER_TOLERANCE_M = 0.0001
_BACKING_EDGE_INSET_M = 0.00015
_BACKING_FRONT_OFFSET_M = 0.00015
_BACKING_BACK_OFFSET_M = 0.00005
_CAPSULE_ARC_STEPS = 24


@dataclass(frozen=True)
class Component:
    triangles: tuple[int, ...]
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]
    extents: tuple[float, float, float]


def _read_binary_stl(path: Path) -> tuple[bytes, list[bytes], list[tuple[bytes, ...]]]:
    payload = path.read_bytes()
    if len(payload) < _HEADER_SIZE:
        raise ValueError(f"{path} is too short to be a binary STL")

    triangle_count = struct.unpack_from("<I", payload, 80)[0]
    expected_size = _HEADER_SIZE + triangle_count * _TRIANGLE_SIZE
    if len(payload) != expected_size:
        raise ValueError(
            f"{path} is not the expected binary STL layout "
            f"({len(payload)} bytes, expected {expected_size})"
        )

    records: list[bytes] = []
    vertex_keys: list[tuple[bytes, ...]] = []
    for index in range(triangle_count):
        start = _HEADER_SIZE + index * _TRIANGLE_SIZE
        record = payload[start : start + _TRIANGLE_SIZE]
        records.append(record)
        vertex_keys.append(tuple(record[offset : offset + 12] for offset in (12, 24, 36)))
    return payload[:80], records, vertex_keys


def _components(records: list[bytes], vertex_keys: list[tuple[bytes, ...]]) -> list[Component]:
    count = len(records)
    parent = list(range(count))
    sizes = [1] * count

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(left: int, right: int) -> None:
        left_root = find(left)
        right_root = find(right)
        if left_root == right_root:
            return
        if sizes[left_root] < sizes[right_root]:
            left_root, right_root = right_root, left_root
        parent[right_root] = left_root
        sizes[left_root] += sizes[right_root]

    vertex_owner: dict[bytes, int] = {}
    for triangle_index, keys in enumerate(vertex_keys):
        for key in keys:
            previous = vertex_owner.setdefault(key, triangle_index)
            union(triangle_index, previous)

    grouped: dict[int, list[int]] = {}
    for triangle_index in range(count):
        grouped.setdefault(find(triangle_index), []).append(triangle_index)

    components: list[Component] = []
    for triangle_indices in grouped.values():
        minimum = [float("inf")] * 3
        maximum = [float("-inf")] * 3
        for triangle_index in triangle_indices:
            record = records[triangle_index]
            for offset in (12, 24, 36):
                vertex = struct.unpack_from("<3f", record, offset)
                for axis, value in enumerate(vertex):
                    minimum[axis] = min(minimum[axis], value)
                    maximum[axis] = max(maximum[axis], value)
        components.append(
            Component(
                triangles=tuple(triangle_indices),
                minimum=tuple(minimum),
                maximum=tuple(maximum),
                extents=tuple(maximum[axis] - minimum[axis] for axis in range(3)),
            )
        )
    return components


def _is_badge(component: Component) -> bool:
    return all(
        abs(actual - expected) <= _EXTENT_TOLERANCE_M
        for actual, expected in zip(component.extents, _BADGE_EXTENTS_M)
    )


def _is_wordmark_counter(component: Component) -> bool:
    return any(
        all(
            abs(actual - expected) <= _COUNTER_TOLERANCE_M
            for actual, expected in zip(component.extents, expected_extents)
        )
        for expected_extents in _COUNTER_EXTENTS_M
    )


def _write_binary_stl(path: Path, header: bytes, records: list[bytes], indices: list[int]) -> None:
    label = b"reBot RS split Seeed badge"
    output_header = (label + header[len(label) :])[:80]
    with path.open("wb") as stream:
        stream.write(output_header)
        stream.write(struct.pack("<I", len(indices)))
        for triangle_index in indices:
           stream.write(records[triangle_index])


def _triangle_record(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
    third: tuple[float, float, float],
) -> bytes:
    left = tuple(second[axis] - first[axis] for axis in range(3))
    right = tuple(third[axis] - first[axis] for axis in range(3))
    normal = (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )
    length = math.sqrt(sum(value * value for value in normal))
    unit_normal = tuple(value / length for value in normal)
    return struct.pack("<12fH", *unit_normal, *first, *second, *third, 0)


def _wordmark_backing_records(badge_components: list[Component]) -> list[bytes]:
    records: list[bytes] = []
    for component in badge_components:
        x_min = component.minimum[0] + _BACKING_EDGE_INSET_M
        x_max = component.maximum[0] - _BACKING_EDGE_INSET_M
        y_min = component.minimum[1] + _BACKING_EDGE_INSET_M
        y_max = component.maximum[1] - _BACKING_EDGE_INSET_M
        center_y = (y_min + y_max) / 2
        radius = (y_max - y_min) / 2
        left_center_x = x_min + radius
        right_center_x = x_max - radius

        outline: list[tuple[float, float]] = []
        for step in range(_CAPSULE_ARC_STEPS + 1):
            angle = -math.pi / 2 + math.pi * step / _CAPSULE_ARC_STEPS
            outline.append(
                (
                    right_center_x + radius * math.cos(angle),
                    center_y + radius * math.sin(angle),
                )
            )
        for step in range(_CAPSULE_ARC_STEPS + 1):
            angle = math.pi / 2 + math.pi * step / _CAPSULE_ARC_STEPS
            outline.append(
                (
                    left_center_x + radius * math.cos(angle),
                    center_y + radius * math.sin(angle),
                )
            )

        outward = 1.0 if component.maximum[2] > 0 else -1.0
        inner_z = component.minimum[2] if outward > 0 else component.maximum[2]
        front_z = inner_z + outward * _BACKING_FRONT_OFFSET_M
        back_z = inner_z + outward * _BACKING_BACK_OFFSET_M
        center = ((x_min + x_max) / 2, center_y)

        for index, point in enumerate(outline):
            following = outline[(index + 1) % len(outline)]
            front_center = (center[0], center[1], front_z)
            front_first = (point[0], point[1], front_z)
            front_second = (following[0], following[1], front_z)
            back_center = (center[0], center[1], back_z)
            back_first = (point[0], point[1], back_z)
            back_second = (following[0], following[1], back_z)

            if outward > 0:
                records.append(_triangle_record(front_center, front_first, front_second))
                records.append(_triangle_record(back_center, back_second, back_first))
            else:
                records.append(_triangle_record(front_center, front_second, front_first))
                records.append(_triangle_record(back_center, back_first, back_second))

            records.append(_triangle_record(back_first, front_first, front_second))
            records.append(_triangle_record(back_first, front_second, back_second))
    return records


def _write_generated_stl(path: Path, header: bytes, records: list[bytes]) -> None:
    label = b"reBot RS Seeed wordmark backing"
    output_header = (label + header[len(label) :])[:80]
    with path.open("wb") as stream:
        stream.write(output_header)
        stream.write(struct.pack("<I", len(records)))
        for record in records:
            stream.write(record)


def split_mesh(source: Path) -> tuple[Path, Path, Path, int, int, int]:
    header, records, vertex_keys = _read_binary_stl(source)
    components = _components(records, vertex_keys)
    badge_components = [component for component in components if _is_badge(component)]
    if len(badge_components) != 2:
        extents = ", ".join(str(tuple(round(value, 5) for value in item.extents)) for item in components)
        raise RuntimeError(
            f"Expected two badge shells in {source}, found {len(badge_components)}; "
            f"component extents: {extents}"
        )

    counter_components = [
        component for component in components if _is_wordmark_counter(component)
    ]
    if len(counter_components) != 12:
        extents = ", ".join(str(tuple(round(value, 5) for value in item.extents)) for item in components)
        raise RuntimeError(
            f"Expected twelve wordmark counter islands in {source}, "
            f"found {len(counter_components)}; component extents: {extents}"
        )

    badge_indices = sorted(
        index
        for component in (*badge_components, *counter_components)
        for index in component.triangles
    )
    badge_set = set(badge_indices)
    body_indices = [index for index in range(len(records)) if index not in badge_set]

    stem = source.stem.removesuffix("_black")
    body_path = source.with_name(f"{stem}_black_without_seeed_badge.STL")
    badge_path = source.with_name(f"{stem}_seeed_badge_with_counters.STL")
    backing_path = source.with_name(f"{stem}_seeed_wordmark_backing.STL")
    backing_records = _wordmark_backing_records(badge_components)
    _write_binary_stl(body_path, header, records, body_indices)
    _write_binary_stl(badge_path, header, records, badge_indices)
    _write_generated_stl(backing_path, header, backing_records)
    return (
        body_path,
        badge_path,
        backing_path,
        len(body_indices),
        len(badge_indices),
        len(backing_records),
    )


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    default_mesh_dirs = [
        project_root / "rebotarm_ros2/src/rebotarm_bringup/description/meshes_rs",
        project_root / "reBotArm_simulator-RS/description/meshes_rs",
        project_root
        / "rebotarm_ros2/third_party/reBot-B601-RS-for-mujoco_sim"
        / "assets/00_arm_rs_asm_v3/meshes",
    ]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mesh-dir",
        action="append",
        type=Path,
        help="Mesh directory to update; may be supplied more than once.",
    )
    args = parser.parse_args()

    for mesh_dir in args.mesh_dir or default_mesh_dirs:
        # Only pla3 carries the Seeed Studio wordmark. pla2's lower-arm detail
        # must remain exactly as exported, so it is intentionally not split.
        for source_name in ("pla3_black.STL",):
            body, badge, backing, body_count, badge_count, backing_count = split_mesh(
                mesh_dir / source_name
            )
            print(
                f"{mesh_dir}: {body.name}={body_count} triangles, "
                f"{badge.name}={badge_count} triangles, "
                f"{backing.name}={backing_count} triangles"
            )


if __name__ == "__main__":
    main()
