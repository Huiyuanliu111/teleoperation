#!/usr/bin/env python3
"""Convert a joint-action LeRobot v3 dataset to TCP Cartesian delta actions.

The observation state remains ``[q1..q7, gripper_width]``. The action becomes
``[dx, dy, dz, drotvec_x, drotvec_y, drotvec_z, dgripper]``, expressed in the
Panda base/world frame between the current observation state and the existing
next-frame joint target. FK is evaluated at the URDF frame ``panda_hand_tcp``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from scipy.spatial.transform import Rotation


ACTION_NAMES = [
    "dx",
    "dy",
    "dz",
    "drotvec_x",
    "drotvec_y",
    "drotvec_z",
    "dgripper",
]
BASE_LINK = "panda_link0"
TCP_LINK = "panda_hand_tcp"


def _parse_vector(value: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if value is None:
        return np.asarray(default, dtype=np.float64)
    result = np.fromstring(value, sep=" ", dtype=np.float64)
    if result.shape != (3,):
        raise ValueError(f"expected a 3-vector, got {value!r}")
    return result


def origin_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    transform[:3, 3] = xyz
    return transform


class UrdfForwardKinematics:
    """Minimal URDF FK for the unique base-to-TCP serial chain."""

    def __init__(self, urdf_path: Path, base_link: str = BASE_LINK, tcp_link: str = TCP_LINK):
        root = ET.parse(urdf_path).getroot()
        by_child: dict[str, dict[str, object]] = {}
        for joint in root.findall("joint"):
            parent_element = joint.find("parent")
            child_element = joint.find("child")
            if parent_element is None or child_element is None:
                continue
            origin = joint.find("origin")
            axis = joint.find("axis")
            child = str(child_element.attrib["link"])
            by_child[child] = {
                "name": joint.attrib["name"],
                "type": joint.attrib["type"],
                "parent": str(parent_element.attrib["link"]),
                "child": child,
                "origin": origin_transform(
                    _parse_vector(origin.attrib.get("xyz") if origin is not None else None, (0, 0, 0)),
                    _parse_vector(origin.attrib.get("rpy") if origin is not None else None, (0, 0, 0)),
                ),
                "axis": _parse_vector(
                    axis.attrib.get("xyz") if axis is not None else None, (1, 0, 0)
                ),
            }

        chain: list[dict[str, object]] = []
        link = tcp_link
        while link != base_link:
            if link not in by_child:
                raise ValueError(f"URDF has no chain from {base_link!r} to {tcp_link!r}")
            joint = by_child[link]
            chain.append(joint)
            link = str(joint["parent"])
        self.chain = list(reversed(chain))
        self.movable = [joint for joint in self.chain if joint["type"] in {"revolute", "continuous"}]
        if len(self.movable) != 7:
            names = [joint["name"] for joint in self.movable]
            raise ValueError(f"expected seven movable Panda joints, got {names}")

    def pose(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError(f"expected a finite 7D joint vector, got shape {q.shape}")
        transform = np.eye(4, dtype=np.float64)
        q_index = 0
        for joint in self.chain:
            transform = transform @ np.asarray(joint["origin"])
            if joint["type"] in {"revolute", "continuous"}:
                axis = np.asarray(joint["axis"], dtype=np.float64)
                axis /= np.linalg.norm(axis)
                rotation = np.eye(4, dtype=np.float64)
                rotation[:3, :3] = Rotation.from_rotvec(axis * q[q_index]).as_matrix()
                transform = transform @ rotation
                q_index += 1
            elif joint["type"] != "fixed":
                raise ValueError(f"unsupported joint type {joint['type']!r} on TCP chain")
        return transform

    def poses(self, joints: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        joints = np.asarray(joints, dtype=np.float64)
        positions = np.empty((len(joints), 3), dtype=np.float64)
        rotations = np.empty((len(joints), 3, 3), dtype=np.float64)
        for index, q in enumerate(joints):
            pose = self.pose(q)
            positions[index] = pose[:3, 3]
            rotations[index] = pose[:3, :3]
        return positions, rotations


def cartesian_delta_actions(
    states: np.ndarray, joint_targets: np.ndarray, fk: UrdfForwardKinematics
) -> np.ndarray:
    states = np.asarray(states, dtype=np.float64)
    joint_targets = np.asarray(joint_targets, dtype=np.float64)
    if states.shape != joint_targets.shape or states.ndim != 2 or states.shape[1] != 8:
        raise ValueError(f"expected matching Nx8 state/action arrays, got {states.shape} and {joint_targets.shape}")
    current_position, current_rotation = fk.poses(states[:, :7])
    target_position, target_rotation = fk.poses(joint_targets[:, :7])
    # Left-multiplication error is expressed in the base/world coordinates.
    relative_rotation = target_rotation @ np.swapaxes(current_rotation, 1, 2)
    rotation_delta = Rotation.from_matrix(relative_rotation).as_rotvec()
    result = np.concatenate(
        (
            target_position - current_position,
            rotation_delta,
            joint_targets[:, 7:8] - states[:, 7:8],
        ),
        axis=1,
    )
    return result.astype(np.float32)


def _fixed_list_to_numpy(column: pa.ChunkedArray) -> np.ndarray:
    return np.asarray(column.to_pylist(), dtype=np.float32)


def _fixed_float_list(values: np.ndarray) -> pa.FixedSizeListArray:
    values = np.asarray(values, dtype=np.float32)
    return pa.FixedSizeListArray.from_arrays(
        pa.array(values.reshape(-1), type=pa.float32()), values.shape[1]
    )


def _jsonable(value: object) -> object:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    return value


def _update_huggingface_metadata(table: pa.Table, actions: np.ndarray) -> pa.Table:
    metadata = dict(table.schema.metadata or {})
    if b"huggingface" in metadata:
        huggingface = json.loads(metadata[b"huggingface"].decode("utf-8"))
        action_feature = huggingface["info"]["features"]["action"]
        action_feature["length"] = 7
        huggingface["fingerprint"] = hashlib.sha256(actions.tobytes()).hexdigest()[:16]
        metadata[b"huggingface"] = json.dumps(huggingface, separators=(",", ":")).encode()
    return table.replace_schema_metadata(metadata)


def _episode_action_stats(actions: np.ndarray, episode_indices: np.ndarray) -> list[dict]:
    from lerobot.datasets.compute_stats import compute_episode_stats

    feature = {"action": {"dtype": "float32", "shape": (7,), "names": ACTION_NAMES}}
    return [
        compute_episode_stats({"action": actions[episode_indices == episode]}, feature)
        for episode in np.unique(episode_indices)
    ]


def _rewrite_episode_stats(path: Path, episode_stats: list[dict]) -> None:
    table = pq.read_table(path)
    if len(table) != len(episode_stats):
        raise ValueError("episode metadata count does not match data episode count")
    for stat_name in episode_stats[0]["action"]:
        column_name = f"stats/action/{stat_name}"
        column_index = table.schema.get_field_index(column_name)
        if column_index < 0:
            raise KeyError(f"missing episode metadata column {column_name!r}")
        values = [_jsonable(stats["action"][stat_name]) for stats in episode_stats]
        field = table.schema.field(column_index)
        table = table.set_column(column_index, field, pa.array(values, type=field.type))
    pq.write_table(table, path, compression="zstd")


def convert(source: Path, output: Path, urdf_path: Path) -> dict:
    source = source.resolve()
    output = output.resolve()
    urdf_path = urdf_path.resolve()
    building = output.with_name(output.name + ".building")
    if output.exists() or building.exists():
        raise FileExistsError(f"refusing to overwrite {output} or temporary {building}")
    info_path = source / "meta" / "info.json"
    if not info_path.exists():
        raise FileNotFoundError(f"missing LeRobot metadata: {info_path}")
    source_info = json.loads(info_path.read_text())
    if not str(source_info.get("codebase_version", "")).startswith("v3"):
        raise ValueError("source must be a LeRobot v3 dataset")
    if source_info["features"]["observation.state"]["shape"] != [8]:
        raise ValueError("source observation.state must be [q1..q7, gripper_width]")
    if source_info["features"]["action"]["shape"] != [8]:
        raise ValueError("source action must be next-frame [q1..q7, gripper_width]")

    fk = UrdfForwardKinematics(urdf_path)
    shutil.copytree(source, building, copy_function=shutil.copy2)
    try:
        parquet_paths = sorted((building / "data").rglob("*.parquet"))
        if not parquet_paths:
            raise FileNotFoundError("source has no data parquet files")
        all_actions: list[np.ndarray] = []
        all_episodes: list[np.ndarray] = []
        episode_diagnostics: list[dict[str, float | int]] = []
        for parquet_path in parquet_paths:
            table = pq.read_table(parquet_path)
            states = _fixed_list_to_numpy(table.column("observation.state"))
            joint_targets = _fixed_list_to_numpy(table.column("action"))
            episodes = np.asarray(table.column("episode_index"), dtype=np.int64)
            actions = cartesian_delta_actions(states, joint_targets, fk)
            action_index = table.schema.get_field_index("action")
            table = table.set_column(action_index, "action", _fixed_float_list(actions))
            table = _update_huggingface_metadata(table, actions)
            pq.write_table(table, parquet_path, compression="zstd")
            all_actions.append(actions)
            all_episodes.append(episodes)

        actions = np.concatenate(all_actions)
        episode_indices = np.concatenate(all_episodes)
        episode_stats = _episode_action_stats(actions, episode_indices)
        _rewrite_episode_stats(
            building / "meta" / "episodes" / "chunk-000" / "file-000.parquet",
            episode_stats,
        )

        from lerobot.datasets.compute_stats import aggregate_stats

        stats_path = building / "meta" / "stats.json"
        stats = json.loads(stats_path.read_text())
        stats["action"] = _jsonable(aggregate_stats(episode_stats)["action"])
        stats_path.write_text(json.dumps(stats, indent=2, ensure_ascii=False) + "\n")

        info_path = building / "meta" / "info.json"
        info = json.loads(info_path.read_text())
        info["features"]["action"] = {
            "dtype": "float32",
            "shape": [7],
            "names": ACTION_NAMES,
        }
        info_path.write_text(json.dumps(info, indent=4, ensure_ascii=False))

        for episode in np.unique(episode_indices):
            values = actions[episode_indices == episode]
            episode_diagnostics.append(
                {
                    "episode_index": int(episode),
                    "frames": int(len(values)),
                    "max_translation_delta_m": float(np.linalg.norm(values[:, :3], axis=1).max()),
                    "max_rotation_delta_rad": float(np.linalg.norm(values[:, 3:6], axis=1).max()),
                    "max_gripper_delta_m": float(np.abs(values[:, 6]).max()),
                }
            )
        report = {
            "format": "LeRobotDataset-v3.0",
            "source": str(source),
            "output": str(output),
            "fk_urdf": str(urdf_path),
            "fk_base_link": BASE_LINK,
            "fk_tcp_link": TCP_LINK,
            "observation_state": "joint [q1..q7, gripper_width]",
            "source_action": "next-frame joint [q1..q7, gripper_width]",
            "action": ACTION_NAMES,
            "translation_frame": "panda base/world",
            "rotation_convention": "rotvec(R_target @ R_current.T), panda base/world frame",
            "total_frames": int(len(actions)),
            "episodes": episode_diagnostics,
        }
        (building / "meta" / "cartesian_conversion_report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n"
        )
        source_conversion = building / "meta" / "vla_conversion_report.json"
        if source_conversion.exists():
            source_conversion.rename(building / "meta" / "source_vla_conversion_report.json")
        stale_validation = building / "meta" / "validation_report.json"
        if stale_validation.exists():
            stale_validation.unlink()
        os.replace(building, output)
        return report
    except BaseException:
        print(f"partial output remains at {building}; inspect it before retrying")
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path("remote_controller/src/remote_controller/assets/panda/panda_arm.urdf"),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    report = convert(args.source, args.output, args.urdf)
    print(json.dumps({"output": report["output"], "frames": report["total_frames"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
