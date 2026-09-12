#!/usr/bin/env python3
"""Build the high-resolution two-camera point-cloud dataset for MVT ARP.

Rows are recovered by exact state/action matching against the filtered 6 Hz
LeRobot dataset.  RGB-D always comes from the original 640x480 recordings; no
2-D resize or crop is applied.  Only invalid depth and points outside the fixed
paper-style scene cube are rejected.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import h5py
import numpy as np
import pyarrow.parquet as pq

from build_vla_pointcloud_dataset import (
    DEFAULT_BOUNDS_M, DEFAULT_FIXED_CAMERAS, _camera_geometry, _video_count,
    camera_rays, depth_to_base_points,
)
from convert_vla_to_lerobot_v3 import (
    SequentialVideoReader, close_depth_source, discover_trials,
    read_camera_timestamp_csv, read_follower_matrix, timestamp_alignment,
)


def _columns(path: Path) -> dict[str, np.ndarray]:
    table = pq.read_table(path / "data/chunk-000/file-000.parquet")
    return {
        "episode": np.asarray(table["episode_index"], dtype=np.int64),
        "state": np.asarray(table["observation.state"].to_pylist(), dtype=np.float32),
        "action": np.asarray(table["action"].to_pylist(), dtype=np.float32),
    }


def recover_source_rows(source: dict[str, np.ndarray], final: dict[str, np.ndarray],
                        episode: int) -> np.ndarray:
    """Match final filtered rows to the source 30 Hz table exactly."""
    source_indices = np.flatnonzero(source["episode"] == episode)
    final_indices = np.flatnonzero(final["episode"] == episode)
    candidates = source_indices[::5]
    # State and action together make repeated stationary rows unambiguous.  A
    # monotone scan preserves temporal identity even when values repeat.
    result, begin = [], 0
    for target in final_indices:
        same_state = np.max(np.abs(source["state"][candidates] - final["state"][target]), axis=1) <= 1e-7
        same_action = np.max(np.abs(source["action"][candidates] - final["action"][target]), axis=1) <= 1e-7
        found = np.flatnonzero(same_state & same_action & (np.arange(len(candidates)) >= begin))
        if not len(found):
            raise ValueError(f"episode {episode}: cannot map filtered row {target}")
        chosen = int(found[0])
        result.append(int(candidates[chosen] - source_indices[0]))
        begin = chosen + 1
    rows = np.asarray(result, dtype=np.int64)
    if len(rows) and (np.any(rows % 5) or np.any(np.diff(rows) <= 0)):
        raise AssertionError(f"episode {episode}: recovered rows are not monotone stride-5 rows")
    return rows


def deterministic_surface_points(points: list[np.ndarray], colors: list[np.ndarray],
                                 bounds: np.ndarray, max_points: int,
                                 voxel_m: float) -> tuple[np.ndarray, np.ndarray, int, int]:
    xyz = np.concatenate(points)
    rgb = np.concatenate(colors)
    keep = np.isfinite(xyz).all(1) & (xyz >= bounds[:3]).all(1) & (xyz <= bounds[3:]).all(1)
    xyz, rgb = xyz[keep], rgb[keep]
    raw_count = len(xyz)
    if raw_count == 0:
        raise ValueError("scene bounds removed every valid depth point")
    voxel = np.floor((xyz - bounds[:3]) / voxel_m).astype(np.int32)
    key = np.ravel_multi_index(voxel.T, tuple((np.ceil((bounds[3:] - bounds[:3]) / voxel_m) + 1).astype(int)))
    # Stable first return keeps a real observed surface sample per occupied voxel.
    order = np.argsort(key, kind="stable")
    unique = order[np.r_[True, key[order][1:] != key[order][:-1]]]
    if len(unique) > max_points:
        unique = unique[np.linspace(0, len(unique) - 1, max_points).round().astype(np.int64)]
    valid = len(unique)
    out_xyz = np.zeros((max_points, 3), dtype=np.float32)
    out_rgb = np.zeros((max_points, 3), dtype=np.uint8)
    out_xyz[:valid], out_rgb[:valid] = xyz[unique], rgb[unique]
    return out_xyz, out_rgb, valid, raw_count


def build(args: argparse.Namespace) -> None:
    raw_root, output = args.raw_root.resolve(), args.output.resolve()
    if output.exists() or output.with_suffix(output.suffix + ".partial").exists():
        raise FileExistsError(f"refusing to overwrite {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    source, final = _columns(args.source_dataset.resolve()), _columns(args.filtered_dataset.resolve())
    calibration = json.loads(args.calibration.read_text())
    trials = discover_trials(raw_root)
    episode_count = int(final["episode"].max()) + 1
    if episode_count != len(trials):
        raise ValueError(f"filtered data has {episode_count} episodes but raw root has {len(trials)}")
    bounds = np.asarray(args.bounds, dtype=np.float32)
    partial = output.with_suffix(output.suffix + ".partial")
    reports = []
    with h5py.File(partial, "w") as target:
        target.attrs.update({
            "format": "threading-mvt-pointcloud-v1",
            "raw_root": str(raw_root), "source_dataset": str(args.source_dataset.resolve()),
            "filtered_dataset": str(args.filtered_dataset.resolve()),
            "calibration_path": str(args.calibration.resolve()),
            "source_image_size": json.dumps([640, 480]),
            "virtual_image_size": 420, "virtual_views_json": json.dumps(["top", "left"]),
            "bounds_m": bounds, "max_points": args.max_points, "voxel_m": args.voxel_m,
            "temporal_stride": 5, "zero_actions_removed": True,
        })
        for episode, trial in enumerate(trials):
            source_rows = recover_source_rows(source, final, episode)
            final_indices = np.flatnonzero(final["episode"] == episode)
            matrix = read_follower_matrix(trial / "DATA_follower.m")
            robot_timestamps = matrix[:, 0].astype(np.int64)
            captures, readers, geometries, depth_sources = [], [], [], []
            try:
                streams, paths = [], []
                for filename, key in DEFAULT_FIXED_CAMERAS:
                    path = trial / filename
                    cap = cv2.VideoCapture(str(path))
                    if not cap.isOpened(): raise ValueError(f"cannot open {path}")
                    captures.append(cap); paths.append(path)
                    streams.append(read_camera_timestamp_csv(
                        trial / f"{Path(filename).stem}_timestamps.csv", _video_count(cap, path)))
                camera_rows, robot_rows, camera_error, robot_error = timestamp_alignment(
                    robot_timestamps, streams, args.max_camera_skew_ms, args.max_robot_skew_ms)
                expected = int(np.count_nonzero(source["episode"] == episode))
                if len(robot_rows) - 1 != expected:
                    raise ValueError(f"{trial}: alignment has {len(robot_rows)-1} rows, source has {expected}")
                readers = [SequentialVideoReader(c, p) for c, p in zip(captures, paths, strict=True)]
                for (filename, key), stream in zip(DEFAULT_FIXED_CAMERAS, streams, strict=True):
                    geometry = _camera_geometry(trial, filename, key, calibration, len(stream))
                    geometries.append(geometry); depth_sources.append(geometry[0])

                group = target.create_group(f"episode_{episode:06d}")
                n = len(source_rows)
                point_ds = group.create_dataset("points", (n, args.max_points, 3), dtype="f2",
                    chunks=(1, args.max_points, 3), compression="lzf")
                color_ds = group.create_dataset("colors", (n, args.max_points, 3), dtype="u1",
                    chunks=(1, args.max_points, 3), compression="lzf")
                valid = np.zeros(n, np.int32); raw_counts = np.zeros(n, np.int32)
                selected_camera_rows = camera_rows[source_rows]
                for dst, (source_row, cam_pair) in enumerate(zip(source_rows, selected_camera_rows, strict=True)):
                    pcs, rgbs = [], []
                    for cam, (reader, geometry) in enumerate(zip(readers, geometries, strict=True)):
                        depth, metadata, rays, camera_from_base = geometry
                        frame = int(cam_pair[cam])
                        bgr = reader.read(frame)
                        xyz, depth_valid = depth_to_base_points(depth[frame], rays,
                            float(metadata["depth_scale_m"]), camera_from_base)
                        pcs.append(xyz)
                        rgbs.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)[depth_valid])
                    xyz, rgb, valid[dst], raw_counts[dst] = deterministic_surface_points(
                        pcs, rgbs, bounds, args.max_points, args.voxel_m)
                    point_ds[dst], color_ds[dst] = xyz.astype(np.float16), rgb
                group.create_dataset("valid_points", data=valid)
                group.create_dataset("raw_valid_points", data=raw_counts)
                group.create_dataset("source_row", data=source_rows)
                group.create_dataset("camera_frame_index", data=selected_camera_rows)
                group.create_dataset("observation_state", data=final["state"][final_indices])
                group.create_dataset("action", data=final["action"][final_indices])
                report = {"episode": episode, "trial": trial.name, "frames": n,
                          "min_points": int(valid.min()), "max_points": int(valid.max()),
                          "max_camera_skew_ms": float(camera_error.max()/1e6),
                          "max_robot_skew_ms": float(robot_error.max()/1e6)}
                reports.append(report)
                print(json.dumps(report), file=sys.stderr, flush=True)
            finally:
                for depth in depth_sources: close_depth_source(depth)
                for capture in captures: capture.release()
        target.attrs["report_json"] = json.dumps(reports)
    partial.replace(output)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--raw-root", type=Path, default=Path("data/threading_new_1"))
    p.add_argument("--source-dataset", type=Path, default=Path("data/threading_new_1_lerobot_v3_cartesian_stride5_sg5_30hz"))
    p.add_argument("--filtered-dataset", type=Path, default=Path("data/threading_new_1_smolvla_6hz_sg5_nozero"))
    p.add_argument("--calibration", type=Path, default=Path("threading_real/calibration/block_grasp_spatial.json"))
    p.add_argument("--output", type=Path, default=Path("data/threading_new_1_mvt_6hz_sg5_nozero.h5"))
    p.add_argument("--bounds", type=float, nargs=6, default=DEFAULT_BOUNDS_M)
    p.add_argument("--max-points", type=int, default=131072)
    p.add_argument("--voxel-m", type=float, default=0.001)
    p.add_argument("--max-camera-skew-ms", type=float, default=25.0)
    p.add_argument("--max-robot-skew-ms", type=float, default=5.0)
    return p


if __name__ == "__main__":
    build(parser().parse_args())
