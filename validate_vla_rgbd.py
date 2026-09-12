#!/usr/bin/env python3
"""Validate timestamp, RGB, and aligned-depth integrity before conversion."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

from convert_vla_to_lerobot_v3 import (
    TIMESTAMPED_ROBOT_COLUMNS,
    close_depth_source,
    discover_trials,
    follower_column_offset,
    open_depth_memmap,
    read_camera_timestamp_csv,
    read_follower_matrix,
    timestamp_alignment,
)


LEGACY_CAMERAS = ("cam1", "cam2", "cam3")


def video_frame_count(path: Path) -> int:
    capture = cv2.VideoCapture(str(path))
    try:
        if not capture.isOpened():
            raise ValueError(f"cannot open video: {path}")
        count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        if count < 2:
            raise ValueError(f"video has fewer than two frames: {path}")
        return count
    finally:
        capture.release()


def validate_episode(
    trial: Path, raw_root: Path, max_camera_skew_ms: float, max_robot_skew_ms: float
) -> dict:
    manifest_path = trial / "recording_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"missing v2 recording manifest: {manifest_path}")
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("format") not in {
        "threading-rgbd-recording-v2",
        "maze-rgbd-recording-v3",
    }:
        raise ValueError(f"{trial}: unsupported recording manifest format")
    if manifest.get("role") not in (None, "follower"):
        raise ValueError(f"{trial}: recording manifest is not from the follower")
    camera_names = tuple(manifest.get("recorded_cameras", LEGACY_CAMERAS))
    if not camera_names or any(name not in LEGACY_CAMERAS for name in camera_names):
        raise ValueError(f"{trial}: invalid recorded_cameras {camera_names}")
    cameras = tuple(f"{name}.mp4" for name in camera_names)
    matrix = read_follower_matrix(trial / "DATA_follower.m")
    if matrix.shape[1] != TIMESTAMPED_ROBOT_COLUMNS or follower_column_offset(matrix) != 1:
        raise ValueError(f"{trial}: expected timestamped 30-column follower data")
    robot_timestamps = matrix[:, 0].astype(np.int64)
    if np.any(np.diff(robot_timestamps) <= 0):
        raise ValueError(f"{trial}: robot host timestamps are not strictly increasing")

    counts = [video_frame_count(trial / filename) for filename in cameras]
    camera_timestamps = []
    depth_frames = []
    serials = []
    for filename, count in zip(cameras, counts, strict=True):
        stem = Path(filename).stem
        timestamps = read_camera_timestamp_csv(
            trial / f"{stem}_timestamps.csv", count
        )
        depth, metadata = open_depth_memmap(trial, filename, len(timestamps))
        camera_timestamps.append(timestamps)
        depth_frames.append(int(depth.shape[0]))
        serials.append(str(metadata["serial"]))
        # Force boundary-frame decompression for indexed Zstd sources; raw
        # memmaps make these reads effectively free.
        _ = depth[0]
        _ = depth[-1]
        close_depth_source(depth)

    manifest_serials = manifest.get("camera_serials", {})
    for camera_name, serial in zip(camera_names, serials, strict=True):
        expected = manifest_serials.get(camera_name)
        if expected is not None and str(expected) != serial:
            raise ValueError(
                f"{trial}: {camera_name} metadata serial {serial} does not match "
                f"manifest serial {expected}"
            )
    manifest_counts = manifest.get("committed_camera_frames", {})
    if manifest.get("complete", False):
        for camera_name, video_count, timestamp_count, depth_count in zip(
            camera_names, counts, map(len, camera_timestamps), depth_frames, strict=True
        ):
            expected = manifest_counts.get(camera_name)
            if expected is not None and int(expected) != timestamp_count:
                raise ValueError(
                    f"{trial}: {camera_name} manifest commits {expected} frames but "
                    f"timestamp CSV commits {timestamp_count}"
                )
            if video_count != timestamp_count or depth_count != timestamp_count:
                raise ValueError(
                    f"{trial}: complete {camera_name} stream has video/timestamp/depth "
                    f"counts {video_count}/{timestamp_count}/{depth_count}"
                )

    camera_rows, robot_rows, camera_errors, robot_errors = timestamp_alignment(
        robot_timestamps,
        camera_timestamps,
        max_camera_skew_ms,
        max_robot_skew_ms,
    )
    reference_count = len(camera_timestamps[0])
    if len(robot_rows) < 3:
        raise ValueError(f"{trial}: fewer than three timestamp-aligned samples")
    return {
        "trial": str(trial.relative_to(raw_root)),
        "manifest_complete": bool(manifest.get("complete", False)),
        "robot_rows": int(len(matrix)),
        "video_frames": counts,
        "committed_camera_frames": [int(len(item)) for item in camera_timestamps],
        "complete_depth_frames": depth_frames,
        "camera_serials": serials,
        "camera_names": camera_names,
        "aligned_frames": int(len(robot_rows)),
        "reference_frames": reference_count,
        "retained_fraction": float(len(robot_rows) / reference_count),
        "max_camera_skew_ms": float(camera_errors.max() / 1_000_000.0),
        "p99_camera_skew_ms": float(
            np.percentile(camera_errors.max(axis=1), 99) / 1_000_000.0
        ),
        "max_robot_skew_ms": float(robot_errors.max() / 1_000_000.0),
        "p99_robot_skew_ms": float(np.percentile(robot_errors, 99) / 1_000_000.0),
        "first_robot_row": int(robot_rows[0]),
        "last_robot_row": int(robot_rows[-1]),
        "first_camera_rows": camera_rows[0].tolist(),
        "last_camera_rows": camera_rows[-1].tolist(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_root", type=Path)
    parser.add_argument("--max-camera-skew-ms", type=float, default=25.0)
    parser.add_argument("--max-robot-skew-ms", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    raw_root = args.raw_root.expanduser().resolve()
    reports = []
    failures = []
    for trial in discover_trials(raw_root):
        try:
            report = validate_episode(
                trial, raw_root, args.max_camera_skew_ms, args.max_robot_skew_ms
            )
            reports.append(report)
            print(
                f"[ok] {report['trial']}: aligned={report['aligned_frames']}/"
                f"{report['reference_frames']} camera_p99="
                f"{report['p99_camera_skew_ms']:.2f}ms robot_p99="
                f"{report['p99_robot_skew_ms']:.2f}ms"
            )
        except Exception as error:
            failures.append({"trial": str(trial.relative_to(raw_root)), "error": str(error)})
            print(f"[failed] {trial.relative_to(raw_root)}: {error}", file=sys.stderr)
    payload = {
        "format": "threading-rgbd-validation-v1",
        "raw_root": str(raw_root),
        "episodes": reports,
        "failures": failures,
    }
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, indent=2) + "\n")
    print(json.dumps({"valid": len(reports), "failed": len(failures)}, indent=2))
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
