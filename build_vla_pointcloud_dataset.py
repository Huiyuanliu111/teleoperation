#!/usr/bin/env python3
"""Build synchronized base-frame point clouds from follower RGB-D recordings.

The fixed side and front RealSense streams are deprojected with their recorded
color intrinsics and transformed into the Franka base frame with the live PnP
calibration. The wrist stream remains in the raw recording but is deliberately
not fused until a hand-eye calibration is available.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import cv2
import h5py
import numpy as np

from convert_vla_to_lerobot_v3 import (
    SequentialVideoReader,
    close_depth_source,
    discover_trials,
    follower_column_offset,
    longest_active_span,
    open_depth_memmap,
    read_camera_timestamp_csv,
    read_follower_matrix,
    timestamp_alignment,
)


DEFAULT_FIXED_CAMERAS = (
    ("cam1.mp4", "sideview"),
    ("cam3.mp4", "frontview"),
)
DEFAULT_BOUNDS_M = (0.15, -0.40, -0.15, 0.75, 0.30, 0.50)


def task_span_to_first_close(matrix: np.ndarray, close_width_m: float,
                             post_close_ms: float) -> tuple[int, int]:
    """Keep motion from the first active row through the first gripper close."""
    active = matrix[:, follower_column_offset(matrix)] > 0.5
    active_rows = np.flatnonzero(active)
    if not len(active_rows):
        raise ValueError("robot matrix has no teleoperation-active rows")
    gripper_column = follower_column_offset(matrix) + 8
    closed = np.flatnonzero(
        (np.arange(len(matrix)) >= active_rows[0])
        & (matrix[:, gripper_column] <= close_width_m)
    )
    if not len(closed):
        raise ValueError(f"gripper never reaches close threshold {close_width_m:.3f} m")
    close_row = int(closed[0])
    stop_time = int(matrix[close_row, 0] + post_close_ms * 1e6)
    stop = int(np.searchsorted(matrix[:, 0], stop_time, side="right"))
    return int(active_rows[0]), min(len(matrix), max(close_row + 2, stop))


def parse_camera(value: str) -> tuple[str, str]:
    try:
        filename, calibration_key = value.split("=", 1)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "camera must be RAW_FILENAME=CALIBRATION_KEY"
        ) from exc
    if not filename or not calibration_key:
        raise argparse.ArgumentTypeError("camera filename and key must be non-empty")
    return filename, calibration_key


def camera_rays(metadata: dict[str, Any]) -> np.ndarray:
    """Return one pinhole ray [x/z, y/z, 1] per aligned color pixel."""
    intrinsic = metadata["color_intrinsics"]
    width, height = int(metadata["width"]), int(metadata["height"])
    if (int(intrinsic["width"]), int(intrinsic["height"])) != (width, height):
        raise ValueError("recorded color intrinsics do not match RGB-D dimensions")
    coefficients = np.asarray(intrinsic.get("coeffs", np.zeros(5)), dtype=np.float64)
    if np.any(np.abs(coefficients) > 1e-6):
        raise ValueError(
            "non-zero color distortion is not supported by the numpy deprojector; "
            "rectify the stream before point-cloud conversion"
        )
    fx, fy = float(intrinsic["fx"]), float(intrinsic["fy"])
    ppx, ppy = float(intrinsic["ppx"]), float(intrinsic["ppy"])
    if fx <= 0 or fy <= 0:
        raise ValueError("camera focal lengths must be positive")
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    return np.stack(((u - ppx) / fx, (v - ppy) / fy, np.ones_like(u)), axis=-1).astype(
        np.float32
    )


def depth_to_base_points(
    depth_z16: np.ndarray,
    rays: np.ndarray,
    depth_scale_m: float,
    camera_from_base: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Deproject valid depth and transform camera-frame XYZ into base-frame XYZ."""
    depth = np.asarray(depth_z16)
    if depth.shape != rays.shape[:2]:
        raise ValueError(f"depth/ray shape mismatch: {depth.shape} vs {rays.shape[:2]}")
    scale = float(depth_scale_m)
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("depth scale must be finite and positive")
    transform = np.asarray(camera_from_base, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("camera_from_base must be a finite 4x4 matrix")
    valid = depth > 0
    camera_points = rays[valid].astype(np.float64) * (depth[valid, None] * scale)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    # camera_from_base means p_camera = R * p_base + t.
    base_points = (camera_points - translation) @ rotation
    return base_points.astype(np.float32), valid


def crop_and_sample(
    points: list[np.ndarray],
    colors: list[np.ndarray],
    camera_ids: list[np.ndarray],
    bounds_m: np.ndarray,
    num_points: int,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    xyz = np.concatenate(points, axis=0)
    rgb = np.concatenate(colors, axis=0)
    source = np.concatenate(camera_ids, axis=0)
    lower, upper = bounds_m[:3], bounds_m[3:]
    keep = np.isfinite(xyz).all(axis=1) & (xyz > lower).all(axis=1) & (xyz < upper).all(axis=1)
    xyz, rgb, source = xyz[keep], rgb[keep], source[keep]
    valid_count = len(xyz)
    if valid_count == 0:
        raise ValueError("workspace crop removed every point")
    replace = valid_count < num_points
    chosen = rng.choice(valid_count, size=num_points, replace=replace)
    return xyz[chosen], rgb[chosen], source[chosen], valid_count


def _video_count(capture: cv2.VideoCapture, path: Path) -> int:
    count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if count < 2:
        raise ValueError(f"{path}: video reports fewer than two frames")
    return count


def _camera_geometry(
    trial: Path,
    filename: str,
    calibration_key: str,
    calibration: dict[str, Any],
    committed_frames: int,
) -> tuple[np.memmap, dict[str, Any], np.ndarray, np.ndarray]:
    depth, metadata = open_depth_memmap(trial, filename, committed_frames)
    try:
        calibrated = calibration["cameras"][calibration_key]
    except KeyError as exc:
        raise KeyError(f"calibration has no camera {calibration_key!r}") from exc
    recorded_serial = str(metadata["serial"])
    calibrated_serial = str(calibrated.get("serial", ""))
    if calibrated_serial and recorded_serial != calibrated_serial:
        raise ValueError(
            f"{trial / filename}: recorded serial {recorded_serial} does not match "
            f"calibrated {calibration_key} serial {calibrated_serial}"
        )
    return (
        depth,
        metadata,
        camera_rays(metadata),
        np.asarray(calibrated["camera_from_base"], dtype=np.float64),
    )


def build(args: argparse.Namespace) -> dict[str, Any]:
    raw_root = args.raw_root.expanduser().resolve()
    output = args.output.expanduser().resolve()
    partial = output.with_name(output.name + ".partial")
    if output.exists() or partial.exists():
        raise FileExistsError(f"refusing to overwrite existing output/partial file: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)

    calibration_path = args.calibration.expanduser().resolve()
    calibration = json.loads(calibration_path.read_text())
    if calibration.get("format") != "threading-spatial-projection-v1":
        raise ValueError(f"unsupported calibration format in {calibration_path}")
    camera_specs = tuple(args.camera or DEFAULT_FIXED_CAMERAS)
    if len(camera_specs) < 2:
        raise ValueError("point-cloud fusion requires at least two fixed cameras")
    if len({item[1] for item in camera_specs}) != len(camera_specs):
        raise ValueError("calibration camera keys must be unique")
    bounds = np.asarray(args.bounds, dtype=np.float32)
    if bounds.shape != (6,) or np.any(bounds[:3] >= bounds[3:]):
        raise ValueError("bounds must be xmin ymin zmin xmax ymax zmax")

    trials = discover_trials(raw_root)
    reports: list[dict[str, Any]] = []
    with h5py.File(partial, "w") as output_file:
        output_file.attrs["format"] = "threading-base-pointcloud-v1"
        output_file.attrs["raw_root"] = str(raw_root)
        output_file.attrs["calibration_path"] = str(calibration_path)
        output_file.attrs["camera_mapping_json"] = json.dumps(camera_specs)
        output_file.attrs["bounds_m"] = bounds
        output_file.attrs["num_points"] = args.num_points
        output_file.attrs["temporal_stride"] = args.stride

        for episode_index, trial in enumerate(trials):
            matrix = read_follower_matrix(trial / "DATA_follower.m")
            if follower_column_offset(matrix) != 1:
                raise ValueError(f"{trial}: point clouds require a timestamped v2 recording")
            if args.keep_inactive or args.span_mode == "all":
                active_start, active_stop = 0, len(matrix)
            elif args.span_mode == "to_first_close":
                active_start, active_stop = task_span_to_first_close(
                    matrix, args.close_width_m, args.post_close_ms
                )
            else:
                active_start, active_stop = longest_active_span(matrix)
            selected_matrix = matrix[active_start:active_stop]
            robot_timestamps = selected_matrix[:, 0].astype(np.int64)
            states = np.concatenate(
                (selected_matrix[:, 2:9], selected_matrix[:, 9:10]), axis=1
            ).astype(np.float32)

            captures: list[cv2.VideoCapture] = []
            depth_sources = []
            readers = []
            try:
                counts = []
                timestamp_streams = []
                paths = []
                for filename, _ in camera_specs:
                    path = trial / filename
                    capture = cv2.VideoCapture(str(path))
                    if not capture.isOpened():
                        raise ValueError(f"cannot open camera video: {path}")
                    captures.append(capture)
                    paths.append(path)
                    count = _video_count(capture, path)
                    counts.append(count)
                    timestamp_streams.append(
                        read_camera_timestamp_csv(
                            trial / f"{Path(filename).stem}_timestamps.csv", count
                        )
                    )
                camera_rows, robot_rows, camera_errors, robot_errors = timestamp_alignment(
                    robot_timestamps,
                    timestamp_streams,
                    args.max_camera_skew_ms,
                    args.max_robot_skew_ms,
                )
                sample_rows = np.arange(0, len(robot_rows), args.stride, dtype=np.int64)
                if len(sample_rows) < 2:
                    raise ValueError(f"{trial}: fewer than two synchronized stride samples")
                observation_rows, action_rows = sample_rows[:-1], sample_rows[1:]
                readers = [
                    SequentialVideoReader(capture, path)
                    for capture, path in zip(captures, paths, strict=True)
                ]
                depth_sources = [
                    _camera_geometry(
                        trial, filename, key, calibration, len(timestamps)
                    )
                    for (filename, key), timestamps in zip(
                        camera_specs, timestamp_streams, strict=True
                    )
                ]

                group = output_file.create_group(f"episode_{episode_index:06d}")
                group.attrs["source_trial"] = str(trial.relative_to(raw_root))
                frame_count = len(observation_rows)
                point_dataset = group.create_dataset(
                    "points",
                    shape=(frame_count, args.num_points, 3),
                    dtype="f4",
                    chunks=(1, args.num_points, 3),
                    compression="lzf",
                )
                color_dataset = group.create_dataset(
                    "colors",
                    shape=(frame_count, args.num_points, 3),
                    dtype="u1",
                    chunks=(1, args.num_points, 3),
                    compression="lzf",
                )
                source_dataset = group.create_dataset(
                    "camera_id",
                    shape=(frame_count, args.num_points),
                    dtype="u1",
                    chunks=(1, args.num_points),
                    compression="lzf",
                )
                valid_counts = np.empty(frame_count, dtype=np.int64)

                for destination_row, aligned_row in enumerate(observation_rows):
                    points_per_camera = []
                    colors_per_camera = []
                    ids_per_camera = []
                    for camera_index, (reader, geometry) in enumerate(
                        zip(readers, depth_sources, strict=True)
                    ):
                        depth, metadata, rays, camera_from_base = geometry
                        source_row = int(camera_rows[aligned_row, camera_index])
                        bgr = reader.read(source_row)
                        base_points, valid_depth = depth_to_base_points(
                            depth[source_row],
                            rays,
                            float(metadata["depth_scale_m"]),
                            camera_from_base,
                        )
                        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)[valid_depth]
                        points_per_camera.append(base_points)
                        colors_per_camera.append(rgb)
                        ids_per_camera.append(
                            np.full(len(base_points), camera_index, dtype=np.uint8)
                        )
                    rng = np.random.default_rng(
                        args.seed + episode_index * 1_000_003 + int(aligned_row)
                    )
                    xyz, rgb, source, valid_count = crop_and_sample(
                        points_per_camera,
                        colors_per_camera,
                        ids_per_camera,
                        bounds,
                        args.num_points,
                        rng,
                    )
                    point_dataset[destination_row] = xyz
                    color_dataset[destination_row] = rgb
                    source_dataset[destination_row] = source
                    valid_counts[destination_row] = valid_count

                absolute_robot_rows = active_start + robot_rows
                group.create_dataset(
                    "observation_state", data=states[robot_rows[observation_rows]], dtype="f4"
                )
                group.create_dataset(
                    "action_state", data=states[robot_rows[action_rows]], dtype="f4"
                )
                group.create_dataset(
                    "robot_row_index",
                    data=absolute_robot_rows[observation_rows],
                    dtype="i8",
                )
                group.create_dataset(
                    "action_robot_row_index",
                    data=absolute_robot_rows[action_rows],
                    dtype="i8",
                )
                group.create_dataset(
                    "camera_frame_index", data=camera_rows[observation_rows], dtype="i8"
                )
                group.create_dataset(
                    "robot_host_steady_timestamp_ns",
                    data=robot_timestamps[robot_rows[observation_rows]],
                    dtype="i8",
                )
                group.create_dataset("valid_points_before_sampling", data=valid_counts)
                report = {
                    "episode": group.name.lstrip("/"),
                    "source_trial": str(trial.relative_to(raw_root)),
                    "frames": frame_count,
                    "min_valid_points": int(valid_counts.min()),
                    "max_camera_skew_ms": float(camera_errors.max() / 1e6),
                    "max_robot_skew_ms": float(robot_errors.max() / 1e6),
                }
                reports.append(report)
                print(
                    f"[pointcloud] {report['source_trial']}: frames={frame_count} "
                    f"min_points={report['min_valid_points']}",
                    file=sys.stderr,
                )
            finally:
                for depth_source, _, _, _ in depth_sources:
                    close_depth_source(depth_source)
                for capture in captures:
                    capture.release()

        output_file.attrs["episodes_json"] = json.dumps(reports)
        output_file.flush()
    partial.replace(output)
    return {
        "format": "threading-base-pointcloud-v1",
        "output": str(output),
        "episodes": reports,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_root", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--num-points", type=int, default=8192)
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-camera-skew-ms", type=float, default=25.0)
    parser.add_argument("--max-robot-skew-ms", type=float, default=5.0)
    parser.add_argument(
        "--bounds",
        type=float,
        nargs=6,
        default=DEFAULT_BOUNDS_M,
        metavar=("XMIN", "YMIN", "ZMIN", "XMAX", "YMAX", "ZMAX"),
    )
    parser.add_argument(
        "--camera",
        action="append",
        type=parse_camera,
        metavar="RAW_FILE=CALIBRATION_KEY",
        help="repeat for fixed cameras; defaults to cam1=sideview and cam3=frontview",
    )
    parser.add_argument(
        "--keep-inactive",
        action="store_true",
        help="keep setup rows outside the longest teleoperation-active interval",
    )
    parser.add_argument(
        "--span-mode", choices=("longest_active", "to_first_close", "all"),
        default="longest_active",
        help="to_first_close preserves multi-segment approach and the gripper-close label",
    )
    parser.add_argument("--close-width-m", type=float, default=0.03)
    parser.add_argument("--post-close-ms", type=float, default=500.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.num_points <= 0 or args.stride <= 0:
        raise ValueError("num-points and stride must be positive")
    if args.max_camera_skew_ms <= 0 or args.max_robot_skew_ms <= 0:
        raise ValueError("timestamp skew limits must be positive")
    if not 0 < args.close_width_m < 0.08:
        raise ValueError("close-width-m must lie in (0, 0.08)")
    if args.post_close_ms < 0:
        raise ValueError("post-close-ms must be non-negative")
    result = build(args)
    print(json.dumps({"output": result["output"], "episodes": len(result["episodes"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
