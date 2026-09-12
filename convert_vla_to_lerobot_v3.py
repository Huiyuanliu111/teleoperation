#!/usr/bin/env python3
"""Convert vla_finetune raw RGB-D trials into a LeRobot v3 dataset.

New recordings persist a common host monotonic timestamp for robot and camera
samples. They are joined by nearest timestamp and aligned Z16 depth is copied
to an HDF5 sidecar. Legacy RGB-only recordings remain supported through the
old normalized-episode-progress fallback, which is marked as approximate.

Example:
    python convert_vla_to_lerobot_v3.py \
        vla_finetune/data/Threading_20260826_120000 \
        data/threading_vla_lerobot_v3

Default camera mapping is ``cam1.mp4`` = side view and ``cam3.mp4`` = front
view. The optional ``cam2.mp4`` wrist view can be supplied with ``--camera``.
Use repeated ``--camera RAW_FILE=FEATURE`` arguments if a deployment uses the
opposite numbering.
"""
from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import json
from pathlib import Path
import shutil
import sys
from typing import Any

import numpy as np


LEGACY_ROBOT_COLUMNS = 29
TIMESTAMPED_ROBOT_COLUMNS = 30
STATE_NAMES = [f"q{i}" for i in range(1, 8)] + ["gripper_width"]
DEFAULT_CAMERAS = (
    ("cam1.mp4", "observation.images.exterior_image_2_right"),
    ("cam3.mp4", "observation.images.exterior_image_1_left"),
)


def read_follower_matrix(path: Path) -> np.ndarray:
    """Read the Eigen matrix written by Recorder.cpp as a MATLAB assignment."""
    text = path.read_text(errors="strict")
    start = text.find("[")
    stop = text.rfind("]")
    if start < 0 or stop <= start:
        raise ValueError(f"{path}: expected a MATLAB matrix assignment")
    rows = [
        np.fromstring(line.replace(";", " "), sep=" ")
        for line in text[start + 1 : stop].splitlines()
        if line.strip()
    ]
    widths = {len(row) for row in rows}
    if not rows or len(widths) != 1 or next(iter(widths)) not in {
        LEGACY_ROBOT_COLUMNS,
        TIMESTAMPED_ROBOT_COLUMNS,
    }:
        raise ValueError(
            f"{path}: expected rows with {LEGACY_ROBOT_COLUMNS} legacy or "
            f"{TIMESTAMPED_ROBOT_COLUMNS} timestamped columns, got {sorted(widths)}"
        )
    matrix = np.stack(rows)
    if not np.isfinite(matrix).all():
        raise ValueError(f"{path}: robot matrix contains NaN or Inf")
    return matrix


def follower_column_offset(matrix: np.ndarray) -> int:
    """Return one for timestamped v2 rows and zero for legacy rows."""
    if matrix.ndim != 2 or matrix.shape[1] not in {
        LEGACY_ROBOT_COLUMNS,
        TIMESTAMPED_ROBOT_COLUMNS,
    }:
        raise ValueError(f"unsupported follower matrix shape {matrix.shape}")
    return int(matrix.shape[1] == TIMESTAMPED_ROBOT_COLUMNS)


def longest_active_span(matrix: np.ndarray) -> tuple[int, int]:
    """Return [start, stop) for the longest contiguous teleoperation-active run."""
    active = matrix[:, follower_column_offset(matrix)] > 0.5
    padded = np.pad(active.astype(np.int8), (1, 1))
    edges = np.flatnonzero(np.diff(padded))
    if len(edges) == 0:
        raise ValueError("robot matrix has no teleoperation-active rows")
    starts, stops = edges[::2], edges[1::2]
    index = int(np.argmax(stops - starts))
    return int(starts[index]), int(stops[index])


def resample_indices(num_robot_rows: int, num_video_frames: int) -> np.ndarray:
    """Map video frame times to robot rows using normalized episode progress."""
    if num_robot_rows < 2:
        raise ValueError("at least two robot rows are required")
    if num_video_frames < 2:
        raise ValueError("at least two video frames are required")
    return np.rint(np.linspace(0, num_robot_rows - 1, num_video_frames)).astype(np.int64)


def nearest_timestamp_indices(
    timestamps_ns: np.ndarray, query_ns: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Return nearest source rows and absolute timestamp errors."""
    source = np.asarray(timestamps_ns, dtype=np.int64)
    query = np.asarray(query_ns, dtype=np.int64)
    if source.ndim != 1 or query.ndim != 1 or len(source) == 0:
        raise ValueError("timestamp arrays must be non-empty one-dimensional arrays")
    if np.any(np.diff(source) <= 0):
        raise ValueError("source timestamps must be strictly increasing")
    right = np.clip(np.searchsorted(source, query, side="left"), 0, len(source) - 1)
    left = np.clip(right - 1, 0, len(source) - 1)
    choose_left = np.abs(query - source[left]) <= np.abs(source[right] - query)
    indices = np.where(choose_left, left, right)
    return indices, np.abs(source[indices] - query)


def timestamp_alignment(
    robot_timestamps_ns: np.ndarray,
    camera_timestamps_ns: list[np.ndarray],
    max_camera_skew_ms: float,
    max_robot_skew_ms: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Align all streams to camera zero and reject samples outside skew limits."""
    if not camera_timestamps_ns:
        raise ValueError("at least one camera timestamp stream is required")
    reference = np.asarray(camera_timestamps_ns[0], dtype=np.int64)
    camera_matches = [
        nearest_timestamp_indices(timestamps, reference)
        for timestamps in camera_timestamps_ns
    ]
    camera_rows = np.stack([match[0] for match in camera_matches], axis=1)
    camera_errors = np.stack([match[1] for match in camera_matches], axis=1)
    robot_rows, robot_errors = nearest_timestamp_indices(robot_timestamps_ns, reference)
    valid = (
        camera_errors.max(axis=1) <= max_camera_skew_ms * 1_000_000.0
    ) & (robot_errors <= max_robot_skew_ms * 1_000_000.0)
    return camera_rows[valid], robot_rows[valid], camera_errors[valid], robot_errors[valid]


def read_camera_timestamp_csv(path: Path, video_frames: int) -> np.ndarray:
    table = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    table = np.atleast_1d(table)
    required = {"frame_index", "host_steady_timestamp_ns"}
    if table.dtype.names is None or not required.issubset(table.dtype.names):
        raise ValueError(f"{path}: missing timestamp columns {sorted(required)}")
    frame_indices = np.asarray(table["frame_index"], dtype=np.int64)
    timestamps = np.asarray(table["host_steady_timestamp_ns"], dtype=np.int64)
    usable = min(len(timestamps), int(video_frames))
    if usable < 2 or not np.array_equal(frame_indices[:usable], np.arange(usable)):
        raise ValueError(f"{path}: committed frame indices are not contiguous from zero")
    timestamps = timestamps[:usable]
    if np.any(np.diff(timestamps) <= 0):
        raise ValueError(f"{path}: host timestamps are not strictly increasing")
    return timestamps


def parse_camera(value: str) -> tuple[str, str]:
    try:
        filename, feature = value.split("=", 1)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("camera must be RAW_FILENAME=LEROBOT_FEATURE") from exc
    if not filename or not feature.startswith("observation.images."):
        raise argparse.ArgumentTypeError(
            "camera feature must look like observation.images.camera_name"
        )
    return filename, feature


def discover_trials(raw_root: Path) -> list[Path]:
    if not raw_root.is_dir():
        raise FileNotFoundError(f"raw recording directory not found: {raw_root}")
    trials = sorted({path.parent for path in raw_root.rglob("DATA_follower.m")})
    if not trials:
        raise FileNotFoundError(f"no DATA_follower.m files found under {raw_root}")
    return trials


def _video_frame_count(capture: Any, path: Path, cv2: Any) -> int:
    count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if count < 2:
        raise ValueError(f"{path}: video reports fewer than two frames")
    return count


class SequentialVideoReader:
    """Read monotonically requested frames without loading a full video."""

    def __init__(self, capture: Any, path: Path) -> None:
        self.capture = capture
        self.path = path
        self.index = -1
        self.frame: np.ndarray | None = None

    def read(self, target: int) -> np.ndarray:
        if target < self.index:
            raise ValueError(f"{self.path}: requested non-monotonic frame {target}")
        while self.index < target:
            ok, frame = self.capture.read()
            if not ok:
                raise ValueError(f"{self.path}: could not decode frame {self.index + 1}")
            self.index += 1
            self.frame = frame
        assert self.frame is not None
        return self.frame


class ZstdDepthFrames:
    """Random-access reader for independently compressed Z16 frames."""

    def __init__(
        self,
        path: Path,
        offsets: np.ndarray,
        compressed_sizes: np.ndarray,
        height: int,
        width: int,
    ) -> None:
        library_path = ctypes.util.find_library("zstd")
        if library_path is None:
            raise RuntimeError("libzstd is required to read compressed depth frames")
        self.library = ctypes.CDLL(library_path)
        self.library.ZSTD_decompress.argtypes = [
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_void_p,
            ctypes.c_size_t,
        ]
        self.library.ZSTD_decompress.restype = ctypes.c_size_t
        self.library.ZSTD_isError.argtypes = [ctypes.c_size_t]
        self.library.ZSTD_isError.restype = ctypes.c_uint
        self.library.ZSTD_getErrorName.argtypes = [ctypes.c_size_t]
        self.library.ZSTD_getErrorName.restype = ctypes.c_char_p
        self.path = path
        self.offsets = np.asarray(offsets, dtype=np.int64)
        self.compressed_sizes = np.asarray(compressed_sizes, dtype=np.int64)
        self.shape = (len(self.offsets), int(height), int(width))
        self.raw_bytes = int(height) * int(width) * 2
        file_size = path.stat().st_size
        if (
            len(self.offsets) == 0
            or len(self.offsets) != len(self.compressed_sizes)
            or np.any(self.offsets < 0)
            or np.any(self.compressed_sizes <= 0)
            or np.any(self.offsets + self.compressed_sizes > file_size)
        ):
            raise ValueError(f"{path}: invalid compressed depth frame index")
        self.stream = path.open("rb")

    def __getitem__(self, index: int) -> np.ndarray:
        index = int(index)
        if index < 0:
            index += self.shape[0]
        if index < 0 or index >= self.shape[0]:
            raise IndexError(index)
        self.stream.seek(int(self.offsets[index]))
        compressed = self.stream.read(int(self.compressed_sizes[index]))
        if len(compressed) != int(self.compressed_sizes[index]):
            raise ValueError(f"{self.path}: truncated compressed depth frame {index}")
        source = ctypes.create_string_buffer(compressed)
        output = np.empty((self.shape[1], self.shape[2]), dtype="<u2")
        result = self.library.ZSTD_decompress(
            output.ctypes.data_as(ctypes.c_void_p),
            self.raw_bytes,
            ctypes.cast(source, ctypes.c_void_p),
            len(compressed),
        )
        if self.library.ZSTD_isError(result):
            message = self.library.ZSTD_getErrorName(result).decode("utf-8")
            raise ValueError(f"{self.path}: frame {index} zstd error: {message}")
        if int(result) != self.raw_bytes:
            raise ValueError(
                f"{self.path}: frame {index} expands to {result}, expected {self.raw_bytes} bytes"
            )
        return output

    def close(self) -> None:
        self.stream.close()


def close_depth_source(source: Any) -> None:
    close = getattr(source, "close", None)
    if callable(close):
        close()
        return
    current = source
    seen: set[int] = set()
    while current is not None and id(current) not in seen:
        seen.add(id(current))
        mmap = getattr(current, "_mmap", None)
        if mmap is not None:
            mmap.close()
            return
        current = getattr(current, "base", None)


def open_depth_memmap(trial: Path, video_filename: str, committed_frames: int):
    """Open legacy raw or v2 per-frame-Zstd depth with one indexing interface."""
    stem = Path(video_filename).stem
    metadata_path = trial / f"{stem}_metadata.json"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"missing RGB-D metadata/depth for {trial / stem}")
    metadata = json.loads(metadata_path.read_text())
    width = int(metadata["width"])
    height = int(metadata["height"])
    compressed_path = trial / f"{stem}_depth.z16.zst"
    raw_path = trial / f"{stem}_depth.z16"
    if compressed_path.is_file():
        timestamp_path = trial / f"{stem}_timestamps.csv"
        table = np.atleast_1d(
            np.genfromtxt(timestamp_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        )
        required = {"depth_offset_bytes", "depth_compressed_bytes"}
        if table.dtype.names is None or not required.issubset(table.dtype.names):
            raise ValueError(f"{timestamp_path}: missing compressed depth index columns")
        usable = min(len(table), int(committed_frames))
        source = ZstdDepthFrames(
            compressed_path,
            np.asarray(table["depth_offset_bytes"][:usable], dtype=np.int64),
            np.asarray(table["depth_compressed_bytes"][:usable], dtype=np.int64),
            height,
            width,
        )
        return source, metadata
    depth_path = raw_path
    if not depth_path.is_file():
        raise FileNotFoundError(f"missing RGB-D depth for {trial / stem}")
    pixels_per_frame = width * height
    stored_frames, remainder = divmod(depth_path.stat().st_size, pixels_per_frame * 2)
    if remainder or stored_frames < committed_frames:
        raise ValueError(
            f"{depth_path}: {stored_frames} complete depth frames for "
            f"{committed_frames} committed RGB frames"
        )
    values = np.memmap(depth_path, mode="r", dtype="<u2")
    return values.reshape(stored_frames, height, width), metadata


def convert(args: argparse.Namespace) -> dict[str, Any]:
    try:
        import cv2
    except ImportError as exc:
        raise ImportError("opencv-python is required to decode the raw MP4 files") from exc
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise ImportError("lerobot>=0.4.0 is required to write Dataset v3.0") from exc

    raw_root = args.raw_root.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"output already exists; refusing to overwrite: {output}")

    camera_specs = tuple(args.camera or DEFAULT_CAMERAS)
    if len({feature for _, feature in camera_specs}) != len(camera_specs):
        raise ValueError("camera feature names must be unique")
    trials = discover_trials(raw_root)

    features: dict[str, dict[str, Any]] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (8,),
            "names": STATE_NAMES,
        },
        "action": {
            "dtype": "float32",
            "shape": (8,),
            "names": STATE_NAMES,
        },
    }
    for _, feature in camera_specs:
        features[feature] = {
            "dtype": "video",
            "shape": (args.image_size, args.image_size, 3),
            "names": ["height", "width", "channels"],
        }

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features=features,
        root=output,
        robot_type="franka",
        use_videos=True,
    )
    reports: list[dict[str, Any]] = []
    try:
        for output_episode_index, trial in enumerate(trials):
            matrix = read_follower_matrix(trial / "DATA_follower.m")
            column_offset = follower_column_offset(matrix)
            total_robot_rows = len(matrix)
            teleop_active_rows = int(
                np.count_nonzero(matrix[:, column_offset] > 0.5)
            )
            if args.active_only:
                active_start, active_stop = longest_active_span(matrix)
            else:
                active_start, active_stop = 0, len(matrix)
            matrix = matrix[active_start:active_stop]
            states = np.concatenate(
                (
                    matrix[:, column_offset + 1 : column_offset + 8],
                    matrix[:, column_offset + 8 : column_offset + 9],
                ),
                axis=1,
            ).astype(np.float32)
            robot_timestamps_ns = (
                matrix[:, 0].astype(np.int64) if column_offset else None
            )

            captures: list[Any] = []
            video_paths: list[Path] = []
            depth_sources: list[tuple[Any, dict[str, Any]]] = []
            depth_file = None
            try:
                counts = []
                for filename, _ in camera_specs:
                    path = trial / filename
                    capture = cv2.VideoCapture(str(path))
                    if not capture.isOpened():
                        raise ValueError(f"cannot open camera video: {path}")
                    captures.append(capture)
                    video_paths.append(path)
                    counts.append(_video_frame_count(capture, path, cv2))

                timestamp_paths = [
                    trial / f"{Path(filename).stem}_timestamps.csv"
                    for filename, _ in camera_specs
                ]
                has_all_camera_timestamps = all(path.is_file() for path in timestamp_paths)
                if column_offset and not has_all_camera_timestamps:
                    missing = [str(path) for path in timestamp_paths if not path.is_file()]
                    raise FileNotFoundError(
                        "timestamped robot recording is missing camera timestamps: "
                        + ", ".join(missing)
                    )

                if column_offset:
                    assert robot_timestamps_ns is not None
                    camera_timestamps = [
                        read_camera_timestamp_csv(path, count)
                        for path, count in zip(timestamp_paths, counts, strict=True)
                    ]
                    camera_rows, row_indices, camera_errors, robot_errors = timestamp_alignment(
                        robot_timestamps_ns,
                        camera_timestamps,
                        args.max_camera_skew_ms,
                        args.max_robot_skew_ms,
                    )
                    alignment_method = "host_steady_timestamp_nearest"
                    if len(row_indices) < 3:
                        raise ValueError(
                            f"{trial}: fewer than three samples pass timestamp skew limits"
                        )
                    max_camera_skew_ms = float(camera_errors.max() / 1_000_000.0)
                    max_robot_skew_ms = float(robot_errors.max() / 1_000_000.0)
                else:
                    target_frames = min(counts)
                    camera_rows = np.repeat(
                        np.arange(target_frames, dtype=np.int64)[:, None],
                        len(camera_specs),
                        axis=1,
                    )
                    row_indices = resample_indices(len(states), target_frames)
                    alignment_method = "normalized_episode_progress"
                    max_camera_skew_ms = None
                    max_robot_skew_ms = None

                readers = [
                    SequentialVideoReader(capture, path)
                    for capture, path in zip(captures, video_paths, strict=True)
                ]
                depth_datasets = []
                depth_sidecar = None
                if column_offset and not args.skip_depth:
                    try:
                        import h5py
                    except ImportError as exc:
                        raise ImportError(
                            "h5py is required to preserve aligned Z16 depth; "
                            "install h5py or explicitly pass --skip-depth"
                        ) from exc
                    depth_sources = [
                        open_depth_memmap(trial, filename, len(timestamps))
                        for (filename, _), timestamps in zip(
                            camera_specs, camera_timestamps, strict=True
                        )
                    ]
                    rgbd_dir = output / "rgbd" / f"episode_{output_episode_index:06d}"
                    rgbd_dir.mkdir(parents=True, exist_ok=True)
                    depth_path = rgbd_dir / "depth.h5"
                    depth_file = h5py.File(depth_path, "w")
                    depth_file.attrs["format"] = "threading-aligned-depth-v1"
                    depth_file.attrs["source_trial"] = str(trial.relative_to(raw_root))
                    depth_file.attrs["rgb_files_json"] = json.dumps(
                        [filename for filename, _ in camera_specs]
                    )
                    depth_file.create_dataset(
                        "robot_row_index", data=row_indices[:-1], dtype="i8"
                    )
                    depth_file.create_dataset(
                        "camera_frame_index", data=camera_rows[:-1], dtype="i8"
                    )
                    depth_file.create_dataset(
                        "robot_host_steady_timestamp_ns",
                        data=robot_timestamps_ns[row_indices[:-1]],
                        dtype="i8",
                    )
                    depth_file.create_dataset(
                        "camera_host_steady_timestamp_ns",
                        data=np.stack(
                            [
                                timestamps[camera_rows[:-1, camera_index]]
                                for camera_index, timestamps in enumerate(camera_timestamps)
                            ],
                            axis=1,
                        ),
                        dtype="i8",
                    )
                    for (_, feature), (source, metadata) in zip(
                        camera_specs, depth_sources, strict=True
                    ):
                        dataset = depth_file.create_dataset(
                            feature,
                            shape=(len(row_indices) - 1, source.shape[1], source.shape[2]),
                            dtype="<u2",
                            chunks=(1, source.shape[1], source.shape[2]),
                            compression="lzf",
                        )
                        dataset.attrs["depth_scale_m"] = float(metadata["depth_scale_m"])
                        dataset.attrs["camera_metadata_json"] = json.dumps(metadata)
                        depth_datasets.append((dataset, source))
                    depth_sidecar = str(depth_path.relative_to(output))

                written = 0
                # The final image has no future camera-time state to use as an action.
                for frame_index in range(len(row_indices) - 1):
                    images: dict[str, np.ndarray] = {}
                    for camera_index, (reader, (_, feature)) in enumerate(
                        zip(readers, camera_specs, strict=True)
                    ):
                        source_frame = int(camera_rows[frame_index, camera_index])
                        image = reader.read(source_frame)
                        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                        if image.shape[:2] != (args.image_size, args.image_size):
                            image = cv2.resize(
                                image,
                                (args.image_size, args.image_size),
                                interpolation=cv2.INTER_AREA,
                            )
                        images[feature] = np.ascontiguousarray(image, dtype=np.uint8)
                    dataset.add_frame(
                        {
                            **images,
                            "observation.state": states[row_indices[frame_index]],
                            "action": states[row_indices[frame_index + 1]],
                            "task": args.task,
                        }
                    )
                    for camera_index, (depth_dataset, depth_source) in enumerate(
                        depth_datasets
                    ):
                        depth_dataset[written] = depth_source[
                            int(camera_rows[frame_index, camera_index])
                        ]
                    written += 1
                if written < 2:
                    dataset.clear_episode_buffer()
                    raise ValueError(f"{trial}: fewer than two synchronized frames could be read")
                dataset.save_episode()
                if column_offset and not args.skip_depth:
                    for path in video_paths:
                        shutil.copy2(path, rgbd_dir / path.name)
            finally:
                if depth_file is not None:
                    depth_file.close()
                for depth_source, _ in depth_sources:
                    close_depth_source(depth_source)
                for capture in captures:
                    capture.release()

            report = {
                "trial": str(trial.relative_to(raw_root)),
                "raw_robot_rows": total_robot_rows,
                "teleop_active_rows": teleop_active_rows,
                "selected_robot_rows": int(active_stop - active_start),
                "kept_inactive_rows": not args.active_only,
                "active_row_span": [active_start, active_stop],
                "source_video_frames": dict(
                    zip((feature for _, feature in camera_specs), counts, strict=True)
                ),
                "written_frames": written,
                "alignment": alignment_method,
                "max_camera_skew_ms": max_camera_skew_ms,
                "max_robot_skew_ms": max_robot_skew_ms,
                "depth_sidecar": depth_sidecar,
            }
            reports.append(report)
            print(f"converted {trial}: {written} frames", file=sys.stderr)
    finally:
        dataset.finalize()

    conversion_report = {
        "format": "LeRobotDataset-v3.0",
        "source": str(raw_root),
        "output": str(output),
        "repo_id": args.repo_id,
        "fps": args.fps,
        "task": args.task,
        "camera_features": [feature for _, feature in camera_specs],
        "state": "follower [q1..q7, gripper_width]",
        "action": "next video-frame follower [q1..q7, gripper_width]",
        "timestamp_warning": (
            "Legacy episodes without host timestamps use normalized episode progress and "
            "remain approximate; v2 RGB-D episodes use host steady-clock nearest matching."
        ),
        "episodes": reports,
    }
    report_path = output / "meta" / "vla_conversion_report.json"
    report_path.write_text(json.dumps(conversion_report, indent=2, ensure_ascii=False))
    return conversion_report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_root", type=Path, help="session or data directory containing trials")
    parser.add_argument("output", type=Path, help="new LeRobot v3 dataset directory")
    parser.add_argument("--repo-id", default="local/vla_threading")
    parser.add_argument("--task", default="threading")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--image-size", type=int, default=96)
    parser.add_argument("--max-camera-skew-ms", type=float, default=25.0)
    parser.add_argument("--max-robot-skew-ms", type=float, default=5.0)
    parser.add_argument(
        "--skip-depth",
        action="store_true",
        help="do not copy aligned Z16 depth sidecars (timestamp alignment is still used)",
    )
    parser.add_argument(
        "--camera",
        action="append",
        type=parse_camera,
        metavar="RAW_FILE=FEATURE",
        help="repeat for each camera; defaults to cam1=sideview and cam3=frontview",
    )
    parser.add_argument(
        "--active-only",
        action="store_true",
        help="keep only the longest teleoperation-active span; all rows are kept by default",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if (
        args.fps <= 0
        or args.image_size <= 0
        or args.max_camera_skew_ms <= 0
        or args.max_robot_skew_ms <= 0
    ):
        raise ValueError("fps, image-size, and timestamp skew limits must be positive")
    report = convert(args)
    print(json.dumps({"output": report["output"], "episodes": len(report["episodes"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
