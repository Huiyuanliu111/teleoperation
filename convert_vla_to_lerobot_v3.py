#!/usr/bin/env python3
"""Convert vla_finetune raw trials into an official LeRobot v3 dataset.

The raw recorder stores one ``DATA_follower.m`` matrix and one MP4 per camera
inside every trial directory. Robot rows are sampled at roughly 1 kHz while
the videos are recorded at 30 FPS. Because the current recorder does not save
clock timestamps, this converter aligns both streams by normalized episode
progress. The approximation is recorded in ``meta/vla_conversion_report.json``.

Example:
    python convert_vla_to_lerobot_v3.py \
        vla_finetune/data/Threading_20260826_120000 \
        data/threading_vla_lerobot_v3

Default camera mapping is ``cam1.mp4`` = side view and ``cam2.mp4`` = wrist.
Use repeated ``--camera RAW_FILE=FEATURE`` arguments if a deployment uses the
opposite numbering.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np


ROBOT_COLUMNS = 29
STATE_NAMES = [f"q{i}" for i in range(1, 8)] + ["gripper_width"]
DEFAULT_CAMERAS = (
    ("cam1.mp4", "observation.images.exterior_image_2_right"),
    ("cam2.mp4", "observation.images.wrist_image_left"),
)


def read_follower_matrix(path: Path) -> np.ndarray:
    """Read the Eigen matrix written by Recorder.cpp as a MATLAB assignment."""
    text = path.read_text(errors="strict")
    start = text.find("[")
    stop = text.rfind("]")
    if start < 0 or stop <= start:
        raise ValueError(f"{path}: expected a MATLAB matrix assignment")
    values = np.fromstring(text[start + 1 : stop].replace(";", " "), sep=" ")
    if values.size == 0 or values.size % ROBOT_COLUMNS:
        raise ValueError(
            f"{path}: found {values.size} values, not a non-empty multiple of {ROBOT_COLUMNS}"
        )
    matrix = values.reshape(-1, ROBOT_COLUMNS)
    if not np.isfinite(matrix).all():
        raise ValueError(f"{path}: robot matrix contains NaN or Inf")
    return matrix


def longest_active_span(matrix: np.ndarray) -> tuple[int, int]:
    """Return [start, stop) for the longest contiguous teleoperation-active run."""
    active = matrix[:, 0] > 0.5
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
        for trial in trials:
            matrix = read_follower_matrix(trial / "DATA_follower.m")
            if args.keep_inactive:
                active_start, active_stop = 0, len(matrix)
            else:
                active_start, active_stop = longest_active_span(matrix)
            matrix = matrix[active_start:active_stop]
            states = np.concatenate((matrix[:, 1:8], matrix[:, 8:9]), axis=1).astype(
                np.float32
            )

            captures: list[Any] = []
            video_paths: list[Path] = []
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
                target_frames = min(counts)
                row_indices = resample_indices(len(states), target_frames)
                written = 0
                # The final image has no future camera-time state to use as an action.
                for frame_index in range(target_frames - 1):
                    images: dict[str, np.ndarray] = {}
                    complete = True
                    for capture, (_, feature) in zip(captures, camera_specs, strict=True):
                        ok, image = capture.read()
                        if not ok:
                            complete = False
                            break
                        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                        if image.shape[:2] != (args.image_size, args.image_size):
                            image = cv2.resize(
                                image,
                                (args.image_size, args.image_size),
                                interpolation=cv2.INTER_AREA,
                            )
                        images[feature] = np.ascontiguousarray(image, dtype=np.uint8)
                    if not complete:
                        break
                    dataset.add_frame(
                        {
                            **images,
                            "observation.state": states[row_indices[frame_index]],
                            "action": states[row_indices[frame_index + 1]],
                            "task": args.task,
                        }
                    )
                    written += 1
                if written < 2:
                    dataset.clear_episode_buffer()
                    raise ValueError(f"{trial}: fewer than two synchronized frames could be read")
                dataset.save_episode()
            finally:
                for capture in captures:
                    capture.release()

            report = {
                "trial": str(trial.relative_to(raw_root)),
                "raw_robot_rows": int(active_stop - active_start),
                "active_row_span": [active_start, active_stop],
                "source_video_frames": dict(
                    zip((feature for _, feature in camera_specs), counts, strict=True)
                ),
                "written_frames": written,
                "alignment": "normalized_episode_progress",
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
            "Raw recorder timestamps are not persisted; robot/video alignment uses normalized "
            "episode progress and is approximate."
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
    parser.add_argument(
        "--camera",
        action="append",
        type=parse_camera,
        metavar="RAW_FILE=FEATURE",
        help="repeat for each camera; defaults to cam1=sideview and cam2=wrist",
    )
    parser.add_argument(
        "--keep-inactive",
        action="store_true",
        help="keep all robot rows instead of the longest teleoperation-active span",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.fps <= 0 or args.image_size <= 0:
        raise ValueError("fps and image-size must be positive")
    report = convert(args)
    print(json.dumps({"output": report["output"], "episodes": len(report["episodes"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
