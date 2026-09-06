#!/usr/bin/env python3
"""Detect and remove the pre-task prefix before the arm enters cam1.

Detection uses the fixed cam1 background: only positive (brighter) changes near
the top edge are considered, which rejects the large dark shadow that precedes
the white robot.  A short persistence window rejects compression noise.

Run detection first so every boundary can be reviewed or edited before trim:

    python trim_invisible_prefix.py detect data/original trim_manifest.json review
    python trim_invisible_prefix.py trim data/original data/original_trimmed \
        trim_manifest.json
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
from typing import Any

import cv2
import numpy as np


CAMERA_FILES = ("cam1.mp4", "cam2.mp4")
ROBOT_FILE = "DATA_follower.m"
MINIMUM_COMPONENT_AREA = 50


def discover_trials(raw_root: Path) -> list[Path]:
    trials = sorted(path.parent for path in raw_root.rglob(ROBOT_FILE))
    if not trials:
        raise FileNotFoundError(f"no {ROBOT_FILE} files found under {raw_root}")
    return trials


def video_info(path: Path) -> dict[str, float | int]:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise ValueError(f"cannot open video: {path}")
    try:
        return {
            "frames": int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),
            "fps": float(capture.get(cv2.CAP_PROP_FPS)),
            "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        }
    finally:
        capture.release()


def entry_score(gray: np.ndarray, background: np.ndarray) -> int:
    """Return the largest bright foreground component touching the top edge."""
    positive = cv2.subtract(gray, background)
    # The white robot enters through the black top border. Requiring both an
    # absolute brightness and contact with that border rejects the moving dark
    # shadow (and the table revealed as that shadow moves).
    mask = ((positive > 18) & (gray > 100)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    return max(
        (
            int(area)
            for _x, y, _width, height, area in stats[1:]
            if y <= 3 and height >= 8
        ),
        default=0,
    )


def first_persistent_hit(
    scores: list[int],
    *,
    threshold: int = MINIMUM_COMPONENT_AREA,
    window: int = 10,
    required: int = 9,
) -> int:
    hits = [score >= threshold for score in scores]
    for window_start in range(len(hits) - window + 1):
        window_hits = hits[window_start : window_start + window]
        if sum(window_hits) >= required:
            # Never return a tolerated miss: the selected frame itself must
            # contain a detected part of the arm.
            return window_start + window_hits.index(True)
    raise ValueError("no persistent robot entry was detected")


def detect_start(path: Path) -> tuple[int, list[int], dict[str, float | int]]:
    info = video_info(path)
    if info["frames"] < 20 or info["fps"] <= 0:
        raise ValueError(f"video is too short or has invalid fps: {path}")

    capture = cv2.VideoCapture(str(path))
    warmup: list[np.ndarray] = []
    try:
        for _ in range(15):
            ok, frame = capture.read()
            if not ok:
                break
            small = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
            warmup.append(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY))
        if len(warmup) < 10:
            raise ValueError(f"could not read background frames: {path}")
        background = np.median(np.stack(warmup), axis=0).astype(np.uint8)

        capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
        scores: list[int] = []
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            small = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            scores.append(entry_score(gray, background))
    finally:
        capture.release()

    return first_persistent_hit(scores), scores, info


def read_video_frame(path: Path, frame_index: int) -> np.ndarray:
    capture = cv2.VideoCapture(str(path))
    try:
        capture.set(cv2.CAP_PROP_POS_FRAMES, max(0, frame_index))
        ok, frame = capture.read()
        if not ok:
            raise ValueError(f"could not read frame {frame_index} from {path}")
        return frame
    finally:
        capture.release()


def write_review_image(
    source: Path, destination: Path, trial_name: str, start: int, fps: float
) -> None:
    offsets = (-15, -1, 0, 15)
    panels: list[np.ndarray] = []
    total_frames = int(video_info(source)["frames"])
    for offset in offsets:
        index = min(max(start + offset, 0), total_frames - 1)
        frame = read_video_frame(source, index)
        frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
        label = f"frame {index}  t={index / fps:.3f}s"
        if offset == 0:
            label += "  START"
        panel = np.zeros((270, 320, 3), dtype=np.uint8)
        panel[30:] = frame
        cv2.putText(panel, label, (7, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        panels.append(panel)
    review = np.concatenate(panels, axis=1)
    cv2.putText(review, trial_name, (7, 265), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(destination), review):
        raise OSError(f"could not write review image: {destination}")


def matlab_row_count(path: Path) -> int:
    text = path.read_text(errors="strict")
    start, stop = text.find("["), text.rfind("]")
    if start < 0 or stop <= start:
        raise ValueError(f"invalid MATLAB matrix: {path}")
    return len(text[start + 1 : stop].splitlines())


def robot_start_row(start_frame: int, video_frames: int, robot_rows: int) -> int:
    if video_frames < 2 or robot_rows < 2:
        raise ValueError("video and robot streams must each contain at least two samples")
    return int(round(start_frame / (video_frames - 1) * (robot_rows - 1)))


def detect_command(args: argparse.Namespace) -> None:
    raw_root = args.raw_root.resolve()
    manifest_path = args.manifest.resolve()
    review_dir = args.review_dir.resolve()
    episodes: list[dict[str, Any]] = []
    for trial in discover_trials(raw_root):
        relative = trial.relative_to(raw_root)
        cam1 = trial / "cam1.mp4"
        cam2 = trial / "cam2.mp4"
        start, scores, cam1_info = detect_start(cam1)
        cam2_info = video_info(cam2)
        rows = matlab_row_count(trial / ROBOT_FILE)
        start_row = robot_start_row(start, int(cam1_info["frames"]), rows)
        next_scores = scores[start : start + 10]
        episode = {
            "trial": relative.as_posix(),
            "start_frame": start,
            "start_time_seconds": start / float(cam1_info["fps"]),
            "robot_start_row": start_row,
            "cam1": cam1_info,
            "cam2": cam2_info,
            "robot_rows": rows,
            "detection_score": scores[start],
            "median_next_10_score": float(np.median(next_scores)),
        }
        episodes.append(episode)
        review_path = review_dir / relative.parent / f"{relative.name}.jpg"
        write_review_image(
            cam1, review_path, relative.as_posix(), start, float(cam1_info["fps"])
        )
        print(f"detected {relative}: frame={start}, time={episode['start_time_seconds']:.3f}s", flush=True)

    manifest = {
        "version": 1,
        "raw_root": str(raw_root),
        "detector": {
            "resize": [320, 240],
            "positive_gray_threshold": 18,
            "absolute_gray_threshold": 100,
            "component_top_y_lte": 3,
            "minimum_component_area": MINIMUM_COMPONENT_AREA,
            "persistence": "9 of 10 frames",
            "safety_margin_frames": 0,
        },
        "episodes": episodes,
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    print(f"wrote {manifest_path} and {review_dir}")


def slice_matlab_rows(source: Path, destination: Path, start_row: int) -> None:
    text = source.read_text(errors="strict")
    start, stop = text.find("["), text.rfind("]")
    if start < 0 or stop <= start:
        raise ValueError(f"invalid MATLAB matrix: {source}")
    rows = text[start + 1 : stop].splitlines()
    if not 0 <= start_row < len(rows) - 1:
        raise ValueError(f"invalid start row {start_row} for {source} with {len(rows)} rows")
    destination.write_text(text[: start + 1] + "\n".join(rows[start_row:]) + text[stop:])


def trim_video(source: Path, destination: Path, start_frame: int) -> None:
    temporary = destination.with_suffix(".tmp.mp4")
    command = [
        "ffmpeg", "-nostdin", "-loglevel", "error", "-y", "-i", str(source),
        "-vf", f"trim=start_frame={start_frame},setpts=PTS-STARTPTS",
        "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "18",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(temporary),
    ]
    subprocess.run(command, check=True)
    os.replace(temporary, destination)


def trim_command(args: argparse.Namespace) -> None:
    raw_root = args.raw_root.resolve()
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {output}")
    manifest = json.loads(args.manifest.resolve().read_text())
    entries = {entry["trial"]: entry for entry in manifest["episodes"]}
    trials = discover_trials(raw_root)
    expected = {trial.relative_to(raw_root).as_posix() for trial in trials}
    if set(entries) != expected:
        raise ValueError("manifest trials do not exactly match the raw dataset")

    output.mkdir(parents=True)
    try:
        for trial in trials:
            relative = trial.relative_to(raw_root)
            entry = entries[relative.as_posix()]
            destination = output / relative
            destination.mkdir(parents=True)
            start_frame = int(entry["start_frame"])
            slice_matlab_rows(
                trial / ROBOT_FILE,
                destination / ROBOT_FILE,
                int(entry["robot_start_row"]),
            )
            for camera in CAMERA_FILES:
                trim_video(trial / camera, destination / camera, start_frame)
            print(f"trimmed {relative}: removed {start_frame} video frames", flush=True)
        shutil.copy2(args.manifest.resolve(), output / "trim_manifest.json")
    except BaseException:
        print(f"partial output remains at {output}; inspect it before retrying")
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    detect = subparsers.add_parser("detect")
    detect.add_argument("raw_root", type=Path)
    detect.add_argument("manifest", type=Path)
    detect.add_argument("review_dir", type=Path)
    detect.set_defaults(func=detect_command)
    trim = subparsers.add_parser("trim")
    trim.add_argument("raw_root", type=Path)
    trim.add_argument("output", type=Path)
    trim.add_argument("manifest", type=Path)
    trim.set_defaults(func=trim_command)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    args.func(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
