from __future__ import annotations

from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build_vla_pointcloud_dataset import (
    camera_rays,
    crop_and_sample,
    depth_to_base_points,
    parse_camera,
)
from convert_vla_to_lerobot_v3 import close_depth_source, open_depth_memmap


def metadata(width: int = 2, height: int = 2) -> dict:
    return {
        "width": width,
        "height": height,
        "color_intrinsics": {
            "width": width,
            "height": height,
            "fx": 1.0,
            "fy": 1.0,
            "ppx": 0.0,
            "ppy": 0.0,
            "coeffs": [0.0] * 5,
        },
    }


def test_depth_deprojection_identity() -> None:
    rays = camera_rays(metadata())
    points, valid = depth_to_base_points(
        np.array([[1000, 0], [2000, 1000]], dtype=np.uint16),
        rays,
        0.001,
        np.eye(4),
    )
    assert valid.tolist() == [[True, False], [True, True]]
    np.testing.assert_allclose(points, [[0, 0, 1], [0, 2, 2], [1, 1, 1]])


def test_camera_from_base_is_inverted() -> None:
    camera_from_base = np.eye(4)
    camera_from_base[0, 3] = 1.0
    points, _ = depth_to_base_points(
        np.array([[1000]], dtype=np.uint16),
        camera_rays(metadata(1, 1)),
        0.001,
        camera_from_base,
    )
    np.testing.assert_allclose(points, [[-1.0, 0.0, 1.0]])


def test_nonzero_distortion_is_rejected() -> None:
    value = metadata()
    value["color_intrinsics"]["coeffs"][0] = 0.1
    with pytest.raises(ValueError, match="distortion"):
        camera_rays(value)


def test_crop_and_sample_is_fixed_size_and_deterministic() -> None:
    points = [np.array([[0, 0, 0.5], [2, 0, 0.5]], dtype=np.float32)]
    colors = [np.array([[1, 2, 3], [4, 5, 6]], dtype=np.uint8)]
    camera_ids = [np.array([0, 0], dtype=np.uint8)]
    args = (points, colors, camera_ids, np.array([-1, -1, 0, 1, 1, 1]), 4)
    first = crop_and_sample(*args, np.random.default_rng(3))
    second = crop_and_sample(*args, np.random.default_rng(3))
    assert first[0].shape == (4, 3)
    assert first[3] == 1
    for left, right in zip(first[:3], second[:3], strict=True):
        np.testing.assert_array_equal(left, right)


def test_camera_argument() -> None:
    assert parse_camera("cam1.mp4=sideview") == ("cam1.mp4", "sideview")
    with pytest.raises(Exception):
        parse_camera("sideview")


def test_indexed_zstd_depth_round_trip(tmp_path: Path) -> None:
    frames = [
        np.array([[1, 2], [3, 4]], dtype="<u2"),
        np.array([[100, 200], [300, 400]], dtype="<u2"),
    ]
    compressed = [
        subprocess.run(
            ["zstd", "-1", "-q", "-c"],
            input=frame.tobytes(),
            check=True,
            capture_output=True,
        ).stdout
        for frame in frames
    ]
    offsets = [0, len(compressed[0])]
    (tmp_path / "cam1_depth.z16.zst").write_bytes(b"".join(compressed))
    (tmp_path / "cam1_metadata.json").write_text(
        '{"width": 2, "height": 2, "depth_scale_m": 0.001}'
    )
    (tmp_path / "cam1_timestamps.csv").write_text(
        "frame_index,host_steady_timestamp_ns,depth_offset_bytes,depth_compressed_bytes\n"
        f"0,100,{offsets[0]},{len(compressed[0])}\n"
        f"1,200,{offsets[1]},{len(compressed[1])}\n"
    )
    source, _ = open_depth_memmap(tmp_path, "cam1.mp4", 2)
    try:
        np.testing.assert_array_equal(source[0], frames[0])
        np.testing.assert_array_equal(source[1], frames[1])
    finally:
        close_depth_source(source)
