"""Readers for sonar image sequences and poses."""
from __future__ import annotations

import io
import re
import zipfile
from pathlib import Path
from typing import Callable, List, Tuple

import numpy as np

IMAGE_EXT = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")


def _natural(s: str):
    return [float(t) if re.fullmatch(r"\d+(\.\d+)?", t) else t for t in re.split(r"(\d+(?:\.\d+)?)", s)]


def image_sequence(path) -> Tuple[List[str], Callable[[str], np.ndarray]]:
    """Names (natural order) and a loader of the images in a folder or a .zip archive (grayscale arrays)."""
    from PIL import Image
    p = Path(path)
    if p.suffix == ".zip":
        z = zipfile.ZipFile(p)
        names = [n for n in z.namelist() if n.lower().endswith(IMAGE_EXT)]
        load = lambda n: np.asarray(Image.open(io.BytesIO(z.read(n))).convert("L"))
    elif p.is_dir():
        names = [str(f) for f in p.iterdir() if f.suffix.lower() in IMAGE_EXT]
        load = lambda n: np.asarray(Image.open(n).convert("L"))
    else:
        raise FileNotFoundError(f"{path}: not a folder or a .zip of images")
    names.sort(key=lambda n: _natural(Path(n).name))
    return names, load


def stamps_from_names(names) -> np.ndarray:
    """Timestamps from file names such as ``1500993223.613.png`` (NaN when a name is not a number)."""
    out = []
    for n in names:
        try:
            out.append(float(Path(n).stem))
        except ValueError:
            out.append(np.nan)
    return np.array(out)


def read_aracati_poses(path) -> Tuple[np.ndarray, np.ndarray]:
    """Aracati 2017 ``/pose_gt`` as text (``stamp bagtime x y z qx qy qz qw`` per line): stamps (N,) and
    4x4 poses (N, 4, 4)."""
    from scipy.spatial.transform import Rotation
    g = np.loadtxt(path, ndmin=2)
    T = np.tile(np.eye(4), (len(g), 1, 1))
    T[:, :3, 3] = g[:, 2:5]
    T[:, :3, :3] = Rotation.from_quat(g[:, 5:9]).as_matrix()
    return g[:, 0], T


def interpolate_positions(t: np.ndarray, t_ref: np.ndarray, T_ref: np.ndarray) -> np.ndarray:
    """(len(t), 3) positions interpolated linearly in time."""
    return np.stack([np.interp(t, t_ref, T_ref[:, i, 3]) for i in range(3)], axis=1)


def keyframes_by_distance(xyz: np.ndarray, step: float) -> np.ndarray:
    """Indices of the frames at least ``step`` meters (path length) apart, starting with frame 0."""
    keep, last = [0], xyz[0]
    for i in range(1, len(xyz)):
        if np.linalg.norm(xyz[i] - last) >= step:
            keep.append(i)
            last = xyz[i]
    return np.array(keep)
