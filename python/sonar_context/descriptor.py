"""Sonar Context and polar key of one imaging-sonar frame.

The input is the *polar* sonar image with rows = range bins and columns = beams (bearing), so that
a column shift of the descriptor is a rotation of the vehicle and a row shift a motion along the
beams. Many datasets store the polar image the other way around (Aracati 2017: beams x range);
pass ``layout="beam_range"`` for those.
"""
from __future__ import annotations

import numpy as np

LAYOUTS = ("range_beam", "beam_range")


def as_range_beam(image: np.ndarray, layout: str = "range_beam") -> np.ndarray:
    """The image as (range bins, beams), grayscale float."""
    if layout not in LAYOUTS:
        raise ValueError(f"layout must be one of {LAYOUTS}, not {layout!r}")
    img = np.asarray(image, dtype=np.float64)
    if img.ndim == 3:  # color: mean over channels
        img = img.mean(axis=2)
    if img.ndim != 2:
        raise ValueError(f"expected a 2D image, got shape {np.shape(image)}")
    return img.T if layout == "beam_range" else img


def sonar_context(image: np.ndarray, patch=(4, 4), layout: str = "range_beam", skip_range: int = 0) -> np.ndarray:
    """Sonar Context: the maximum intensity of each ``patch`` (range bins x beams) of the polar image.

    Returns an array (rows = range rings, columns = bearing sectors). Pixels that do not fill a whole
    patch at the end of the image are dropped (as in the original implementation). ``skip_range`` drops
    the first range bins (the bright near field), 0 by default.
    """
    img = as_range_beam(image, layout)[int(skip_range):]
    pr, pc = int(patch[0]), int(patch[1])
    if pr < 1 or pc < 1:
        raise ValueError("patch sizes must be >= 1")
    r, c = img.shape[0] // pr, img.shape[1] // pc
    if r == 0 or c == 0:
        raise ValueError(f"image {img.shape} is smaller than one patch {patch}")
    return img[: r * pr, : c * pc].reshape(r, pr, c, pc).max(axis=(1, 3))


def polar_key(sc: np.ndarray) -> np.ndarray:
    """Polar key: the mean of each range ring of a Sonar Context; invariant to the bearing (column) shift."""
    return np.asarray(sc, dtype=np.float64).mean(axis=1)
