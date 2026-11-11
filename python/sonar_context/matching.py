"""Distances between two Sonar Contexts.

``column_distance`` searches the bearing (column) shift, ``row_distance`` the range (row) shift.
For each shift the two descriptors overlap on some columns (rows); the similarity at that shift is
the mean cosine similarity of the overlapping, non-empty column (row) pairs, and the distance is
1 - the best similarity. Shift 0 is included, and the shift that wins is returned with it.

Fixes over the code of the ICRA 2023 paper (kept at tag v1.0.0):

* the best shift was read one entry off (``argmax + 1``), and could index past the array;
* shift 0 (the same heading) was never tested;
* ``row_distance_sc`` compared the unshifted rows, so the row search had no effect;
* a shift with no non-empty overlap divided by zero (NaN).
"""
from __future__ import annotations

import numpy as np


def _unit(x: np.ndarray, axis: int):
    n = np.linalg.norm(x, axis=axis, keepdims=True)
    u = np.divide(x, n, out=np.zeros_like(x, dtype=np.float64), where=n > 0)
    return u, np.squeeze(n > 0, axis=axis)


def _shift_search(a: np.ndarray, b: np.ndarray, max_shift: int):
    """a, b: (n, d) vectors (columns or rows). Best mean cosine over shifts s in [-max_shift, max_shift]
    where a[j - s] is compared with b[j]. Returns (similarity, shift)."""
    ua, va = _unit(a, 1)
    ub, vb = _unit(b, 1)
    C = ua @ ub.T                     # C[i, j]: cosine of a_i and b_j
    V = va[:, None] & vb[None, :]
    n = len(a)
    best, best_s = -np.inf, 0
    for s in sorted(range(-max_shift, max_shift + 1), key=abs):  # ties go to the smallest |shift|
        j = np.arange(max(s, 0), min(n, n + s))
        if len(j) == 0:
            continue
        v = V[j - s, j]
        k = v.sum()
        if k == 0:
            continue
        sim = C[j - s, j][v].sum() / k
        if sim > best + 1e-12:
            best, best_s = sim, s
    if not np.isfinite(best):
        return 0.0, 0
    return float(best), int(best_s)


def column_distance(sc1: np.ndarray, sc2: np.ndarray, bound: float = 0.5):
    """Distance over bearing shifts of up to ``bound`` x the number of columns, both ways.

    Returns ``(distance, shift)``: shift > 0 means sc1's column j - shift matches sc2's column j.
    """
    sc1, sc2 = np.asarray(sc1, float), np.asarray(sc2, float)
    if sc1.shape != sc2.shape:
        raise ValueError(f"shapes differ: {sc1.shape} vs {sc2.shape}")
    sim, s = _shift_search(sc1.T, sc2.T, int(sc1.shape[1] * bound))
    return 1.0 - sim, s


def row_distance(sc1: np.ndarray, sc2: np.ndarray, bound: float = 0.1):
    """Distance over range shifts of up to ``bound`` x the number of rows, both ways: ``(distance, shift)``."""
    sc1, sc2 = np.asarray(sc1, float), np.asarray(sc2, float)
    if sc1.shape != sc2.shape:
        raise ValueError(f"shapes differ: {sc1.shape} vs {sc2.shape}")
    sim, s = _shift_search(sc1, sc2, int(sc1.shape[0] * bound))
    return 1.0 - sim, s


def column_similarity_matrix(A: np.ndarray, B: np.ndarray, bound: float = 0.5, chunk: int = 64) -> np.ndarray:
    """(len(A), len(B)) column similarities (1 - column_distance) of two stacks of Sonar Contexts.

    For all pairs at once: the sum of column cosines at every bearing shift is a cross-correlation along
    the columns, summed over the rows, done with FFTs (one matrix product per frequency).
    """
    A, B = np.asarray(A, float), np.asarray(B, float)
    if A.shape[1:] != B.shape[1:]:
        raise ValueError(f"shapes differ: {A.shape[1:]} vs {B.shape[1:]}")
    ua, va = _unit(A, 1)  # (N, rows, cols): unit columns, zero where empty
    ub, vb = _unit(B, 1)
    cols = A.shape[2]
    m = int(cols * bound)
    L = 2 * cols
    FA, FB = np.fft.rfft(ua, L, axis=2), np.fft.rfft(ub, L, axis=2)          # (N, rows, F)
    GA, GB = np.fft.rfft(va.astype(float), L, axis=1), np.fft.rfft(vb.astype(float), L, axis=1)  # (N, F)
    FBt = FB.transpose(2, 1, 0)                                               # (F, rows, Nb)
    shifts = np.arange(-m, m + 1)
    S = np.zeros((len(A), len(B)))
    for i0 in range(0, len(A), chunk):
        fa = np.conj(FA[i0:i0 + chunk]).transpose(2, 0, 1)                   # (F, a, rows)
        num = np.fft.irfft(np.matmul(fa, FBt).transpose(1, 2, 0), L, axis=2)  # (a, b, L): lag s = j - i
        cnt = np.fft.irfft(np.conj(GA[i0:i0 + chunk])[:, None] * GB[None], L, axis=2)
        num, cnt = num[..., shifts % L], np.rint(cnt[..., shifts % L])
        sim = np.where(cnt > 0, num / np.maximum(cnt, 1), -np.inf)
        best = sim.max(axis=-1)
        S[i0:i0 + chunk] = np.where(np.isfinite(best), best, 0.0)
    return S
