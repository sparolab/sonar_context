import numpy as np
import pytest

from sonar_context import (SonarContextManager, column_distance, column_similarity_matrix, polar_key, row_distance,
                           sonar_context)


def scene(seed=0, rows=120, cols=60):
    rng = np.random.default_rng(seed)
    img = rng.random((rows, cols)) * 20
    for _ in range(25):  # bright blobs (returns)
        r, c = rng.integers(0, rows - 6), rng.integers(0, cols - 4)
        img[r:r + 6, c:c + 4] += rng.uniform(100, 255)
    return img


def test_descriptor_shape_and_layout():
    img = scene()
    sc = sonar_context(img, (4, 4))
    assert sc.shape == (30, 15)
    assert np.allclose(sc[0, 0], img[:4, :4].max())
    assert np.allclose(sonar_context(img.T, (4, 4), layout="beam_range"), sc)
    assert polar_key(sc).shape == (30,)
    with pytest.raises(ValueError):
        sonar_context(img, (4, 4), layout="xy")


def test_identical_is_zero_at_shift_zero():
    sc = sonar_context(scene())
    d, s = column_distance(sc, sc)
    assert d == pytest.approx(0.0, abs=1e-12) and s == 0
    d, s = row_distance(sc, sc)
    assert d == pytest.approx(0.0, abs=1e-12) and s == 0


@pytest.mark.parametrize("shift", [-5, -1, 1, 3, 7])
def test_column_shift_recovered(shift):
    sc = sonar_context(scene(1))
    moved = np.roll(sc, shift, axis=1)
    if shift > 0:
        moved[:, :shift] = 0
    else:
        moved[:, shift:] = 0
    d, s = column_distance(sc, moved)
    assert s == shift and d < 1e-9


@pytest.mark.parametrize("shift", [-2, 2])
def test_row_shift_recovered(shift):
    sc = sonar_context(scene(2))
    moved = np.roll(sc, shift, axis=0)
    if shift > 0:
        moved[:shift] = 0
    else:
        moved[shift:] = 0
    d, s = row_distance(sc, moved)
    assert s == shift and d < 1e-9


def test_empty_overlap_no_nan():
    a = np.zeros((10, 8))
    d, s = column_distance(a, a)
    assert np.isfinite(d)


def test_matrix_matches_pairwise():
    A = np.stack([sonar_context(scene(i)) for i in range(4)])
    B = np.stack([np.roll(A[0], 2, axis=1), A[3]])
    S = column_similarity_matrix(A, B)
    for i in range(len(A)):
        for j in range(len(B)):
            assert S[i, j] == pytest.approx(1 - column_distance(A[i], B[j])[0], abs=1e-9)


def test_manager_finds_revisit():
    m = SonarContextManager(exclude_recent=3)
    seq = [scene(i) for i in range(10)]
    for img in seq:
        assert m.update(img) is None
    lp = m.update(np.roll(seq[2], 8, axis=1))
    assert lp is not None and lp.match == 2 and lp.query == 10
