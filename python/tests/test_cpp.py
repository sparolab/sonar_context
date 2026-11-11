"""C++ and Python give the same distances and shifts (needs cpp/build/sonar_context_distances)."""
import os
import subprocess
from pathlib import Path

import numpy as np
import pytest

from sonar_context import column_distance, row_distance

TOOL = Path(os.environ.get("SONAR_CONTEXT_CPP_TOOL",
                           Path(__file__).resolve().parents[2] / "cpp/build/sonar_context_distances"))


@pytest.mark.skipif(not TOOL.exists(), reason="C++ tool not built")
def test_cpp_matches_python(tmp_path):
    rng = np.random.default_rng(0)
    base = rng.random((6, 40, 25)) ** 4 * 255
    base[:, :, :3] = 0                                   # empty columns
    sc = np.concatenate([base, np.roll(base, 3, axis=2), np.roll(base, -2, axis=1)])
    pairs = [(i, j) for i in range(len(sc)) for j in range(len(sc)) if (i + 2 * j) % 5 == 0]
    sc.astype(np.float64).tofile(tmp_path / "sc.f64")
    (tmp_path / "pairs.txt").write_text("\n".join(f"{i} {j}" for i, j in pairs))
    out = subprocess.run([str(TOOL), str(tmp_path / "sc.f64"), str(len(sc)), "40", "25", str(tmp_path / "pairs.txt")],
                         check=True, capture_output=True, text=True).stdout.split("\n")
    for (i, j), line in zip(pairs, out):
        _, _, cd, cs, rd, rs = line.split()
        d, s = column_distance(sc[i], sc[j])
        assert float(cd) == pytest.approx(d, abs=1e-9) and int(cs) == s
        d, s = row_distance(sc[i], sc[j])
        assert float(rd) == pytest.approx(d, abs=1e-9) and int(rs) == s
