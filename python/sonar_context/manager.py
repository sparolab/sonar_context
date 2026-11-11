"""Online place recognition: add one frame at a time and ask for a loop closure."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from scipy.spatial import cKDTree

from .descriptor import polar_key, sonar_context
from .matching import column_distance, row_distance


@dataclass
class Loop:
    query: int            # index of the current frame
    match: int            # index of the revisited frame
    distance: float       # column distance (or row distance when the row search accepted it)
    column_shift: int     # bearing shift, in Sonar Context columns
    row_shift: int        # range shift, in Sonar Context rows (0 unless the row search accepted it)
    by: str               # "column" or "row"


class SonarContextManager:
    """Keeps the Sonar Contexts and polar keys of the frames seen so far.

    ``detect_loop`` (i) finds the ``num_candidates`` nearest polar keys among the frames older than
    ``exclude_recent``, (ii) accepts the candidate with the smallest column distance if it is below
    ``col_threshold``, else (iii) accepts it if its row distance is below ``row_threshold``.
    """

    def __init__(self, col_threshold: float = 0.045, row_threshold: float = 0.072, exclude_recent: int = 30,
                 num_candidates: int = 1, patch=(4, 4), layout: str = "range_beam",
                 col_bound: float = 0.5, row_bound: float = 0.1, skip_range: int = 0):
        self.col_threshold, self.row_threshold = col_threshold, row_threshold
        self.exclude_recent, self.num_candidates = exclude_recent, num_candidates
        self.patch, self.layout = patch, layout
        self.col_bound, self.row_bound = col_bound, row_bound
        self.skip_range = skip_range
        self.contexts: List[np.ndarray] = []
        self.keys: List[np.ndarray] = []
        self._tree, self._tree_n = None, 0

    def __len__(self) -> int:
        return len(self.contexts)

    def add(self, sc: np.ndarray, pk: Optional[np.ndarray] = None) -> int:
        """Add a Sonar Context (and its polar key, computed if omitted); returns its index."""
        sc = np.asarray(sc, dtype=np.float64)
        self.contexts.append(sc)
        self.keys.append(polar_key(sc) if pk is None else np.asarray(pk, dtype=np.float64))
        return len(self.contexts) - 1

    def add_image(self, image: np.ndarray) -> int:
        """Add a polar sonar image; returns its index."""
        return self.add(sonar_context(image, self.patch, self.layout, self.skip_range))

    def candidates(self, idx: int) -> List[int]:
        n = idx - self.exclude_recent
        if n < 1:
            return []
        if self._tree is None or self._tree_n != n:
            self._tree, self._tree_n = cKDTree(np.asarray(self.keys[:n])), n
        k = min(self.num_candidates, n)
        _, ii = self._tree.query(self.keys[idx], k=k)
        return [int(i) for i in np.atleast_1d(ii)]

    def detect_loop(self, idx: Optional[int] = None) -> Optional[Loop]:
        """A loop closure for frame ``idx`` (default: the last one added), or None."""
        idx = len(self) - 1 if idx is None else idx
        cands = self.candidates(idx)
        if not cands:
            return None
        q = self.contexts[idx]
        scored = [(column_distance(self.contexts[c], q, self.col_bound), c) for c in cands]
        (d, s), c = min(scored, key=lambda t: t[0][0])
        if d < self.col_threshold:
            return Loop(idx, c, d, s, 0, "column")
        rd, rs = row_distance(self.contexts[c], q, self.row_bound)
        if rd < self.row_threshold:
            return Loop(idx, c, rd, s, rs, "row")
        return None

    def update(self, image: np.ndarray) -> Optional[Loop]:
        """add_image + detect_loop."""
        self.add_image(image)
        return self.detect_loop()
