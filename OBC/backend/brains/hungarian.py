"""
SwarmTech3_UpRising: Hungarian Kuhn-Munkres Optimal Bipartite Matching (hungarian.py)
Provides minimal-displacement slot assignment for formation inversion and U-turns (TKT-43).
Zero external dependencies (pure NumPy O(N^3) potential-based solver).
"""

from __future__ import annotations
from typing import List, Tuple
import numpy as np

try:
    from scipy.optimize import linear_sum_assignment as _scipy_lsa
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False


def hungarian_assignment(cost_matrix: np.ndarray) -> Tuple[List[int], List[int]]:
    """Solve the linear sum assignment problem (minimum weight bipartite matching).
    Given an N x M cost matrix C, finds a 1-to-1 matching between rows and columns
    that minimizes the total cost sum(C[i, col[i]]).

    Returns:
        (row_ind, col_ind): Parallel lists of matched row and column indices.
    """
    cost = np.array(cost_matrix, dtype=float)
    if cost.size == 0:
        return [], []

    if HAVE_SCIPY:
        r, c = _scipy_lsa(cost)
        return r.tolist(), c.tolist()

    transposed = False
    if cost.shape[0] > cost.shape[1]:
        cost = cost.T
        transposed = True

    n_rows, n_cols = cost.shape
    u = np.zeros(n_rows + 1, dtype=float)
    v = np.zeros(n_cols + 1, dtype=float)
    p = np.zeros(n_cols + 1, dtype=int)
    way = np.zeros(n_cols + 1, dtype=int)

    for i in range(1, n_rows + 1):
        p[0] = i
        j0 = 0
        minv = np.full(n_cols + 1, np.inf, dtype=float)
        used = np.zeros(n_cols + 1, dtype=bool)

        while True:
            used[j0] = True
            i0 = p[j0]
            delta = np.inf
            j1 = 0

            for j in range(1, n_cols + 1):
                if not used[j]:
                    cur = cost[i0 - 1, j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j

            for j in range(n_cols + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta

            j0 = j1
            if p[j0] == 0:
                break

        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break

    if not transposed:
        ans = [-1] * n_rows
        for j in range(1, n_cols + 1):
            if p[j] != 0:
                ans[p[j] - 1] = j - 1
        return list(range(n_rows)), ans
    else:
        ans = [-1] * n_cols
        for j in range(1, n_cols + 1):
            if p[j] != 0:
                ans[j - 1] = p[j] - 1
        valid_rows = [i for i, c in enumerate(ans) if c != -1]
        valid_cols = [ans[i] for i in valid_rows]
        return valid_rows, valid_cols
