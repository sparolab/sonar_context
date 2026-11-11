"""Trajectory error of a SLAM session against Aracati 2017's /pose_gt (positions only: 2D Umeyama alignment).

    python evaluate.py <session> --gt pose_gt.txt [--plot out.png]
"""
import argparse
from pathlib import Path

import numpy as np


def align2d(P, Q):
    """R, t minimizing |R P + t - Q| (rows are 2D points)."""
    mp, mq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - mp).T @ (Q - mq))
    D = np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, mq - R @ mp


def ate(P, Q):
    R, t = align2d(P, Q)
    e = np.linalg.norm(P @ R.T + t - Q, axis=1)
    return np.sqrt((e ** 2).mean()), e, P @ R.T + t


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("session")
    p.add_argument("--gt", required=True)
    p.add_argument("--plot")
    a = p.parse_args()
    s = Path(a.session)
    t = np.loadtxt(s / "times.txt")
    g = np.loadtxt(a.gt)
    Q = np.c_[np.interp(t, g[:, 0], g[:, 2]), np.interp(t, g[:, 0], g[:, 3])]
    out = {}
    for name in ("odom_poses", "poses"):
        P = np.loadtxt(s / f"{name}.txt")[:, [3, 7]]
        rmse, e, Pa = ate(P, Q)
        out[name] = (rmse, e, Pa)
        print(f"{name:10s}: ATE RMSE {rmse:.2f} m, mean {e.mean():.2f} m, max {e.max():.2f} m ({len(P)} keyframes)")
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 6))
        ax.plot(Q[:, 0], Q[:, 1], "k-", lw=1, label="ground truth (DGPS)")
        ax.plot(*out["odom_poses"][2].T, "-", c="#9ca3af", lw=1, label=f"DISO odometry ({out['odom_poses'][0]:.2f} m)")
        ax.plot(*out["poses"][2].T, "-", c="#2563eb", lw=1.5, label=f"+ Sonar Context loops ({out['poses'][0]:.2f} m)")
        ax.set_aspect("equal"); ax.legend(); fig.tight_layout(); fig.savefig(a.plot, dpi=150)


if __name__ == "__main__":
    main()
