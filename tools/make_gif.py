"""The README GIF: Sonar Context SLAM replayed keyframe by keyframe.

    python tools/make_gif.py <session> --images kf_polar.u8 --shape 300 1500 --layout beam_range --flip \
        --gt pose_gt.txt --out assets/sonar_context_slam.gif

<session>: a SLAM session (poses.txt, odom_poses.txt, times.txt, loops.csv, points/). --images: the keyframes'
polar images, uint8, in session order. Left: the sonar image and its Sonar Context; right: the sonar map on the
optimized poses (a mosaic of the dense sonar images), DISO's odometry, the SLAM trajectory and its loops.
"""
import argparse
import csv
import io
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.ndimage import map_coordinates

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from sonar_context import sonar_context  # noqa: E402

BG, FG, MUTED = "#0d1117", "#e6edf3", "#7d8590"
BLUE, GREEN, GREY, ORANGE = "#3b82f6", "#22c55e", "#6b7280", "#f59e0b"


def fan(polar_rb, fov=130.0, size=360):
    """(range, beams) polar image -> fan image, apex at the bottom center."""
    nr, nb = polar_rb.shape
    H, W = size, int(size * 2 * np.sin(np.radians(fov / 2)))
    yy, xx = np.mgrid[0:H, 0:W]
    dx, dy = xx - W / 2, H - yy
    rho = np.hypot(dx, dy) / H * nr
    th = np.degrees(np.arctan2(dx, dy))                 # 0 up, + right
    b = (fov / 2 - th) / fov * nb                       # beam 0 on the right (y negative), as in polar_points
    out = map_coordinates(polar_rb.astype(float), [rho, b], order=1, cval=0)
    out[(rho >= nr) | (np.abs(th) > fov / 2)] = np.nan
    return out


def kitti_xy(path):
    P = np.loadtxt(path)
    return P[:, [3, 7]], np.arctan2(P[:, 4], P[:, 0])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("session")
    p.add_argument("--images", required=True)
    p.add_argument("--shape", type=int, nargs=2, required=True)
    p.add_argument("--layout", default="beam_range")
    p.add_argument("--flip", action="store_true", help="mirror the beams (as the SLAM did)")
    p.add_argument("--gt", help="Aracati pose_gt.txt, drawn faintly after a 2D alignment")
    p.add_argument("--every", type=int, default=3)
    p.add_argument("--mosaic-res", type=float, default=0.25)
    p.add_argument("--mosaic-gain", type=float, default=0, help="intensity shown as white (0: the 96th percentile)")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--out", required=True)
    a = p.parse_args()

    s = Path(a.session)
    xy, yaw = kitti_xy(s / "poses.txt")
    oxy, _ = kitti_xy(s / "odom_poses.txt")
    n = len(xy)
    loops = [(int(r["query"]), int(r["match"])) for r in csv.DictReader(open(s / "loops.csv"))]
    imgs = np.fromfile(a.images, np.uint8).reshape(-1, *a.shape)[:n]
    # odometry drawn in the SLAM frame (same start)
    gt = None
    if a.gt:
        t = np.loadtxt(s / "times.txt")
        g = np.loadtxt(a.gt)
        Q = np.c_[np.interp(t, g[:, 0], g[:, 2]), np.interp(t, g[:, 0], g[:, 3])]
        A, B = Q - Q.mean(0), xy - xy.mean(0)
        U, _, Vt = np.linalg.svd(A.T @ B)
        R = Vt.T @ np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
        gt = A @ R.T + xy.mean(0)
    allp = np.concatenate([xy, oxy])
    lo, hi = allp.min(0) - 12, allp.max(0) + 12
    res = a.mosaic_res
    gx, gy = np.arange(lo[0], hi[0], res) + res / 2, np.arange(lo[1], hi[1], res) + res / 2
    GX, GY = np.meshgrid(gx, gy, indexing="xy")          # (ny, nx)
    msum, mcnt = np.zeros_like(GX), np.zeros_like(GX)

    def paint(k):
        c = np.asarray(Image.open(s / "cartesian" / f"{k}.png"), dtype=float) / 256.0
        N = c.shape[0]
        cres, half = 0.25, N * 0.25 / 2
        sel = (np.abs(GX - xy[k, 0]) < half) & (np.abs(GY - xy[k, 1]) < half)
        dx, dy = GX[sel] - xy[k, 0], GY[sel] - xy[k, 1]
        cs, sn = np.cos(yaw[k]), np.sin(yaw[k])
        lx, ly = cs * dx + sn * dy, -sn * dx + cs * dy
        v = map_coordinates(c, [(lx + half) / cres - 0.5, (ly + half) / cres - 0.5], order=1, cval=0)
        inside = v > 0
        idx = np.flatnonzero(sel)[inside]
        msum.flat[idx] += v[inside]
        mcnt.flat[idx] += 1

    for k in range(n):                       # the final mosaic sets the brightness
        paint(k)
    final = msum / np.maximum(mcnt, 1)
    gain = a.mosaic_gain or np.percentile(final[mcnt > 0], 96.0)
    msum[:], mcnt[:] = 0, 0

    plt.rcParams.update({"font.family": "DejaVu Sans", "text.color": FG})
    frames = []
    loop_at = {}
    for q, m in loops:
        loop_at.setdefault(q, []).append(m)
    ks = list(range(0, n, a.every)) + [n - 1] * a.fps * 2
    painted = -1
    for k in ks:
        while painted < k:
            painted += 1
            paint(painted)
        mosaic = msum / np.maximum(mcnt, 1)
        fig = plt.figure(figsize=(12.8, 6.4), dpi=80, facecolor=BG)
        ax_img = fig.add_axes([0.02, 0.36, 0.36, 0.56])
        ax_sc = fig.add_axes([0.05, 0.07, 0.30, 0.22])
        ax_map = fig.add_axes([0.42, 0.04, 0.56, 0.90])
        im = imgs[k].astype(float)
        im = im.T if a.layout == "beam_range" else im
        if a.flip:
            im = im[:, ::-1]
        f = fan(im)
        ax_img.imshow(f, cmap="gray", vmin=0, vmax=200)
        ax_img.set_facecolor(BG)
        ax_img.axis("off")
        recent = [q for q in loop_at if k - a.every < q <= k]
        ax_img.set_title(f"imaging sonar  ·  keyframe {k}", color=FG, fontsize=13, loc="left")
        sc = sonar_context(im, (50, 4), "range_beam", 150)
        ax_sc.imshow(sc, cmap="magma", aspect="auto", origin="lower")
        ax_sc.set_xticks([]); ax_sc.set_yticks([])
        for sp in ax_sc.spines.values():
            sp.set_color(GREEN if recent else MUTED)
            sp.set_linewidth(3 if recent else 1)
        ax_sc.set_title("Sonar Context (range x bearing)", color=MUTED, fontsize=11, loc="left")
        if recent:
            ax_img.text(0.98, 0.02, f"LOOP  {recent[-1]} → {loop_at[recent[-1]][-1]}", transform=ax_img.transAxes,
                        ha="right", va="bottom", color=BG, fontsize=13, weight="bold",
                        bbox=dict(boxstyle="round,pad=0.35", fc=GREEN, ec="none"))

        ax_map.set_facecolor(BG)
        ax_map.imshow(np.clip(mosaic / gain, 0, 1) ** 0.7, cmap="bone", origin="lower", vmin=0, vmax=1,
                      extent=[gx[0] - res / 2, gx[-1] + res / 2, gy[0] - res / 2, gy[-1] + res / 2], interpolation="bilinear")
        if gt is not None:
            ax_map.plot(gt[: k + 1, 0], gt[: k + 1, 1], ":", c=FG, lw=1.0, alpha=0.5, label="ground truth (DGPS)")
        ax_map.plot(oxy[: k + 1, 0], oxy[: k + 1, 1], "--", c=GREY, lw=1.2, label="DISO odometry")
        for q, m in loops:
            if q <= k:
                ax_map.plot([xy[q, 0], xy[m, 0]], [xy[q, 1], xy[m, 1]], "-", c=GREEN, lw=0.8, alpha=0.8)
        ax_map.plot(xy[: k + 1, 0], xy[: k + 1, 1], "-", c=BLUE, lw=2.0, label="+ Sonar Context loops")
        ax_map.plot(*xy[k], "o", c=ORANGE, ms=7)
        ax_map.set_xlim(lo[0], hi[0]); ax_map.set_ylim(lo[1], hi[1]); ax_map.set_aspect("equal")
        ax_map.axis("off")
        nl = sum(1 for q, _ in loops if q <= k)
        ax_map.set_title(f"Aracati 2017  ·  {nl} loops", color=FG, fontsize=13, loc="left")
        leg = ax_map.legend(loc="lower right", facecolor=BG, edgecolor=MUTED, labelcolor=FG, fontsize=10)
        buf = io.BytesIO()
        fig.savefig(buf, format="png", facecolor=BG)
        plt.close(fig)
        frames.append(Image.open(buf).convert("RGB"))
    pal = frames[len(frames) // 2].quantize(colors=128, method=Image.Quantize.MEDIANCUT)
    q = [fr.quantize(palette=pal, dither=Image.Dither.NONE) for fr in frames]
    q[0].save(a.out, save_all=True, append_images=q[1:], duration=int(1000 / a.fps), loop=0, optimize=True)
    print(f"{a.out}: {len(frames)} frames, {Path(a.out).stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
