"""Command line: describe a sonar sequence, detect its loops, or compare two sessions.

    sonar-context describe <images.zip|folder> --out seq.npz [--layout beam_range] [--patch 4 4]
    sonar-context loops seq.npz --out loops.csv [--poses poses.txt]
    sonar-context affinity a.npz b.npz --out affinity.npy
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from . import __version__
from .descriptor import LAYOUTS, polar_key, sonar_context
from .manager import SonarContextManager
from .matching import column_similarity_matrix


def _describe(a):
    from .datasets import image_sequence, interpolate_positions, keyframes_by_distance, read_aracati_poses, stamps_from_names
    names, load = image_sequence(a.images)
    if not names:
        sys.exit(f"no images in {a.images}")
    idx = np.arange(len(names))
    xyz = None
    if a.aracati_poses:
        t_ref, T_ref = read_aracati_poses(a.aracati_poses)
        xyz = interpolate_positions(stamps_from_names(names), t_ref, T_ref)
        if a.step > 0:
            idx = keyframes_by_distance(xyz, a.step)
    elif a.every > 1:
        idx = idx[:: a.every]
    try:
        from tqdm import tqdm
        it = tqdm(idx, desc="sonar context", mininterval=1)
    except ImportError:
        it = idx
    sc = np.stack([sonar_context(load(names[i]), a.patch, a.layout, a.skip_range) for i in it]).astype(np.float32)
    out = dict(sc=sc, pk=sc.mean(axis=2), names=np.array([Path(names[i]).name for i in idx]))
    if xyz is not None:
        out["xyz"] = xyz[idx]
    np.savez_compressed(a.out, **out)
    print(f"{a.out}: {len(idx)} frames, Sonar Context {sc.shape[1:]} (range rings x bearing sectors)")


def _positions(path):
    P = np.loadtxt(path, ndmin=2)
    if P.shape[1] == 12:  # KITTI 3x4
        return P[:, [3, 7, 11]]
    return P[:, :3] if P.shape[1] >= 3 else np.c_[P, np.zeros(len(P))]


def _loops(a):
    d = np.load(a.descriptors)
    m = SonarContextManager(a.col_threshold, a.row_threshold, a.exclude_recent, a.candidates)
    loops = []
    for sc, pk in zip(d["sc"], d["pk"]):
        m.add(sc, pk)
        lp = m.detect_loop()
        if lp is not None:
            loops.append(lp)
    xyz = _positions(a.poses) if a.poses else (d["xyz"] if "xyz" in d else None)
    with open(a.out, "w") as f:
        f.write("query,match,distance,column_shift,row_shift,by" + (",gap_m" if xyz is not None else "") + "\n")
        for lp in loops:
            row = f"{lp.query},{lp.match},{lp.distance:.5f},{lp.column_shift},{lp.row_shift},{lp.by}"
            if xyz is not None:
                row += f",{np.linalg.norm(xyz[lp.query, :2] - xyz[lp.match, :2]):.3f}"
            f.write(row + "\n")
    msg = f"{a.out}: {len(loops)} loops in {len(d['sc'])} frames"
    if xyz is not None and loops:
        ok = sum(np.linalg.norm(xyz[l.query, :2] - xyz[l.match, :2]) < a.true_within for l in loops)
        msg += f", {ok} within {a.true_within} m ({100 * ok / len(loops):.1f} % precision)"
    print(msg)


def _affinity(a):
    A, B = np.load(a.a)["sc"], np.load(a.b)["sc"]
    S = column_similarity_matrix(A, B, a.bound).astype(np.float32)
    np.save(a.out, S)
    print(f"{a.out}: affinity {S.shape}, range {S.min():.3f} .. {S.max():.3f}")


def main(argv=None):
    p = argparse.ArgumentParser(prog="sonar-context", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--version", action="version", version=f"sonar-context {__version__}")
    sub = p.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("describe", help="Sonar Contexts of an image sequence")
    d.add_argument("images", help="folder or .zip of polar sonar images")
    d.add_argument("--out", required=True, help=".npz with sc, pk, names (and xyz)")
    d.add_argument("--layout", choices=LAYOUTS, default="range_beam",
                   help="rows x columns of the stored image (Aracati 2017: beam_range)")
    d.add_argument("--patch", type=int, nargs=2, default=[4, 4], metavar=("RANGE", "BEAM"))
    d.add_argument("--skip-range", type=int, default=0, help="range bins of the near field to drop")
    d.add_argument("--every", type=int, default=1, help="keep every n-th image")
    d.add_argument("--aracati-poses", help="pose_gt.txt (stamp bagtime x y z qx qy qz qw): keyframes by distance")
    d.add_argument("--step", type=float, default=1.0, help="keyframe spacing with --aracati-poses (m)")
    d.set_defaults(fn=_describe)
    l = sub.add_parser("loops", help="online loop detection over a described sequence")
    l.add_argument("descriptors")
    l.add_argument("--out", required=True, help="loops.csv")
    l.add_argument("--poses", help="positions to score the loops (x y z per line, or KITTI 3x4)")
    l.add_argument("--col-threshold", type=float, default=0.045)
    l.add_argument("--row-threshold", type=float, default=0.072)
    l.add_argument("--exclude-recent", type=int, default=30)
    l.add_argument("--candidates", type=int, default=1, help="polar-key nearest neighbours re-ranked")
    l.add_argument("--true-within", type=float, default=3.0, help="a loop is correct within this distance (m)")
    l.set_defaults(fn=_loops)
    f = sub.add_parser("affinity", help="similarity matrix of two sessions (for multi-session SLAM, Commerge)")
    f.add_argument("a")
    f.add_argument("b")
    f.add_argument("--out", required=True)
    f.add_argument("--bound", type=float, default=0.5, help="largest bearing shift, as a share of the columns")
    f.set_defaults(fn=_affinity)
    a = p.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()
