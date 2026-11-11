"""Two sonar robots from Aracati 2017, for Commerge (multi-robot map merging).

    python make_sessions.py --images <Aracati>/polar_image.zip --poses <Aracati>/pose_gt.txt --out runs/aracati
    commerge runs/aracati/robot_a runs/aracati/robot_b --affinity runs/aracati/affinity.npy \
        --threshold 0.9 --min-area 50 --registration pairs --policy all --out runs/aracati/merged \
        --gt-a runs/aracati/robot_a/gt_poses.txt --gt-b runs/aracati/robot_b/gt_poses.txt

Robot A is the first half of the run and robot B the second half, keyframes every 1 m; robot B's poses are
moved into its own frame (yaw 60 deg, (30, -20, 0) m) so that Commerge has to find the transform. The
affinity between the robots is the Sonar Context column similarity (sonar_context.column_similarity_matrix).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from sonar_context import column_similarity_matrix, sonar_context
from sonar_context.datasets import image_sequence, read_aracati_poses, stamps_from_names


def interpolate_poses(t, t_ref, T_ref):
    tc = np.clip(t, t_ref[0], t_ref[-1])
    T = np.tile(np.eye(4), (len(t), 1, 1))
    for k in range(3):
        T[:, k, 3] = np.interp(tc, t_ref, T_ref[:, k, 3])
    T[:, :3, :3] = Slerp(t_ref, Rotation.from_matrix(T_ref[:, :3, :3]))(tc).as_matrix()
    return T


def keyframes(T, step):
    keep = [0]
    for i in range(1, len(T)):
        if np.linalg.norm(T[i, :3, 3] - T[keep[-1], :3, 3]) >= step:
            keep.append(i)
    return np.array(keep)


def write_session(root: Path, gt, move, times, frames):
    root.mkdir(parents=True, exist_ok=True)
    flat = lambda P: P[:, :3, :4].reshape(len(P), 12)
    np.savetxt(root / "gt_poses.txt", flat(gt), fmt="%.6f")
    np.savetxt(root / "odom_poses.txt", flat(move @ gt), fmt="%.6f")
    np.savetxt(root / "times.txt", times, fmt="%.6f")
    (root / "frames.txt").write_text("\n".join(frames) + "\n")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--images", required=True, help="Aracati polar_image.zip (or a folder of <stamp>.png)")
    p.add_argument("--poses", required=True, help="pose_gt.txt: stamp bagtime x y z qx qy qz qw")
    p.add_argument("--out", required=True)
    p.add_argument("--step", type=float, default=1.0, help="keyframe spacing (m)")
    p.add_argument("--split", type=float, default=0.5, help="share of the keyframes in robot A")
    p.add_argument("--yaw", type=float, default=60.0, help="robot B's frame: yaw (deg)")
    p.add_argument("--shift", type=float, nargs=3, default=[30.0, -20.0, 0.0], help="robot B's frame: shift (m)")
    a = p.parse_args()

    names, load = image_sequence(a.images)
    t = stamps_from_names(names)
    T = interpolate_poses(t, *read_aracati_poses(a.poses))
    k = keyframes(T, a.step)
    print(f"{len(names)} sonar images, {len(k)} keyframes every {a.step} m")
    SC = np.stack([sonar_context(load(names[i]), (4, 4), layout="beam_range") for i in k])
    s = int(len(k) * a.split)
    ka, kb = k[:s], k[s:]
    S = column_similarity_matrix(SC[:s], SC[s:]).astype(np.float32)

    move = np.eye(4)
    move[:3, :3] = Rotation.from_euler("z", a.yaw, degrees=True).as_matrix()
    move[:3, 3] = a.shift
    out = Path(a.out)
    write_session(out / "robot_a", T[ka], np.eye(4), t[ka], [Path(names[i]).name for i in ka])
    write_session(out / "robot_b", T[kb], move, t[kb], [Path(names[i]).name for i in kb])
    np.save(out / "affinity.npy", S)
    d = np.linalg.norm(T[ka][:, None, :2, 3] - T[kb][None, :, :2, 3], axis=2)
    print(f"affinity {S.shape}: revisits (< 3 m) {S[d < 3].mean():.3f}, others {S[d >= 3].mean():.3f}")
    print(f"-> {out}: robot_a {len(ka)}, robot_b {len(kb)} keyframes")


if __name__ == "__main__":
    main()
