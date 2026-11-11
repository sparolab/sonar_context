# Multi-session SLAM

Two sessions saved by the SLAM (`../slam`), each in its own frame, aligned through the places they share:

1. for each keyframe of B, the polar-key nearest neighbours in A, accepted by the column distance (`--threshold`);
2. each candidate verified by dense phase correlation of the two sonar frames (`--min-peak`);
3. every verified loop implies a transform from B to A; the largest set that agrees on one (3 m, 5°) is kept;
4. both sessions' trajectories and the agreeing loops in one pose graph (Levenberg-Marquardt).

```bash
sonar_context_align_sessions session_a/ session_b/ merged/ --threshold 0.25 [--candidates 5] [--min-peak 0.1]
```

Writes `merged/poses_a.txt`, `poses_b.txt` (both in A's frame), `inter_loops.csv`, `transform.txt`. Built with the
SLAM (ROS 1 or ROS 2 image: `/ws/devel/lib/sonar_context_slam/` or `/ws/install/sonar_context_slam/lib/...`).

Aracati 2017 split in two sessions (354 + 354 keyframes, B in its own frame): 47 inter-session loops, 37 agree on
one transform; the two sessions together come to 2.51 m APE against the ground truth (17.88 m with B left unaligned,
1.96 m and 2.33 m for each session alone).
