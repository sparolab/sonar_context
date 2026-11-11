# Agent guide: Sonar Context

Imaging-sonar place recognition (ICRA 2023) as a library, with SLAM, multi-session and multi-robot applications.

## Layout

- `python/sonar_context/`: the package (`pip install sonar-context`; pyproject at the root, `package-dir = python`).
  `descriptor.py` (sonar_context, polar_key), `matching.py` (column/row distance, column_similarity_matrix),
  `manager.py` (online loops), `datasets.py`, `cli.py` (`sonar-context describe | loops | affinity`). Tests: `python/tests`.
- `cpp/`: the same in C++17 (Eigen only), target `sonar_context::sonar_context`; `ctest` and the parity test
  `python/tests/test_cpp.py` (needs `cpp/build/sonar_context_distances`).
- `applications/slam/{core,ros1,ros2,tools,tests}`: SLAM back-end (GTSAM, OpenCV) and its nodes; `docker/Dockerfile.slam`
  (Noetic, with DISO) and `docker/Dockerfile.slam-ros2` (Humble).
- `applications/multi-session-slam/align_sessions.cpp`, `applications/multi-robot-slam/` (Commerge).
- `thirdparty/DISO` (GPLv3), `thirdparty/Aracati2017_DISO` (MIT): submodules, not modified.

## Checks

```bash
pip install -e . pytest && pytest -q python/tests
cmake -S cpp -B cpp/build && cmake --build cpp/build -j && ctest --test-dir cpp/build
docker build -f docker/Dockerfile.slam -t sonar-context-slam:noetic .
docker run --rm sonar-context-slam:noetic /ws/devel/lib/sonar_context_slam/sonar_context_test_dense
```

## Conventions

- Polar images are (range bins, beams) inside the libraries; Aracati's `polar_image.zip` is (beams, range): `layout="beam_range"`.
- Beam 0 is on the sensor's right (y < 0); x forward, y left.
- DISO publishes z-down poses: `flip_frame: true` flips the poses and the beams.
- The ROS 1 and ROS 2 packages share the name `sonar_context_slam`: keep only one in a workspace.
- Do not redistribute datasets.
