# Sonar Context SLAM

Keyframes from any odometry (every 1 m or 15°), a Sonar Context per keyframe, loops from the polar-key nearest
neighbour accepted by the column distance (< 0.2), each loop verified by **dense phase correlation** of the two
sonar frames and closed in a Pose2 graph with iSAM2. Odometry by default: [DISO](https://github.com/SenseRoboticsLab/DISO)
(`thirdparty/DISO`, ROS 1), whose motion prior is the dead reckoning of [Aracati2017_DISO](https://github.com/SenseRoboticsLab/Aracati2017_DISO).

```
core/          ROS-free back-end (SonarSlam, DenseRegistration): used by both nodes and the tools
ros1/          sonar_context_slam (Noetic): node, aracati2017.launch (bag + odom + DISO + SLAM), config
ros2/          sonar_context_slam (Humble): the same node on ROS 2, aracati2017.launch.py (plays a ROS 2 bag)
tools/         replay.cpp (no ROS: images + poses -> session), evaluate.py (APE against Aracati's /pose_gt)
tests/         test_dense.cpp (simulated views)
```

## ROS 1 (Noetic): DISO + SLAM

```bash
docker build -f docker/Dockerfile.slam -t sonar-context-slam:noetic .
docker run --rm -v <Aracati>:/data:ro -v $PWD/out:/out sonar-context-slam:noetic \
    roslaunch sonar_context_slam aracati2017.launch bag:=/data/ARACATI_2017_8bits_full.bag save_dir:=/out/slam \
    record:=/out/aracati_diso.bag [rviz:=true]
```

Without Docker: clone this repository with its submodules into a catkin workspace (`src/`), remove (or `CATKIN_IGNORE`)
`applications/slam/ros2` (same package name), install DISO's dependencies (`thirdparty/DISO/docker`) and GTSAM, then
`catkin_make`.

Topics: `/sonar_context/path`, `/sonar_context/odom_path`, `/sonar_context/loops` (markers), `/sonar_context/map`
(sonar points on the optimized poses). On exit the node writes a session to `save_dir`: `poses.txt`, `odom_poses.txt`
(KITTI 3×4), `times.txt`, `loops.csv`, `contexts.f64` (+ `.shape`), `points/`, `cartesian/`.

## ROS 2 (Humble)

DISO runs on ROS 1 only; record its poses with the ROS 1 launch (`record:=`), convert the bag and replay it:

```bash
pip install rosbags && rosbags-convert --src out/aracati_diso.bag --dst out/aracati_diso_ros2 --dst-version 8
docker build -f docker/Dockerfile.slam-ros2 -t sonar-context-slam:humble .
docker run --rm -v $PWD/out:/out sonar-context-slam:humble \
    ros2 launch sonar_context_slam aracati2017.launch.py bag:=/out/aracati_diso_ros2 save_dir:=/out/slam_ros2
```

Any other odometry: set `pose_topic` (`geometry_msgs/PoseStamped`, or `pose_type: odometry` for `nav_msgs/Odometry`),
`image_topic` and `image_layout` (`fan`, `range_beam`, `beam_range`).

## Without ROS

```bash
sonar_context_replay images.u8 <n> <rows> <cols> beam_range poses.txt out/ --skip-range 150 --patch-range 50 --flip
python tools/evaluate.py out/ --gt pose_gt.txt --plot out/trajectory.png
```

## Parameters (`ros1/config/aracati2017.yaml`)

| | |
| --- | --- |
| `skip_range`, `patch_range`, `patch_beam` | Sonar Context: near-field bins dropped (75 of 750 = 5 m), patch size (25 range bins × 4 beams) |
| `col_threshold` | column distance to accept a candidate (0.2) |
| `min_peak` | dense-correlation peak to accept a loop (0.10) |
| `flip_frame` | odometry in a z-down frame (DISO): flip it and the beams to z-up |
| `keyframe_distance`, `keyframe_yaw_deg` | 1 m, 15° |

## Aracati 2017

APE against the DGPS ground truth (`evo_ape -a`, scale-corrected in parentheses): DISO odometry 6.72 m (6.20 m),
with Sonar Context loops **1.86 m (1.17 m)**.
