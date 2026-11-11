<div align="center">
  <h1>Sonar Context</h1>
  <a href="https://github.com/sparolab/sonar_context/tree/main/cpp"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
  <a href="https://github.com/sparolab/sonar_context/tree/main/python"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
  <a href="https://pypi.org/project/sonar-context/"><img src="https://img.shields.io/pypi/v/sonar-context?logo=pypi&logoColor=white&label=PyPI" /></a>
  <a href="https://github.com/sparolab/sonar_context/tree/main/applications/slam/ros1"><img src="https://img.shields.io/badge/ROS-Noetic-blue" /></a>
  <a href="https://github.com/sparolab/sonar_context/tree/main/applications/slam/ros2"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
  <a href="#docker"><img src="https://img.shields.io/badge/Ubuntu-E95420?logo=ubuntu&logoColor=white" /></a>
  <a href="#docker"><img src="https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white" /></a>
  <br />
  <a href="https://sites.google.com/view/sonar-context"><img src="https://img.shields.io/badge/Project-Page-blue" /></a>
  <a href="https://ieeexplore.ieee.org/document/10161518"><img src="https://img.shields.io/badge/Paper-ICRA%202023-yellow" /></a>
  <a href="https://www.youtube.com/watch?v=JRD_xuqtHZU"><img src="https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white" /></a>
  <a href="#robot-run-with-your-agent"><img src="https://img.shields.io/badge/Claude%20Code-ready-D97757?logo=claude&logoColor=white" alt="Claude Code ready" /></a>
  <a href="#robot-run-with-your-agent"><img src="https://img.shields.io/badge/Codex-ready-000000?logo=data:image/svg%2bxml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNCAyNCI+PHJlY3QgeD0iMS41IiB5PSIzIiB3aWR0aD0iMjEiIGhlaWdodD0iMTgiIHJ4PSIzIiBmaWxsPSJub25lIiBzdHJva2U9IndoaXRlIiBzdHJva2Utd2lkdGg9IjIiLz48cGF0aCBkPSJNNiA5bDMuNSAzTDYgMTUiIGZpbGw9Im5vbmUiIHN0cm9rZT0id2hpdGUiIHN0cm9rZS13aWR0aD0iMiIgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIiBzdHJva2UtbGluZWpvaW49InJvdW5kIi8+PHBhdGggZD0iTTEyIDE1LjVoNiIgc3Ryb2tlPSJ3aGl0ZSIgc3Ryb2tlLXdpZHRoPSIyIiBzdHJva2UtbGluZWNhcD0icm91bmQiLz48L3N2Zz4=" alt="Codex ready" /></a>
  <a href="https://github.com/sparolab/sonar_context/actions/workflows/ci.yml"><img src="https://github.com/sparolab/sonar_context/actions/workflows/ci.yml/badge.svg?branch=main" alt="CI" /></a>
  <br />
  <a href="https://pypi.org/project/sonar-context/"><img src="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=14&duration=2500&pause=800&center=true&vCenter=true&width=280&height=24&lines=%24+pip+install+sonar-context" alt="pip install sonar-context" /></a>
  <br />
  <img src="https://raw.githubusercontent.com/sparolab/sonar_context/main/assets/sonar_context_slam.gif" alt="Sonar Context SLAM on Aracati 2017: each forward-looking sonar keyframe gets a Sonar Context; revisits are found from it, verified by dense correlation and closed in a pose graph over DISO's sonar odometry, and the sonar mosaic of the marina comes together" width="95%" />
  <p>
    <strong>[ICRA 2023] Robust Imaging Sonar-based Place Recognition and Localization in Underwater Environments</strong>
  </p>
</div>

______________________________________________________________________

Underwater there is no GPS and little light; a forward-looking imaging sonar sees 50 m ahead through murky water, but its images are noisy and change with the viewpoint. Sonar Context turns one sonar image into a small, robust place descriptor, so the robot knows it has been here before:

- :ocean: **Sonar Context**: the polar sonar image pooled into patches (range rings × bearing sectors), plus a **polar key** (one value per ring) that does not change with the heading.
- :mag: **Coarse to fine**: the polar key finds candidates in a k-d tree; the Sonar Context compares them over every bearing shift (a rotation) and range shift.
- :zap: **Lightweight**: NumPy or C++17 with Eigen, no GPU and no training.
- :repeat: **Online loop closing**: [DISO](https://github.com/SenseRoboticsLab/DISO) sonar odometry + Sonar Context loops, verified by dense phase correlation, in an iSAM2 pose graph, on ROS 1 and ROS 2.
- :world_map: **Multi-session SLAM**: sessions recorded apart are aligned into one map, LT-SLAM style ([guide](applications/multi-session-slam)).
- :busts_in_silhouette: **Multi-robot**: map merging from a few exchanged keyframes ([Commerge](https://github.com/sparolab/Commerge)).

## :floppy_disk: Datasets

The examples use **[Aracati 2017](https://github.com/matheusbg8/aracati2017)** (an ROV with a BlueView P900-130 in a marina): the bag (`/son`, `/pose_gt`, `/cmd_vel`) for the SLAM and the polar images (`polar_image.zip`, stored beams × range: `layout="beam_range"`) for the library. DISO's [Aracati2017_DISO](https://github.com/SenseRoboticsLab/Aracati2017_DISO) (a submodule) adds the dead-reckoning prior. Also: [HoloOcean](https://holoocean.readthedocs.io/) simulation ([Google Drive](https://drive.google.com/drive/folders/1tPEZzdvOCRTILfkeq2X0KLuOM3v53kQL?usp=sharing)); KRISO data is not public.

______________________________________________________________________

## :package: Installation

```bash
pip install sonar-context                 # the library (NumPy, SciPy, Pillow)
```

### Docker

```bash
git clone --recursive https://github.com/sparolab/sonar_context.git && cd sonar_context
docker build -f docker/Dockerfile.slam -t sonar-context-slam:noetic .          # ROS 1: DISO + SLAM
docker build -f docker/Dockerfile.slam-ros2 -t sonar-context-slam:humble .     # ROS 2: SLAM
```

### From source

<details open>
<summary><strong>C++ library</strong></summary>

```bash
sudo apt install libeigen3-dev
cmake -S cpp -B cpp/build && cmake --build cpp/build -j && ctest --test-dir cpp/build
```

In your project, `add_subdirectory(sonar_context/cpp)` and link `sonar_context::sonar_context`. Same distances and shifts as Python (checked in CI).

</details>

<details open>
<summary><strong>ROS 1 Noetic / ROS 2 Humble</strong></summary>

GTSAM and OpenCV, then [applications/slam](applications/slam) in a catkin or colcon workspace (keep only the ROS version you build; ROS 1 also needs DISO's dependencies). The Dockerfiles list every step.

</details>

______________________________________________________________________

## :rocket: Run

```bash
# ROS 1: the bag + DISO + Sonar Context SLAM (record:= also writes a bag for ROS 2)
docker run --rm -v <Aracati>:/data:ro -v $PWD/out:/out sonar-context-slam:noetic \
    roslaunch sonar_context_slam aracati2017.launch bag:=/data/ARACATI_2017_8bits_full.bag save_dir:=/out/slam record:=/out/aracati_diso.bag
# ROS 2: the same SLAM on the recorded DISO poses (rosbags-convert the bag first)
docker run --rm -v $PWD/out:/out sonar-context-slam:humble \
    ros2 launch sonar_context_slam aracati2017.launch.py bag:=/out/aracati_diso_ros2 save_dir:=/out/slam_ros2
# Your own odometry: set pose_topic (PoseStamped or Odometry), image_topic and image_layout
```

Keyframes every 1 m; Sonar Context proposes revisits; each is verified by **dense phase correlation** of the two sonar frames and closed with iSAM2. On **Aracati 2017** (620 m in a marina), the loops bring DISO's sonar odometry from 4.54 m to **1.91 m** APE against the DGPS ground truth. Results: [applications/slam](applications/slam#aracati-2017).

Without ROS:

```python
import sonar_context as sc
desc = sc.sonar_context(image, patch=(4, 4))             # rows = range bins, columns = beams
dist, shift = sc.column_distance(desc_a, desc_b)         # 1 - similarity, best bearing shift
m = sc.SonarContextManager(col_threshold=0.2, exclude_recent=30)
loop = m.update(image)                                   # Loop(query, match, distance, column_shift, ...) or None
```

```bash
sonar-context describe polar_image.zip --layout beam_range --out seq.npz      # descriptors of a sequence
sonar-context loops seq.npz --out loops.csv --poses poses.txt               # online loop detection
sonar-context affinity a.npz b.npz --out affinity.npy                       # two sessions, for Commerge
```

______________________________________________________________________

## :toolbox: Tools

### Multi-session SLAM

LT-SLAM style ([LT-mapper](https://github.com/gisbi-kim/lt-mapper)), as in [RadLoc](https://github.com/sparolab/RadLoc) and [ReFeree](https://github.com/sparolab/ReFeree): sessions saved by the SLAM keep their own frames and get one anchor node each; inter-session Sonar Context loops, verified by dense correlation, connect them through the anchors. Aracati 2017 split in two sessions, each in its own frame, comes together at **2.52 m** APE. Results: [applications/multi-session-slam](applications/multi-session-slam).

```bash
sonar_context_align_sessions session_a/ session_b/ merged/ --threshold 0.25
```

See the [multi-session SLAM guide](applications/multi-session-slam).

### Multi-Robot Map Merging

Each robot keeps its Sonar Contexts; [Commerge](https://github.com/sparolab/Commerge) (IJRR 2026) picks the few keyframes that carry the overlap, and only those are exchanged: on Aracati split into two robots, 4.5 % of the keyframes (2.6 KB of poses) merge the maps within 1.3 m.

<p align="center"><a href="https://github.com/sparolab/Commerge"><img src="applications/multi-robot-slam/commerge_sonar.gif" alt="Commerge on Aracati 2017 split into two sonar robots" width="95%" /></a></p>

See [applications/multi-robot-slam](applications/multi-robot-slam).

______________________________________________________________________

## :robot: Run with your agent

Open [Claude Code](https://claude.com/claude-code) or [Codex](https://openai.com/codex/) in an empty folder and paste:

```text
Clone https://github.com/sparolab/sonar_context, read its AGENTS.md, and follow it to build Sonar Context and run its SLAM on Aracati 2017 on this machine. Ask me what you need along the way.
```

The agent follows [`AGENTS.md`](AGENTS.md): it builds the Docker images, runs the SLAM on the bag and checks the result.

______________________________________________________________________

## :wrench: What changed in v2

The matching of the ICRA 2023 code had bugs, fixed in both libraries (the old code is at [v1.0.0](https://github.com/sparolab/sonar_context/tree/v1.0.0)):

- the best bearing shift was read one entry off (`argmax + 1`), and could index past the array;
- shift 0 (the same heading) was never tested;
- the row search compared unshifted rows, so it had no effect;
- a shift with no non-empty overlap divided by zero.

New: `skip_range` drops the bright near field (on Aracati, 5 m), the polar image layout is explicit (`range_beam` / `beam_range`: Aracati's polar images are stored beams × range), and the descriptors of two sessions compare in one call (`column_similarity_matrix`, FFT-based).

______________________________________________________________________

## Citation

If you use Sonar Context in your research, please cite our [paper](https://ieeexplore.ieee.org/document/10161518).

<details>
  <summary><strong>See bibtex</strong></summary>

```bibtex
@inproceedings{kim2023robust,
  title={Robust Imaging Sonar-based Place Recognition and Localization in Underwater Environments},
  author={Kim, Hogyun and Kang, Gilhwan and Jeong, Seokhwan and Ma, Seungjun and Cho, Younggun},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={1083--1089},
  year={2023},
  organization={IEEE}
}
```

</details>

## Contributing

Issues and pull requests are welcome.

<a href="https://github.com/sparolab/sonar_context/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=sparolab/sonar_context" />
</a>

## Acknowledgements

Sonar Context builds on [Scan Context](https://github.com/gisbi-kim/scancontext) (Kim and Kim, IROS 2018). The SLAM uses [DISO](https://github.com/SenseRoboticsLab/DISO) (Xu et al., ICRA 2024, GPLv3, a submodule) for odometry and [GTSAM](https://github.com/borglab/gtsam). Data: [Aracati 2017](https://github.com/matheusbg8/aracati2017) (dos Santos et al., RA-L 2022) and [HoloOcean](https://github.com/byu-holoocean/HoloOcean) (Potokar et al., ICRA 2022); multi-session underwater SLAM: Jang et al., ICRA 2021. Related: [SOLiD](https://github.com/sparolab/SOLiD), [Commerge](https://github.com/sparolab/Commerge), [ReFeree](https://github.com/sparolab/ReFeree), [RadLoc](https://github.com/sparolab/RadLoc).

## Special Thanks

We sincerely thank **[Giseop Kim](https://gisbi-kim.github.io/)** for open-sourcing [Scan Context](https://github.com/gisbi-kim/scancontext), which Sonar Context builds on, and [LT-mapper](https://github.com/gisbi-kim/lt-mapper), whose multi-session design our multi-session SLAM follows.

## License

BSD 3-Clause for academic use; for any commercial purpose, please contact the authors (hg.kim@inha.edu). DISO (`thirdparty/DISO`) is GPLv3 and is not part of this license.
