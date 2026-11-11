// Multi-session sonar SLAM: align session B to session A (both written by the SLAM, ../slam) through
// inter-session Sonar Context loops, verified by dense phase correlation, a consensus transform, and one joint
// pose graph.
//
//   sonar_context_align_sessions <session_a> <session_b> <out_dir> [--threshold 0.2] [--candidates 5] [--min-peak 0.1]
//
// Writes <out_dir>/poses_a.txt, poses_b.txt (both in A's frame, KITTI 3x4), inter_loops.csv, transform.txt.
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include <opencv2/imgcodecs.hpp>

#include "sonar_slam.hpp"

namespace fs = std::filesystem;
using gtsam::Pose2;
using namespace sonar_context;

struct Session {
  std::vector<Pose2> poses;
  std::vector<Descriptor> contexts;
  std::vector<cv::Mat> cartesian;
};

static Session load(const fs::path& d) {
  Session s;
  std::ifstream fp(d / "poses.txt");
  std::string line;
  while (std::getline(fp, line)) {
    std::istringstream is(line);
    double m[12];
    for (double& v : m) is >> v;
    s.poses.emplace_back(m[3], m[7], std::atan2(m[4], m[0]));
  }
  int n, r, c;
  std::ifstream(d / "contexts.shape") >> n >> r >> c;
  std::ifstream fc(d / "contexts.f64", std::ios::binary);
  for (int i = 0; i < n; ++i) {
    Descriptor sc(r, c);
    fc.read(reinterpret_cast<char*>(sc.data()), sizeof(double) * r * c);
    s.contexts.push_back(sc);
    cv::Mat c16 = cv::imread((d / "cartesian" / (std::to_string(i) + ".png")).string(), cv::IMREAD_UNCHANGED), c;
    if (c16.empty()) throw std::runtime_error(d.string() + ": missing cartesian/" + std::to_string(i) + ".png");
    c16.convertTo(c, CV_32F, 1.0 / 256);
    s.cartesian.push_back(c);
  }
  if (static_cast<int>(s.poses.size()) != n) throw std::runtime_error(d.string() + ": poses and contexts differ");
  return s;
}

static void write_kitti(const fs::path& f, const std::vector<Pose2>& P) {
  std::ofstream o(f);
  o.precision(9);
  for (const auto& p : P) {
    const double c = std::cos(p.theta()), s = std::sin(p.theta());
    o << c << ' ' << -s << " 0 " << p.x() << ' ' << s << ' ' << c << " 0 " << p.y() << " 0 0 1 0\n";
  }
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: sonar_context_align_sessions <session_a> <session_b> <out_dir> [--threshold 0.2] "
                 "[--candidates 5] [--min-peak 0.1] [--fov 130] [--patch-beam 4] [--beams 300]\n";
    return 2;
  }
  double threshold = 0.2, fov = 130, min_peak = 0.10;
  int candidates = 5, patch_beam = 4, beams = 300;
  for (int i = 4; i + 1 < argc; i += 2) {
    std::string k = argv[i];
    if (k == "--threshold") threshold = std::stod(argv[i + 1]);
    else if (k == "--candidates") candidates = std::stoi(argv[i + 1]);
    else if (k == "--min-peak") min_peak = std::stod(argv[i + 1]);
    else if (k == "--fov") fov = std::stod(argv[i + 1]);
    else if (k == "--patch-beam") patch_beam = std::stoi(argv[i + 1]);
    else if (k == "--beams") beams = std::stoi(argv[i + 1]);
  }
  const Session A = load(argv[1]), B = load(argv[2]);
  const fs::path out = argv[3];
  fs::create_directories(out);
  std::printf("A: %zu keyframes, B: %zu keyframes\n", A.poses.size(), B.poses.size());

  // 1. candidates: polar-key nearest neighbours in A for each keyframe of B, accepted by the column distance
  std::vector<Eigen::VectorXd> keysA;
  for (const auto& c : A.contexts) keysA.push_back(polar_key(c));
  SonarGeometry g{fov, 50.0, beams, patch_beam};
  struct Inter { int a, b; double dist; Pose2 rel; double peak; };
  DenseRegistration dense;
  std::vector<Inter> inter;
  for (int b = 0; b < static_cast<int>(B.poses.size()); ++b) {
    const Eigen::VectorXd kb = polar_key(B.contexts[b]);
    std::vector<std::pair<double, int>> d;
    for (int a = 0; a < static_cast<int>(keysA.size()); ++a) d.emplace_back((keysA[a] - kb).squaredNorm(), a);
    const int k = std::min<int>(candidates, d.size());
    std::partial_sort(d.begin(), d.begin() + k, d.end());
    int best = -1;
    Match bm;
    for (int i = 0; i < k; ++i) {
      Match m = column_distance(A.contexts[d[i].second], B.contexts[b]);
      if (best < 0 || m.distance < bm.distance) bm = m, best = d[i].second;
    }
    if (bm.distance >= threshold) continue;
    // 2. dense phase correlation of B's keyframe against A's
    const DenseMatch dm = dense.match(A.cartesian[best], B.cartesian[b], loop_yaws(g.yaw_of_shift(bm.shift), 6.0, 12.0));
    if (dm.peak < min_peak) continue;
    inter.push_back({best, b, bm.distance, Pose2(dm.x, dm.y, dm.yaw), dm.peak});
  }
  std::printf("inter-session loops verified by dense correlation: %zu\n", inter.size());
  if (inter.size() < 2) {
    std::fprintf(stderr, "too few inter-session loops to align\n");
    return 1;
  }

  // 3. consensus: each loop implies T_AB = P_A[a] * rel * P_B[b]^-1; keep the largest agreeing set
  std::vector<Pose2> T(inter.size());
  for (size_t i = 0; i < inter.size(); ++i)
    T[i] = A.poses[inter[i].a] * inter[i].rel * B.poses[inter[i].b].inverse();
  size_t best_i = 0, best_n = 0;
  std::vector<int> inl;
  for (size_t i = 0; i < T.size(); ++i) {
    std::vector<int> in;
    for (size_t j = 0; j < T.size(); ++j) {
      const Pose2 d = T[i].between(T[j]);
      if (d.translation().norm() < 3.0 && std::abs(d.theta()) < 5 * M_PI / 180) in.push_back(j);
    }
    if (in.size() > best_n) best_n = in.size(), best_i = i, inl = in;
  }
  std::printf("consensus: %zu of %zu loops agree\n", best_n, inter.size());

  // 4. joint graph: A's and B's odometry chains (from their own SLAM poses) + the agreeing loops
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values init;
  const int off = 1000000;
  auto chain = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.05, 0.05, 1 * M_PI / 180));
  auto loopn = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 2 * M_PI / 180)));
  graph.add(gtsam::PriorFactor<Pose2>(0, A.poses[0], gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1e-3, 1e-3, 1e-4))));
  for (size_t i = 0; i < A.poses.size(); ++i) {
    init.insert(i, A.poses[i]);
    if (i) graph.add(gtsam::BetweenFactor<Pose2>(i - 1, i, A.poses[i - 1].between(A.poses[i]), chain));
  }
  for (size_t i = 0; i < B.poses.size(); ++i) {
    init.insert(off + i, T[best_i] * B.poses[i]);
    if (i) graph.add(gtsam::BetweenFactor<Pose2>(off + i - 1, off + i, B.poses[i - 1].between(B.poses[i]), chain));
  }
  for (int j : inl) graph.add(gtsam::BetweenFactor<Pose2>(inter[j].a, off + inter[j].b, inter[j].rel, loopn));
  const gtsam::Values res = gtsam::LevenbergMarquardtOptimizer(graph, init).optimize();
  std::vector<Pose2> PA, PB;
  for (size_t i = 0; i < A.poses.size(); ++i) PA.push_back(res.at<Pose2>(i));
  for (size_t i = 0; i < B.poses.size(); ++i) PB.push_back(res.at<Pose2>(off + i));
  write_kitti(out / "poses_a.txt", PA);
  write_kitti(out / "poses_b.txt", PB);
  std::ofstream fl(out / "inter_loops.csv");
  fl << "a,b,descriptor_distance,peak,consensus\n";
  for (size_t i = 0; i < inter.size(); ++i)
    fl << inter[i].a << ',' << inter[i].b << ',' << inter[i].dist << ',' << inter[i].peak
       << ',' << (std::find(inl.begin(), inl.end(), static_cast<int>(i)) != inl.end()) << '\n';
  const Pose2 Tab = res.at<Pose2>(off) * B.poses[0].inverse();  // B's frame in A's
  std::ofstream(out / "transform.txt") << Tab.x() << ' ' << Tab.y() << ' ' << Tab.theta() << '\n';
  std::printf("B -> A: x %.2f m, y %.2f m, yaw %.2f deg -> %s\n", Tab.x(), Tab.y(), Tab.theta() * 180 / M_PI,
              out.c_str());
  return 0;
}
