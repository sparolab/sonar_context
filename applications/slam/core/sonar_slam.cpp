#include "sonar_slam.hpp"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <unordered_map>

namespace sonar_context {
namespace fs = std::filesystem;
using gtsam::Pose2;

namespace {

struct Grid {
  double cell;
  std::unordered_map<long long, std::vector<int>> cells;
  const std::vector<Eigen::Vector2d>* pts;
  static long long key(long long x, long long y) { return (x << 32) ^ (y & 0xffffffffLL); }
  Grid(const std::vector<Eigen::Vector2d>& p, double c) : cell(c), pts(&p) {
    for (int i = 0; i < static_cast<int>(p.size()); ++i)
      cells[key(std::floor(p[i].x() / c), std::floor(p[i].y() / c))].push_back(i);
  }
  // nearest within cell size
  int nearest(const Eigen::Vector2d& q, double& d2) const {
    const long long cx = std::floor(q.x() / cell), cy = std::floor(q.y() / cell);
    int best = -1;
    d2 = cell * cell;
    for (long long dx = -1; dx <= 1; ++dx)
      for (long long dy = -1; dy <= 1; ++dy) {
        auto it = cells.find(key(cx + dx, cy + dy));
        if (it == cells.end()) continue;
        for (int i : it->second) {
          const double d = ((*pts)[i] - q).squaredNorm();
          if (d < d2) d2 = d, best = i;
        }
      }
    return best;
  }
};

Eigen::Vector2d apply(const Pose2& T, const Eigen::Vector2d& p) {
  const double c = std::cos(T.theta()), s = std::sin(T.theta());
  return {c * p.x() - s * p.y() + T.x(), s * p.x() + c * p.y() + T.y()};
}

}  // namespace

std::vector<Eigen::Vector2d> voxel_downsample(const std::vector<Eigen::Vector2d>& pts, double voxel) {
  std::unordered_map<long long, std::pair<Eigen::Vector2d, int>> acc;
  for (const auto& p : pts) {
    auto& a = acc[Grid::key(std::floor(p.x() / voxel), std::floor(p.y() / voxel))];
    a.first += p, a.second += 1;
  }
  std::vector<Eigen::Vector2d> out;
  out.reserve(acc.size());
  for (auto& [k, a] : acc) out.push_back(a.first / a.second);
  return out;
}

std::vector<double> loop_yaws(double descriptor_yaw, double step_deg, double refine_deg) {
  std::vector<double> y;
  for (double d = -180; d < 180; d += step_deg) y.push_back(d * M_PI / 180);
  for (double d = -refine_deg; d <= refine_deg + 1e-9; d += 3) {
    y.push_back(descriptor_yaw + d * M_PI / 180);
    y.push_back(-descriptor_yaw + d * M_PI / 180);  // the sign of the shift depends on the beam order
  }
  return y;
}

SonarSlam::SonarSlam(SlamOptions options) : opt_(options), manager_(options.loop), dense_(options.dense) {
  gtsam::ISAM2Params p;
  p.relinearizeThreshold = 0.01;
  p.relinearizeSkip = 1;
  isam_ = gtsam::ISAM2(p);
}

bool SonarSlam::add(double time, const Pose2& odom, const Image& polar) {
  if (!keyframes_.empty()) {
    const Pose2 d = keyframes_.back().odom.between(odom);
    if (d.translation().norm() < opt_.keyframe_distance &&
        std::abs(d.theta()) < opt_.keyframe_yaw_deg * M_PI / 180.0)
      return false;
  }
  Keyframe kf;
  kf.time = time;
  kf.odom = odom;
  const auto& lo = opt_.loop;
  kf.context = make_sonar_context(polar, lo.patch_range, lo.patch_beam, Layout::RangeBeam, lo.skip_range);
  opt_.geometry.beams = static_cast<int>(polar.cols());
  opt_.geometry.patch_beam = lo.patch_beam;
  kf.points = voxel_downsample(polar_points(polar, opt_.geometry, opt_.keep_percentile, opt_.min_range), opt_.voxel);
  kf.cartesian = dense_.cartesian(polar);
  keyframes_.push_back(std::move(kf));
  const int k = static_cast<int>(keyframes_.size()) - 1;
  manager_.add(keyframes_.back().context);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values init;
  const double ys = opt_.odom_sigma_yaw_deg * M_PI / 180.0;
  if (k == 0) {
    graph.add(gtsam::PriorFactor<Pose2>(0, odom, gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1e-3, 1e-3, 1e-4))));
    init.insert(0, odom);
  } else {
    const Pose2 d = keyframes_[k - 1].odom.between(odom);
    graph.add(gtsam::BetweenFactor<Pose2>(k - 1, k, d,
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(opt_.odom_sigma_xy, opt_.odom_sigma_xy, ys))));
    init.insert(k, estimate_.at<Pose2>(k - 1) * d);
  }
  last_loop_.reset();
  if (auto lc = close_loop(k)) {
    const double ls = opt_.loop_sigma_yaw_deg * M_PI / 180.0;
    auto noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(opt_.loop_sigma_xy, opt_.loop_sigma_xy, ls)));
    graph.add(gtsam::BetweenFactor<Pose2>(lc->match, k, lc->relative, noise));
    loops_.push_back(*lc);
    last_loop_ = lc;
  }
  isam_.update(graph, init);
  if (last_loop_) for (int i = 0; i < 3; ++i) isam_.update();
  estimate_ = isam_.calculateEstimate();
  return true;
}

std::optional<LoopClosure> SonarSlam::close_loop(int q) {
  auto loop = manager_.detect_loop(q);
  if (!loop) return std::nullopt;
  const int m = loop->match;
  const double yaw = opt_.geometry.yaw_of_shift(loop->column_shift);
  const DenseMatch dm = dense_.match(keyframes_[m].cartesian, keyframes_[q].cartesian,
                                     loop_yaws(yaw, opt_.yaw_step_deg, opt_.yaw_refine_deg));
  const bool ok = dm.peak >= opt_.min_peak;
  if (opt_.verbose)
    std::printf("candidate %d -> %d: descriptor %.3f (shift %d), peak %.3f at (%.1f, %.1f, %.0f deg) %s\n", q, m,
                loop->distance, loop->column_shift, dm.peak, dm.x, dm.y, dm.yaw * 180 / M_PI, ok ? "ACCEPT" : "reject");
  if (!ok) return std::nullopt;
  LoopClosure lc;
  lc.query = q, lc.match = m, lc.descriptor_distance = loop->distance, lc.peak = dm.peak;
  lc.relative = Pose2(dm.x, dm.y, dm.yaw);
  return lc;
}

std::vector<Pose2> SonarSlam::poses() const {
  std::vector<Pose2> out;
  for (int i = 0; i < static_cast<int>(keyframes_.size()); ++i) out.push_back(estimate_.at<Pose2>(i));
  return out;
}

void SonarSlam::save(const std::string& dir) const {
  fs::create_directories(fs::path(dir) / "points");
  fs::create_directories(fs::path(dir) / "cartesian");
  auto kitti = [](std::ofstream& f, const Pose2& p) {
    const double c = std::cos(p.theta()), s = std::sin(p.theta());
    f << c << ' ' << -s << " 0 " << p.x() << ' ' << s << ' ' << c << " 0 " << p.y() << " 0 0 1 0\n";
  };
  std::ofstream fp(fs::path(dir) / "poses.txt"), fo(fs::path(dir) / "odom_poses.txt"), ft(fs::path(dir) / "times.txt");
  fp.precision(9), fo.precision(9), ft.precision(16);
  const auto P = poses();
  for (size_t i = 0; i < keyframes_.size(); ++i) {
    kitti(fp, P[i]);
    kitti(fo, keyframes_[i].odom);
    ft << keyframes_[i].time << '\n';
    cv::Mat c16;
    keyframes_[i].cartesian.convertTo(c16, CV_16U, 256.0);  // intensity x 256
    cv::imwrite((fs::path(dir) / "cartesian" / (std::to_string(i) + ".png")).string(), c16);
    std::ofstream pb(fs::path(dir) / "points" / (std::to_string(i) + ".bin"), std::ios::binary);
    for (const auto& p : keyframes_[i].points) {
      const float v[4] = {static_cast<float>(p.x()), static_cast<float>(p.y()), 0.f, 0.f};
      pb.write(reinterpret_cast<const char*>(v), sizeof(v));
    }
  }
  std::ofstream fl(fs::path(dir) / "loops.csv");
  fl << "query,match,descriptor_distance,peak,dx,dy,dyaw\n";
  for (const auto& l : loops_)
    fl << l.query << ',' << l.match << ',' << l.descriptor_distance << ',' << l.peak << ','
       << l.relative.x() << ',' << l.relative.y() << ',' << l.relative.theta() << '\n';
  if (!keyframes_.empty()) {
    const auto& c0 = keyframes_[0].context;
    std::ofstream fc(fs::path(dir) / "contexts.f64", std::ios::binary);
    for (const auto& k : keyframes_) fc.write(reinterpret_cast<const char*>(k.context.data()), sizeof(double) * k.context.size());
    std::ofstream(fs::path(dir) / "contexts.shape") << keyframes_.size() << ' ' << c0.rows() << ' ' << c0.cols() << '\n';
  }
}

}  // namespace sonar_context
