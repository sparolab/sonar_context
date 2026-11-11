// Sonar Context SLAM back-end, ROS-free: keyframes from any odometry, loops from Sonar Context, verified by 2D
// ICP against a local submap, and a Pose2 graph optimized with iSAM2. The ROS 1 node (../ros1) and the offline
// runner (../tools) both drive this class.
#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <optional>
#include <string>
#include <vector>

#include "dense_registration.hpp"
#include "sonar_context/sonar_context.hpp"

namespace sonar_context {

struct SlamOptions {
  // keyframes
  double keyframe_distance = 1.0;  // m
  double keyframe_yaw_deg = 15.0;
  // descriptor and loop search
  ManagerOptions loop;
  SonarGeometry geometry;
  // points of a keyframe, for the map
  double keep_percentile = 99.5;
  double min_range = 8.0;  // m
  double voxel = 0.3;      // m
  // loop verification: dense phase correlation of the two frames
  DenseOptions dense;
  double min_peak = 0.10;        // normalized correlation peak to accept a loop
  double yaw_step_deg = 6.0;     // global yaw search step
  double yaw_refine_deg = 12.0;  // +- around the yaw of the column shift, in steps of 3 deg
  // noise (1 sigma)
  double odom_sigma_xy = 0.05, odom_sigma_yaw_deg = 1.0;  // per keyframe
  double loop_sigma_xy = 0.3, loop_sigma_yaw_deg = 2.0;
  bool verbose = false;  // print every loop candidate and its ICP result
};

struct Keyframe {
  double time = 0;
  gtsam::Pose2 odom;                   // odometry pose
  Descriptor context;                  // Sonar Context
  std::vector<Eigen::Vector2d> points; // strongest returns, keyframe frame
  cv::Mat cartesian;                   // dense image for loop verification
};

struct LoopClosure {
  int query = -1, match = -1;
  double descriptor_distance = 1, peak = 0;
  gtsam::Pose2 relative;  // pose of query in match's frame
};

// Yaws to search for a loop: a global grid and a finer one around the yaw of the descriptor's column shift.
std::vector<double> loop_yaws(double descriptor_yaw, double step_deg, double refine_deg);

std::vector<Eigen::Vector2d> voxel_downsample(const std::vector<Eigen::Vector2d>& pts, double voxel);

class SonarSlam {
 public:
  explicit SonarSlam(SlamOptions options = {});

  // One odometry pose with its polar sonar image (range bins x beams). Returns true if it became a keyframe.
  bool add(double time, const gtsam::Pose2& odom, const Image& polar);

  const std::vector<Keyframe>& keyframes() const { return keyframes_; }
  const std::vector<LoopClosure>& loops() const { return loops_; }
  std::optional<LoopClosure> last_loop() const { return last_loop_; }
  std::vector<gtsam::Pose2> poses() const;  // optimized keyframe poses
  const SlamOptions& options() const { return opt_; }

  // Session folder: poses.txt / odom_poses.txt (KITTI 3x4), times.txt, loops.csv, contexts.f64 + shape,
  // points/<k>.bin (float32 x y 0 0), cartesian/<k>.png (16-bit, for multi-session alignment).
  void save(const std::string& dir) const;

 private:
  std::optional<LoopClosure> close_loop(int query);

  SlamOptions opt_;
  Manager manager_;
  DenseRegistration dense_;
  std::vector<Keyframe> keyframes_;
  std::vector<LoopClosure> loops_;
  std::optional<LoopClosure> last_loop_;
  gtsam::ISAM2 isam_;
  gtsam::Values estimate_;
};

}  // namespace sonar_context
