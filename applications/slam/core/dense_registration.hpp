// Dense sonar-to-sonar registration for loop verification: the polar image, with its static pattern (a running
// mean of past frames) and its per-range background removed, is resampled to a Cartesian grid around the sensor;
// a query is matched to a candidate by phase correlation over a set of yaws. The normalized correlation peak tells
// a real revisit (a sharp peak) from a false one.
#pragma once

#include <opencv2/core.hpp>
#include <vector>

#include "sonar_context/sonar_context.hpp"

namespace sonar_context {

struct DenseOptions {
  double resolution = 0.25;  // m per cell
  double half_size = 52.0;   // m, grid covers [-half, half]^2 around the sensor
  double min_range = 6.0;    // m
  double fov_deg = 130.0, max_range = 50.0;
  double static_rate = 0.02;  // running-mean rate of the static pattern
  double taper = 2.0;         // m, soft edge of the fan
};

struct DenseMatch {
  double peak = 0;           // normalized phase-correlation peak
  double x = 0, y = 0, yaw = 0;  // pose of the source frame in the target frame (m, m, rad)
};

class DenseRegistration {
 public:
  explicit DenseRegistration(DenseOptions o = {}) : opt_(o) {}
  // Cartesian image of a polar frame (range bins x beams); updates the static pattern when `learn` is set.
  cv::Mat cartesian(const Image& polar, bool learn = true);
  // Match `source` to `target` (both from cartesian()) over the given yaws (rad).
  DenseMatch match(const cv::Mat& target, const cv::Mat& source, const std::vector<double>& yaws) const;
  const DenseOptions& options() const { return opt_; }

 private:
  void build_maps(int range_bins, int beams);
  DenseOptions opt_;
  cv::Mat static_, map_r_, map_b_, mask_, window_;
  int bins_ = 0, beams_ = 0;
};

}  // namespace sonar_context
