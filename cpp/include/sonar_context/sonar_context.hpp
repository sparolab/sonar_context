// Sonar Context: imaging-sonar place recognition (ICRA 2023). C++17, Eigen only.
//
// The input is the polar sonar image with rows = range bins and columns = beams, so that a column shift of
// the descriptor is a rotation of the vehicle and a row shift a motion along the beams. Same results as the
// Python package (python/sonar_context).
#pragma once

#include <Eigen/Core>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sonar_context {

inline constexpr const char* kVersion = "2.0.0";

using Image = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Descriptor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

enum class Layout { RangeBeam, BeamRange };

// Max intensity of each patch (range bins x beams). Pixels that do not fill a whole patch are dropped;
// skip_range drops the first range bins (the bright near field).
Descriptor make_sonar_context(const Image& image, int patch_range = 4, int patch_beam = 4,
                              Layout layout = Layout::RangeBeam, int skip_range = 0);

// Mean of each range ring: invariant to the bearing (column) shift.
Eigen::VectorXd polar_key(const Descriptor& sc);

struct Match {
  double distance = 1.0;  // 1 - best mean cosine similarity
  int shift = 0;          // > 0: sc1's column (row) j - shift matches sc2's column (row) j
};

// Bearing shifts up to bound x columns, both ways (shift 0 included).
Match column_distance(const Descriptor& sc1, const Descriptor& sc2, double bound = 0.5);
// Range shifts up to bound x rows, both ways.
Match row_distance(const Descriptor& sc1, const Descriptor& sc2, double bound = 0.1);

struct Loop {
  int query = -1, match = -1;
  double distance = 1.0;
  int column_shift = 0, row_shift = 0;
  std::string by;  // "column" or "row"
};

struct ManagerOptions {
  double col_threshold = 0.045;
  double row_threshold = 0.072;
  int exclude_recent = 30;
  int num_candidates = 1;
  double col_bound = 0.5, row_bound = 0.1;
  int patch_range = 4, patch_beam = 4, skip_range = 0;
  Layout layout = Layout::RangeBeam;
};

// Online place recognition: polar-key nearest neighbours among frames older than exclude_recent, accepted by
// the column distance, else by the row distance.
class Manager {
 public:
  explicit Manager(ManagerOptions options = {}) : opt_(options) {}
  int add(const Descriptor& sc);
  int add_image(const Image& image);
  std::vector<int> candidates(int idx) const;
  std::optional<Loop> detect_loop(int idx = -1) const;
  std::optional<Loop> update(const Image& image) { add_image(image); return detect_loop(); }
  int size() const { return static_cast<int>(contexts_.size()); }
  const Descriptor& context(int i) const { return contexts_[i]; }
  const ManagerOptions& options() const { return opt_; }

 private:
  ManagerOptions opt_;
  std::vector<Descriptor> contexts_;
  std::vector<Eigen::VectorXd> keys_;
};

// Sonar geometry: bearing of a descriptor column and the yaw of a column shift.
struct SonarGeometry {
  double fov_deg = 130.0;   // horizontal aperture
  double max_range = 50.0;  // m
  int beams = 0;            // image columns (after the layout is applied)
  int patch_beam = 4;
  double yaw_of_shift(int column_shift) const;  // rad
};

// Polar image (range bins x beams) to 2D points (x forward, y left) of the strongest returns.
std::vector<Eigen::Vector2d> polar_points(const Image& image, const SonarGeometry& g, double keep_percentile = 99.0,
                                          double min_range = 2.0);

// Fan-shaped (Cartesian) sonar image to polar (range bins x beams), apex at the bottom center.
Image fan_to_polar(const Image& fan, double fov_deg, int range_bins, int beams);

}  // namespace sonar_context
