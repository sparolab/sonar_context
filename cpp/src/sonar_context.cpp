#include "sonar_context/sonar_context.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace sonar_context {

Descriptor make_sonar_context(const Image& image_in, int pr, int pc, Layout layout, int skip_range) {
  if (pr < 1 || pc < 1) throw std::invalid_argument("patch sizes must be >= 1");
  Image full = layout == Layout::BeamRange ? Image(image_in.transpose()) : image_in;
  skip_range = std::clamp(skip_range, 0, static_cast<int>(full.rows()));
  Image image = full.bottomRows(full.rows() - skip_range);
  const int r = static_cast<int>(image.rows()) / pr, c = static_cast<int>(image.cols()) / pc;
  if (r == 0 || c == 0) throw std::invalid_argument("image is smaller than one patch");
  Descriptor sc(r, c);
  for (int i = 0; i < r; ++i)
    for (int j = 0; j < c; ++j) sc(i, j) = image.block(i * pr, j * pc, pr, pc).maxCoeff();
  return sc;
}

Eigen::VectorXd polar_key(const Descriptor& sc) { return sc.rowwise().mean(); }

namespace {

// a, b: (n, d) row vectors. Best mean cosine over shifts s in [-m, m] (a[j - s] against b[j]); ties to the
// smallest |s|, then the negative one (same order as the Python package).
Match shift_search(const Descriptor& a, const Descriptor& b, int m) {
  const int n = static_cast<int>(a.rows());
  Eigen::VectorXd na = a.rowwise().norm(), nb = b.rowwise().norm();
  std::vector<int> shifts;
  shifts.push_back(0);
  for (int s = 1; s <= m; ++s) shifts.push_back(-s), shifts.push_back(s);
  double best = -std::numeric_limits<double>::infinity();
  int best_s = 0;
  for (int s : shifts) {
    double sum = 0.0;
    int k = 0;
    for (int j = std::max(s, 0); j < std::min(n, n + s); ++j) {
      const int i = j - s;
      if (na[i] <= 0 || nb[j] <= 0) continue;
      sum += a.row(i).dot(b.row(j)) / (na[i] * nb[j]);
      ++k;
    }
    if (k == 0) continue;
    const double sim = sum / k;
    if (sim > best + 1e-12) best = sim, best_s = s;
  }
  if (!std::isfinite(best)) return {1.0, 0};
  return {1.0 - best, best_s};
}

}  // namespace

Match column_distance(const Descriptor& sc1, const Descriptor& sc2, double bound) {
  if (sc1.rows() != sc2.rows() || sc1.cols() != sc2.cols()) throw std::invalid_argument("shapes differ");
  return shift_search(sc1.transpose(), sc2.transpose(), static_cast<int>(sc1.cols() * bound));
}

Match row_distance(const Descriptor& sc1, const Descriptor& sc2, double bound) {
  if (sc1.rows() != sc2.rows() || sc1.cols() != sc2.cols()) throw std::invalid_argument("shapes differ");
  return shift_search(sc1, sc2, static_cast<int>(sc1.rows() * bound));
}

int Manager::add(const Descriptor& sc) {
  contexts_.push_back(sc);
  keys_.push_back(polar_key(sc));
  return size() - 1;
}

int Manager::add_image(const Image& image) {
  return add(make_sonar_context(image, opt_.patch_range, opt_.patch_beam, opt_.layout, opt_.skip_range));
}

std::vector<int> Manager::candidates(int idx) const {
  const int n = idx - opt_.exclude_recent;
  if (n < 1) return {};
  std::vector<std::pair<double, int>> d;
  d.reserve(n);
  for (int i = 0; i < n; ++i) d.emplace_back((keys_[i] - keys_[idx]).squaredNorm(), i);
  const int k = std::min(opt_.num_candidates, n);
  std::partial_sort(d.begin(), d.begin() + k, d.end());
  std::vector<int> out;
  for (int i = 0; i < k; ++i) out.push_back(d[i].second);
  return out;
}

std::optional<Loop> Manager::detect_loop(int idx) const {
  if (idx < 0) idx = size() - 1;
  const auto cands = candidates(idx);
  if (cands.empty()) return std::nullopt;
  const Descriptor& q = contexts_[idx];
  Match best;
  int c = -1;
  for (int cand : cands) {
    Match m = column_distance(contexts_[cand], q, opt_.col_bound);
    if (c < 0 || m.distance < best.distance) best = m, c = cand;
  }
  if (best.distance < opt_.col_threshold) return Loop{idx, c, best.distance, best.shift, 0, "column"};
  Match r = row_distance(contexts_[c], q, opt_.row_bound);
  if (r.distance < opt_.row_threshold) return Loop{idx, c, r.distance, best.shift, r.shift, "row"};
  return std::nullopt;
}

double SonarGeometry::yaw_of_shift(int column_shift) const {
  const double sector = fov_deg / std::max(1, beams) * patch_beam;
  return column_shift * sector * M_PI / 180.0;
}

std::vector<Eigen::Vector2d> polar_points(const Image& image, const SonarGeometry& g, double keep_percentile,
                                          double min_range) {
  const int nr = static_cast<int>(image.rows()), nb = static_cast<int>(image.cols());
  std::vector<float> v(image.data(), image.data() + image.size());
  const size_t k = std::min(v.size() - 1, static_cast<size_t>(keep_percentile / 100.0 * (v.size() - 1)));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  const float thr = std::max(v[k], 1.0f);
  std::vector<Eigen::Vector2d> pts;
  for (int r = 0; r < nr; ++r) {
    const double range = (r + 0.5) / nr * g.max_range;
    if (range < min_range) continue;
    for (int b = 0; b < nb; ++b) {
      if (image(r, b) <= thr) continue;
      const double th = ((b + 0.5) / nb * g.fov_deg - g.fov_deg / 2) * M_PI / 180.0;
      pts.emplace_back(range * std::cos(th), range * std::sin(th));
    }
  }
  return pts;
}

Image fan_to_polar(const Image& fan, double fov_deg, int range_bins, int beams) {
  const int H = static_cast<int>(fan.rows()), W = static_cast<int>(fan.cols());
  const double half = fov_deg / 2 * M_PI / 180.0;
  const double R = std::min<double>(H, (W / 2.0) / std::sin(half));  // pixels from the apex to the arc
  const double cx = W / 2.0, cy = H;                                     // apex: bottom center
  Image out = Image::Zero(range_bins, beams);
  for (int r = 0; r < range_bins; ++r) {
    const double rho = (r + 0.5) / range_bins * R;
    for (int b = 0; b < beams; ++b) {
      const double th = -half + (b + 0.5) / beams * 2 * half;  // left (b = 0) to right
      const double x = cx + rho * std::sin(th), y = cy - rho * std::cos(th);
      const int xi = static_cast<int>(std::floor(x)), yi = static_cast<int>(std::floor(y));
      if (xi < 0 || yi < 0 || xi + 1 >= W || yi + 1 >= H) continue;
      const double fx = x - xi, fy = y - yi;
      out(r, b) = static_cast<float>((1 - fx) * (1 - fy) * fan(yi, xi) + fx * (1 - fy) * fan(yi, xi + 1) +
                                     (1 - fx) * fy * fan(yi + 1, xi) + fx * fy * fan(yi + 1, xi + 1));
    }
  }
  return out;
}

}  // namespace sonar_context
