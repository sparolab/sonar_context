#include "dense_registration.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace sonar_context {

void DenseRegistration::build_maps(int range_bins, int beams) {
  bins_ = range_bins, beams_ = beams;
  const int N = static_cast<int>(std::round(2 * opt_.half_size / opt_.resolution));
  map_r_.create(N, N, CV_32F), map_b_.create(N, N, CV_32F), mask_ = cv::Mat::zeros(N, N, CV_8U);
  for (int i = 0; i < N; ++i)       // row i: x (forward)
    for (int j = 0; j < N; ++j) {   // column j: y (left)
      const double x = (i + 0.5) * opt_.resolution - opt_.half_size, y = (j + 0.5) * opt_.resolution - opt_.half_size;
      const double rho = std::hypot(x, y), th = std::atan2(y, x) * 180.0 / M_PI;
      map_r_.at<float>(i, j) = static_cast<float>(rho / opt_.max_range * range_bins - 0.5);  // polar row
      map_b_.at<float>(i, j) = static_cast<float>((th + opt_.fov_deg / 2) / opt_.fov_deg * beams - 0.5);
      if (rho >= opt_.min_range && rho <= opt_.max_range - 1 && std::abs(th) <= opt_.fov_deg / 2 - 1)
        mask_.at<uint8_t>(i, j) = 1;
    }
  // a soft window over the fan: its border would otherwise correlate with itself at any shift of the sensor
  mask_.convertTo(window_, CV_32F);
  const double sigma = opt_.taper / opt_.resolution;
  if (sigma > 0) {
    cv::Mat w;
    cv::erode(window_, w, cv::Mat(), cv::Point(-1, -1), static_cast<int>(2 * sigma));
    cv::GaussianBlur(w, window_, cv::Size(0, 0), sigma);
  }
}

cv::Mat DenseRegistration::cartesian(const Image& polar, bool learn) {
  const int R = static_cast<int>(polar.rows()), B = static_cast<int>(polar.cols());
  if (R != bins_ || B != beams_) build_maps(R, B), static_.release();
  cv::Mat p(R, B, CV_32F, const_cast<float*>(polar.data()));
  cv::Mat s;
  // sigmaX along the columns (beams) 1, sigmaY along the rows (range bins) 4
  cv::GaussianBlur(p, s, cv::Size(0, 0), 1.0, 4.0);
  if (static_.empty()) static_ = s.clone();
  cv::Mat d = s - static_;
  if (learn) cv::addWeighted(static_, 1.0 - opt_.static_rate, s, opt_.static_rate, 0.0, static_);
  // per-range background: the median over the beams of each range row
  std::vector<float> row(B);
  for (int r = 0; r < R; ++r) {
    float* q = d.ptr<float>(r);
    std::copy(q, q + B, row.begin());
    std::nth_element(row.begin(), row.begin() + B / 2, row.end());
    const float med = row[B / 2];
    for (int b = 0; b < B; ++b) q[b] = std::max(0.0f, q[b] - med);
  }
  cv::Mat out;
  cv::remap(d, out, map_b_, map_r_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
  out.setTo(0, mask_ == 0);
  return out.mul(window_);
}

namespace {
// Normalized phase correlation; returns the peak and its (row, col) shift: target(i, j) ~ source(i - di, j - dj).
double phase_peak(const cv::Mat& Ft, const cv::Mat& source, int& di, int& dj) {
  cv::Mat Fs, R, C;
  cv::dft(source, Fs, cv::DFT_COMPLEX_OUTPUT);
  cv::mulSpectrums(Ft, Fs, R, 0, true);  // Ft * conj(Fs)
  std::vector<cv::Mat> ch;
  cv::split(R, ch);
  cv::Mat mag;
  cv::magnitude(ch[0], ch[1], mag);
  mag += 1e-9;
  ch[0] /= mag, ch[1] /= mag;
  cv::merge(ch, R);
  cv::dft(R, C, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
  double mx;
  cv::Point loc;
  cv::minMaxLoc(C, nullptr, &mx, nullptr, &loc);
  const int N = C.rows;
  di = loc.y > N / 2 ? loc.y - N : loc.y;
  dj = loc.x > N / 2 ? loc.x - N : loc.x;
  return mx;
}
}  // namespace

DenseMatch DenseRegistration::match(const cv::Mat& target, const cv::Mat& source, const std::vector<double>& yaws) const {
  cv::Mat Ft;
  cv::dft(target, Ft, cv::DFT_COMPLEX_OUTPUT);
  const cv::Point2f c(source.cols / 2.0f - 0.5f, source.rows / 2.0f - 0.5f);  // the sensor
  DenseMatch best;
  best.peak = -1;
  for (double yaw : yaws) {
    // rotate the source frame by +yaw about the sensor: a point (x, y) -> R(yaw) (x, y). In the image, rows are x
    // and columns are y (a transposed, mirrored view), which keeps the sense of OpenCV's angle.
    cv::Mat M = cv::getRotationMatrix2D(c, yaw * 180.0 / M_PI, 1.0), rs;
    cv::warpAffine(source, rs, M, source.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    int di, dj;
    const double pk = phase_peak(Ft, rs, di, dj);
    if (pk > best.peak) best = {pk, di * opt_.resolution, dj * opt_.resolution, yaw};
  }
  return best;
}

}  // namespace sonar_context
