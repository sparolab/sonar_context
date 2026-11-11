// Dense registration recovers the relative pose of two simulated sonar views of the same reflectors.
#include <cmath>
#include <cstdio>
#include <random>

#include "dense_registration.hpp"
#include "sonar_slam.hpp"

using namespace sonar_context;

// Polar image (range bins x beams) of world reflectors seen from pose (x, y, yaw): 130 deg, 50 m.
static Image render(const std::vector<Eigen::Vector2d>& world, double x, double y, double yaw, std::mt19937& g) {
  const int R = 750, B = 300;
  std::normal_distribution<float> noise(0, 3);
  Image im(R, B);
  for (int i = 0; i < im.size(); ++i) im.data()[i] = 20 + noise(g);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (const auto& w : world) {
    const double dx = w.x() - x, dy = w.y() - y;
    const double lx = c * dx + s * dy, ly = -s * dx + c * dy;  // world -> sensor
    const double rho = std::hypot(lx, ly), th = std::atan2(ly, lx) * 180 / M_PI;
    if (rho < 6 || rho > 49 || std::abs(th) > 64) continue;
    const int r = static_cast<int>(rho / 50 * R), b = static_cast<int>((th + 65) / 130 * B);
    for (int dr = -3; dr <= 3; ++dr)
      for (int db = -1; db <= 1; ++db)
        if (r + dr >= 0 && r + dr < R && b + db >= 0 && b + db < B) im(r + dr, b + db) = 220;
  }
  return im;
}

int main() {
  std::mt19937 g(7);
  std::uniform_real_distribution<double> u(-60, 60);
  std::vector<Eigen::Vector2d> world;
  for (int i = 0; i < 400; ++i) world.emplace_back(u(g), u(g));
  for (int i = 0; i < 200; ++i) world.emplace_back(20 + 0.3 * i * 0.2, -10 + 0.15 * i);  // a quay wall
  int failures = 0;
  struct Case { double x, y, yaw; } cases[] = {{2.0, 0.5, 10 * M_PI / 180}, {-1.5, 2.0, -25 * M_PI / 180},
                                               {3.0, -1.0, 40 * M_PI / 180}};
  for (const auto& cs : cases) {
    DenseRegistration dr;
    // static pattern learned from a few unrelated frames first
    for (int k = 0; k < 3; ++k) dr.cartesian(render({}, 0, 0, 0, g));
    const cv::Mat a = dr.cartesian(render(world, 0, 0, 0, g), false);
    const cv::Mat b = dr.cartesian(render(world, cs.x, cs.y, cs.yaw, g), false);
    const DenseMatch m = dr.match(a, b, loop_yaws(0, 3.0, 0.0));
    const bool ok = std::hypot(m.x - cs.x, m.y - cs.y) < 0.6 && std::abs(m.yaw - cs.yaw) < 4 * M_PI / 180;
    std::printf("truth (%.1f, %.1f, %.0f) -> (%.2f, %.2f, %.1f) peak %.3f %s\n", cs.x, cs.y, cs.yaw * 180 / M_PI, m.x, m.y,
                m.yaw * 180 / M_PI, m.peak, ok ? "ok" : "FAIL");
    failures += !ok;
  }
  return failures ? 1 : 0;
}
