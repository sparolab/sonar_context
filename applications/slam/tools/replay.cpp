// Offline replay without ROS: polar images + poses -> the SLAM back-end, for tuning and for runs without ROS.
//   sonar_context_replay <images.u8> <n> <rows> <cols> <layout range_beam|beam_range> <poses.txt> <out_dir> [--verbose]
//     [--col-threshold t] [--skip-range n] [--patch-range n] [--patch-beam n] [--min-peak p] [--flip]
// images.u8: n images of rows x cols uint8 (row-major); poses.txt: "time x y yaw" per image.
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "sonar_slam.hpp"

using namespace sonar_context;

int main(int argc, char** argv) {
  if (argc < 8) {
    std::cerr << "usage: sonar_context_replay <images.u8> <n> <rows> <cols> <range_beam|beam_range> <poses.txt> <out_dir> [options]\n";
    return 2;
  }
  const int n = std::stoi(argv[2]), rows = std::stoi(argv[3]), cols = std::stoi(argv[4]);
  const std::string layout = argv[5];
  SlamOptions o;
  bool flip = false;  // poses in a z-down frame: flip them and the beams
  o.loop.col_threshold = 0.2, o.loop.row_threshold = 0.0, o.loop.skip_range = 75, o.loop.patch_range = 25;
  for (int i = 8; i < argc; ++i) {
    std::string k = argv[i];
    auto next = [&]() { return std::stod(argv[++i]); };
    if (k == "--verbose") o.verbose = true;
    else if (k == "--flip") flip = true;
    else if (k == "--col-threshold") o.loop.col_threshold = next();
    else if (k == "--skip-range") o.loop.skip_range = static_cast<int>(next());
    else if (k == "--patch-range") o.loop.patch_range = static_cast<int>(next());
    else if (k == "--patch-beam") o.loop.patch_beam = static_cast<int>(next());
    else if (k == "--candidates") o.loop.num_candidates = static_cast<int>(next());
    else if (k == "--min-peak") o.min_peak = next();
    else if (k == "--keep-percentile") o.keep_percentile = next();
    else if (k == "--keyframe-distance") o.keyframe_distance = next();
  }
  SonarSlam slam(o);
  std::ifstream fi(argv[1], std::ios::binary), fp(argv[6]);
  std::vector<unsigned char> buf(static_cast<size_t>(rows) * cols);
  for (int i = 0; i < n; ++i) {
    double t, x, y, yaw;
    fp >> t >> x >> y >> yaw;
    fi.read(reinterpret_cast<char*>(buf.data()), buf.size());
    if (!fi || !fp) break;
    Image img(rows, cols);
    for (size_t k = 0; k < buf.size(); ++k) img.data()[k] = buf[k];
    if (layout == "beam_range") img = Image(img.transpose());
    if (flip) img = Image(img.rowwise().reverse()), y = -y, yaw = -yaw;
    slam.add(t, gtsam::Pose2(x, y, yaw), img);
  }
  slam.save(argv[7]);
  std::printf("%zu keyframes, %zu loops -> %s\n", slam.keyframes().size(), slam.loops().size(), argv[7]);
  return 0;
}
