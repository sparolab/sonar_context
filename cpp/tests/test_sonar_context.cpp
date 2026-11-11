#include <cmath>
#include <cstdio>
#include <random>

#include "sonar_context/sonar_context.hpp"

using namespace sonar_context;

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

static Image scene(int seed, int rows = 120, int cols = 60) {
  std::mt19937 g(seed);
  std::uniform_real_distribution<float> u(0, 1);
  Image img(rows, cols);
  for (int i = 0; i < img.size(); ++i) img.data()[i] = u(g) * 20;
  for (int k = 0; k < 25; ++k) {
    int r = g() % (rows - 6), c = g() % (cols - 4);
    img.block(r, c, 6, 4).array() += 100 + 155 * u(g);
  }
  return img;
}

int main() {
  Image img = scene(0);
  Descriptor sc = make_sonar_context(img);
  CHECK(sc.rows() == 30 && sc.cols() == 15);
  CHECK(sc(0, 0) == img.block(0, 0, 4, 4).maxCoeff());
  Image t = img.transpose();
  CHECK((make_sonar_context(t, 4, 4, Layout::BeamRange) - sc).cwiseAbs().maxCoeff() == 0);

  auto m = column_distance(sc, sc);
  CHECK(std::abs(m.distance) < 1e-12 && m.shift == 0);
  for (int s : {-5, -1, 1, 3, 7}) {
    Descriptor moved = Descriptor::Zero(sc.rows(), sc.cols());
    for (int j = 0; j < sc.cols(); ++j)
      if (j - s >= 0 && j - s < sc.cols()) moved.col(j) = sc.col(j - s);
    auto mm = column_distance(sc, moved);
    CHECK(mm.shift == s && mm.distance < 1e-9);
  }

  ManagerOptions o;
  o.exclude_recent = 3;
  Manager mg(o);
  std::vector<Image> seq;
  for (int i = 0; i < 10; ++i) seq.push_back(scene(100 + i));
  for (auto& s : seq) CHECK(!mg.update(s));
  Image moved = Image::Zero(seq[2].rows(), seq[2].cols());
  for (int j = 0; j < moved.cols(); ++j) moved.col((j + 8) % moved.cols()) = seq[2].col(j);
  auto lp = mg.update(moved);
  CHECK(lp && lp->match == 2 && lp->query == 10);

  SonarGeometry g{130.0, 50.0, 300, 4};
  CHECK(std::abs(g.yaw_of_shift(1) - 130.0 / 300 * 4 * M_PI / 180) < 1e-12);

  std::printf(failures ? "%d failures\n" : "all passed\n", failures);
  return failures ? 1 : 0;
}
