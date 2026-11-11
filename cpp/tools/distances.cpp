// Distances of descriptor pairs, for the parity check with Python (python/tests/test_cpp.py).
//   sonar_context_distances <descriptors.f64> <n> <rows> <cols> <pairs.txt>
// descriptors.f64: n x rows x cols doubles (row-major); pairs.txt: "i j" per line.
// Prints "i j col_distance col_shift row_distance row_shift" per pair.
#include <cstdio>
#include <fstream>
#include <iostream>

#include "sonar_context/sonar_context.hpp"

int main(int argc, char** argv) {
  if (argc != 6) {
    std::cerr << "usage: sonar_context_distances <descriptors.f64> <n> <rows> <cols> <pairs.txt>\n";
    return 2;
  }
  const int n = std::stoi(argv[2]), r = std::stoi(argv[3]), c = std::stoi(argv[4]);
  std::ifstream f(argv[1], std::ios::binary);
  std::vector<sonar_context::Descriptor> sc(n, sonar_context::Descriptor(r, c));
  for (auto& d : sc) f.read(reinterpret_cast<char*>(d.data()), sizeof(double) * r * c);
  if (!f) {
    std::cerr << "short read\n";
    return 1;
  }
  std::ifstream p(argv[5]);
  int i, j;
  while (p >> i >> j) {
    auto cm = sonar_context::column_distance(sc[i], sc[j]);
    auto rm = sonar_context::row_distance(sc[i], sc[j]);
    std::printf("%d %d %.12f %d %.12f %d\n", i, j, cm.distance, cm.shift, rm.distance, rm.shift);
  }
  return 0;
}
