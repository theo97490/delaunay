#include <string>
#include <vector>
#include <mathematics.h>
#include <fstream>
#include <cassert>

inline std::vector<Vector> load_point_cloud(const std::string& path){
  std::ifstream stream(path);
  assert(!stream.bad() && stream.is_open());

  size_t count;
  stream >> count;

  std::vector<Vector> points;
  points.reserve(count);

  for (size_t i = 0; i < count; i++){
    Vector p;
    stream >> p[0];
    stream >> p[1];
    stream >> p[2];
    points.emplace_back(p);
  }

  return points;
}
