
#pragma once
#include "mesh.h"
#include "tp_geom/triangulation.h"
#include <algorithm>
  
using Edge = std::tuple<size_t, size_t>;

struct EdgeHasher {
  size_t operator()(const Edge &face) const noexcept {
    const auto &[a, b] = face;
    return a + b;
  }
};

using FaceToTriangle = std::unordered_map<Edge, std::pair<size_t, char>, EdgeHasher>;

struct SimpleTriangulation : public TriangulationBase {
  using ArrTriangle = std::array<size_t, 3>;
  void init(const size_t, const size_t);
  void add_vertex(const Vector& point);
  void add_triangle(const std::array<size_t, 3> &vertices);
private:
  FaceToTriangle edge_to_triangle;
};
