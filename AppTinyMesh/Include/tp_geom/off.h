#pragma once
#include "mesh.h"
#include "simple_triangulation.h"
#include <fstream>
#include <filesystem>

inline TriangleMesh load_off(const std::filesystem::path &offFile) {
  std::ifstream stream(offFile);
  assert(!stream.bad() && stream.is_open());

  std::string in;
  stream >> in;
  assert(in == "OFF");

  size_t v_count, f_count;
  stream >> in;
  v_count = std::stoi(in);
  stream >> in;
  f_count = std::stoi(in);
  stream >> in;
  assert(in == "0");

  SimpleTriangulation tri;
  tri.init(v_count, f_count);

  size_t v = 0;
  while (!stream.eof() && v < v_count) {
    Vector vertex; 
    stream >> in;
    vertex[0] = std::stof(in);
    stream >> in;
    vertex[1] = std::stof(in);
    stream >> in;
    vertex[2] = std::stof(in);

    tri.add_vertex(vertex);
    v++;
  }

  size_t f = 0;
  while (!stream.eof() && f < f_count) {
    std::array<size_t, 3> vertices;

    stream >> in;
    assert(in == "3");

    stream >> in;
    vertices[0] = std::stoi(in);
    stream >> in;
    vertices[1] = std::stoi(in);
    stream >> in;
    vertices[2] = std::stoi(in);

    tri.add_triangle(vertices);
    f++;
  }

  return tri.extract_mesh();
}
