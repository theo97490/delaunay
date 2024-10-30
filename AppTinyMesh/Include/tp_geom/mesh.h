#pragma once
#include "utils.h"
#include <vector>
#include <array>
#include <functional>

struct MTriangle {
  std::array<size_t, 3> vertices{size_t_max, size_t_max, size_t_max};
  std::array<size_t, 3> opposite_triangle{size_t_max, size_t_max, size_t_max};

  MTriangle() {}
  MTriangle(const std::array<size_t, 3>& v): vertices(v) {};
  MTriangle(const std::array<size_t, 3>& v, const std::array<size_t, 3>& f): vertices(v), opposite_triangle(f) {};

  LocalId<3> local_id_of(size_t v) const;
  bool is_infinite() const;

  std::pair<size_t, size_t> get_edge(LocalId<3>) const;
  LocalId<3> find_edge(size_t a, size_t b) const;
  LocalId<3> find_edge(std::pair<size_t, size_t>) const;
};

struct FaceAroundVIt;

struct TriangleMesh {
  std::vector<Vector> vertices;
  std::vector<size_t> vertex_to_triangle;
  std::vector<MTriangle> triangles;
  
  static constexpr size_t infinite_point = 0;
  static constexpr size_t v_start_offset = infinite_point + 1;

  TriangleMesh();

  size_t add_point(const Vector& point);

  template <unsigned long N>
  void add_points(const std::array<Vector, N>& points);

  void clear();
  bool is_empty() { return vertices.empty(); }

  MTriangle* get_triangle(const MTriangle &tri, char v) { 
    auto id = tri.opposite_triangle[v];
    if (id >= triangles.size()) return nullptr;
    return &triangles[id];
  }

  Vector get_vertex(const MTriangle &tri, char v) const { return vertices[tri.vertices[v]]; }
  std::array<Vector, 3> get_vertices(const MTriangle &tri) const;

  // Pas besoins de circulateurs ... juste renvoie un tableau
  // std::vector<MTriangle *> faces_around_v(size_t vertex);

  FaceAroundVIt faces_around_v(size_t vertex);
  using LaplacianFunc = std::function<double(TriangleMesh &, size_t)>;
  double laplacian(size_t vertex, const LaplacianFunc &compute);
};

template <unsigned long N>
void TriangleMesh::add_points(const std::array<Vector,N>& points){
  vertices.insert(vertices.end(), points.begin(), points.end());
  vertex_to_triangle.resize(vertex_to_triangle.size() + points.size(), size_t_max);
}

struct FaceAroundVIt {
  FaceAroundVIt(){}
  FaceAroundVIt(TriangleMesh *mesh, size_t triangle, size_t vertex)
      : mesh(mesh), triangle(triangle), vertex(vertex) {}

  MTriangle *get_triangle() const { return &mesh->triangles[triangle]; }
  size_t get_tri_id() const { return triangle; }
  size_t get_vertex() const { return vertex; }
  bool is_valid() const { return mesh != nullptr && triangle != size_t_max && vertex != size_t_max; }
  operator MTriangle*() const { return get_triangle(); }
  MTriangle& operator*() const { return *get_triangle(); }
  MTriangle* operator->() const { return get_triangle(); }
  size_t next_tri(bool sign) const;

  FaceAroundVIt operator-(const int i) const;
  FaceAroundVIt operator+(const int i) const;
  FaceAroundVIt& operator--(const int);
  FaceAroundVIt& operator++(const int);

  bool operator==(const FaceAroundVIt& other) const;
  bool operator!=(const FaceAroundVIt& other) const;
private:
  TriangleMesh* mesh;
  size_t triangle;
  size_t vertex;
};

// asserts that the triangle has valid vertices id, has a valid orientation
void assert_triangles_valid(const TriangleMesh& m, const bool orient_test = true, const bool connectivity_test = true);
void assert_vertices_valid(const TriangleMesh& m);
void assert_triangle_mesh_valid(const TriangleMesh& m);
