#pragma once
#include "mesh.h"
#include "tp_geom/triangulation.h"

/** Iterative solution for creating 2d triangles using a convex hull.
  * The convex hull is oriented CW, as opposed to the triangulation surface which is CCW
  */
struct Triangulation2D : public TriangulationBase {
  virtual size_t add_point(const Vector& point);
  static Triangulation2D from_point_cloud(const std::vector<Vector> points);
private:
  using FoundTri = std::tuple<size_t, char, TriOrient>;
  using SplitResult = std::pair<size_t, std::array<size_t, 3>>;

  FoundTri find_nearest_triangle(const Vector& point);
  void init_first_triangle();
  size_t add_point_outside_hull(size_t nearest_tri, const LocalId<3> nearest_eid, const Vector& point);
};

struct DelaunayTriangulation2D : public Triangulation2D {
  using Base = Triangulation2D;
  size_t add_point(const Vector& point) override;
};

namespace TriMeshAlgorithm {
  bool is_delaunay_2d(const TriangleMesh& mesh);
  bool is_edge_delaunay_2d(const TriangleMesh& mesh, const size_t tri_id, const LocalId<3> edge_id);
  bool fully_delaunay(const TriangleMesh &mesh, const size_t tri_id, const LocalId<3> edge_id);
  TriangleMesh to_delaunay(const Triangulation2D& tri);
}
