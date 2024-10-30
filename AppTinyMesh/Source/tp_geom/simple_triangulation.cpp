#include <tp_geom/simple_triangulation.h>

void SimpleTriangulation::init(const size_t v_size, const size_t t_size) {
  mesh.vertices.clear();
  mesh.triangles.clear();
  mesh.vertex_to_triangle.clear();
  mesh.vertices.reserve(v_size);
  mesh.vertex_to_triangle.assign(v_size, size_t_max);
  mesh.triangles.reserve(t_size);
}

void SimpleTriangulation::add_vertex(const Vector& point) {
  mesh.vertices.emplace_back(point);
}

void SimpleTriangulation::add_triangle(const std::array<size_t, 3> &vertices) {
  MTriangle tri;
  tri.vertices = vertices;

  size_t tri_id = mesh.triangles.size();

  for (int i = 0; i < 3; i++) {
    int a1 = (i + 1) % 3;
    int a2 = (i + 2) % 3;

    // If the Vector didn't have a MTriangle before, associate it
    size_t v_id = vertices[i];
    if (mesh.vertex_to_triangle[v_id] == size_t_max) {
      mesh.vertex_to_triangle[v_id] = tri_id;
    }

    // Sort the vertices id to have unique edges
    auto [min, max] = std::minmax(vertices[a1], vertices[a2]);
    Edge edge = {min, max};

    auto tri_it = edge_to_triangle.find(edge);

    if (tri_it == edge_to_triangle.end()) {
      edge_to_triangle[edge] = {tri_id, i};
      continue;
    }

    // Sew the two triangles
    auto [other_tri_id, other_l_vert] = tri_it->second;
    tri.opposite_triangle[i] = other_tri_id;

    MTriangle &other_triangle = mesh.triangles[other_tri_id];
    other_triangle.opposite_triangle[other_l_vert] = tri_id;
    edge_to_triangle.erase(tri_it);
  }

  mesh.triangles.push_back(std::move(tri));
}
