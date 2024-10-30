#include "tp_geom/utils.h"
#include <tp_geom/mesh.h>

bool MTriangle::is_infinite() const {
  return local_id_of(TriangleMesh::infinite_point) >= 0;
}

std::pair<size_t, size_t>
MTriangle::get_edge(LocalId<3> a) const {
  return {vertices[a + 1], vertices[a + 2]};
}

LocalId<3> MTriangle::find_edge(std::pair<size_t, size_t> p) const {
  return find_edge(p.first, p.second);
}

LocalId<3> MTriangle::find_edge(size_t a, size_t b) const {
  LocalId<3> ret = LocalId<3>::make_invalid();
  assert(a != b);
  bool a_exist = false;
  bool b_exist = false;
  for (int i = 0; i < 3; i++) {
    auto vertex = vertices[i];
    if (vertex == a)
      a_exist = true;
    else if (vertex == b)
      b_exist = true;
    else
      ret = i;
  }

  assert(a_exist);
  assert(b_exist);
  return ret;
}

LocalId<3> MTriangle::local_id_of(size_t v) const {
  char l_id = LocalId<3>::make_invalid();
  for (int i = 0; i < 3; i++) {
    if (vertices[i] == v)
      l_id = i;
  }

  return l_id;
}

// Assumes that the vertex V has a fully circular neighboring of triangle 
// Otherwise good luck with that
FaceAroundVIt TriangleMesh::faces_around_v(size_t vertex){
  const size_t face = vertex_to_triangle[vertex];
  assert(face < triangles.size());
  return FaceAroundVIt(this, face, vertex);
}

// std::vector<MTriangle *> TriangleMesh::faces_around_v(size_t vertex) {
//   size_t start_t_id = vertex_to_triangle[vertex];
//   assert(start_t_id != size_t_max);
//
//   std::vector<MTriangle *> faces;
//
//   MTriangle tri = triangles[start_t_id];
//   size_t t_id = start_t_id;
//   size_t v = vertex;
//
//   while (t_id != size_t_max) {
//     faces.push_back(&triangles[t_id]);
//
//     // Find local id of v
//     char l_id = tri.local_id_of(v);
//
//     // Turn around v, by taking the opposite face of the successor
//     t_id = tri.opposite_triangle[(l_id + 1) % 3];
//     assert(t_id != size_t_max);
//     tri = triangles[t_id];
//
//     if (t_id == start_t_id)
//       break;
//   }
//
//   return faces;
// }

TriangleMesh::TriangleMesh(){
  vertices.resize(1); 
  vertex_to_triangle.resize(1);
  vertex_to_triangle[0] = size_t_max;
  constexpr double inf = std::numeric_limits<double>::infinity();
  vertices[0] = Vector{0,0,inf};
}

size_t TriangleMesh::add_point(const Vector& point){
  size_t ret = vertices.size();
  vertices.emplace_back(point);
  vertex_to_triangle.emplace_back(); 
  return ret;
}

void TriangleMesh::clear(){
  vertices.resize(1);
  vertex_to_triangle.clear();
  triangles.clear();
}

std::array<Vector, 3> TriangleMesh::get_vertices(const MTriangle &tri) const{
  std::array<Vector, 3> ret; 
  ret[0] = get_vertex(tri, 0);
  ret[1] = get_vertex(tri, 1);
  ret[2] = get_vertex(tri, 2);
  return ret;
}

// Only works on vertices that have a fully closed neighboring of triangles
double TriangleMesh::laplacian(size_t vertex, const LaplacianFunc &compute) {
  float this_value = compute(*this, vertex);
  float sum_r = 0, sum_area = 0;

  auto calc_cot = [&](MTriangle &tri, LocalId<3> l_id) {
    Vector n0 = get_vertex(tri, l_id - 1);
    Vector n1 = get_vertex(tri, l_id);
    Vector n2 = get_vertex(tri, l_id + 1);

    return angle_between(n0 - n1, n2 - n1);
  };

  auto calc_area = [&](MTriangle &tri, LocalId<3> l_id) {
    Vector n0 = get_vertex(tri, l_id - 1);
    Vector n1 = get_vertex(tri, l_id);
    Vector n2 = get_vertex(tri, l_id + 1);

    auto ret = 0.5 * bivector_area(n0 - n1, n2 - n1);
    return ret;
  };

  const auto it_start = faces_around_v(vertex);
  auto it = it_start;

  if (it.is_valid()) do {
    MTriangle *tri_0 = it;
    MTriangle *tri_1 = it + 1;

    using LocalId = LocalId<3>;
    LocalId l_0_id = tri_0->local_id_of(vertex);
    LocalId l_0_prev = l_0_id - 1;
    LocalId l_0_next = l_0_id + 1;

    LocalId l_1_id = tri_1->local_id_of(vertex);
    LocalId l_1_next = l_1_id + 1;

    float val_diff = compute(*this, tri_0->vertices[l_0_next]) - this_value;

    float cot_0 = calc_cot(*tri_0, l_0_prev);
    float cot_1 = calc_cot(*tri_1, l_1_next);
    assert(cot_0 > 0);
    assert(cot_1 > 0);

    sum_r += val_diff * (cot_0 + cot_1);
    sum_area = calc_area(*tri_0, l_0_prev);

    it++;

    // Should always be valid because the connected must form a loop
    assert(it.is_valid());
  } while (it != it_start);

  return sum_r / (2 * (1. / 3) * sum_area);
}

inline size_t FaceAroundVIt::next_tri(const bool sign) const {
  MTriangle& tri = mesh->triangles[triangle];
  LocalId<3> id = tri.local_id_of(vertex);
  assert(id >= 0);

  auto next_id = sign ? id + 1 : id - 1;
  return tri.opposite_triangle[next_id];
}

bool FaceAroundVIt::operator==(const FaceAroundVIt& other) const {
  return other.mesh == mesh && other.triangle == triangle && other.vertex == vertex;
}


bool FaceAroundVIt::operator!=(const FaceAroundVIt& other) const {
  return !(other == *this);
}

FaceAroundVIt FaceAroundVIt::operator-(const int count) const{
  if (count == 0) return *this;

  size_t tri = triangle;
  for (int i = 0; i < abs(count); i++)
    tri = next_tri(-count > 0);

  return FaceAroundVIt(mesh, tri, vertex);
}

FaceAroundVIt FaceAroundVIt::operator+(const int count) const{
  if (count == 0) return *this;

  size_t tri = triangle;
  for (int i = 0; i < abs(count); i++)
    tri = next_tri(count > 0);

  return FaceAroundVIt(mesh, tri, vertex);
}

FaceAroundVIt& FaceAroundVIt::operator--(const int) {
  triangle = next_tri(0);
  return *this;
}


FaceAroundVIt& FaceAroundVIt::operator++(const int) {
  triangle = next_tri(1);
  return *this;
}

void assert_triangles_valid(const TriangleMesh& m, const bool orient_test, const bool connectivity_test){
#ifndef NDEBUG
  for (size_t tri_id = 0; tri_id < m.triangles.size(); tri_id++){
    const auto v = [&](const MTriangle& tri, size_t id) { 
      return m.vertices[tri.vertices[id]];
    };

    const MTriangle& tri = m.triangles[tri_id];

    for (int i = 0; i < 3; i++){
      assert(tri.vertices[i] < m.vertices.size());
      size_t opposite_tri = tri.opposite_triangle[i];

      if (connectivity_test && opposite_tri != size_t_max){
        const MTriangle& otri = m.triangles[opposite_tri];
        const auto edge = tri.get_edge(i);
        const auto other_eid = otri.find_edge(edge);
        assert(other_eid != -1);
        assert(otri.opposite_triangle[other_eid] == tri_id);
      }
    }

    if (tri.is_infinite()){
      return;
    }

    if (orient_test)
      assert(orientation_2d({v(tri, 0), v(tri, 1), v(tri, 2)}) >= TriOrient::Flat);
  }
#endif
}

void assert_vertices_valid(const TriangleMesh &m){
#ifndef NDEBUG
  for (size_t vertex_id = 0; vertex_id < m.vertices.size(); vertex_id++){
    size_t tri_id = m.vertex_to_triangle[vertex_id];
    assert(tri_id < m.triangles.size());
    const MTriangle& tri = m.triangles[tri_id];

    // No duplicate vertex
    for (int i = 0; i < 3; i++){
      const size_t v = tri.vertices[i];
      LocalId<3> id(i);
      assert(tri.vertices[id + 1] != v);
      assert(tri.vertices[id + 2] != v);
    }

    // Don't fail if the infinite point is not used 
    if (vertex_id == TriangleMesh::infinite_point && tri_id == size_t_max)
      return;
    
    assert(tri.local_id_of(vertex_id) >= 0);
  }
#endif
}

void assert_triangle_mesh_valid(const TriangleMesh& m){
  assert_vertices_valid(m);
  assert_triangles_valid(m);
}
