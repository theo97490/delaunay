#include "tp_geom/utils.h"
#include <queue>
#include <stack>
#include <tp_geom/delaunay.h>
#include <tp_geom/algo.h>
#include <unordered_set>

Triangulation2D Triangulation2D::from_point_cloud(const std::vector<Vector> points){
  Triangulation2D tri;
  int i = 0;
  for (const Vector& p : points){
    tri.add_point(p);
    i++;
  }

  return tri;
}

TriOrient reorient_ccw(const TriangleMesh& mesh, MTriangle& tri){
  std::array<size_t, 3> ret;
  TriOrient orient;

  const std::array<Vector, 3> vertices = mesh.get_vertices(tri);

  orient = orientation_2d(vertices);
  if (orient == TriOrient::CCW)
    return TriOrient::CCW;

  orient = orientation_2d({ vertices[1], vertices[0], vertices[2] });
  if (orient == TriOrient::CCW){
    tri.vertices = {tri.vertices[1], tri.vertices[0], tri.vertices[2]};
    // tri.opposite_triangle = {tri.opposite_triangle[1], tri.opposite_triangle[0], tri.opposite_triangle[2]};
    return TriOrient::CCW;
  }

  return orient;
}

Triangulation2D::FoundTri Triangulation2D::find_nearest_triangle(const Vector& point) {
  if (mesh.triangles.size() == 0) 
    return {size_t_max, size_t_max, TriOrient::Flat};

  size_t tri_id; 
  const MTriangle* it;
  
  do {
    tri_id = rand() % mesh.triangles.size();
    it = &mesh.triangles[tri_id];
  } while(it->local_id_of(mesh.infinite_point) >= 0);

  std::array<Vector, 3> tri_v = mesh.get_vertices(*it);

  unsigned char min_edge_id = 0;
  TriOrient min_ori = TriOrient::CCW;

  for (int i = 0; i < 3; i++){
    LocalId<3> id = i;
    TriOrient ori = orientation_2d({ tri_v[id + 1], tri_v[id + 2], point });
    if (ori < min_ori) {
      min_edge_id = id;
      min_ori = ori;
    }
  }

  if (min_ori >= TriOrient::Flat)
    return { tri_id, min_edge_id, min_ori };

  while (min_ori == TriOrient::CW){
    size_t next_id = it->opposite_triangle[min_edge_id];
    MTriangle* next = &mesh.triangles[next_id];
    if (next->is_infinite()) break;

    LocalId<3> next_eid = next->find_edge(it->get_edge(min_edge_id));
    tri_v = mesh.get_vertices(*next);

    TriOrient e0 = orientation_2d({ tri_v[next_eid - 1], tri_v[next_eid], point});
    TriOrient e1 = orientation_2d({ tri_v[next_eid], tri_v[next_eid + 1], point});

    min_ori = e0;
    min_edge_id = next_eid + 1;
    
    if (e1 < min_ori){
      min_ori = e1;
      min_edge_id = next_eid - 1;
    }

    tri_id = next_id;
    it = next;
  }

  return {tri_id, min_edge_id, min_ori}; 
}

// Creates the first triangle and hull
// The hull is oriented CW instead of CCW to allow operations 
// like add_point_in_face to generate CCW triangles inside the surface
void Triangulation2D::init_first_triangle(){
  MTriangle tri;
  tri.vertices = { 1, 2, 3 };
  tri.opposite_triangle = { 1, 2, 3 }; 

  TriOrient orient = reorient_ccw(mesh, tri);
  assert(orient == TriOrient::CCW);
  mesh.triangles.push_back(tri);
  
  for (int i = 0; i < 3; i++){
    MTriangle htri;
    LocalId<3> id = i;

    // htri.vertices = { mesh.infinite_point, tri.vertices[id], tri.vertices[id + 1]};
    // htri.opposite_triangle = { 0, 1ul + (id + 1), 1ul + (id + 2) };

    htri.vertices = { mesh.infinite_point, tri.vertices[id - 1], tri.vertices[id + 1] };
    htri.opposite_triangle = { 0, 1ul + (id + 1), 1ul + (id - 1)};
    mesh.triangles.emplace_back(htri);
  }

  mesh.vertex_to_triangle[mesh.infinite_point] = 1;
  mesh.vertex_to_triangle[1] = 0;
  mesh.vertex_to_triangle[2] = 0;
  mesh.vertex_to_triangle[3] = 0;

  assert_triangle_mesh_valid(mesh);
}

size_t Triangulation2D::add_point(const Vector& point){
  if (mesh.triangles.size() == 0) {
    size_t point_id = mesh.add_point(point);
    if (mesh.vertices.size() >= 4)
      init_first_triangle();

    return point_id;
  }

  const auto& [tri_id, edge_id, orient] = find_nearest_triangle(point);

  switch (orient){
    // Inside the triangle, on a edge
    case TriOrient::Flat: 
      return TriMeshAlgorithm::split_edge(mesh, tri_id, edge_id, point);

    // Outside of the domain
    case TriOrient::CW:
      return add_point_outside_hull(tri_id, edge_id, point);

    // Inside the triangle
    case TriOrient::CCW:
      const auto [point_id, _] = TriMeshAlgorithm::split_face(mesh, tri_id, point);
      return point_id;
  }
}

size_t Triangulation2D::add_point_outside_hull(size_t nearest_tri, const LocalId<3> nearest_eid, const Vector& point){
  size_t hull_tri_id = mesh.triangles[nearest_tri].opposite_triangle[nearest_eid];
  MTriangle& hull_tri = mesh.triangles[hull_tri_id];

  const auto [id, new_tris] = TriMeshAlgorithm::split_face(mesh, hull_tri_id, point);
  const size_t point_id = id; // Can't capture id in lambda, had to do this trick

  FaceAroundVIt hull_1, hull_0;

  // Find the two triangle containing point_id around infinite_point
  for (int i = 0; i < 3; i++){
    size_t tri_id = new_tris[i];
    auto tri_it = FaceAroundVIt(&mesh, tri_id, mesh.infinite_point); 
    if (tri_it->local_id_of(mesh.infinite_point) < 0 ) continue;
    if (tri_it->local_id_of(point_id) < 0 ) continue;
    // Next triangle in the CCW direction ( the hull is CW )
    auto o_tri_it = tri_it - 1; 

    if (o_tri_it.is_valid() 
      && o_tri_it->local_id_of(mesh.infinite_point) >= 0
      && o_tri_it->local_id_of(point_id) >= 0){
      hull_1 = tri_it;
      hull_0 = o_tri_it;
      break;
    }
  }

  assert(hull_1.is_valid() && hull_1->local_id_of(mesh.infinite_point) >= 0);
  assert(hull_0.is_valid() && hull_0->local_id_of(mesh.infinite_point) >= 0);

  std::vector<size_t> tris_to_flip;
  tris_to_flip.reserve(10);

  const auto create_faces_around_hull = [&](const FaceAroundVIt& hull_start, char sign){
    tris_to_flip.clear();
    auto face_it = hull_start + sign;
    assert(hull_start->local_id_of(point_id) >= 0 );
    assert(face_it->local_id_of(point_id) < 0 );

    do {
      const LocalId<3> inf_id = face_it->local_id_of(mesh.infinite_point);    
      const Vector p1 = mesh.get_vertex(*face_it, inf_id + 1);
      const Vector p2 = mesh.get_vertex(*face_it, inf_id + 2);

      TriOrient orient = orientation_2d({ p1, p2, point });
      if (orient < TriOrient::Flat) break;
      tris_to_flip.emplace_back(face_it.get_tri_id());

      face_it = face_it + sign;
    } while (face_it != hull_start);

    for (size_t face_id : tris_to_flip){
      const MTriangle& tri = mesh.triangles[face_id];
      const auto inf_id = tri.local_id_of(mesh.infinite_point);    
      assert(inf_id >= 0);
      TriMeshAlgorithm::edge_flip(mesh, face_id, inf_id - sign);
    }
  };

  create_faces_around_hull(hull_0, -1);
  create_faces_around_hull(hull_1, 1);

  return point_id;
}

using Edge = std::pair<size_t, size_t>;
struct EdgeHasher {
  size_t operator()(const Edge &face) const noexcept {
    const auto &[a, b] = face;
    return a + b;
  }
};

Edge sort_edge(Edge edge){
    if (edge.first > edge.second)
      std::swap(edge.first, edge.second);

    return edge;
}

size_t DelaunayTriangulation2D::add_point(const Vector& point) {
  size_t point_id = Base::add_point(point);
  if (mesh.triangles.size() == 0) return point_id;
  
  auto face_begin = mesh.faces_around_v(point_id);
  auto face_it = face_begin;
  using Triangle = std::pair<size_t, LocalId<3>>;
  std::stack<Triangle> to_check;

  do {
    if (face_it->is_infinite()){
      face_it++;
      continue;
    }

    const LocalId<3> edge_id = face_it->local_id_of(point_id);
    assert(edge_id.is_valid());
    to_check.push({face_it.get_tri_id(), edge_id});
    face_it++;
  } while (face_it != face_begin);

  while (!to_check.empty()){
    const auto [tri_id, edge_id] = to_check.top();
    to_check.pop();

    MTriangle& tri = mesh.triangles[tri_id];
    assert(!tri.is_infinite());

    if (!TriMeshAlgorithm::is_edge_delaunay_2d(mesh, tri_id, edge_id)){
      const size_t o_tri_id = tri.opposite_triangle[edge_id];
      assert(o_tri_id != size_t_max);
      MTriangle& o_tri = mesh.triangles[o_tri_id];
      assert(!o_tri.is_infinite());

      const LocalId<3> o_edge_id = o_tri.find_edge(tri.get_edge(edge_id));
      const Edge new_edge = { tri.vertices[edge_id], o_tri.vertices[o_edge_id]};

      TriMeshAlgorithm::edge_flip(mesh, tri_id, edge_id);
      
      assert(tri.local_id_of(point_id).is_valid());
      assert(o_tri.local_id_of(point_id).is_valid());

      to_check.push({tri_id, tri.local_id_of(point_id)});
      to_check.push({o_tri_id, o_tri.local_id_of(point_id)});
    }

  }

  return point_id;
}

namespace TriMeshAlgorithm{
  bool is_delaunay_2d(const TriangleMesh& mesh) {
    std::unordered_set<Edge, EdgeHasher> checked;

    for (size_t t = 0; t < mesh.triangles.size(); t++){
      const MTriangle& tri = mesh.triangles[t]; 
      if (tri.is_infinite()) continue; 

      for (int i = 0; i < 3; i++){
        const Edge edge = sort_edge(tri.get_edge(i));
        if (checked.count(edge))
          continue;

        checked.insert(edge);
        if (!fully_delaunay(mesh, t, i))
          return false;
      }
    }

    return true;
  }

  bool is_edge_delaunay_2d(const TriangleMesh &mesh, const size_t tri_id, const LocalId<3> edge_id){
    const MTriangle& tri = mesh.triangles[tri_id];
    const size_t o_tri_id = tri.opposite_triangle[edge_id];
    const MTriangle& o_tri = mesh.triangles[o_tri_id];
    assert(o_tri_id != size_t_max);

    if (o_tri.is_infinite())
      return true;

    Vector p = mesh.get_vertex(tri, 0);
    Vector q = mesh.get_vertex(tri, 1);
    Vector r = mesh.get_vertex(tri, 2);
    Vector s = mesh.get_vertex(o_tri, o_tri.find_edge(tri.get_edge(edge_id)));

    Vector u = Vector{q[0] - p[0], q[1] - p[1], std::pow(q[0] - p[0], 2) + std::pow(q[1] - p[1], 2)};
    Vector v = Vector{r[0] - p[0], r[1] - p[1], std::pow(r[0] - p[0], 2) + std::pow(r[1] - p[1], 2)};
    Vector w = Vector{s[0] - p[0], s[1] - p[1], std::pow(s[0] - p[0], 2) + std::pow(s[1] - p[1], 2)};

    // in circle = -signe(((Φ(q)-Φ(p))x(Φ(r)-Φ(p))).(Φ(s)-Φ(p))
    // WARN: (u/v)*w somehow return the opposite sign compared to desmos simulation
    return (u/v)*w >= 0; 
  }
  
  bool fully_delaunay(const TriangleMesh &mesh, const size_t tri_id, const LocalId<3> edge_id){
    const MTriangle& tri = mesh.triangles[tri_id];
    const size_t o_tri_id = tri.opposite_triangle[edge_id];
    const MTriangle& o_tri = mesh.triangles[o_tri_id];
    assert(o_tri_id != size_t_max);
    const auto o_edge_id = o_tri.find_edge(tri.get_edge(edge_id));
    if (o_tri.is_infinite() or tri.is_infinite())
      return true;

    return is_edge_delaunay_2d(mesh, tri_id, edge_id) && is_edge_delaunay_2d(mesh, o_tri_id, o_edge_id);
  }

  // TriangleMesh to_delaunay(const Triangulation2D &tri){
  //   TriangleMesh mesh = tri.get_mesh();
  //   std::unordered_set<Edge, EdgeHasher> checked;
  //   std::queue<std::pair<size_t, Edge>> to_check;
  //
  //   const auto check_and_push = [&](size_t tri_id, LocalId<3> edge_id){
  //     const auto edge = sort_edge(mesh.triangles[tri_id].get_edge(edge_id));
  //     if (checked.count(edge)) return;
  //     to_check.push({tri_id, edge});
  //   };
  //
  //   const auto flip_and_push_neighbors = [&](size_t tri_id, LocalId<3> edge_id){
  //     const MTriangle& tri = mesh.triangles[tri_id];
  //     const size_t o_tri_id = tri.opposite_triangle[edge_id];
  //     edge_flip(mesh, tri_id, edge_id);
  //     const MTriangle& o_tri = mesh.triangles[o_tri_id];
  //     
  //     assert(!tri.is_infinite() && !o_tri.is_infinite());
  //     // On pourrait eviter de faire l'operation sur l'edge (ici doublé) flippé mais flemme 
  //     check_and_push(tri_id, 0);
  //     check_and_push(tri_id, 1);
  //     check_and_push(tri_id, 2);
  //     check_and_push(o_tri_id, 0);
  //     check_and_push(o_tri_id, 1);
  //     check_and_push(o_tri_id, 2);
  //   };
  //
  //   for (size_t t = 0; t < mesh.triangles.size(); t++){
  //     const MTriangle& tri = mesh.triangles[t]; 
  //     if (tri.is_infinite()) continue; 
  //
  //     for (int i = 0; i < 3; i++){
  //       const Edge edge = sort_edge(tri.get_edge(i));
  //
  //       if (checked.count(edge) == 0 && !fully_delaunay(mesh, t, i)) {
  //         to_check.push({ t, edge });
  //         // flip_and_push_neighbors(t, i);
  //       }
  //     }
  //   }
  //
  //   while (!to_check.empty()) {
  //     auto [tri_id, edge] = to_check.front();
  //     MTriangle* tri = &mesh.triangles[tri_id];
  //     if (!tri->find_edge(edge).is_valid()){
  //       const auto tri_begin = mesh.faces_around_v(edge.first);
  //       auto tri_it = tri_begin;
  //       do {
  //         if (tri_it->find_edge(edge).is_valid()){
  //           tri = tri_it;
  //           tri_id = tri_it.get_tri_id();
  //           break;
  //         }
  //         tri_it++;
  //       } while (tri_it != tri_begin);
  //
  //       assert(tri->find_edge(edge).is_valid());
  //     }
  //
  //     const size_t edge_id = tri->find_edge(edge);
  //     if (checked.count(edge) == 0 && !fully_delaunay(mesh, tri_id, edge_id)) {
  //       checked.insert(edge);
  //       flip_and_push_neighbors(tri_id, edge_id);
  //     }
  //     to_check.pop();
  //   }
  //
  //   // assert(is_delaunay_2d(mesh));
  //   return mesh;
  // }

  // INFO: Version très naive ... la version au dessus plus intellegente ne fonctionne pas 
  TriangleMesh to_delaunay(const Triangulation2D& tri) {
    TriangleMesh mesh = tri.get_mesh();
    std::unordered_set<Edge, EdgeHasher> checked;

    size_t flips = size_t_max;
    while (flips > 0){
      flips = 0;
      for (size_t t = 0; t < mesh.triangles.size(); t++){
        const MTriangle& tri = mesh.triangles[t]; 
        if (tri.is_infinite()) continue; 

        for (int i = 0; i < 3; i++){
          const Edge edge = sort_edge(tri.get_edge(i));
          if (checked.count(edge))
            continue;

          if (!fully_delaunay(mesh, t, i)){
            checked.insert(edge);
            flips++;
            edge_flip(mesh, t, i);
          }
        }
      }
    }

    return mesh;
  }
}
