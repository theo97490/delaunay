#include "tp_geom/algo.h"

namespace TriMeshAlgorithm {
  SplitResult split_face(TriangleMesh& mesh, size_t tri_id, const Vector& point) {
    auto old_tri = mesh.triangles[tri_id];
    auto point_id = mesh.add_point(point);

    auto old_tri_size = mesh.triangles.size();
    mesh.triangles.resize( old_tri_size + 2 );
    std::array<size_t, 3> new_tri_ids = {tri_id, old_tri_size, old_tri_size + 1};

    const auto make_triangle = [&](LocalId<3> i){
      size_t new_triangle_id = new_tri_ids[i];
      auto &new_triangle = mesh.triangles[new_triangle_id];
      auto edge = old_tri.get_edge(i);
      
      // Set the face on the opposite triangle
      if (old_tri.opposite_triangle[i] != size_t_max){
        auto &tri_adj = mesh.triangles[old_tri.opposite_triangle[i]];
        auto adj_edge_id = tri_adj.find_edge(edge);
        tri_adj.opposite_triangle[adj_edge_id] = new_triangle_id;
      } 

      mesh.vertex_to_triangle[old_tri.vertices[i + 1]] = new_triangle_id;
      new_triangle.vertices = {point_id, edge.first, edge.second};
      new_triangle.opposite_triangle = { old_tri.opposite_triangle[i], new_tri_ids[i + 1], new_tri_ids[i - 1] };
    };

    for (int i = 0; i < 3; i++)
      make_triangle(i);

    mesh.vertex_to_triangle[point_id] = tri_id;

    assert_triangle_mesh_valid(mesh);
    return {point_id, new_tri_ids};
  }

  size_t split_edge(TriangleMesh& mesh, const size_t tri_id, const LocalId<3> edge_id, const Vector& point){
    auto point_id = mesh.add_point(point);
    
    // Consider the two triangles around the specified edge
    // Splits one of the triangle into two
    const auto split_triangle = [&mesh, &point_id]
      (const MTriangle& old_tri, const LocalId<3> edge_id, const size_t tri_id, const size_t new_tri_id, const size_t adj_tri_id, const size_t new_adj_tri_id){
      MTriangle& tri_1 = mesh.triangles[tri_id];
      MTriangle& tri_2 = mesh.triangles[new_tri_id];
      
      tri_1.vertices = { old_tri.vertices[edge_id - 1], old_tri.vertices[edge_id], point_id };
      tri_2.vertices = { old_tri.vertices[edge_id], old_tri.vertices[edge_id + 1], point_id };

      tri_1.opposite_triangle = {
        new_tri_id,
        adj_tri_id,
        old_tri.opposite_triangle[edge_id + 1]
      };

      tri_2.opposite_triangle = {
        new_adj_tri_id,
        tri_id,
        old_tri.opposite_triangle[edge_id - 1]
      };

      // Remesh the opposite triangle on the edge edge+1,
      // to point to link to this one instead if it exists
      size_t adj_id = old_tri.opposite_triangle[edge_id - 1];
      if (adj_id == size_t_max) return;

      MTriangle& opposite_tri_2 = mesh.triangles[adj_id];
      for (size_t& id : opposite_tri_2.opposite_triangle){
        if (id == tri_id){
          id = new_tri_id;
          break;
        }
      }
    }; // end remesh()
    
    MTriangle old_tri_1, old_tri_2;
    old_tri_1 = mesh.triangles[tri_id];

    auto old_tri_count = mesh.triangles.size();
    size_t new_size = old_tri_count + 1;

    size_t adj_tri_id = old_tri_1.opposite_triangle[edge_id]; 
    bool has_adj_tri = adj_tri_id != size_t_max; 
    size_t adj_tri_split_id = size_t_max;

    if (has_adj_tri){
      old_tri_2 = mesh.triangles[adj_tri_id];
      adj_tri_split_id = new_size;
      new_size++;
    }

    mesh.triangles.resize(new_size);

    mesh.vertex_to_triangle[point_id] = tri_id;
    split_triangle(old_tri_1, edge_id, tri_id, old_tri_count, adj_tri_id, adj_tri_split_id);
    
    if (has_adj_tri){
      MTriangle& adj_tri = mesh.triangles[adj_tri_id];
      auto edge = old_tri_1.get_edge(edge_id);
      size_t adj_edge_id = adj_tri.find_edge(edge);

      split_triangle(old_tri_2, adj_edge_id, adj_tri_id, adj_tri_split_id, tri_id, old_tri_count);
    }

    assert_triangle_mesh_valid(mesh);

    return point_id;
  }

  void edge_flip(TriangleMesh& mesh, const size_t tri_id_0, const LocalId<3> edge_id){
    MTriangle& tri_0 = mesh.triangles[tri_id_0];
    const size_t tri_id_1 = tri_0.opposite_triangle[edge_id]; 
    assert(tri_id_1 != size_t_max);

    MTriangle& tri_1 = mesh.triangles[tri_0.opposite_triangle[edge_id]];
    MTriangle old_tri_0 = tri_0, old_tri_1 = tri_1;

    const auto edge_pair = old_tri_0.get_edge(edge_id);
    const LocalId<3> o_edge_id = old_tri_1.find_edge(edge_pair);

    // Peut changer selon l'orientation du triangle en face
    const LocalId<3> tri_0_e0 = edge_id + 1;
    const LocalId<3> tri_0_e1 = edge_id - 1;
    const LocalId<3> tri_1_e0 = tri_1.local_id_of(edge_pair.first);
    const LocalId<3> tri_1_e1 = tri_1.local_id_of(edge_pair.second);

    tri_0.vertices[tri_0_e1] = old_tri_1.vertices[o_edge_id];
    tri_0.opposite_triangle[edge_id] = old_tri_1.opposite_triangle[tri_1_e1];
    tri_0.opposite_triangle[tri_0_e0] = tri_id_1;

    tri_1.vertices[tri_1_e0] = old_tri_0.vertices[edge_id];
    tri_1.opposite_triangle[o_edge_id] = old_tri_0.opposite_triangle[tri_0_e0];
    tri_1.opposite_triangle[tri_1_e1] = tri_id_0;

    mesh.vertex_to_triangle[old_tri_0.vertices[edge_id]] = tri_id_0; 
    mesh.vertex_to_triangle[old_tri_0.vertices[tri_0_e0]] = tri_id_0;
    mesh.vertex_to_triangle[old_tri_0.vertices[tri_0_e1]] = tri_id_1;
    mesh.vertex_to_triangle[old_tri_1.vertices[o_edge_id]] = tri_id_1;

    size_t adj_tri_0 = tri_0.opposite_triangle[edge_id];
    size_t adj_tri_1 = tri_1.opposite_triangle[o_edge_id];

    if (adj_tri_0 != size_t_max){
      const auto edge = tri_0.get_edge(edge_id);
      MTriangle& tri = mesh.triangles[adj_tri_0];
      const LocalId<3> a_edge_id = tri.find_edge(edge);
      tri.opposite_triangle[a_edge_id] = tri_id_0;
    }

    if (adj_tri_1 != size_t_max){
      const auto edge = tri_1.get_edge(o_edge_id);
      MTriangle& tri = mesh.triangles[adj_tri_1];
      const LocalId<3> a_edge_id = tri.find_edge(edge);
      tri.opposite_triangle[a_edge_id] = tri_id_1;
    }

    assert_triangle_mesh_valid(mesh);
  }
}
