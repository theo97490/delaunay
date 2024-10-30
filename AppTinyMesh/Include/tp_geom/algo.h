#include "tp_geom/mesh.h"

namespace TriMeshAlgorithm {
  using SplitResult = std::pair<size_t, std::array<size_t, 3>>;
  SplitResult split_face(TriangleMesh& mesh, size_t tri_id, const Vector& point);
  size_t split_edge(TriangleMesh& mesh, const size_t tri_id, const LocalId<3> edge_id, const Vector& point);
  void edge_flip(TriangleMesh& mesh, const size_t tri_id_0, const LocalId<3> edge_id);
}
