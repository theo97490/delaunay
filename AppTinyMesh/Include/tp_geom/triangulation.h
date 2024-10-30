#pragma once
#include "mesh.h"
#include <algorithm>

struct TriangulationBase {
  const TriangleMesh& get_mesh() const { return mesh; }
  TriangleMesh&& extract_mesh() { return std::move(mesh); }
protected:
  TriangleMesh mesh;
};
