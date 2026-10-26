#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/utils/triangle_mesh_bvh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_data.h"

TUMBLE_NAMESPACE_BEGIN

// a mesh from positions and indices: coincident vertices welded, degenerate and duplicate triangles dropped, active
// edges marked, the bvh built (reordering the triangles) and the triangle boxes precomputed

namespace triangle_mesh_builder {

/** build a triangle mesh from positions and indices */
TriangleMeshShape::Data *build(const TriangleMeshShapeSettings &settings);

}  // namespace triangle_mesh_builder

TUMBLE_NAMESPACE_END
