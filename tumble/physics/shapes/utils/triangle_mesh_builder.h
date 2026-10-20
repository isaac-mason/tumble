#pragma once

// crashcat's triangle-mesh-builder.ts: a triangle mesh from positions and indices. welds
// coincident vertices, drops degenerate and duplicate triangles, marks the active edges, builds
// the bvh (which reorders the triangles) and precomputes the triangle aabbs.

#include <stdint.h>

#include "physics/shapes/utils/triangle_mesh_bvh.h"
#include "physics/shapes/utils/triangle_mesh_data.h"

namespace triangle_mesh_builder {

using triangle_mesh_data::TriangleMeshData;

/** settings for building a triangle mesh */
struct TriangleMeshBuilderSettings {
    /** flat input positions xyz; read during build only */
    const float *positions = nullptr;
    /** the number of vertices, a third of the floats in positions */
    uint32_t num_positions = 0;
    /** three vertex indices per triangle, counter clockwise seen from the front */
    const uint32_t *indices = nullptr;
    uint32_t num_indices = 0;
    /** optional per-triangle material ids, one per triangle (num_indices / 3); null gives -1 (no material) for all */
    const int32_t *material_ids = nullptr;
    /** a triangle is considered degenerate if |cross product| < this value */
    float degenerate_tolerance = 1e-6f;
    triangle_mesh_bvh::BvhSplitStrategy bvh_split_strategy = triangle_mesh_bvh::BVH_SPLIT_STRATEGY_CENTER;
    /** maximum triangles per leaf node in the bvh */
    uint32_t bvh_max_leaf_tris = 8;
    /**
     * cosine threshold for active edge determination: a convex edge whose cos(dihedral angle) is
     * below this is active. concave edges are never active; a negative value keeps every edge active.
     */
    float active_edge_cos_threshold_angle = 0.996195f;  // cos(5 degrees)
};

/** build a triangle mesh from positions and indices; null (error set) when the memory cannot grow. */
TriangleMeshData *build(const TriangleMeshBuilderSettings &settings);

}  // namespace triangle_mesh_builder
