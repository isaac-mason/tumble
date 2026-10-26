#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/utils/bvh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_data.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// a binary bvh over a mesh's triangles; building it reorders them so every leaf holds a contiguous range

namespace triangle_mesh_bvh {

/** cost of traversing a bvh node */
constexpr float TRAVERSAL_COST = 1.0f;

/** cost of intersecting a triangle */
constexpr float TRIANGLE_INTERSECT_COST = 1.25f;

/** number of bins to use for sah */
constexpr uint32_t BIN_COUNT = 32;

}  // namespace triangle_mesh_bvh

/** settings for bvh construction */
struct TriangleMeshBvhBuildSettings {
    /** split strategy to use */
    TriangleMeshShape::BvhSplitStrategy strategy = TriangleMeshShape::BvhSplitStrategy::CENTER;

    /** maximum triangles per leaf node */
    uint32_t max_leaf_tris = 8;
};

namespace triangle_mesh_bvh {

/** get triangle start index (leaf only) */
inline uint32_t node_tri_start(const BvhNode *nodes, int32_t node) { return (uint32_t)nodes[node].right_or_start; }

/** get triangle count (leaf only). decodes from negative flag. */
inline uint32_t node_tri_count(const BvhNode *nodes, int32_t node) { return (uint32_t)(-(nodes[node].axis_or_count + 1)); }

/**
 * builds the bvh over data's triangles into `out` (pre-order, the root at 0; empty when there are
 * no triangles), reordering data.triangles.
 */
void build(collections::Array<BvhNode> &out, TriangleMeshShape::Data &data, const TriangleMeshBvhBuildSettings &settings);

}  // namespace triangle_mesh_bvh

struct TriangleMeshBvhStats {
    uint32_t node_count;
    uint32_t leaf_count;
    uint32_t min_depth;
    uint32_t max_depth;
    float avg_depth;
};

namespace triangle_mesh_bvh {

/** compute depth statistics of the bvh tree, useful for performance analysis */
TriangleMeshBvhStats stats(const BvhNode *nodes, uint32_t node_count);

}  // namespace triangle_mesh_bvh

TUMBLE_NAMESPACE_END
