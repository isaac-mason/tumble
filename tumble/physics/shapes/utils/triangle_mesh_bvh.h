#pragma once

#include <stdint.h>

#include "tumble/physics/shapes/utils/bvh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_data.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// a binary bvh over a mesh's triangles; building it reorders them so every leaf holds a contiguous range
namespace triangle_mesh_bvh {

using triangle_mesh_data::TriangleMeshData;

/** cost of traversing a bvh node */
constexpr float TRAVERSAL_COST = 1.0f;

/** cost of intersecting a triangle */
constexpr float TRIANGLE_INTERSECT_COST = 1.25f;

/** number of bins to use for sah */
constexpr uint32_t BIN_COUNT = 32;

/** bvh split strategies */
enum BvhSplitStrategy : uint32_t {
    /**
     * split at midpoint of center bounds.
     * - fast: O(n) with simple min/max computation
     * - good balance for most geometries
     */
    BVH_SPLIT_STRATEGY_CENTER = 0,

    /**
     * split at arithmetic mean of centers.
     * - fast: O(n) iteration to compute mean
     * - can create unbalanced splits on skewed distributions
     */
    BVH_SPLIT_STRATEGY_AVERAGE = 1,

    /**
     * surface area heuristic with binning.
     * - slow: O(n) with high constant factor (binning + cost evaluation)
     * - best quality: optimal expected query cost
     * - use for static meshes queried many times
     */
    BVH_SPLIT_STRATEGY_SAH = 2,
};

/** settings for bvh construction */
struct BvhBuildSettings {
    /** split strategy to use */
    BvhSplitStrategy strategy = BVH_SPLIT_STRATEGY_CENTER;

    /** maximum triangles per leaf node */
    uint32_t max_leaf_tris = 8;
};

/** get triangle start index (leaf only) */
inline uint32_t node_tri_start(const bvh::Node *nodes, int32_t node) { return (uint32_t)nodes[node].right_or_start; }

/** get triangle count (leaf only). decodes from negative flag. */
inline uint32_t node_tri_count(const bvh::Node *nodes, int32_t node) { return (uint32_t)(-(nodes[node].axis_or_count + 1)); }

/**
 * builds the bvh over data's triangles into `out` (pre-order, the root at 0; empty when there are
 * no triangles), reordering data.triangles. false (error set) when the allocator fails.
 */
bool build(collections::Vec<bvh::Node> &out, TriangleMeshData &data, const BvhBuildSettings &settings);

struct TriangleMeshBvhStats {
    uint32_t node_count;
    uint32_t leaf_count;
    uint32_t min_depth;
    uint32_t max_depth;
    float avg_depth;
};

/** compute depth statistics of the bvh tree, useful for performance analysis */
TriangleMeshBvhStats stats(const bvh::Node *nodes, uint32_t node_count);

}  // namespace triangle_mesh_bvh

TUMBLE_NAMESPACE_END
