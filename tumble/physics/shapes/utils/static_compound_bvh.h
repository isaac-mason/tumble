#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/utils/bvh.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// a binary bvh over a static compound's children, split at the spatial median of their centres; building it reorders
// the children so every leaf holds a contiguous range

namespace static_compound_bvh {

/**
 * the deepest a leaf is built (the root at 0); a node there is a leaf whatever its child count. a walk
 * of a tree this deep never holds more than MAX_DEPTH + 1 nodes on its stack.
 */
constexpr uint32_t MAX_DEPTH = 127;
/** the stack a walk of the tree needs */
constexpr uint32_t STACK_SIZE = MAX_DEPTH + 1;

}  // namespace static_compound_bvh

/** settings for bvh construction */
struct StaticCompoundBvhBuildSettings {
    /** maximum children per leaf node */
    uint32_t max_leaf_children = 4;
};

namespace static_compound_bvh {

/** get child start index (leaf only) */
inline uint32_t node_child_start(const BvhNode *nodes, int32_t node) { return (uint32_t)nodes[node].right_or_start; }

/** get child count (leaf only). decodes from negative flag. */
inline uint32_t node_child_count(const BvhNode *nodes, int32_t node) { return (uint32_t)(-(nodes[node].axis_or_count + 1)); }

/**
 * builds the bvh over the children into `out` (pre-order, the root at 0; empty when there are no
 * children), reordering the children in place.
 */
void build(collections::Array<BvhNode> &out, collections::Array<CompoundShape::Child> &children, const StaticCompoundBvhBuildSettings &settings);

}  // namespace static_compound_bvh

struct StaticCompoundBvhStats {
    uint32_t node_count;
    uint32_t leaf_count;
    uint32_t min_depth;
    uint32_t max_depth;
    float avg_depth;
    uint32_t total_children;
};

namespace static_compound_bvh {

/** compute depth statistics of the bvh tree */
StaticCompoundBvhStats stats(const BvhNode *nodes, uint32_t node_count);

}  // namespace static_compound_bvh

TUMBLE_NAMESPACE_END
