#pragma once

// crashcat's static-compound-bvh.ts: a binary bvh over a static compound's children, split at the
// spatial median of the child centres. building it reorders the children so that every leaf holds a
// contiguous range of them.

#include <stdint.h>

#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/utils/bvh.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

namespace static_compound_bvh {

/**
 * the deepest a leaf is built (the root at 0); a node there is a leaf whatever its child count. a walk
 * of a tree this deep never holds more than MAX_DEPTH + 1 nodes on its stack.
 */
constexpr uint32_t MAX_DEPTH = 127;
/** the stack a walk of the tree needs */
constexpr uint32_t STACK_SIZE = MAX_DEPTH + 1;

/** settings for bvh construction */
struct BvhBuildSettings {
    /** maximum children per leaf node */
    uint32_t max_leaf_children = 4;
};

/** get child start index (leaf only) */
inline uint32_t node_child_start(const bvh::Node *nodes, int32_t node) { return (uint32_t)nodes[node].right_or_start; }

/** get child count (leaf only). decodes from negative flag. */
inline uint32_t node_child_count(const bvh::Node *nodes, int32_t node) { return (uint32_t)(-(nodes[node].axis_or_count + 1)); }

/**
 * builds the bvh over the children into `out` (pre-order, the root at 0; empty when there are no
 * children), reordering the children in place. false (error set) when the memory cannot grow.
 */
bool build(collections::Vec<bvh::Node> &out, collections::Vec<compound::CompoundShapeChild> &children, const BvhBuildSettings &settings);

struct StaticCompoundBvhStats {
    uint32_t node_count;
    uint32_t leaf_count;
    uint32_t min_depth;
    uint32_t max_depth;
    float avg_depth;
    uint32_t total_children;
};

/** compute depth statistics of the bvh tree */
StaticCompoundBvhStats stats(const bvh::Node *nodes, uint32_t node_count);

}  // namespace static_compound_bvh

TUMBLE_NAMESPACE_END
