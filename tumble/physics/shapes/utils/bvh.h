#pragma once

#include <stdint.h>

#include "tumble/math/aabox.h"
#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// binary bvh nodes, NODE_STRIDE words each: [min_x, min_y, min_z, max_x, max_y, max_z, right_or_start, axis_or_count].
// an internal node holds its right child's index and the split axis (0 to 2), a leaf its first primitive and -(count +
// 1). nodes are addressed by index and a left child always follows its parent
namespace bvh {

/** number of 32-bit words per node */
constexpr uint32_t NODE_STRIDE = 8;

// offsets within a node
constexpr uint32_t NODE_MIN_X = 0;
constexpr uint32_t NODE_MIN_Y = 1;
constexpr uint32_t NODE_MIN_Z = 2;
constexpr uint32_t NODE_MAX_X = 3;
constexpr uint32_t NODE_MAX_Y = 4;
constexpr uint32_t NODE_MAX_Z = 5;

/** internal: right child index, leaf: data start index */
constexpr uint32_t NODE_RIGHT_OR_START = 6;

/** internal: split axis (0-2), leaf: negative encoded count */
constexpr uint32_t NODE_AXIS_OR_COUNT = 7;

struct Node {
    Box3f bounds;
    int32_t right_or_start;
    int32_t axis_or_count;
};

static_assert(sizeof(Node) == NODE_STRIDE * 4);
static_assert(__builtin_offsetof(Node, bounds) == NODE_MIN_X * 4);
static_assert(__builtin_offsetof(Node, right_or_start) == NODE_RIGHT_OR_START * 4);
static_assert(__builtin_offsetof(Node, axis_or_count) == NODE_AXIS_OR_COUNT * 4);

/** check if node is a leaf */
inline bool node_is_leaf(const Node *nodes, int32_t node) { return nodes[node].axis_or_count < 0; }

/** get left child index. left child is always contiguous (node + 1). */
inline int32_t node_left(int32_t node) { return node + 1; }

/** get right child index (internal only) */
inline int32_t node_right(const Node *nodes, int32_t node) { return nodes[node].right_or_start; }

/** get split axis (internal only): 0=x, 1=y, 2=z */
inline int32_t node_split_axis(const Node *nodes, int32_t node) { return nodes[node].axis_or_count; }

/** copy bounds into out */
inline Box3f &node_get_bounds(Box3f &out, const Node *nodes, int32_t node) {
    out = nodes[node].bounds;
    return out;
}

/** get center of node bounds */
inline Vec3f node_get_center(const Node *nodes, int32_t node) { return aabox::center(aabox::load(nodes[node].bounds)); }

/** test ray-aabb intersection using node bounds directly: the slab interval overlaps [near, far] */
bool node_intersects_ray(const Node *nodes, int32_t node, Vec3f origin, Vec3f direction, float near, float far);

/** test aabb-aabb intersection using node bounds directly, touching faces included */
inline bool node_intersects_box(const Node *nodes, int32_t node, const AABox &box) { return aabox::intersects(aabox::load(nodes[node].bounds), box); }

}  // namespace bvh

TUMBLE_NAMESPACE_END
