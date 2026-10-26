#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/simd.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/broadphase/body_visitor.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/filter.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

/**
 * leaves are fat boxes, a body's box grown by the expansion margin and replaced only when the body escapes it, which the
 * persistent pairs key on. a changed body widens its ancestors and marks the path; a rebuild re-partitions the marked
 * part and keeps the unmarked subtrees whole
 */
struct QuadTree {
    /** a body's index, or a node's index with quad_tree::IS_NODE set */
    using NodeId = uint32_t;

      // namespace quad_tree

    /**
     * four children, their bounds per axis so a test against all four is one 4-wide compare per bound. 124 bytes of
     * fields, aligned to 16 so every node's bounds load as whole lanes
     */
    struct alignas(16) Node {
        /** bounds of the four children, per axis; an empty child is inverted at quad_tree::LARGE_FLOAT */
        float bounds_min_x[4];
        float bounds_min_y[4];
        float bounds_min_z[4];
        float bounds_max_x[4];
        float bounds_max_y[4];
        float bounds_max_z[4];
        /** a child node or body per slot, quad_tree::INVALID_NODE_ID when empty */
        NodeId child_node_id[4];
        uint32_t parent_node_index;
        /** this part of the tree changed since the last rebuild; if not, a rebuild grafts it as one unit */
        uint32_t is_changed;
    };

    struct BuildEntry {
        uint32_t node_index;
        int32_t child_index;
        int32_t split[5];
        uint32_t depth;
        Vec3f node_bounds_min;
        Vec3f node_bounds_max;
    };

    /** the node pool; free slots are on free_nodes */
    collections::Array<Node> nodes;
    collections::Array<uint32_t> free_nodes;
    /** the nodes a rebuild replaces, freed once the new tree is in */
    collections::Array<uint32_t> free_node_batch;
    uint32_t root_node;
    uint32_t num_bodies;
    /** any add, remove or escape since the last rebuild */
    bool dirty;
    /** the fat-leaf margin, world units */
    float expansion_margin;

    // traversal and rebuild scratch, grown and reused
    collections::Array<NodeId> node_stack;
    collections::Array<float> fraction_stack;
    collections::Array<NodeId> build_ids;
    collections::Array<Vec3f> build_centers;
    collections::Array<BuildEntry> build_stack;
};

namespace quad_tree {

constexpr QuadTree::NodeId IS_NODE = 0x80000000u;
constexpr uint32_t INVALID_NODE_INDEX = 0xffffffffu;
constexpr QuadTree::NodeId INVALID_NODE_ID = 0xffffffffu;

inline bool is_valid(QuadTree::NodeId id) { return id != INVALID_NODE_ID; }
inline bool is_body(QuadTree::NodeId id) { return (id & IS_NODE) == 0; }
inline bool is_node(QuadTree::NodeId id) { return (id & IS_NODE) != 0 && id != INVALID_NODE_ID; }
inline uint32_t node_index(QuadTree::NodeId id) { return id & ~IS_NODE; }
inline QuadTree::NodeId from_node_index(uint32_t index) { return index | IS_NODE; }

/** the bounds of an empty child: no test ever reaches it */
constexpr float LARGE_FLOAT = 1.0e30f;

/** the levels of a freshly built tree born changed, so they are rebuilt every time */
constexpr uint32_t MAX_DEPTH_MARK_CHANGED = 5;

}
static_assert(sizeof(QuadTree::Node) == 128);

namespace quad_tree {

/** an empty tree */
QuadTree create();

/** the body's fat box as a leaf at the root, or in a new root over the old one; the body's broadphase_node is set */
void add(QuadTree &tree, RigidBody *body);

/** empties the body's slot and clears its broadphase_node; ancestors keep their bounds until the rebuild */
void remove(QuadTree &tree, RigidBody *body);

/**
 * true iff the body escaped its fat box: the box is then replaced by the body's, grown by the margin (it may shrink),
 * and the ancestors widened to fit and marked changed. false when the fat box still holds the body.
 */
bool update(QuadTree &tree, const RigidBody *body);

/** the changed part rebuilt, then the nodes it replaced freed */
void rebuild(QuadTree &tree, const Bodies &bodies);

/** the bounds of every body's fat box; empty when there are none */
Box3f &bounds(Box3f &out, const QuadTree &tree);

/** the body's fat box, from its slot */
Box3f &read_body_bounds(Box3f &out, const QuadTree &tree, const RigidBody &body);

/** overlap walk against the fat boxes, no tight re-test and no filtering: the persistent pairs' discovery */
void intersect_aabb_fat_leaves(const Bodies &bodies, QuadTree &tree, const Box3f &aabb, BodyVisitor &visitor);

void intersect_aabb(const Bodies &bodies, QuadTree &tree, const Box3f &aabb, const Filter &query_filter, BodyVisitor &visitor);

void intersect_point(const Bodies &bodies, QuadTree &tree, Vec3f point, const Filter &query_filter, BodyVisitor &visitor);

/** the bodies whose box is within radius of the centre, faces included */
void intersect_sphere(const Bodies &bodies, QuadTree &tree, Vec3f center, float radius, const Filter &query_filter,
                      BodyVisitor &visitor);

/** the bodies whose box overlaps the oriented box */
void intersect_oriented_box(const Bodies &bodies, QuadTree &tree, const OrientedBox &box, const Filter &query_filter,
                            BodyVisitor &visitor);

/** visit every body in the tree: no filtering, no aabb tests */
void walk(const Bodies &bodies, QuadTree &tree, BodyVisitor &visitor);

void cast_ray(const Bodies &bodies, QuadTree &tree, const Rayf &ray, const Filter &query_filter, BodyVisitor &visitor);

void cast_aabb(const Bodies &bodies, QuadTree &tree, const Box3f &cast_bounds, Vec3f displacement, const Filter &query_filter,
               BodyVisitor &visitor);

}  // namespace quad_tree

TUMBLE_NAMESPACE_END
