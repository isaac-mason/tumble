#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/broadphase/body_visitor.h"
#include "tumble/physics/broadphase/quad_tree.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/layers.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

namespace broadphase {

using body_visitor::BodyVisitor;

struct Broadphase {
    /** one per broadphase layer */
    collections::Vec<quad_tree::QuadTree> trees;
    /** the round-robin cursor of optimize */
    uint32_t next_tree_to_optimize;
};

Broadphase init(const layers::Layers &layers);
void release(Broadphase &broadphase);

void add_body(Broadphase &broadphase, rigid_body::RigidBody *body, const layers::Layers &layers);
void remove_body(Broadphase &broadphase, rigid_body::RigidBody *body);

/** rebuilds the first dirty tree from the cursor on, at most one a step; a clean tree costs one check */
void optimize(Broadphase &broadphase, const bodies::Bodies &bodies);

/** updates a body's aabb in the broadphase; returns true iff the body escaped its fat leaf aabb */
bool update_body(Broadphase &broadphase, const rigid_body::RigidBody *body);

/** removes and re-adds a body in the broadphase when its layer changes */
void reinsert_body(Broadphase &broadphase, rigid_body::RigidBody *body, const layers::Layers &layers);

/** the body's fat box: the box its leaf holds, which the persistent pairs key on */
Box3f &read_fat_bounds(Box3f &out, const Broadphase &broadphase, const rigid_body::RigidBody &body);

/** every body of one layer's tree whose fat box overlaps the box: no tight re-test, no filtering (pair discovery) */
void intersect_fat_leaves(Broadphase &broadphase, const bodies::Bodies &bodies, uint32_t broadphase_layer, const Box3f &aabb, BodyVisitor &visitor);

/** finds bodies with aabbs that intersect the given ray */
void cast_ray(Broadphase &broadphase, const bodies::Bodies &bodies, const cast_ray::Rayf &ray, const filter::Filter &query_filter,
              BodyVisitor &visitor);

/** finds bodies with aabbs that intersect the given aabb */
void intersect_aabb(Broadphase &broadphase, const bodies::Bodies &bodies, const Box3f &aabb, const filter::Filter &query_filter,
                    BodyVisitor &visitor);

/** finds bodies with aabbs that contain the given point */
void intersect_point(Broadphase &broadphase, const bodies::Bodies &bodies, Vec3f point, const filter::Filter &query_filter,
                     BodyVisitor &visitor);

/** finds bodies with aabbs that intersect the given swept aabb */
void cast_aabb(Broadphase &broadphase, const bodies::Bodies &bodies, const Box3f &bounds, Vec3f displacement,
               const filter::Filter &query_filter, BodyVisitor &visitor);

/** the bounds of all the trees in the broadphase */
Box3f &bounds(Box3f &out, const Broadphase &broadphase);

}  // namespace broadphase

TUMBLE_NAMESPACE_END
