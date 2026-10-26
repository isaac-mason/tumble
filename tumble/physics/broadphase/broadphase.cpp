#include "tumble/physics/broadphase/broadphase.h"

#include "tumble/math/aabox.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace broadphase {

void init(Broadphase &out, const Layers &layers) {
    out = {};
    out.trees.reserve(layers.broadphase_layers);
    for (uint32_t i = 0; i < layers.broadphase_layers; i++) out.trees.push(quad_tree::create());
}

void add_body(Broadphase &broadphase, RigidBody *body, const Layers &layers) {
    const uint32_t object_layer = body->object_layer;

    if (object_layer >= layers.object_layers) {
        TUMBLE_ASSERT(false);
        return;
    }
    const uint32_t broadphase_layer = layers.object_layer_to_broadphase_layer[object_layer];

    // sets the body's broadphase_node
    quad_tree::add(broadphase.trees[broadphase_layer], body);
    body->broadphase_layer = (int32_t)broadphase_layer;
}

void remove_body(Broadphase &broadphase, RigidBody *body) {
    if (body->broadphase_layer == -1) return;

    if (body->broadphase_node != -1) quad_tree::remove(broadphase.trees[body->broadphase_layer], body);

    body->broadphase_layer = -1;
    body->broadphase_node = -1;
}

void optimize(Broadphase &broadphase, const Bodies &bodies) {
    const uint32_t n = broadphase.trees.length;
    for (uint32_t i = 0; i < n; i++) {
        const uint32_t idx = (broadphase.next_tree_to_optimize + i) % n;
        QuadTree &layer_tree = broadphase.trees[idx];
        if (layer_tree.dirty) {
            quad_tree::rebuild(layer_tree, bodies);
            broadphase.next_tree_to_optimize = (idx + 1) % n;
            return;
        }
    }
}

bool update_body(Broadphase &broadphase, const RigidBody *body) {
    if (body->broadphase_node == -1 || body->broadphase_layer == -1) return false;
    // an escape is a move event: the caller marks the body moved (pairs::mark_moved) so it
    // rediscovers its overlaps in the next find_colliding_pairs.
    return quad_tree::update(broadphase.trees[body->broadphase_layer], body);
}

void reinsert_body(Broadphase &broadphase, RigidBody *body, const Layers &layers) {
    remove_body(broadphase, body);
    add_body(broadphase, body, layers);
}

Box3f &read_fat_bounds(Box3f &out, const Broadphase &broadphase, const RigidBody &body) {
    return quad_tree::read_body_bounds(out, broadphase.trees[(uint32_t)body.broadphase_layer], body);
}

void intersect_fat_leaves(Broadphase &broadphase, const Bodies &bodies, uint32_t broadphase_layer, const Box3f &aabb, BodyVisitor &visitor) {
    quad_tree::intersect_aabb_fat_leaves(bodies, broadphase.trees[broadphase_layer], aabb, visitor);
}

void cast_ray(Broadphase &broadphase, const Bodies &bodies, const Rayf &ray, const Filter &query_filter,
              BodyVisitor &visitor) {
    for (uint32_t broadphase_layer = 0; broadphase_layer < broadphase.trees.length; broadphase_layer++) {
        if (!filter::filter_broadphase_layer(query_filter, broadphase_layer)) continue;

        quad_tree::cast_ray(bodies, broadphase.trees[broadphase_layer], ray, query_filter, visitor);

        if (visitor.should_exit) break;
    }
}

void intersect_aabb(Broadphase &broadphase, const Bodies &bodies, const Box3f &aabb, const Filter &query_filter,
                    BodyVisitor &visitor) {
    for (uint32_t broadphase_layer = 0; broadphase_layer < broadphase.trees.length; broadphase_layer++) {
        if (!filter::filter_broadphase_layer(query_filter, broadphase_layer)) continue;

        quad_tree::intersect_aabb(bodies, broadphase.trees[broadphase_layer], aabb, query_filter, visitor);

        if (visitor.should_exit) break;
    }
}

void intersect_sphere(Broadphase &broadphase, const Bodies &bodies, Vec3f center, float radius, const Filter &query_filter,
                      BodyVisitor &visitor) {
    for (uint32_t broadphase_layer = 0; broadphase_layer < broadphase.trees.length; broadphase_layer++) {
        if (!filter::filter_broadphase_layer(query_filter, broadphase_layer)) continue;

        quad_tree::intersect_sphere(bodies, broadphase.trees[broadphase_layer], center, radius, query_filter, visitor);

        if (visitor.should_exit) break;
    }
}

void intersect_oriented_box(Broadphase &broadphase, const Bodies &bodies, const OrientedBox &box, const Filter &query_filter,
                            BodyVisitor &visitor) {
    for (uint32_t broadphase_layer = 0; broadphase_layer < broadphase.trees.length; broadphase_layer++) {
        if (!filter::filter_broadphase_layer(query_filter, broadphase_layer)) continue;

        quad_tree::intersect_oriented_box(bodies, broadphase.trees[broadphase_layer], box, query_filter, visitor);

        if (visitor.should_exit) break;
    }
}

void intersect_point(Broadphase &broadphase, const Bodies &bodies, Vec3f point, const Filter &query_filter,
                     BodyVisitor &visitor) {
    for (uint32_t broadphase_layer = 0; broadphase_layer < broadphase.trees.length; broadphase_layer++) {
        if (!filter::filter_broadphase_layer(query_filter, broadphase_layer)) continue;

        quad_tree::intersect_point(bodies, broadphase.trees[broadphase_layer], point, query_filter, visitor);

        if (visitor.should_exit) break;
    }
}

void cast_aabb(Broadphase &broadphase, const Bodies &bodies, const Box3f &bounds, Vec3f displacement,
               const Filter &query_filter, BodyVisitor &visitor) {
    for (uint32_t broadphase_layer = 0; broadphase_layer < broadphase.trees.length; broadphase_layer++) {
        if (!filter::filter_broadphase_layer(query_filter, broadphase_layer)) continue;

        quad_tree::cast_aabb(bodies, broadphase.trees[broadphase_layer], bounds, displacement, query_filter, visitor);

        if (visitor.should_exit) break;
    }
}

Box3f &bounds(Box3f &out, const Broadphase &broadphase) {
    AABox all = aabox::empty();
    for (uint32_t i = 0; i < broadphase.trees.length; i++) {
        Box3f tree_bounds;
        all = aabox::union_(all, aabox::load(quad_tree::bounds(tree_bounds, broadphase.trees[i])));
    }
    return aabox::store(out, all);
}

}  // namespace broadphase

TUMBLE_NAMESPACE_END
