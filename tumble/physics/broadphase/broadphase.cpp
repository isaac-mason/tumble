#include "tumble/physics/broadphase/broadphase.h"

#include "tumble/math/aabox.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace broadphase {

bool init(Broadphase &out, const layers::Layers &layers) {
    out = {};
    if (!out.trees.reserve(layers.broadphase_layers)) return false;
    for (uint32_t i = 0; i < layers.broadphase_layers; i++) {
        quad_tree::QuadTree tree = quad_tree::create();
        if (tree.root_node == quad_tree::INVALID_NODE_INDEX) {
            quad_tree::release(tree);
            release(out);
            return false;
        }
        out.trees.push(tree);
    }
    return true;
}

void release(Broadphase &broadphase) {
    for (uint32_t i = 0; i < broadphase.trees.length; i++) quad_tree::release(broadphase.trees[i]);
    broadphase.trees.release();
    broadphase.next_tree_to_optimize = 0;
}

void add_body(Broadphase &broadphase, rigid_body::RigidBody *body, const layers::Layers &layers) {
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

void remove_body(Broadphase &broadphase, rigid_body::RigidBody *body) {
    if (body->broadphase_layer == -1) return;

    if (body->broadphase_node != -1) quad_tree::remove(broadphase.trees[body->broadphase_layer], body);

    body->broadphase_layer = -1;
    body->broadphase_node = -1;
}

void optimize(Broadphase &broadphase, const bodies::Bodies &bodies) {
    const uint32_t n = broadphase.trees.length;
    for (uint32_t i = 0; i < n; i++) {
        const uint32_t idx = (broadphase.next_tree_to_optimize + i) % n;
        quad_tree::QuadTree &layer_tree = broadphase.trees[idx];
        if (layer_tree.dirty) {
            quad_tree::rebuild(layer_tree, bodies);
            broadphase.next_tree_to_optimize = (idx + 1) % n;
            return;
        }
    }
}

bool update_body(Broadphase &broadphase, const rigid_body::RigidBody *body) {
    if (body->broadphase_node == -1 || body->broadphase_layer == -1) return false;
    // an escape is a move event: the caller marks the body moved (pairs::mark_moved) so it
    // rediscovers its overlaps in the next find_colliding_pairs.
    return quad_tree::update(broadphase.trees[body->broadphase_layer], body);
}

void reinsert_body(Broadphase &broadphase, rigid_body::RigidBody *body, const layers::Layers &layers) {
    remove_body(broadphase, body);
    add_body(broadphase, body, layers);
}

Box3f &read_fat_bounds(Box3f &out, const Broadphase &broadphase, const rigid_body::RigidBody &body) {
    return quad_tree::read_body_bounds(out, broadphase.trees[(uint32_t)body.broadphase_layer], body);
}

void intersect_fat_leaves(Broadphase &broadphase, const bodies::Bodies &bodies, uint32_t broadphase_layer, const Box3f &aabb, BodyVisitor &visitor) {
    quad_tree::intersect_aabb_fat_leaves(bodies, broadphase.trees[broadphase_layer], aabb, visitor);
}

void cast_ray(Broadphase &broadphase, const bodies::Bodies &bodies, const cast_ray::Rayf &ray, const filter::Filter &query_filter,
              BodyVisitor &visitor) {
    for (uint32_t broadphase_layer = 0; broadphase_layer < broadphase.trees.length; broadphase_layer++) {
        if (!filter::filter_broadphase_layer(query_filter, broadphase_layer)) continue;

        quad_tree::cast_ray(bodies, broadphase.trees[broadphase_layer], ray, query_filter, visitor);

        if (visitor.should_exit) break;
    }
}

void intersect_aabb(Broadphase &broadphase, const bodies::Bodies &bodies, const Box3f &aabb, const filter::Filter &query_filter,
                    BodyVisitor &visitor) {
    for (uint32_t broadphase_layer = 0; broadphase_layer < broadphase.trees.length; broadphase_layer++) {
        if (!filter::filter_broadphase_layer(query_filter, broadphase_layer)) continue;

        quad_tree::intersect_aabb(bodies, broadphase.trees[broadphase_layer], aabb, query_filter, visitor);

        if (visitor.should_exit) break;
    }
}

void intersect_point(Broadphase &broadphase, const bodies::Bodies &bodies, Vec3f point, const filter::Filter &query_filter,
                     BodyVisitor &visitor) {
    for (uint32_t broadphase_layer = 0; broadphase_layer < broadphase.trees.length; broadphase_layer++) {
        if (!filter::filter_broadphase_layer(query_filter, broadphase_layer)) continue;

        quad_tree::intersect_point(bodies, broadphase.trees[broadphase_layer], point, query_filter, visitor);

        if (visitor.should_exit) break;
    }
}

void cast_aabb(Broadphase &broadphase, const bodies::Bodies &bodies, const Box3f &bounds, Vec3f displacement,
               const filter::Filter &query_filter, BodyVisitor &visitor) {
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
