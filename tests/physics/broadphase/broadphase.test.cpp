#include "tumble/physics/broadphase/broadphase.h"

#include "tumble/math/vec3f.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/layers.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "test.h"
#include "test_math.h"
#include "tumble/util/mem.h"

using rigid_body::RigidBody;

namespace {

/** a moving and a not-moving layer, each with its own broadphase layer */
struct World {
    bodies::Bodies bodies;
    layers::Layers layers;
    broadphase::Broadphase broadphase;
    filter::Filter filter;
    uint32_t broadphase_moving;
    uint32_t broadphase_not_moving;
    uint32_t moving;
    uint32_t not_moving;
};

World make_world() {
    dispatch::init();
    World world = {};
    world.bodies = bodies::init();
    world.broadphase_moving = layers::add_broadphase_layer(world.layers);
    world.broadphase_not_moving = layers::add_broadphase_layer(world.layers);
    world.moving = layers::add_object_layer(world.layers, world.broadphase_moving);
    world.not_moving = layers::add_object_layer(world.layers, world.broadphase_not_moving);
    layers::enable_collision(world.layers, world.moving, world.moving);
    layers::enable_collision(world.layers, world.moving, world.not_moving);
    world.broadphase = broadphase::init(world.layers);
    world.filter = filter::create(world.layers);
    return world;
}

void release_world(World &world) {
    filter::release(world.filter);
    broadphase::release(world.broadphase);
    bodies::release(world.bodies);
    layers::release(world.layers);
}

/** a dynamic box body filling the given world-space box, added to the broadphase */
RigidBody *make_body(World &world, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z, uint32_t object_layer,
                     uint32_t collision_groups = 0xffffffffu, uint32_t collision_mask = 0xffffffffu) {
    const Float3 half_extents = {(max_x - min_x) * 0.5f, (max_y - min_y) * 0.5f, (max_z - min_z) * 0.5f};
    const Float3 center = {(min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f, (min_z + max_z) * 0.5f};
    RigidBody *body = rigid_body::create(world.bodies, world.layers,
                                         {.shape = box::create({.half_extents = half_extents}),
                                          .object_layer = object_layer,
                                          .motion_type = motion_type::MOTION_DYNAMIC,
                                          .position = center,
                                          .collision_groups = collision_groups,
                                          .collision_mask = collision_mask});
    broadphase::add_body(world.broadphase, body, world.layers);
    return body;
}

constexpr uint32_t MAX_HITS = 32;

/** the bodies visited, in visit order. */
struct Hits {
    RigidBody *bodies[MAX_HITS];
    uint32_t count;
    /** when set, each visit sets the early-out to the body's entry fraction along this ray (a closest-hit collector) */
    const cast_ray::Rayf *closest_along;
    /** when set, exit after the first visit */
    bool exit_after_first;
};

void record_hit(body_visitor::BodyVisitor *visitor, RigidBody *body) {
    Hits *hits = static_cast<Hits *>(visitor->context);
    if (hits->count < MAX_HITS) hits->bodies[hits->count] = body;
    hits->count++;
    if (hits->closest_along != nullptr) {
        const cast_ray::Rayf &ray = *hits->closest_along;
        visitor->early_out_fraction = (body->aabb.min.x - vec3f::x(ray.origin)) / (vec3f::x(ray.direction) * ray.length);
    }
    if (hits->exit_after_first) visitor->should_exit = true;
}

body_visitor::BodyVisitor visitor_for(Hits &hits) {
    util::zero(&hits, sizeof(Hits));
    return body_visitor::create(record_hit, &hits);
}

bool contains(const Hits &hits, const RigidBody *body) {
    for (uint32_t i = 0; i < hits.count && i < MAX_HITS; i++) {
        if (hits.bodies[i] == body) return true;
    }
    return false;
}

/** whether a tree holds the body: a walk of it reaches the body. */
bool tree_holds(World &world, uint32_t broadphase_layer, const RigidBody *body) {
    Hits hits;
    body_visitor::BodyVisitor visitor = visitor_for(hits);
    quad_tree::walk(world.bodies, world.broadphase.trees[broadphase_layer], visitor);
    return contains(hits, body);
}

const Box3f EVERYTHING = {{-100, -100, -100}, {100, 100, 100}};

}  // namespace

void test_broadphase_add_to_layer_trees(TestCtx &ctx) {
    World world = make_world();
    CHECK(ctx, world.broadphase.trees.length == 2 && world.broadphase.next_tree_to_optimize == 0);

    RigidBody *a = make_body(world, 0, 0, 0, 1, 1, 1, world.moving);
    RigidBody *b = make_body(world, 2, 0, 0, 3, 1, 1, world.moving);
    RigidBody *c = make_body(world, 4, 0, 0, 5, 1, 1, world.not_moving);

    // each body is in its object layer's broadphase tree, in the leaf it holds, and in no other
    CHECK(ctx, a->broadphase_layer == (int32_t)world.broadphase_moving && b->broadphase_layer == (int32_t)world.broadphase_moving);
    CHECK(ctx, c->broadphase_layer == (int32_t)world.broadphase_not_moving);
    CHECK(ctx, a->broadphase_node != -1 && b->broadphase_node != -1 && c->broadphase_node != -1);
    Box3f fat, expected;
    aabox::store(expected, aabox::expand_by_margin(aabox::load(c->aabb), 0.05f));
    CHECK(ctx, equal_boxes(broadphase::read_fat_bounds(fat, world.broadphase, *c), expected));
    CHECK(ctx, tree_holds(world, world.broadphase_moving, a) && tree_holds(world, world.broadphase_moving, b));
    CHECK(ctx, !tree_holds(world, world.broadphase_moving, c));
    CHECK(ctx, tree_holds(world, world.broadphase_not_moving, c) && !tree_holds(world, world.broadphase_not_moving, a));

    release_world(world);
}

void test_broadphase_optimize_round_robin(TestCtx &ctx) {
    World world = make_world();
    make_body(world, 0, 0, 0, 1, 1, 1, world.moving);
    make_body(world, 2, 0, 0, 3, 1, 1, world.moving);
    make_body(world, 4, 0, 0, 5, 1, 1, world.not_moving);
    quad_tree::QuadTree &moving_tree = world.broadphase.trees[world.broadphase_moving];
    quad_tree::QuadTree &not_moving_tree = world.broadphase.trees[world.broadphase_not_moving];
    CHECK(ctx, moving_tree.dirty && not_moving_tree.dirty);

    // one rebuild per call, from the cursor on
    broadphase::optimize(world.broadphase, world.bodies);
    CHECK(ctx, !moving_tree.dirty && not_moving_tree.dirty && world.broadphase.next_tree_to_optimize == 1);
    broadphase::optimize(world.broadphase, world.bodies);
    CHECK(ctx, !moving_tree.dirty && !not_moving_tree.dirty && world.broadphase.next_tree_to_optimize == 0);

    // clean trees are left alone
    broadphase::optimize(world.broadphase, world.bodies);
    CHECK(ctx, world.broadphase.next_tree_to_optimize == 0);

    // the cursor goes first: with it at the not-moving tree, a dirty moving tree waits its turn
    world.broadphase.next_tree_to_optimize = 1;
    make_body(world, 6, 0, 0, 7, 1, 1, world.moving);
    make_body(world, 8, 0, 0, 9, 1, 1, world.not_moving);
    broadphase::optimize(world.broadphase, world.bodies);
    CHECK(ctx, moving_tree.dirty && !not_moving_tree.dirty && world.broadphase.next_tree_to_optimize == 0);

    release_world(world);
}

void test_broadphase_layer_filter(TestCtx &ctx) {
    World world = make_world();
    RigidBody *moving = make_body(world, 0, 0, 0, 1, 1, 1, world.moving);
    RigidBody *not_moving = make_body(world, 5, 0, 0, 6, 1, 1, world.not_moving);
    RigidBody *overlapping = make_body(world, 0, 0, 0, 2, 2, 2, world.not_moving);
    Hits hits;

    // every layer enabled: the ray along x sees all three
    const cast_ray::Rayf ray = {vec3f::make(-1.0f, 0.5f, 0.5f), vec3f::make(1.0f, 0.0f, 0.0f), 100.0f};
    body_visitor::BodyVisitor visitor = visitor_for(hits);
    broadphase::cast_ray(world.broadphase, world.bodies, ray, world.filter, visitor);
    CHECK(ctx, hits.count == 3 && contains(hits, moving) && contains(hits, not_moving) && contains(hits, overlapping));

    // only the moving layer: its body alone, by every query
    filter::disable_object_layer(world.filter, world.layers, world.not_moving);
    CHECK(ctx, !filter::filter_broadphase_layer(world.filter, world.broadphase_not_moving));
    visitor = visitor_for(hits);
    broadphase::cast_ray(world.broadphase, world.bodies, ray, world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == moving);
    visitor = visitor_for(hits);
    broadphase::intersect_aabb(world.broadphase, world.bodies, EVERYTHING, world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == moving);
    visitor = visitor_for(hits);
    broadphase::intersect_point(world.broadphase, world.bodies, vec3f::make(0.5f, 0.5f, 0.5f), world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == moving);
    visitor = visitor_for(hits);
    broadphase::cast_aabb(world.broadphase, world.bodies, Box3f{{-3.0f, 0.25f, 0.25f}, {-2.5f, 0.75f, 0.75f}}, vec3f::make(20.0f, 0.0f, 0.0f), world.filter,
                          visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == moving);

    // only the not-moving layer
    filter::disable_all_layers(world.filter, world.layers);
    filter::enable_broadphase_layer(world.filter, world.layers, world.broadphase_not_moving);
    visitor = visitor_for(hits);
    broadphase::intersect_aabb(world.broadphase, world.bodies, EVERYTHING, world.filter, visitor);
    CHECK(ctx, hits.count == 2 && contains(hits, not_moving) && contains(hits, overlapping));

    // the tree is enabled but the object layer is not: its bodies are still filtered out
    world.filter.enabled_object_layers[world.not_moving] = 0;
    visitor = visitor_for(hits);
    broadphase::intersect_aabb(world.broadphase, world.bodies, EVERYTHING, world.filter, visitor);
    CHECK(ctx, hits.count == 0);

    release_world(world);
}

void test_broadphase_groups_and_mask(TestCtx &ctx) {
    World world = make_world();

    // three bodies at one place with different groups and masks
    RigidBody *first = make_body(world, 0, 0, 0, 2, 2, 2, world.moving, 0b0001, 0b0010);
    RigidBody *second = make_body(world, 0, 0, 0, 2, 2, 2, world.moving, 0b0010, 0b0001);
    RigidBody *third = make_body(world, 0, 0, 0, 2, 2, 2, world.moving, 0b0100, 0b1000);
    world.filter.collision_groups = 0b0001;
    world.filter.collision_mask = 0b0010;

    // the query's group is in the second's mask and the second's group in the query's mask; the
    // first and third fail one way or the other
    CHECK(ctx, filter::should_pair_collide(0b0001, 0b0010, second->collision_groups, second->collision_mask));
    CHECK(ctx, !filter::should_pair_collide(0b0001, 0b0010, first->collision_groups, first->collision_mask));
    CHECK(ctx, !filter::should_pair_collide(0b0001, 0b0010, third->collision_groups, third->collision_mask));

    Hits hits;
    body_visitor::BodyVisitor visitor = visitor_for(hits);
    broadphase::intersect_point(world.broadphase, world.bodies, vec3f::make(1.0f, 1.0f, 1.0f), world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == second);
    visitor = visitor_for(hits);
    broadphase::intersect_aabb(world.broadphase, world.bodies, EVERYTHING, world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == second);
    visitor = visitor_for(hits);
    broadphase::cast_ray(world.broadphase, world.bodies, cast_ray::Rayf{vec3f::make(-5.0f, 1.0f, 1.0f), vec3f::make(1.0f, 0.0f, 0.0f), 10.0f}, world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == second);

    // each in the other's mask
    RigidBody *a = make_body(world, 10, 0, 0, 12, 2, 2, world.moving, 0b0001, 0b0010);
    world.filter.collision_groups = 0b0010;
    world.filter.collision_mask = 0b0001;
    visitor = visitor_for(hits);
    broadphase::intersect_aabb(world.broadphase, world.bodies, {{9, 0, 0}, {13, 2, 2}}, world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == a);

    release_world(world);
}

namespace {

/** a body filter's context: the body it rejects, and the bodies it was asked about. */
struct RejectOne {
    const RigidBody *rejected;
    uint32_t calls;
};

bool reject_one(const RigidBody *body, void *context) {
    RejectOne *reject = static_cast<RejectOne *>(context);
    reject->calls++;
    return body != reject->rejected;
}

}  // namespace

void test_broadphase_body_filter(TestCtx &ctx) {
    World world = make_world();
    RigidBody *a = make_body(world, 0, 0, 0, 1, 1, 1, world.moving);
    RigidBody *b = make_body(world, 2, 0, 0, 3, 1, 1, world.moving);
    RigidBody *c = make_body(world, 4, 0, 0, 5, 1, 1, world.not_moving);
    make_body(world, 0, 10, 0, 1, 11, 1, world.moving);

    // consulted with its context for each body the query reaches, and its rejection holds
    RejectOne reject = {b, 0};
    world.filter.body_filter = reject_one;
    world.filter.body_filter_context = &reject;
    Hits hits;
    body_visitor::BodyVisitor visitor = visitor_for(hits);
    broadphase::cast_ray(world.broadphase, world.bodies, cast_ray::Rayf{vec3f::make(-1.0f, 0.5f, 0.5f), vec3f::make(1.0f, 0.0f, 0.0f), 10.0f}, world.filter, visitor);
    CHECK(ctx, reject.calls == 3);
    CHECK(ctx, hits.count == 2 && contains(hits, a) && contains(hits, c) && !contains(hits, b));

    // asked only after the layer and group tests pass
    filter::disable_object_layer(world.filter, world.layers, world.not_moving);
    reject.calls = 0;
    visitor = visitor_for(hits);
    broadphase::intersect_aabb(world.broadphase, world.bodies, {{-1, 0, 0}, {6, 1, 1}}, world.filter, visitor);
    CHECK(ctx, reject.calls == 2 && hits.count == 1 && hits.bodies[0] == a);
    world.filter.collision_mask = 0;
    reject.calls = 0;
    visitor = visitor_for(hits);
    broadphase::intersect_aabb(world.broadphase, world.bodies, EVERYTHING, world.filter, visitor);
    CHECK(ctx, reject.calls == 0 && hits.count == 0);

    release_world(world);
}

void test_broadphase_update_moved_body(TestCtx &ctx) {
    World world = make_world();
    RigidBody *body = make_body(world, 0, 0, 0, 1, 1, 1, world.moving);
    make_body(world, 4, 0, 0, 5, 1, 1, world.moving);
    broadphase::optimize(world.broadphase, world.bodies);
    Hits hits;

    // a move inside the fat margin is no escape
    rigid_body::set_position(world.bodies, body, vec3f::make(0.52f, 0.5f, 0.5f), false);
    CHECK(ctx, !broadphase::update_body(world.broadphase, body));

    // moved far: an escape, found at its new place and not at its old
    rigid_body::set_position(world.bodies, body, vec3f::make(20.5f, 0.5f, -7.5f), false);
    CHECK(ctx, broadphase::update_body(world.broadphase, body));
    CHECK(ctx, world.broadphase.trees[world.broadphase_moving].dirty);
    body_visitor::BodyVisitor visitor = visitor_for(hits);
    broadphase::intersect_point(world.broadphase, world.bodies, vec3f::make(20.5f, 0.5f, -7.5f), world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == body);
    visitor = visitor_for(hits);
    broadphase::intersect_aabb(world.broadphase, world.bodies, {{-0.5f, -0.5f, -0.5f}, {1.5f, 1.5f, 1.5f}}, world.filter, visitor);
    CHECK(ctx, hits.count == 0);
    visitor = visitor_for(hits);
    broadphase::cast_ray(world.broadphase, world.bodies, cast_ray::Rayf{vec3f::make(20.5f, 10.0f, -7.5f), vec3f::make(0.0f, -1.0f, 0.0f), 20.0f}, world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == body);

    // still found after the rebuild settles the tree
    broadphase::optimize(world.broadphase, world.bodies);
    visitor = visitor_for(hits);
    broadphase::intersect_point(world.broadphase, world.bodies, vec3f::make(20.5f, 0.5f, -7.5f), world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == body);

    // a body out of the broadphase has nothing to update
    broadphase::remove_body(world.broadphase, body);
    CHECK(ctx, !broadphase::update_body(world.broadphase, body));

    release_world(world);
}

void test_broadphase_remove(TestCtx &ctx) {
    World world = make_world();
    RigidBody *a = make_body(world, 0, 0, 0, 1, 1, 1, world.moving);
    RigidBody *b = make_body(world, 0, 0, 0, 1, 1, 1, world.not_moving);
    RigidBody *c = make_body(world, 3, 0, 0, 4, 1, 1, world.moving);

    broadphase::remove_body(world.broadphase, b);
    CHECK(ctx, b->broadphase_layer == -1 && b->broadphase_node == -1);
    Box3f not_moving_bounds;
    CHECK(ctx, aabox::is_empty(aabox::load(quad_tree::bounds(not_moving_bounds, world.broadphase.trees[world.broadphase_not_moving]))));

    // gone from every query; the rest still found
    Hits hits;
    body_visitor::BodyVisitor visitor = visitor_for(hits);
    broadphase::intersect_aabb(world.broadphase, world.bodies, EVERYTHING, world.filter, visitor);
    CHECK(ctx, hits.count == 2 && contains(hits, a) && contains(hits, c));
    visitor = visitor_for(hits);
    broadphase::intersect_point(world.broadphase, world.bodies, vec3f::make(0.5f, 0.5f, 0.5f), world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == a);
    visitor = visitor_for(hits);
    broadphase::cast_ray(world.broadphase, world.bodies, cast_ray::Rayf{vec3f::make(-1.0f, 0.5f, 0.5f), vec3f::make(1.0f, 0.0f, 0.0f), 10.0f}, world.filter, visitor);
    CHECK(ctx, hits.count == 2 && !contains(hits, b));
    visitor = visitor_for(hits);
    broadphase::cast_aabb(world.broadphase, world.bodies, Box3f{{-2.0f, 0.4f, 0.4f}, {-1.8f, 0.6f, 0.6f}}, vec3f::make(10.0f, 0.0f, 0.0f), world.filter,
                          visitor);
    CHECK(ctx, hits.count == 2 && !contains(hits, b));

    // removing twice is a no-op; removing the rest empties every query
    broadphase::remove_body(world.broadphase, b);
    broadphase::remove_body(world.broadphase, a);
    broadphase::remove_body(world.broadphase, c);
    visitor = visitor_for(hits);
    broadphase::intersect_aabb(world.broadphase, world.bodies, EVERYTHING, world.filter, visitor);
    CHECK(ctx, hits.count == 0);
    Box3f all;
    CHECK(ctx, aabox::is_empty(aabox::load(broadphase::bounds(all, world.broadphase))));

    release_world(world);
}

void test_broadphase_reinsert_after_layer_change(TestCtx &ctx) {
    World world = make_world();
    RigidBody *body = make_body(world, 0, 0, 0, 1, 1, 1, world.moving);
    make_body(world, 3, 0, 0, 4, 1, 1, world.moving);

    rigid_body::set_object_layer(world.layers, body, world.not_moving);
    broadphase::reinsert_body(world.broadphase, body, world.layers);

    // moved from the moving tree to the not-moving one
    CHECK(ctx, body->broadphase_layer == (int32_t)world.broadphase_not_moving);
    CHECK(ctx, !tree_holds(world, world.broadphase_moving, body) && tree_holds(world, world.broadphase_not_moving, body));

    // and the layer filter follows it
    filter::disable_object_layer(world.filter, world.layers, world.not_moving);
    Hits hits;
    body_visitor::BodyVisitor visitor = visitor_for(hits);
    broadphase::intersect_point(world.broadphase, world.bodies, vec3f::make(0.5f, 0.5f, 0.5f), world.filter, visitor);
    CHECK(ctx, hits.count == 0);
    filter::set_all_enabled(world.filter, world.layers);
    filter::disable_object_layer(world.filter, world.layers, world.moving);
    visitor = visitor_for(hits);
    broadphase::intersect_point(world.broadphase, world.bodies, vec3f::make(0.5f, 0.5f, 0.5f), world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == body);

    release_world(world);
}

void test_broadphase_cast_ray_layers(TestCtx &ctx) {
    World world = make_world();
    // along x: not-moving at 2, moving at 4, not-moving at 6, moving at 8
    RigidBody *moving_near = make_body(world, 4, 0, 0, 5, 1, 1, world.moving);
    RigidBody *moving_far = make_body(world, 8, 0, 0, 9, 1, 1, world.moving);
    RigidBody *not_moving_near = make_body(world, 2, 0, 0, 3, 1, 1, world.not_moving);
    RigidBody *not_moving_far = make_body(world, 6, 0, 0, 7, 1, 1, world.not_moving);
    make_body(world, 4, 5, 0, 5, 6, 1, world.moving);

    // the trees are walked in layer order, each closest first
    const cast_ray::Rayf ray = {vec3f::make(0.0f, 0.5f, 0.5f), vec3f::make(1.0f, 0.0f, 0.0f), 20.0f};
    Hits hits;
    body_visitor::BodyVisitor visitor = visitor_for(hits);
    broadphase::cast_ray(world.broadphase, world.bodies, ray, world.filter, visitor);
    CHECK(ctx, hits.count == 4);
    CHECK(ctx, hits.bodies[0] == moving_near && hits.bodies[1] == moving_far);
    CHECK(ctx, hits.bodies[2] == not_moving_near && hits.bodies[3] == not_moving_far);

    // a closest-hit visitor carries its early-out from tree to tree: the moving tree's nearer body
    // prunes its farther one, and the not-moving tree is walked against that bound, which its
    // nearer body beats and its farther body does not
    visitor = visitor_for(hits);
    hits.closest_along = &ray;
    broadphase::cast_ray(world.broadphase, world.bodies, ray, world.filter, visitor);
    CHECK(ctx, hits.count == 2 && hits.bodies[0] == moving_near && hits.bodies[1] == not_moving_near);
    CHECK_NEAR(ctx, visitor.early_out_fraction, 2.0f / 20.0f, 1e-6f);

    // early exit stops at the first tree
    visitor = visitor_for(hits);
    hits.exit_after_first = true;
    broadphase::cast_ray(world.broadphase, world.bodies, ray, world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == moving_near);

    // the sweep finds the body along its path
    visitor = visitor_for(hits);
    broadphase::cast_aabb(world.broadphase, world.bodies, Box3f{{-0.5f, 4.5f, -0.5f}, {0.5f, 5.5f, 0.5f}}, vec3f::make(10.0f, 0.0f, 0.0f), world.filter,
                          visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0]->aabb.min.y == 5.0f);

    release_world(world);
}

void test_broadphase_point_queries(TestCtx &ctx) {
    World world = make_world();
    RigidBody *a = make_body(world, 0, 0, 0, 4, 4, 4, world.moving);
    RigidBody *b = make_body(world, 2, 2, 2, 6, 6, 6, world.not_moving);
    Hits hits;

    // inside both, inside one, in empty space
    body_visitor::BodyVisitor visitor = visitor_for(hits);
    broadphase::intersect_point(world.broadphase, world.bodies, vec3f::make(3, 3, 3), world.filter, visitor);
    CHECK(ctx, hits.count == 2 && contains(hits, a) && contains(hits, b));
    visitor = visitor_for(hits);
    broadphase::intersect_point(world.broadphase, world.bodies, vec3f::make(1, 1, 1), world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == a);
    visitor = visitor_for(hits);
    broadphase::intersect_point(world.broadphase, world.bodies, vec3f::make(5, 1, 1), world.filter, visitor);
    CHECK(ctx, hits.count == 0);

    // early exit: the first body ends the query, the second tree is never walked
    visitor = visitor_for(hits);
    hits.exit_after_first = true;
    broadphase::intersect_point(world.broadphase, world.bodies, vec3f::make(3, 3, 3), world.filter, visitor);
    CHECK(ctx, hits.count == 1 && hits.bodies[0] == a);

    release_world(world);
}

void test_broadphase_bounds(TestCtx &ctx) {
    World world = make_world();
    Box3f all;
    CHECK(ctx, aabox::is_empty(aabox::load(broadphase::bounds(all, world.broadphase))));

    make_body(world, -3, 0, 0, -2, 1, 1, world.moving);
    make_body(world, 0, 0, 0, 1, 2, 1, world.moving);
    RigidBody *not_moving = make_body(world, 5, -4, 2, 6, -3, 9, world.not_moving);

    // the union of the two trees' root boxes, each the union of its fat leaves
    broadphase::bounds(all, world.broadphase);
    Box3f moving_root, not_moving_root, expected;
    quad_tree::bounds(moving_root, world.broadphase.trees[world.broadphase_moving]);
    quad_tree::bounds(not_moving_root, world.broadphase.trees[world.broadphase_not_moving]);
    aabox::store(expected, aabox::union_(aabox::load(moving_root), aabox::load(not_moving_root)));
    CHECK(ctx, equal_boxes(all, expected));
    const float margin = world.broadphase.trees[0].expansion_margin;
    CHECK_NEAR(ctx, all.min.x, -3.0f - margin, 1e-6f);
    CHECK_NEAR(ctx, all.min.y, -4.0f - margin, 1e-6f);
    CHECK_NEAR(ctx, all.min.z, 0.0f - margin, 1e-6f);
    CHECK_NEAR(ctx, all.max.x, 6.0f + margin, 1e-6f);
    CHECK_NEAR(ctx, all.max.y, 2.0f + margin, 1e-6f);
    CHECK_NEAR(ctx, all.max.z, 9.0f + margin, 1e-6f);

    // one tree empty: the other's bounds alone
    broadphase::remove_body(world.broadphase, not_moving);
    CHECK(ctx, equal_boxes(broadphase::bounds(all, world.broadphase), moving_root));

    release_world(world);
}
