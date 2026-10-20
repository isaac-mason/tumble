#include "physics/broadphase/quad_tree.h"
#include "math/aabox.h"

#include "instance.h"
#include "math/vec3f.h"
#include "physics/body/bodies.h"
#include "physics/body/rigid_body.h"
#include "physics/filter.h"
#include "physics/layers.h"
#include "physics/shapes/box.h"
#include "physics/shapes/dispatch.h"
#include "test.h"
#include "test_math.h"
#include "test_rng.h"
#include "util/alloc.h"
#include "util/mem.h"

using rigid_body::RigidBody;

namespace {

constexpr uint32_t MAX_BODIES = 1024;

struct World {
    bodies::Bodies bodies;
    uint32_t count;
    layers::Layers layers;
    filter::Filter filter;
    quad_tree::QuadTree tree;
};

World make_world() {
    dispatch::init();
    World world = {};
    world.bodies = bodies::init();
    layers::add_broadphase_layer(world.layers);
    layers::add_object_layer(world.layers, 0);
    world.filter = filter::create(world.layers);
    world.tree = quad_tree::create();
    return world;
}

void release_world(World &world) {
    quad_tree::release(world.tree);
    filter::release(world.filter);
    layers::release(world.layers);
    bodies::release(world.bodies);
}

RigidBody *body_at(const World &world, uint32_t index) { return world.bodies.pool[index]; }

/** a dynamic box body filling the given world-space box. */
RigidBody *make_body(World &world, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z) {
    const Float3 half_extents = {(max_x - min_x) * 0.5f, (max_y - min_y) * 0.5f, (max_z - min_z) * 0.5f};
    const Float3 center = {(min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f, (min_z + max_z) * 0.5f};
    RigidBody *body = rigid_body::create(world.bodies, world.layers,
                                         {.shape = box::create({.half_extents = half_extents}),
                                          .object_layer = 0,
                                          .motion_type = motion_type::MOTION_DYNAMIC,
                                          .position = center});
    world.count++;
    return body;
}

void move_by(World &world, RigidBody *body, const Float3 &delta) {
    rigid_body::set_position(world.bodies, body, vec3f::add(vec3f::load(body->position), vec3f::load(delta)), false);
}

/** the unit boxes [2i, 2i+1] on a grid of `side` per axis; `side_y` layers high. */
void make_grid(World &world, uint32_t side, uint32_t side_y) {
    for (uint32_t y = 0; y < side_y; y++) {
        for (uint32_t z = 0; z < side; z++) {
            for (uint32_t x = 0; x < side; x++) {
                quad_tree::add(world.tree, make_body(world, x * 2.0f, y * 2.0f, z * 2.0f, x * 2.0f + 1.0f, y * 2.0f + 1.0f, z * 2.0f + 1.0f));
            }
        }
    }
}

struct Visits {
    uint32_t times[MAX_BODIES];
    uint32_t total;
    int32_t first;
    /** when set, a visit takes this body's fraction as the early-out (a closest-hit collector) */
    const float *fraction_of;
};

void record_visit(body_visitor::BodyVisitor *visitor, RigidBody *body) {
    Visits *visits = static_cast<Visits *>(visitor->context);
    if (visits->total == 0) visits->first = (int32_t)body->index;
    visits->times[body->index]++;
    visits->total++;
    if (visits->fraction_of != nullptr) visitor->early_out_fraction = visits->fraction_of[body->index];
}

Visits *new_visits() {
    Visits *visits = util::alloc<Visits>(1);
    util::zero(visits, sizeof(Visits));
    visits->first = -1;
    return visits;
}

body_visitor::BodyVisitor visitor_for(Visits *visits) {
    util::zero(visits->times, sizeof(visits->times));
    visits->total = 0;
    visits->first = -1;
    return body_visitor::create(record_visit, visits);
}

template <typename Expected>
bool visited_exactly(const World &world, const Visits *visits, Expected expected) {
    for (uint32_t i = 0; i < world.count; i++) {
        if (visits->times[i] != (expected(*body_at(world, i)) ? 1u : 0u)) return false;
    }
    return true;
}

uint32_t current_root(const quad_tree::QuadTree &tree) { return tree.root_node; }

/**
 * structural invariants: parent links, every slot holding its child node's bounds, changed => parent changed, every body
 * in exactly one reachable slot whose box is its fat box and whose location it holds, empty slots inverted.
 */
bool tree_valid(World &world, uint32_t *max_depth = nullptr) {
    const quad_tree::QuadTree &tree = world.tree;
    uint32_t *seen = util::alloc<uint32_t>(MAX_BODIES);
    util::zero(seen, MAX_BODIES * sizeof(uint32_t));
    collections::Vec<uint32_t> stack = {};
    collections::Vec<uint32_t> depths = {};
    bool ok = tree.nodes[current_root(tree)].parent_node_index == quad_tree::INVALID_NODE_INDEX;
    stack.push(current_root(tree));
    depths.push(1);
    uint32_t deepest = 0;
    while (stack.length > 0) {
        const uint32_t index = stack[stack.length - 1];
        const uint32_t depth = depths[depths.length - 1];
        stack.length--;
        depths.length--;
        if (depth > deepest) deepest = depth;
        const quad_tree::Node &node = tree.nodes[index];
        for (uint32_t i = 0; i < 4; i++) {
            const quad_tree::NodeId id = node.child_node_id[i];
            const Box3f slot = {{node.bounds_min_x[i], node.bounds_min_y[i], node.bounds_min_z[i]},
                                {node.bounds_max_x[i], node.bounds_max_y[i], node.bounds_max_z[i]}};
            if (!quad_tree::is_valid(id)) {
                ok = ok && slot.min.x == quad_tree::LARGE_FLOAT && slot.max.x == -quad_tree::LARGE_FLOAT;
                continue;
            }
            if (quad_tree::is_body(id)) {
                const RigidBody *body = body_at(world, id);
                seen[id]++;
                const uint32_t location = (uint32_t)body->broadphase_node;
                ok = ok && (location & 0x3fffffffu) == index && (location >> 30) == i;
                Box3f fat;
                ok = ok && aabox::contains(aabox::load(slot), aabox::load(body->aabb)) && equal_boxes(quad_tree::read_body_bounds(fat, tree, *body), slot);
                continue;
            }
            const uint32_t child = quad_tree::node_index(id);
            const quad_tree::Node &child_node = tree.nodes[child];
            ok = ok && child_node.parent_node_index == index;
            if (child_node.is_changed) ok = ok && node.is_changed;
            for (uint32_t k = 0; k < 4; k++) {
                if (!quad_tree::is_valid(child_node.child_node_id[k])) continue;
                ok = ok && slot.min.x <= child_node.bounds_min_x[k] && slot.min.y <= child_node.bounds_min_y[k] &&
                     slot.min.z <= child_node.bounds_min_z[k] && slot.max.x >= child_node.bounds_max_x[k] &&
                     slot.max.y >= child_node.bounds_max_y[k] && slot.max.z >= child_node.bounds_max_z[k];
            }
            stack.push(child);
            depths.push(depth + 1);
        }
    }
    uint32_t alive = 0;
    for (uint32_t i = 0; i < world.count; i++) {
        const bool in_tree = body_at(world, i)->broadphase_node != -1;
        alive += in_tree ? 1 : 0;
        ok = ok && seen[i] == (in_tree ? 1u : 0u);
    }
    ok = ok && alive == tree.num_bodies;
    if (max_depth != nullptr) *max_depth = deepest;
    stack.release();
    depths.release();
    util::free(seen);
    return ok;
}

}  // namespace

void test_quad_tree_add(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    World world = make_world();
    quad_tree::QuadTree &tree = world.tree;
    Box3f all;
    CHECK(ctx, tree.num_bodies == 0 && !tree.dirty && aabox::is_empty(aabox::load(quad_tree::bounds(all, tree))));

    // the first four take the root's slots in order, each its fat box
    RigidBody *first = nullptr;
    for (uint32_t i = 0; i < 4; i++) {
        RigidBody *body = make_body(world, i * 2.0f, 0, 0, i * 2.0f + 1.0f, 1, 1);
        quad_tree::add(tree, body);
        if (i == 0) first = body;
        CHECK(ctx, (uint32_t)body->broadphase_node == (current_root(tree) | (i << 30)));
    }
    Box3f fat, expected;
    CHECK(ctx, equal_boxes(quad_tree::read_body_bounds(fat, tree, *first), aabox::store(expected, aabox::expand_by_margin(aabox::load(first->aabb), 0.05f))));
    CHECK(ctx, tree.dirty && tree.num_bodies == 4);
    const uint32_t old_root = current_root(tree);

    // a fifth finds the root full: a new root over the old one and the body, its slots the old root's bounds and the body's
    RigidBody *fifth = make_body(world, 8, 0, 0, 9, 1, 1);
    quad_tree::add(tree, fifth);
    const uint32_t new_root = current_root(tree);
    CHECK(ctx, new_root != old_root && tree.nodes[old_root].parent_node_index == new_root && tree.nodes[new_root].is_changed);
    CHECK(ctx, tree.nodes[new_root].child_node_id[0] == quad_tree::from_node_index(old_root));
    CHECK(ctx, (uint32_t)fifth->broadphase_node == (new_root | (1u << 30)));
    CHECK(ctx, tree_valid(world));
    CHECK_NEAR(ctx, quad_tree::bounds(all, tree).min.x, -0.05f, 1e-6f);
    CHECK_NEAR(ctx, all.max.x, 9.05f, 1e-6f);

    release_world(world);
}

void test_quad_tree_rebuild(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    World world = make_world();
    make_grid(world, 8, 8);
    quad_tree::QuadTree &tree = world.tree;

    // 512 bodies added one at a time are a chain of roots; the rebuild makes a quad tree of them
    uint32_t depth = 0;
    CHECK(ctx, tree_valid(world, &depth) && depth > 100);
    quad_tree::rebuild(tree, world.bodies);
    CHECK(ctx, tree_valid(world, &depth) && !tree.dirty);
    // 4^5 > 512: five levels hold them, a few more where the median splits fall unevenly
    CHECK(ctx, depth <= 7);

    // the levels under MAX_DEPTH_MARK_CHANGED are born changed; deeper ones are not
    CHECK(ctx, tree.nodes[current_root(tree)].is_changed);
    const uint32_t nodes_after = tree.nodes.length - tree.free_nodes.length;

    // a second rebuild with nothing moved: the same structure in the same node count, every body still found
    quad_tree::rebuild(tree, world.bodies);
    CHECK(ctx, tree_valid(world, &depth) && depth <= 7);
    CHECK(ctx, tree.nodes.length - tree.free_nodes.length == nodes_after);

    // moving one body marks only its path; a rebuild grafts every unchanged subtree whole, keeping its node index
    RigidBody *moved = body_at(world, 100);
    const uint32_t untouched_location = (uint32_t)body_at(world, 400)->broadphase_node;
    const uint32_t untouched_node = untouched_location & 0x3fffffffu;
    const bool untouched_unchanged = !tree.nodes[untouched_node].is_changed;
    move_by(world, moved, {0.5f, 0.0f, 0.0f});
    CHECK(ctx, quad_tree::update(tree, moved) && tree.dirty);
    CHECK(ctx, tree_valid(world));
    quad_tree::rebuild(tree, world.bodies);
    CHECK(ctx, tree_valid(world));
    if (untouched_unchanged && !tree.nodes[untouched_node].is_changed) {
        CHECK(ctx, (uint32_t)body_at(world, 400)->broadphase_node == untouched_location);
    }

    // removing every body leaves an empty tree after the rebuild, and the node pool holds one root
    for (uint32_t i = 0; i < world.count; i++) quad_tree::remove(tree, body_at(world, i));
    CHECK(ctx, tree.num_bodies == 0 && tree_valid(world));
    quad_tree::rebuild(tree, world.bodies);
    Box3f all;
    CHECK(ctx, tree_valid(world) && aabox::is_empty(aabox::load(quad_tree::bounds(all, tree))));
    CHECK(ctx, tree.nodes.length - tree.free_nodes.length == 1);

    release_world(world);
}

void test_quad_tree_update(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    World world = make_world();
    make_grid(world, 6, 2);
    quad_tree::QuadTree &tree = world.tree;
    quad_tree::rebuild(tree, world.bodies);
    RigidBody *body = body_at(world, 20);
    Box3f before, after, expected;
    quad_tree::read_body_bounds(before, tree, *body);

    // inside the fat margin: no escape, nothing marked
    move_by(world, body, {0.03f, 0.0f, 0.0f});
    CHECK(ctx, !quad_tree::update(tree, body) && !tree.dirty);
    CHECK(ctx, equal_boxes(quad_tree::read_body_bounds(after, tree, *body), before));

    // out of it: the fat box is replaced (it moves with the body, it does not just grow), the path up is changed and holds it
    move_by(world, body, {3.0f, 1.0f, 0.0f});
    CHECK(ctx, quad_tree::update(tree, body) && tree.dirty);
    CHECK(ctx, equal_boxes(quad_tree::read_body_bounds(after, tree, *body), aabox::store(expected, aabox::expand_by_margin(aabox::load(body->aabb), 0.05f))));
    CHECK(ctx, after.min.x > before.max.x - 1.0f);
    for (uint32_t index = (uint32_t)body->broadphase_node & 0x3fffffffu; index != quad_tree::INVALID_NODE_INDEX; index = tree.nodes[index].parent_node_index) {
        CHECK(ctx, tree.nodes[index].is_changed);
    }
    CHECK(ctx, tree_valid(world));

    // back to where it was: the box shrinks back with it
    move_by(world, body, {-3.0f, -1.0f, 0.0f});
    CHECK(ctx, quad_tree::update(tree, body));
    CHECK(ctx, equal_boxes(quad_tree::read_body_bounds(after, tree, *body), aabox::store(expected, aabox::expand_by_margin(aabox::load(body->aabb), 0.05f))));

    release_world(world);
}

void test_quad_tree_queries(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    World world = make_world();
    make_grid(world, 5, 5);
    quad_tree::rebuild(world.tree, world.bodies);
    Visits *visits = new_visits();

    // an aabb over the 3 x 3 x 3 corner: exactly its 27 boxes
    body_visitor::BodyVisitor visitor = visitor_for(visits);
    quad_tree::intersect_aabb(world.bodies, world.tree, {{-0.5f, -0.5f, -0.5f}, {4.5f, 4.5f, 4.5f}}, world.filter, visitor);
    CHECK(ctx, visits->total == 27);
    CHECK(ctx, visited_exactly(world, visits, [](const RigidBody &b) { return b.aabb.max.x <= 5.0f && b.aabb.max.y <= 5.0f && b.aabb.max.z <= 5.0f; }));

    // a box in a gap, inside the fat margins: the fat leaves reach it, the bodies do not
    const Box3f gap = {{1.02f, 1.02f, 1.02f}, {1.03f, 1.03f, 1.03f}};
    visitor = visitor_for(visits);
    quad_tree::intersect_aabb(world.bodies, world.tree, gap, world.filter, visitor);
    CHECK(ctx, visits->total == 0);
    visitor = visitor_for(visits);
    quad_tree::intersect_aabb_fat_leaves(world.bodies, world.tree, gap, visitor);
    CHECK(ctx, visits->total == 1 && body_at(world, visits->first)->aabb.min.x == 0.0f);

    // a point inside one box, and on the shared face of none (faces included)
    visitor = visitor_for(visits);
    quad_tree::intersect_point(world.bodies, world.tree, vec3f::make(4.5f, 2.5f, 6.5f), world.filter, visitor);
    CHECK(ctx, visits->total == 1 && body_at(world, visits->first)->aabb.min.x == 4.0f && body_at(world, visits->first)->aabb.min.z == 6.0f);
    visitor = visitor_for(visits);
    quad_tree::intersect_point(world.bodies, world.tree, vec3f::make(5.0f, 2.5f, 6.5f), world.filter, visitor);
    CHECK(ctx, visits->total == 1);

    // walk: everything once
    visitor = visitor_for(visits);
    quad_tree::walk(world.bodies, world.tree, visitor);
    CHECK(ctx, visits->total == 125 && visited_exactly(world, visits, [](const RigidBody &) { return true; }));

    // straight up through the column at x = 2, z = 3: its five boxes, closest first
    const Float3 origin = {4.5f, -5.0f, 6.5f};
    const Float3 up = {0.0f, 1.0f, 0.0f};
    const auto in_column = [](const RigidBody &b) { return b.aabb.min.x == 4.0f && b.aabb.min.z == 6.0f; };
    visitor = visitor_for(visits);
    quad_tree::cast_ray(world.bodies, world.tree, {vec3f::load(origin), vec3f::load(up), 20.0f}, world.filter, visitor);
    CHECK(ctx, visits->total == 5 && visited_exactly(world, visits, in_column));
    CHECK(ctx, visits->first >= 0 && body_at(world, visits->first)->aabb.min.y == 0.0f);

    // too short to reach past the third layer
    visitor = visitor_for(visits);
    quad_tree::cast_ray(world.bodies, world.tree, {vec3f::load(origin), vec3f::load(up), 9.5f}, world.filter, visitor);
    CHECK(ctx, visits->total == 3);

    // a closest-hit visitor prunes everything behind the first box
    float *fractions = util::alloc<float>(MAX_BODIES);
    for (uint32_t i = 0; i < world.count; i++) fractions[i] = (body_at(world, i)->aabb.min.y - origin.y) / 20.0f;
    visits->fraction_of = fractions;
    visitor = visitor_for(visits);
    quad_tree::cast_ray(world.bodies, world.tree, {vec3f::load(origin), vec3f::load(up), 20.0f}, world.filter, visitor);
    CHECK(ctx, visits->total == 1 && body_at(world, visits->first)->aabb.min.y == 0.0f);
    visits->fraction_of = nullptr;

    // a box swept along +x straddling the rows z = 1 and z = 2 at y = 0: both rows
    visitor = visitor_for(visits);
    quad_tree::cast_aabb(world.bodies, world.tree, {{-5.75f, 0.25f, 2.75f}, {-4.25f, 0.75f, 4.25f}}, vec3f::make(30.0f, 0.0f, 0.0f), world.filter, visitor);
    CHECK(ctx, visits->total == 10);
    CHECK(ctx, visited_exactly(world, visits, [](const RigidBody &b) { return b.aabb.min.y == 0.0f && (b.aabb.min.z == 2.0f || b.aabb.min.z == 4.0f); }));
    CHECK(ctx, body_at(world, visits->first)->aabb.min.x == 0.0f);

    // a still cast is an overlap test of the start bounds
    visitor = visitor_for(visits);
    quad_tree::cast_aabb(world.bodies, world.tree, {{1.5f, 0.25f, 2.25f}, {4.5f, 0.75f, 2.75f}}, vec3f::make(0.0f, 0.0f, 0.0f), world.filter, visitor);
    CHECK(ctx, visits->total == 2);

    util::free(fractions);
    util::free(visits);
    release_world(world);
}

/** whether the segment origin + t displacement, t in [0, 1], enters the box (faces included) */
bool segment_enters(const Float3 &origin, const Float3 &displacement, const Box3f &box) {
    const float o[3] = {origin.x, origin.y, origin.z};
    const float d[3] = {displacement.x, displacement.y, displacement.z};
    const float lo[3] = {box.min.x, box.min.y, box.min.z};
    const float hi[3] = {box.max.x, box.max.y, box.max.z};
    float t_min = 0.0f, t_max = 1.0f;
    for (uint32_t axis = 0; axis < 3; axis++) {
        if (scalar::abs(d[axis]) < 1e-20f) {
            if (o[axis] < lo[axis] || o[axis] > hi[axis]) return false;
            continue;
        }
        float t0 = (lo[axis] - o[axis]) / d[axis];
        float t1 = (hi[axis] - o[axis]) / d[axis];
        if (t0 > t1) {
            const float t = t0;
            t0 = t1;
            t1 = t;
        }
        t_min = scalar::max(t_min, t0);
        t_max = scalar::min(t_max, t1);
    }
    return t_min <= t_max;
}

bool boxes_overlap(const Box3f &a, const Box3f &b) {
    return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y && a.max.y >= b.min.y && a.min.z <= b.max.z && a.max.z >= b.min.z;
}

void test_quad_tree_agrees_with_brute_force(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    World world = make_world();
    Visits *visits = new_visits();
    uint32_t seed = 4242;

    // a thousand random boxes, then churn and moves
    for (uint32_t i = 0; i < 1000; i++) {
        const float x = lcg32_unit(seed) * 100.0f;
        const float y = lcg32_unit(seed) * 30.0f;
        const float z = lcg32_unit(seed) * 100.0f;
        const float size = 0.2f + lcg32_unit(seed) * 3.0f;
        quad_tree::add(world.tree, make_body(world, x, y, z, x + size, y + size, z + size));
    }
    bool escapes_right = true;
    for (uint32_t round = 0; round < 6; round++) {
        for (uint32_t k = 0; k < 120; k++) {
            RigidBody *b = body_at(world, (uint32_t)(lcg32_unit(seed) * world.count));
            const float pick = lcg32_unit(seed);
            if (pick < 0.3f) {
                if (b->broadphase_node != -1) {
                    quad_tree::remove(world.tree, b);
                } else {
                    quad_tree::add(world.tree, b);
                }
            } else if (b->broadphase_node != -1) {
                // the body escapes when its box leaves its fat box
                Box3f fat;
                quad_tree::read_body_bounds(fat, world.tree, *b);
                move_by(world, b, {(lcg32_unit(seed) - 0.5f) * 0.4f, 0.0f, (lcg32_unit(seed) - 0.5f) * 0.4f});
                const bool escaped = quad_tree::update(world.tree, b);
                escapes_right = escapes_right && escaped == !aabox::contains(aabox::load(fat), aabox::load(b->aabb));
            }
        }
        if (round % 2 == 1) quad_tree::rebuild(world.tree, world.bodies);
    }
    CHECK(ctx, escapes_right);
    CHECK(ctx, tree_valid(world));

    // every fat box holds its body
    bool fat_holds = true;
    for (uint32_t i = 0; i < world.count; i++) {
        RigidBody *b = body_at(world, i);
        if (b->broadphase_node == -1) continue;
        Box3f fat;
        fat_holds = fat_holds && aabox::contains(aabox::load(quad_tree::read_body_bounds(fat, world.tree, *b)), aabox::load(b->aabb));
    }
    CHECK(ctx, fat_holds);

    // every query visits exactly the bodies a test of each body in the tree finds, once each
    bool agrees = true;
    for (uint32_t q = 0; q < 40; q++) {
        const float x = lcg32_unit(seed) * 90.0f;
        const float z = lcg32_unit(seed) * 90.0f;
        const auto check = [&](auto query, auto expected) {
            body_visitor::BodyVisitor visitor = visitor_for(visits);
            query(visitor);
            for (uint32_t i = 0; i < world.count; i++) {
                const RigidBody &b = *body_at(world, i);
                agrees = agrees && visits->times[i] == (b.broadphase_node != -1 && expected(b) ? 1u : 0u);
            }
        };
        const Box3f region = {{x, 0, z}, {x + 12.0f, 20.0f, z + 12.0f}};
        check([&](body_visitor::BodyVisitor &v) { quad_tree::intersect_aabb(world.bodies, world.tree, region, world.filter, v); },
              [&](const RigidBody &b) { return boxes_overlap(b.aabb, region); });
        check([&](body_visitor::BodyVisitor &v) { quad_tree::intersect_aabb_fat_leaves(world.bodies, world.tree, region, v); },
              [&](const RigidBody &b) {
                  Box3f fat;
                  return boxes_overlap(quad_tree::read_body_bounds(fat, world.tree, b), region);
              });
        const cast_ray::Rayf ray = {vec3f::make(x, 40.0f, z), vec3f::make(0.36f, -0.8f, 0.48f), 60.0f};
        check([&](body_visitor::BodyVisitor &v) { quad_tree::cast_ray(world.bodies, world.tree, ray, world.filter, v); },
              [&](const RigidBody &b) { return segment_enters({x, 40.0f, z}, {0.36f * 60.0f, -0.8f * 60.0f, 0.48f * 60.0f}, b.aabb); });
        // a box cast is its centre cast against every box grown by its half extents
        const Box3f swept = {{x, 2.0f, -10.0f}, {x + 1.5f, 3.5f, -8.5f}};
        check([&](body_visitor::BodyVisitor &v) { quad_tree::cast_aabb(world.bodies, world.tree, swept, vec3f::make(0.0f, 0.0f, 120.0f), world.filter, v); },
              [&](const RigidBody &b) {
                  const Box3f grown = {{b.aabb.min.x - 0.75f, b.aabb.min.y - 0.75f, b.aabb.min.z - 0.75f},
                                       {b.aabb.max.x + 0.75f, b.aabb.max.y + 0.75f, b.aabb.max.z + 0.75f}};
                  return segment_enters({x + 0.75f, 2.75f, -9.25f}, {0.0f, 0.0f, 120.0f}, grown);
              });
        const Float3 point = {x, lcg32_unit(seed) * 30.0f, z};
        check([&](body_visitor::BodyVisitor &v) { quad_tree::intersect_point(world.bodies, world.tree, vec3f::load(point), world.filter, v); },
              [&](const RigidBody &b) { return aabox::contains_point(aabox::load(b.aabb), vec3f::load(point)); });
    }
    CHECK(ctx, agrees);

    util::free(visits);
    release_world(world);
}
