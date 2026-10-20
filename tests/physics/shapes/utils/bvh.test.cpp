#include "physics/shapes/utils/bvh.h"

#include "test.h"
#include "test_math.h"

/** a root over two leaves: left [0, 1]^3 holding primitives 0..2, right [2, 3] x [0, 1] x [0, 1] holding 3..4 */
static void make_tree(bvh::Node nodes[3]) {
    nodes[0] = {{{0, 0, 0}, {3, 1, 1}}, 2, 0};
    nodes[1] = {{{0, 0, 0}, {1, 1, 1}}, 0, -(3 + 1)};
    nodes[2] = {{{2, 0, 0}, {3, 1, 1}}, 3, -(2 + 1)};
}

void test_bvh_node_accessors(TestCtx &ctx) {
    bvh::Node nodes[3];
    make_tree(nodes);

    CHECK(ctx, !bvh::node_is_leaf(nodes, 0));
    CHECK(ctx, bvh::node_is_leaf(nodes, 1) && bvh::node_is_leaf(nodes, 2));
    CHECK(ctx, bvh::node_left(0) == 1);
    CHECK(ctx, bvh::node_right(nodes, 0) == 2);
    CHECK(ctx, bvh::node_split_axis(nodes, 0) == 0);

    Box3f bounds;
    bvh::node_get_bounds(bounds, nodes, 2);
    CHECK(ctx, bounds.min.x == 2 && bounds.min.y == 0 && bounds.max.x == 3 && bounds.max.z == 1);

    const Vec3f center = bvh::node_get_center(nodes, 0);
    CHECK(ctx, vec3f::x(center) == 1.5f && vec3f::y(center) == 0.5f && vec3f::z(center) == 0.5f);
}

void test_bvh_node_intersects_box(TestCtx &ctx) {
    bvh::Node nodes[3];
    make_tree(nodes);

    // overlapping, containing and contained boxes
    CHECK(ctx, bvh::node_intersects_box(nodes, 1, AABox{vec3f::make(0.5f, 0.5f, 0.5f), vec3f::make(2, 2, 2)}));
    CHECK(ctx, bvh::node_intersects_box(nodes, 1, AABox{vec3f::make(-1, -1, -1), vec3f::make(5, 5, 5)}));
    CHECK(ctx, bvh::node_intersects_box(nodes, 1, AABox{vec3f::make(0.25f, 0.25f, 0.25f), vec3f::make(0.5f, 0.5f, 0.5f)}));
    // touching faces count
    CHECK(ctx, bvh::node_intersects_box(nodes, 1, {vec3f::make(1, 0, 0), vec3f::make(2, 1, 1)}));
    CHECK(ctx, bvh::node_intersects_box(nodes, 2, {vec3f::make(1, 0, 0), vec3f::make(2, 1, 1)}));
    // the gap between the leaves touches neither once it is narrowed
    CHECK(ctx, !bvh::node_intersects_box(nodes, 1, {vec3f::make(1.25f, 0, 0), vec3f::make(1.75f, 1, 1)}));
    CHECK(ctx, !bvh::node_intersects_box(nodes, 2, {vec3f::make(1.25f, 0, 0), vec3f::make(1.75f, 1, 1)}));
    // separated on one axis only
    CHECK(ctx, !bvh::node_intersects_box(nodes, 1, {vec3f::make(0, 0, 1.5f), vec3f::make(1, 1, 2)}));
    CHECK(ctx, !bvh::node_intersects_box(nodes, 1, {vec3f::make(0, -2, 0), vec3f::make(1, -0.5f, 1)}));
}

void test_bvh_node_intersects_ray(TestCtx &ctx) {
    bvh::Node nodes[3];
    make_tree(nodes);

    // along +x through both leaves: enters the left at t = 1, the right at t = 3
    const Float3 origin = {-1, 0.5f, 0.5f};
    const Float3 along_x = {1, 0, 0};
    CHECK(ctx, bvh::node_intersects_ray(nodes, 1, vec3f::load(origin), vec3f::load(along_x), 0, 10));
    CHECK(ctx, bvh::node_intersects_ray(nodes, 2, vec3f::load(origin), vec3f::load(along_x), 0, 10));
    // the [near, far] window: the left box spans t in [1, 2], the right t in [3, 4]
    CHECK(ctx, !bvh::node_intersects_ray(nodes, 1, vec3f::load(origin), vec3f::load(along_x), 0, 0.5f));
    CHECK(ctx, bvh::node_intersects_ray(nodes, 1, vec3f::load(origin), vec3f::load(along_x), 0, 1));
    CHECK(ctx, !bvh::node_intersects_ray(nodes, 2, vec3f::load(origin), vec3f::load(along_x), 0, 2.5f));
    CHECK(ctx, !bvh::node_intersects_ray(nodes, 1, vec3f::load(origin), vec3f::load(along_x), 2.5f, 10));
    CHECK(ctx, bvh::node_intersects_ray(nodes, 2, vec3f::load(origin), vec3f::load(along_x), 2.5f, 10));

    // pointing away misses (the slab interval lies behind the origin)
    CHECK(ctx, !bvh::node_intersects_ray(nodes, 1, vec3f::load(origin), vec3f::make(-1, 0, 0), 0, 10));

    // negative direction from beyond the right box
    const Float3 beyond = {5, 0.5f, 0.5f};
    CHECK(ctx, bvh::node_intersects_ray(nodes, 2, vec3f::load(beyond), vec3f::make(-1, 0, 0), 0, 2));
    CHECK(ctx, !bvh::node_intersects_ray(nodes, 1, vec3f::load(beyond), vec3f::make(-1, 0, 0), 0, 3.5f));
    CHECK(ctx, bvh::node_intersects_ray(nodes, 1, vec3f::load(beyond), vec3f::make(-1, 0, 0), 0, 4));

    // parallel to an axis: inside the slab hits, outside misses
    CHECK(ctx, bvh::node_intersects_ray(nodes, 1, vec3f::make(0.5f, -1, 0.5f), vec3f::make(0, 1, 0), 0, 10));
    CHECK(ctx, !bvh::node_intersects_ray(nodes, 1, vec3f::make(1.5f, -1, 0.5f), vec3f::make(0, 1, 0), 0, 10));
    CHECK(ctx, !bvh::node_intersects_ray(nodes, 1, vec3f::make(0.5f, -1, 1.5f), vec3f::make(0, 1, 0), 0, 10));

    // starting inside always hits
    CHECK(ctx, bvh::node_intersects_ray(nodes, 1, vec3f::make(0.5f, 0.5f, 0.5f), vec3f::make(0.6f, 0.8f, 0), 0, 0.01f));

    // a diagonal ray that passes over the left box's corner misses it, one through the corner hits
    const float s = 0.70710678f;
    CHECK(ctx, !bvh::node_intersects_ray(nodes, 1, vec3f::make(-1, 0.5f, 1.25f), vec3f::make(s, 0, s), 0, 10));
    CHECK(ctx, bvh::node_intersects_ray(nodes, 1, vec3f::make(-0.75f, 0.5f, 0), vec3f::make(s, 0, s), 0, 10));
}
