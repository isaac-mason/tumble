#include "tumble/instance.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/utils/convex_hull_builder_2d.h"
#include "test.h"
#include "test_math.h"
#include "test_rng.h"

namespace hull_2d = convex_hull_builder_2d;

static bool contains(const collections::Vec<int32_t> &edges, int32_t index) {
    for (uint32_t i = 0; i < edges.length; i++) {
        if (edges[i] == index) return true;
    }
    return false;
}

/** whether `edges` is `expected` up to where the loop starts. */
static bool is_rotation_of(const collections::Vec<int32_t> &edges, const int32_t *expected, uint32_t count) {
    if (edges.length != count) return false;
    for (uint32_t start = 0; start < count; start++) {
        bool same = true;
        for (uint32_t i = 0; i < count && same; i++) same = edges[(start + i) % count] == expected[i];
        if (same) return true;
    }
    return false;
}

/** every point is on or left of every directed hull edge (counter clockwise), within tolerance. */
static bool hull_contains_all(const Float3 *points, uint32_t num_points, const collections::Vec<int32_t> &edges, float tolerance) {
    for (uint32_t e = 0; e < edges.length; e++) {
        const Float3 &a = points[edges[e]];
        const Float3 &b = points[edges[(e + 1) % edges.length]];
        const float ex = b.x - a.x, ey = b.y - a.y;
        const float edge_length = __builtin_sqrtf(ex * ex + ey * ey);
        for (uint32_t i = 0; i < num_points; i++) {
            const float cross = ex * (points[i].y - a.y) - ey * (points[i].x - a.x);
            if (cross < -tolerance * edge_length) return false;
        }
    }
    return true;
}

/** every turn of the loop is strictly to the left. */
static bool is_strictly_convex_ccw(const Float3 *points, const collections::Vec<int32_t> &edges) {
    for (uint32_t e = 0; e < edges.length; e++) {
        const Float3 &a = points[edges[e]];
        const Float3 &b = points[edges[(e + 1) % edges.length]];
        const Float3 &c = points[edges[(e + 2) % edges.length]];
        if ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x) <= 0.0f) return false;
    }
    return true;
}

/** the points as the builder takes them, in `out` */
static const Vec3f *as_vec3f(Vec3f *out, const Float3 *points, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) out[i] = vec3f::load(points[i]);
    return out;
}

void test_convex_hull_builder_2d_square(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    const Float3 points[4] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    Vec3f vec_points[4];
    hull_2d::ConvexHullBuilder2D builder = hull_2d::create(as_vec3f(vec_points, points, 4), 4);
    collections::Vec<int32_t> edges = {};

    CHECK(ctx, hull_2d::initialize(builder, 0, 1, 2, 0x7fffffff, 0.001f, edges) == hull_2d::SUCCESS);
    CHECK(ctx, edges.length == 4);
    for (int32_t i = 0; i < 4; i++) CHECK(ctx, contains(edges, i));
    CHECK(ctx, hull_2d::validate_edges(builder));
    CHECK(ctx, builder.num_edges == 4);

    edges.release();
    hull_2d::release(builder);
}

void test_convex_hull_builder_2d_square_with_interior_points(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    // the corners first, counter clockwise, then a grid strictly inside
    Float3 points[4 + 49];
    Vec3f vec_points[4 + 49];
    points[0] = {-2, -1, 0};
    points[1] = {2, -1, 0};
    points[2] = {2, 3, 0};
    points[3] = {-2, 3, 0};
    for (int32_t i = 0; i < 49; i++) points[4 + i] = {-1.5f + 0.5f * (float)(i % 7), -0.5f + 0.5f * (float)(i / 7), 0.0f};

    hull_2d::ConvexHullBuilder2D builder = hull_2d::create(as_vec3f(vec_points, points, 53), 53);
    collections::Vec<int32_t> edges = {};

    // start from an interior triangle given clockwise: the builder turns it around and grows out to the corners
    CHECK(ctx, hull_2d::initialize(builder, 4 + 8, 4 + 40, 4 + 12, 0x7fffffff, 0.001f, edges) == hull_2d::SUCCESS);
    const int32_t corners[4] = {0, 1, 2, 3};
    CHECK(ctx, is_rotation_of(edges, corners, 4));
    CHECK(ctx, hull_2d::validate_edges(builder));
    CHECK(ctx, hull_contains_all(points, 53, edges, 1e-6f));

    // a second build on the same builder reuses the freed edges and gives the same loop
    CHECK(ctx, hull_2d::initialize(builder, 0, 1, 2, 0x7fffffff, 0.001f, edges) == hull_2d::SUCCESS);
    CHECK(ctx, is_rotation_of(edges, corners, 4));
    CHECK(ctx, hull_2d::validate_edges(builder));

    edges.release();
    hull_2d::release(builder);
}

void test_convex_hull_builder_2d_collinear_points_keep_only_ends(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    // a triangle with points along each of its edges: only the three ends survive
    Float3 points[3 + 3 * 9];
    Vec3f vec_points[3 + 3 * 9];
    points[0] = {0, 0, 0};
    points[1] = {4, 0, 0};
    points[2] = {0, 4, 0};
    for (int32_t i = 0; i < 9; i++) {
        const float t = 0.1f * (float)(i + 1);
        points[3 + i] = {4.0f * t, 0.0f, 0.0f};
        points[12 + i] = {4.0f - 4.0f * t, 4.0f * t, 0.0f};
        points[21 + i] = {0.0f, 4.0f * t, 0.0f};
    }

    hull_2d::ConvexHullBuilder2D builder = hull_2d::create(as_vec3f(vec_points, points, 30), 30);
    collections::Vec<int32_t> edges = {};
    CHECK(ctx, hull_2d::initialize(builder, 0, 1, 2, 0x7fffffff, 0.001f, edges) == hull_2d::SUCCESS);
    const int32_t ends[3] = {0, 1, 2};
    CHECK(ctx, is_rotation_of(edges, ends, 3));

    // started from three points on one line, the start point between the other two is never removed:
    // only points strictly in front of an edge replace it
    CHECK(ctx, hull_2d::initialize(builder, 0, 3, 1, 0x7fffffff, 0.001f, edges) == hull_2d::SUCCESS);
    const int32_t with_start[4] = {0, 3, 1, 2};
    CHECK(ctx, is_rotation_of(edges, with_start, 4));
    CHECK(ctx, hull_2d::validate_edges(builder));

    edges.release();
    hull_2d::release(builder);
}

void test_convex_hull_builder_2d_random_points(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    uint32_t rng = 7;
    for (uint32_t round = 0; round < 8; round++) {
        Float3 points[300];
        Vec3f vec_points[300];
        for (uint32_t i = 0; i < 300; i++) points[i] = {mulberry32_unit(rng) * 10.0f - 5.0f, mulberry32_unit(rng) * 6.0f - 3.0f, 0.0f};

        hull_2d::ConvexHullBuilder2D builder = hull_2d::create(as_vec3f(vec_points, points, 300), 300);
        collections::Vec<int32_t> edges = {};
        CHECK(ctx, hull_2d::initialize(builder, 0, 1, 2, 0x7fffffff, 1e-4f, edges) == hull_2d::SUCCESS);
        CHECK(ctx, edges.length >= 3);
        CHECK(ctx, hull_2d::validate_edges(builder));
        CHECK(ctx, builder.num_edges == (int32_t)edges.length);
        CHECK(ctx, hull_contains_all(points, 300, edges, 1e-4f));
        CHECK(ctx, is_strictly_convex_ccw(points, edges));

        // each hull vertex appears once
        for (uint32_t i = 0; i < edges.length; i++) {
            for (uint32_t j = i + 1; j < edges.length; j++) CHECK(ctx, edges[i] != edges[j]);
        }

        edges.release();
        hull_2d::release(builder);
    }
}

void test_convex_hull_builder_2d_max_vertices(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    // a regular 16-gon: capped at 5 vertices the builder stops with 5 of them
    Float3 points[16];
    Vec3f vec_points[16];
    for (int32_t i = 0; i < 16; i++) {
        const float angle = 2.0f * scalar::PI * (float)i / 16.0f;
        points[i] = {scalar::cos(angle), scalar::sin(angle), 0.0f};
    }
    hull_2d::ConvexHullBuilder2D builder = hull_2d::create(as_vec3f(vec_points, points, 16), 16);
    collections::Vec<int32_t> edges = {};
    CHECK(ctx, hull_2d::initialize(builder, 0, 5, 10, 5, 1e-4f, edges) == hull_2d::MAX_VERTICES_REACHED);
    CHECK(ctx, edges.length == 5);
    CHECK(ctx, hull_2d::validate_edges(builder));
    CHECK(ctx, is_strictly_convex_ccw(points, edges));

    CHECK(ctx, hull_2d::initialize(builder, 0, 5, 10, 100, 1e-4f, edges) == hull_2d::SUCCESS);
    CHECK(ctx, edges.length == 16);

    edges.release();
    hull_2d::release(builder);
}
