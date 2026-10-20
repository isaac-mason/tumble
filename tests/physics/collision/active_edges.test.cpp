#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/active_edges.h"
#include "tumble/physics/collision/gjk.h"
#include "tumble/physics/collision/support.h"
#include "test.h"
#include "test_math.h"

/** the default triangle mesh threshold in crashcat, cos(5 degrees). */
static constexpr float MESH_COS_THRESHOLD = 0.996195f;

/** the unit normal of (b - a) x (c - a), divided through by its length; zero for a degenerate triangle */
static Vec3f face_normal(const Float3 &a, const Float3 &b, const Float3 &c) {
    const Vec3f va = vec3f::load(a);
    const Vec3f n = vec3f::cross(vec3f::subtract(vec3f::load(b), va), vec3f::subtract(vec3f::load(c), va));
    const float length = vec3f::length(n);
    return length > 0.0f ? vec3f::divide(n, vec3f::splat(length)) : n;
}

void test_active_edges_is_edge_active(TestCtx &ctx) {
    const float cos10 = scalar::cos(10.0f * scalar::DEGREES_TO_RADIANS);

    // coplanar triangles: inactive
    CHECK(ctx, !active_edges::is_edge_active(vec3f::make(0, 1, 0), vec3f::make(0, 1, 0), vec3f::make(1, 0, 0), cos10));
    // sharp convex edge (90 degrees): n1 x n2 = (0, 0, -1) points along the edge
    CHECK(ctx, active_edges::is_edge_active(vec3f::make(0, 1, 0), vec3f::make(1, 0, 0), vec3f::make(0, 0, -1), cos10));
    // sharp convex edge (45 degrees)
    CHECK(ctx, active_edges::is_edge_active(vec3f::make(0, 1, 0), unit(vec3f::make(1, 1, 0)), vec3f::make(0, 0, -1), cos10));
    // small convex edge (5 degrees) under a 10 degree threshold: inactive
    const float five = 5.0f * scalar::DEGREES_TO_RADIANS;
    CHECK(ctx, !active_edges::is_edge_active(vec3f::make(0, 1, 0), unit(vec3f::make(scalar::sin(five), scalar::cos(five), 0)), vec3f::make(0, 0, 1), cos10));
    // concave edge: inactive even with a permissive threshold
    CHECK(ctx, !active_edges::is_edge_active(vec3f::make(0, 1, 0), vec3f::make(-1, 0, 0), vec3f::make(0, 0, 1), 0.0f));
    // back to back: active whatever the edge direction
    CHECK(ctx, active_edges::is_edge_active(vec3f::make(0, 1, 0), vec3f::make(0, -1, 0), vec3f::make(1, 0, 0), cos10));
    CHECK(ctx, active_edges::is_edge_active(vec3f::make(0, 1, 0), vec3f::make(0, -1, 0), vec3f::make(-1, 0, 0), cos10));
    // nearly back to back (179 degrees): active
    const float near_opposite = scalar::cos(179.0f * scalar::DEGREES_TO_RADIANS);
    CHECK(ctx, active_edges::is_edge_active(vec3f::make(0, 1, 0), vec3f::make(0, near_opposite, 0), vec3f::make(0, 0, 1), cos10));
    // swapping which side is left flips convex to concave
    CHECK(ctx, !active_edges::is_edge_active(vec3f::make(1, 0, 0), vec3f::make(0, 1, 0), vec3f::make(0, 0, -1), cos10));
}

/**
 * the active-edge masks of two triangles sharing one edge, bit k for the edge from corner k to
 * corner k + 1 (ab, bc, ca), as crashcat's mesh data stores them. the triangle that walks the shared
 * edge forward is the left one (its interior is on the left, looking along the edge from above).
 */
static void pair_active_edges(uint32_t out[2], const Float3 *vertices, const uint32_t tri0[3], const uint32_t tri1[3], float cos_threshold) {
    out[0] = 0b111;
    out[1] = 0b111;
    for (uint32_t k0 = 0; k0 < 3; k0++) {
        for (uint32_t k1 = 0; k1 < 3; k1++) {
            if (tri0[k0] != tri1[(k1 + 1) % 3] || tri0[(k0 + 1) % 3] != tri1[k1]) continue;
            const Vec3f n0 = face_normal(vertices[tri0[0]], vertices[tri0[1]], vertices[tri0[2]]);
            const Vec3f n1 = face_normal(vertices[tri1[0]], vertices[tri1[1]], vertices[tri1[2]]);
            const Vec3f edge = vec3f::subtract(vec3f::load(vertices[tri0[(k0 + 1) % 3]]), vec3f::load(vertices[tri0[k0]]));
            if (!active_edges::is_edge_active(n0, n1, edge, cos_threshold)) {
                out[0] &= ~(1u << k0);
                out[1] &= ~(1u << k1);
            }
        }
    }
}

void test_active_edges_quad(TestCtx &ctx) {
    // a unit quad in the xz plane split along its v0-v2 diagonal, both triangles facing +y
    Float3 vertices[4] = {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}};
    // v0 v2 v1 walks the diagonal forward as its edge ab; v0 v3 v2 walks it back as its edge ca
    const uint32_t tri0[3] = {0, 2, 1};
    const uint32_t tri1[3] = {0, 3, 2};
    CHECK(ctx, vec3f::y(face_normal(vertices[0], vertices[2], vertices[1])) == 1.0f);
    CHECK(ctx, vec3f::y(face_normal(vertices[0], vertices[3], vertices[2])) == 1.0f);

    uint32_t masks[2];

    // flat: the shared edge is inactive in both, ab in the first and ca in the second
    pair_active_edges(masks, vertices, tri0, tri1, MESH_COS_THRESHOLD);
    CHECK(ctx, masks[0] == 0b110);
    CHECK(ctx, masks[1] == 0b011);

    // folded into a ridge by dropping v1 0.5: a convex crease of about 35 degrees, active
    vertices[1].y = -0.5f;
    pair_active_edges(masks, vertices, tri0, tri1, MESH_COS_THRESHOLD);
    CHECK(ctx, masks[0] == 0b111);
    CHECK(ctx, masks[1] == 0b111);

    // folded just past the threshold (about 5.7 degrees): still active
    vertices[1].y = -0.07f;
    pair_active_edges(masks, vertices, tri0, tri1, MESH_COS_THRESHOLD);
    CHECK(ctx, masks[0] == 0b111 && masks[1] == 0b111);

    // folded just under it (about 4 degrees): smooth enough to be inactive
    vertices[1].y = -0.05f;
    pair_active_edges(masks, vertices, tri0, tri1, MESH_COS_THRESHOLD);
    CHECK(ctx, masks[0] == 0b110 && masks[1] == 0b011);

    // folded up into a valley: concave, inactive however sharp
    vertices[1].y = 0.5f;
    pair_active_edges(masks, vertices, tri0, tri1, MESH_COS_THRESHOLD);
    CHECK(ctx, masks[0] == 0b110 && masks[1] == 0b011);
    vertices[1].y = 5.0f;
    pair_active_edges(masks, vertices, tri0, tri1, MESH_COS_THRESHOLD);
    CHECK(ctx, masks[0] == 0b110 && masks[1] == 0b011);
}

void test_active_edges_fix_normal(TestCtx &ctx) {
    // crashcat's fixNormal cases: a right triangle in the xz plane, facing +y
    const Float3 v0 = {0, 0, 0}, v1 = {1, 0, 0}, v2 = {0, 0, 1};
    const Float3 triangle_normal = {0, 1, 0};
    const Float3 still = {0, 0, 0};
    struct Case {
        uint32_t active_edges;
        Float3 point;
        Float3 normal;
        bool keeps_normal;
    };
    const Case cases[] = {
        {0b111, {0.5f, 0, 0.5f}, {0.1f, 0.9f, 0.1f}, true},           // all edges active
        {0b000, {0.5f, 0, 0.5f}, {0.1f, 0.9f, 0.1f}, false},          // no edges active
        {0b101, {0.4f, 0, 0.4f}, {0.1f, 0.9f, 0.1f}, false},          // interior hit
        {0b001, {0.5f, 0, 0}, {0, 0.9f, -0.1f}, true},                // on active edge v0-v1
        {0b110, {0.5f, 0, 0}, {0, 0.9f, -0.1f}, false},               // on inactive edge v0-v1
        {0b010, {0.5f, 0, 0.5f}, {0.1f, 0.9f, 0}, true},              // on active edge v1-v2
        {0b100, {0, 0, 0.5f}, {-0.1f, 0.9f, 0}, true},                // on active edge v2-v0
        {0b001, {0.00001f, 0, 0.00001f}, {0, 0.9f, 0.1f}, true},      // at v0, edge 0 active
        {0b010, {0.00001f, 0, 0.00001f}, {0, 0.9f, 0.1f}, false},     // at v0, neither neighbour active
        {0b010, {0.99999f, 0, 0.00001f}, {0.1f, 0.9f, 0}, true},      // at v1, edge 1 active
        {0b100, {0.00001f, 0, 0.99999f}, {-0.1f, 0.9f, 0.1f}, true},  // at v2, edge 2 active
    };
    for (const Case &c : cases) {
        const Vec3f result = active_edges::fix_normal(vec3f::load(v0), vec3f::load(v1), vec3f::load(v2), vec3f::load(triangle_normal), c.active_edges,
                                                      vec3f::load(c.point), vec3f::load(c.normal), vec3f::load(still));
        CHECK(ctx, vec3f::exact_equals(result, vec3f::load(c.keeps_normal ? c.normal : triangle_normal)));
    }

    // the movement heuristic: a normal that opposes the motion less than the triangle normal is kept
    const Float3 slanted = {0.707f, 0.707f, 0};
    const Float3 moving = {-1, 0, 0};
    CHECK(ctx, vec3f::exact_equals(active_edges::fix_normal(vec3f::load(v0), vec3f::load(v1), vec3f::load(v2), vec3f::load(triangle_normal), 0b000,
                                                            vec3f::make(0.5f, 0, 0), vec3f::load(slanted), vec3f::load(moving)),
                                   vec3f::load(slanted)));

    // a normal within a degree of the triangle normal skips the edge test
    const Vec3f nearly = unit(vec3f::make(0.001f, 1, 0.001f));
    CHECK(ctx, vec3f::exact_equals(active_edges::fix_normal(vec3f::load(v0), vec3f::load(v1), vec3f::load(v2), vec3f::load(triangle_normal), 0b001,
                                                            vec3f::make(0.5f, 0, 0.5f), nearly, vec3f::load(still)),
                                   nearly));
}

void test_active_edges_fix_normal_sphere_on_quad(TestCtx &ctx) {
    // the flat quad of test_active_edges_quad: its first triangle v0 v2 v1 has the diagonal as edge ab
    const Float3 vertices[4] = {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}};
    const uint32_t tri0[3] = {0, 2, 1};
    const uint32_t tri1[3] = {0, 3, 2};
    uint32_t masks[2];
    pair_active_edges(masks, vertices, tri0, tri1, MESH_COS_THRESHOLD);
    CHECK(ctx, masks[0] == 0b110);

    // a sphere of radius 0.2 resting over the second triangle, just across the diagonal and sunk into
    // the quad: against the first triangle alone its contact is on the diagonal and the penetration
    // axis tilts toward the sphere's centre. all in the sphere's space, as the mesh narrowphase works.
    const Float3 centre = {0.45f, 0.1f, 0.55f};
    const Float3 a = {vertices[tri0[0]].x - centre.x, vertices[tri0[0]].y - centre.y, vertices[tri0[0]].z - centre.z};
    const Float3 b = {vertices[tri0[1]].x - centre.x, vertices[tri0[1]].y - centre.y, vertices[tri0[1]].z - centre.z};
    const Float3 c = {vertices[tri0[2]].x - centre.x, vertices[tri0[2]].y - centre.y, vertices[tri0[2]].z - centre.z};

    support::Support sphere = support::create();
    sphere.get_support = support::sphere_support;
    sphere.convex_radius = 0.2f;
    support::Support triangle = support::create();
    support::set_triangle_support(triangle, vec3f::load(a), vec3f::load(b), vec3f::load(c), 0.0f);

    gjk::GjkClosestPoints result = gjk::create_gjk_closest_points();
    gjk::closest_points(result, sphere, triangle, 1e-4f, vec3f::make(0, -1, 0), __FLT_MAX__);
    // the core is within the sphere's radius of the diagonal's midpoint
    CHECK(ctx, scalar::sqrt(result.squared_distance) < sphere.convex_radius);
    CHECK(ctx, scalar::abs(vec3f::x(result.point_b) - (0.5f - centre.x)) < 1e-5f && scalar::abs(vec3f::z(result.point_b) - (0.5f - centre.z)) < 1e-5f);
    const Vec3f axis = unit(result.penetration_axis);
    CHECK(ctx, vec3f::x(axis) > 0.1f && vec3f::z(axis) < -0.1f);

    // the triangle normal as the narrowphase passes it: pointing into the triangle, along the axis
    const Float3 into_triangle = {0, -1, 0};
    const Float3 still = {0, 0, 0};

    // the diagonal is inactive: the contact pushes straight out of the quad
    const Vec3f fixed = active_edges::fix_normal(vec3f::load(a), vec3f::load(b), vec3f::load(c), vec3f::load(into_triangle), masks[0], result.point_b,
                                                 result.penetration_axis, vec3f::load(still));
    CHECK(ctx, vec3f::exact_equals(fixed, vec3f::load(into_triangle)));

    // were the diagonal a real edge (active), the tilted axis stands
    const Vec3f kept = active_edges::fix_normal(vec3f::load(a), vec3f::load(b), vec3f::load(c), vec3f::load(into_triangle), 0b001, result.point_b,
                                                result.penetration_axis, vec3f::load(still));
    CHECK(ctx, vec3f::exact_equals(kept, result.penetration_axis));
}
