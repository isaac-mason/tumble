#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/closest_points.h"
#include "tumble/physics/collision/gjk.h"
#include "test.h"
#include "test_math.h"
#include "test_rng.h"

static constexpr float EPS = 1e-5f;

static Float3 sub(const Float3 &a, const Float3 &b) {
    Float3 out;
    return vec3f::store(out, vec3f::subtract(vec3f::load(a), vec3f::load(b)));
}

static Float3 random_point(uint32_t &rng, float extent) {
    return {(mulberry32_unit(rng) * 2.0f - 1.0f) * extent, (mulberry32_unit(rng) * 2.0f - 1.0f) * extent, (mulberry32_unit(rng) * 2.0f - 1.0f) * extent};
}

/**
 * the point is reproduced from the barycentrics of the vertices `point_set` selects (relative to
 * the query, as the kernels see them), which sum to one and lie in [0, 1].
 */
static void check_reproduced(TestCtx &ctx, const GjkClosestPoints::Result &result, const Float3 *vertices, uint32_t count) {
    Float3 picked[4];
    uint32_t n = 0;
    for (uint32_t i = 0; i < count; i++) {
        if (result.point_set & (1u << i)) picked[n++] = vertices[i];
    }
    if (n == 1) {
        CHECK(ctx, near3(result.point, picked[0], EPS));
    } else if (n == 2) {
        BarycentricCoordinatesResult bary;
        closest_points::compute_barycentric_coordinates_2d(bary, vec3f::load(picked[0]), vec3f::load(picked[1]), 1e-10f);
        CHECK(ctx, bary.is_valid);
        CHECK_NEAR(ctx, bary.u + bary.v, 1.0f, EPS);
        CHECK(ctx, bary.u >= -EPS && bary.v >= -EPS);
        Vec3f p = vec3f::scale(vec3f::load(picked[0]), bary.u);
        p = vec3f::scale_and_add(p, vec3f::load(picked[1]), bary.v);
        CHECK(ctx, near3(p, result.point, 1e-4f));
    } else if (n == 3) {
        BarycentricCoordinatesResult bary;
        closest_points::compute_barycentric_coordinates_3d(bary, vec3f::load(picked[0]), vec3f::load(picked[1]), vec3f::load(picked[2]), 1e-10f);
        CHECK(ctx, bary.is_valid);
        CHECK_NEAR(ctx, bary.u + bary.v + bary.w, 1.0f, EPS);
        CHECK(ctx, bary.u >= -EPS && bary.v >= -EPS && bary.w >= -EPS);
        Vec3f p = vec3f::scale(vec3f::load(picked[0]), bary.u);
        p = vec3f::scale_and_add(p, vec3f::load(picked[1]), bary.v);
        p = vec3f::scale_and_add(p, vec3f::load(picked[2]), bary.w);
        CHECK(ctx, near3(p, result.point, 1e-4f));
    } else {
        // the whole tetrahedron: the origin is inside
        CHECK(ctx, n == 4 && near3(result.point, 0.0f, 0.0f, 0.0f, EPS));
    }
}

void test_closest_points_barycentric_2d(TestCtx &ctx) {
    BarycentricCoordinatesResult result;

    // origin halfway between the ends of a symmetric segment
    closest_points::compute_barycentric_coordinates_2d(result, vec3f::make(1, 0, 0), vec3f::make(0, 1, 0), 1e-12f);
    CHECK(ctx, result.is_valid);
    CHECK_NEAR(ctx, result.u, 0.5f, EPS);
    CHECK_NEAR(ctx, result.v, 0.5f, EPS);

    // origin at a
    closest_points::compute_barycentric_coordinates_2d(result, vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), 1e-12f);
    CHECK(ctx, result.is_valid && result.u == 1.0f && result.v == 0.0f);

    // origin a quarter of the way along
    closest_points::compute_barycentric_coordinates_2d(result, vec3f::make(-1, 2, 0), vec3f::make(3, 2, 0), 1e-12f);
    CHECK(ctx, result.is_valid);
    CHECK_NEAR(ctx, result.u, 0.75f, EPS);
    CHECK_NEAR(ctx, result.v, 0.25f, EPS);

    // beyond b: v > 1, u < 0, still summing to one
    closest_points::compute_barycentric_coordinates_2d(result, vec3f::make(-3, 1, 0), vec3f::make(-1, 1, 0), 1e-12f);
    CHECK(ctx, result.is_valid);
    CHECK_NEAR(ctx, result.v, 1.5f, EPS);
    CHECK_NEAR(ctx, result.u + result.v, 1.0f, EPS);

    // degenerate segment falls back to the nearer end and is invalid
    closest_points::compute_barycentric_coordinates_2d(result, vec3f::make(1, 1, 1), vec3f::make(1, 1, 1), 1e-12f);
    CHECK(ctx, !result.is_valid);
    closest_points::compute_barycentric_coordinates_2d(result, vec3f::make(2, 0, 0), vec3f::make(2.000001f, 0, 0), 1e-10f);
    CHECK(ctx, !result.is_valid && result.u == 1.0f && result.v == 0.0f);
    closest_points::compute_barycentric_coordinates_2d(result, vec3f::make(2.000001f, 0, 0), vec3f::make(2, 0, 0), 1e-10f);
    CHECK(ctx, !result.is_valid && result.u == 0.0f && result.v == 1.0f);
}

void test_closest_points_barycentric_3d(TestCtx &ctx) {
    BarycentricCoordinatesResult result;

    // origin at a
    closest_points::compute_barycentric_coordinates_3d(result, vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), vec3f::make(0, 1, 0), 1e-12f);
    CHECK(ctx, result.is_valid);
    CHECK_NEAR(ctx, result.u, 1.0f, EPS);
    CHECK_NEAR(ctx, result.v, 0.0f, EPS);
    CHECK_NEAR(ctx, result.w, 0.0f, EPS);

    // the symmetric triangle projects the origin onto its centroid
    closest_points::compute_barycentric_coordinates_3d(result, vec3f::make(1, 0, 0), vec3f::make(0, 1, 0), vec3f::make(0, 0, 1), 1e-12f);
    CHECK(ctx, result.is_valid);
    CHECK_NEAR(ctx, result.u, 1.0f / 3.0f, EPS);
    CHECK_NEAR(ctx, result.v, 1.0f / 3.0f, EPS);
    CHECK_NEAR(ctx, result.w, 1.0f / 3.0f, EPS);

    // both branches (d00 <= d22 and not) reproduce the projection of the origin
    const Float3 tris[2][3] = {{{-1, -1, 2}, {3, -1, 2}, {-1, 2, 2}}, {{-1, -1, 2}, {0.5f, -0.5f, 2}, {-1, 2, 2}}};
    for (const auto &t : tris) {
        closest_points::compute_barycentric_coordinates_3d(result, vec3f::load(t[0]), vec3f::load(t[1]), vec3f::load(t[2]), 1e-12f);
        CHECK(ctx, result.is_valid);
        CHECK_NEAR(ctx, result.u + result.v + result.w, 1.0f, EPS);
        Vec3f p = vec3f::scale(vec3f::load(t[0]), result.u);
        p = vec3f::scale_and_add(p, vec3f::load(t[1]), result.v);
        p = vec3f::scale_and_add(p, vec3f::load(t[2]), result.w);
        CHECK(ctx, near3(p, 0, 0, 2, EPS));
    }

    // collinear points are degenerate
    closest_points::compute_barycentric_coordinates_3d(result, vec3f::make(0, 0, 0), vec3f::make(1, 0, 0), vec3f::make(2, 0, 0), 1e-12f);
    CHECK(ctx, !result.is_valid);
    CHECK_NEAR(ctx, result.u + result.v + result.w, 1.0f, EPS);
    // so is a long thin sliver: the apex 1e-5 off a 20 long base is far under the 0.06 degree cutoff, and the fallback
    // lands on an edge
    closest_points::compute_barycentric_coordinates_3d(result, vec3f::make(-10, 1, 0), vec3f::make(10, 1, 0), vec3f::make(0, 1.00001f, 0), 1e-12f);
    CHECK(ctx, !result.is_valid);
    CHECK_NEAR(ctx, result.u + result.v + result.w, 1.0f, EPS);
    CHECK(ctx, result.u == 0.0f || result.v == 0.0f || result.w == 0.0f);
}

void test_closest_points_line(TestCtx &ctx) {
    GjkClosestPoints::Result result;
    // query at the origin, segment from (-1, 1, 0) to (3, 1, 0): interior
    {
        const Float3 v[2] = {{-1, 1, 0}, {3, 1, 0}};
        gjk::compute_closest_point_on_line(result, vec3f::load(v[0]), vec3f::load(v[1]), 1e-10f);
        CHECK(ctx, result.point_set == 0b11);
        CHECK(ctx, near3(result.point, 0, 1, 0, EPS));
        check_reproduced(ctx, result, v, 2);
    }
    // vertex a
    {
        const Float3 v[2] = {{1, 1, 0}, {3, 2, 0}};
        gjk::compute_closest_point_on_line(result, vec3f::load(v[0]), vec3f::load(v[1]), 1e-10f);
        CHECK(ctx, result.point_set == 0b01);
        CHECK(ctx, near3(result.point, v[0], EPS));
    }
    // vertex b
    {
        const Float3 v[2] = {{-3, 1, 1}, {-1, 1, 0}};
        gjk::compute_closest_point_on_line(result, vec3f::load(v[0]), vec3f::load(v[1]), 1e-10f);
        CHECK(ctx, result.point_set == 0b10);
        CHECK(ctx, near3(result.point, v[1], EPS));
    }
    // a degenerate segment returns its nearer end
    {
        gjk::compute_closest_point_on_line(result, vec3f::make(2, 0, 0), vec3f::make(2.000001f, 0, 0), 1e-10f);
        CHECK(ctx, result.point_set == 0b01);
    }
}

/** the reference: the plane projection when inside, else the best clamped edge point. */
static void reference_triangle(GjkClosestPoints::Result &out, const Float3 &stored_a, const Float3 &stored_b, const Float3 &stored_c) {
    const Vec3f a = vec3f::load(stored_a), b = vec3f::load(stored_b), c = vec3f::load(stored_c);
    const Vec3f n = vec3f::cross(vec3f::subtract(b, a), vec3f::subtract(c, a));
    const float nn = vec3f::dot(n, n);
    const Vec3f projected = vec3f::scale(n, vec3f::dot(a, n) / nn);
    const Vec3f to_a = vec3f::subtract(a, projected), to_b = vec3f::subtract(b, projected), to_c = vec3f::subtract(c, projected);
    const Vec3f cross_bc = vec3f::cross(to_b, to_c);
    const Vec3f cross_ca = vec3f::cross(to_c, to_a);
    const Vec3f cross_ab = vec3f::cross(to_a, to_b);
    if (vec3f::dot(cross_bc, n) >= 0.0f && vec3f::dot(cross_ca, n) >= 0.0f && vec3f::dot(cross_ab, n) >= 0.0f) {
        out.point = projected;
        out.point_set = 0b111;
        return;
    }
    const Vec3f ends[3][2] = {{a, b}, {b, c}, {a, c}};
    const uint32_t bits[3][2] = {{0b001, 0b010}, {0b010, 0b100}, {0b001, 0b100}};
    float best = __builtin_inff();
    for (uint32_t e = 0; e < 3; e++) {
        const Vec3f p = ends[e][0];
        const Vec3f d = vec3f::subtract(ends[e][1], p);
        const float t = scalar::clamp(-vec3f::dot(p, d) / vec3f::dot(d, d), 0.0f, 1.0f);
        const Vec3f q = vec3f::scale_and_add(p, d, t);
        const float dist = vec3f::dot(q, q);
        if (dist < best) {
            best = dist;
            out.point = q;
            out.point_set = t == 0.0f ? bits[e][0] : t == 1.0f ? bits[e][1] : (bits[e][0] | bits[e][1]);
        }
    }
}

/** the closest point to `query` on triangle abc, run with the triangle moved so the query is at the origin. */
static void closest_on_triangle(TestCtx &ctx, const Float3 tri[3], const Float3 &query, const Float3 &expected, uint32_t expected_set) {
    const Float3 v[3] = {sub(tri[0], query), sub(tri[1], query), sub(tri[2], query)};
    GjkClosestPoints::Result result;
    gjk::compute_closest_point_on_triangle(result, vec3f::load(v[0]), vec3f::load(v[1]), vec3f::load(v[2]), false, 1e-10f);
    CHECK(ctx, result.point_set == expected_set);
    CHECK(ctx, near3(result.point, sub(expected, query), EPS));
    check_reproduced(ctx, result, v, 3);
}

void test_closest_points_triangle_regions(TestCtx &ctx) {
    // |bc| > |ac|: the kernel keeps a and c
    const Float3 t[3] = {{0, 0, 0}, {4, 0, 0}, {0, 4, 0}};
    closest_on_triangle(ctx, t, {1, 1, 1}, {1, 1, 0}, 0b111);   // face
    closest_on_triangle(ctx, t, {2, -1, 1}, {2, 0, 0}, 0b011);  // edge ab
    closest_on_triangle(ctx, t, {-1, 2, 1}, {0, 2, 0}, 0b101);  // edge ac
    closest_on_triangle(ctx, t, {3, 3, 1}, {2, 2, 0}, 0b110);   // edge bc
    closest_on_triangle(ctx, t, {-1, -1, 1}, t[0], 0b001);      // vertex a
    closest_on_triangle(ctx, t, {5, -1, -1}, t[1], 0b010);      // vertex b
    closest_on_triangle(ctx, t, {-1, 5, 1}, t[2], 0b100);       // vertex c

    // |bc| < |ac|: the kernel swaps a and c internally; the reported sets are still in input order
    const Float3 s[3] = {{0, 0, 0}, {4, 0, 0}, {3, 1, 0}};
    closest_on_triangle(ctx, s, {2.5f, 0.5f, -2}, {2.5f, 0.5f, 0}, 0b111);  // face
    closest_on_triangle(ctx, s, {2, -1, 1}, {2, 0, 0}, 0b011);              // edge ab
    closest_on_triangle(ctx, s, {1, 3, 1}, {1.8f, 0.6f, 0}, 0b101);         // edge ac
    closest_on_triangle(ctx, s, {5, 2, 1}, {3.5f, 0.5f, 0}, 0b110);         // edge bc
    closest_on_triangle(ctx, s, {-1, -1, 1}, s[0], 0b001);                  // vertex a
    closest_on_triangle(ctx, s, {6, -1, 0}, s[1], 0b010);                   // vertex b
    closest_on_triangle(ctx, s, {3.2f, 3, 0}, s[2], 0b100);                 // vertex c
}

void test_closest_points_triangle_random(TestCtx &ctx) {
    uint32_t rng = 7;
    for (uint32_t i = 0; i < 400; i++) {
        const Float3 v[3] = {random_point(rng, 3.0f), random_point(rng, 3.0f), random_point(rng, 3.0f)};
        const Vec3f n = vec3f::cross(vec3f::load(sub(v[1], v[0])), vec3f::load(sub(v[2], v[0])));
        if (vec3f::length(n) < 0.5f) continue;
        GjkClosestPoints::Result result, expected;
        gjk::compute_closest_point_on_triangle(result, vec3f::load(v[0]), vec3f::load(v[1]), vec3f::load(v[2]), false, 1e-10f);
        reference_triangle(expected, v[0], v[1], v[2]);
        CHECK(ctx, near3(result.point, expected.point, 1e-4f));
        check_reproduced(ctx, result, v, 3);
    }
}

void test_closest_points_triangle_degenerate(TestCtx &ctx) {
    GjkClosestPoints::Result result;
    // collinear: nearest is vertex a, unless c must be part of the answer
    gjk::compute_closest_point_on_triangle(result, vec3f::make(1, 1, 0), vec3f::make(2, 1, 0), vec3f::make(3, 1, 0), false, 1e-10f);
    CHECK(ctx, result.point_set == 0b0001 && near3(result.point, 1, 1, 0, EPS));
    gjk::compute_closest_point_on_triangle(result, vec3f::make(1, 1, 0), vec3f::make(2, 1, 0), vec3f::make(3, 1, 0), true, 1e-10f);
    CHECK(ctx, result.point_set == 0b0101 && near3(result.point, 1, 1, 0, EPS));
    // collinear through the line's interior
    gjk::compute_closest_point_on_triangle(result, vec3f::make(-1, 1, 0), vec3f::make(1, 1, 0), vec3f::make(3, 1, 0), false, 1e-10f);
    CHECK(ctx, near3(result.point, 0, 1, 0, EPS));
    CHECK(ctx, (result.point_set & (result.point_set - 1)) != 0);  // an edge, not a vertex
}

/** the closest point to `query` on the unit corner tetrahedron, run with it moved so the query is at the origin. */
static void closest_on_tetrahedron(TestCtx &ctx, const Float3 &query, const Float3 &expected, uint32_t expected_set) {
    const Float3 t[4] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const Float3 v[4] = {sub(t[0], query), sub(t[1], query), sub(t[2], query), sub(t[3], query)};
    GjkClosestPoints::Result result;
    gjk::compute_closest_point_on_tetrahedron(result, vec3f::load(v[0]), vec3f::load(v[1]), vec3f::load(v[2]), vec3f::load(v[3]), false, 1e-5f);
    CHECK(ctx, result.point_set == expected_set);
    CHECK(ctx, near3(result.point, sub(expected, query), EPS));
    check_reproduced(ctx, result, v, 4);
}

void test_closest_points_tetrahedron_regions(TestCtx &ctx) {
    const float third = 1.0f / 3.0f;
    closest_on_tetrahedron(ctx, {0.1f, 0.1f, 0.1f}, {0.1f, 0.1f, 0.1f}, 0b1111);  // inside
    closest_on_tetrahedron(ctx, {0.2f, 0.2f, -1}, {0.2f, 0.2f, 0}, 0b0111);        // face abc
    closest_on_tetrahedron(ctx, {-1, 0.2f, 0.2f}, {0, 0.2f, 0.2f}, 0b1101);        // face acd
    closest_on_tetrahedron(ctx, {0.2f, -1, 0.2f}, {0.2f, 0, 0.2f}, 0b1011);        // face adb
    closest_on_tetrahedron(ctx, {1, 1, 1}, {third, third, third}, 0b1110);         // face bcd
    closest_on_tetrahedron(ctx, {0.5f, -1, -1}, {0.5f, 0, 0}, 0b0011);             // edge ab
    closest_on_tetrahedron(ctx, {-1, 0.5f, -1}, {0, 0.5f, 0}, 0b0101);             // edge ac
    closest_on_tetrahedron(ctx, {-1, -1, 0.5f}, {0, 0, 0.5f}, 0b1001);             // edge ad
    closest_on_tetrahedron(ctx, {1, 1, -1}, {0.5f, 0.5f, 0}, 0b0110);              // edge bc
    closest_on_tetrahedron(ctx, {1, -1, 1}, {0.5f, 0, 0.5f}, 0b1010);              // edge bd
    closest_on_tetrahedron(ctx, {-1, 1.5f, 1.5f}, {0, 0.5f, 0.5f}, 0b1100);        // edge cd
    closest_on_tetrahedron(ctx, {-1, -1, -1}, {0, 0, 0}, 0b0001);                  // vertex a
    closest_on_tetrahedron(ctx, {3, -1, -1}, {1, 0, 0}, 0b0010);                   // vertex b
    closest_on_tetrahedron(ctx, {-1, 3, -1}, {0, 1, 0}, 0b0100);                   // vertex c
    closest_on_tetrahedron(ctx, {-1, -1, 3}, {0, 0, 1}, 0b1000);                   // vertex d
}
