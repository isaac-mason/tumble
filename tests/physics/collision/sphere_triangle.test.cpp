#include "math/scalar.h"
#include "math/vec3f.h"
#include "physics/collision/sphere_triangle.h"
#include "physics/collision/triangle.h"
#include "test.h"
#include "test_math.h"

static bool is_inf(float f) { return f == __builtin_inff(); }

void test_sphere_triangle_ray_sphere(TestCtx &ctx) {
    const Float3 center = {0.0f, 0.0f, 0.0f};

    // from z = -5 over 10 units: enters the unit sphere at z = -1, 4 units in
    CHECK_NEAR(ctx, sphere_triangle::ray_sphere(vec3f::make(0.0f, 0.0f, -5.0f), vec3f::make(0.0f, 0.0f, 10.0f), vec3f::load(center), 1.0f), 0.4f, 1e-6f);
    // offset past the radius: miss
    CHECK(ctx, is_inf(sphere_triangle::ray_sphere(vec3f::make(2.0f, 0.0f, -5.0f), vec3f::make(0.0f, 0.0f, 10.0f), vec3f::load(center), 1.0f)));
    // grazing at exactly the radius: the tangent point, half way
    CHECK_NEAR(ctx, sphere_triangle::ray_sphere(vec3f::make(1.0f, 0.0f, -5.0f), vec3f::make(0.0f, 0.0f, 10.0f), vec3f::load(center), 1.0f), 0.5f, 1e-6f);
    // too short to reach the surface
    CHECK(ctx, is_inf(sphere_triangle::ray_sphere(vec3f::make(0.0f, 0.0f, -5.0f), vec3f::make(0.0f, 0.0f, 3.0f), vec3f::load(center), 1.0f)));
    // pointing away
    CHECK(ctx, is_inf(sphere_triangle::ray_sphere(vec3f::make(0.0f, 0.0f, -5.0f), vec3f::make(0.0f, 0.0f, -10.0f), vec3f::load(center), 1.0f)));
    // starting inside: fraction 0, whether the ray leaves or stays inside
    CHECK(ctx, sphere_triangle::ray_sphere(vec3f::make(0.0f, 0.0f, 0.5f), vec3f::make(0.0f, 0.0f, 10.0f), vec3f::load(center), 1.0f) == 0.0f);
    CHECK(ctx, sphere_triangle::ray_sphere(vec3f::make(0.0f, 0.0f, 0.0f), vec3f::make(0.0f, 0.0f, 0.1f), vec3f::load(center), 1.0f) == 0.0f);
    // zero-length ray: inside is 0, outside a miss
    CHECK(ctx, sphere_triangle::ray_sphere(vec3f::make(0.0f, 0.2f, 0.0f), vec3f::make(0.0f, 0.0f, 0.0f), vec3f::load(center), 1.0f) == 0.0f);
    CHECK(ctx, is_inf(sphere_triangle::ray_sphere(vec3f::make(0.0f, 2.0f, 0.0f), vec3f::make(0.0f, 0.0f, 0.0f), vec3f::load(center), 1.0f)));

    // from the origin: same answers as ray_sphere with the sphere moved instead
    CHECK_NEAR(ctx, sphere_triangle::ray_sphere_from_origin(vec3f::make(10.0f, 0.0f, 0.0f), vec3f::make(5.0f, 0.0f, 0.0f), 1.0f), 0.4f, 1e-6f);
    CHECK(ctx, is_inf(sphere_triangle::ray_sphere_from_origin(vec3f::make(10.0f, 0.0f, 0.0f), vec3f::make(5.0f, 2.0f, 0.0f), 1.0f)));
    CHECK(ctx, sphere_triangle::ray_sphere_from_origin(vec3f::make(10.0f, 0.0f, 0.0f), vec3f::make(0.5f, 0.0f, 0.0f), 1.0f) == 0.0f);
    const float moved = sphere_triangle::ray_sphere(vec3f::make(0.0f, 0.0f, 0.0f), vec3f::make(3.0f, 4.0f, 0.0f), vec3f::make(1.0f, 2.0f, 0.0f), 0.5f);
    CHECK_NEAR(ctx, sphere_triangle::ray_sphere_from_origin(vec3f::make(3.0f, 4.0f, 0.0f), vec3f::make(1.0f, 2.0f, 0.0f), 0.5f), moved, 1e-6f);
}

void test_sphere_triangle_ray_cylinder(TestCtx &ctx) {
    // a unit-radius cylinder along x from -1 to 1, centered at z = 3; the ray goes 10 units up z
    const Float3 direction = {0.0f, 0.0f, 10.0f};
    CHECK_NEAR(ctx, sphere_triangle::ray_cylinder(vec3f::load(direction), vec3f::make(-1.0f, 0.0f, 3.0f), vec3f::make(1.0f, 0.0f, 3.0f), 1.0f), 0.2f, 1e-6f);
    // endpoints swapped: same side surface
    CHECK_NEAR(ctx, sphere_triangle::ray_cylinder(vec3f::load(direction), vec3f::make(1.0f, 0.0f, 3.0f), vec3f::make(-1.0f, 0.0f, 3.0f), 1.0f), 0.2f, 1e-6f);
    // axis 0.6 off the ray in y: the chord starts at z = 3 - 0.8
    CHECK_NEAR(ctx, sphere_triangle::ray_cylinder(vec3f::load(direction), vec3f::make(-1.0f, -0.6f, 3.0f), vec3f::make(1.0f, -0.6f, 3.0f), 1.0f), 0.22f, 1e-6f);
    // the ray passes beside the segment (past cap a or past cap b): miss
    CHECK(ctx, is_inf(sphere_triangle::ray_cylinder(vec3f::load(direction), vec3f::make(1.0f, 0.0f, 3.0f), vec3f::make(3.0f, 0.0f, 3.0f), 1.0f)));
    CHECK(ctx, is_inf(sphere_triangle::ray_cylinder(vec3f::load(direction), vec3f::make(-3.0f, 0.0f, 3.0f), vec3f::make(-1.0f, 0.0f, 3.0f), 1.0f)));
    // too short
    CHECK(ctx, is_inf(sphere_triangle::ray_cylinder(vec3f::make(0.0f, 0.0f, 1.0f), vec3f::make(-1.0f, 0.0f, 3.0f), vec3f::make(1.0f, 0.0f, 3.0f), 1.0f)));
    // radius too small to reach the ray
    CHECK(ctx, is_inf(sphere_triangle::ray_cylinder(vec3f::load(direction), vec3f::make(-1.0f, 2.0f, 3.0f), vec3f::make(1.0f, 2.0f, 3.0f), 1.0f)));
    // along the axis: parallel, no side hit
    CHECK(ctx, is_inf(sphere_triangle::ray_cylinder(vec3f::load(direction), vec3f::make(0.5f, 0.0f, 3.0f), vec3f::make(0.5f, 0.0f, 5.0f), 1.0f)));
    // starting inside the side surface: only entering counts, so a miss
    CHECK(ctx, is_inf(sphere_triangle::ray_cylinder(vec3f::load(direction), vec3f::make(-1.0f, 0.0f, 0.0f), vec3f::make(1.0f, 0.0f, 0.0f), 1.0f)));
}

/** what triangle-mesh does for a sphere at the origin against a triangle: depth and the axis toward the triangle. */
struct SphereVsTriangle {
    float penetration;
    Float3 axis;
    uint32_t feature;
};

static SphereVsTriangle sphere_vs_triangle(const Float3 &a, const Float3 &b, const Float3 &c, const Float3 &center, float radius) {
    const Vec3f ra = vec3f::subtract(vec3f::load(a), vec3f::load(center));
    const Vec3f rb = vec3f::subtract(vec3f::load(b), vec3f::load(center));
    const Vec3f rc = vec3f::subtract(vec3f::load(c), vec3f::load(center));
    triangle::ClosestPointOnTriangleResult closest = triangle::create_closest_point_on_triangle_result();
    triangle::get_closest_point_on_triangle(closest, ra, rb, rc);
    SphereVsTriangle out;
    out.penetration = radius - scalar::sqrt(closest.distance_sq);
    vec3f::store(out.axis, unit(closest.point));
    out.feature = closest.feature;
    return out;
}

void test_sphere_triangle_penetration_regions(TestCtx &ctx) {
    const Float3 a = {0.0f, 0.0f, 0.0f};
    const Float3 b = {2.0f, 0.0f, 0.0f};
    const Float3 c = {0.0f, 0.0f, 2.0f};
    const float inv_sqrt2 = 1.0f / scalar::sqrt(2.0f);

    // radius 1 resting 0.75 above the interior: 0.25 deep, pushed straight down onto the face
    SphereVsTriangle r = sphere_vs_triangle(a, b, c, {0.5f, 0.75f, 0.5f}, 1.0f);
    CHECK(ctx, r.feature == triangle::FACE);
    CHECK_NEAR(ctx, r.penetration, 0.25f, 1e-6f);
    CHECK_NEAR(ctx, r.axis.x, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, r.axis.y, -1.0f, 1e-6f);
    CHECK_NEAR(ctx, r.axis.z, 0.0f, 1e-6f);
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[r.feature] == 0b000);

    // beyond edge ab, 0.5 out and 0.5 up: the edge is sqrt(0.5) away, axis diagonal toward it
    r = sphere_vs_triangle(a, b, c, {1.0f, 0.5f, -0.5f}, 1.0f);
    CHECK(ctx, r.feature == triangle::EDGE_AB);
    CHECK_NEAR(ctx, r.penetration, 1.0f - inv_sqrt2, 1e-6f);
    CHECK_NEAR(ctx, r.axis.x, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, r.axis.y, -inv_sqrt2, 1e-6f);
    CHECK_NEAR(ctx, r.axis.z, inv_sqrt2, 1e-6f);
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[r.feature] == 0b001);

    // beyond vertex b along the diagonal: 0.3 from it in each of x, y and -z
    r = sphere_vs_triangle(a, b, c, {2.3f, 0.3f, -0.3f}, 1.0f);
    CHECK(ctx, r.feature == triangle::VERTEX_B);
    const float d = scalar::sqrt(0.27f);
    CHECK_NEAR(ctx, r.penetration, 1.0f - d, 1e-5f);
    CHECK_NEAR(ctx, r.axis.x, -0.3f / d, 1e-5f);
    CHECK_NEAR(ctx, r.axis.y, -0.3f / d, 1e-5f);
    CHECK_NEAR(ctx, r.axis.z, 0.3f / d, 1e-5f);
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[r.feature] == 0b011);

    // out of reach beyond the hypotenuse: negative penetration is the separation
    r = sphere_vs_triangle(a, b, c, {2.0f, 0.0f, 2.0f}, 1.0f);
    CHECK(ctx, r.feature == triangle::EDGE_BC);
    CHECK_NEAR(ctx, r.penetration, 1.0f - scalar::sqrt(2.0f), 1e-6f);
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[r.feature] == 0b010);
}

void test_sphere_triangle_feature_to_active_edges(TestCtx &ctx) {
    // edge bits as triangle-mesh-builder lays them out: ab bit 0, bc bit 1, ca bit 2
    constexpr uint32_t AB = 0b001, BC = 0b010, CA = 0b100;
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[0] == 0);
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[triangle::VERTEX_A] == (AB | CA));
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[triangle::VERTEX_B] == (AB | BC));
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[triangle::VERTEX_C] == (BC | CA));
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[triangle::EDGE_AB] == AB);
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[triangle::EDGE_BC] == BC);
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[triangle::EDGE_AC] == CA);
    CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[triangle::FACE] == 0);
    // a vertex needs exactly the two edges whose feature bits contain it
    const uint32_t vertices[3] = {triangle::VERTEX_A, triangle::VERTEX_B, triangle::VERTEX_C};
    const uint32_t edges[3] = {triangle::EDGE_AB, triangle::EDGE_BC, triangle::EDGE_AC};
    for (uint32_t v = 0; v < 3; v++) {
        uint32_t expected = 0;
        for (uint32_t e = 0; e < 3; e++) {
            if ((edges[e] & vertices[v]) != 0) expected |= sphere_triangle::FEATURE_TO_ACTIVE_EDGES[edges[e]];
        }
        CHECK(ctx, sphere_triangle::FEATURE_TO_ACTIVE_EDGES[vertices[v]] == expected);
    }
}
