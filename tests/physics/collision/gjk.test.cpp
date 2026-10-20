#include "instance.h"
#include "math/mat4f.h"
#include "math/quatf.h"
#include "math/scalar.h"
#include "math/vec3f.h"
#include "physics/collision/gjk.h"
#include "physics/collision/support.h"
#include "physics/shapes/box.h"
#include "physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

static constexpr float FLOAT_MAX = __FLT_MAX__;
static constexpr float TOLERANCE = 1e-4f;

/** a sphere support: include mode rounds the origin by the radius, exclude mode reports it instead. */
static support::Support sphere_support(float radius, bool include_radius) {
    support::Support s = support::create();
    s.get_support = support::sphere_support;
    s.sphere.radius = include_radius ? radius : 0.0f;
    s.convex_radius = include_radius ? 0.0f : radius;
    return s;
}

static support::Support box_support(const Float3 &half_extents) {
    support::Support s = support::create();
    s.get_support = support::box_support;
    s.box.half_extents = vec3f::load(half_extents);
    return s;
}

static void place(support::Support &s, const Float3 &position) {
    s.has_transform = true;
    s.transform = mat4f::from_translation(vec3f::load(position));
}

static void place(support::Support &s, const Float4 &rotation, const Float3 &position) {
    s.has_transform = true;
    s.transform = mat4f::from_rotation_translation(quatf::load(rotation), vec3f::load(position));
}

static gjk::GjkClosestPoints closest(support::Support &a, support::Support &b) {
    gjk::GjkClosestPoints out = gjk::create_gjk_closest_points();
    gjk::closest_points(out, a, b, TOLERANCE, vec3f::make(1.0f, 0.0f, 0.0f), FLOAT_MAX);
    return out;
}

void test_gjk_spheres_apart(TestCtx &ctx) {
    const Float3 centre_b = {3.0f, 1.0f, -0.5f};
    const float radius_a = 1.0f, radius_b = 0.5f;
    const float centre_distance = vec3f::length(vec3f::load(centre_b));
    const float gap = centre_distance - radius_a - radius_b;
    const Vec3f axis = unit(vec3f::load(centre_b));
    const Vec3f surface_a = vec3f::scale(axis, radius_a);
    const Vec3f surface_b = vec3f::scale_and_add(vec3f::load(centre_b), axis, -radius_b);

    // include mode: gjk walks the round surfaces
    {
        support::Support a = sphere_support(radius_a, true);
        support::Support b = sphere_support(radius_b, true);
        place(b, centre_b);
        const gjk::GjkClosestPoints result = closest(a, b);
        CHECK_NEAR(ctx, scalar::sqrt(result.squared_distance), gap, 1e-3f);
        CHECK(ctx, near3(result.point_a, surface_a, 1e-2f));
        CHECK(ctx, near3(result.point_b, surface_b, 1e-2f));
        // the separating axis points from a to b
        CHECK(ctx, near3(unit(result.penetration_axis), axis, 1e-3f));
    }

    // exclude mode: the cores are the centres, the reported radii take the rest
    {
        support::Support a = sphere_support(radius_a, false);
        support::Support b = sphere_support(radius_b, false);
        place(b, centre_b);
        const gjk::GjkClosestPoints result = closest(a, b);
        CHECK_NEAR(ctx, scalar::sqrt(result.squared_distance) - a.convex_radius - b.convex_radius, gap, 1e-5f);
        CHECK(ctx, near3(result.point_a, 0, 0, 0, 1e-5f));
        CHECK(ctx, near3(result.point_b, vec3f::load(centre_b), 1e-5f));
        const Vec3f n = unit(result.penetration_axis);
        CHECK(ctx, near3(n, axis, 1e-5f));
        const Vec3f on_a = vec3f::scale_and_add(result.point_a, n, a.convex_radius);
        const Vec3f on_b = vec3f::scale_and_add(result.point_b, n, -b.convex_radius);
        CHECK(ctx, near3(on_a, surface_a, 1e-5f));
        CHECK(ctx, near3(on_b, surface_b, 1e-5f));
    }
}

void test_gjk_boxes_face_to_face(TestCtx &ctx) {
    support::Support a = box_support({1.0f, 1.0f, 1.0f});
    support::Support b = box_support({0.5f, 0.5f, 0.5f});
    place(b, {2.75f, 0.2f, 0.1f});
    const gjk::GjkClosestPoints result = closest(a, b);
    // b's -x face at 2.25, a's +x face at 1
    CHECK_NEAR(ctx, scalar::sqrt(result.squared_distance), 1.25f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::x(result.point_a), 1.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::x(result.point_b), 2.25f, 1e-5f);
    // the points face each other across the gap, on both faces
    CHECK_NEAR(ctx, vec3f::y(result.point_a), vec3f::y(result.point_b), 1e-5f);
    CHECK_NEAR(ctx, vec3f::z(result.point_a), vec3f::z(result.point_b), 1e-5f);
    CHECK(ctx, vec3f::y(result.point_b) >= -0.3f - 1e-5f && vec3f::y(result.point_b) <= 0.7f + 1e-5f);
    CHECK(ctx, near3(unit(result.penetration_axis), 1, 0, 0, 1e-5f));

    // swapped operands: the same gap, the axis reversed
    support::Support small = box_support({0.5f, 0.5f, 0.5f});
    place(small, {2.75f, 0.2f, 0.1f});
    support::Support big = box_support({1.0f, 1.0f, 1.0f});
    const gjk::GjkClosestPoints swapped = closest(small, big);
    CHECK_NEAR(ctx, scalar::sqrt(swapped.squared_distance), 1.25f, 1e-5f);
    CHECK(ctx, near3(unit(swapped.penetration_axis), -1, 0, 0, 1e-5f));
}

void test_gjk_overlapping(TestCtx &ctx) {
    support::Support a = sphere_support(1.0f, true);
    support::Support b = box_support({1.0f, 1.0f, 1.0f});
    place(b, {1.5f, 0.3f, 0.0f});
    gjk::GjkClosestPoints result = closest(a, b);
    CHECK(ctx, result.squared_distance == 0.0f);

    // deep overlap: one inside the other
    place(b, {0.1f, 0.0f, 0.0f});
    result = closest(a, b);
    CHECK(ctx, result.squared_distance == 0.0f);

    // farther than max_distance_squared: reported as infinitely far
    place(b, {10.0f, 0.0f, 0.0f});
    gjk::closest_points(result, a, b, TOLERANCE, vec3f::make(1.0f, 0.0f, 0.0f), 4.0f);
    CHECK(ctx, result.squared_distance == FLOAT_MAX);
}

void test_gjk_rotated_box_vs_sphere(TestCtx &ctx) {
    // a unit box turned 45 degrees about z presents its vertical edge to the sphere at x = 3 - sqrt(2)
    Float4 rotation = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI * 0.25f);
    support::Support a = sphere_support(0.5f, false);
    support::Support b = box_support({1.0f, 1.0f, 1.0f});
    place(b, rotation, {3.0f, 0.0f, 0.0f});
    const gjk::GjkClosestPoints result = closest(a, b);
    const float edge_x = 3.0f - scalar::sqrt(2.0f);
    CHECK_NEAR(ctx, scalar::sqrt(result.squared_distance) - a.convex_radius, edge_x - 0.5f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::x(result.point_b), edge_x, 1e-5f);
    CHECK_NEAR(ctx, vec3f::y(result.point_b), 0.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::z(result.point_b), 0.0f, 1e-5f);

    // turned 45 degrees about x instead, the box shows its flat +-x face to the sphere
    rotation = stored_axis_angle(vec3f::make(1.0f, 0.0f, 0.0f), scalar::PI * 0.25f);
    place(b, rotation, {3.0f, 0.0f, 0.0f});
    const gjk::GjkClosestPoints face = closest(a, b);
    CHECK_NEAR(ctx, scalar::sqrt(face.squared_distance) - a.convex_radius, 1.5f, 1e-5f);
}

void test_gjk_point_vs_box(TestCtx &ctx) {
    support::Support box = box_support({1.0f, 2.0f, 1.0f});

    support::Support point = support::create();
    support::set_point_support(point, vec3f::make(0.2f, 0.3f, -0.1f));
    gjk::GjkClosestPoints result = closest(point, box);
    CHECK(ctx, result.squared_distance == 0.0f);

    // outside the +x face
    support::set_point_support(point, vec3f::make(2.0f, 0.5f, 0.25f));
    result = closest(point, box);
    CHECK_NEAR(ctx, result.squared_distance, 1.0f, 1e-5f);
    CHECK(ctx, near3(result.point_a, 2.0f, 0.5f, 0.25f, 1e-6f));
    CHECK(ctx, near3(result.point_b, 1.0f, 0.5f, 0.25f, 1e-5f));

    // outside a corner
    support::set_point_support(point, vec3f::make(2.0f, 3.0f, -2.0f));
    result = closest(point, box);
    CHECK_NEAR(ctx, result.squared_distance, 3.0f, 1e-5f);
    CHECK(ctx, near3(result.point_b, 1.0f, 2.0f, -1.0f, 1e-5f));
}

void test_gjk_cast_ray(TestCtx &ctx) {
    gjk::GjkCastRayResult result = gjk::create_gjk_cast_ray_result();

    // sphere of radius 1 at (5, 0, 0)
    support::Support sphere = sphere_support(1.0f, true);
    place(sphere, {5.0f, 0.0f, 0.0f});

    gjk::cast_ray(result, vec3f::make(0, 0, 0), vec3f::make(10, 0, 0), TOLERANCE, sphere);
    CHECK(ctx, result.is_hit_found);
    CHECK_NEAR(ctx, result.lambda, 0.4f, 1e-4f);

    // off axis by 0.5: enters at x = 5 - sqrt(1 - 0.25)
    gjk::cast_ray(result, vec3f::make(0, 0.5f, 0), vec3f::make(10, 0, 0), TOLERANCE, sphere);
    CHECK(ctx, result.is_hit_found);
    CHECK_NEAR(ctx, result.lambda, (5.0f - scalar::sqrt(0.75f)) / 10.0f, 1e-4f);

    // pointing away
    gjk::cast_ray(result, vec3f::make(0, 0, 0), vec3f::make(-10, 0, 0), TOLERANCE, sphere);
    CHECK(ctx, !result.is_hit_found);
    // passing by
    gjk::cast_ray(result, vec3f::make(0, 1.5f, 0), vec3f::make(10, 0, 0), TOLERANCE, sphere);
    CHECK(ctx, !result.is_hit_found);
    // too short: the hit would be at 4/3 of the ray
    gjk::cast_ray(result, vec3f::make(0, 0, 0), vec3f::make(3, 0, 0), TOLERANCE, sphere);
    CHECK(ctx, !result.is_hit_found);
    // max_lambda extends the ray
    gjk::cast_ray(result, vec3f::make(0, 0, 0), vec3f::make(3, 0, 0), TOLERANCE, sphere, 2.0f);
    CHECK(ctx, result.is_hit_found);
    CHECK_NEAR(ctx, result.lambda, 4.0f / 3.0f, 1e-4f);
    // starting inside: a hit at zero
    gjk::cast_ray(result, vec3f::make(5.2f, 0, 0), vec3f::make(10, 0, 0), TOLERANCE, sphere);
    CHECK(ctx, result.is_hit_found && result.lambda == 0.0f);

    // box of half extent 1 at (5, 0, 0)
    support::Support box = box_support({1.0f, 1.0f, 1.0f});
    place(box, {5.0f, 0.0f, 0.0f});
    gjk::cast_ray(result, vec3f::make(0, 0.3f, 0.2f), vec3f::make(10, 0, 0), TOLERANCE, box);
    CHECK(ctx, result.is_hit_found);
    CHECK_NEAR(ctx, result.lambda, 0.4f, 1e-4f);
    // diagonal, onto the +y face at y = 1: from (5, 4, 0) along (0, -6, 0.5)
    gjk::cast_ray(result, vec3f::make(5, 4, 0), vec3f::make(0, -6, 0.5f), TOLERANCE, box);
    CHECK(ctx, result.is_hit_found);
    CHECK_NEAR(ctx, result.lambda, 0.5f, 1e-4f);
    gjk::cast_ray(result, vec3f::make(0, 0.3f, 0.2f), vec3f::make(0, 10, 0), TOLERANCE, box);
    CHECK(ctx, !result.is_hit_found);
}

void test_gjk_cast_shape(TestCtx &ctx) {
    // a sphere of radius 0.5 (exclude mode: a point core) sweeping along +x into a unit box at the origin,
    // the box rounded by 0.05 as the narrowphase passes it
    support::Support sphere = sphere_support(0.5f, false);
    support::Support box = box_support({0.95f, 0.95f, 0.95f});
    box.convex_radius = 0.05f;

    Mat4f a_to_b = mat4f::from_translation(vec3f::make(-5.0f, 0.2f, -0.1f));
    gjk::GjkCastShapeResult result = gjk::create_gjk_cast_shape_result();
    gjk::cast_shape(result, a_to_b, sphere, box, vec3f::make(10, 0, 0), TOLERANCE, sphere.convex_radius, box.convex_radius, 1.0f);
    CHECK(ctx, result.hit);
    // first touch with the sphere's centre at x = -1.5: 3.5 of the 10
    CHECK_NEAR(ctx, result.lambda, 0.35f, 1e-4f);
    // a's support got the transform installed
    CHECK(ctx, sphere.has_transform && v4f::lane<0>(sphere.transform.c[3]) == -5.0f);
    // contact on the box's -x face, in line with the sphere's centre
    CHECK(ctx, near3(result.point_b, -1.0f, 0.2f, -0.1f, 1e-3f));
    CHECK(ctx, near3(result.point_a, result.point_b, 0.0f));
    CHECK(ctx, near3(unit(result.separating_axis), 1, 0, 0, 1e-3f));

    // sweeping past
    gjk::cast_shape(result, a_to_b, sphere, box, vec3f::make(0, 10, 0), TOLERANCE, sphere.convex_radius, box.convex_radius, 1.0f);
    CHECK(ctx, !result.hit);
    // stopping short
    gjk::cast_shape(result, a_to_b, sphere, box, vec3f::make(3, 0, 0), TOLERANCE, sphere.convex_radius, box.convex_radius, 1.0f);
    CHECK(ctx, !result.hit);

    // a box (exclude mode) sweeping down onto a sphere: its bottom face at y = 2 - 0.5, the sphere top at 1
    support::Support falling = box_support({0.45f, 0.45f, 0.45f});
    falling.convex_radius = 0.05f;
    support::Support ball = sphere_support(1.0f, false);
    a_to_b = mat4f::from_translation(vec3f::make(0.0f, 2.0f, 0.0f));
    gjk::cast_shape(result, a_to_b, falling, ball, vec3f::make(0, -2, 0), TOLERANCE, falling.convex_radius, ball.convex_radius, 1.0f);
    CHECK(ctx, result.hit);
    CHECK_NEAR(ctx, result.lambda, 0.25f, 1e-4f);
    CHECK(ctx, near3(result.point_b, 0.0f, 1.0f, 0.0f, 1e-3f));
}

void test_gjk_convex_radius_contract(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    shapes::Shape *box_shape = box::create({.half_extents = {1.0f, 2.0f, 1.5f}, .convex_radius = 0.1f});
    shapes::Shape *sphere_shape = sphere::create({.radius = 0.5f});
    const Float3 unit_scale = {1.0f, 1.0f, 1.0f};

    // the sphere off each face of the box, the gap known: 1.5 from the face
    const Float3 centres[3] = {{3.0f, 0.5f, -0.2f}, {0.3f, -4.0f, 0.4f}, {-0.5f, 0.2f, 3.5f}};
    const float gaps[3] = {1.5f, 1.5f, 1.5f};

    for (uint32_t i = 0; i < 3; i++) {
        float distances[2];
        for (uint32_t mode = 0; mode < 2; mode++) {
            const support::SupportFunctionMode m = mode == 0 ? support::INCLUDE_CONVEX_RADIUS : support::EXCLUDE_CONVEX_RADIUS;
            support::Support a = support::create();
            support::Support b = support::create();
            box::set_support(a, box_shape, m, vec3f::load(unit_scale));
            sphere::set_support(b, sphere_shape, m, vec3f::load(unit_scale));
            place(b, centres[i]);
            const gjk::GjkClosestPoints result = closest(a, b);
            distances[mode] = scalar::sqrt(result.squared_distance) - a.convex_radius - b.convex_radius;
            if (mode == 1) CHECK(ctx, a.convex_radius == 0.05f && b.convex_radius == 0.5f);
        }
        CHECK_NEAR(ctx, distances[0], gaps[i], 1e-3f);
        CHECK_NEAR(ctx, distances[1], gaps[i], 1e-4f);
        CHECK_NEAR(ctx, distances[0], distances[1], 1e-3f);
    }

    box::destroy(box_shape);
    sphere::destroy(sphere_shape);
}
