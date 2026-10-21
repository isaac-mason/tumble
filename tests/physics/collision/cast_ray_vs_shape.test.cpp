#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

using cast_ray::CastRayCollector;
using cast_ray::CastRaySettings;

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};

/** a ray against a shape through its def, as the narrowphase dispatches it */
static void ray(CastRayCollector &collector, const CastRaySettings &settings, const Float3 &origin, const Float3 &direction, float length,
                const shapes::Shape *shape, const Transformf &transform) {
    shapes::def_of(shape).cast_ray(collector, settings, {vec3f::load(origin), unit(vec3f::load(direction)), length}, shape, NO_SUB_SHAPE, transform);
}

static CastRaySettings settings_with(bool triangles, bool convex, bool solid) {
    CastRaySettings settings = cast_ray::default_settings();
    settings.collide_with_backfaces_triangles = triangles;
    settings.collide_with_backfaces_convex = convex;
    settings.treat_convex_as_solid = solid;
    return settings;
}

void test_cast_ray_vs_shape_sphere(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *shape = sphere::create({.radius = 1.0f});
    const CastRaySettings settings = cast_ray::default_settings();
    CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);
    collector.body_id_b = 7;

    // from outside: the near surface at x = 4 of a 1000 long ray
    ray(collector, settings, {0, 0, 0}, {1, 0, 0}, 1000, shape, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].status == cast_ray::COLLIDING);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.004f, 1e-6f);
    CHECK(ctx, collector.hits[0].sub_shape_id == sub_shape::EMPTY_SUB_SHAPE_ID);

    // pointing away from it
    cast_ray::reset(collector);
    ray(collector, settings, {0, 5, 0}, {0, 1, 0}, 1000, shape, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // off axis rays enter at 5 - sqrt(1 - y^2)
    const float offsets[3] = {-0.5f, -0.1f, 0.0f};
    for (float y : offsets) {
        cast_ray::reset(collector);
        ray(collector, settings, {0, y, 0}, {1, 0, 0}, 1000, shape, pose({5, 0, 0}));
        CHECK(ctx, collector.hits.length == 1);
        CHECK_NEAR(ctx, collector.hits[0].fraction, (5.0f - scalar::sqrt(1.0f - y * y)) / 1000.0f, 1e-6f);
    }

    // the case from the interactive example: centre 0.0447 off the ray
    const float off = 0.044676984915871f;
    cast_ray::reset(collector);
    ray(collector, settings, {-3, 0, 0}, {1, 0, 0}, 10, shape, pose({2, off, 0}));
    CHECK(ctx, collector.hits.length == 1);
    // the sphere's ray is gjk's, converged to 1e-3 along the ray
    CHECK_NEAR(ctx, collector.hits[0].fraction, (5.0f - scalar::sqrt(1.0f - off * off)) / 10.0f, 1e-4f);

    // scaled by 2: the surface at x = 3
    cast_ray::reset(collector);
    ray(collector, settings, {0, 0, 0}, {1, 0, 0}, 1000, shape, pose({5, 0, 0}, {0, 0, 0, 1}, {2, 2, 2}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.003f, 1e-6f);

    cast_ray::release(collector);
    sphere::destroy(shape);
}

void test_cast_ray_vs_shape_box(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *shape = box::create({.half_extents = {1, 1, 1}, .material_id = 4});
    const CastRaySettings settings = cast_ray::default_settings();
    CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);

    // from the front: the face at x = 4
    ray(collector, settings, {0, 0, 0}, {1, 0, 0}, 1000, shape, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.004f, 1e-6f);
    CHECK(ctx, shapes::get_shape_material(shape, collector.hits[0].sub_shape_id) == 4);

    // pointing away
    cast_ray::reset(collector);
    ray(collector, settings, {0, 0, 0}, {-1, 0, 0}, 1000, shape, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // straight down at x = 0, beside the box spanning x in [4, 6]
    cast_ray::reset(collector);
    ray(collector, settings, {0, 5, 0}, {0, -1, 0}, 1000, shape, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);
    // down over it: the top face at y = 1
    cast_ray::reset(collector);
    ray(collector, settings, {5, 5, 0}, {0, -1, 0}, 1000, shape, pose({5, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.004f, 1e-6f);

    // turned 45 degrees about z: the ray meets the edge at x = 5 - sqrt 2
    Float4 rotation = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI / 4.0f);
    cast_ray::reset(collector);
    ray(collector, settings, {0, 0, 0}, {1, 0, 0}, 1000, shape, pose({5, 0, 0}, rotation));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, (5.0f - scalar::sqrt(2.0f)) / 1000.0f, 1e-6f);

    // scaled down to half extents 0.1, a ray 0.2 off the centre misses
    cast_ray::reset(collector);
    ray(collector, settings, {0, 0.2f, 0}, {1, 0, 0}, 1000, shape, pose({5, 0, 0}, {0, 0, 0, 1}, {0.1f, 0.1f, 0.1f}));
    CHECK(ctx, collector.hits.length == 0);
    // scaled up by 2, one through the centre meets the face at x = 3
    cast_ray::reset(collector);
    ray(collector, settings, {0, 0, 0}, {1, 0, 0}, 1000, shape, pose({5, 0, 0}, {0, 0, 0, 1}, {2, 2, 2}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.003f, 1e-6f);

    cast_ray::release(collector);
    box::destroy(shape);
}

void test_cast_ray_vs_shape_treat_convex_as_solid(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *shapes_under_test[3] = {
        sphere::create({.radius = 1.0f}),
        box::create({.half_extents = {1, 1, 1}}),
        capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f}),
    };
    const CastRaySettings solid = settings_with(false, false, true);
    const CastRaySettings hollow = settings_with(false, false, false);

    for (shapes::Shape *shape : shapes_under_test) {
        CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ANY);
        collector.body_id_b = 1;

        // starting inside: solid hits at 0, hollow has no surface to hit on the way out
        ray(collector, solid, {0, 0, 0}, {1, 0, 0}, 10, shape, pose({0, 0, 0}));
        CHECK(ctx, collector.has_hit && cast_ray::hit(collector)->fraction == 0.0f);
        CHECK(ctx, cast_ray::hit(collector)->body_id_b == 1);
        CHECK(ctx, cast_ray::should_early_out(collector));

        cast_ray::reset(collector);
        ray(collector, hollow, {0, 0, 0}, {1, 0, 0}, 10, shape, pose({0, 0, 0}));
        CHECK(ctx, !collector.has_hit);

        // starting outside: both hit the surface at x = -1, 4 of the 10
        cast_ray::reset(collector);
        ray(collector, solid, {-5, 0, 0}, {1, 0, 0}, 10, shape, pose({0, 0, 0}));
        CHECK(ctx, collector.has_hit);
        CHECK_NEAR(ctx, cast_ray::hit(collector)->fraction, 0.4f, 1e-5f);

        cast_ray::reset(collector);
        ray(collector, hollow, {-5, 0, 0}, {1, 0, 0}, 10, shape, pose({0, 0, 0}));
        CHECK(ctx, collector.has_hit);
        CHECK_NEAR(ctx, cast_ray::hit(collector)->fraction, 0.4f, 1e-5f);

        cast_ray::release(collector);
    }

    sphere::destroy(shapes_under_test[0]);
    box::destroy(shapes_under_test[1]);
    capsule::destroy(shapes_under_test[2]);
}

void test_cast_ray_vs_shape_back_face_modes(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *box_shape = box::create({.half_extents = {1, 2, 3}});
    shapes::Shape *sphere_shape = sphere::create({.radius = 1.0f});
    CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);

    // box with half extents (1, 2, 3) scaled by 2 along x, so its faces are at x = +-2. a ray from x = 0.5 along +x
    // leaves it at fraction (2 - 0.5) / 10
    const auto from_inside_box = [&](const CastRaySettings &settings) {
        cast_ray::reset(collector);
        ray(collector, settings, {0.5f, 0.25f, -0.5f}, {1, 0, 0}, 10, box_shape, pose({0, 0, 0}, {0, 0, 0, 1}, {2, 1, 1}));
        return collector.hits.length;
    };
    // ray from inside a unit sphere along +x: it exits at x = 1, so at fraction (1 - 0.25) / 10
    const auto from_inside_sphere = [&](const CastRaySettings &settings) {
        cast_ray::reset(collector);
        ray(collector, settings, {0.25f, 0, 0}, {1, 0, 0}, 10, sphere_shape, pose({0, 0, 0}));
        return collector.hits.length;
    };

    // a hollow box with convex back faces hits the far face at the analytic fraction
    CHECK(ctx, from_inside_box(settings_with(false, true, false)) == 1);
    CHECK(ctx, collector.hits[0].status == cast_ray::COLLIDING);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.15f, 1e-6f);
    // a hollow box without convex back faces misses, triangle back faces or not
    CHECK(ctx, from_inside_box(settings_with(false, false, false)) == 0);
    CHECK(ctx, from_inside_box(settings_with(true, false, false)) == 0);
    // a solid box with convex back faces reports both the start and the exit
    CHECK(ctx, from_inside_box(settings_with(false, true, true)) == 2);
    CHECK(ctx, collector.hits[0].fraction == 0.0f);
    CHECK_NEAR(ctx, collector.hits[1].fraction, 0.15f, 1e-6f);

    // a hollow convex with convex back faces hits the far surface on the way out
    CHECK(ctx, from_inside_sphere(settings_with(false, true, false)) == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.075f, 1e-4f);
    // a hollow convex without back faces misses, and triangle back faces alone do not make it report its exit
    CHECK(ctx, from_inside_sphere(settings_with(false, false, false)) == 0);
    CHECK(ctx, from_inside_sphere(settings_with(true, false, false)) == 0);
    // a solid convex hits at fraction 0
    CHECK(ctx, from_inside_sphere(cast_ray::default_settings()) == 1);
    CHECK(ctx, collector.hits[0].fraction == 0.0f);

    cast_ray::release(collector);
    box::destroy(box_shape);
    sphere::destroy(sphere_shape);
}

void test_cast_ray_vs_shape_initial_early_out_fraction(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *shape = box::create({.half_extents = {1, 1, 1}});
    CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);

    // a unit-length ray along +x against a box whose near face sits at x = face_x, so the hit fraction is face_x
    const auto at_face_x = [&](float face_x) {
        cast_ray::reset(collector);
        ray(collector, cast_ray::default_settings(), {0, 0, 0}, {1, 0, 0}, 1, shape, pose({face_x + 1.0f, 0, 0}));
        return collector.hits.length;
    };

    // exactly at the ray's end it is kept
    CHECK(ctx, at_face_x(1.0f) == 1);
    CHECK(ctx, collector.hits[0].fraction == 1.0f);
    // 1 + 2^-22 is the first fraction past the bound 1 + FLT_EPSILON (2^-23) that the box at 2 + 2^-22 can produce
    CHECK(ctx, at_face_x(1.0f + 2.0f * 1.1920929e-7f) == 0);
    CHECK(ctx, at_face_x(1.0f + 5e-5f) == 0);

    cast_ray::release(collector);
    box::destroy(shape);
}

void test_cast_ray_vs_shape_collectors(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *near_box = box::create({.half_extents = {1, 1, 1}, .material_id = 1});
    shapes::Shape *far_sphere = sphere::create({.radius = 1.0f, .material_id = 2});
    const CastRaySettings settings = cast_ray::default_settings();

    // closest: the box at x = 3 (face at 2) wins over the sphere at x = 8 (surface at 7), whichever goes first
    CastRayCollector closest = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    ray(closest, settings, {0, 0, 0}, {1, 0, 0}, 10, far_sphere, pose({8, 0, 0}));
    CHECK_NEAR(ctx, closest.early_out_fraction, 0.7f, 1e-5f);
    ray(closest, settings, {0, 0, 0}, {1, 0, 0}, 10, near_box, pose({3, 0, 0}));
    CHECK(ctx, closest.hits.length == 1);
    CHECK_NEAR(ctx, cast_ray::hit(closest)->fraction, 0.2f, 1e-5f);
    CHECK_NEAR(ctx, closest.early_out_fraction, 0.2f, 1e-5f);
    // the sphere again, now behind the bound: not taken
    ray(closest, settings, {0, 0, 0}, {1, 0, 0}, 10, far_sphere, pose({8, 0, 0}));
    CHECK_NEAR(ctx, cast_ray::hit(closest)->fraction, 0.2f, 1e-5f);
    CHECK(ctx, !cast_ray::should_early_out(closest));

    // any: the first hit stops the query
    CastRayCollector any = cast_ray::create_collector(collector::COLLECT_ANY);
    CHECK(ctx, !cast_ray::should_early_out(any));
    ray(any, settings, {0, 0, 0}, {1, 0, 0}, 10, far_sphere, pose({8, 0, 0}));
    CHECK(ctx, cast_ray::should_early_out(any) && any.early_out_fraction == collector::SHOULD_EARLY_OUT_FRACTION);
    CHECK_NEAR(ctx, cast_ray::hit(any)->fraction, 0.7f, 1e-5f);

    cast_ray::release(closest);
    cast_ray::release(any);
    box::destroy(near_box);
    sphere::destroy(far_sphere);
}

void test_cast_ray_vs_shape_sort(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *shape = sphere::create({.radius = 0.5f});
    const CastRaySettings settings = cast_ray::default_settings();
    CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);

    // spheres far, near, middle so the hits arrive out of order
    const float centers[3] = {9.0f, 3.0f, 6.0f};
    for (float x : centers) ray(collector, settings, {0, 0, 0}, {1, 0, 0}, 100, shape, pose({x, 0, 0}));
    CHECK(ctx, collector.hits.length == 3);

    cast_ray::sort(collector);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 2.5f / 100.0f, 1e-6f);
    CHECK_NEAR(ctx, collector.hits[1].fraction, 5.5f / 100.0f, 1e-6f);
    CHECK_NEAR(ctx, collector.hits[2].fraction, 8.5f / 100.0f, 1e-6f);

    cast_ray::release(collector);
    sphere::destroy(shape);
}
