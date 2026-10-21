#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/physics/collision/collide_point_vs_shape.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

using collide_point::CollidePointCollector;

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};

static Float4 about_y(float degrees) {
    Float4 q;
    return q = stored_axis_angle(vec3f::make(0, 1, 0), degrees * scalar::DEGREES_TO_RADIANS);
}

/** a point against a shape through its def, as the narrowphase dispatches it */
static void collide(CollidePointCollector &collector, const Float3 &point, const shapes::Shape *shape, const Transformf &transform) {
    shapes::def_of(shape).collide_point(collector, collide_point::default_settings(), vec3f::load(point), shape, NO_SUB_SHAPE, transform);
}

/** whether an any collector reports the point inside */
static bool inside(const Float3 &point, const shapes::Shape *shape, const Transformf &transform) {
    CollidePointCollector collector = collide_point::create_collector(collector::COLLECT_ANY);
    collide(collector, point, shape, transform);
    const bool hit = collector.has_hit;
    collide_point::release(collector);
    return hit;
}

void test_collide_point_vs_shape_sphere(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *unit = sphere::create({.radius = 1.0f});
    shapes::Shape *two = sphere::create({.radius = 2.0f});

    CollidePointCollector any = collide_point::create_collector(collector::COLLECT_ANY);
    any.body_id_b = 42;
    collide(any, {0, 0, 0}, unit, pose({0, 0, 0}));
    CHECK(ctx, any.has_hit && collide_point::hit(any)->body_id_b == 42);
    CHECK(ctx, collide_point::hit(any)->sub_shape_id_b == sub_shape::EMPTY_SUB_SHAPE_ID);
    collide_point::release(any);

    // offset from the origin, on the surface, outside
    CHECK(ctx, inside({5, 3, -2}, two, pose({5, 3, -2})));
    CHECK(ctx, inside({6.9f, 3, -2}, two, pose({5, 3, -2})));
    CHECK(ctx, inside({1, 0, 0}, unit, pose({0, 0, 0})));
    CHECK(ctx, !inside({2, 0, 0}, unit, pose({0, 0, 0})));
    CHECK(ctx, !inside({1.001f, 0, 0}, unit, pose({0, 0, 0})));
    // scaled by 2 the radius is 2
    CHECK(ctx, inside({1.5f, 0, 0}, unit, pose({0, 0, 0}, {0, 0, 0, 1}, {2, 2, 2})));
    CHECK(ctx, !inside({1.5f, 0, 0}, unit, pose({0, 0, 0})));

    // all: one hit with the body id
    CollidePointCollector all = collide_point::create_collector(collector::COLLECT_ALL);
    all.body_id_b = 99;
    collide(all, {0, 0, 0}, unit, pose({0, 0, 0}));
    CHECK(ctx, all.hits.length == 1 && all.hits[0].body_id_b == 99 && all.hits[0].sub_shape_id_b == sub_shape::EMPTY_SUB_SHAPE_ID);
    collide_point::release(all);

    sphere::destroy(unit);
    sphere::destroy(two);
}

void test_collide_point_vs_shape_box(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *unit = box::create({.half_extents = {1, 1, 1}, .material_id = 7});
    shapes::Shape *two = box::create({.half_extents = {2, 2, 2}});

    CollidePointCollector any = collide_point::create_collector(collector::COLLECT_ANY);
    any.body_id_b = 1;
    collide(any, {0, 0, 0}, unit, pose({0, 0, 0}));
    CHECK(ctx, any.has_hit && collide_point::hit(any)->body_id_b == 1);
    // its id names the box, so the box's material
    CHECK(ctx, shapes::get_shape_material(unit, collide_point::hit(any)->sub_shape_id_b) == 7);
    collide_point::release(any);

    CHECK(ctx, inside({1, 1, 1}, two, pose({0, 0, 0})));
    // on a face and on a corner count; outside does not
    CHECK(ctx, inside({1, 0, 0}, unit, pose({0, 0, 0})));
    CHECK(ctx, inside({1, 1, 1}, unit, pose({0, 0, 0})));
    CHECK(ctx, !inside({2, 0, 0}, unit, pose({0, 0, 0})));
    // turned 45 degrees about y the corner reaches sqrt 2 along x
    CHECK(ctx, inside({1.4f, 0, 0}, unit, pose({0, 0, 0}, about_y(45))));
    CHECK(ctx, !inside({1.42f, 0, 0}, unit, pose({0, 0, 0}, about_y(45))));
    CHECK(ctx, !inside({1.4f, 0, 0}, unit, pose({0, 0, 0})));
    CHECK(ctx, inside({0.5f, 0, 0}, unit, pose({0, 0, 0}, about_y(90))));
    // uniform and non-uniform scale
    CHECK(ctx, inside({1.5f, 0, 0}, unit, pose({0, 0, 0}, {0, 0, 0, 1}, {2, 2, 2})));
    CHECK(ctx, inside({0, 2, 0}, unit, pose({0, 0, 0}, {0, 0, 0, 1}, {1, 3, 1})));
    CHECK(ctx, !inside({2, 0, 0}, unit, pose({0, 0, 0}, {0, 0, 0, 1}, {1, 3, 1})));
    // translated, and translated and rotated
    CHECK(ctx, inside({5, 3, -2}, unit, pose({5, 3, -2})));
    CHECK(ctx, inside({11, 5, 3}, unit, pose({10, 5, 3}, about_y(45))));
    CHECK(ctx, !inside({11.5f, 5, 3}, unit, pose({10, 5, 3}, about_y(45))));
    // negative scale works on the absolute value
    CHECK(ctx, inside({0.5f, 0, 0}, unit, pose({0, 0, 0}, {0, 0, 0, 1}, {-1, -1, -1})));

    CollidePointCollector all = collide_point::create_collector(collector::COLLECT_ALL);
    collide(all, {0, 0, 0}, unit, pose({0, 0, 0}));
    CHECK(ctx, all.hits.length == 1);
    collide_point::release(all);

    box::destroy(unit);
    box::destroy(two);
}

void test_collide_point_vs_shape_any_collector_early_out(TestCtx &ctx) {
    dispatch::init();
    shapes::Shape *unit = sphere::create({.radius = 1.0f});
    // the first hit forces the deepest possible early out, as for collide shape
    CollidePointCollector any = collide_point::create_collector(collector::COLLECT_ANY);
    CHECK(ctx, !collide_point::should_early_out(any));
    collide(any, {0, 0, 0}, unit, pose({0, 0, 0}));
    CHECK(ctx, any.has_hit);
    CHECK(ctx, any.early_out_fraction == -collector::UNBOUNDED);
    CHECK(ctx, collide_point::should_early_out(any));
    collide_point::release(any);
    sphere::destroy(unit);
}
