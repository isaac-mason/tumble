#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/collision/cast_shape_vs_shape.h"
#include "tumble/physics/collision/collide_shape_vs_shape.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

using compound::CompoundShapeChild;
using shapes::Shape;

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};
static const Float4 IDENTITY = {0, 0, 0, 1};

/** `count` copies of `child` 10 apart along x: only the one at the origin is near a query at the origin */
static Shape *row(Shape *child, uint32_t count) {
    CompoundShapeChild children[8];
    for (uint32_t i = 0; i < count; i++) children[i] = {{(float)i * 10.0f, 0, 0}, IDENTITY, child};
    return compound::create({.children = children, .num_children = count});
}

// a box ray reports either a hit or a miss, so counting both counts the children the walk hands the ray to
static void count_ray_hit(cast_ray::CastRayCollector &c, const cast_ray::CastRayHit &) { (*static_cast<uint32_t *>(c.context))++; }
static void count_ray_miss(cast_ray::CastRayCollector &c) { (*static_cast<uint32_t *>(c.context))++; }

static uint32_t ray_visits(const Shape *shape, const Transformf &transform, const Float3 &origin) {
    uint32_t visits = 0;
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_CUSTOM);
    collector.custom_add_hit = count_ray_hit;
    collector.custom_add_miss = count_ray_miss;
    collector.context = &visits;
    shapes::def_of(shape).cast_ray(collector, cast_ray::default_settings(), {vec3f::load(origin), vec3f::make(0, -1, 0), 20}, shape, NO_SUB_SHAPE, transform);
    return visits;
}

// the sphere-vs-sphere row swapped for a count, so a compound of spheres counts the children its walk hands on
static uint32_t sphere_collides = 0;
static void count_sphere_collide(collide_shape::CollideShapeCollector &, const collide_shape::CollideShapeSettings &, const Shape *,
                                 const sub_shape::SubShapeIdBuilder &, const Transformf &, const Shape *, const sub_shape::SubShapeIdBuilder &,
                                 const Transformf &) {
    sphere_collides++;
}

static uint32_t collide_visits(const Shape *ball, const Shape *shape) {
    shapes::set_collide_shape_fn(shapes::SHAPE_SPHERE, shapes::SHAPE_SPHERE, count_sphere_collide);
    sphere_collides = 0;
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    shapes::collide_fn(ball->type, shape->type)(collector, collide_shape::default_settings(), ball, NO_SUB_SHAPE, pose({0, 0.8f, 0}), shape,
                                                NO_SUB_SHAPE, pose({0, 0, 0}));
    collide_shape::release(collector);
    dispatch::init();
    return sphere_collides;
}

void test_compound_culling_ray(TestCtx &ctx) {
    dispatch::init();
    Shape *cube = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.0f});
    Shape *many = row(cube, 8);

    // straight down onto the child at the origin: only it is visited
    CHECK(ctx, ray_visits(many, pose({0, 0, 0}), {0, 10, 0}) == 1);

    // posed: a quarter turn about y takes local +x to world -z, scaled by 2 the child at local x = 30 sits 60 along -z
    Float4 turn = stored_axis_angle(vec3f::make(0, 1, 0), 1.5707963f);
    CHECK(ctx, ray_visits(many, pose({5, 0, 5}, turn, {2, 2, 2}), {5, 10, 5 - 60}) == 1);

    compound::destroy(many);
    box::destroy(cube);
}

void test_compound_culling_collide(TestCtx &ctx) {
    dispatch::init();
    Shape *ball = sphere::create({.radius = 0.5f});
    Shape *many = row(ball, 8);

    // a sphere overlapping the child at the origin: only it is visited
    CHECK(ctx, collide_visits(ball, many) == 1);

    compound::destroy(many);
    sphere::destroy(ball);
}

void test_compound_culling_cast(TestCtx &ctx) {
    dispatch::init();
    Shape *ball = sphere::create({.radius = 0.5f});
    Shape *many = row(ball, 8);
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);

    // posed and scaled: the swept ball still finds the far child, and only it
    Float4 turn = stored_axis_angle(vec3f::make(0, 1, 0), 1.5707963f);
    shapes::cast_fn(ball->type, many->type)(collector, cast_shape::default_settings(), ball, NO_SUB_SHAPE, pose({5, 5, 5 - 60}), vec3f::make(0, -10, 0),
                                            many, NO_SUB_SHAPE, pose({5, 0, 5}, turn, {2, 2, 2}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].fraction, (5.0f - 0.5f - 1.0f) / 10.0f, 1e-3f);

    cast_shape::release(collector);
    compound::destroy(many);
    sphere::destroy(ball);
}
