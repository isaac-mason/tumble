#include "tumble/physics/collision/narrowphase.h"

#include "tumble/instance.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/physics/shapes/sphere.h"
#include "test.h"
#include "test_math.h"

namespace {

const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};


/** a few shapes over the dispatch tables: a unit sphere, a box of half extent 1, a capsule along y, an empty shape */
struct NarrowphaseWorld {
    shapes::Shape *sphere;
    shapes::Shape *box;
    shapes::Shape *capsule;
    shapes::Shape *empty;
};

NarrowphaseWorld make_world(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    NarrowphaseWorld world;
    world.sphere = sphere::create({.radius = 1.0f});
    world.box = box::create({.half_extents = {1.0f, 1.0f, 1.0f}});
    world.capsule = capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.5f});
    world.empty = empty::create();
    return world;
}

void release(NarrowphaseWorld &world) {
    sphere::destroy(world.sphere);
    box::destroy(world.box);
    capsule::destroy(world.capsule);
    empty::destroy(world.empty);
}

/** the closest hit's fraction of a ray along +x from `origin`, 10 long, or -1 for none */
float ray_fraction(const shapes::Shape *shape, const Float3 &origin, const Float3 &position) {
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    narrowphase::cast_ray_vs_shape(collector, cast_ray::default_settings(), {vec3f::load(origin), vec3f::make(1.0f, 0.0f, 0.0f), 10.0f}, shape, NO_SUB_SHAPE,
                                   pose(position));
    const float fraction = collector.has_hit ? collector.hits[0].fraction : -1.0f;
    cast_ray::release(collector);
    return fraction;
}

/** the closest hit's fraction of a swept along +10 x from (-5, 0, 0) against b at the origin, or -1 for none */
float cast_fraction(const shapes::Shape *a, const shapes::Shape *b) {
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    narrowphase::cast_shape_vs_shape(collector, cast_shape::default_settings(), a, NO_SUB_SHAPE, pose({-5.0f, 0.0f, 0.0f}),
                                     vec3f::make(10.0f, 0.0f, 0.0f), b, NO_SUB_SHAPE, pose({0.0f, 0.0f, 0.0f}));
    const float fraction = collector.has_hit ? collector.hits[0].fraction : -1.0f;
    cast_shape::release(collector);
    return fraction;
}

bool point_hits(const shapes::Shape *shape, const Float3 &point, const Float3 &position) {
    collide_point::CollidePointCollector collector = collide_point::create_collector(collector::COLLECT_ANY);
    narrowphase::collide_point_vs_shape(collector, collide_point::default_settings(), vec3f::load(point), shape, NO_SUB_SHAPE, pose(position));
    const bool hit = collector.has_hit;
    collide_point::release(collector);
    return hit;
}

}  // namespace

void test_narrowphase_cast_ray_goes_to_the_shapes_def(TestCtx &ctx) {
    NarrowphaseWorld world = make_world(ctx);

    // from x = -5, 10 long: the unit sphere's near side at x = -1 is 4 in
    CHECK_NEAR(ctx, ray_fraction(world.sphere, {-5.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}), 0.4f, 1e-5f);
    CHECK_NEAR(ctx, ray_fraction(world.sphere, {-5.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}), 0.6f, 1e-5f);
    CHECK_NEAR(ctx, ray_fraction(world.box, {-5.0f, 0.5f, 0.5f}, {0.0f, 0.0f, 0.0f}), 0.4f, 1e-5f);
    CHECK_NEAR(ctx, ray_fraction(world.capsule, {-5.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f}), 0.45f, 1e-5f);
    CHECK(ctx, ray_fraction(world.sphere, {-5.0f, 1.5f, 0.0f}, {0.0f, 0.0f, 0.0f}) < 0.0f);
    CHECK(ctx, ray_fraction(world.empty, {-5.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}) < 0.0f);

    release(world);
}

void test_narrowphase_cast_shape_goes_through_the_cast_table(TestCtx &ctx) {
    NarrowphaseWorld world = make_world(ctx);

    // a unit sphere swept from x = -5 meets the box's face at x = -1 when its centre is at -2: 3 of the 10
    CHECK_NEAR(ctx, cast_fraction(world.sphere, world.box), 0.3f, 1e-3f);
    // two unit spheres meet with the moving one's centre at -2 likewise
    CHECK_NEAR(ctx, cast_fraction(world.sphere, world.sphere), 0.3f, 1e-3f);
    // a capsule of radius 0.5 meets the sphere with its centre at -1.5
    CHECK_NEAR(ctx, cast_fraction(world.capsule, world.sphere), 0.35f, 1e-3f);

    // a pair with no entry in the table is no collision
    CHECK(ctx, shapes::cast_fn(world.empty->type, world.sphere->type) == nullptr);
    CHECK(ctx, cast_fraction(world.empty, world.sphere) < 0.0f);

    release(world);
}

void test_narrowphase_collide_shape_goes_through_the_collide_table(TestCtx &ctx) {
    NarrowphaseWorld world = make_world(ctx);
    const collide_shape::CollideShapeSettings settings = collide_shape::default_settings();

    // the unit sphere at x = 1.5 is half a unit into the box, pushed out along +x
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    narrowphase::collide_shape_vs_shape(collector, settings, world.box, NO_SUB_SHAPE, pose({0.0f, 0.0f, 0.0f}), world.sphere, NO_SUB_SHAPE,
                                        pose({1.5f, 0.0f, 0.0f}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-4f);
    const Vec3f axis = vec3f::normalize(collector.hits[0].penetration_axis);
    CHECK_NEAR(ctx, vec3f::x(axis), 1.0f, 1e-4f);

    // the capsule standing at x = 1.25 overlaps the box by 0.25
    collide_shape::reset(collector);
    narrowphase::collide_shape_vs_shape(collector, settings, world.box, NO_SUB_SHAPE, pose({0.0f, 0.0f, 0.0f}), world.capsule, NO_SUB_SHAPE,
                                        pose({1.25f, 0.0f, 0.0f}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.25f, 1e-3f);

    // apart, and a pair with no entry
    collide_shape::reset(collector);
    narrowphase::collide_shape_vs_shape(collector, settings, world.box, NO_SUB_SHAPE, pose({0.0f, 0.0f, 0.0f}), world.sphere, NO_SUB_SHAPE,
                                        pose({3.0f, 0.0f, 0.0f}));
    CHECK(ctx, collector.hits.length == 0);
    CHECK(ctx, shapes::collide_fn(world.empty->type, world.box->type) == nullptr);
    narrowphase::collide_shape_vs_shape(collector, settings, world.empty, NO_SUB_SHAPE, pose({0.0f, 0.0f, 0.0f}), world.box, NO_SUB_SHAPE,
                                        pose({0.0f, 0.0f, 0.0f}));
    CHECK(ctx, collector.hits.length == 0);

    collide_shape::release(collector);
    release(world);
}

void test_narrowphase_collide_with_internal_edge_removal(TestCtx &ctx) {
    NarrowphaseWorld world = make_world(ctx);

    // the caller asks for no faces and active edges only; the wrapper forces faces on, the face contact passes
    // straight through to the caller's collector with the same depth
    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    settings.collect_faces = false;
    settings.collide_only_with_active_edges = true;
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    narrowphase::collide_shape_vs_shape_with_internal_edge_removal(collector, settings, world.sphere, NO_SUB_SHAPE,
                                                                  pose({0.0f, 1.75f, 0.0f}), world.box, NO_SUB_SHAPE,
                                                                  pose({0.0f, 0.0f, 0.0f}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.25f, 1e-4f);
    CHECK(ctx, collector.hits[0].face_b.num_vertices == 4);
    for (uint32_t i = 0; i < collector.hits[0].face_b.num_vertices; i++) {
        CHECK_NEAR(ctx, vec3f::y(collector.hits[0].face_b.vertices[i]), 1.0f, 1e-5f);
    }

    // a closest collector keeps its own early-out through the wrapper
    collide_shape::CollideShapeCollector closest = collide_shape::create_collector(collector::COLLECT_CLOSEST);
    narrowphase::collide_shape_vs_shape_with_internal_edge_removal(closest, settings, world.sphere, NO_SUB_SHAPE, pose({0.0f, 1.75f, 0.0f}),
                                                                  world.box, NO_SUB_SHAPE, pose({0.0f, 0.0f, 0.0f}));
    CHECK(ctx, closest.has_hit);
    CHECK_NEAR(ctx, closest.early_out_fraction, -0.25f, 1e-4f);

    // apart: nothing held back, nothing reported
    collide_shape::reset(collector);
    narrowphase::collide_shape_vs_shape_with_internal_edge_removal(collector, settings, world.sphere, NO_SUB_SHAPE, pose({0.0f, 3.0f, 0.0f}),
                                                                  world.box, NO_SUB_SHAPE, pose({0.0f, 0.0f, 0.0f}));
    CHECK(ctx, collector.hits.length == 0);

    collide_shape::release(closest);
    collide_shape::release(collector);
    release(world);
}

void test_narrowphase_collide_point_goes_to_the_shapes_def(TestCtx &ctx) {
    NarrowphaseWorld world = make_world(ctx);

    CHECK(ctx, point_hits(world.box, {0.5f, 0.5f, 0.5f}, {0.0f, 0.0f, 0.0f}));
    CHECK(ctx, !point_hits(world.box, {1.5f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}));
    CHECK(ctx, point_hits(world.box, {4.5f, 0.0f, 0.0f}, {4.0f, 0.0f, 0.0f}));
    CHECK(ctx, point_hits(world.sphere, {0.0f, 0.9f, 0.0f}, {0.0f, 0.0f, 0.0f}));
    CHECK(ctx, !point_hits(world.sphere, {0.8f, 0.8f, 0.0f}, {0.0f, 0.0f, 0.0f}));
    CHECK(ctx, point_hits(world.capsule, {0.0f, 1.4f, 0.0f}, {0.0f, 0.0f, 0.0f}));
    CHECK(ctx, !point_hits(world.capsule, {0.6f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}));
    CHECK(ctx, !point_hits(world.empty, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}));

    release(world);
}
