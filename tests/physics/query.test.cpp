#include "tumble/physics/query.h"

#include "world_test_scene.h"
#include "test_math.h"

using rigid_body::RigidBody;

namespace {

/** a 2 m static box at the origin */
WorldScene make_box_scene(TestCtx &ctx, RigidBody **out_box = nullptr) {
    WorldScene scene = make_world_scene(ctx);
    RigidBody *box_body = add_body(scene, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    if (out_box != nullptr) *out_box = box_body;
    return scene;
}

}  // namespace


void test_query_cast_ray_hits_box(TestCtx &ctx) {
    RigidBody *box_body;
    WorldScene scene = make_box_scene(ctx, &box_body);
    filter::Filter query_filter = filter::create(scene.world->settings.layers);
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);
    query::cast_ray(*scene.world, collector, cast_ray::default_settings(), cast_ray::Rayf{vec3f::make(-5.0f, 0.0f, 0.0f), vec3f::make(1.0f, 0.0f, 0.0f), 10.0f}, query_filter);
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        // the face at x = -1, 4 m along a 10 m ray, on the box's body
        CHECK_NEAR(ctx, collector.hits[0].fraction, 0.4f, 1e-5f);
        CHECK(ctx, collector.hits[0].body_id_b == box_body->id);
    }
    cast_ray::release(collector);
    filter::release(query_filter);
    release_world_scene(scene);
}

void test_query_cast_ray_misses_box(TestCtx &ctx) {
    WorldScene scene = make_box_scene(ctx);
    filter::Filter query_filter = filter::create(scene.world->settings.layers);
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);
    query::cast_ray(*scene.world, collector, cast_ray::default_settings(), cast_ray::Rayf{vec3f::make(-5.0f, 5.0f, 0.0f), vec3f::make(1.0f, 0.0f, 0.0f), 10.0f}, query_filter);
    CHECK(ctx, collector.hits.length == 0);
    cast_ray::release(collector);
    filter::release(query_filter);
    release_world_scene(scene);
}

void test_query_cast_ray_closest_of_two_bodies(TestCtx &ctx) {
    WorldScene scene = make_box_scene(ctx);
    RigidBody *near_box = add_body(scene, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}), motion_type::MOTION_STATIC, {-3.0f, 0.0f, 0.0f});
    filter::Filter query_filter = filter::create(scene.world->settings.layers);
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    query::cast_ray(*scene.world, collector, cast_ray::default_settings(), cast_ray::Rayf{vec3f::make(-5.0f, 0.0f, 0.0f), vec3f::make(1.0f, 0.0f, 0.0f), 10.0f}, query_filter);
    CHECK(ctx, collector.has_hit);
    if (collector.has_hit) {
        CHECK(ctx, collector.hits[0].body_id_b == near_box->id);
        CHECK_NEAR(ctx, collector.hits[0].fraction, 0.15f, 1e-5f);
    }
    // a filter without the static layer sees nothing
    filter::disable_object_layer(query_filter, scene.world->settings.layers, scene.not_moving);
    cast_ray::reset(collector);
    query::cast_ray(*scene.world, collector, cast_ray::default_settings(), cast_ray::Rayf{vec3f::make(-5.0f, 0.0f, 0.0f), vec3f::make(1.0f, 0.0f, 0.0f), 10.0f}, query_filter);
    CHECK(ctx, !collector.has_hit);
    cast_ray::release(collector);
    filter::release(query_filter);
    release_world_scene(scene);
}

void test_query_cast_shape_hits_box(TestCtx &ctx) {
    WorldScene scene = make_box_scene(ctx);
    filter::Filter query_filter = filter::create(scene.world->settings.layers);
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);
    query::cast_shape(*scene.world, collector, cast_shape::default_settings(), sphere::create({.radius = 0.5f}), pose({-3.0f, 0.0f, 0.0f}),
                      vec3f::make(6.0f, 0.0f, 0.0f), query_filter);
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        // the sphere's front reaches x = -1 after 1.5 of the 6 m sweep
        CHECK_NEAR(ctx, collector.hits[0].fraction, 0.25f, 1e-3f);
    }
    cast_shape::release(collector);
    filter::release(query_filter);
    release_world_scene(scene);
}

void test_query_cast_shape_misses_box(TestCtx &ctx) {
    WorldScene scene = make_box_scene(ctx);
    filter::Filter query_filter = filter::create(scene.world->settings.layers);
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_ALL);
    query::cast_shape(*scene.world, collector, cast_shape::default_settings(), sphere::create({.radius = 0.5f}), pose({-5.0f, 5.0f, 0.0f}),
                      vec3f::make(10.0f, 0.0f, 0.0f), query_filter);
    CHECK(ctx, collector.hits.length == 0);
    cast_shape::release(collector);
    filter::release(query_filter);
    release_world_scene(scene);
}

namespace {

/**
 * two slabs side by side, the left one's top at y = 0 over x in [-2, 0], the right one's at y = 0.3 over x in [0, 2]; a
 * radius 0.5 sphere at (0, 0.4) starts 0.1 into the left and 0.4 into the right
 */
void check_closest_reports_deeper_start_overlap(TestCtx &ctx, bool right_first) {
    WorldScene scene = make_world_scene(ctx);
    Float3 slabs[2] = {{-1.0f, -0.5f, 0.0f}, {1.0f, -0.2f, 0.0f}};
    if (right_first) {
        const Float3 first = slabs[0];
        slabs[0] = slabs[1];
        slabs[1] = first;
    }
    for (uint32_t i = 0; i < 2; i++) {
        add_body(scene, box::create({.half_extents = {1.0f, 0.5f, 1.0f}, .convex_radius = 0.0f}), motion_type::MOTION_STATIC, slabs[i]);
    }
    filter::Filter query_filter = filter::create(scene.world->settings.layers);
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    settings.return_deepest_point = true;
    query::cast_shape(*scene.world, collector, settings, sphere::create({.radius = 0.5f}), pose({0.0f, 0.4f, 0.0f}), vec3f::make(0.0f, -1.0f, 0.0f),
                      query_filter);
    CHECK(ctx, collector.has_hit);
    if (collector.has_hit) {
        CHECK(ctx, collector.hits[0].fraction == 0.0f);
        CHECK_NEAR(ctx, collector.hits[0].penetration_depth, 0.4f, 1e-4f);
        CHECK_NEAR(ctx, vec3f::y(collector.hits[0].point_b), 0.3f, 1e-4f);
    }
    cast_shape::release(collector);
    filter::release(query_filter);
    release_world_scene(scene);
}

}  // namespace

void test_query_cast_shape_closest_deeper_start_overlap_left_first(TestCtx &ctx) { check_closest_reports_deeper_start_overlap(ctx, false); }

void test_query_cast_shape_closest_deeper_start_overlap_right_first(TestCtx &ctx) { check_closest_reports_deeper_start_overlap(ctx, true); }

void test_query_collide_point_inside_and_outside(TestCtx &ctx) {
    RigidBody *box_body;
    WorldScene scene = make_box_scene(ctx, &box_body);
    filter::Filter query_filter = filter::create(scene.world->settings.layers);
    collide_point::CollidePointCollector collector = collide_point::create_collector(collector::COLLECT_ALL);
    query::collide_point(*scene.world, collector, collide_point::default_settings(), vec3f::make(0.5f, 0.5f, 0.5f), query_filter);
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) CHECK(ctx, collector.hits[0].body_id_b == box_body->id);

    collide_point::reset(collector);
    query::collide_point(*scene.world, collector, collide_point::default_settings(), vec3f::make(5.0f, 5.0f, 5.0f), query_filter);
    CHECK(ctx, collector.hits.length == 0);
    collide_point::release(collector);
    filter::release(query_filter);
    release_world_scene(scene);
}

void test_query_collide_shape_overlap_and_apart(TestCtx &ctx) {
    RigidBody *box_body;
    WorldScene scene = make_box_scene(ctx, &box_body);
    filter::Filter query_filter = filter::create(scene.world->settings.layers);
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    shapes::Shape *query_shape = sphere::create({.radius = 1.0f});

    // a unit sphere centred at x = 1.5 overlaps the box's x = 1 face by 0.5
    query::collide_shape(*scene.world, collector, collide_shape::default_settings(), query_shape, pose({1.5f, 0.0f, 0.0f}), query_filter);
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) {
        CHECK_NEAR(ctx, collector.hits[0].penetration, 0.5f, 1e-4f);
        CHECK(ctx, collector.hits[0].body_id_b == box_body->id);
    }

    collide_shape::reset(collector);
    query::collide_shape(*scene.world, collector, collide_shape::default_settings(), query_shape, pose({5.0f, 5.0f, 5.0f}), query_filter);
    CHECK(ctx, collector.hits.length == 0);

    // within the max separation distance a separated shape reports a negative penetration
    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    settings.max_separation_distance = 0.5f;
    collide_shape::reset(collector);
    query::collide_shape(*scene.world, collector, settings, query_shape, pose({2.3f, 0.0f, 0.0f}), query_filter);
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length == 1) CHECK_NEAR(ctx, collector.hits[0].penetration, -0.3f, 1e-4f);
    collide_shape::release(collector);
    filter::release(query_filter);
    release_world_scene(scene);
}

namespace {

struct BodyLog {
    body_id::BodyId begun[4];
    uint32_t num_begun;
    uint32_t num_ended;
};

void log_body(collide_shape::CollideShapeCollector &collector, body_id::BodyId id) {
    BodyLog &log = *static_cast<BodyLog *>(collector.context);
    if (log.num_begun < 4) log.begun[log.num_begun] = id;
    log.num_begun++;
}

void log_body_end(collide_shape::CollideShapeCollector &collector) { static_cast<BodyLog *>(collector.context)->num_ended++; }

}  // namespace

void test_query_collide_shape_tells_collector_each_body(TestCtx &ctx) {
    RigidBody *box_body;
    WorldScene scene = make_box_scene(ctx, &box_body);
    RigidBody *other = add_body(scene, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}), motion_type::MOTION_STATIC, {3.0f, 0.0f, 0.0f});
    filter::Filter query_filter = filter::create(scene.world->settings.layers);
    BodyLog log = {};
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    collector.on_body = log_body;
    collector.on_body_end = log_body_end;
    collector.context = &log;

    // between the two boxes, touching both, through the edge removal wrapper too
    query::collide_shape_with_internal_edge_removal(*scene.world, collector, collide_shape::default_settings(), sphere::create({.radius = 1.0f}),
                                                    pose({1.5f, 0.0f, 0.0f}), query_filter);
    CHECK(ctx, log.num_begun == 2 && log.num_ended == 2);
    CHECK(ctx, (log.begun[0] == box_body->id && log.begun[1] == other->id) || (log.begun[0] == other->id && log.begun[1] == box_body->id));
    CHECK(ctx, collector.hits.length == 2);
    for (uint32_t i = 0; i < collector.hits.length; i++) CHECK_NEAR(ctx, collector.hits[i].penetration, 0.5f, 1e-4f);
    collide_shape::release(collector);
    filter::release(query_filter);
    release_world_scene(scene);
}
