#include "physics/world_test_scene.h"
#include "test.h"
#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/broadphase_query.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/sphere.h"

// a grid of unturned boxes with no convex radius, so each body's bounds are its shape: the broadphase queries are checked
// against the shape queries on the same boxes, a separate path to the same answer

namespace {

constexpr uint32_t GRID = 6;

/** a box per cell of a 6 x 3 x 6 grid, its half extents varied by cell */
WorldScene make_grid_scene(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    for (uint32_t x = 0; x < GRID; x++) {
        for (uint32_t y = 0; y < 3; y++) {
            for (uint32_t z = 0; z < GRID; z++) {
                const Float3 half = {0.3f + 0.1f * (float)((x + y) % 4), 0.4f + 0.1f * (float)(z % 3), 0.3f + 0.1f * (float)((x * z) % 3)};
                Shape *shape = box::create({.half_extents = half, .convex_radius = 0.0f});
                add_body(scene, shape, MotionType::STATIC, {2.5f * (float)x, 2.5f * (float)y, 2.5f * (float)z});
            }
        }
    }
    return scene;
}

struct Random {
    uint32_t state;
    float next(float min, float max) {
        state = state * 1664525u + 1013904223u;
        return min + (max - min) * (float)(state >> 8) / 16777216.0f;
    }
};

bool holds(const collections::Array<BodyId> &ids, BodyId id) {
    for (uint32_t i = 0; i < ids.length; i++) {
        if (ids[i] == id) return true;
    }
    return false;
}

/** the bodies the shape at the pose overlaps, by the shape queries */
void overlapped(collections::Array<BodyId> &out, World &world, const Shape *shape, const Transformf &pose,
                const Filter &filter) {
    CollideShapeCollector collector = collide_shape::create_collector(CollectorKind::ALL);
    query::collide_shape(world, collector, collide_shape::default_settings(), shape, pose, filter);
    out.length = 0;
    for (uint32_t i = 0; i < collector.hits.length; i++) {
        if (!holds(out, collector.hits[i].body_id_b)) out.push(collector.hits[i].body_id_b);
    }
}

/**
 * every body the shape grown by margin overlaps holds what the broadphase found, and the broadphase found every body the
 * shape shrunk by margin overlaps: the two only differ where a body barely touches
 */
void check_between(TestCtx &ctx, const collections::Array<BodyId> &found, const collections::Array<BodyId> &inner,
                   const collections::Array<BodyId> &outer) {
    for (uint32_t i = 0; i < inner.length; i++) CHECK(ctx, holds(found, inner[i]));
    for (uint32_t i = 0; i < found.length; i++) CHECK(ctx, holds(outer, found[i]));
}

Transformf pose_of(Vec3f position, Quatf quaternion) {
    Transformf pose;
    vec3f::store(pose.position, position);
    quatf::store(pose.quaternion, quaternion);
    pose.scale = {1.0f, 1.0f, 1.0f};
    return pose;
}

}  // namespace

void test_broadphase_query_oriented_box_meets_the_bodies_its_shape_does(TestCtx &ctx) {
    WorldScene scene = make_grid_scene(ctx);
    Filter filter = filter::create(scene.world->settings.layers);
    BodyCollector found = broadphase_query::create_body_collector(CollectorKind::ALL);
    collections::Array<BodyId> inner = {}, outer = {};
    Random random = {7};
    uint32_t hits = 0;
    for (uint32_t i = 0; i < 200; i++) {
        const Vec3f position = vec3f::make(random.next(-1.0f, 14.0f), random.next(-1.0f, 6.0f), random.next(-1.0f, 14.0f));
        const Vec3f axis = vec3f::normalize(vec3f::make(random.next(-1.0f, 1.0f), random.next(-1.0f, 1.0f), random.next(-1.0f, 1.0f)));
        const Quatf rotation = quatf::set_axis_angle(axis, random.next(0.0f, 3.1f));
        const Vec3f half = vec3f::make(random.next(0.1f, 2.0f), random.next(0.1f, 2.0f), random.next(0.1f, 2.0f));

        broadphase_query::reset(found);
        broadphase_query::collide_oriented_box(*scene.world, found, {mat4f::from_rotation_translation(rotation, position), half}, filter);

        Float3 shrunk, grown;
        vec3f::store(shrunk, vec3f::subtract(half, vec3f::splat(1e-3f)));
        vec3f::store(grown, vec3f::add(half, vec3f::splat(1e-3f)));
        Ref<Shape> inner_box = box::create({.half_extents = shrunk, .convex_radius = 0.0f});
        Ref<Shape> outer_box = box::create({.half_extents = grown, .convex_radius = 0.0f});
        overlapped(inner, *scene.world, inner_box, pose_of(position, rotation), filter);
        overlapped(outer, *scene.world, outer_box, pose_of(position, rotation), filter);
        check_between(ctx, found.hits, inner, outer);
        hits += found.hits.length;
        inner_box = nullptr;
        outer_box = nullptr;
    }
    // the queries met the grid, not only empty space
    CHECK(ctx, hits > 200);
    inner.release();
    outer.release();
    release_world_scene(scene);
}

void test_broadphase_query_sphere_meets_the_bodies_its_shape_does(TestCtx &ctx) {
    WorldScene scene = make_grid_scene(ctx);
    Filter filter = filter::create(scene.world->settings.layers);
    BodyCollector found = broadphase_query::create_body_collector(CollectorKind::ALL);
    collections::Array<BodyId> inner = {}, outer = {};
    Random random = {11};
    uint32_t hits = 0;
    for (uint32_t i = 0; i < 200; i++) {
        const Vec3f center = vec3f::make(random.next(-1.0f, 14.0f), random.next(-1.0f, 6.0f), random.next(-1.0f, 14.0f));
        const float radius = random.next(0.1f, 2.5f);

        broadphase_query::reset(found);
        broadphase_query::collide_sphere(*scene.world, found, center, radius, filter);

        Ref<Shape> inner_sphere = sphere::create({.radius = radius - 1e-3f});
        Ref<Shape> outer_sphere = sphere::create({.radius = radius + 1e-3f});
        overlapped(inner, *scene.world, inner_sphere, pose_of(center, quatf::identity()), filter);
        overlapped(outer, *scene.world, outer_sphere, pose_of(center, quatf::identity()), filter);
        check_between(ctx, found.hits, inner, outer);
        hits += found.hits.length;
        inner_sphere = nullptr;
        outer_sphere = nullptr;
    }
    CHECK(ctx, hits > 200);
    inner.release();
    outer.release();
    release_world_scene(scene);
}

void test_broadphase_query_box_and_point_meet_the_bodies_their_shapes_do(TestCtx &ctx) {
    WorldScene scene = make_grid_scene(ctx);
    Filter filter = filter::create(scene.world->settings.layers);
    BodyCollector found = broadphase_query::create_body_collector(CollectorKind::ALL);
    collections::Array<BodyId> inner = {}, outer = {};
    Random random = {13};
    for (uint32_t i = 0; i < 200; i++) {
        const Vec3f center = vec3f::make(random.next(-1.0f, 14.0f), random.next(-1.0f, 6.0f), random.next(-1.0f, 14.0f));
        const Vec3f half = vec3f::make(random.next(0.1f, 2.0f), random.next(0.1f, 2.0f), random.next(0.1f, 2.0f));
        broadphase_query::reset(found);
        broadphase_query::collide_aabb(*scene.world, found, aabox::make(vec3f::subtract(center, half), vec3f::add(center, half)), filter);
        Float3 shrunk, grown;
        vec3f::store(shrunk, vec3f::subtract(half, vec3f::splat(1e-3f)));
        vec3f::store(grown, vec3f::add(half, vec3f::splat(1e-3f)));
        Ref<Shape> inner_box = box::create({.half_extents = shrunk, .convex_radius = 0.0f});
        Ref<Shape> outer_box = box::create({.half_extents = grown, .convex_radius = 0.0f});
        overlapped(inner, *scene.world, inner_box, pose_of(center, quatf::identity()), filter);
        overlapped(outer, *scene.world, outer_box, pose_of(center, quatf::identity()), filter);
        check_between(ctx, found.hits, inner, outer);
        inner_box = nullptr;
        outer_box = nullptr;

        // a point inside a body is in that body alone, as the grid's boxes are apart
        broadphase_query::reset(found);
        broadphase_query::collide_point(*scene.world, found, center, filter);
        CollidePointCollector inside = collide_point::create_collector(CollectorKind::ALL);
        query::collide_point(*scene.world, inside, collide_point::default_settings(), center, filter);
        CHECK(ctx, found.hits.length == inside.hits.length);
        if (inside.hits.length == 1) CHECK(ctx, holds(found.hits, inside.hits[0].body_id_b));
    }
    inner.release();
    outer.release();
    release_world_scene(scene);
}

void test_broadphase_query_casts_find_the_nearest_body_where_the_shape_casts_do(TestCtx &ctx) {
    WorldScene scene = make_grid_scene(ctx);
    Filter filter = filter::create(scene.world->settings.layers);
    BodyCastCollector nearest = broadphase_query::create_body_cast_collector(CollectorKind::CLOSEST);
    BodyCastCollector all = broadphase_query::create_body_cast_collector(CollectorKind::ALL);
    Random random = {17};
    uint32_t hits = 0;
    for (uint32_t i = 0; i < 200; i++) {
        const Vec3f origin = vec3f::make(random.next(-3.0f, 16.0f), random.next(-3.0f, 8.0f), random.next(-3.0f, 16.0f));
        const Vec3f direction = vec3f::normalize(vec3f::make(random.next(-1.0f, 1.0f), random.next(-1.0f, 1.0f), random.next(-1.0f, 1.0f)));
        const Rayf ray = {origin, direction, random.next(1.0f, 15.0f)};

        broadphase_query::reset(nearest);
        broadphase_query::cast_ray(*scene.world, nearest, ray, filter);
        CastRayCollector shape_ray = cast_ray::create_collector(CollectorKind::CLOSEST);
        query::cast_ray(*scene.world, shape_ray, cast_ray::default_settings(), ray, filter);
        const CastRayHit *shape_hit = cast_ray::hit(shape_ray);
        const BodyCastHit *body_hit = broadphase_query::hit(nearest);
        // a ray starting inside a box: the shape query's back faces are off, the bounds are entered at 0
        CHECK(ctx, (shape_hit != nullptr) == (body_hit != nullptr) || (body_hit != nullptr && body_hit->fraction == 0.0f));
        if (shape_hit != nullptr && body_hit != nullptr) {
            CHECK_NEAR(ctx, body_hit->fraction, shape_hit->fraction, 1e-4f);
            hits++;
        }

        // every hit, sorted, starts with the nearest
        broadphase_query::reset(all);
        broadphase_query::cast_ray(*scene.world, all, ray, filter);
        broadphase_query::sort(all);
        CHECK(ctx, (all.hits.length == 0) == (body_hit == nullptr));
        if (body_hit != nullptr) CHECK(ctx, all.hits[0].fraction == body_hit->fraction);

        // a box swept as far: the shape cast of the same box agrees
        const Vec3f half = vec3f::make(random.next(0.1f, 1.0f), random.next(0.1f, 1.0f), random.next(0.1f, 1.0f));
        const Vec3f displacement = vec3f::scale(direction, ray.length);
        broadphase_query::reset(nearest);
        broadphase_query::cast_aabb(*scene.world, nearest, aabox::make(vec3f::subtract(origin, half), vec3f::add(origin, half)), displacement, filter);
        Float3 half_extents;
        vec3f::store(half_extents, half);
        Ref<Shape> swept = box::create({.half_extents = half_extents, .convex_radius = 0.0f});
        // back faces on: a cast's first touch can give a contact axis against the motion, which jolt and tumble both drop
        // as a back face, and a start inside a box is a hit at 0 only with them
        CastShapeSettings cast_settings = cast_shape::default_settings();
        cast_settings.collide_with_backfaces_convex = true;
        CastShapeCollector shape_cast = cast_shape::create_collector(CollectorKind::CLOSEST);
        query::cast_shape(*scene.world, shape_cast, cast_settings, swept, pose_of(origin, quatf::identity()), displacement, filter);
        const CastShapeHit *cast_hit = cast_shape::hit(shape_cast);
        const BodyCastHit *box_hit = broadphase_query::hit(nearest);
        CHECK(ctx, (cast_hit != nullptr) == (box_hit != nullptr));
        if (cast_hit != nullptr && box_hit != nullptr) CHECK_NEAR(ctx, box_hit->fraction, cast_hit->fraction, 1e-3f);
        swept = nullptr;
    }
    CHECK(ctx, hits > 20);
    release_world_scene(scene);
}

void test_broadphase_query_collectors_stop_and_filter(TestCtx &ctx) {
    WorldScene scene = make_grid_scene(ctx);
    Filter filter = filter::create(scene.world->settings.layers);
    const AABox everything = aabox::make(vec3f::splat(-5.0f), vec3f::splat(20.0f));

    BodyCollector all = broadphase_query::create_body_collector(CollectorKind::ALL);
    broadphase_query::collide_aabb(*scene.world, all, everything, filter);
    CHECK(ctx, all.hits.length == GRID * 3 * GRID);

    // any: one body, then the walk stops
    BodyCollector any = broadphase_query::create_body_collector(CollectorKind::ANY);
    broadphase_query::collide_aabb(*scene.world, any, everything, filter);
    CHECK(ctx, any.has_hit && any.hits.length == 1);

    // custom: its own bookkeeping, stopping after three
    struct Count {
        uint32_t seen;
    } count = {0};
    BodyCollector custom = broadphase_query::create_body_collector(CollectorKind::CUSTOM);
    custom.context = &count;
    custom.custom_add_hit = [](BodyCollector &c, BodyId) {
        if (++static_cast<Count *>(c.context)->seen == 3) broadphase_query::force_early_out(c);
    };
    broadphase_query::collide_aabb(*scene.world, custom, everything, filter);
    CHECK(ctx, count.seen == 3);

    // the filter's layers hold: the grid is all on the static layer
    filter::disable_object_layer(filter, scene.world->settings.layers, scene.not_moving);
    broadphase_query::reset(all);
    broadphase_query::collide_aabb(*scene.world, all, everything, filter);
    broadphase_query::collide_sphere(*scene.world, all, vec3f::splat(5.0f), 10.0f, filter);
    broadphase_query::collide_point(*scene.world, all, vec3f::zero(), filter);
    broadphase_query::collide_oriented_box(*scene.world, all, {mat4f::from_rotation_translation(quatf::identity(), vec3f::splat(5.0f)), vec3f::splat(10.0f)},
                                           filter);
    CHECK(ctx, all.hits.length == 0);

    release_world_scene(scene);
}
