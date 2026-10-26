#include <stdio.h>

#include "tumble/tumble.h"

using namespace tumble;

static bool near(float a, float b, float tolerance) { return a - b <= tolerance && b - a <= tolerance; }

/* SNIPPET_START: body-filter */
/** keeps every body but one: the caller's own, say */
bool not_this_body(const RigidBody *body, void *context) { return body != static_cast<const RigidBody *>(context); }
/* SNIPPET_END: body-filter */

int main() {
    dispatch::init();
    WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    World *world = world::create(settings);
    Ref<Shape> ground_shape = box::create({.half_extents = {20.0f, 0.5f, 20.0f}});
    Ref<Shape> crate_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    RigidBody *ground = rigid_body::create(*world, {.shape = ground_shape, .object_layer = not_moving,
                                                                 .motion_type = MotionType::STATIC, .position = {0.0f, -0.5f, 0.0f}});
    RigidBody *crate = rigid_body::create(*world, {.shape = crate_shape, .object_layer = moving,
                                                                .motion_type = MotionType::DYNAMIC, .position = {0.0f, 0.5f, 0.0f}});
    world::step(*world, 1.0f / 60.0f);

    {
        /* SNIPPET_START: cast-ray */
        // down from above the crate, 10 m: the closest hit, its point and the surface normal there
        Filter filter = filter::create(world->settings.layers);
        CastRayCollector collector = cast_ray::create_collector(CollectorKind::CLOSEST);
        const Rayf ray = {vec3f::make(0.0f, 5.0f, 0.0f), vec3f::make(0.0f, -1.0f, 0.0f), 10.0f};
        query::cast_ray(*world, collector, cast_ray::default_settings(), ray, filter);
        if (const CastRayHit *hit = cast_ray::hit(collector)) {
            const Vec3f point = vec3f::scale_and_add(ray.origin, ray.direction, ray.length * hit->fraction);
            const RigidBody *body = rigid_body::get(*world, hit->body_id_b);
            const Vec3f normal = rigid_body::get_world_space_surface_normal(body, hit->sub_shape_id, point);
            printf("hit at y %.2f, normal y %.2f\n", vec3f::y(point), vec3f::y(normal));
            /* HIDE_START */
            if (body != crate || !near(vec3f::y(point), 1.0f, 1e-4f) || !near(vec3f::y(normal), 1.0f, 1e-4f)) return 1;
            /* HIDE_END */
        }
        /* HIDE_START */
        if (!collector.has_hit) return 1;
        /* HIDE_END */

        // every hit along it, nearest first
        CastRayCollector all = cast_ray::create_collector(CollectorKind::ALL);
        query::cast_ray(*world, all, cast_ray::default_settings(), ray, filter);
        cast_ray::sort(all);
        printf("%u hits\n", all.hits.length);
        /* HIDE_START */
        if (all.hits.length != 2 || all.hits[0].body_id_b != crate->id || all.hits[1].body_id_b != ground->id) return 1;
        /* HIDE_END */

        // the collectors and the filter own memory
        /* SNIPPET_END: cast-ray */
    }

    {
        Filter filter = filter::create(world->settings.layers);
        /* SNIPPET_START: collide-point */
        // which bodies hold this point
        CollidePointCollector inside = collide_point::create_collector(CollectorKind::ALL);
        query::collide_point(*world, inside, collide_point::default_settings(), vec3f::make(0.2f, 0.5f, 0.1f), filter);
        /* SNIPPET_END: collide-point */
        /* HIDE_START */
        if (inside.hits.length != 1 || inside.hits[0].body_id_b != crate->id) return 1;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: collide-shape */
        // everything a 2 m sphere at the crate's corner overlaps, and how deep: an explosion's reach
        Ref<Shape> blast = sphere::create({.radius = 2.0f});
        Filter filter = filter::create(world->settings.layers);
        CollideShapeCollector overlaps = collide_shape::create_collector(CollectorKind::ALL);
        const Transformf at = {{1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
        query::collide_shape(*world, overlaps, collide_shape::default_settings(), blast, at, filter);
        for (uint32_t i = 0; i < overlaps.hits.length; i++) {
            printf("overlaps body %u by %.2f m\n", body_id::index(overlaps.hits[i].body_id_b), overlaps.hits[i].penetration);
        }
        /* SNIPPET_END: collide-shape */
        /* HIDE_START */
        if (overlaps.hits.length != 2) return 1;
        /* HIDE_END */

        /* SNIPPET_START: cast-shape */
        // the sphere swept along a path: where it first touches, for a projectile with a size
        Filter only_moving = filter::create(world->settings.layers);
        filter::disable_object_layer(only_moving, world->settings.layers, not_moving);
        CastShapeCollector sweep = cast_shape::create_collector(CollectorKind::CLOSEST);
        const Shape *small_ball = blast;
        const Transformf start = {{-10.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.25f, 0.25f, 0.25f}};
        query::cast_shape(*world, sweep, cast_shape::default_settings(), small_ball, start, vec3f::make(20.0f, 0.0f, 0.0f), only_moving);
        if (const CastShapeHit *hit = cast_shape::hit(sweep)) printf("stops after %.2f m\n", 20.0f * hit->fraction);
        /* SNIPPET_END: cast-shape */
        /* HIDE_START */
        // the quarter scale ball (radius 0.5) meets the crate's face at x = -0.5 after 9 m
        if (!sweep.has_hit || !near(20.0f * sweep.hits[0].fraction, 9.0f, 1e-3f)) return 1;
        blast = nullptr;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: broadphase */
        // the bodies whose bounds a 1 m box around the origin overlaps, their shapes not tested
        Filter filter = filter::create(world->settings.layers);
        BodyCollector near_origin = broadphase_query::create_body_collector(CollectorKind::ALL);
        broadphase_query::collide_aabb(*world, near_origin, aabox::make(vec3f::splat(-0.5f), vec3f::splat(0.5f)), filter);

        // a sphere and a turned box, likewise
        broadphase_query::reset(near_origin);
        broadphase_query::collide_sphere(*world, near_origin, vec3f::make(0.0f, 3.0f, 0.0f), 2.1f, filter);
        const Quatf turn = quatf::set_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), 0.7f);
        const OrientedBox turned = {mat4f::from_rotation_translation(turn, vec3f::make(0.0f, 0.5f, 0.0f)), vec3f::make(0.2f, 0.2f, 0.2f)};
        BodyCollector in_turned = broadphase_query::create_body_collector(CollectorKind::ALL);
        broadphase_query::collide_oriented_box(*world, in_turned, turned, filter);

        // a box swept down from above: the first body whose bounds it enters, and where
        BodyCastCollector first = broadphase_query::create_body_cast_collector(CollectorKind::CLOSEST);
        const AABox falling = aabox::make(vec3f::make(-0.1f, 4.9f, -0.1f), vec3f::make(0.1f, 5.1f, 0.1f));
        broadphase_query::cast_aabb(*world, first, falling, vec3f::make(0.0f, -10.0f, 0.0f), filter);
        if (const BodyCastHit *hit = broadphase_query::hit(first)) printf("bounds entered at %.2f\n", hit->fraction);
        /* SNIPPET_END: broadphase */
        /* HIDE_START */
        if (near_origin.hits.length != 1 || near_origin.hits[0] != crate->id) return 1;
        if (in_turned.hits.length != 1 || in_turned.hits[0] != crate->id) return 1;
        if (!first.has_hit || first.hits[0].body_id != crate->id || !near(first.hits[0].fraction, 0.39f, 1e-4f)) return 1;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: filters */
        // a filter starts with every layer, then narrows: by object or broadphase layer, by group and mask, and by a
        // function of the body
        Filter filter = filter::create(world->settings.layers);
        filter::disable_object_layer(filter, world->settings.layers, moving);
        filter.body_filter = not_this_body;
        filter.body_filter_context = ground;
        CastRayCollector collector = cast_ray::create_collector(CollectorKind::ANY);
        query::cast_ray(*world, collector, cast_ray::default_settings(), {vec3f::make(0.0f, 5.0f, 0.0f), vec3f::make(0.0f, -1.0f, 0.0f), 10.0f},
                        filter);
        // the crate's layer is off and the ground is filtered out: nothing hit
        /* SNIPPET_END: filters */
        /* HIDE_START */
        if (collector.has_hit) return 1;
        /* HIDE_END */
    }

    world::destroy(world);
    crate_shape = nullptr;
    ground_shape = nullptr;
    return 0;
}
