#include <stdio.h>

#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/world.h"

using namespace tumble;

static bool near(float a, float b, float tolerance) { return a - b <= tolerance && b - a <= tolerance; }

/* SNIPPET_START: body-filter */
/** keeps every body but one: the caller's own, say */
bool not_this_body(const rigid_body::RigidBody *body, void *context) { return body != static_cast<const rigid_body::RigidBody *>(context); }
/* SNIPPET_END: body-filter */

int main() {
    dispatch::init();
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    world::World *world = world::create(settings);
    world_settings::release(settings);
    shapes::Shape *ground_shape = box::create({.half_extents = {20.0f, 0.5f, 20.0f}});
    shapes::Shape *crate_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    rigid_body::RigidBody *ground = rigid_body::create(*world, {.shape = ground_shape, .object_layer = not_moving,
                                                                 .motion_type = motion_type::MOTION_STATIC, .position = {0.0f, -0.5f, 0.0f}});
    rigid_body::RigidBody *crate = rigid_body::create(*world, {.shape = crate_shape, .object_layer = moving,
                                                                .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.0f, 0.5f, 0.0f}});
    world::step(*world, 1.0f / 60.0f);

    {
        /* SNIPPET_START: cast-ray */
        // down from above the crate, 10 m: the closest hit, its point and the surface normal there
        filter::Filter filter = filter::create(world->settings.layers);
        cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);
        const cast_ray::Rayf ray = {vec3f::make(0.0f, 5.0f, 0.0f), vec3f::make(0.0f, -1.0f, 0.0f), 10.0f};
        query::cast_ray(*world, collector, cast_ray::default_settings(), ray, filter);
        if (const cast_ray::CastRayHit *hit = cast_ray::hit(collector)) {
            const Vec3f point = vec3f::scale_and_add(ray.origin, ray.direction, ray.length * hit->fraction);
            const rigid_body::RigidBody *body = rigid_body::get(*world, hit->body_id_b);
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
        cast_ray::CastRayCollector all = cast_ray::create_collector(collector::COLLECT_ALL);
        query::cast_ray(*world, all, cast_ray::default_settings(), ray, filter);
        cast_ray::sort(all);
        printf("%u hits\n", all.hits.length);
        /* HIDE_START */
        if (all.hits.length != 2 || all.hits[0].body_id_b != crate->id || all.hits[1].body_id_b != ground->id) return 1;
        /* HIDE_END */

        // the collectors and the filter own memory
        cast_ray::release(all);
        cast_ray::release(collector);
        filter::release(filter);
        /* SNIPPET_END: cast-ray */
    }

    {
        filter::Filter filter = filter::create(world->settings.layers);
        /* SNIPPET_START: collide-point */
        // which bodies hold this point
        collide_point::CollidePointCollector inside = collide_point::create_collector(collector::COLLECT_ALL);
        query::collide_point(*world, inside, collide_point::default_settings(), vec3f::make(0.2f, 0.5f, 0.1f), filter);
        /* SNIPPET_END: collide-point */
        /* HIDE_START */
        if (inside.hits.length != 1 || inside.hits[0].body_id_b != crate->id) return 1;
        collide_point::release(inside);
        filter::release(filter);
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: collide-shape */
        // everything a 2 m sphere at the crate's corner overlaps, and how deep: an explosion's reach
        shapes::Shape *blast = sphere::create({.radius = 2.0f});
        filter::Filter filter = filter::create(world->settings.layers);
        collide_shape::CollideShapeCollector overlaps = collide_shape::create_collector(collector::COLLECT_ALL);
        const Transformf at = {{1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
        query::collide_shape(*world, overlaps, collide_shape::default_settings(), blast, at, filter);
        for (uint32_t i = 0; i < overlaps.hits.length; i++) {
            printf("overlaps body %u by %.2f m\n", body_id::index(overlaps.hits[i].body_id_b), overlaps.hits[i].penetration);
        }
        /* SNIPPET_END: collide-shape */
        /* HIDE_START */
        if (overlaps.hits.length != 2) return 1;
        collide_shape::release(overlaps);
        filter::release(filter);
        /* HIDE_END */

        /* SNIPPET_START: cast-shape */
        // the sphere swept along a path: where it first touches, for a projectile with a size
        filter::Filter only_moving = filter::create(world->settings.layers);
        filter::disable_object_layer(only_moving, world->settings.layers, not_moving);
        cast_shape::CastShapeCollector sweep = cast_shape::create_collector(collector::COLLECT_CLOSEST);
        const shapes::Shape *small_ball = blast;
        const Transformf start = {{-10.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.25f, 0.25f, 0.25f}};
        query::cast_shape(*world, sweep, cast_shape::default_settings(), small_ball, start, vec3f::make(20.0f, 0.0f, 0.0f), only_moving);
        if (const cast_shape::CastShapeHit *hit = cast_shape::hit(sweep)) printf("stops after %.2f m\n", 20.0f * hit->fraction);
        /* SNIPPET_END: cast-shape */
        /* HIDE_START */
        // the quarter scale ball (radius 0.5) meets the crate's face at x = -0.5 after 9 m
        if (!sweep.has_hit || !near(20.0f * sweep.hits[0].fraction, 9.0f, 1e-3f)) return 1;
        cast_shape::release(sweep);
        filter::release(only_moving);
        sphere::destroy(blast);
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: filters */
        // a filter starts with every layer, then narrows: by object or broadphase layer, by group and mask, and by a
        // function of the body
        filter::Filter filter = filter::create(world->settings.layers);
        filter::disable_object_layer(filter, world->settings.layers, moving);
        filter.body_filter = not_this_body;
        filter.body_filter_context = ground;
        cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ANY);
        query::cast_ray(*world, collector, cast_ray::default_settings(), {vec3f::make(0.0f, 5.0f, 0.0f), vec3f::make(0.0f, -1.0f, 0.0f), 10.0f},
                        filter);
        // the crate's layer is off and the ground is filtered out: nothing hit
        /* SNIPPET_END: filters */
        /* HIDE_START */
        if (collector.has_hit) return 1;
        /* HIDE_END */
        cast_ray::release(collector);
        filter::release(filter);
    }

    world::destroy(world);
    box::destroy(crate_shape);
    box::destroy(ground_shape);
    return 0;
}
