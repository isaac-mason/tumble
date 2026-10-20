#include <stdio.h>
#include <string.h>

#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/world.h"

using namespace tumble;

static world::World *make_pile(shapes::Shape *ground_shape, shapes::Shape *ball_shape) {
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    world::World *world = world::create(settings);
    world_settings::release(settings);
    rigid_body::create(*world, {.shape = ground_shape, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC,
                                .position = {0.0f, -0.5f, 0.0f}});
    for (int i = 0; i < 20; i++) {
        rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving, .motion_type = motion_type::MOTION_DYNAMIC,
                                    .position = {0.05f * (float)(i % 3), 0.6f + 1.1f * (float)i, 0.03f * (float)(i % 4)}});
    }
    return world;
}

int main() {
    dispatch::init();
    shapes::Shape *ground_shape = box::create({.half_extents = {10.0f, 0.5f, 10.0f}});
    shapes::Shape *ball_shape = sphere::create({.radius = 0.45f});

    /* SNIPPET_START: replay */
    // the same scene, made in the same order and stepped by the same steps, is the same to the last bit: in another
    // world, another run, another machine, or the browser
    world::World *first = make_pile(ground_shape, ball_shape);
    world::World *second = make_pile(ground_shape, ball_shape);
    for (int i = 0; i < 300; i++) {
        world::step(*first, 1.0f / 60.0f);
        world::step(*second, 1.0f / 60.0f);
    }
    bool same = true;
    for (uint32_t i = 0; i < first->bodies.pool.length; i++) {
        const rigid_body::RigidBody &a = *first->bodies.pool[i];
        const rigid_body::RigidBody &b = *second->bodies.pool[i];
        same = same && memcmp(&a.position, &b.position, sizeof(Float3)) == 0 && memcmp(&a.quaternion, &b.quaternion, sizeof(Float4)) == 0;
    }
    printf("%s\n", same ? "bit for bit the same" : "they differ");
    /* SNIPPET_END: replay */
    /* HIDE_START */
    if (!same) return 1;
    /* HIDE_END */

    world::destroy(first);
    world::destroy(second);
    sphere::destroy(ball_shape);
    box::destroy(ground_shape);
    return 0;
}
