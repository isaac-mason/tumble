// the smallest working setup: a world, a ground, a box dropped onto it, and its position printed as it falls
#include <stdio.h>

#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/world.h"

using namespace tumble;

int main() {
    dispatch::init();

    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    world::World *world = world::create(settings);
    world_settings::release(settings);

    shapes::Shape *ground_shape = box::create({.half_extents = {50.0f, 0.5f, 50.0f}});
    shapes::Shape *box_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    rigid_body::create(*world, {.shape = ground_shape, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC,
                                .position = {0.0f, -0.5f, 0.0f}});
    rigid_body::RigidBody *box = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                              .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.0f, 5.0f, 0.0f}});

    // until the box sleeps on the ground
    int step = 0;
    while (!box->sleeping && step < 600) {
        world::step(*world, 1.0f / 60.0f);
        step++;
        if (step % 10 == 0) {
            const Float3 velocity = box->motion_properties.linear_velocity;
            printf("step %3d: position (%.3f, %.3f, %.3f), velocity (%.3f, %.3f, %.3f)\n", step, box->position.x, box->position.y,
                   box->position.z, velocity.x, velocity.y, velocity.z);
        }
    }
    printf("asleep after %d steps at y %.3f\n", step, box->position.y);
    const bool rested = box->sleeping && box->position.y > 0.45f && box->position.y < 0.55f;

    world::destroy(world);
    box::destroy(box_shape);
    box::destroy(ground_shape);
    return rested ? 0 : 1;
}
