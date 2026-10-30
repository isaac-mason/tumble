// the smallest working setup: a world, a ground, a box dropped onto it, and its position printed as it falls
#include <stdio.h>

#include "tumble/tumble.h"

using namespace tumble;

int main() {
    dispatch::init();

    WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    World *world = world::create(settings);

    Ref<Shape> ground_shape = box::create({.half_extents = vec3f::make(50.0f, 0.5f, 50.0f)});
    Ref<Shape> box_shape = box::create({.half_extents = vec3f::splat(0.5f)});
    rigid_body::create(*world, {.shape = ground_shape, .object_layer = not_moving, .motion_type = MotionType::STATIC,
                                .position = vec3f::make(0.0f, -0.5f, 0.0f)});
    RigidBody *box = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                 .motion_type = MotionType::DYNAMIC, .position = vec3f::make(0.0f, 5.0f, 0.0f)});

    // until the box sleeps on the ground
    int step = 0;
    while (!rigid_body::is_sleeping(*box) && step < 600) {
        world::step(*world, 1.0f / 60.0f);
        step++;
        if (step % 10 == 0) {
            const Vec3f position = box->position;
            const Vec3f velocity = box->motion_properties.linear_velocity;
            printf("step %3d: position (%.3f, %.3f, %.3f), velocity (%.3f, %.3f, %.3f)\n", step, vec3f::x(position), vec3f::y(position),
                   vec3f::z(position), vec3f::x(velocity), vec3f::y(velocity), vec3f::z(velocity));
        }
    }
    printf("asleep after %d steps at y %.3f\n", step, vec3f::y(box->position));
    const bool rested = rigid_body::is_sleeping(*box) && vec3f::y(box->position) > 0.45f && vec3f::y(box->position) < 0.55f;

    world::destroy(world);
    box_shape = nullptr;
    ground_shape = nullptr;
    return rested ? 0 : 1;
}
