/* SNIPPET_START: quick-start */
#include <stdio.h>

#include "tumble/tumble.h"

using namespace tumble;

int main() {
    // register the shape types and their pair functions, once before anything collides
    dispatch::init();

    // two object layers, one for what moves and one for what does not. each has its own broadphase tree
    WorldSettings settings = world_settings::create();
    const uint32_t broadphase_moving = world_settings::add_broadphase_layer(settings);
    const uint32_t broadphase_not_moving = world_settings::add_broadphase_layer(settings);
    const uint32_t moving = world_settings::add_object_layer(settings, broadphase_moving);
    const uint32_t not_moving = world_settings::add_object_layer(settings, broadphase_not_moving);
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);

    // the world copies the settings
    World *world = world::create(settings);

    // a static floor with its top at y = 0, and a box and a ball dropped onto it
    Ref<Shape> floor_shape = box::create({.half_extents = {20.0f, 0.5f, 20.0f}});
    Ref<Shape> box_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    Ref<Shape> ball_shape = sphere::create({.radius = 0.5f});
    rigid_body::create(*world, {.shape = floor_shape, .object_layer = not_moving, .motion_type = MotionType::STATIC,
                                .position = {0.0f, -0.5f, 0.0f}});
    RigidBody *box = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                              .motion_type = MotionType::DYNAMIC, .position = {0.0f, 4.0f, 0.0f}});
    RigidBody *ball = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                               .motion_type = MotionType::DYNAMIC, .position = {0.2f, 6.0f, 0.0f}});

    // two seconds at 60 steps a second
    for (int i = 0; i < 120; i++) world::step(*world, 1.0f / 60.0f);
    printf("box at y %.2f, ball at y %.2f\n", box->position.y, ball->position.y);
    /* HIDE_START */
    // the box rests on the floor and the ball on the box or the floor, nothing fell through
    if (box->position.y < 0.45f || box->position.y > 0.55f || ball->position.y < 0.45f) return 1;
    /* HIDE_END */

    // the world frees its bodies. your shapes are freed once nothing uses them
    world::destroy(world);
    ball_shape = nullptr;
    box_shape = nullptr;
    floor_shape = nullptr;
    return 0;
}
/* SNIPPET_END: quick-start */
