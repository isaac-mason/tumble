/* SNIPPET_START: quick-start */
#include <stdio.h>

#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/world.h"

using namespace tumble;

int main() {
    // the shape types' functions and every pair of them, once before anything collides
    dispatch::init();

    // two object layers: things that move, and things that do not, each in a broadphase tree of its own
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t broadphase_moving = world_settings::add_broadphase_layer(settings);
    const uint32_t broadphase_not_moving = world_settings::add_broadphase_layer(settings);
    const uint32_t moving = world_settings::add_object_layer(settings, broadphase_moving);
    const uint32_t not_moving = world_settings::add_object_layer(settings, broadphase_not_moving);
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);

    // the world copies the settings
    world::World *world = world::create(settings);
    world_settings::release(settings);

    // a static floor, its top at y = 0, and a box and a ball dropped onto it
    shapes::Shape *floor_shape = box::create({.half_extents = {20.0f, 0.5f, 20.0f}});
    shapes::Shape *box_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    shapes::Shape *ball_shape = sphere::create({.radius = 0.5f});
    rigid_body::create(*world, {.shape = floor_shape, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC,
                                .position = {0.0f, -0.5f, 0.0f}});
    rigid_body::RigidBody *box = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                              .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.0f, 4.0f, 0.0f}});
    rigid_body::RigidBody *ball = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                               .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.2f, 6.0f, 0.0f}});

    // two seconds at 60 steps a second
    for (int i = 0; i < 120; i++) world::step(*world, 1.0f / 60.0f);
    printf("box at y %.2f, ball at y %.2f\n", box->position.y, ball->position.y);
    /* HIDE_START */
    // the box rests on the floor and the ball on the box or the floor, nothing fell through
    if (box->position.y < 0.45f || box->position.y > 0.55f || ball->position.y < 0.45f) return 1;
    /* HIDE_END */

    // the world frees its bodies; the shapes are yours, freed once nothing uses them
    world::destroy(world);
    sphere::destroy(ball_shape);
    box::destroy(box_shape);
    box::destroy(floor_shape);
    return 0;
}
/* SNIPPET_END: quick-start */
