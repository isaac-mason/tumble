#include <stdio.h>

#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/debug.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/world.h"

using namespace tumble;

/** a renderer's line list upload, standing in for yours: here it keeps the highest point drawn */
static float highest_drawn = -1e30f;
void upload_line_vertices(const float *vertices, uint32_t vertex_count) {
    for (uint32_t i = 0; i < vertex_count; i++) {
        if (vertices[i * 6 + 1] > highest_drawn) highest_drawn = vertices[i * 6 + 1];
    }
}

/* SNIPPET_START: read */
// line_count lines of two vertices each, a vertex x y z r g b: a vertex array as it stands
void draw_lines(const debug::DebugBuffers &buffers) { upload_line_vertices(buffers.lines.data, buffers.line_count * 2); }
/* SNIPPET_END: read */

int main() {
    dispatch::init();
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    world::World *world = world::create(settings);
    world_settings::release(settings);
    shapes::Shape *ground_shape = box::create({.half_extents = {5.0f, 0.5f, 5.0f}});
    shapes::Shape *ball_shape = sphere::create({.radius = 0.5f});
    rigid_body::RigidBody *ground = rigid_body::create(*world, {.shape = ground_shape, .object_layer = not_moving,
                                                                 .motion_type = motion_type::MOTION_STATIC, .position = {0.0f, -0.5f, 0.0f}});
    rigid_body::RigidBody *ball = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                               .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.0f, 0.5f, 0.0f}});
    point_constraint::create(*world, ground, ball, {.point_a = {0.0f, 1.0f, 0.0f}, .point_b = {0.0f, 1.0f, 0.0f}});
    world::step(*world, 1.0f / 60.0f);

    /* SNIPPET_START: draw */
    // buffers that grow as they are drawn into and keep their memory when cleared, for a frame's worth each frame
    debug::DebugBuffers *buffers = debug::create();
    debug::clear(*buffers);
    // the bodies, coloured by motion type, with their velocities
    debug::bodies(*buffers, *world, {.color_mode = debug::COLOR_MOTION_TYPE, .show_linear_velocity = true});
    // each contact's points and normal, and the constraints' anchors and axes
    debug::contacts(*buffers, *world);
    debug::joints(*buffers, *world, {});
    // a shape anywhere: a preview of where something would go
    debug::shape(*buffers, ball_shape, vec3f::make(3.0f, 0.5f, 0.0f), quatf::identity(), {.color = {0.0f, 1.0f, 0.0f}});
    draw_lines(*buffers);
    printf("%u lines\n", buffers->line_count);
    /* HIDE_START */
    // the ground's twelve edges and more; the ball's top, 1 m up, the highest thing drawn
    if (buffers->line_count < 12 || highest_drawn < 0.99f || highest_drawn > 1.2f) return 1;
    /* HIDE_END */
    debug::release(buffers);
    /* SNIPPET_END: draw */

    world::destroy(world);
    sphere::destroy(ball_shape);
    box::destroy(ground_shape);
    return 0;
}
