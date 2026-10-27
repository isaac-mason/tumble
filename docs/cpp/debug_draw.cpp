#include <stdio.h>

#include "tumble/tumble.h"

using namespace tumble;

/** a renderer's line list upload, standing in for yours: here it keeps the highest point drawn */
static float highest_drawn = -1e30f;
void upload_line_vertices(const float *vertices, uint32_t vertex_count) {
    for (uint32_t i = 0; i < vertex_count; i++) {
        if (vertices[i * 6 + 1] > highest_drawn) highest_drawn = vertices[i * 6 + 1];
    }
}

/* SNIPPET_START: read */
// line_count lines of two vertices each. each vertex is x y z r g b, ready to use as a vertex array
void draw_lines(const DebugDraw &buffers) {
    upload_line_vertices(buffers.lines.data, buffers.line_count * 2);
}
/* SNIPPET_END: read */

int main() {
    dispatch::init();
    WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    World *world = world::create(settings);
    Ref<Shape> ground_shape = box::create({.half_extents = {5.0f, 0.5f, 5.0f}});
    Ref<Shape> ball_shape = sphere::create({.radius = 0.5f});
    RigidBody *ground = rigid_body::create(*world, {
        .shape = ground_shape,
        .object_layer = not_moving,
        .motion_type = MotionType::STATIC,
        .position = {0.0f, -0.5f, 0.0f},
    });
    RigidBody *ball = rigid_body::create(*world, {
        .shape = ball_shape,
        .object_layer = moving,
        .motion_type = MotionType::DYNAMIC,
        .position = {0.0f, 0.5f, 0.0f},
    });
    point_constraint::create(*world, ground, ball, {.point_a = {0.0f, 1.0f, 0.0f}, .point_b = {0.0f, 1.0f, 0.0f}});
    world::step(*world, 1.0f / 60.0f);

    /* SNIPPET_START: draw */
    // the buffers grow as you draw into them, and keep their memory when cleared. clear and redraw
    // them each frame
    DebugDraw buffers;
    debug::clear(buffers);

    // the bodies, coloured by motion type, with their velocities
    debug::bodies(buffers, *world, {
        .color_mode = DebugDraw::BodyColorMode::MOTION_TYPE,
        .show_linear_velocity = true,
    });

    // each contact's points and normal, and the constraints' anchors and axes
    debug::contacts(buffers, *world);
    debug::joints(buffers, *world, {});

    // a shape at any pose, such as a preview of where something would go
    const Vec3f preview_position = vec3f::make(3.0f, 0.5f, 0.0f);
    const Float3 green = {0.0f, 1.0f, 0.0f};
    debug::shape(buffers, ball_shape, preview_position, quatf::identity(), {.color = green});
    draw_lines(buffers);
    printf("%u lines\n", buffers.line_count);
    /* HIDE_START */
    // the ground's twelve edges and more; the ball's top, 1 m up, the highest thing drawn
    if (buffers.line_count < 12 || highest_drawn < 0.99f || highest_drawn > 1.2f) return 1;
    /* HIDE_END */
    /* SNIPPET_END: draw */

    world::destroy(world);
    ball_shape = nullptr;
    ground_shape = nullptr;
    return 0;
}
