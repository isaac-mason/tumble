#include <stdio.h>

#include "app/app.h"
#include "app/examples.h"
#include "imgui.h"
#include "tumble/physics/shapes/box.h"

using namespace tumble;

static constexpr int BOX_COUNT = 10;
static constexpr float SPACING = 2.0f;
static constexpr float START_X = -10.0f;
static constexpr float START_HEIGHT = 0.5f;
static constexpr float LAUNCH_SPEED = 8.0f;
/** the boxes are launched again this many steps after all have stopped, a second at 60 */
static constexpr int RESET_DELAY_STEPS = 60;

static struct {
    app::Layers layers;
    rigid_body::RigidBody *boxes[BOX_COUNT];
    int steps;
    /** steps until the boxes are launched again, 0 while they are moving */
    int reset_countdown;
} state;

/** text centred on a world point, over the scene */
static float damping_of(int i) { return 0.1f * (float)i; }
static float x_of(int i) { return START_X + SPACING * (float)i; }

static void launch(App &app) {
    for (int i = 0; i < BOX_COUNT; i++) {
        rigid_body::set_position(*app.world, state.boxes[i], vec3f::make(x_of(i), START_HEIGHT, 0.0f), true);
        rigid_body::set_linear_velocity(*app.world, state.boxes[i], vec3f::make(0.0f, 0.0f, LAUNCH_SPEED));
        rigid_body::set_angular_velocity(*app.world, state.boxes[i], vec3f::zero());
    }
}

static void init(App &app) {
    state.steps = 0;
    state.reset_countdown = 0;
    app.world = app::create_world(state.layers, {0.0f, 0.0f, 0.0f});
    shapes::Shape *cube = app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}));
    for (int i = 0; i < BOX_COUNT; i++) {
        rigid_body::RigidBodySettings settings;
        settings.shape = cube;
        settings.object_layer = state.layers.moving;
        settings.motion_type = motion_type::MOTION_DYNAMIC;
        settings.position = {x_of(i), START_HEIGHT, 0.0f};
        settings.restitution = 0.0f;
        settings.friction = 1.0f;
        settings.linear_damping = damping_of(i);
        state.boxes[i] = rigid_body::create(*app.world, settings);
    }
    launch(app);
    camera::look_at(app.camera, {0.0f, 8.0f, 25.0f}, {0.0f, 2.0f, 0.0f});
    // nothing to slide on
    app.drawing.grid = false;
}

static void step(App &app, float) {
    state.steps++;
    if (state.reset_countdown > 0) {
        if (--state.reset_countdown == 0) launch(app);
        return;
    }
    // stopped, or far enough away
    bool all_stopped = true;
    for (int i = 0; i < BOX_COUNT; i++) {
        const rigid_body::RigidBody *box = state.boxes[i];
        all_stopped = all_stopped && (scalar::abs(box->motion_properties.linear_velocity.z) < 0.01f || box->position.z > 15.0f);
    }
    if (all_stopped && state.steps > 60) state.reset_countdown = RESET_DELAY_STEPS;
}

static void ui(App &app) {
    for (int i = 0; i < BOX_COUNT; i++) {
        char text[16];
        snprintf(text, sizeof(text), "d=%.1f", damping_of(i));
        app::label(app, {x_of(i), START_HEIGHT + 1.5f, 0.0f}, text, {2.0f, 1.0f, 0.44f});
    }
}

static const Example EXAMPLE_LINEAR_DAMPING = {"Linear Damping", init, step, nullptr, ui, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_LINEAR_DAMPING);
