#include <stdio.h>

#include "app/app.h"
#include "app/examples.h"
#include "imgui.h"
#include "sokol_app.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/sphere.h"

using namespace tumble;

static constexpr int BALL_COUNT = 6;
static const float RESTITUTIONS[BALL_COUNT] = {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f};
static constexpr float SPACING = 3.0f;
static constexpr float DROP_HEIGHT = 5.0f;

static struct {
    app::Layers layers;
    rigid_body::RigidBody *balls[BALL_COUNT];
    bool reset_held;
} state;

/** text centred on a world point, over the scene */
static float ball_x(int i) { return ((float)i - 0.5f * (float)(BALL_COUNT - 1)) * SPACING; }

static void reset_balls(App &app) {
    for (int i = 0; i < BALL_COUNT; i++) {
        rigid_body::set_position(*app.world, state.balls[i], vec3f::make(ball_x(i), DROP_HEIGHT, 0.0f), true);
        app::snap(app, *state.balls[i]);
        rigid_body::set_linear_velocity(*app.world, state.balls[i], vec3f::zero());
        rigid_body::set_angular_velocity(*app.world, state.balls[i], vec3f::zero());
    }
}

static void init(App &app) {
    state.reset_held = false;
    app.world = app::create_world(state.layers);

    rigid_body::RigidBodySettings floor;
    floor.shape = app::keep(app, box::create({.half_extents = {15.0f, 1.0f, 10.0f}}));
    floor.object_layer = state.layers.not_moving;
    floor.motion_type = motion_type::MOTION_STATIC;
    floor.restitution = 0.0f;
    floor.friction = 0.5f;
    rigid_body::create(*app.world, floor);

    shapes::Shape *ball = app::keep(app, sphere::create({.radius = 0.5f}));
    for (int i = 0; i < BALL_COUNT; i++) {
        rigid_body::RigidBodySettings settings;
        settings.shape = ball;
        settings.object_layer = state.layers.moving;
        settings.motion_type = motion_type::MOTION_DYNAMIC;
        settings.position = {ball_x(i), DROP_HEIGHT, 0.0f};
        settings.restitution = RESTITUTIONS[i];
        settings.friction = 0.5f;
        state.balls[i] = rigid_body::create(*app.world, settings);
    }
    camera::look_at(app.camera, {0.0f, 8.0f, 20.0f}, {0.0f, 3.0f, 0.0f});
}

static void update(App &app, float) {
    const bool held = app.keys[SAPP_KEYCODE_R];
    if (held && !state.reset_held) reset_balls(app);
    state.reset_held = held;
}

static void ui(App &app) {
    for (int i = 0; i < BALL_COUNT; i++) {
        char text[16];
        snprintf(text, sizeof(text), "e=%.1f", RESTITUTIONS[i]);
        app::label(app, {ball_x(i), DROP_HEIGHT + 1.5f, 0.0f}, text, {2.0f, 1.0f, 0.44f});
    }
    ImGui::TextUnformatted("R: drop the balls again");
}

static const Example EXAMPLE_RESTITUTION = {"Restitution", init, nullptr, update, ui, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_RESTITUTION);
