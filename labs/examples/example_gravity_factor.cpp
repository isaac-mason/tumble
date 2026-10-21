#include <stdio.h>

#include "app/app.h"
#include "app/examples.h"
#include "imgui.h"
#include "tumble/physics/shapes/box.h"

using namespace tumble;

static constexpr int BOX_COUNT = 11;
static constexpr float SPACING = 2.0f;
static constexpr float START_X = -10.0f;
static constexpr float START_HEIGHT = 8.0f;
/** the boxes are dropped again after this many steps with all settled, two seconds at 60 */
static constexpr int RESET_STEPS = 120;

static struct {
    app::Layers layers;
    rigid_body::RigidBody *boxes[BOX_COUNT];
    int settled_steps;
} state;

/** text centred on a world point, over the scene */
static float gravity_factor_of(int i) { return 0.2f * (float)i; }
static float x_of(int i) { return START_X + SPACING * (float)i; }

static void drop(App &app) {
    for (int i = 0; i < BOX_COUNT; i++) {
        rigid_body::set_position(*app.world, state.boxes[i], vec3f::make(x_of(i), START_HEIGHT, 0.0f), true);
        rigid_body::set_linear_velocity(*app.world, state.boxes[i], vec3f::zero());
        rigid_body::set_angular_velocity(*app.world, state.boxes[i], vec3f::zero());
    }
    state.settled_steps = 0;
}

static void init(App &app) {
    state.settled_steps = 0;
    app.world = app::create_world(state.layers);

    rigid_body::RigidBodySettings floor;
    floor.shape = app::keep(app, box::create({.half_extents = {20.0f, 0.5f, 10.0f}}));
    floor.object_layer = state.layers.not_moving;
    floor.motion_type = motion_type::MOTION_STATIC;
    floor.position = {0.0f, -0.5f, 0.0f};
    floor.restitution = 0.3f;
    floor.friction = 0.5f;
    rigid_body::create(*app.world, floor);

    shapes::Shape *cube = app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}));
    for (int i = 0; i < BOX_COUNT; i++) {
        rigid_body::RigidBodySettings settings;
        settings.shape = cube;
        settings.object_layer = state.layers.moving;
        settings.motion_type = motion_type::MOTION_DYNAMIC;
        settings.position = {x_of(i), START_HEIGHT, 0.0f};
        settings.gravity_factor = gravity_factor_of(i);
        settings.restitution = 0.3f;
        settings.friction = 0.5f;
        state.boxes[i] = rigid_body::create(*app.world, settings);
    }
    camera::look_at(app.camera, {0.0f, 8.0f, 15.0f}, {0.0f, 2.0f, 0.0f});
}

static void step(App &app, float) {
    // resting on the floor, or fallen off it
    bool all_settled = true;
    for (int i = 0; i < BOX_COUNT; i++) {
        const rigid_body::RigidBody *box = state.boxes[i];
        const bool on_floor = box->position.y < 1.5f && scalar::abs(box->motion_properties.linear_velocity.y) < 0.1f;
        all_settled = all_settled && (on_floor || box->position.y < -5.0f);
    }
    if (all_settled && ++state.settled_steps > RESET_STEPS) drop(app);
}

static void ui(App &app) {
    // the labels follow the boxes
    for (int i = 0; i < BOX_COUNT; i++) {
        char text[16];
        snprintf(text, sizeof(text), "g x%.1f", gravity_factor_of(i));
        const Float3 &at = state.boxes[i]->position;
        app::label(app, {at.x, at.y + 1.5f, at.z}, text, {2.0f, 1.0f, 0.44f});
    }
}

static const Example EXAMPLE_GRAVITY_FACTOR = {"Gravity Factor", init, step, nullptr, ui, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_GRAVITY_FACTOR);
