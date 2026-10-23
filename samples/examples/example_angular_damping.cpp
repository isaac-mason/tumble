#include <stdio.h>

#include "app/app.h"
#include "app/examples.h"
#include "imgui.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/shapes/box.h"

using namespace tumble;

static constexpr int BOX_COUNT = 10;
static constexpr float SPACING = 3.0f;
static constexpr float START_X = -13.5f;
static constexpr float START_Y = 0.0f;
/** the boxes are spun again this many steps after all have stopped, a second at 60 */
static constexpr int RESET_DELAY_STEPS = 60;

static struct {
    rigid_body::RigidBody *boxes[BOX_COUNT];
    int steps;
    /** steps until the boxes are spun again, 0 while they are spinning */
    int reset_countdown;
} state;

/** text centred on a world point, over the scene */
static float damping_of(int i) { return 0.1f * (float)i; }
static float x_of(int i) { return START_X + SPACING * (float)i; }

static void spin(App &app) {
    for (int i = 0; i < BOX_COUNT; i++) {
        rigid_body::set_transform(*app.world, state.boxes[i], vec3f::make(x_of(i), START_Y, 0.0f), quatf::identity(), true);
        app::snap(app, *state.boxes[i]);
        rigid_body::set_linear_velocity(*app.world, state.boxes[i], vec3f::zero());
        rigid_body::set_angular_velocity(*app.world, state.boxes[i], vec3f::make(3.0f, 5.0f, 2.0f));
    }
}

static void init(App &app) {
    state.steps = 0;
    state.reset_countdown = 0;
    // one layer that collides with nothing, and no gravity: the boxes only spin
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    settings.gravity = {0.0f, 0.0f, 0.0f};
    app.world = world::create(settings);
    world_settings::release(settings);

    shapes::Shape *cube = app::keep(app, box::create({.half_extents = {0.75f, 0.75f, 0.75f}}));
    for (int i = 0; i < BOX_COUNT; i++) {
        rigid_body::RigidBodySettings body;
        body.shape = cube;
        body.object_layer = moving;
        body.motion_type = motion_type::MOTION_DYNAMIC;
        body.position = {x_of(i), START_Y, 0.0f};
        body.angular_damping = damping_of(i);
        state.boxes[i] = rigid_body::create(*app.world, body);
    }
    spin(app);
    camera::look_at(app.camera, {0.0f, 0.0f, 25.0f}, {0.0f, 0.0f, 0.0f});
    // nothing to stand on
    app.drawing.grid = false;
}

static void step(App &app, float) {
    state.steps++;
    if (state.reset_countdown > 0) {
        if (--state.reset_countdown == 0) spin(app);
        return;
    }
    bool all_stopped = true;
    for (int i = 0; i < BOX_COUNT; i++) {
        const Float3 &w = state.boxes[i]->motion_properties.angular_velocity;
        all_stopped = all_stopped && scalar::sqrt(w.x * w.x + w.y * w.y + w.z * w.z) < 0.05f;
    }
    if (all_stopped && state.steps > 60) state.reset_countdown = RESET_DELAY_STEPS;
}

static void ui(App &app) {
    for (int i = 0; i < BOX_COUNT; i++) {
        char text[16];
        snprintf(text, sizeof(text), "d=%.1f", damping_of(i));
        app::label(app, {x_of(i), START_Y + 2.0f, 0.0f}, text, {2.0f, 1.0f, 0.44f});
    }
}

static const Example EXAMPLE_ANGULAR_DAMPING = {"Angular Damping", init, step, nullptr, ui, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_ANGULAR_DAMPING);
