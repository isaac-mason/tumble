#include <stdio.h>

#include "app/app.h"
#include "app/examples.h"
#include "imgui.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/sphere.h"

using namespace tumble;

static constexpr int ROW_COUNT = 10;
static constexpr float SPACING = 2.0f;
static constexpr float ROW_WIDTH = (ROW_COUNT - 1) * SPACING;
static constexpr float START_HEIGHT = 0.5f;
static constexpr int KIND_COUNT = 3;

static struct {
    app::Layers layers;
} state;

/** text centred on a world point, over the scene */
static float friction_of(int i) { return (float)i / (float)(ROW_COUNT - 1); }

/** spheres on the left, boxes that cannot turn in the middle, capsules lying on their side on the right */
static float x_of(int kind, int i) { return -0.5f * ROW_WIDTH + (float)(kind - 1) * (ROW_WIDTH + SPACING) + SPACING * (float)i; }

static void init(App &app) {
    app.world = app::create_world(state.layers);

    RigidBodySettings floor;
    floor.shape = app::keep(app, box::create({.half_extents = {50.0f, 0.5f, 300.0f}}));
    floor.object_layer = state.layers.not_moving;
    floor.motion_type = MotionType::STATIC;
    floor.position = {0.0f, -0.5f, 0.0f};
    floor.restitution = 0.0f;
    floor.friction = 1.0f;
    rigid_body::create(*app.world, floor);

    Shape *ball = app::keep(app, sphere::create({.radius = 0.5f}));
    Shape *block = app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}));
    Shape *pill = app::keep(app, capsule::create({.half_height_of_cylinder = 0.3f, .radius = 0.5f}));
    for (int kind = 0; kind < KIND_COUNT; kind++) {
        for (int i = 0; i < ROW_COUNT; i++) {
            RigidBodySettings settings;
            settings.shape = kind == 0 ? ball : kind == 1 ? block : pill;
            settings.object_layer = state.layers.moving;
            settings.motion_type = MotionType::DYNAMIC;
            settings.position = {x_of(kind, i), START_HEIGHT, 0.0f};
            if (kind == 1) settings.allowed_degrees_of_freedom = dof::DOF_TRANSLATION_ONLY;
            if (kind == 2) settings.quaternion = {0.0f, 0.0f, scalar::sin(0.25f * scalar::PI), scalar::cos(0.25f * scalar::PI)};
            settings.restitution = 0.0f;
            settings.friction = friction_of(i);
            RigidBody *body = rigid_body::create(*app.world, settings);
            rigid_body::set_linear_velocity(*app.world, body, vec3f::make(0.0f, 0.0f, 10.0f));
        }
    }
    camera::look_at(app.camera, {0.0f, 8.0f, 25.0f}, {0.0f, 2.0f, 0.0f});
}

static void ui(App &app) {
    for (int kind = 0; kind < KIND_COUNT; kind++) {
        for (int i = 0; i < ROW_COUNT; i++) {
            char text[16];
            snprintf(text, sizeof(text), "mu=%.1f", friction_of(i));
            app::label(app, {x_of(kind, i), START_HEIGHT + 1.5f, 0.0f}, text, {2.0f, 1.0f, 0.44f});
        }
    }
}

static const Example EXAMPLE_FRICTION = {"Friction", init, nullptr, nullptr, ui, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_FRICTION);
