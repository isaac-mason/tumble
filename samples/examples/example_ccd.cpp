#include "app/app.h"
#include "app/examples.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/sphere.h"

using namespace tumble;

static constexpr float GROUND_HEIGHT = 0.1f;
static constexpr int WALLS = 5;
static constexpr int WALL_HEIGHT = 8;

static struct {
    app::Layers layers;
} state;

/** a stack of bricks in the yz plane, each row one brick shorter and set in by half a brick */
static void create_wall(App &app, Shape *brick, Float3 offset, int stack_height) {
    const float shift_y = 1.0f;
    const float shift_z = 2.0f;
    for (int i = 0; i < stack_height; i++) {
        for (int j = i; j < stack_height; j++) {
            RigidBodySettings settings;
            settings.shape = brick;
            settings.object_layer = state.layers.moving;
            settings.motion_type = MotionType::DYNAMIC;
            settings.position = {offset.x, (float)i * shift_y + offset.y,
                                 (float)i * shift_z / 2.0f + (float)(j - i) * shift_z + offset.z - (float)stack_height};
            settings.restitution = 0.0f;
            settings.friction = 0.5f;
            rigid_body::create(*app.world, settings);
        }
    }
}

static void init(App &app) {
    app.world = app::create_world(state.layers);

    RigidBodySettings ground;
    ground.shape = app::keep(app, box::create({.half_extents = {30.0f, GROUND_HEIGHT, 30.0f}}));
    ground.object_layer = state.layers.not_moving;
    ground.motion_type = MotionType::STATIC;
    ground.restitution = 0.0f;
    ground.friction = 0.5f;
    rigid_body::create(*app.world, ground);

    Shape *brick = app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 1.0f}}));
    const float shift_y = GROUND_HEIGHT + 0.5f;
    for (int i = 0; i < WALLS; i++) create_wall(app, brick, {(float)i * 6.0f, shift_y, 0.0f}, WALL_HEIGHT);

    // heavy and fast enough to pass through a brick in a step, but swept, so it ploughs through the walls instead
    RigidBodySettings ball;
    ball.shape = app::keep(app, sphere::create({.radius = 1.0f, .density = 10000.0f}));
    ball.object_layer = state.layers.moving;
    ball.motion_type = MotionType::DYNAMIC;
    ball.position = {-20.0f, shift_y + 2.0f, 0.0f};
    ball.restitution = 0.0f;
    ball.friction = 0.5f;
    ball.motion_quality = MotionQuality::LINEAR_CAST;
    RigidBody *fast_ball = rigid_body::create(*app.world, ball);
    rigid_body::add_impulse(*app.world, fast_ball, vec3f::make(150000000.0f, 0.0f, 0.0f));

    camera::look_at(app.camera, {-31.96f, 19.73f, -27.86f}, {-0.0505f, -0.4126f, -0.0229f});
}

static const Example EXAMPLE_CCD = {"Continuous Collision Detection", init, nullptr, nullptr, nullptr, nullptr, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_CCD);
