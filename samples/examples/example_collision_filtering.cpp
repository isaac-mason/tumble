#include "app/app.h"
#include "app/examples.h"
#include "tumble/physics/shapes/box.h"

using namespace tumble;

/** cubes of group a land on the lower platform, b on the upper; each collides only with its own group */
static constexpr uint32_t GROUP_A = 1u << 0;
static constexpr uint32_t GROUP_B = 1u << 1;

static struct {
    app::Layers layers;
} state;

static void init(App &app) {
    app.world = app::create_world(state.layers);

    RigidBodySettings ground;
    ground.shape = app::keep(app, box::create({.half_extents = {5.0f, 0.1f, 5.0f}}));
    ground.object_layer = state.layers.not_moving;
    ground.motion_type = MotionType::STATIC;
    ground.friction = 0.5f;
    rigid_body::create(*app.world, ground);

    Shape *platform_shape = app::keep(app, box::create({.half_extents = {1.0f, 0.1f, 1.0f}}));
    RigidBodySettings lower = ground;
    lower.shape = platform_shape;
    lower.position = {0.0f, 1.0f, 0.0f};
    lower.collision_groups = GROUP_A;
    lower.collision_mask = GROUP_A;
    rigid_body::create(*app.world, lower);

    RigidBodySettings upper = lower;
    upper.position = {0.0f, 2.0f, 0.0f};
    upper.collision_groups = GROUP_B;
    upper.collision_mask = GROUP_B;
    rigid_body::create(*app.world, upper);

    // a 4 x 4 x 4 block of cubes, alternating groups along z
    const int count = 4;
    const float half = 0.1f;
    const float shift = half * 2.0f;
    const float centre_x = shift * (float)(count / 2);
    const float centre_y = 2.5f;
    const float centre_z = shift * (float)(count / 2);
    Shape *cube = app::keep(app, box::create({.half_extents = {half, half, half}}));
    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < count; i++) {
            for (int k = 0; k < count; k++) {
                const uint32_t group = k % 2 == 0 ? GROUP_A : GROUP_B;
                RigidBodySettings settings;
                settings.shape = cube;
                settings.object_layer = state.layers.moving;
                settings.motion_type = MotionType::DYNAMIC;
                settings.position = {(float)i * shift - centre_x, (float)j * shift + centre_y, (float)k * shift - centre_z};
                settings.collision_groups = group;
                settings.collision_mask = group;
                settings.friction = 1.0f;
                settings.restitution = 0.0f;
                rigid_body::create(*app.world, settings);
            }
        }
    }

    camera::look_at(app.camera, {10.0f, 5.0f, 10.0f}, {0.0f, 0.0f, 0.0f});
}

static const Example EXAMPLE_COLLISION_FILTERING = {"Collision Filtering", init, nullptr, nullptr, nullptr, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_COLLISION_FILTERING);
