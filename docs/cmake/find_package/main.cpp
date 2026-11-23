#include "tumble/tumble.h"

using namespace tumble;

int main() {
    dispatch::init();

    WorldSettings settings = world_settings::create();
    const uint32_t broadphase = world_settings::add_broadphase_layer(settings);
    const uint32_t layer = world_settings::add_object_layer(settings, broadphase);
    world_settings::enable_collision(settings, layer, layer);

    World *world = world::create(settings);

    Ref<Shape> floor_shape = box::create({.half_extents = vec3f::make(20.0f, 0.5f, 20.0f)});
    rigid_body::create(*world, {
        .shape = floor_shape,
        .object_layer = layer,
        .motion_type = MotionType::STATIC,
        .position = vec3f::make(0.0f, -0.5f, 0.0f),
    });

    Ref<Shape> ball_shape = sphere::create({.radius = 0.5f});
    RigidBody *ball = rigid_body::create(*world, {
        .shape = ball_shape,
        .object_layer = layer,
        .motion_type = MotionType::DYNAMIC,
        .position = vec3f::make(0.0f, 2.0f, 0.0f),
    });

    for (int i = 0; i < 120; i++) world::step(*world, 1.0f / 60.0f);
    const float height = vec3f::y(ball->position);

    world::destroy(world);
    ball_shape = nullptr;
    floor_shape = nullptr;
    return height > 0.45f && height < 0.55f ? 0 : 1;
}
