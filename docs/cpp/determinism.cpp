#include <stdio.h>
#include <string.h>

#include "tumble/tumble.h"

using namespace tumble;

/** the pile's hash, the same natively, in the wasm, and through the js package */
static constexpr uint32_t PILE_HASH = 0x2a2c339cu;

static World *make_pile(Shape *ground_shape, Shape *ball_shape) {
    WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    World *world = world::create(settings);
    rigid_body::create(*world, {.shape = ground_shape, .object_layer = not_moving, .motion_type = MotionType::STATIC,
                                .position = {0.0f, -0.5f, 0.0f}});
    for (int i = 0; i < 20; i++) {
        // offsets exact in every float type, so js builds the same pile
        rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving, .motion_type = MotionType::DYNAMIC,
                                    .position = {0.0625f * (float)(i % 3), 0.625f + 1.125f * (float)i, 0.03125f * (float)(i % 4)}});
    }
    return world;
}

int main() {
    dispatch::init();
    Ref<Shape> ground_shape = box::create({.half_extents = {10.0f, 0.5f, 10.0f}});
    Ref<Shape> ball_shape = sphere::create({.radius = 0.45f});

    /* SNIPPET_START: replay */
    // the same scene, built in the same order and stepped the same way, gives exactly the same result. this holds in
    // another world, another run, another machine, or the browser
    World *first = make_pile(ground_shape, ball_shape);
    World *second = make_pile(ground_shape, ball_shape);
    for (int i = 0; i < 300; i++) {
        world::step(*first, 1.0f / 60.0f);
        world::step(*second, 1.0f / 60.0f);
    }
    bool same = true;
    for (uint32_t i = 0; i < first->bodies.pool.length; i++) {
        const RigidBody &a = *first->bodies.pool[i];
        const RigidBody &b = *second->bodies.pool[i];
        same = same && memcmp(&a.position, &b.position, sizeof(Float3)) == 0 && memcmp(&a.quaternion, &b.quaternion, sizeof(Float4)) == 0;
    }
    printf("%s\n", same ? "bit for bit the same" : "they differ");
    /* SNIPPET_END: replay */
    /* HIDE_START */
    if (!same) return 1;
    // the pile's poses, hashed as js/docs/determinism.ts hashes them: the js package must give the same bits
    uint32_t hash = 2166136261u;
    const auto word = [&hash](uint32_t w) {
        for (int k = 0; k < 4; k++) {
            hash ^= (w >> (k * 8)) & 0xffu;
            hash *= 16777619u;
        }
    };
    for (uint32_t i = 0; i < first->bodies.pool.length; i++) {
        const RigidBody &body = *first->bodies.pool[i];
        const float values[7] = {body.position.x,   body.position.y,   body.position.z,  body.quaternion.x,
                                 body.quaternion.y, body.quaternion.z, body.quaternion.w};
        for (float value : values) {
            uint32_t bits;
            memcpy(&bits, &value, sizeof bits);
            word(bits);
        }
    }
    printf("pile 0x%08x\n", hash);
    if (hash != PILE_HASH) return 1;
    /* HIDE_END */

    world::destroy(first);
    world::destroy(second);
    ball_shape = nullptr;
    ground_shape = nullptr;
    return 0;
}
