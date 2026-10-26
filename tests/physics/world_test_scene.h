#pragma once

#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/broadphase/broadphase.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/physics/pairs.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/update.h"
#include "tumble/physics/world.h"
#include "test.h"

constexpr float WORLD_TEST_DELTA_TIME = 1.0f / 60.0f;

/** a moving and a not moving layer */
struct WorldScene {
    World *world;
    uint32_t moving;
    uint32_t not_moving;
};

inline WorldSettings make_world_test_settings(uint32_t &out_moving, uint32_t &out_not_moving) {
    WorldSettings settings = world_settings::create();
    const uint32_t broadphase_moving = world_settings::add_broadphase_layer(settings);
    const uint32_t broadphase_not_moving = world_settings::add_broadphase_layer(settings);
    out_moving = world_settings::add_object_layer(settings, broadphase_moving);
    out_not_moving = world_settings::add_object_layer(settings, broadphase_not_moving);
    world_settings::enable_collision(settings, out_moving, out_moving);
    world_settings::enable_collision(settings, out_moving, out_not_moving);
    return settings;
}

/** a world over the two layers; `tune` adjusts the settings first */
inline WorldScene make_world_scene(TestCtx &ctx, void (*tune)(WorldSettings &) = nullptr) {
    dispatch::init();
    WorldScene scene = {};
    WorldSettings settings = make_world_test_settings(scene.moving, scene.not_moving);
    if (tune != nullptr) tune(settings);
    scene.world = world::create(settings);
    CHECK(ctx, scene.world != nullptr);
    return scene;
}

inline void release_world_scene(WorldScene &scene) { world::destroy(scene.world); }

inline RigidBodySettings body_settings(Shape *shape, uint32_t object_layer, MotionType motion_type, const Float3 &position) {
    RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = object_layer;
    settings.motion_type = motion_type;
    settings.position = position;
    return settings;
}

inline RigidBody *add_body(WorldScene &scene, Shape *shape, MotionType motion_type, const Float3 &position) {
    const uint32_t layer = motion_type == MotionType::STATIC ? scene.not_moving : scene.moving;
    return rigid_body::create(*scene.world, body_settings(shape, layer, motion_type, position));
}

inline void step_world(World &world, float delta_time = WORLD_TEST_DELTA_TIME) { world::step(world, delta_time); }

/** the contacts nested under the pair record of the two bodies, 0 when there is no record */
inline uint32_t pair_contact_count(const World &world, const RigidBody &a, const RigidBody &b) {
    const int32_t record = pairs::find_pair_record(world.pairs, a, b);
    if (record == pairs::INVALID_PAIR_KEY) return 0;
    uint32_t count = 0;
    for (int32_t id = world.pairs.first_contact[(uint32_t)record]; id != contacts::INVALID_CONTACT_KEY;
         id = world.contacts.contacts[(uint32_t)id].next_in_pair) {
        count++;
    }
    return count;
}

/** the live contacts of a body, through its pair edges and each record's chain */
inline uint32_t body_contact_count(const World &world, const RigidBody &body) {
    uint32_t count = 0;
    int32_t edge_key = body.head_pair_key;
    while (edge_key != pairs::INVALID_PAIR_KEY) {
        const int32_t record = pairs::get_pair_edge_record(edge_key);
        const int32_t side = pairs::get_pair_edge_side(edge_key);
        for (int32_t id = world.pairs.first_contact[(uint32_t)record]; id != contacts::INVALID_CONTACT_KEY;
             id = world.contacts.contacts[(uint32_t)id].next_in_pair) {
            count++;
        }
        edge_key = world.pairs.records[(uint32_t)record].edges[side].next;
    }
    return count;
}

/** whether the body index is in this step's moved buffer */
