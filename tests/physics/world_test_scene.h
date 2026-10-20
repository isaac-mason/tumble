#pragma once

// the world tests' scene: crashcat's createTestWorld (a moving and a not moving layer), the world level body calls the
// facade will make (create into the broadphase, remove with its constraints and pairs, teleport), and the step as js
// drives it, the five phases in order.

#include "instance.h"
#include "math/quatf.h"
#include "math/vec3f.h"
#include "physics/body/rigid_body.h"
#include "physics/broadphase/broadphase.h"
#include "physics/constraints/constraints.h"
#include "physics/pairs.h"
#include "physics/shapes/box.h"
#include "physics/shapes/capsule.h"
#include "physics/shapes/dispatch.h"
#include "physics/shapes/sphere.h"
#include "physics/update.h"
#include "physics/world.h"
#include "test.h"

constexpr float WORLD_TEST_DELTA_TIME = 1.0f / 60.0f;

struct WorldScene {
    world::World *world;
    uint32_t moving;
    uint32_t not_moving;
};

inline world_settings::WorldSettings make_world_test_settings(uint32_t &out_moving, uint32_t &out_not_moving) {
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t broadphase_moving = world_settings::add_broadphase_layer(settings);
    const uint32_t broadphase_not_moving = world_settings::add_broadphase_layer(settings);
    out_moving = world_settings::add_object_layer(settings, broadphase_moving);
    out_not_moving = world_settings::add_object_layer(settings, broadphase_not_moving);
    world_settings::enable_collision(settings, out_moving, out_moving);
    world_settings::enable_collision(settings, out_moving, out_not_moving);
    return settings;
}

/** a fresh heap and a world over the two layers; `tune` adjusts the settings first */
inline WorldScene make_world_scene(TestCtx &ctx, void (*tune)(world_settings::WorldSettings &) = nullptr) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    WorldScene scene = {};
    world_settings::WorldSettings settings = make_world_test_settings(scene.moving, scene.not_moving);
    if (tune != nullptr) tune(settings);
    scene.world = world::create(settings);
    CHECK(ctx, scene.world != nullptr);
    return scene;
}

inline void release_world_scene(WorldScene &scene) { world::destroy(scene.world); }

/** rigidBody.create(world, ...): the body, in the broadphase and the move set */
inline rigid_body::RigidBody *add_body(world::World &world, const rigid_body::RigidBodySettings &settings) {
    rigid_body::RigidBody *body = rigid_body::create(world.bodies, world.settings.layers, settings);
    if (body == nullptr) return nullptr;
    broadphase::add_body(world.broadphase, body, world.settings.layers);
    pairs::mark_moved(world.pairs, *body);
    return body;
}

inline rigid_body::RigidBodySettings body_settings(shapes::Shape *shape, uint32_t object_layer, motion_type::MotionType motion_type, const Float3 &position) {
    rigid_body::RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = object_layer;
    settings.motion_type = motion_type;
    settings.position = position;
    return settings;
}

inline rigid_body::RigidBody *add_body(WorldScene &scene, shapes::Shape *shape, motion_type::MotionType motion_type, const Float3 &position) {
    const uint32_t layer = motion_type == motion_type::MOTION_STATIC ? scene.not_moving : scene.moving;
    return add_body(*scene.world, body_settings(shape, layer, motion_type, position));
}

/** rigidBody.remove(world, ...): off the active list, its constraints and pairs gone (removals queued), out of the broadphase */
inline bool remove_body(world::World &world, rigid_body::RigidBody *body) {
    if (!bodies::is_valid(world.bodies, body->id)) return false;
    sleep::remove_body_from_active_bodies(world.bodies, body);
    constraints::destroy_body_constraints(world.constraints, world.bodies, *body);
    pairs::purge_body_pairs(world.pairs, world.contacts, world.bodies, *body, world.events);
    broadphase::remove_body(world.broadphase, body);
    return rigid_body::remove(world.bodies, body);
}

/** rigidBody.setTransform(world, ...): the pose, published to the broadphase */
inline void set_body_transform(world::World &world, rigid_body::RigidBody *body, const Float3 &position, const Float4 &quaternion, bool wake) {
    rigid_body::set_transform(world.bodies, body, vec3f::load(position), quatf::load(quaternion), wake);
    if (broadphase::update_body(world.broadphase, body)) pairs::mark_moved(world.pairs, *body);
}

/** one step as js makes it, answering nothing: find_pairs and the narrowphase again while the narrowphase woke bodies */
inline void step_world(world::World &world, float delta_time = WORLD_TEST_DELTA_TIME) {
    update::step_begin(world, delta_time);
    do {
        update::find_pairs(world);
        update::narrowphase(world);
    } while (world.events.woken > 0);
    update::solve(world);
    update::step_end(world);
}

/** the contacts nested under the pair record of the two bodies, 0 when there is no record */
inline uint32_t pair_contact_count(const world::World &world, const rigid_body::RigidBody &a, const rigid_body::RigidBody &b) {
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
inline uint32_t body_contact_count(const world::World &world, const rigid_body::RigidBody &body) {
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
inline bool moved_this_step(const world::World &world, uint32_t body_index) {
    for (uint32_t i = 0; i < world.events.moved.length; i++) {
        if (world.events.moved[i] == body_index) return true;
    }
    return false;
}
