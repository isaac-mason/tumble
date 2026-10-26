#pragma once

#include "test.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/rigid_body_step.h"
#include "tumble/physics/constraints/contact_constraints.h"
#include "tumble/physics/contacts.h"
#include "tumble/physics/events.h"
#include "tumble/physics/pairs.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/world.h"

/** a world whose bodies never collide on their own (one layer, colliding with nothing): manifolds written and phases run by hand */
struct ContactScene {
    World *world;
    uint32_t layer;
    collections::Array<uint32_t> indices;
};

constexpr float CONTACT_SCENE_DELTA_TIME = 1.0f / 60.0f;

inline ContactScene make_contact_scene() {
    dispatch::init();
    ContactScene scene = {};
    WorldSettings settings = world_settings::create();
    scene.layer = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    scene.world = world::create(settings);
    return scene;
}

inline void release_contact_scene(ContactScene &scene) {
    scene.indices.release();
    world::destroy(scene.world);
}

struct ContactBodySettings {
    float friction = 0.0f;
    float restitution = 0.0f;
    bool sensor = false;
};

/** a body with no damping; the shape is the body's to keep (the tests never free bodies one by one) */
inline RigidBody *add_contact_body(ContactScene &scene, Shape *shape, MotionType motion_type, const Float3 &position,
                                               const ContactBodySettings &body_settings = {}) {
    RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = scene.layer;
    settings.motion_type = motion_type;
    settings.position = position;
    settings.linear_damping = 0.0f;
    settings.angular_damping = 0.0f;
    settings.friction = body_settings.friction;
    settings.restitution = body_settings.restitution;
    settings.sensor = body_settings.sensor;
    return rigid_body::create(*scene.world, settings);
}

/** a manifold of world points (on a and on b alike, touching) with a unit normal from a to b */
inline ContactManifold touching_manifold(const Float3 *points, uint32_t num_points, const Float3 &normal, float penetration = 0.0f) {
    ContactManifold out = manifold::create();
    out.num_contact_points = num_points;
    out.world_space_normal = vec3f::load(normal);
    out.penetration_depth = penetration;
    for (uint32_t i = 0; i < num_points; i++) {
        out.relative_contact_points_on_a[i] = vec3f::load(points[i]);
        out.relative_contact_points_on_b[i] = vec3f::load(points[i]);
    }
    return out;
}

/** a new step: no constraints, no contact events, the stamps advanced */
inline void begin_contact_step(ContactScene &scene) {
    scene.world->contact_constraints.count = 0;
    scene.world->events.contacts_added.length = 0;
    scene.world->events.contacts_persisted.length = 0;
    scene.world->contacts.frame_stamp++;
    scene.world->bodies.step_stamp++;
}

/** the narrowphase's call for one manifold, through the pair's record (made on first use) */
inline bool add_contact(ContactScene &scene, RigidBody *body_a, RigidBody *body_b,
                        const ContactManifold &manifold) {
    int32_t record = pairs::find_pair_record(scene.world->pairs, *body_a, *body_b);
    if (record == pairs::INVALID_PAIR_KEY) record = pairs::add_pair_record(scene.world->pairs, *body_a, *body_b);
    return contact_constraints::add_contact_constraint(scene.world->contact_constraints, scene.world->contacts, scene.world->pairs, record, *body_a, *body_b,
                                                       manifold, scene.world->settings, scene.world->events, CONTACT_SCENE_DELTA_TIME,
                                                       scene.world->bodies.step_stamp);
}

/** every live constraint, in pool order */
inline const collections::Array<uint32_t> &all_contact_indices(ContactScene &scene) {
    scene.indices.length = 0;
    for (uint32_t i = 0; i < scene.world->contact_constraints.count; i++) scene.indices.push(i);
    return scene.indices;
}

/** warm start, then `iterations` velocity iterations over every constraint */
inline void solve_contact_velocities(ContactScene &scene, uint32_t iterations) {
    const collections::Array<uint32_t> &indices = all_contact_indices(scene);
    contact_constraints::warm_start_velocity_constraints(scene.world->contact_constraints, scene.world->bodies, indices, 1.0f);
    for (uint32_t i = 0; i < iterations; i++) {
        contact_constraints::solve_velocity_constraints_for_island(scene.world->contact_constraints, scene.world->bodies, indices);
    }
}

/** the impulses into the cache, which becomes the next step's read side */
inline void end_contact_step(ContactScene &scene) {
    contact_constraints::store_applied_impulses(scene.world->contact_constraints, scene.world->contacts);
    contacts::flip_manifold_cache(scene.world->contacts);
}
