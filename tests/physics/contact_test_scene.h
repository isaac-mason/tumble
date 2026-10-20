#pragma once

// the contact constraint and island tests' world without a world: bodies, pairs, contacts and contact
// constraints, with manifolds written by hand and the solver phases run by hand.

#include "instance.h"
#include "math/vec3f.h"
#include "math/vec3f.h"
#include "math/vec3f.h"
#include "physics/body/rigid_body.h"
#include "math/vec3f.h"
#include "physics/body/rigid_body_step.h"
#include "math/vec3f.h"
#include "physics/constraints/contact_constraints.h"
#include "math/vec3f.h"
#include "physics/contacts.h"
#include "math/vec3f.h"
#include "physics/events.h"
#include "math/vec3f.h"
#include "physics/pairs.h"
#include "math/vec3f.h"
#include "physics/shapes/box.h"
#include "math/vec3f.h"
#include "physics/shapes/dispatch.h"
#include "math/vec3f.h"
#include "physics/shapes/sphere.h"
#include "math/vec3f.h"
#include "physics/world_settings.h"
#include "math/vec3f.h"
#include "test.h"
#include "math/vec3f.h"

struct ContactScene {
    bodies::Bodies bodies;
    layers::Layers layers;
    pairs::Pairs pairs;
    contacts::Contacts contacts;
    events::PhysicsEvents events;
    events::PhysicsAnswers answers;
    contact_constraints::ContactConstraints contact_constraints;
    world_settings::WorldSettings settings;
    uint32_t layer;
    collections::Vec<uint32_t> indices;
};

constexpr float CONTACT_SCENE_DELTA_TIME = 1.0f / 60.0f;

inline ContactScene make_contact_scene(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    ContactScene scene = {};
    scene.bodies = bodies::init();
    scene.layers = layers::create();
    scene.layer = layers::add_object_layer(scene.layers, layers::add_broadphase_layer(scene.layers));
    scene.pairs = pairs::create();
    scene.contacts = contacts::create();
    scene.events = events::create_events();
    scene.answers = events::create_answers();
    scene.contact_constraints = contact_constraints::create();
    scene.settings = world_settings::create();
    return scene;
}

inline void release_contact_scene(ContactScene &scene) {
    scene.indices.release();
    world_settings::release(scene.settings);
    contact_constraints::release(scene.contact_constraints);
    events::release(scene.answers);
    events::release(scene.events);
    contacts::release(scene.contacts);
    pairs::release(scene.pairs);
    bodies::release(scene.bodies);
    layers::release(scene.layers);
}

struct ContactBodySettings {
    float friction = 0.0f;
    float restitution = 0.0f;
    bool sensor = false;
};

/** a body with no damping; the shape is the body's to keep (the tests never free bodies one by one) */
inline rigid_body::RigidBody *add_contact_body(ContactScene &scene, shapes::Shape *shape, motion_type::MotionType motion_type, const Float3 &position,
                                               const ContactBodySettings &body_settings = {}) {
    rigid_body::RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = scene.layer;
    settings.motion_type = motion_type;
    settings.position = position;
    settings.linear_damping = 0.0f;
    settings.angular_damping = 0.0f;
    settings.friction = body_settings.friction;
    settings.restitution = body_settings.restitution;
    settings.sensor = body_settings.sensor;
    return rigid_body::create(scene.bodies, scene.layers, settings);
}

/** a manifold of world points (on a and on b alike, touching) with a unit normal from a to b */
inline manifold::ContactManifold touching_manifold(const Float3 *points, uint32_t num_points, const Float3 &normal, float penetration = 0.0f) {
    manifold::ContactManifold out = manifold::create();
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
    scene.contact_constraints.count = 0;
    scene.events.contacts_added.length = 0;
    scene.events.contacts_persisted.length = 0;
    scene.contacts.frame_stamp++;
    scene.bodies.step_stamp++;
}

/** the narrowphase's call for one manifold, through the pair's record (made on first use) */
inline bool add_contact(ContactScene &scene, rigid_body::RigidBody *body_a, rigid_body::RigidBody *body_b,
                        const manifold::ContactManifold &manifold) {
    int32_t record = pairs::find_pair_record(scene.pairs, *body_a, *body_b);
    if (record == pairs::INVALID_PAIR_KEY) record = pairs::add_pair_record(scene.pairs, *body_a, *body_b);
    return contact_constraints::add_contact_constraint(scene.contact_constraints, scene.contacts, scene.pairs, record, *body_a, *body_b,
                                                       manifold, scene.settings, scene.events, CONTACT_SCENE_DELTA_TIME,
                                                       scene.bodies.step_stamp);
}

/** every live constraint, in pool order */
inline const collections::Vec<uint32_t> &all_contact_indices(ContactScene &scene) {
    scene.indices.length = 0;
    for (uint32_t i = 0; i < scene.contact_constraints.count; i++) scene.indices.push(i);
    return scene.indices;
}

/** warm start, then `iterations` velocity iterations over every constraint */
inline void solve_contact_velocities(ContactScene &scene, uint32_t iterations) {
    const collections::Vec<uint32_t> &indices = all_contact_indices(scene);
    contact_constraints::warm_start_velocity_constraints(scene.contact_constraints, scene.bodies, indices, 1.0f);
    for (uint32_t i = 0; i < iterations; i++) {
        contact_constraints::solve_velocity_constraints_for_island(scene.contact_constraints, scene.bodies, indices);
    }
}

/** the impulses into the cache, which becomes the next step's read side */
inline void end_contact_step(ContactScene &scene) {
    contact_constraints::store_applied_impulses(scene.contact_constraints, scene.contacts);
    contacts::flip_manifold_cache(scene.contacts);
}
