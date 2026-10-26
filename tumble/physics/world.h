#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/broadphase/broadphase.h"
#include "tumble/physics/ccd.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/physics/constraints/contact_constraints.h"
#include "tumble/physics/contact_pairs.h"
#include "tumble/physics/contacts.h"
#include "tumble/physics/events.h"
#include "tumble/physics/islands.h"
#include "tumble/physics/pairs.h"
#include "tumble/physics/world_settings.h"

TUMBLE_NAMESPACE_BEGIN

struct World {
    /** owned: the layers are released with the world */
    WorldSettings settings;
    Broadphase broadphase;
    Pairs pairs;
    Bodies bodies;
    ContactConstraints contact_constraints;
    Constraints constraints;
    Contacts contacts;
    /**
     * the contact pairs a user observes, keyed by their sides, not by the contacts above, whose identities are not those
     * pairs: a ccd contact reports a contact the narrowphase already reported this step, and a pair is added or persisted
     * by whether it was there last step
     */
    ContactPairs contact_pairs;
    Islands islands;
    CCD ccd;
    WorldEvents events;
    WorldAnswers answers;
    /** the step in flight, set by update::step_begin */
    float delta_time;
    /** the step before's, for the warm start ratio */
    float previous_time_step;
};

namespace world {

/** a world over a copy of the settings, which stay the caller's */
World *create(const WorldSettings &settings);
/** frees every body, constraint, contact and buffer, then the world */
void destroy(World *world);

/**
 * the surfaces material ids name: a shape's or a triangle's material id picks materials[id] for its side of a contact,
 * in place of the body's friction and restitution; an id past the end, or -1, takes the body's. replaces the table
 */
void set_materials(World &world, const WorldMaterial *materials, uint32_t count);

/**
 * one step answering nothing: update.h's phases in order, finding pairs again while the narrowphase wakes bodies.
 * a caller that rejects pairs or overrides contacts runs the phases itself
 */
void step(World &world, float delta_time);

/** how many bodies are on the active list: the dynamic and kinematic bodies awake, which the next step simulates */
inline uint32_t get_num_active_bodies(const World &world) { return world.bodies.active_body_count; }

/**
 * the active list, get_num_active_bodies long: body indices into bodies.pool, in no particular order. valid until the
 * list next changes, which a step, a body added or removed, or a body woken or put to sleep can do
 */
inline const uint32_t *get_active_bodies(const World &world) { return world.bodies.active_body_indices.data; }

/** told of every body joining or leaving the active list from now on; null for none. the listener stays the caller's */
inline void set_body_activation_listener(World &world, BodyActivationListener *listener) {
    world.bodies.activation_listener = listener;
}

}  // namespace world

TUMBLE_NAMESPACE_END
