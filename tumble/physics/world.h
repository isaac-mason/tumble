#pragma once

#include <stdint.h>

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
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace world {

struct World {
    /** owned: the layers are released with the world */
    world_settings::WorldSettings settings;
    broadphase::Broadphase broadphase;
    pairs::Pairs pairs;
    bodies::Bodies bodies;
    contact_constraints::ContactConstraints contact_constraints;
    constraints::Constraints constraints;
    contacts::Contacts contacts;
    /**
     * the contact pairs a user observes, keyed by their sides, not by the contacts above, whose identities are not those
     * pairs: a ccd contact reports a contact the narrowphase already reported this step, and a pair is added or persisted
     * by whether it was there last step
     */
    contact_pairs::ContactPairs contact_pairs;
    islands::Islands islands;
    ccd::CCD ccd;
    events::PhysicsEvents events;
    events::PhysicsAnswers answers;
    /** the step in flight, set by update::step_begin */
    float delta_time;
    /** the step before's, for the warm start ratio */
    float previous_time_step;
};

/** a world over a copy of the settings, which stay the caller's to release or reuse; null (error set) when the allocator fails */
World *create(const world_settings::WorldSettings &settings);
/** frees every body, constraint, contact and buffer, then the world */
void destroy(World *world);

/**
 * the surfaces material ids name: a shape's or a triangle's material id picks materials[id] for its side of a contact,
 * in place of the body's friction and restitution; an id past the end, or -1, takes the body's. replaces the table;
 * false (error set, the table empty) when the allocator fails
 */
bool set_materials(World &world, const contact_constraints::Material *materials, uint32_t count);

/**
 * one step answering nothing: update.h's phases in order, finding pairs again while the narrowphase wakes bodies.
 * a caller that rejects pairs or overrides contacts runs the phases itself
 */
void step(World &world, float delta_time);

}  // namespace world

TUMBLE_NAMESPACE_END
