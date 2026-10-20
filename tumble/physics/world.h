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
    /** the step in flight, set by physics_step_begin */
    float delta_time;
    /** the step before's, for the warm start ratio */
    float previous_time_step;
};

/** a world over the settings, which it takes (the layers move in); null when the allocator fails */
World *create(const world_settings::WorldSettings &settings);
/** frees every body, constraint, contact and buffer, then the world */
void destroy(World *world);

}  // namespace world

TUMBLE_NAMESPACE_END
