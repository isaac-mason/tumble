#pragma once

// crashcat's world.ts: a physics world, the parts crashcat gives it plus the plan's event and answer
// buffers and the step in flight. created and freed through the heap; the step is update.h.

#include <stdint.h>

#include "physics/body/bodies.h"
#include "physics/broadphase/broadphase.h"
#include "physics/ccd.h"
#include "physics/constraints/constraints.h"
#include "physics/constraints/contact_constraints.h"
#include "physics/contact_pairs.h"
#include "physics/contacts.h"
#include "physics/events.h"
#include "physics/islands.h"
#include "physics/pairs.h"
#include "physics/world_settings.h"

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

/** a world over the settings, which it takes (the layers move in); null when the memory cannot grow */
World *create(const world_settings::WorldSettings &settings);
/** frees every body, constraint, contact and buffer, then the world */
void destroy(World *world);

}  // namespace world
