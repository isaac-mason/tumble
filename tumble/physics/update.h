#pragma once

// crashcat's update.ts as the plan's five phases. updateWorld is one function in crashcat because js
// calling js is free; here js calls the phases in order every tick and, between them, reads the
// world's event buffers and writes its answer buffers (events.h). update.cpp.

#include "physics/world.h"

namespace update {

/**
 * flushes the removals queued last step into contacts_removed, clears the ccd state, resets the constraints,
 * advances the stamps, integrates accelerations and optimises the broadphase. clears every event buffer.
 */
void step_begin(world::World &world, float delta_time);

/**
 * pairs::find_colliding_pairs: fills events.pairs, and sizes answers.pair_reject for js to set. called again after a
 * narrowphase that woke bodies (events.woken above zero), it appends only those bodies' pairs (jolt finds a newly
 * activated body's pairs before its narrowphase ends), so a sleeping stack hit from above wakes and solves whole in one
 * step
 */
void find_pairs(world::World &world);

/**
 * the pairs find_pairs added since the last narrowphase, those js did not reject: the pair cache or gjk and epa,
 * manifolds, contact constraints; appends to contacts_added and contacts_persisted and sizes answers.contact_override.
 * sets events.woken to the bodies its contacts woke: js then calls find_pairs and narrowphase again, and writes the
 * overrides after the last
 */
void narrowphase(world::World &world);

/**
 * applies the overrides (a rejected contact's constraint is dropped), wakes the bodies of user constraints,
 * integrates velocities, runs ccd, builds the islands, solves them and updates sleep
 */
void solve(world::World &world);

/** resets forces and fills events.moved; contacts_removed holds what the step dropped */
void step_end(world::World &world);

}  // namespace update
