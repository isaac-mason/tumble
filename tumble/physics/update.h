#pragma once

#include "tumble/config.h"
#include "tumble/physics/world.h"

TUMBLE_NAMESPACE_BEGIN

// a step is these phases in order. between them the caller reads the world's events, and may reject or change them (events.h)

namespace update {

/**
 * flushes the removals queued last step into contacts_removed, clears the ccd state, resets the constraints,
 * advances the stamps, integrates accelerations and optimises the broadphase. clears every event buffer
 */
void step_begin(World &world, float delta_time);

/**
 * fills events.pairs, for the caller to reject any before the narrowphase. run again after a narrowphase that woke
 * bodies (events.woken above zero), it appends only those bodies' pairs, so a sleeping stack hit from above wakes and
 * solves whole in one step
 */
void find_pairs(World &world);

/**
 * the pairs find_pairs added since the last narrowphase that the caller did not reject: the pair cache or gjk and
 * epa, manifolds, contact constraints. appends to contacts_added and contacts_persisted. sets events.woken to the
 * bodies its contacts woke: the caller then runs find_pairs and narrowphase again, and rejects or changes contacts after
 * the last
 */
void narrowphase(World &world);

/**
 * applies the caller's rejected and changed contacts, wakes the bodies of user constraints,
 * integrates velocities, runs ccd, builds the islands, solves them and updates sleep
 */
void solve(World &world);

/** resets forces; contacts_removed holds what the step dropped */
void step_end(World &world);

}  // namespace update

TUMBLE_NAMESPACE_END
