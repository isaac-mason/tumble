#pragma once

// crashcat's islands.ts, the types: union-find over the active bodies through their contacts and
// constraints, finalised into islands the solver runs one at a time. the building is islands.cpp.

#include <stdint.h>

#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/broadphase/broadphase.h"
#include "tumble/physics/constraints/constraint_id.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/physics/constraints/contact_constraints.h"
#include "tumble/physics/contacts.h"
#include "tumble/physics/events.h"
#include "tumble/physics/pairs.h"
#include "tumble/physics/world_settings.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

namespace islands {

using constraint_id::ConstraintId;

struct Island {
    uint32_t index;
    collections::Vec<uint32_t> body_indices;
    collections::Vec<uint32_t> contact_indices;
    collections::Vec<ConstraintId> constraint_ids;
    /** the solver iterations this island runs, the world's unless a constraint overrides */
    uint32_t num_velocity_steps;
    uint32_t num_position_steps;
};

struct Islands {
    /** union-find parent per active body (active index to active index) */
    collections::Vec<uint32_t> body_links;
    /** island per active body after finalize */
    collections::Vec<uint32_t> body_islands;
    /** contact index to the lowest active body index it links */
    collections::Vec<uint32_t> contact_links;
    /** active constraint index to the lowest active body index it links */
    collections::Vec<uint32_t> constraint_links;
    /** the active constraints in order, parallel to constraint_links */
    collections::Vec<ConstraintId> constraint_ids;
    /** after finalize */
    collections::Vec<Island> islands;
};

Islands create();
void release(Islands &islands);

/** sizes the links for this step's active bodies and contact constraints, every body its own root */
void prepare(Islands &islands, const bodies::Bodies &bodies, uint32_t num_contact_constraints);
void link_bodies(Islands &islands, const bodies::Bodies &bodies, uint32_t active_index_a, uint32_t active_index_b);
void link_contact(Islands &islands, uint32_t contact_index, const rigid_body::RigidBody &body_a, const rigid_body::RigidBody &body_b);
void link_constraint(Islands &islands, const bodies::Bodies &bodies, ConstraintId id, const rigid_body::RigidBody &body_a,
                     const rigid_body::RigidBody &body_b);
void link_contact_constraints(Islands &islands, const contact_constraints::ContactConstraints &contact_constraints,
                              const contacts::Contacts &contacts, const bodies::Bodies &bodies);
void link_user_constraints(Islands &islands, const constraints::Constraints &constraints, const bodies::Bodies &bodies);
/** groups the roots into islands, each with its bodies, contacts, constraints and iteration counts */
void finalize(Islands &islands, const bodies::Bodies &bodies, const constraints::Constraints &constraints,
              const world_settings::WorldSettings &settings);

/**
 * the end of an island's step (update.cpp): each body's position and bounds derived from its centre of mass and published
 * to the broadphase, its forces cleared, its index appended to events.moved and its sleep test run; when every body can
 * sleep the island's dynamic bodies go to sleep
 */
void finish_island_step(const Island &island, bodies::Bodies &bodies, broadphase::Broadphase &broadphase, pairs::Pairs &pairs,
                        const world_settings::WorldSettings &settings, float delta_time, events::PhysicsEvents &events);

}  // namespace islands

TUMBLE_NAMESPACE_END
