#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/broadphase/broadphase.h"
#include "tumble/physics/constraints/constraint_id.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/physics/constraints/contact_constraints.h"
#include "tumble/physics/contacts.h"
#include "tumble/physics/pairs.h"
#include "tumble/physics/world_settings.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// union-find over the active bodies through their contacts and constraints; the solver runs one island at a time

struct Islands {
    struct Island {
        collections::Array<uint32_t> body_indices;
        collections::Array<uint32_t> contact_indices;
        collections::Array<ConstraintId> constraint_ids;
        /** the solver iterations this island runs, the world's unless a constraint overrides */
        uint32_t num_velocity_steps;
        uint32_t num_position_steps;
    };

    /** union-find parent per active body (active index to active index) */
    collections::Array<uint32_t> body_links;
    /** island per active body after finalize */
    collections::Array<uint32_t> body_islands;
    /** contact index to the lowest active body index it links */
    collections::Array<uint32_t> contact_links;
    /** active constraint index to the lowest active body index it links */
    collections::Array<uint32_t> constraint_links;
    /** the active constraints in order, parallel to constraint_links */
    collections::Array<ConstraintId> constraint_ids;
    /** after finalize */
    collections::Array<Island> islands;
    /** after finalize: island indices, the most constraints first, the order the step solves and finishes them in */
    collections::Array<uint32_t> sorted;
    /** finalize's scratch for that order: each island's constraint count, and the merge's buffer */
    collections::Array<uint32_t> sort_counts;
    collections::Array<uint32_t> sort_scratch;
};

namespace islands {

/** the i-th island in the solve order */
inline Islands::Island &sorted_island(Islands &islands, uint32_t i) { return islands.islands[islands.sorted[i]]; }
inline const Islands::Island &sorted_island(const Islands &islands, uint32_t i) { return islands.islands[islands.sorted[i]]; }

Islands create();

/** sizes the links for this step's active bodies and contact constraints, every body its own root */
void prepare(Islands &islands, const Bodies &bodies, uint32_t num_contact_constraints);
void link_bodies(Islands &islands, const Bodies &bodies, uint32_t active_index_a, uint32_t active_index_b);
void link_contact(Islands &islands, uint32_t contact_index, const RigidBody &body_a, const RigidBody &body_b);
void link_constraint(Islands &islands, const Bodies &bodies, ConstraintId id, const RigidBody &body_a,
                     const RigidBody &body_b);
void link_contact_constraints(Islands &islands, const ContactConstraints &contact_constraints,
                              const Contacts &contacts, const Bodies &bodies);
void link_user_constraints(Islands &islands, const Constraints &constraints, const Bodies &bodies);
/** groups the roots into islands, each with its bodies, contacts, constraints and iteration counts */
void finalize(Islands &islands, const Bodies &bodies, const Constraints &constraints,
              const WorldSettings &settings);

/**
 * the end of an island's step (update.cpp): each body's position and bounds derived from its centre of mass and published
 * to the broadphase, its forces cleared and its sleep test run; when every body can sleep the island's dynamic bodies go
 * to sleep
 */
void finish_island_step(const Islands::Island &island, Bodies &bodies, Broadphase &broadphase, Pairs &pairs,
                        const WorldSettings &settings, float delta_time);

}  // namespace islands

TUMBLE_NAMESPACE_END
