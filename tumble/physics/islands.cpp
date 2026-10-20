#include "tumble/physics/islands.h"

#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/rigid_body_step.h"
#include "tumble/physics/body/sleep.h"
#include "tumble/util/assert.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

namespace islands {

using rigid_body::RigidBody;
using sleep::INACTIVE_BODY_INDEX;

Islands create() {
    Islands islands = {};
    return islands;
}

static void release_island(Island &island) {
    island.body_indices.release();
    island.contact_indices.release();
    island.constraint_ids.release();
}

void release(Islands &islands) {
    islands.body_links.release();
    islands.body_islands.release();
    islands.contact_links.release();
    islands.constraint_links.release();
    islands.constraint_ids.release();
    for (uint32_t i = 0; i < islands.islands.length; i++) release_island(islands.islands[i]);
    islands.islands.release();
}

void prepare(Islands &islands, const bodies::Bodies &bodies, uint32_t num_contact_constraints) {
    // body.island_index is assigned for every active body in finalize and cleared when a body leaves the active set
    // (sleep::remove_body_from_active_bodies), so there is no pool-wide reset here

    const uint32_t num_active_bodies = bodies.active_body_count;
    if (!islands.body_links.reserve(num_active_bodies) || !islands.body_islands.reserve(num_active_bodies) ||
        !islands.contact_links.reserve(num_contact_constraints)) {
        return;
    }
    islands.body_links.length = num_active_bodies;
    islands.body_islands.length = num_active_bodies;

    // contact_links is indexed by contact constraint index, every entry unlinked until link_contact
    islands.contact_links.length = num_contact_constraints;
    for (uint32_t i = 0; i < num_contact_constraints; i++) islands.contact_links[i] = INACTIVE_BODY_INDEX;

    // each body starts pointing to itself (its own island)
    for (uint32_t i = 0; i < num_active_bodies; i++) {
        islands.body_links[i] = i;
        islands.body_islands[i] = 0;
    }

    // filled by link_constraint
    islands.constraint_links.length = 0;
    islands.constraint_ids.length = 0;
}

/** the lowest body index in an island chain; follows the chain without path compression, which link_bodies does */
static uint32_t get_lowest_body_index(const Islands &islands, uint32_t body_index) {
    uint32_t index = body_index;
    while (islands.body_links[index] != index) index = islands.body_links[index];
    return index;
}

void link_bodies(Islands &islands, const bodies::Bodies &bodies, uint32_t active_index_a, uint32_t active_index_b) {
    // both need to be active, we don't want to create an island with static objects
    const uint32_t max_active_bodies = bodies.active_body_count;
    if (active_index_a >= max_active_bodies || active_index_b >= max_active_bodies) return;

    // follow the chains until we get to the bodies with the lowest index
    const uint32_t first_link_to = get_lowest_body_index(islands, active_index_a);
    const uint32_t second_link_to = get_lowest_body_index(islands, active_index_b);

    // if the targets are the same, the bodies are already connected
    if (first_link_to == second_link_to) return;

    // we always link the highest to the lowest
    if (first_link_to < second_link_to) {
        islands.body_links[second_link_to] = first_link_to;
    } else {
        islands.body_links[first_link_to] = second_link_to;
    }

    // path compression: chains of bodies can become really long, an O(n) loop. point the bodies that were passed in
    // straight at the lowest index found (only if lower than their current link)
    const uint32_t lowest_link_to = first_link_to < second_link_to ? first_link_to : second_link_to;
    if (islands.body_links[active_index_a] > lowest_link_to) islands.body_links[active_index_a] = lowest_link_to;
    if (islands.body_links[active_index_b] > lowest_link_to) islands.body_links[active_index_b] = lowest_link_to;
}

void link_contact(Islands &islands, uint32_t contact_index, const RigidBody &body_a, const RigidBody &body_b) {
    // the minimum active index: the smaller when both are active, the active one when one is (INACTIVE_BODY_INDEX is
    // the max value), INACTIVE_BODY_INDEX when neither is. no union here: that is link_bodies'
    islands.contact_links[contact_index] = body_a.active_index < body_b.active_index ? body_a.active_index : body_b.active_index;
}

void link_constraint(Islands &islands, const bodies::Bodies &bodies, ConstraintId id, const RigidBody &body_a, const RigidBody &body_b) {
    // link the bodies together (if both active)
    if (body_a.active_index != INACTIVE_BODY_INDEX && body_b.active_index != INACTIVE_BODY_INDEX) {
        link_bodies(islands, bodies, body_a.active_index, body_b.active_index);
    }

    // the id and the minimum active body index, the two arrays grow together
    islands.constraint_ids.push(id);
    islands.constraint_links.push(body_a.active_index < body_b.active_index ? body_a.active_index : body_b.active_index);
}

void link_contact_constraints(Islands &islands, const contact_constraints::ContactConstraints &contact_constraints,
                              const contacts::Contacts &contacts, const bodies::Bodies &bodies) {
    for (uint32_t constraint_index = 0; constraint_index < contact_constraints.count; constraint_index++) {
        const contact_constraints::ContactConstraint &constraint = contact_constraints.pool[constraint_index];
        const contacts::Contact &contact = contacts.contacts[(uint32_t)constraint.contact_index];
        const RigidBody &body_a = *bodies.pool[contact.body_index_a];
        const RigidBody &body_b = *bodies.pool[contact.body_index_b];

        // link the bodies if both are active (dynamic or kinematic)
        if (body_a.active_index != INACTIVE_BODY_INDEX && body_b.active_index != INACTIVE_BODY_INDEX) {
            link_bodies(islands, bodies, body_a.active_index, body_b.active_index);
        }

        link_contact(islands, constraint_index, body_a, body_b);
    }
}

void link_user_constraints(Islands &islands, const constraints::Constraints &constraints, const bodies::Bodies &bodies) {
    for (uint32_t type = 0; type < constraint_id::CONSTRAINT_TYPE_COUNT; type++) {
        const constraints::ConstraintPool &pool = constraints.pools[type];
        for (uint32_t i = 0; i < pool.count; i++) {
            const constraints::ConstraintBase *constraint = constraints::base_at(pool, i);
            if (constraint->pooled || !constraint->enabled) continue;
            const RigidBody &body_a = *bodies.pool[constraint->body_index_a];
            const RigidBody &body_b = *bodies.pool[constraint->body_index_b];

            // only link if at least one body is active
            if (body_a.active_index != INACTIVE_BODY_INDEX || body_b.active_index != INACTIVE_BODY_INDEX) {
                link_constraint(islands, bodies, constraint->id, body_a, body_b);
            }
        }
    }
}

static uint32_t island_constraint_count(const Island &island) { return island.contact_indices.length + island.constraint_ids.length; }

/** islands by constraint count, largest first, stable */
static void sort_islands_by_size(collections::Vec<Island> &islands) {
    sort::merge_sort(islands.data, islands.data + islands.length,
                     [](const Island &a, const Island &b) { return island_constraint_count(a) > island_constraint_count(b); });
}

void finalize(Islands &islands, const bodies::Bodies &bodies, const constraints::Constraints &constraints,
              const world_settings::WorldSettings &settings) {
    const uint32_t num_active_bodies = bodies.active_body_count;

    // an island index for every active body from the union-find
    uint32_t num_islands = 0;
    for (uint32_t i = 0; i < num_active_bodies; i++) {
        const uint32_t link_to = islands.body_links[i];
        if (link_to != i) {
            // links to another body: take its island (filled in already, as we loop from low to high)
            CC_ASSERT(link_to < i);
            islands.body_islands[i] = islands.body_islands[link_to];
        } else {
            // links to no other body: the start of a new island
            islands.body_islands[i] = num_islands;
            num_islands++;
        }
    }

    // island records and their arrays are reused across steps; the ones past this step's count give their memory back,
    // so every slot past the length is unowned
    collections::Vec<Island> &list = islands.islands;
    for (uint32_t i = num_islands; i < list.length; i++) release_island(list[i]);
    if (num_islands > list.length) {
        if (!list.reserve(num_islands)) return;
        for (uint32_t i = list.length; i < num_islands; i++) list.data[i] = Island{};
    }
    list.length = num_islands;
    for (uint32_t i = 0; i < num_islands; i++) {
        Island &island = list[i];
        island.index = i;
        island.body_indices.length = 0;
        island.contact_indices.length = 0;
        island.constraint_ids.length = 0;
        island.num_velocity_steps = 0;
        island.num_position_steps = 0;
    }

    // bodies by island, through the active list (which never holds a pooled slot)
    for (uint32_t active_index = 0; active_index < num_active_bodies; active_index++) {
        const uint32_t island_index = islands.body_islands[active_index];
        CC_ASSERT(island_index < num_islands);
        list[island_index].body_indices.push(active_index);
    }

    // contacts by the island of the active body they link to
    for (uint32_t contact_index = 0; contact_index < islands.contact_links.length; contact_index++) {
        const uint32_t active_body_index = islands.contact_links[contact_index];

        // both bodies static, or the contact never linked
        if (active_body_index == INACTIVE_BODY_INDEX) continue;
        CC_ASSERT(active_body_index < num_active_bodies);

        list[islands.body_islands[active_body_index]].contact_indices.push(contact_index);
    }

    // constraints likewise; the island keeps the id, which carries the type, index and sequence
    for (uint32_t active_constraint_index = 0; active_constraint_index < islands.constraint_ids.length; active_constraint_index++) {
        const uint32_t active_body_index = islands.constraint_links[active_constraint_index];
        if (active_body_index == INACTIVE_BODY_INDEX) continue;
        CC_ASSERT(active_body_index < num_active_bodies);

        list[islands.body_islands[active_body_index]].constraint_ids.push(islands.constraint_ids[active_constraint_index]);
    }

    // the biggest islands first (jolt: the jobs that take longest run first for better load balancing)
    sort_islands_by_size(list);

    // the sorted index on each island and on each of its bodies
    for (uint32_t i = 0; i < list.length; i++) {
        Island &island = list[i];
        island.index = i;
        for (uint32_t b = 0; b < island.body_indices.length; b++) {
            bodies.pool[bodies.active_body_indices[island.body_indices[b]]]->island_index = (int32_t)i;
        }
    }

    // each island's solver iterations: the max of its dynamic bodies' and constraints' overrides, and the world's
    // default when any of them asks for it with 0
    for (uint32_t i = 0; i < list.length; i++) {
        Island &island = list[i];
        uint32_t num_velocity_steps = 0;
        uint32_t num_position_steps = 0;
        bool apply_default_velocity = false;
        bool apply_default_position = false;

        for (uint32_t b = 0; b < island.body_indices.length; b++) {
            const RigidBody &body = *bodies.pool[bodies.active_body_indices[island.body_indices[b]]];
            if (body.motion_type != motion_type::MOTION_DYNAMIC) continue;
            const uint32_t velocity_override = body.motion_properties.num_velocity_steps_override;
            const uint32_t position_override = body.motion_properties.num_position_steps_override;
            if (velocity_override > num_velocity_steps) num_velocity_steps = velocity_override;
            if (velocity_override == 0) apply_default_velocity = true;
            if (position_override > num_position_steps) num_position_steps = position_override;
            if (position_override == 0) apply_default_position = true;
        }

        for (uint32_t c = 0; c < island.constraint_ids.length; c++) {
            constraints::ConstraintIterationOverrides overrides;
            constraints::get_constraint_iteration_overrides(overrides, constraints, island.constraint_ids[c]);
            if (overrides.velocity > num_velocity_steps) num_velocity_steps = overrides.velocity;
            if (overrides.velocity == 0) apply_default_velocity = true;
            if (overrides.position > num_position_steps) num_position_steps = overrides.position;
            if (overrides.position == 0) apply_default_position = true;
        }

        if (apply_default_velocity && settings.solver.velocity_iterations > num_velocity_steps) {
            num_velocity_steps = settings.solver.velocity_iterations;
        }
        if (apply_default_position && settings.solver.position_iterations > num_position_steps) {
            num_position_steps = settings.solver.position_iterations;
        }

        island.num_velocity_steps = num_velocity_steps;
        island.num_position_steps = num_position_steps;
    }
}

void finish_island_step(const Island &island, bodies::Bodies &bodies, broadphase::Broadphase &broadphase, pairs::Pairs &pairs,
                        const world_settings::WorldSettings &settings, float delta_time, events::PhysicsEvents &events) {
    const bool allow_sleeping = settings.sleeping.allow_sleeping;
    const float time_before_sleep = settings.sleeping.time_before_sleep;
    const float max_movement = settings.sleeping.point_velocity_sleep_threshold * time_before_sleep;

    bool all_can_sleep = allow_sleeping;

    // sleeping bodies are left out of islands when they are built
    for (uint32_t i = 0; i < island.body_indices.length; i++) {
        rigid_body::RigidBody *body = bodies.pool[bodies.active_body_indices[island.body_indices[i]]];

        rigid_body_step::derive_transform(body);
        if (broadphase::update_body(broadphase, body)) pairs::mark_moved(pairs, *body);
        rigid_body::clear_forces(body);
        events.moved.push(body->index);

        if (allow_sleeping && body->motion_type == motion_type::MOTION_DYNAMIC) {
            if (!sleep::update_sleep_state(body, delta_time, max_movement, time_before_sleep)) all_can_sleep = false;
        }
    }

    // every body can sleep: the island sleeps. sleep swap-removes from the active list the island's indices point into,
    // which holds because a slot is only overwritten when the body living in it is the one being slept, and islands own
    // disjoint slots: no island resolves a slot another has clobbered, and here a slot is clobbered only after its own pass
    if (all_can_sleep) {
        for (uint32_t i = 0; i < island.body_indices.length; i++) {
            rigid_body::RigidBody *body = bodies.pool[bodies.active_body_indices[island.body_indices[i]]];
            if (body->motion_type == motion_type::MOTION_DYNAMIC) rigid_body::sleep(bodies, body);
        }
    }
}

}  // namespace islands

TUMBLE_NAMESPACE_END
