#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/constraints/constraint_id.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct World;

// a world's user constraints, a pool per type; every record starts with the base

enum class ConstraintSpace : uint32_t {
    /** points given in world space */
    WORLD = 0,
    /** points given relative to each body */
    LOCAL = 1,
};

/** the fields every constraint record begins with: `struct PointConstraint { ConstraintBase base; ... }` */
struct ConstraintBase {
    /** in the pool's free list, so skipped */
    bool pooled;
    /** both bodies asleep, set each step before the solve */
    bool sleeping;
    bool enabled;
    ConstraintId id;
    uint32_t index;
    uint32_t sequence;
    /** the place in Constraints::list, which orders the solve at equal priority, as jolt's constraint index; ~0u when not in it */
    uint32_t list_index;
    /** solved in ascending order, so the highest priority is solved last and is most likely to hold */
    uint32_t constraint_priority;
    /** 0 takes the world's iteration counts */
    uint32_t num_velocity_steps_override;
    uint32_t num_position_steps_override;
    uint64_t user_data;
    uint32_t body_index_a;
    uint32_t body_index_b;
};

namespace constraints {

inline ConstraintBase make_constraint_base() {
    ConstraintBase base = {};
    base.pooled = true;
    base.enabled = true;
    base.id = constraint_id::INVALID_CONSTRAINT_ID;
    base.index = ~0u;
    base.sequence = ~0u;
    base.list_index = ~0u;
    base.body_index_a = ~0u;
    base.body_index_b = ~0u;
    return base;
}

}  // namespace constraints

struct Constraints {
      // namespace constraints

    /** one type's records, `stride` bytes each, with a free list; constraints.cpp sets the stride from the type */
    struct Pool {
        ConstraintType type;
        uint32_t stride;
        collections::Array<uint8_t> storage;
        collections::Array<uint32_t> free_indices;
        /** records in storage, live and free */
        uint32_t count;
    };

    struct IterationOverrides {
        uint32_t velocity;
        uint32_t position;
    };

    struct SortFields {
        uint32_t priority;
        uint32_t index;
    };

    /** indexed by ConstraintType; a pool with stride 0 has never been used */
    Pool pools[constraint_id::CONSTRAINT_TYPE_COUNT];
    /** every live constraint, in creation order but for removal moving the last into the gap, as jolt's constraint manager */
    collections::Array<ConstraintId> list;
};

namespace constraints {

inline ConstraintBase *base_at(Constraints::Pool &pool, uint32_t index) {
    return reinterpret_cast<ConstraintBase *>(pool.storage.data + index * pool.stride);
}

inline const ConstraintBase *base_at(const Constraints::Pool &pool, uint32_t index) {
    return reinterpret_cast<const ConstraintBase *>(pool.storage.data + index * pool.stride);
}

/** the sequence for a constraint taking the record: the record's last sequence plus one, so an id its last constraint had stops naming the new one */
inline uint32_t next_sequence(const Constraints::Pool &pool, uint32_t index) {
    return (base_at(pool, index)->sequence + 1) & constraint_id::SEQUENCE_MASK;
}

}
namespace constraints {

Constraints create();

/** the type's pool, its stride set on first use */
Constraints::Pool &ensure_pool(Constraints &constraints, ConstraintType type);
/** a free record index of the pool, reusing a freed one or appending one whose sequence is 0; ~0u (Error::QUOTA) when the pool holds MAX_CONSTRAINT_INDEX + 1 records */
uint32_t allocate_index(Constraints::Pool &pool);
/** the constraint at the end of the world's list, in creation order */
void add_to_list(Constraints &constraints, ConstraintBase &constraint);
/** takes the constraint out of the list, the last one moving into its place */
void remove_from_list(Constraints &constraints, ConstraintBase &constraint);
/** the record of the id, or null when its sequence no longer matches */
ConstraintBase *get(Constraints &constraints, ConstraintId id);

/** frees a constraint of any type and unlinks it from both bodies; false when the id names no constraint */
bool remove(World &world, ConstraintId id);
/** frees every constraint of the body */
void destroy_body_constraints(Constraints &constraints, Bodies &bodies, RigidBody &body);

void get_constraint_iteration_overrides(Constraints::IterationOverrides &out, const Constraints &constraints, ConstraintId id);
void get_constraint_sort_fields(Constraints::SortFields &out, const Constraints &constraints, ConstraintId id);
/** by priority ascending, then index */
void sort_constraint_ids(const Constraints &constraints, collections::Array<ConstraintId> &ids);

/** marks each constraint sleeping when both its bodies are */
void update_sleeping(Constraints &constraints, const Bodies &bodies);
void setup_velocity_constraints(Constraints &constraints, Bodies &bodies, float delta_time);
/** an island's user constraints, in their solve order */
void warm_start_velocity_constraints(Constraints &constraints, Bodies &bodies, const collections::Array<ConstraintId> &ids, float warm_start_impulse_ratio);
/** true when any impulse was applied */
bool solve_velocity_constraints_for_island(Constraints &constraints, Bodies &bodies, const collections::Array<ConstraintId> &ids,
                                           float delta_time);
bool solve_position_constraints_for_island(Constraints &constraints, Bodies &bodies, const collections::Array<ConstraintId> &ids,
                                           float baumgarte_factor, float delta_time);

/** the body's shape centre of mass moved by delta_com in its local space: each of its constraints moves its point on the body to match */
void notify_shape_changed(Constraints &constraints, const RigidBody &body, Vec3f delta_com);

/** swap-removes the id from the body's list */
void remove_constraint_id_from_body(RigidBody &body, ConstraintId id);

}  // namespace constraints

TUMBLE_NAMESPACE_END
