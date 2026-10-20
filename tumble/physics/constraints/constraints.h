#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/constraints/constraint_id.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// a world's user constraints, a pool per type; every record starts with the base
namespace constraints {

using constraint_id::ConstraintId;
using constraint_id::ConstraintType;

enum ConstraintSpace : uint32_t {
    /** points given in world space */
    SPACE_WORLD = 0,
    /** points given relative to each body */
    SPACE_LOCAL = 1,
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
    /** solved in ascending order, so the highest priority is solved last and is most likely to hold */
    int32_t constraint_priority;
    /** 0 takes the world's iteration counts */
    uint32_t num_velocity_steps_override;
    uint32_t num_position_steps_override;
    uint64_t user_data;
    uint32_t body_index_a;
    uint32_t body_index_b;
};

inline ConstraintBase make_constraint_base() {
    ConstraintBase base = {};
    base.pooled = true;
    base.enabled = true;
    base.id = constraint_id::INVALID_CONSTRAINT_ID;
    base.index = ~0u;
    base.sequence = ~0u;
    base.body_index_a = ~0u;
    base.body_index_b = ~0u;
    return base;
}

/** one type's records, `stride` bytes each, with a free list; constraints.cpp sets the stride from the type */
struct ConstraintPool {
    ConstraintType type;
    uint32_t stride;
    collections::Vec<uint8_t> storage;
    collections::Vec<uint32_t> free_indices;
    /** records in storage, live and free */
    uint32_t count;
};

inline ConstraintBase *base_at(ConstraintPool &pool, uint32_t index) {
    return reinterpret_cast<ConstraintBase *>(pool.storage.data + index * pool.stride);
}

inline const ConstraintBase *base_at(const ConstraintPool &pool, uint32_t index) {
    return reinterpret_cast<const ConstraintBase *>(pool.storage.data + index * pool.stride);
}

/** the sequence for a constraint taking the record: the record's last sequence plus one, so an id its last constraint had stops naming the new one */
inline uint32_t next_sequence(const ConstraintPool &pool, uint32_t index) {
    return (base_at(pool, index)->sequence + 1) & constraint_id::SEQUENCE_MASK;
}

struct Constraints {
    /** indexed by ConstraintType; a pool with stride 0 has never been used */
    ConstraintPool pools[constraint_id::CONSTRAINT_TYPE_COUNT];
};

struct ConstraintIterationOverrides {
    uint32_t velocity;
    uint32_t position;
};

struct ConstraintSortFields {
    int32_t priority;
    uint32_t index;
};

Constraints create();
void release(Constraints &constraints);

/** the type's pool, its stride set on first use */
ConstraintPool &ensure_pool(Constraints &constraints, ConstraintType type);
/** a free record index of the pool, reusing a freed one or appending one whose sequence is 0; ~0u (error set) when the storage cannot grow or the pool holds MAX_CONSTRAINT_INDEX + 1 records */
uint32_t allocate_index(ConstraintPool &pool);
/** the record of the id, or null when its sequence no longer matches */
ConstraintBase *get(Constraints &constraints, ConstraintId id);

/** frees the constraint and unlinks it from both bodies */
void remove_constraint_by_id(Constraints &constraints, bodies::Bodies &bodies, ConstraintId id);
/** frees every constraint of the body */
void destroy_body_constraints(Constraints &constraints, bodies::Bodies &bodies, rigid_body::RigidBody &body);

void get_constraint_iteration_overrides(ConstraintIterationOverrides &out, const Constraints &constraints, ConstraintId id);
void get_constraint_sort_fields(ConstraintSortFields &out, const Constraints &constraints, ConstraintId id);
/** by priority ascending, then index */
void sort_constraint_ids(const Constraints &constraints, collections::Vec<ConstraintId> &ids);

/** marks each constraint sleeping when both its bodies are */
void update_sleeping(Constraints &constraints, const bodies::Bodies &bodies);
void setup_velocity_constraints(Constraints &constraints, bodies::Bodies &bodies, float delta_time);
/** an island's user constraints, in their solve order */
void warm_start_velocity_constraints(Constraints &constraints, bodies::Bodies &bodies, const collections::Vec<ConstraintId> &ids, float warm_start_impulse_ratio);
/** true when any impulse was applied */
bool solve_velocity_constraints_for_island(Constraints &constraints, bodies::Bodies &bodies, const collections::Vec<ConstraintId> &ids,
                                           float delta_time);
bool solve_position_constraints_for_island(Constraints &constraints, bodies::Bodies &bodies, const collections::Vec<ConstraintId> &ids,
                                           float baumgarte_factor, float delta_time);

/** the body's shape centre of mass moved by delta_com in its local space: each of its constraints moves its point on the body to match */
void notify_shape_changed(Constraints &constraints, const rigid_body::RigidBody &body, Vec3f delta_com);

/** swap-removes the id from the body's list */
void remove_constraint_id_from_body(rigid_body::RigidBody &body, ConstraintId id);

}  // namespace constraints

TUMBLE_NAMESPACE_END
