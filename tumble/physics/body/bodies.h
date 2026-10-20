#pragma once

// crashcat's bodies.ts: the world's pool of bodies, its active list and its step stamp. a body
// index is a slot in `pool`; the slot's sequence is what makes a stale BodyId stop resolving.

#include <stdint.h>

#include "tumble/physics/body/body_id.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

namespace shapes {
struct Shape;
}

namespace bodies {

/** the sequence of a slot in `free_indices`, which no BodyId carries. */
constexpr uint32_t SLOT_FREE = 0xffffffffu;

struct Bodies {
    /** pool of bodies in the world by body index; a freed slot keeps its body for the next add to reuse */
    collections::Vec<rigid_body::RigidBody *> pool;
    /** per slot: the sequence of the id that holds it, SLOT_FREE while the slot is free */
    collections::Vec<uint32_t> sequences;
    /** per slot: the sequence its last body was given, which the next body to take it bumps (jolt's mBodySequenceNumbers) */
    collections::Vec<uint8_t> slot_sequences;
    /** pool of freed body indices for reuse */
    collections::Vec<uint32_t> free_indices;
    /**
     * active body indices (body index values), maintained incrementally via swap-remove. its length
     * tracks the pool's, so the first active_body_count entries can always take one more.
     */
    collections::Vec<uint32_t> active_body_indices;
    /** current count of active bodies */
    uint32_t active_body_count;
    /**
     * incremented once at the start of every world step. keys the per-step memos on bodies
     * (motion_properties.world_inverse_inertia_stamp); never equals STEP_STAMP_NONE.
     */
    uint32_t step_stamp;
    /** the empty shape removed bodies hold, so a free slot keeps no reference to a shape it was given. made on first remove */
    shapes::Shape *pooled_body_shape;
};

Bodies init();
/** frees the pool, every body record in it, removed or not, and the shape removed bodies hold. */
void release(Bodies &bodies);

/**
 * claims a slot for a new body: the last freed index if there is one, else a new one at the end
 * whose pool entry is null. returns the slot's id with the slot's sequence bumped, or INVALID_BODY_ID
 * (error set) when the pool cannot grow or already holds body_id::MAX_BODIES.
 */
body_id::BodyId add_slot(Bodies &bodies);

/** frees the slot `id` holds, so it and every copy of it stop resolving. false if `id` is stale. */
bool remove_slot(Bodies &bodies, body_id::BodyId id);

/** whether `id` names a slot that is held, by the sequence it was handed out with. */
bool is_valid(const Bodies &bodies, body_id::BodyId id);

/** the body in the slot `id` names, or null if `id` is stale. */
rigid_body::RigidBody *get(const Bodies &bodies, body_id::BodyId id);

}  // namespace bodies

TUMBLE_NAMESPACE_END
