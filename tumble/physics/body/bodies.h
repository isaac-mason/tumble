#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/body/body_activation.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/util/collections.h"
#include "tumble/util/ref.h"

TUMBLE_NAMESPACE_BEGIN

struct RigidBody;

struct Shape;

namespace bodies {

/** the sequence of a slot in `free_indices`, which no BodyId carries. */
constexpr uint32_t SLOT_FREE = 0xffffffffu;

}  // namespace bodies

struct Bodies {
    /** pool of bodies in the world by body index; a freed slot keeps its body for the next add to reuse */
    collections::Array<RigidBody *> pool;
    /** per slot: the sequence of the id that holds it, bodies::SLOT_FREE while the slot is free */
    collections::Array<uint32_t> sequences;
    /** per slot: the sequence its last body was given, which the next body to take it bumps */
    collections::Array<uint8_t> slot_sequences;
    /** pool of freed body indices for reuse */
    collections::Array<uint32_t> free_indices;
    /**
     * active body indices (body index values), maintained incrementally via swap-remove. its length
     * tracks the pool's, so the first active_body_count entries can always take one more.
     */
    collections::Array<uint32_t> active_body_indices;
    /** current count of active bodies */
    uint32_t active_body_count = 0;
    /**
     * incremented once at the start of every world step. keys the per-step memos on bodies
     * (motion_properties.world_inverse_inertia_stamp); never equals STEP_STAMP_NONE.
     */
    uint32_t step_stamp = 0;
    /** the empty shape removed bodies hold, so a free slot keeps no reference to a shape it was given. made on first remove */
    Ref<Shape> pooled_body_shape;
    /** told of every change to the active list (sleep::wake and sleep::sleep); null for none. the caller's */
    BodyActivationListener *activation_listener = nullptr;

    Bodies() = default;
    // the pool owns its bodies through plain pointers: a copy would free them twice
    Bodies(const Bodies &) = delete;
    Bodies &operator=(const Bodies &) = delete;
    /** frees every body record in the pool, removed or not */
    ~Bodies();
};

namespace bodies {

/**
 * claims a slot for a new body: the last freed index if there is one, else a new one at the end
 * whose pool entry is null. returns the slot's id with the slot's sequence bumped, or INVALID_BODY_ID
 * (Error::QUOTA) when the pool already holds body_id::MAX_BODIES.
 */
BodyId add_slot(Bodies &bodies);

/** frees the slot `id` holds, so it and every copy of it stop resolving. false if `id` is stale. */
bool remove_slot(Bodies &bodies, BodyId id);

/** whether `id` names a slot that is held, by the sequence it was handed out with. */
bool is_valid(const Bodies &bodies, BodyId id);

/** the body in the slot `id` names, or null if `id` is stale. */
RigidBody *get(const Bodies &bodies, BodyId id);

}  // namespace bodies

TUMBLE_NAMESPACE_END
