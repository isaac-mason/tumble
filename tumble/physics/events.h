#pragma once

// the plan's replacement for crashcat's listener: the step is exported as phases, and between them
// js reads what the core found (event buffers, appended while a phase runs, read after it, cleared by
// the next phase that fills them; they grow through the heap and never drop an entry) and writes its
// answers (answer buffers, one entry per event, read by the next phase). the step fills and clears
// them in update.cpp; this header is the records.

#include <stdint.h>

#include "math/types.h"
#include "physics/body/body_id.h"
#include "physics/body/sub_shape.h"
#include "physics/constraints/contact_settings.h"
#include "physics/manifold/manifold.h"
#include "util/collections.h"

namespace events {

/** a pair the broadphase found this step (was onBodyPairValidate); js may reject it before the narrowphase */
struct PairEvent {
    uint32_t body_index_a;
    uint32_t body_index_b;
    body_id::BodyId body_id_a;
    body_id::BodyId body_id_b;
    /** the bodies' user data */
    uint64_t user_data_a;
    uint64_t user_data_b;
    /** the persistent pair record, stable for the step */
    int32_t pair_record;
};

/** a contact the narrowphase added or kept this step (was onContactAdded / onContactPersisted), with its manifold */
struct ContactEvent {
    int32_t contact_index;
    uint32_t body_index_a;
    uint32_t body_index_b;
    body_id::BodyId body_id_a;
    body_id::BodyId body_id_b;
    uint64_t user_data_a;
    uint64_t user_data_b;
    sub_shape::SubShapeId sub_shape_id_a;
    sub_shape::SubShapeId sub_shape_id_b;
    int32_t material_id_a;
    int32_t material_id_b;
    /** unit, world space, from a to b */
    Float3 normal;
    float penetration;
    uint32_t num_points;
    /** world space */
    Float3 points_a[manifold::MAX_CONTACT_POINTS];
    Float3 points_b[manifold::MAX_CONTACT_POINTS];
    /** what the constraint was built from (the combined material, no scales, no surface velocity): the starting point of an override */
    contact_constraints::ContactSettings settings;
};

/** a contact that was there last step and is gone (was onContactRemoved); the bodies may be gone too */
struct ContactRemovedEvent {
    body_id::BodyId body_id_a;
    body_id::BodyId body_id_b;
    sub_shape::SubShapeId sub_shape_id_a;
    sub_shape::SubShapeId sub_shape_id_b;
};

struct PhysicsEvents {
    /** physics_find_pairs */
    collections::Vec<PairEvent> pairs;
    /** physics_narrowphase */
    collections::Vec<ContactEvent> contacts_added;
    collections::Vec<ContactEvent> contacts_persisted;
    /** physics_step_end: removals queued during the step and the one before (jolt reports a removed body's contacts the update after) */
    collections::Vec<ContactRemovedEvent> contacts_removed;
    /** physics_step_end: the indices of the bodies the step moved, for js to drain into its tuples */
    collections::Vec<uint32_t> moved;
    /**
     * physics_narrowphase: the bodies its contacts woke, whose pairs are not found yet. while it is above zero js calls
     * physics_find_pairs, answers the pairs it appended, and calls physics_narrowphase again
     */
    uint32_t woken;
};

/** js's answer to one contacts_added or contacts_persisted entry; `written` says js touched it at all */
struct ContactOverride {
    bool written;
    /** drop the contact before the solve */
    bool reject;
    contact_constraints::ContactSettings settings;
};

struct PhysicsAnswers {
    /** one bit per pairs entry */
    collections::Vec<uint32_t> pair_reject;
    /** one per contacts_added entry, then one per contacts_persisted entry */
    collections::Vec<ContactOverride> contact_override;
};

inline PhysicsEvents create_events() {
    PhysicsEvents e = {};
    return e;
}

inline void release(PhysicsEvents &e) {
    e.pairs.release();
    e.contacts_added.release();
    e.contacts_persisted.release();
    e.contacts_removed.release();
    e.moved.release();
    e.woken = 0;
}

inline PhysicsAnswers create_answers() {
    PhysicsAnswers a = {};
    return a;
}

inline void release(PhysicsAnswers &a) {
    a.pair_reject.release();
    a.contact_override.release();
}

inline bool pair_rejected(const PhysicsAnswers &a, uint32_t pair) {
    const uint32_t word = pair >> 5;
    return word < a.pair_reject.length && (a.pair_reject[word] >> (pair & 31)) & 1u;
}

/**
 * one accept bit per pairs entry: the words the pairs appended since `first_pair` start clear. a word the earlier pairs
 * share is clear past them already, as js sets bits only below the count
 */
inline bool reset_pair_answers(PhysicsAnswers &a, const PhysicsEvents &e, uint32_t first_pair) {
    const uint32_t words = (e.pairs.length + 31) >> 5;
    if (!a.pair_reject.reserve(words)) return false;
    for (uint32_t i = (first_pair + 31) >> 5; i < words; i++) a.pair_reject.data[i] = 0;
    a.pair_reject.length = words;
    return true;
}

/**
 * one override per contact event, added then persisted: the ones the narrowphase appended since `first_override` start
 * unwritten. js writes them after the step's last narrowphase, so the earlier ones are unwritten still
 */
inline bool reset_contact_answers(PhysicsAnswers &a, const PhysicsEvents &e, uint32_t first_override) {
    const uint32_t overrides = e.contacts_added.length + e.contacts_persisted.length;
    if (!a.contact_override.reserve(overrides)) return false;
    for (uint32_t i = first_override; i < overrides; i++) a.contact_override.data[i].written = false;
    a.contact_override.length = overrides;
    return true;
}

}  // namespace events
