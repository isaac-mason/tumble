#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/constraints/contact_settings.h"
#include "tumble/physics/manifold/manifold.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// instead of listener callbacks: a phase of the step (update.h) appends events, the caller reads them before the next
// phase and writes answers (one per event) that the next phase reads. the buffers never drop an entry

/** a pair the broadphase found this step; the caller may reject it before the narrowphase */
struct PairEvent {
    uint32_t body_index_a;
    uint32_t body_index_b;
    BodyId body_id_a;
    BodyId body_id_b;
    uint64_t user_data_a;
    uint64_t user_data_b;
    /** the persistent pair record, stable for the step */
    int32_t pair_record;
};

/** a contact the narrowphase added or kept this step, with its manifold */
struct ContactEvent {
    int32_t contact_index;
    uint32_t body_index_a;
    uint32_t body_index_b;
    BodyId body_id_a;
    BodyId body_id_b;
    uint64_t user_data_a;
    uint64_t user_data_b;
    SubShapeId sub_shape_id_a;
    SubShapeId sub_shape_id_b;
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
    ContactSettings settings;
};

/** a contact that was there last step and is gone; the bodies may be gone too */
struct ContactRemovedEvent {
    BodyId body_id_a;
    BodyId body_id_b;
    SubShapeId sub_shape_id_a;
    SubShapeId sub_shape_id_b;
};

struct WorldEvents {
    /** by find_pairs */
    collections::Array<PairEvent> pairs;
    /** by narrowphase */
    collections::Array<ContactEvent> contacts_added;
    collections::Array<ContactEvent> contacts_persisted;
    /** by step_end: this step's removals, and the ones queued by bodies removed since the step before */
    collections::Array<ContactRemovedEvent> contacts_removed;
    /**
     * by narrowphase: the bodies its contacts woke, whose pairs are not found yet. while it is above zero the caller runs
     * find_pairs, answers the pairs it appended, and runs narrowphase again
     */
    uint32_t woken;
};

/** the answer to one contacts_added or contacts_persisted entry; `written` when the caller touched it at all */
struct ContactOverride {
    bool written;
    /** drop the contact before the solve */
    bool reject;
    ContactSettings settings;
};

struct WorldAnswers {
    /** one bit per pairs entry */
    collections::Array<uint32_t> pair_reject;
    /** one per contacts_added entry, then one per contacts_persisted entry */
    collections::Array<ContactOverride> contact_override;
};

namespace events {

inline WorldEvents create_events() {
    WorldEvents e = {};
    return e;
}

inline WorldAnswers create_answers() {
    WorldAnswers a = {};
    return a;
}

/** no narrowphase for pairs entry `pair` this step, and the contacts the pair had are gone */
inline void reject_pair(WorldAnswers &a, uint32_t pair) { a.pair_reject[pair >> 5] |= 1u << (pair & 31); }

/** the event an override index answers: contacts_added's, then contacts_persisted's */
inline const ContactEvent &contact_event(const WorldEvents &e, uint32_t index) {
    const uint32_t added = e.contacts_added.length;
    return index < added ? e.contacts_added[index] : e.contacts_persisted[index - added];
}

/** the answer to contact event `index`, written, its settings the event's until the caller changes them */
inline ContactOverride &override_contact(WorldAnswers &a, const WorldEvents &e, uint32_t index) {
    ContactOverride &answer = a.contact_override[index];
    if (!answer.written) {
        answer.written = true;
        answer.reject = false;
        answer.settings = contact_event(e, index).settings;
    }
    return answer;
}

inline bool pair_rejected(const WorldAnswers &a, uint32_t pair) {
    const uint32_t word = pair >> 5;
    return word < a.pair_reject.length && (a.pair_reject[word] >> (pair & 31)) & 1u;
}

/**
 * one reject bit per pairs entry: the words the pairs appended since `first_pair` start clear. a word the earlier pairs
 * share is clear past them already, as the caller sets bits only below the count
 */
inline void reset_pair_answers(WorldAnswers &a, const WorldEvents &e, uint32_t first_pair) {
    const uint32_t words = (e.pairs.length + 31) >> 5;
    a.pair_reject.reserve(words);
    for (uint32_t i = (first_pair + 31) >> 5; i < words; i++) a.pair_reject.data[i] = 0;
    a.pair_reject.length = words;
}

/**
 * one override per contact event, added then persisted: the ones the narrowphase appended since `first_override` start
 * unwritten. the caller writes them after the step's last narrowphase, so the earlier ones are unwritten still
 */
inline void reset_contact_answers(WorldAnswers &a, const WorldEvents &e, uint32_t first_override) {
    const uint32_t overrides = e.contacts_added.length + e.contacts_persisted.length;
    a.contact_override.reserve(overrides);
    for (uint32_t i = first_override; i < overrides; i++) a.contact_override.data[i].written = false;
    a.contact_override.length = overrides;
}

}  // namespace events

TUMBLE_NAMESPACE_END
