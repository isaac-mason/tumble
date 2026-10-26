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

// instead of listener callbacks: a phase of the step (update.h) appends events, and the caller reads them before the
// next phase. the caller may reject a pair or a contact, or change a contact's settings, on the event itself, and the
// next phase reads that. the buffers never drop an entry

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
    /** set it before the narrowphase to skip the pair: it gets no contacts, and the ones it had are gone */
    bool rejected;
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
    /**
     * what the constraint was built from: the combined material, no scales, no surface velocity. change it before the solve
     * through events::change_contact
     */
    ContactSettings settings;
    /** set it before the solve to drop the contact */
    bool rejected;
    /** set by events::change_contact: the solve rebuilds the constraint from settings */
    bool settings_changed;
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
     * find_pairs, may reject the pairs it appended, and runs narrowphase again
     */
    uint32_t woken;
};

namespace events {

inline WorldEvents create_events() {
    WorldEvents e = {};
    return e;
}

/** the contact's settings, to change before the solve, which rebuilds its constraint from them */
inline ContactSettings &change_contact(ContactEvent &contact) {
    contact.settings_changed = true;
    return contact.settings;
}

}  // namespace events

TUMBLE_NAMESPACE_END
