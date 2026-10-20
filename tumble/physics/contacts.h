#pragma once

// crashcat's contacts.ts, the types: the contacts of a world, each nested under one pair record in a
// doubly linked chain, with a double-buffered cached manifold for warm starting. the chain and cache
// logic is contacts.cpp. the listener's onContactRemoved is the contacts_removed event buffer.

#include <stdint.h>

#include "math/types.h"
#include "physics/body/body_id.h"
#include "physics/body/rigid_body.h"
#include "physics/body/sub_shape.h"
#include "physics/events.h"
#include "physics/manifold/manifold.h"
#include "physics/pairs.h"
#include "util/collections.h"

namespace contacts {

/** the end of a pair's contact chain, and a freed contact's index */
constexpr int32_t INVALID_CONTACT_KEY = -1;

enum CachedManifoldFlags : uint32_t {
    MANIFOLD_FLAG_NONE = 0,
    /** matched and reused from the step before */
    MANIFOLD_FLAG_CONTACT_PERSISTED = 1u << 0,
    /** made by ccd */
    MANIFOLD_FLAG_CCD_CONTACT = 1u << 1,
};

/** a point with the normal impulse that built up on it, for warm starting */
struct CachedContactPoint {
    /** in body a's local centre of mass space */
    Float3 position_1;
    /** in body b's local centre of mass space */
    Float3 position_2;
    float normal_lambda;
};

/** the per-step cache: read from the step before, written this step */
struct CachedManifold {
    /** in body b's local space */
    Float3 contact_normal;
    uint32_t num_contact_points;
    CachedContactPoint contact_points[manifold::MAX_CONTACT_POINTS];
    /** the manifold's friction impulses, for warm starting */
    float friction_lambda_1;
    float friction_lambda_2;
    float angular_friction_lambda;
    /** CachedManifoldFlags */
    uint32_t flags;
};

struct Contact {
    /** index in Contacts::contacts when live, INVALID_CONTACT_KEY when freed */
    int32_t contact_index;
    /** a's id is at most b's: a consistent order per pair */
    body_id::BodyId body_id_a;
    body_id::BodyId body_id_b;
    uint32_t body_index_a;
    uint32_t body_index_b;
    sub_shape::SubShapeId sub_shape_id_a;
    sub_shape::SubShapeId sub_shape_id_b;
    /** Contacts::frame_stamp of the last step that processed this contact; 0 at first */
    uint32_t last_processed_frame;
    /** the owning pair record, INVALID_PAIR_KEY when freed */
    int32_t pair_record;
    /** the chain through the owning pair's contacts, INVALID_CONTACT_KEY at the ends */
    int32_t prev_in_pair;
    int32_t next_in_pair;
    /** the double buffer; read through get_read_manifold / get_write_manifold, never by index */
    CachedManifold manifolds[2];
};

struct Contacts {
    /** every contact, live or freed */
    collections::Vec<Contact> contacts;
    collections::Vec<int32_t> free_indices;
    /** which manifold buffer holds the step before's data; flipped once per step after the solver stores impulses */
    uint32_t read_index;
    /**
     * removals queued as [body_id_a, body_id_b, sub_shape_id_a, sub_shape_id_b] to report at the start of the next
     * step (jolt reports a removed body's contacts the update after), body ids as two u32 each
     */
    collections::Vec<uint32_t> pending_contact_removed;
    /** incremented at the top of every step; a contact is fresh this step iff last_processed_frame == frame_stamp */
    uint32_t frame_stamp;
};

Contacts create();
void release(Contacts &contacts);

/** physics_step_begin: moves the queued removals into events.contacts_removed */
void flush_pending_contact_removed(Contacts &contacts, events::PhysicsEvents &events);

inline CachedManifold &get_read_manifold(Contact &contact, const Contacts &contacts) { return contact.manifolds[contacts.read_index]; }
inline CachedManifold &get_write_manifold(Contact &contact, const Contacts &contacts) { return contact.manifolds[1 - contacts.read_index]; }
inline void flip_manifold_cache(Contacts &contacts) { contacts.read_index = 1 - contacts.read_index; }

/**
 * a contact under the pair record, linked at the head of its chain, from the pool or new. the pointer is into
 * Contacts::contacts and holds until the next create.
 */
Contact *create_contact(Contacts &contacts, pairs::Pairs &pairs, int32_t record, const rigid_body::RigidBody &body_a,
                        const rigid_body::RigidBody &body_b, sub_shape::SubShapeId sub_shape_id_a, sub_shape::SubShapeId sub_shape_id_b);
/** unlinks the contact from its pair, frees it and reports it removed */
void destroy_contact(Contacts &contacts, pairs::Pairs &pairs, Contact &contact, events::PhysicsEvents &events);
/** destroys every contact of the pair; `queue_events` defers the removal reports to the next step's flush */
void destroy_pair_chain(Contacts &contacts, pairs::Pairs &pairs, int32_t record, events::PhysicsEvents &events, bool queue_events);
/** destroys the pair's contacts the current step did not touch */
void destroy_stale_contacts_in_pair(Contacts &contacts, pairs::Pairs &pairs, int32_t record, events::PhysicsEvents &events);
/** the pair's contact between these sub shapes, or null */
Contact *find_contact_in_pair(Contacts &contacts, const pairs::Pairs &pairs, int32_t record, sub_shape::SubShapeId sub_shape_id_a,
                              sub_shape::SubShapeId sub_shape_id_b);

}  // namespace contacts
