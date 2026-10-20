#include "tumble/physics/contacts.h"

#include "tumble/math/vec3f.h"
#include "tumble/util/mem.h"

TUMBLE_NAMESPACE_BEGIN

namespace contacts {

/** a queued removal: [body_id_a, body_id_b, sub_shape_id_a, sub_shape_id_b] */
static constexpr uint32_t PENDING_REMOVED_STRIDE = 4;

Contacts create() {
    Contacts contacts = {};
    contacts.read_index = 0;
    contacts.frame_stamp = 0;
    return contacts;
}

void release(Contacts &contacts) {
    contacts.contacts.release();
    contacts.free_indices.release();
    contacts.pending_contact_removed.release();
    contacts.read_index = 0;
    contacts.frame_stamp = 0;
}

static void push_removed_event(events::PhysicsEvents &events, body_id::BodyId body_id_a, body_id::BodyId body_id_b,
                               sub_shape::SubShapeId sub_shape_id_a, sub_shape::SubShapeId sub_shape_id_b) {
    events.contacts_removed.push({body_id_a, body_id_b, sub_shape_id_a, sub_shape_id_b});
}

static void queue_removed_event(Contacts &contacts, const Contact &contact) {
    if (!contacts.pending_contact_removed.reserve(contacts.pending_contact_removed.length + PENDING_REMOVED_STRIDE)) return;
    collections::Vec<uint32_t> &pending = contacts.pending_contact_removed;
    pending.push(contact.body_id_a);
    pending.push(contact.body_id_b);
    pending.push(contact.sub_shape_id_a);
    pending.push(contact.sub_shape_id_b);
}

void flush_pending_contact_removed(Contacts &contacts, events::PhysicsEvents &events) {
    collections::Vec<uint32_t> &pending = contacts.pending_contact_removed;
    if (pending.length == 0) return;

    for (uint32_t i = 0; i < pending.length; i += PENDING_REMOVED_STRIDE) {
        push_removed_event(events, pending[i], pending[i + 1], pending[i + 2], pending[i + 3]);
    }
    pending.length = 0;
}

/** reset a cached manifold to default empty values */
static void reset_cached_manifold(CachedManifold &m) {
    m.contact_normal = {0.0f, 0.0f, 0.0f};
    m.num_contact_points = 0;
    m.friction_lambda_1 = 0.0f;
    m.friction_lambda_2 = 0.0f;
    m.angular_friction_lambda = 0.0f;
    m.flags = MANIFOLD_FLAG_NONE;
    for (uint32_t i = 0; i < manifold::MAX_CONTACT_POINTS; i++) {
        CachedContactPoint &point = m.contact_points[i];
        point.position_1 = {0.0f, 0.0f, 0.0f};
        point.position_2 = {0.0f, 0.0f, 0.0f};
        point.normal_lambda = 0.0f;
    }
}

/** set a contact to initial state with new body / sub shape ids */
static void set_contact(Contact &contact, int32_t contact_index, const rigid_body::RigidBody &body_a, const rigid_body::RigidBody &body_b,
                        sub_shape::SubShapeId sub_shape_id_a, sub_shape::SubShapeId sub_shape_id_b) {
    contact.contact_index = contact_index;
    contact.body_id_a = body_a.id;
    contact.body_index_a = body_a.index;
    contact.body_id_b = body_b.id;
    contact.body_index_b = body_b.index;
    contact.sub_shape_id_a = sub_shape_id_a;
    contact.sub_shape_id_b = sub_shape_id_b;
    contact.last_processed_frame = 0;

    // reset both manifold buffers: neither holds valid prior data for a fresh contact
    reset_cached_manifold(contact.manifolds[0]);
    reset_cached_manifold(contact.manifolds[1]);
}

/** unlink a contact from its owning pair's chain, updating neighbours and the record's head */
static void unlink_contact_from_chain(Contacts &contacts, pairs::Pairs &pairs, Contact &contact) {
    const int32_t prev = contact.prev_in_pair;
    const int32_t next = contact.next_in_pair;

    if (prev == INVALID_CONTACT_KEY) {
        // was the chain head: repoint the record's head to the next contact
        pairs.first_contact[contact.pair_record] = next;
    } else {
        contacts.contacts[prev].next_in_pair = next;
    }

    if (next != INVALID_CONTACT_KEY) contacts.contacts[next].prev_in_pair = prev;

    contact.prev_in_pair = INVALID_CONTACT_KEY;
    contact.next_in_pair = INVALID_CONTACT_KEY;
}

Contact *create_contact(Contacts &contacts, pairs::Pairs &pairs, int32_t record, const rigid_body::RigidBody &body_a,
                        const rigid_body::RigidBody &body_b, sub_shape::SubShapeId sub_shape_id_a, sub_shape::SubShapeId sub_shape_id_b) {
    int32_t contact_index;
    if (contacts.free_indices.length > 0) {
        contact_index = contacts.free_indices[contacts.free_indices.length - 1];
        contacts.free_indices.length--;
    } else {
        contact_index = (int32_t)contacts.contacts.length;
        Contact empty;
        util::zero(&empty, sizeof(Contact));
        if (!contacts.contacts.push(empty)) return nullptr;
    }
    Contact &contact = contacts.contacts[contact_index];

    set_contact(contact, contact_index, body_a, body_b, sub_shape_id_a, sub_shape_id_b);

    // link at the head of the record's chain
    contact.pair_record = record;
    contact.prev_in_pair = INVALID_CONTACT_KEY;
    const int32_t old_head = pairs.first_contact[record];
    contact.next_in_pair = old_head;
    if (old_head != INVALID_CONTACT_KEY) contacts.contacts[old_head].prev_in_pair = contact_index;
    pairs.first_contact[record] = contact_index;

    return &contact;
}

void destroy_contact(Contacts &contacts, pairs::Pairs &pairs, Contact &contact, events::PhysicsEvents &events) {
    // report before destroying
    push_removed_event(events, contact.body_id_a, contact.body_id_b, contact.sub_shape_id_a, contact.sub_shape_id_b);

    unlink_contact_from_chain(contacts, pairs, contact);

    const int32_t contact_index = contact.contact_index;
    contact.contact_index = INVALID_CONTACT_KEY;
    contact.pair_record = pairs::INVALID_PAIR_KEY;
    contacts.free_indices.push(contact_index);
}

void destroy_pair_chain(Contacts &contacts, pairs::Pairs &pairs, int32_t record, events::PhysicsEvents &events, bool queue_events) {
    int32_t contact_index = pairs.first_contact[record];
    while (contact_index != INVALID_CONTACT_KEY) {
        Contact &contact = contacts.contacts[contact_index];
        const int32_t next = contact.next_in_pair;

        if (queue_events) {
            queue_removed_event(contacts, contact);
        } else {
            push_removed_event(events, contact.body_id_a, contact.body_id_b, contact.sub_shape_id_a, contact.sub_shape_id_b);
        }

        // free directly: the whole chain is going away, so clear the head once at the end instead of unlinking each node
        contact.contact_index = INVALID_CONTACT_KEY;
        contact.pair_record = pairs::INVALID_PAIR_KEY;
        contact.prev_in_pair = INVALID_CONTACT_KEY;
        contact.next_in_pair = INVALID_CONTACT_KEY;
        contacts.free_indices.push(contact_index);

        contact_index = next;
    }
    pairs.first_contact[record] = INVALID_CONTACT_KEY;
}

void destroy_stale_contacts_in_pair(Contacts &contacts, pairs::Pairs &pairs, int32_t record, events::PhysicsEvents &events) {
    int32_t contact_index = pairs.first_contact[record];
    while (contact_index != INVALID_CONTACT_KEY) {
        Contact &contact = contacts.contacts[contact_index];
        // save next before a possible destroy mutates the chain
        const int32_t next = contact.next_in_pair;
        if (contact.last_processed_frame != contacts.frame_stamp) destroy_contact(contacts, pairs, contact, events);
        contact_index = next;
    }
}

Contact *find_contact_in_pair(Contacts &contacts, const pairs::Pairs &pairs, int32_t record, sub_shape::SubShapeId sub_shape_id_a,
                              sub_shape::SubShapeId sub_shape_id_b) {
    int32_t contact_index = pairs.first_contact[record];
    while (contact_index != INVALID_CONTACT_KEY) {
        Contact &contact = contacts.contacts[contact_index];
        if (contact.sub_shape_id_a == sub_shape_id_a && contact.sub_shape_id_b == sub_shape_id_b) return &contact;
        contact_index = contact.next_in_pair;
    }
    return nullptr;
}

}  // namespace contacts

TUMBLE_NAMESPACE_END
