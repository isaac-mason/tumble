#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/broadphase/broadphase.h"
#include "tumble/physics/events.h"
#include "tumble/physics/layers.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct Contacts;

// the persistent overlapping pairs, found from the move set; records chain into each body's pair list by edge keys

namespace pairs {

/** the end of a body's pair list, and a free record's body_index[0] */
constexpr int32_t INVALID_PAIR_KEY = -1;

}
struct Pairs {
      // namespace pairs

    /** a record's links through one body's pair list, as edge keys */
    struct Edge {
        int32_t prev;
        int32_t next;
    };

    struct Record {
        /** body a's index, then body b's, by side; pairs::INVALID_PAIR_KEY first on a free record */
        int32_t body_index[2];
        /** each body's links, by side */
        Edge edges[2];
    };

    /** body b's centre of mass relative to a in a's frame, and inv(ra) * rb, at the pair's last narrowphase */
    struct PoseCache {
        bool valid;
        Float3 delta_position;
        Float4 delta_rotation;
    };

    /** this step's narrowphase input: the more active body first, and the record */
    struct CollidingPair {
        uint32_t body_index_a;
        uint32_t body_index_b;
        int32_t pair_record;
    };

    /** a free list with stable indices */
    collections::Array<Record> records;
    collections::Array<int32_t> free_records;
    /** live records (records.length counts live and free) */
    uint32_t record_count;
    /** per record; a reused record starts invalid */
    collections::Array<PoseCache> pose_cache;
    collections::Array<CollidingPair> colliding_pairs;
    /** per record, the head of its contact chain or contacts::INVALID_CONTACT_KEY */
    collections::Array<int32_t> first_contact;
    /** the move set: bodies whose fat leaf changed, awaiting pair discovery; deduped by the body's in_move_set */
    collections::Array<uint32_t> moved;
    /** whether find_colliding_pairs ran this step (the step clears it): the step's later pair finding is find_woken_body_pairs */
    bool swept;
    /** how far into the active list this step's pairs are found: bodies woken past it still need theirs */
    uint32_t active_read_index;
    /** the colliding pairs before it have run the narrowphase this step */
    uint32_t narrowphase_index;
};

namespace pairs {

Pairs create();

/** adds a body to the move set once, so the next find_colliding_pairs discovers its pairs */
void mark_moved(Pairs &pairs, RigidBody &body);

/** a record index and a side (0 = body a's edge, 1 = body b's) packed into one key */
inline int32_t pair_edge_key(int32_t record_index, int32_t side) { return (record_index << 1) | side; }
inline int32_t get_pair_edge_record(int32_t key) { return key >> 1; }
inline int32_t get_pair_edge_side(int32_t key) { return key & 1; }

/** a new record for the pair, linked into both bodies' lists, its cache invalid and its chain empty */
int32_t add_pair_record(Pairs &pairs, RigidBody &body_a, RigidBody &body_b);
void set_cache(Pairs &pairs, int32_t record, Vec3f delta_position, Quatf delta_rotation);
/** the record of the pair, or INVALID_PAIR_KEY */
int32_t find_pair_record(const Pairs &pairs, const RigidBody &body_a, const RigidBody &body_b);
/** every pair of the body runs the narrowphase next step instead of reusing its cached manifolds: its shape changed */
void invalidate_body_cache(Pairs &pairs, const RigidBody &body);
/** unlinks and frees every record of the body, destroying their contact chains */
void purge_body_pairs(Pairs &pairs, Contacts &contacts, Bodies &bodies, RigidBody &body,
                      WorldEvents &events);

/**
 * the move set queries the trees with its fat leaves expanded by the speculative distance and registers new records;
 * then every live record whose bodies still overlap is emitted to colliding_pairs and to events.pairs, where the caller
 * may reject it. the records it drops, and the ones it keeps but does not emit, lose their contact chains
 */
void find_colliding_pairs(Pairs &pairs, Contacts &contacts, Broadphase &broadphase, const Layers &layers,
                          Bodies &bodies, float speculative_contact_distance, WorldEvents &events);

/**
 * after find_colliding_pairs, the pairs of the bodies a contact woke since, appended to colliding_pairs and
 * events.pairs: each such body's records that pass the sweep's overlap tests and that it reports (the other body
 * asleep, or woken after it). a record that no longer overlaps is left for the next sweep to destroy
 */
void find_woken_body_pairs(Pairs &pairs, Broadphase &broadphase, const Layers &layers, Bodies &bodies,
                           float speculative_contact_distance, WorldEvents &events);

}  // namespace pairs

TUMBLE_NAMESPACE_END
