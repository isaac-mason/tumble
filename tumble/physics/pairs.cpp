#include "tumble/physics/pairs.h"

#include "tumble/math/aabox.h"
#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/sleep.h"
#include "tumble/physics/contacts.h"
#include "tumble/physics/filter.h"

TUMBLE_NAMESPACE_BEGIN

namespace pairs {


Pairs create() {
    Pairs pairs = {};
    pairs.record_count = 0;
    return pairs;
}

void mark_moved(Pairs &pairs, RigidBody &body) {
    if (body.in_move_set) return;
    pairs.moved.push(body.index);
    body.in_move_set = true;
}

/** the edge an edge key names: its record's links on its side */
static Pairs::Edge &edge_of(Pairs &pairs, int32_t key) { return pairs.records[(uint32_t)get_pair_edge_record(key)].edges[get_pair_edge_side(key)]; }

// whether two bodies can ever collide given their motion types, sensor roles, and the kinematic-vs-non-dynamic opt-in
// (independent of position, layer and group). two non-dynamic bodies (static-static, static-kinematic without opt-in,
// ...) never can. shared by pair discovery (skip creating a dead record) and emission (skip reporting) so the two can't
// drift.
static bool bodies_can_collide(const RigidBody &a, const RigidBody &b) {
    return a.collide_kinematic_vs_non_dynamic || b.collide_kinematic_vs_non_dynamic || a.motion_type == MotionType::DYNAMIC ||
           b.motion_type == MotionType::DYNAMIC || (a.motion_type == MotionType::KINEMATIC && b.sensor) || (b.motion_type == MotionType::KINEMATIC && a.sensor);
}

/**
 * the emission predicate for the sweep: whether a persistent record emits an (active_body, other_body) narrowphase
 * pair this step. layers, groups, the shared bodies_can_collide gate, and the active_index dedup.
 */
static bool should_report_pair(const Layers &layers, const RigidBody &active_body, const RigidBody &other_body) {
    // self-collision + deduplication: report only when active_body.active_index < other_body.active_index. sleeping and
    // static bodies have INACTIVE_BODY_INDEX (the max value), so an active body always reports pairs with them; two
    // active bodies report exactly once. bodies are woken only after the pair loop, so the active list is frozen while
    // querying.
    if (active_body.active_index >= other_body.active_index) return false;

    // motion type / sensor gate
    if (!bodies_can_collide(active_body, other_body)) return false;

    // object layer pair table
    if (!layers::object_layer_collides_with_object_layer(layers, active_body.object_layer, other_body.object_layer)) return false;

    // collision group / mask
    if (!filter::should_pair_collide(active_body.collision_groups, active_body.collision_mask, other_body.collision_groups,
                                     other_body.collision_mask)) {
        return false;
    }

    return true;
}

/** the sweep's keep test: body a's fat leaf grown by the speculative distance against body b's */
static bool fat_leaves_overlap(const Broadphase &broadphase, const RigidBody &body_a, const RigidBody &body_b,
                               float speculative_contact_distance) {
    Box3f fat_a, fat_b;
    broadphase::read_fat_bounds(fat_a, broadphase, body_a);
    broadphase::read_fat_bounds(fat_b, broadphase, body_b);
    // mirrors the discovery reach (fat + speculative distance vs fat) so a just-discovered pair always survives the
    // same-step sweep, whatever the relative sizes of the expansion margin and the speculative distance
    return aabox::intersects(aabox::expand_by_margin(aabox::load(fat_a), speculative_contact_distance), aabox::load(fat_b));
}

/** the emission's tight test: the more active body's box grown by the speculative distance against the other's */
static bool overlaps_for_narrowphase(const RigidBody &more_active, const RigidBody &less_active, float speculative_contact_distance) {
    return aabox::intersects(aabox::expand_by_margin(aabox::load(more_active.aabb), speculative_contact_distance), aabox::load(less_active.aabb));
}

/** link a record's edge at the head of the body's intrusive pair list */
static void link_pair_edge(Pairs &pairs, int32_t record, int32_t side, RigidBody &body) {
    Pairs::Edge &edge = pairs.records[(uint32_t)record].edges[side];
    edge.prev = INVALID_PAIR_KEY;
    edge.next = body.head_pair_key;

    if (body.head_pair_key != INVALID_PAIR_KEY) {
        edge_of(pairs, body.head_pair_key).prev = pair_edge_key(record, side);
    }

    body.head_pair_key = pair_edge_key(record, side);
}

/** unlink a record's edge from its body's pair list, updating neighbours and the body's head */
static void unlink_pair_edge(Pairs &pairs, Bodies &bodies, int32_t record, int32_t side) {
    Pairs::Record &pair_record = pairs.records[(uint32_t)record];
    Pairs::Edge &edge = pair_record.edges[side];
    const int32_t prev_key = edge.prev;
    const int32_t next_key = edge.next;

    if (prev_key == INVALID_PAIR_KEY) {
        // this edge was the head: update the body's head pointer
        bodies.pool[(uint32_t)pair_record.body_index[side]]->head_pair_key = next_key;
    } else {
        edge_of(pairs, prev_key).next = next_key;
    }

    if (next_key != INVALID_PAIR_KEY) {
        edge_of(pairs, next_key).prev = prev_key;
    }

    edge.prev = INVALID_PAIR_KEY;
    edge.next = INVALID_PAIR_KEY;
}

/** room for `needed`, at least doubling when it has to grow */
template <typename T>
static void reserve_doubling(collections::Array<T> &vec, uint32_t needed) {
    if (needed <= vec.capacity) return;
    vec.reserve(needed > vec.capacity * 2 ? needed : vec.capacity * 2);
}

/** a new record slot at the end of every per-record array */
static void grow_record_slots(Pairs &pairs) {
    const uint32_t record_slots = pairs.records.length + 1;
    reserve_doubling(pairs.records, record_slots);
    reserve_doubling(pairs.pose_cache, record_slots);
    reserve_doubling(pairs.first_contact, record_slots);
    pairs.records.push({{INVALID_PAIR_KEY, INVALID_PAIR_KEY}, {{INVALID_PAIR_KEY, INVALID_PAIR_KEY}, {INVALID_PAIR_KEY, INVALID_PAIR_KEY}}});
    pairs.pose_cache.push({});
    pairs.first_contact.push(contacts::INVALID_CONTACT_KEY);
}

int32_t add_pair_record(Pairs &pairs, RigidBody &body_a, RigidBody &body_b) {
    int32_t record;
    if (pairs.free_records.length > 0) {
        record = pairs.free_records[pairs.free_records.length - 1];
        pairs.free_records.length--;
    } else {
        record = (int32_t)pairs.records.length;
        grow_record_slots(pairs);
    }

    pairs.records[(uint32_t)record].body_index[0] = (int32_t)body_a.index;
    pairs.records[(uint32_t)record].body_index[1] = (int32_t)body_b.index;

    // link each edge at the head of its body's pair list
    link_pair_edge(pairs, record, 0, body_a);
    link_pair_edge(pairs, record, 1, body_b);

    // a fresh record has no last-narrowphase pose and no contacts yet
    pairs.pose_cache[(uint32_t)record].valid = false;
    pairs.first_contact[(uint32_t)record] = contacts::INVALID_CONTACT_KEY;
    pairs.record_count++;

    return record;
}

/**
 * destroys the record: its contact chain first (so a live contact always has a live record listing it), then both
 * edges unlinked from their bodies' lists and the slot back on the freelist. record indices are stable. the chain's
 * removals are reported to `events` now, or queued for the next step's flush when `queue_events` is set (body removal).
 */
static void remove_pair_record_at(Pairs &pairs, Contacts &contacts, Bodies &bodies, int32_t record,
                                  WorldEvents &events, bool queue_events) {
    contacts::destroy_pair_chain(contacts, pairs, record, events, queue_events);

    unlink_pair_edge(pairs, bodies, record, 0);
    unlink_pair_edge(pairs, bodies, record, 1);

    // body_index[0] == INVALID_PAIR_KEY marks a free slot
    pairs.records[(uint32_t)record].body_index[0] = INVALID_PAIR_KEY;
    pairs.free_records.push(record);
    pairs.record_count--;
}

void set_cache(Pairs &pairs, int32_t record, Vec3f delta_position, Quatf delta_rotation) {
    Pairs::PoseCache &cache = pairs.pose_cache[(uint32_t)record];
    cache.valid = true;
    vec3f::store(cache.delta_position, delta_position);
    quatf::store(cache.delta_rotation, delta_rotation);
}

int32_t find_pair_record(const Pairs &pairs, const RigidBody &body_a, const RigidBody &body_b) {
    int32_t edge_key = body_a.head_pair_key;
    while (edge_key != INVALID_PAIR_KEY) {
        const int32_t record = get_pair_edge_record(edge_key);
        const int32_t side = get_pair_edge_side(edge_key);
        const Pairs::Record &pair_record = pairs.records[(uint32_t)record];
        if (pair_record.body_index[1 - side] == (int32_t)body_b.index) return record;
        edge_key = pair_record.edges[side].next;
    }
    return INVALID_PAIR_KEY;
}

void invalidate_body_cache(Pairs &pairs, const RigidBody &body) {
    int32_t edge_key = body.head_pair_key;
    while (edge_key != INVALID_PAIR_KEY) {
        const int32_t record = get_pair_edge_record(edge_key);
        pairs.pose_cache[(uint32_t)record].valid = false;
        edge_key = pairs.records[(uint32_t)record].edges[get_pair_edge_side(edge_key)].next;
    }
}

void purge_body_pairs(Pairs &pairs, Contacts &contacts, Bodies &bodies, RigidBody &body, WorldEvents &events) {
    int32_t edge_key = body.head_pair_key;
    while (edge_key != INVALID_PAIR_KEY) {
        const int32_t record = get_pair_edge_record(edge_key);
        const int32_t side = get_pair_edge_side(edge_key);

        // save the next key before destroying (the unlink clears this record's edges)
        edge_key = pairs.records[(uint32_t)record].edges[side].next;

        remove_pair_record_at(pairs, contacts, bodies, record, events, true);
    }
}

/**
 * discovery visitor state: fat-aabb query candidates for a moved body become persistent pair records. no semantics
 * beyond the static object-layer prune: sleeping, kinematic and mask gates are applied at emission in the sweep, so
 * the persistent set stays valid when those flags change later. discovery never emits.
 */
struct PairDiscovery {
    const Layers *layers;
    Pairs *pairs;
    RigidBody *moved_body;
};

static void visit_pair_discovery(BodyVisitor *visitor, RigidBody *other_body) {
    PairDiscovery &discovery = *static_cast<PairDiscovery *>(visitor->context);
    RigidBody &moved_body = *discovery.moved_body;

    if (other_body->index == moved_body.index) return;

    // motion-type gate: two bodies that can't drive a collision (e.g. static-static) never need a persistent record,
    // emission would only reject it. skipping it here keeps the sweep off dead slots. a later motion type change
    // re-marks the body moved, so the pair is (re)discovered once a dynamic partner exists.
    if (!bodies_can_collide(moved_body, *other_body)) return;

    // static prune only: the object-layer pair table (a layer change marks the body moved for rediscovery, and the
    // sweep re-applies the table at emission, so stale records never emit)
    if (!layers::object_layer_collides_with_object_layer(*discovery.layers, moved_body.object_layer, other_body->object_layer)) return;

    // already tracked? O(pair degree), bounded by how many fat aabbs one body can overlap
    if (find_pair_record(*discovery.pairs, moved_body, *other_body) != INVALID_PAIR_KEY) return;

    add_pair_record(*discovery.pairs, moved_body, *other_body);
}

/**
 * the single emission point: every pair the narrowphase consumes is written here, with its event. the event orders
 * the bodies lower motion type first, then lower id, so the caller sees a pair the same way round every step; colliding_pairs
 * keeps the more active body first.
 */
static void emit_colliding_pair(Pairs &pairs, const RigidBody &more_active, const RigidBody &less_active, int32_t record,
                                WorldEvents &events) {
    const RigidBody *sorted_a = &more_active;
    const RigidBody *sorted_b = &less_active;
    if (sorted_a->motion_type > sorted_b->motion_type || (sorted_a->motion_type == sorted_b->motion_type && sorted_b->id < sorted_a->id)) {
        sorted_a = &less_active;
        sorted_b = &more_active;
    }
    pairs.colliding_pairs.push({more_active.index, less_active.index, record});

    events.pairs.push({sorted_a->index, sorted_b->index, sorted_a->id, sorted_b->id, sorted_a->user_data, sorted_b->user_data, record, false});
}

/** the pairs of one body woken after the sweep */
static void emit_woken_body_pairs(Pairs &pairs, Broadphase &broadphase, const Layers &layers, Bodies &bodies,
                                  const RigidBody &body, float speculative_contact_distance, WorldEvents &events) {
    if (body.broadphase_layer == -1) return;

    int32_t key = body.head_pair_key;
    while (key != INVALID_PAIR_KEY) {
        const int32_t record = get_pair_edge_record(key);
        const int32_t side = get_pair_edge_side(key);
        const Pairs::Record &pair_record = pairs.records[(uint32_t)record];
        key = pair_record.edges[side].next;
        const RigidBody &other = *bodies.pool[(uint32_t)pair_record.body_index[1 - side]];

        // the sweep's tests, the field compares first: most of a woken body's partners were awake at the sweep and
        // reported the pair then. the record's own order for the fat leaves, as the sweep keeps it
        if (other.broadphase_layer == -1 || !should_report_pair(layers, body, other)) continue;
        const RigidBody &body_a = side == 0 ? body : other;
        const RigidBody &body_b = side == 0 ? other : body;
        if (fat_leaves_overlap(broadphase, body_a, body_b, speculative_contact_distance) &&
            overlaps_for_narrowphase(body, other, speculative_contact_distance)) {
            emit_colliding_pair(pairs, body, other, record, events);
        }
    }
}

void find_colliding_pairs(Pairs &pairs, Contacts &contacts, Broadphase &broadphase, const Layers &layers,
                          Bodies &bodies, float speculative_contact_distance, WorldEvents &events) {
    pairs.swept = true;
    pairs.narrowphase_index = 0;

    pairs.colliding_pairs.length = 0;

    // (a) discovery: moved bodies query the trees with their fat leaf aabb (expanded by the speculative distance) and
    // register new persistent pairs. runs before the sweep so pairs discovered this step emit through the same single
    // point this same step.
    collections::Array<uint32_t> &moved = pairs.moved;
    PairDiscovery discovery = {&layers, &pairs, nullptr};
    for (uint32_t i = 0; i < moved.length; i++) {
        RigidBody *body = bodies.pool[moved[i]];

        // guard against bodies removed (and not reused) after being marked this step
        if (bodies.sequences[moved[i]] == bodies::SLOT_FREE || body->broadphase_layer == -1) continue;

        const uint32_t object_layer = body->object_layer;
        const uint32_t body_broadphase_layer = (uint32_t)body->broadphase_layer;

        // fat leaf aabb expanded by the speculative distance: the discovery reach
        Box3f fat_leaf, expanded_fat_aabb;
        broadphase::read_fat_bounds(fat_leaf, broadphase, *body);
        aabox::store(expanded_fat_aabb, aabox::expand_by_margin(aabox::load(fat_leaf), speculative_contact_distance));

        for (uint32_t other_broadphase_layer = 0; other_broadphase_layer < broadphase.trees.length; other_broadphase_layer++) {
            if (!layers::object_layer_collides_with_broadphase_layer(layers, object_layer, other_broadphase_layer)) continue;
            if (!layers::broadphase_layer_collides_with_broadphase_layer(layers, body_broadphase_layer, other_broadphase_layer)) continue;

            discovery.moved_body = body;
            BodyVisitor visitor = body_visitor::create(visit_pair_discovery, &discovery);
            broadphase::intersect_fat_leaves(broadphase, bodies, other_broadphase_layer, expanded_fat_aabb, visitor);
        }
    }

    // moved flags stay set through the sweep: the step-invariant skip below reads them. the discovery batch is consumed
    // (flags cleared, entries dequeued) after the sweep.
    const uint32_t discovered_count = moved.length;

    // (b) sweep: walk the persistent pair set, destroying separated pairs and emitting the rest. record indices are
    // stable (freelist storage), so the records written into colliding_pairs stay valid for the whole step. emission
    // order follows record-slot order (insertion history + freelist reuse); downstream must not depend on it (contact
    // constraints are sorted by deterministic keys).
    const uint32_t num_slots = pairs.records.length;
    for (uint32_t slot = 0; slot < num_slots; slot++) {
        const int32_t record = (int32_t)slot;
        const Pairs::Record &pair_record = pairs.records[slot];

        if (pair_record.body_index[0] == INVALID_PAIR_KEY) continue;

        RigidBody &body_a = *bodies.pool[(uint32_t)pair_record.body_index[0]];
        RigidBody &body_b = *bodies.pool[(uint32_t)pair_record.body_index[1]];

        // step-invariant skip: two inactive, unmoved bodies with an empty contact chain. their leaves are frozen (so
        // the fat-overlap answer is last step's "keep"), emission is impossible (dedup rejects both-inactive), and there
        // is no chain to reconcile. a steady-state sleeping pair costs a few field compares per step.
        if (body_a.active_index == sleep::INACTIVE_BODY_INDEX && body_b.active_index == sleep::INACTIVE_BODY_INDEX && !body_a.in_move_set &&
            !body_b.in_move_set && pairs.first_contact[slot] == contacts::INVALID_CONTACT_KEY) {
            continue;
        }

        // destroy if either leaf is gone, or the two fat leaf aabbs no longer overlap
        if (body_a.broadphase_layer == -1 || body_b.broadphase_layer == -1) {
            remove_pair_record_at(pairs, contacts, bodies, record, events, false);
            continue;
        }
        if (!fat_leaves_overlap(broadphase, body_a, body_b, speculative_contact_distance)) {
            remove_pair_record_at(pairs, contacts, bodies, record, events, false);
            continue;
        }

        // more_active = smaller active_index, as should_report_pair's first argument (both inactive => dedup rejects)
        const RigidBody &more_active = body_a.active_index <= body_b.active_index ? body_a : body_b;
        const RigidBody &less_active = &more_active == &body_a ? body_b : body_a;

        // the tight-aabb re-test, then the report gates
        bool emitted = false;
        if (overlaps_for_narrowphase(more_active, less_active, speculative_contact_distance) && should_report_pair(layers, more_active, less_active)) {
            emit_colliding_pair(pairs, more_active, less_active, record, events);
            emitted = true;
        }

        // a kept-but-not-emitted pair with a live contact chain has its chain destroyed here, reported now: sleep
        // transitions (both-inactive dedup), teleports that separate tight but not fat boxes, motion / layer / group /
        // mask gates and tight-aabb misses all land here, each destroying a chain exactly once. the empty-chain check
        // keeps steady-state sleeping pairs O(1) per visit. a pair the caller rejects after this phase has its chain destroyed
        // by the narrowphase.
        if (!emitted && pairs.first_contact[slot] != contacts::INVALID_CONTACT_KEY) {
            contacts::destroy_pair_chain(contacts, pairs, record, events, false);
        }
    }

    // consume the move set: clear the discovery batch's flags and dequeue it
    for (uint32_t i = 0; i < discovered_count; i++) bodies.pool[moved[i]]->in_move_set = false;
    moved.length = 0;

    pairs.active_read_index = bodies.active_body_count;
}

void find_woken_body_pairs(Pairs &pairs, Broadphase &broadphase, const Layers &layers, Bodies &bodies,
                           float speculative_contact_distance, WorldEvents &events) {
    while (pairs.active_read_index < bodies.active_body_count) {
        const RigidBody &woken = *bodies.pool[bodies.active_body_indices[pairs.active_read_index++]];
        emit_woken_body_pairs(pairs, broadphase, layers, bodies, woken, speculative_contact_distance, events);
    }
}

}  // namespace pairs

TUMBLE_NAMESPACE_END
