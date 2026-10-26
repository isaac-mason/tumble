#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/collision/collector.h"
#include "tumble/physics/utils/face.h"
#include "tumble/util/collections.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

struct Shape;

struct CollideShapeHit {
    /** contact on body a, world space */
    Vec3f point_a;
    /** contact on body b, world space */
    Vec3f point_b;
    /** direction to move shape b out of collision; magnitude meaningless; world space */
    Vec3f penetration_axis;
    /** positive when overlapping; negative when max_separation_distance allows */
    float penetration;
    SubShapeId sub_shape_id_a;
    SubShapeId sub_shape_id_b;
    /** supporting faces, world space, filled when the settings ask */
    SupportingFace face_a;
    SupportingFace face_b;
    BodyId body_id_b;
};

namespace collide_shape {

inline CollideShapeHit create_hit() {
    CollideShapeHit h = {};
    h.sub_shape_id_a = sub_shape::EMPTY_SUB_SHAPE_ID;
    h.sub_shape_id_b = sub_shape::EMPTY_SUB_SHAPE_ID;
    h.body_id_b = body_id::INVALID_BODY_ID;
    return h;
}

/** copies a hit, faces only as far as they are filled. */
inline void copy_hit(CollideShapeHit &out, const CollideShapeHit &h) {
    out.point_a = h.point_a;
    out.point_b = h.point_b;
    out.penetration_axis = h.penetration_axis;
    out.penetration = h.penetration;
    out.sub_shape_id_a = h.sub_shape_id_a;
    out.sub_shape_id_b = h.sub_shape_id_b;
    face::copy(out.face_a, h.face_a);
    face::copy(out.face_b, h.face_b);
    out.body_id_b = h.body_id_b;
}

}  // namespace collide_shape

struct CollideShapeSettings {
    /** contacts are found up to this separation; 0 for touching only. clamped to [0, 1] for epa */
    float max_separation_distance;
    /** closer than this is colliding (gjk), meters */
    float collision_tolerance;
    /** the penetration depth search stops when the squared distance changes by less than this fraction of the depth squared */
    float penetration_tolerance;
    /** back facing triangles (triangle meshes) are collided with, otherwise they are ignored */
    bool collide_with_backfaces;
    /** only edges a moving object should bump into count */
    bool collide_only_with_active_edges;
    /** with active edges only: the movement, so an inactive edge's triangle normal is used when it impedes the movement less */
    Vec3f active_edge_movement_direction;
    /** fill the supporting faces of the hits */
    bool collect_faces;
    /** max squared distance to consider a vertex to be the same as another vertex, used by the internal edge removal algorithm to determine if two edges are shared (meters squared) */
    float internal_edge_removal_vertex_tolerance_sq;
};

namespace collide_shape {

constexpr float DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ = 1e-8f;

/** back faces off */
inline CollideShapeSettings default_settings() {
    return {0.0f, 1e-4f, 1e-4f, false, true, vec3f::zero(), false, DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ};
}

}  // namespace collide_shape

struct CollideShapeCollector;
typedef void (*CollideShapeAddHit)(CollideShapeCollector &collector, const CollideShapeHit &hit);
typedef void (*CollideShapeAddMiss)(CollideShapeCollector &collector);
typedef void (*CollideShapeOnBody)(CollideShapeCollector &collector, BodyId body_id);
typedef void (*CollideShapeOnBodyEnd)(CollideShapeCollector &collector);

struct CollideShapeCollector {
    CollectorKind kind;
    BodyId body_id_b;
    /** a penetration bound: a shape may skip contacts shallower than this */
    float early_out_fraction;
    collections::Array<CollideShapeHit> hits;
    bool has_hit;
    CollideShapeAddHit custom_add_hit;
    CollideShapeAddMiss custom_add_miss;
    /** optional, any kind: told when a query moves on to the next body and when it is done with it */
    CollideShapeOnBody on_body;
    CollideShapeOnBodyEnd on_body_end;
    void *context;
};

namespace collide_shape {

inline CollideShapeCollector create_collector(CollectorKind kind) {
    CollideShapeCollector c = {};
    c.kind = kind;
    c.body_id_b = body_id::INVALID_BODY_ID;
    c.early_out_fraction = collector::UNBOUNDED;
    return c;
}

inline const CollideShapeHit *hit(const CollideShapeCollector &c) { return c.has_hit ? &c.hits[0] : nullptr; }

inline void reset(CollideShapeCollector &c) {
    c.body_id_b = body_id::INVALID_BODY_ID;
    c.early_out_fraction = collector::UNBOUNDED;
    c.hits.length = 0;
    c.has_hit = false;
}

inline void keep_one(CollideShapeCollector &c, const CollideShapeHit &h) {
    if (c.hits.length == 0) c.hits.push(h);
    else copy_hit(c.hits[0], h);
    c.has_hit = true;
}

inline void add_hit(CollideShapeCollector &c, const CollideShapeHit &h) {
    switch (c.kind) {
        case CollectorKind::ALL:
            c.hits.push(h);
            c.has_hit = true;
            break;
        case CollectorKind::ANY:
            keep_one(c, h);
            // the bound is -penetration, so -FLT_MAX prunes every further candidate
            c.early_out_fraction = -collector::UNBOUNDED;
            break;
        case CollectorKind::CLOSEST:
            // the deepest penetration wins; the bound is -penetration, so a shape may skip anything whose
            // -penetration is not below it
            if (!c.has_hit || h.penetration > c.hits[0].penetration) {
                keep_one(c, h);
                c.early_out_fraction = -h.penetration;
            }
            break;
        case CollectorKind::CUSTOM:
            c.custom_add_hit(c, h);
            break;
    }
}

inline void add_miss(CollideShapeCollector &c) {
    if (c.kind == CollectorKind::CUSTOM && c.custom_add_miss != nullptr) c.custom_add_miss(c);
}

inline bool should_early_out(const CollideShapeCollector &c) { return c.early_out_fraction <= -collector::UNBOUNDED; }

/** stops the query: for a custom collector's add_hit */
inline void force_early_out(CollideShapeCollector &c) { c.early_out_fraction = -collector::UNBOUNDED; }

/** stable, deepest first */
inline void sort(CollideShapeCollector &c) {
    sort::insertion_sort(
        c.hits.data, c.hits.data + c.hits.length,
        [](const CollideShapeHit &a, const CollideShapeHit &b) { return a.penetration > b.penetration; }, copy_hit);
}

inline void on_body(CollideShapeCollector &c, BodyId id) {
    if (c.on_body != nullptr) c.on_body(c, id);
}

inline void on_body_end(CollideShapeCollector &c) {
    if (c.on_body_end != nullptr) c.on_body_end(c);
}

// the inverted collector: a collision run with a and b swapped reports through it, and it swaps the
// hit back before the collector it wraps (the context) sees it

void inverted_add_hit(CollideShapeCollector &c, const CollideShapeHit &h);
void inverted_add_miss(CollideShapeCollector &c);

}  // namespace collide_shape

typedef void (*CollideShapeFn)(CollideShapeCollector &collector, const CollideShapeSettings &settings, const Shape *shape_a,
                          const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, const Shape *shape_b,
                          const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

namespace collide_shape {

/** `fn` with a and b swapped, its hits swapped back: the handler for the reverse pair */
template <CollideShapeFn fn>
void reversed_collide_shape_vs_shape(CollideShapeCollector &collector, const CollideShapeSettings &settings, const Shape *shape_a,
                                     const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                     const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                     const Transformf &transform_b) {
    CollideShapeCollector inverted = create_collector(CollectorKind::CUSTOM);
    inverted.body_id_b = collector.body_id_b;
    inverted.early_out_fraction = collector.early_out_fraction;
    inverted.custom_add_hit = inverted_add_hit;
    inverted.custom_add_miss = inverted_add_miss;
    inverted.context = &collector;
    fn(inverted, settings, shape_b, sub_shape_id_b, transform_b, shape_a, sub_shape_id_a, transform_a);
}

}  // namespace collide_shape

TUMBLE_NAMESPACE_END
