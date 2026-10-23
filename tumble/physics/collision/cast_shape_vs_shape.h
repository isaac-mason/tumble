#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/collision/collector.h"
#include "tumble/physics/utils/face.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

namespace shapes {
struct Shape;
}

namespace cast_shape {

using collector::CollectorKind;

enum CastShapeStatus : uint32_t {
    NOT_COLLIDING = 0,
    COLLIDING = 1,
};

struct CastShapeHit {
    CastShapeStatus status;
    /** along the sweep: 0 at the start, 1 at the end */
    float fraction;
    /** point on shape a, world space */
    Vec3f point_a;
    /** point on shape b, world space */
    Vec3f point_b;
    float penetration_depth;
    /** direction to move shape b out of collision; magnitude meaningless; the contact normal a -> b when fraction > 0 */
    Vec3f penetration_axis;
    /** -normalize(penetration_axis): from b towards a */
    Vec3f normal;
    sub_shape::SubShapeId sub_shape_id_a;
    sub_shape::SubShapeId sub_shape_id_b;
    /** supporting faces, world space, filled when the settings ask */
    face::Face face_a;
    face::Face face_b;
    body_id::BodyId body_id_b;
};

inline CastShapeHit create_hit() {
    CastShapeHit h = {};
    h.status = NOT_COLLIDING;
    h.fraction = 1.0f;
    h.sub_shape_id_a = sub_shape::EMPTY_SUB_SHAPE_ID;
    h.sub_shape_id_b = sub_shape::EMPTY_SUB_SHAPE_ID;
    h.body_id_b = body_id::INVALID_BODY_ID;
    return h;
}

/** copies a hit, faces only as far as they are filled. */
inline void copy_hit(CastShapeHit &out, const CastShapeHit &h) {
    out.status = h.status;
    out.fraction = h.fraction;
    out.point_a = h.point_a;
    out.point_b = h.point_b;
    out.penetration_depth = h.penetration_depth;
    out.penetration_axis = h.penetration_axis;
    out.normal = h.normal;
    out.sub_shape_id_a = h.sub_shape_id_a;
    out.sub_shape_id_b = h.sub_shape_id_b;
    face::copy(out.face_a, h.face_a);
    face::copy(out.face_b, h.face_b);
    out.body_id_b = h.body_id_b;
}

struct CastShapeSettings {
    /** back facing triangles (triangle meshes) are hit, otherwise they are ignored */
    bool collide_with_backfaces_triangles;
    /** convex shapes (and planes) are hit when the cast moves away from them, otherwise those hits are ignored */
    bool collide_with_backfaces_convex;
    /** fill the supporting faces of the hits */
    bool collect_faces;
    /** closer than this is colliding (gjk), meters */
    float collision_tolerance;
    /** the penetration depth search stops when the squared distance changes by less than this fraction of the depth squared */
    float penetration_tolerance;
    /** when intersecting at the start (fraction 0), find the deepest point too */
    bool return_deepest_point;
    /** shrink by the convex radius and expand again: faster, a more accurate normal, a rounder shape */
    bool use_shrunken_shape_and_convex_radius;
    /** only edges a moving object should bump into count */
    bool collide_only_with_active_edges;
    /** with active edges only: the movement, so an inactive edge's triangle normal is used when it impedes the movement less */
    Vec3f active_edge_movement_direction;
    /** above 0, the cast shape is inflated by this much on every side, as an extra convex radius, meters */
    float extra_convex_radius;
};

inline CastShapeSettings default_settings() { return {false, false, false, 1e-4f, 1e-4f, false, false, true, vec3f::zero(), 0.0f}; }

struct CastShapeCollector;
typedef void (*CastShapeAddHit)(CastShapeCollector &collector, const CastShapeHit &hit);
typedef void (*CastShapeAddMiss)(CastShapeCollector &collector);

struct CastShapeCollector {
    CollectorKind kind;
    body_id::BodyId body_id_b;
    float early_out_fraction;
    collections::Vec<CastShapeHit> hits;
    bool has_hit;
    CastShapeAddHit custom_add_hit;
    CastShapeAddMiss custom_add_miss;
    void *context;
};

inline CastShapeCollector create_collector(CollectorKind kind) {
    CastShapeCollector c = {};
    c.kind = kind;
    c.body_id_b = body_id::INVALID_BODY_ID;
    c.early_out_fraction = collector::INITIAL_EARLY_OUT_FRACTION;
    return c;
}

inline const CastShapeHit *hit(const CastShapeCollector &c) { return c.has_hit ? &c.hits[0] : nullptr; }

inline void reset(CastShapeCollector &c) {
    c.body_id_b = body_id::INVALID_BODY_ID;
    c.early_out_fraction = collector::INITIAL_EARLY_OUT_FRACTION;
    c.hits.length = 0;
    c.has_hit = false;
}

inline void release(CastShapeCollector &c) { c.hits.release(); }

inline void keep_one(CastShapeCollector &c, const CastShapeHit &h) {
    if (c.hits.length == 0) c.hits.push(h);
    else copy_hit(c.hits[0], h);
    c.has_hit = true;
}

/**
 * what a cast hit is ordered by and bounds further searching with: the fraction, or
 * for a hit that starts overlapping (fraction 0) the negated penetration depth, so deeper overlaps sort first
 */
inline float early_out_fraction_of(const CastShapeHit &h) { return h.fraction > 0.0f ? h.fraction : -h.penetration_depth; }

inline void add_hit(CastShapeCollector &c, const CastShapeHit &h) {
    switch (c.kind) {
        case collector::COLLECT_ALL:
            c.hits.push(h);
            c.has_hit = true;
            break;
        case collector::COLLECT_ANY:
            keep_one(c, h);
            // hits at fraction 0 are bounded by -penetration, so -FLT_MAX prunes every further candidate
            c.early_out_fraction = -collector::UNBOUNDED;
            break;
        case collector::COLLECT_CLOSEST:
            // ordered and bounded by the early-out value, so a deeper initial overlap beats a shallower one
            if (!c.has_hit || early_out_fraction_of(h) < early_out_fraction_of(c.hits[0])) {
                keep_one(c, h);
                c.early_out_fraction = early_out_fraction_of(h);
            }
            break;
        case collector::COLLECT_CUSTOM:
            c.custom_add_hit(c, h);
            break;
    }
}

inline void add_miss(CastShapeCollector &c) {
    if (c.kind == collector::COLLECT_CUSTOM && c.custom_add_miss != nullptr) c.custom_add_miss(c);
}

inline bool should_early_out(const CastShapeCollector &c) { return c.early_out_fraction <= -collector::UNBOUNDED; }

/** stops the query: for a custom collector's add_hit */
inline void force_early_out(CastShapeCollector &c) { c.early_out_fraction = -collector::UNBOUNDED; }

/** stable, closest first by their early-out value (deepest first among those that start overlapping) */
inline void sort(CastShapeCollector &c) {
    sort::insertion_sort(
        c.hits.data, c.hits.data + c.hits.length,
        [](const CastShapeHit &a, const CastShapeHit &b) { return early_out_fraction_of(a) < early_out_fraction_of(b); }, copy_hit);
}

// the inverted collector: a cast run with a and b swapped (and the displacement negated) reports
// through it, and it swaps the hit back before the collector it wraps sees it

/** the inverted collector's context: the collector it wraps and the world space displacement of the swapped cast */
struct InvertedContext {
    CastShapeCollector *base;
    Vec3f reversed_displacement;
};

void inverted_add_hit(CastShapeCollector &c, const CastShapeHit &h);
void inverted_add_miss(CastShapeCollector &c);

typedef void (*CastFn)(CastShapeCollector &collector, const CastShapeSettings &settings, const shapes::Shape *shape_a,
                       const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                       const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/** `fn` with a and b swapped and b swept by the negated displacement, its hits swapped back: the handler for the reverse pair */
template <CastFn fn>
void reversed_cast_shape_vs_shape(CastShapeCollector &collector, const CastShapeSettings &settings, const shapes::Shape *shape_a,
                                  const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                  Vec3f displacement_a, const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                  const Transformf &transform_b) {
    CastShapeCollector inverted = create_collector(collector::COLLECT_CUSTOM);
    inverted.body_id_b = collector.body_id_b;
    inverted.early_out_fraction = collector.early_out_fraction;
    inverted.custom_add_hit = inverted_add_hit;
    inverted.custom_add_miss = inverted_add_miss;
    const Vec3f displacement_b = vec3f::negate(displacement_a);
    InvertedContext context = {&collector, displacement_b};
    inverted.context = &context;
    fn(inverted, settings, shape_b, sub_shape_id_b, transform_b, displacement_b, shape_a, sub_shape_id_a, transform_a);
}

}  // namespace cast_shape

TUMBLE_NAMESPACE_END
