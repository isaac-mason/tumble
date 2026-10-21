#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/collision/collector.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

namespace cast_ray {

using collector::CollectorKind;

/** a ray as every cast takes it: origin, unit direction, length. */
struct Rayf {
    Vec3f origin;
    Vec3f direction;
    float length;
};

enum CastRayStatus : uint32_t {
    NOT_COLLIDING = 0,
    COLLIDING = 1,
};

struct CastRayHit {
    CastRayStatus status;
    /** along the ray: 0 at the origin, 1 at length */
    float fraction;
    sub_shape::SubShapeId sub_shape_id;
    body_id::BodyId body_id_b;
};

inline CastRayHit create_hit() { return {NOT_COLLIDING, 1.0f, sub_shape::EMPTY_SUB_SHAPE_ID, body_id::INVALID_BODY_ID}; }

struct CastRaySettings {
    /** back facing triangles (triangle meshes) are hit, otherwise they are ignored */
    bool collide_with_backfaces_triangles;
    /** convex shapes (and planes) are also hit where the ray exits them, otherwise only where it enters */
    bool collide_with_backfaces_convex;
    /** a ray starting inside a convex shape hits at fraction 0; otherwise only the surface hits */
    bool treat_convex_as_solid;
};

inline CastRaySettings default_settings() { return {false, false, true}; }

struct CastRayCollector;
typedef void (*CastRayAddHit)(CastRayCollector &collector, const CastRayHit &hit);
typedef void (*CastRayAddMiss)(CastRayCollector &collector);
typedef bool (*CastRayShouldEarlyOut)(const CastRayCollector &collector);

struct CastRayCollector {
    CollectorKind kind;
    body_id::BodyId body_id_b;
    /** a shape may skip anything that cannot hit before this fraction */
    float early_out_fraction;
    /** ALL: every hit in order. ANY and CLOSEST: one entry once has_hit. */
    collections::Vec<CastRayHit> hits;
    bool has_hit;
    /** CUSTOM: the behaviour, with whatever it needs in context (a wrapped collector, say) */
    CastRayAddHit custom_add_hit;
    CastRayAddMiss custom_add_miss;
    CastRayShouldEarlyOut custom_should_early_out;
    void *context;
};

inline CastRayCollector create_collector(CollectorKind kind) {
    CastRayCollector c = {};
    c.kind = kind;
    c.body_id_b = body_id::INVALID_BODY_ID;
    c.early_out_fraction = collector::INITIAL_EARLY_OUT_FRACTION;
    return c;
}

/** the first hit, for ANY and CLOSEST; null before any. */
inline const CastRayHit *hit(const CastRayCollector &c) { return c.has_hit ? &c.hits[0] : nullptr; }

inline void reset(CastRayCollector &c) {
    c.body_id_b = body_id::INVALID_BODY_ID;
    c.early_out_fraction = collector::INITIAL_EARLY_OUT_FRACTION;
    c.hits.length = 0;
    c.has_hit = false;
}

inline void release(CastRayCollector &c) { c.hits.release(); }

/** keeps one hit at hits[0]. */
inline void keep_one(CastRayCollector &c, const CastRayHit &h) {
    if (c.hits.length == 0) c.hits.push(h);
    else c.hits[0] = h;
    c.has_hit = true;
}

inline void add_hit(CastRayCollector &c, const CastRayHit &h) {
    switch (c.kind) {
        case collector::COLLECT_ALL:
            c.hits.push(h);
            c.has_hit = true;
            break;
        case collector::COLLECT_ANY:
            keep_one(c, h);
            c.early_out_fraction = collector::SHOULD_EARLY_OUT_FRACTION;
            break;
        case collector::COLLECT_CLOSEST:
            if (!c.has_hit || h.fraction < c.hits[0].fraction) {
                keep_one(c, h);
                c.early_out_fraction = h.fraction;
            }
            break;
        case collector::COLLECT_CUSTOM:
            c.custom_add_hit(c, h);
            break;
    }
}

inline void add_miss(CastRayCollector &c) {
    if (c.kind == collector::COLLECT_CUSTOM && c.custom_add_miss != nullptr) c.custom_add_miss(c);
}

inline bool should_early_out(const CastRayCollector &c) {
    switch (c.kind) {
        case collector::COLLECT_ANY:
            return c.has_hit;
        case collector::COLLECT_CUSTOM:
            return c.custom_should_early_out(c);
        default:
            return false;
    }
}

/** stable, closest first */
inline void sort(CastRayCollector &c) {
    sort::insertion_sort(c.hits.data, c.hits.data + c.hits.length,
                         [](const CastRayHit &a, const CastRayHit &b) { return a.fraction < b.fraction; });
}

}  // namespace cast_ray

TUMBLE_NAMESPACE_END
