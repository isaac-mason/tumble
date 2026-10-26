#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/collision/collector.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/world.h"
#include "tumble/util/collections.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

// the bodies whose bounds a query meets, under the filter, without looking at their shapes: jolt's BroadPhaseQuery. for
// the shapes themselves, query.h

struct BodyCollector;
typedef void (*BodyAddHit)(BodyCollector &collector, BodyId body_id);

/** the bodies an overlap query meets; overlaps have no distance, so CollectorKind::CLOSEST is not a kind it takes */
struct BodyCollector {
    CollectorKind kind;
    /** UNBOUNDED until the collector is done; an overlap has nothing for it to bound */
    float early_out_fraction;
    collections::Array<BodyId> hits;
    bool has_hit;
    BodyAddHit custom_add_hit;
    void *context;
};

namespace broadphase_query {

inline BodyCollector create_body_collector(CollectorKind kind) {
    TUMBLE_ASSERT(kind != CollectorKind::CLOSEST);
    BodyCollector c = {};
    c.kind = kind;
    c.early_out_fraction = collector::UNBOUNDED;
    return c;
}

inline void reset(BodyCollector &c) {
    c.early_out_fraction = collector::UNBOUNDED;
    c.hits.length = 0;
    c.has_hit = false;
}

inline void add_hit(BodyCollector &c, BodyId body_id) {
    switch (c.kind) {
        case CollectorKind::ALL:
            c.hits.push(body_id);
            c.has_hit = true;
            break;
        case CollectorKind::ANY:
            if (c.hits.length == 0) c.hits.push(body_id);
            else c.hits[0] = body_id;
            c.has_hit = true;
            c.early_out_fraction = -collector::UNBOUNDED;
            break;
        case CollectorKind::CUSTOM:
            c.custom_add_hit(c, body_id);
            break;
        default:
            break;
    }
}

inline bool should_early_out(const BodyCollector &c) { return c.early_out_fraction <= -collector::UNBOUNDED; }

/** stops the query: for a custom collector's add_hit */
inline void force_early_out(BodyCollector &c) { c.early_out_fraction = -collector::UNBOUNDED; }

}  // namespace broadphase_query

/** a body whose bounds a cast entered, and the fraction of the cast where */
struct BodyCastHit {
    BodyId body_id;
    float fraction;
};

struct BodyCastCollector;
typedef void (*BodyCastAddHit)(BodyCastCollector &collector, const BodyCastHit &hit);

/** the bodies a cast meets; a hit at or past early_out_fraction is not reported, and CollectorKind::CLOSEST lowers it each hit */
struct BodyCastCollector {
    CollectorKind kind;
    float early_out_fraction;
    collections::Array<BodyCastHit> hits;
    bool has_hit;
    BodyCastAddHit custom_add_hit;
    void *context;
};

namespace broadphase_query {

inline BodyCastCollector create_body_cast_collector(CollectorKind kind) {
    BodyCastCollector c = {};
    c.kind = kind;
    c.early_out_fraction = collector::INITIAL_EARLY_OUT_FRACTION;
    return c;
}

inline void reset(BodyCastCollector &c) {
    c.early_out_fraction = collector::INITIAL_EARLY_OUT_FRACTION;
    c.hits.length = 0;
    c.has_hit = false;
}

/** the closest or first hit, or null */
inline const BodyCastHit *hit(const BodyCastCollector &c) { return c.has_hit ? &c.hits[0] : nullptr; }

inline void add_hit(BodyCastCollector &c, const BodyCastHit &h) {
    switch (c.kind) {
        case CollectorKind::ALL:
            c.hits.push(h);
            c.has_hit = true;
            break;
        case CollectorKind::ANY:
            if (c.hits.length == 0) c.hits.push(h);
            else c.hits[0] = h;
            c.has_hit = true;
            c.early_out_fraction = collector::SHOULD_EARLY_OUT_FRACTION;
            break;
        case CollectorKind::CLOSEST:
            if (h.fraction >= c.early_out_fraction) break;
            if (c.hits.length == 0) c.hits.push(h);
            else c.hits[0] = h;
            c.has_hit = true;
            c.early_out_fraction = h.fraction;
            break;
        case CollectorKind::CUSTOM:
            c.custom_add_hit(c, h);
            break;
    }
}

inline bool should_early_out(const BodyCastCollector &c) { return c.early_out_fraction <= collector::SHOULD_EARLY_OUT_FRACTION; }

/** stops the query: for a custom collector's add_hit */
inline void force_early_out(BodyCastCollector &c) { c.early_out_fraction = collector::SHOULD_EARLY_OUT_FRACTION; }

/** a CollectorKind::ALL collector's hits, nearest first */
inline void sort(BodyCastCollector &c) {
    sort::insertion_sort(c.hits.data, c.hits.data + c.hits.length,
                         [](const BodyCastHit &a, const BodyCastHit &b) { return a.fraction < b.fraction; });
}

/** the bodies whose box the ray's segment enters */
void cast_ray(World &world, BodyCastCollector &collector, const Rayf &ray, const Filter &filter);

/** the bodies whose box overlaps the box, faces included */
void collide_aabb(World &world, BodyCollector &collector, const AABox &box, const Filter &filter);

/** the bodies whose box is within radius of the centre */
void collide_sphere(World &world, BodyCollector &collector, Vec3f center, float radius, const Filter &filter);

/** the bodies whose box holds the point, faces included */
void collide_point(World &world, BodyCollector &collector, Vec3f point, const Filter &filter);

/** the bodies whose box overlaps the oriented box */
void collide_oriented_box(World &world, BodyCollector &collector, const OrientedBox &box, const Filter &filter);

/** the bodies whose box the box meets as it moves by the displacement */
void cast_aabb(World &world, BodyCastCollector &collector, const AABox &box, Vec3f displacement, const Filter &filter);

}  // namespace broadphase_query

TUMBLE_NAMESPACE_END
