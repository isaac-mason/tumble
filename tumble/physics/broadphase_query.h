#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/collision/collector.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/world.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

// the bodies whose bounds a query meets, under the filter, without looking at their shapes: jolt's BroadPhaseQuery. for
// the shapes themselves, query.h
namespace broadphase_query {

using collector::CollectorKind;

struct BodyCollector;
typedef void (*BodyAddHit)(BodyCollector &collector, body_id::BodyId body_id);
typedef bool (*BodyShouldEarlyOut)(const BodyCollector &collector);

/** the bodies an overlap query meets; overlaps have no distance, so COLLECT_CLOSEST is not a kind it takes */
struct BodyCollector {
    CollectorKind kind;
    collections::Vec<body_id::BodyId> hits;
    bool has_hit;
    BodyAddHit custom_add_hit;
    BodyShouldEarlyOut custom_should_early_out;
    void *context;
};

inline BodyCollector create_body_collector(CollectorKind kind) {
    TUMBLE_ASSERT(kind != collector::COLLECT_CLOSEST);
    BodyCollector c = {};
    c.kind = kind;
    return c;
}

inline void reset(BodyCollector &c) {
    c.hits.length = 0;
    c.has_hit = false;
}

inline void release(BodyCollector &c) { c.hits.release(); }

inline void add_hit(BodyCollector &c, body_id::BodyId body_id) {
    switch (c.kind) {
        case collector::COLLECT_ALL:
            c.hits.push(body_id);
            c.has_hit = true;
            break;
        case collector::COLLECT_ANY:
            if (c.hits.length == 0) c.hits.push(body_id);
            else c.hits[0] = body_id;
            c.has_hit = true;
            break;
        case collector::COLLECT_CUSTOM:
            c.custom_add_hit(c, body_id);
            break;
        default:
            break;
    }
}

inline bool should_early_out(const BodyCollector &c) {
    switch (c.kind) {
        case collector::COLLECT_ANY:
            return c.has_hit;
        case collector::COLLECT_CUSTOM:
            return c.custom_should_early_out != nullptr && c.custom_should_early_out(c);
        default:
            return false;
    }
}

/** a body whose bounds a cast entered, and the fraction of the cast where */
struct BodyCastHit {
    body_id::BodyId body_id;
    float fraction;
};

struct BodyCastCollector;
typedef void (*BodyCastAddHit)(BodyCastCollector &collector, const BodyCastHit &hit);
typedef bool (*BodyCastShouldEarlyOut)(const BodyCastCollector &collector);

/** the bodies a cast meets; a hit at or past early_out_fraction is not reported, and COLLECT_CLOSEST lowers it each hit */
struct BodyCastCollector {
    CollectorKind kind;
    float early_out_fraction;
    collections::Vec<BodyCastHit> hits;
    bool has_hit;
    BodyCastAddHit custom_add_hit;
    BodyCastShouldEarlyOut custom_should_early_out;
    void *context;
};

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

inline void release(BodyCastCollector &c) { c.hits.release(); }

/** the closest or first hit, or null */
inline const BodyCastHit *hit(const BodyCastCollector &c) { return c.has_hit ? &c.hits[0] : nullptr; }

inline void add_hit(BodyCastCollector &c, const BodyCastHit &h) {
    switch (c.kind) {
        case collector::COLLECT_ALL:
            c.hits.push(h);
            c.has_hit = true;
            break;
        case collector::COLLECT_ANY:
            if (c.hits.length == 0) c.hits.push(h);
            else c.hits[0] = h;
            c.has_hit = true;
            c.early_out_fraction = collector::SHOULD_EARLY_OUT_FRACTION;
            break;
        case collector::COLLECT_CLOSEST:
            if (h.fraction >= c.early_out_fraction) break;
            if (c.hits.length == 0) c.hits.push(h);
            else c.hits[0] = h;
            c.has_hit = true;
            c.early_out_fraction = h.fraction;
            break;
        case collector::COLLECT_CUSTOM:
            c.custom_add_hit(c, h);
            break;
    }
}

inline bool should_early_out(const BodyCastCollector &c) {
    switch (c.kind) {
        case collector::COLLECT_ANY:
            return c.has_hit;
        case collector::COLLECT_CUSTOM:
            return c.custom_should_early_out != nullptr && c.custom_should_early_out(c);
        default:
            return false;
    }
}

/** a COLLECT_ALL collector's hits, nearest first */
inline void sort(BodyCastCollector &c) {
    sort::insertion_sort(c.hits.data, c.hits.data + c.hits.length,
                         [](const BodyCastHit &a, const BodyCastHit &b) { return a.fraction < b.fraction; });
}

/** the bodies whose box the ray's segment enters */
void cast_ray(world::World &world, BodyCastCollector &collector, const cast_ray::Rayf &ray, const filter::Filter &filter);

/** the bodies whose box overlaps the box, faces included */
void collide_aabb(world::World &world, BodyCollector &collector, const AABox &box, const filter::Filter &filter);

/** the bodies whose box is within radius of the centre */
void collide_sphere(world::World &world, BodyCollector &collector, Vec3f center, float radius, const filter::Filter &filter);

/** the bodies whose box holds the point, faces included */
void collide_point(world::World &world, BodyCollector &collector, Vec3f point, const filter::Filter &filter);

/** the bodies whose box overlaps the oriented box */
void collide_oriented_box(world::World &world, BodyCollector &collector, const OrientedBox &box, const filter::Filter &filter);

/** the bodies whose box the box meets as it moves by the displacement */
void cast_aabb(world::World &world, BodyCastCollector &collector, const AABox &box, Vec3f displacement, const filter::Filter &filter);

}  // namespace broadphase_query

TUMBLE_NAMESPACE_END
