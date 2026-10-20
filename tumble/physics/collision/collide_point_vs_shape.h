#pragma once

// crashcat's collide-point-vs-shape.ts, the types: a point query's hit, settings and collector.
// the query itself (dispatch to the shape's collide_point) is collide_point_vs_shape.cpp.

#include <stdint.h>

#include "physics/body/body_id.h"
#include "physics/body/sub_shape.h"
#include "physics/collision/collector.h"
#include "util/collections.h"

namespace collide_point {

using collector::CollectorKind;

struct CollidePointHit {
    /** EMPTY_SUB_SHAPE_ID unless shape b is a compound */
    sub_shape::SubShapeId sub_shape_id_b;
    body_id::BodyId body_id_b;
    int32_t material_id;
};

inline CollidePointHit create_hit() { return {sub_shape::EMPTY_SUB_SHAPE_ID, body_id::INVALID_BODY_ID, -1}; }

struct CollidePointSettings {
    float collision_tolerance;
};

inline CollidePointSettings default_settings() { return {1e-4f}; }

struct CollidePointCollector;
typedef void (*CollidePointAddHit)(CollidePointCollector &collector, const CollidePointHit &hit);
typedef void (*CollidePointAddMiss)(CollidePointCollector &collector);
typedef bool (*CollidePointShouldEarlyOut)(const CollidePointCollector &collector);

struct CollidePointCollector {
    CollectorKind kind;
    body_id::BodyId body_id_b;
    float early_out_fraction;
    collections::Vec<CollidePointHit> hits;
    bool has_hit;
    CollidePointAddHit custom_add_hit;
    CollidePointAddMiss custom_add_miss;
    CollidePointShouldEarlyOut custom_should_early_out;
    void *context;
};

inline CollidePointCollector create_collector(CollectorKind kind) {
    CollidePointCollector c = {};
    c.kind = kind;
    c.body_id_b = body_id::INVALID_BODY_ID;
    c.early_out_fraction = collector::UNBOUNDED;
    return c;
}

inline const CollidePointHit *hit(const CollidePointCollector &c) { return c.has_hit ? &c.hits[0] : nullptr; }

inline void reset(CollidePointCollector &c) {
    c.body_id_b = body_id::INVALID_BODY_ID;
    c.early_out_fraction = collector::UNBOUNDED;
    c.hits.length = 0;
    c.has_hit = false;
}

inline void release(CollidePointCollector &c) { c.hits.release(); }

inline void add_hit(CollidePointCollector &c, const CollidePointHit &h) {
    switch (c.kind) {
        case collector::COLLECT_ALL:
            c.hits.push(h);
            c.has_hit = true;
            break;
        case collector::COLLECT_ANY:
            if (c.hits.length == 0) c.hits.push(h);
            else c.hits[0] = h;
            c.has_hit = true;
            c.early_out_fraction = -collector::UNBOUNDED;
            break;
        case collector::COLLECT_CLOSEST:
            if (c.hits.length == 0) c.hits.push(h);
            else c.hits[0] = h;
            c.has_hit = true;
            c.early_out_fraction = 0.0f;
            break;
        case collector::COLLECT_CUSTOM:
            c.custom_add_hit(c, h);
            break;
    }
}

inline void add_miss(CollidePointCollector &c) {
    if (c.kind == collector::COLLECT_CUSTOM && c.custom_add_miss != nullptr) c.custom_add_miss(c);
}

inline bool should_early_out(const CollidePointCollector &c) {
    switch (c.kind) {
        case collector::COLLECT_ANY:
        case collector::COLLECT_CLOSEST:
            return c.has_hit;
        case collector::COLLECT_CUSTOM:
            return c.custom_should_early_out(c);
        default:
            return false;
    }
}

}  // namespace collide_point
