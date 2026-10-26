#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/collision/collector.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct CollidePointHit {
    /** EMPTY_SUB_SHAPE_ID unless shape b is a compound */
    SubShapeId sub_shape_id_b;
    BodyId body_id_b;
};

namespace collide_point {

inline CollidePointHit create_hit() { return {sub_shape::EMPTY_SUB_SHAPE_ID, body_id::INVALID_BODY_ID}; }

}  // namespace collide_point

struct CollidePointSettings {
    float collision_tolerance;
};

namespace collide_point {

inline CollidePointSettings default_settings() { return {1e-4f}; }

}  // namespace collide_point

struct CollidePointCollector;
typedef void (*CollidePointAddHit)(CollidePointCollector &collector, const CollidePointHit &hit);
typedef void (*CollidePointAddMiss)(CollidePointCollector &collector);

struct CollidePointCollector {
    CollectorKind kind;
    BodyId body_id_b;
    float early_out_fraction;
    collections::Array<CollidePointHit> hits;
    bool has_hit;
    CollidePointAddHit custom_add_hit;
    CollidePointAddMiss custom_add_miss;
    void *context;
};

namespace collide_point {

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

inline void add_hit(CollidePointCollector &c, const CollidePointHit &h) {
    switch (c.kind) {
        case CollectorKind::ALL:
            c.hits.push(h);
            c.has_hit = true;
            break;
        case CollectorKind::ANY:
            if (c.hits.length == 0) c.hits.push(h);
            else c.hits[0] = h;
            c.has_hit = true;
            c.early_out_fraction = -collector::UNBOUNDED;
            break;
        case CollectorKind::CLOSEST:
            // a point is in a shape or not, so the first hit is as close as any
            if (c.hits.length == 0) c.hits.push(h);
            else c.hits[0] = h;
            c.has_hit = true;
            c.early_out_fraction = -collector::UNBOUNDED;
            break;
        case CollectorKind::CUSTOM:
            c.custom_add_hit(c, h);
            break;
    }
}

inline void add_miss(CollidePointCollector &c) {
    if (c.kind == CollectorKind::CUSTOM && c.custom_add_miss != nullptr) c.custom_add_miss(c);
}

inline bool should_early_out(const CollidePointCollector &c) { return c.early_out_fraction <= -collector::UNBOUNDED; }

/** stops the query: for a custom collector's add_hit */
inline void force_early_out(CollidePointCollector &c) { c.early_out_fraction = -collector::UNBOUNDED; }

}  // namespace collide_point

TUMBLE_NAMESPACE_END
