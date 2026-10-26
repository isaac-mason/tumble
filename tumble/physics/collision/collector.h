#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/scalar.h"

TUMBLE_NAMESPACE_BEGIN

// a collector is a struct with a kind; CollectorKind::CUSTOM hands its hits and misses to function pointers with a context,
// which is how a wrapper chains to the collector it wraps. a query calls the collectors' free functions, never the
// pointers. a shape reads early_out_fraction to skip work that cannot beat the best hit, and a query stops once it is at
// or below the collector's bound; a custom collector stops one with force_early_out, a wrapper by copying the fraction
// of the collector it wraps

enum class CollectorKind : uint32_t {
    /** every hit, in order, in `hits` */
    ALL = 0,
    /** the first hit at `hits[0]`, then stop */
    ANY = 1,
    /** the best hit at `hits[0]` (lowest fraction for a cast, deepest penetration for a collide), each new best tightening early_out_fraction */
    CLOSEST = 2,
    /** the function pointers decide */
    CUSTOM = 3,
};

namespace collector {

/** a cast collector's early-out fraction before any hit: just past the end of the sweep. */
/** 1 + FLT_EPSILON: a hit a rounding error past the end of the cast still counts */
constexpr float INITIAL_EARLY_OUT_FRACTION = 1.0f + scalar::FLOAT_EPSILON;
/** the bound at or below which a ray or a broadphase cast stops */
constexpr float SHOULD_EARLY_OUT_FRACTION = 0.0f;
/** a collide collector's early-out "fraction" is a penetration bound; this is unbounded, and -UNBOUNDED the bound at or
 * below which a shape cast, a collide, a point collide or a broadphase overlap stops */
constexpr float UNBOUNDED = scalar::FLOAT_MAX;

}  // namespace collector

TUMBLE_NAMESPACE_END
