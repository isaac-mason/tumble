#pragma once

#include <stdint.h>

#include "tumble/math/scalar.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// a collector is a struct with a kind; COLLECT_CUSTOM hands its calls to function pointers with a context, which is how
// a wrapper chains to the collector it wraps. a query calls the collectors' free functions, never the pointers, and a
// shape reads early_out_fraction to skip work that cannot beat the best hit
namespace collector {

enum CollectorKind : uint32_t {
    /** every hit, in order, in `hits` */
    COLLECT_ALL = 0,
    /** the first hit at `hits[0]`, then stop */
    COLLECT_ANY = 1,
    /** the best hit at `hits[0]` (lowest fraction for a cast, deepest penetration for a collide), each new best tightening early_out_fraction */
    COLLECT_CLOSEST = 2,
    /** the function pointers decide */
    COLLECT_CUSTOM = 3,
};

/** a cast collector's early-out fraction before any hit: just past the end of the sweep. */
/** 1 + FLT_EPSILON: a hit a rounding error past the end of the cast still counts */
constexpr float INITIAL_EARLY_OUT_FRACTION = 1.0f + scalar::FLOAT_EPSILON;
/** the fraction a ray any-collector drops to, and the bound at or below which a cast collector stops. */
constexpr float SHOULD_EARLY_OUT_FRACTION = 0.0f;
/** a collide collector's early-out "fraction" is a penetration bound; this is unbounded. */
constexpr float UNBOUNDED = scalar::FLOAT_MAX;

}  // namespace collector

TUMBLE_NAMESPACE_END
