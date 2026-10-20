#pragma once

// the math package's triangle3.ts: functions over three points, no Triangle3f record needed to call them.

#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/math/types.h"
#include "math/storage/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace triangle3f {

inline Box3f &bounds(Box3f &out, const Float3 &a, const Float3 &b, const Float3 &c) {
    const v128_t va = v4f::load3(&a.x);
    const v128_t vb = v4f::load3(&b.x);
    const v128_t vc = v4f::load3(&c.x);
    vec3f::store(out.min, v4f::min(va, v4f::min(vb, vc)));
    vec3f::store(out.max, v4f::max(va, v4f::max(vb, vc)));
    return out;
}

/** the unit normal of (b - a) x (c - a); zero for a degenerate triangle. */
inline Float3 &normal(Float3 &out, const Float3 &a, const Float3 &b, const Float3 &c) {
    const v128_t va = v4f::load3(&a.x);
    const v128_t n = v4f::cross(v4f::sub(v4f::load3(&b.x), va), v4f::sub(v4f::load3(&c.x), va));
    const float length = scalar::sqrt(v4f::dot3(n, n));
    return vec3f::store(out, length > 0.0f ? v4f::div(n, v4f::splat(length)) : n);
}

inline Float3 &centroid(Float3 &out, const Float3 &a, const Float3 &b, const Float3 &c) {
    return vec3f::store(out, v4f::div(v4f::add(v4f::load3(&a.x), v4f::add(v4f::load3(&b.x), v4f::load3(&c.x))), v4f::splat(3.0f)));
}

}  // namespace triangle3f

TUMBLE_NAMESPACE_END
