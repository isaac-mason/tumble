#pragma once

#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace vec4f {

inline Vec4f make(float x, float y, float z, float w) { return {v4f::make(x, y, z, w)}; }

inline float x(Vec4f a) { return v4f::lane<0>(a.v); }
inline float y(Vec4f a) { return v4f::lane<1>(a.v); }
inline float z(Vec4f a) { return v4f::lane<2>(a.v); }
inline float w(Vec4f a) { return v4f::lane<3>(a.v); }

/** the zero vector stays zero; the length is summed (x + z) + (y + w) */
inline Vec4f normalize(Vec4f a) {
    float len = v4f::dot4(a.v, a.v);
    if (len > 0.0f) len = 1.0f / scalar::sqrt(len);
    return {v4f::mul(a.v, v4f::splat(len))};
}

}  // namespace vec4f

TUMBLE_NAMESPACE_END
