#pragma once

#include "tumble/math/vec4f.h"

TUMBLE_NAMESPACE_BEGIN

namespace vec4f {

inline Float4 &store(Float4 &out, v128_t v) {
    v4f::store4(&out.x, v);
    return out;
}

inline Float4 &scale(Float4 &out, const Float4 &a, float b) { return store(out, v4f::mul(v4f::load4(&a.x), v4f::splat(b))); }
inline float dot(const Float4 &a, const Float4 &b) { return v4f::dot4(v4f::load4(&a.x), v4f::load4(&b.x)); }
inline float squared_length(const Float4 &a) { return dot(a, a); }
/** the zero vector stays zero */
inline Float4 &normalize(Float4 &out, const Float4 &a) {
    float len = squared_length(a);
    if (len > 0.0f) len = 1.0f / scalar::sqrt(len);
    return scale(out, a, len);
}

}  // namespace vec4f

TUMBLE_NAMESPACE_END
