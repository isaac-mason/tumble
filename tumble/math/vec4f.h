#pragma once

// the math package's vec4.ts on Vec4f: the arithmetic quat.ts borrows plus what crashcat calls. each function gives
// the bits of its scalar reference over Float4 (test/math/storage/vec4f.h); the dot is (x + z) + (y + w), as
// v4f::dot4 sums.

#include "math/scalar.h"
#include "math/simd.h"
#include "math/types.h"

namespace vec4f {

inline Vec4f load(const Float4 &a) { return {v4f::load4(&a.x)}; }
inline Float4 &store(Float4 &out, Vec4f a) {
    v4f::store4(&out.x, a.v);
    return out;
}

inline Vec4f make(float x, float y, float z, float w) { return {v4f::make(x, y, z, w)}; }
inline Vec4f splat(float s) { return {v4f::splat(s)}; }
inline Vec4f zero() { return {v4f::zero()}; }

inline float x(Vec4f a) { return v4f::lane<0>(a.v); }
inline float y(Vec4f a) { return v4f::lane<1>(a.v); }
inline float z(Vec4f a) { return v4f::lane<2>(a.v); }
inline float w(Vec4f a) { return v4f::lane<3>(a.v); }

inline Vec4f add(Vec4f a, Vec4f b) { return {v4f::add(a.v, b.v)}; }
inline Vec4f subtract(Vec4f a, Vec4f b) { return {v4f::sub(a.v, b.v)}; }
inline Vec4f multiply(Vec4f a, Vec4f b) { return {v4f::mul(a.v, b.v)}; }
inline Vec4f divide(Vec4f a, Vec4f b) { return {v4f::div(a.v, b.v)}; }
inline Vec4f min(Vec4f a, Vec4f b) { return {v4f::min(a.v, b.v)}; }
inline Vec4f max(Vec4f a, Vec4f b) { return {v4f::max(a.v, b.v)}; }
inline Vec4f negate(Vec4f a) { return {v4f::neg(a.v)}; }
inline Vec4f scale(Vec4f a, float b) { return {v4f::mul(a.v, v4f::splat(b))}; }
/** a + b * s. */
inline Vec4f scale_and_add(Vec4f a, Vec4f b, float s) { return {v4f::mul_add(b.v, v4f::splat(s), a.v)}; }

inline float dot(Vec4f a, Vec4f b) { return v4f::dot4(a.v, b.v); }
inline float squared_length(Vec4f a) { return dot(a, a); }
inline float length(Vec4f a) { return scalar::sqrt(squared_length(a)); }
inline float squared_distance(Vec4f a, Vec4f b) {
    const v128_t d = v4f::sub(b.v, a.v);
    return v4f::dot4(d, d);
}
inline float distance(Vec4f a, Vec4f b) { return scalar::sqrt(squared_distance(a, b)); }

/** the zero vector stays zero, as the js does. */
inline Vec4f normalize(Vec4f a) {
    float len = squared_length(a);
    if (len > 0.0f) len = 1.0f / scalar::sqrt(len);
    return scale(a, len);
}

inline Vec4f lerp(Vec4f a, Vec4f b, float t) { return {v4f::mul_add(v4f::sub(b.v, a.v), v4f::splat(t), a.v)}; }

inline bool exact_equals(Vec4f a, Vec4f b) { return v4f::all_true(v4f::eq(a.v, b.v)); }

inline bool equals(Vec4f a, Vec4f b) {
    return scalar::equals(x(a), x(b)) && scalar::equals(y(a), y(b)) && scalar::equals(z(a), z(b)) &&
           scalar::equals(w(a), w(b));
}

inline Vec4f div(Vec4f a, Vec4f b) { return divide(a, b); }
inline float len(Vec4f a) { return length(a); }
inline float sqr_len(Vec4f a) { return squared_length(a); }

}  // namespace vec4f
