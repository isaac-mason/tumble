#pragma once

// the scalar functions over the storage types, retired from src when the register types took over: the reference the
// register functions are checked against, bit for bit.

#include "math/vec4f.h"

namespace vec4f {

inline Float4 create() { return {0.0f, 0.0f, 0.0f, 0.0f}; }
inline Float4 clone(const Float4 &a) { return a; }
inline Float4 from_values(float x, float y, float z, float w) { return {x, y, z, w}; }

inline Float4 &copy(Float4 &out, const Float4 &a) {
    out = a;
    return out;
}

inline Float4 &set(Float4 &out, float x, float y, float z, float w) {
    out.x = x;
    out.y = y;
    out.z = z;
    out.w = w;
    return out;
}

inline Float4 &zero(Float4 &out) { return set(out, 0.0f, 0.0f, 0.0f, 0.0f); }

inline Float4 &from_buffer(Float4 &out, const float *buffer, uint32_t start) {
    return set(out, buffer[start], buffer[start + 1], buffer[start + 2], buffer[start + 3]);
}

inline void to_buffer(float *buffer, const Float4 &v, uint32_t start) {
    buffer[start] = v.x;
    buffer[start + 1] = v.y;
    buffer[start + 2] = v.z;
    buffer[start + 3] = v.w;
}

inline Float4 &store(Float4 &out, v128_t v) {
    v4f::store4(&out.x, v);
    return out;
}

inline Float4 &add(Float4 &out, const Float4 &a, const Float4 &b) { return store(out, v4f::add(v4f::load4(&a.x), v4f::load4(&b.x))); }
inline Float4 &subtract(Float4 &out, const Float4 &a, const Float4 &b) { return store(out, v4f::sub(v4f::load4(&a.x), v4f::load4(&b.x))); }
inline Float4 &multiply(Float4 &out, const Float4 &a, const Float4 &b) { return store(out, v4f::mul(v4f::load4(&a.x), v4f::load4(&b.x))); }
inline Float4 &divide(Float4 &out, const Float4 &a, const Float4 &b) { return store(out, v4f::div(v4f::load4(&a.x), v4f::load4(&b.x))); }
inline Float4 &min(Float4 &out, const Float4 &a, const Float4 &b) { return store(out, v4f::min(v4f::load4(&a.x), v4f::load4(&b.x))); }
inline Float4 &max(Float4 &out, const Float4 &a, const Float4 &b) { return store(out, v4f::max(v4f::load4(&a.x), v4f::load4(&b.x))); }
inline Float4 &negate(Float4 &out, const Float4 &a) { return store(out, v4f::neg(v4f::load4(&a.x))); }
inline Float4 &scale(Float4 &out, const Float4 &a, float b) { return store(out, v4f::mul(v4f::load4(&a.x), v4f::splat(b))); }
inline Float4 &scale_and_add(Float4 &out, const Float4 &a, const Float4 &b, float s) {
    return store(out, v4f::mul_add(v4f::load4(&b.x), v4f::splat(s), v4f::load4(&a.x)));
}

inline float dot(const Float4 &a, const Float4 &b) { return v4f::dot4(v4f::load4(&a.x), v4f::load4(&b.x)); }
inline float squared_length(const Float4 &a) { return dot(a, a); }
inline float length(const Float4 &a) { return scalar::sqrt(squared_length(a)); }

inline float squared_distance(const Float4 &a, const Float4 &b) {
    const v128_t d = v4f::sub(v4f::load4(&b.x), v4f::load4(&a.x));
    return v4f::dot4(d, d);
}
inline float distance(const Float4 &a, const Float4 &b) { return scalar::sqrt(squared_distance(a, b)); }

/** the zero vector stays zero, as the js does. */
inline Float4 &normalize(Float4 &out, const Float4 &a) {
    float len = squared_length(a);
    if (len > 0.0f) len = 1.0f / scalar::sqrt(len);
    return scale(out, a, len);
}

inline Float4 &lerp(Float4 &out, const Float4 &a, const Float4 &b, float t) {
    const v128_t va = v4f::load4(&a.x);
    return store(out, v4f::mul_add(v4f::sub(v4f::load4(&b.x), va), v4f::splat(t), va));
}

inline bool exact_equals(const Float4 &a, const Float4 &b) { return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w; }

inline bool equals(const Float4 &a, const Float4 &b) {
    return scalar::equals(a.x, b.x) && scalar::equals(a.y, b.y) && scalar::equals(a.z, b.z) && scalar::equals(a.w, b.w);
}

inline Float4 &sub(Float4 &out, const Float4 &a, const Float4 &b) { return subtract(out, a, b); }
inline Float4 &mul(Float4 &out, const Float4 &a, const Float4 &b) { return multiply(out, a, b); }
inline Float4 &div(Float4 &out, const Float4 &a, const Float4 &b) { return divide(out, a, b); }
inline float len(const Float4 &a) { return length(a); }
inline float sqr_len(const Float4 &a) { return squared_length(a); }

}  // namespace vec4f
