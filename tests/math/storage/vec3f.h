#pragma once

#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace vec3f {

inline Float3 from_values(float x, float y, float z) { return {x, y, z}; }

inline Float3 &set(Float3 &out, float x, float y, float z) {
    out.x = x;
    out.y = y;
    out.z = z;
    return out;
}

inline Float3 &zero(Float3 &out) { return set(out, 0.0f, 0.0f, 0.0f); }

inline Float3 &store(Float3 &out, v128_t v) {
    v4f::store3(&out.x, v);
    return out;
}

inline Float3 &add(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::add(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &subtract(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::sub(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &multiply(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::mul(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &divide(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::div(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &min(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::min(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &max(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::max(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &negate(Float3 &out, const Float3 &a) { return store(out, v4f::neg(v4f::load3(&a.x))); }
inline Float3 &scale(Float3 &out, const Float3 &a, float b) { return store(out, v4f::mul(v4f::load3(&a.x), v4f::splat(b))); }
inline Float3 &scale_and_add(Float3 &out, const Float3 &a, const Float3 &b, float s) {
    return store(out, v4f::mul_add(v4f::load3(&b.x), v4f::splat(s), v4f::load3(&a.x)));
}

inline Float3 &inverse(Float3 &out, const Float3 &a) { return set(out, 1.0f / a.x, 1.0f / a.y, 1.0f / a.z); }

inline float dot(const Float3 &a, const Float3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float squared_length(const Float3 &a) { return dot(a, a); }
inline float length(const Float3 &a) { return scalar::sqrt(squared_length(a)); }

inline float squared_distance(const Float3 &a, const Float3 &b) {
    const float x = b.x - a.x;
    const float y = b.y - a.y;
    const float z = b.z - a.z;
    return x * x + y * y + z * z;
}
inline float distance(const Float3 &a, const Float3 &b) { return scalar::sqrt(squared_distance(a, b)); }

/** the zero vector stays zero, as the js does. */
inline Float3 &normalize(Float3 &out, const Float3 &a) {
    float len = squared_length(a);
    if (len > 0.0f) len = 1.0f / scalar::sqrt(len);
    return scale(out, a, len);
}

inline Float3 &cross(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, cross_lanes(v4f::load3(&a.x), v4f::load3(&b.x))); }

/** a unit vector perpendicular to a; +x for the zero vector, since every direction is. */
inline Float3 &perpendicular(Float3 &out, const Float3 &a) {
    if (scalar::abs(a.x) > scalar::abs(a.y)) {
        const float inv_len = 1.0f / scalar::sqrt(a.x * a.x + a.z * a.z);
        return set(out, a.z * inv_len, 0.0f, -a.x * inv_len);
    }
    const float len = scalar::sqrt(a.y * a.y + a.z * a.z);
    if (len == 0.0f) return set(out, 1.0f, 0.0f, 0.0f);
    const float inv_len = 1.0f / len;
    return set(out, 0.0f, a.z * inv_len, -a.y * inv_len);
}

inline Float3 &lerp(Float3 &out, const Float3 &a, const Float3 &b, float t) {
    const v128_t va = v4f::load3(&a.x);
    return store(out, v4f::mul_add(v4f::sub(v4f::load3(&b.x), va), v4f::splat(t), va));
}

/** a as a point through m, with the perspective divide; a zero (or nan) w divides by one, as the js `w || 1`. */
inline Float3 &transform_mat4(Float3 &out, const Float3 &a, const Float4x4 &m) {
    v128_t r = v4f::mul(v4f::load4(&m.m[0]), v4f::splat(a.x));
    r = v4f::mul_add(v4f::load4(&m.m[4]), v4f::splat(a.y), r);
    r = v4f::mul_add(v4f::load4(&m.m[8]), v4f::splat(a.z), r);
    r = v4f::add(r, v4f::load4(&m.m[12]));
    float w = v4f::lane<3>(r);
    if (w == 0.0f || w != w) w = 1.0f;
    return store(out, v4f::div(r, v4f::splat(w)));
}

/** rotates a by q: a + 2w (q x a) + 2 (q x (q x a)). */
inline Float3 &transform_quat(Float3 &out, const Float3 &a, const Float4 &q) {
    const v128_t qv = v4f::load4(&q.x);
    const v128_t va = v4f::load3(&a.x);
    const v128_t uv = cross_lanes(qv, va);
    const v128_t uuv = cross_lanes(qv, uv);
    v128_t r = v4f::mul_add(uv, v4f::splat(q.w * 2.0f), va);
    r = v4f::mul_add(uuv, v4f::splat(2.0f), r);
    return store(out, r);
}

inline bool exact_equals(const Float3 &a, const Float3 &b) { return a.x == b.x && a.y == b.y && a.z == b.z; }


/** an odd number of negative components flips handedness. */
inline bool is_scale_inside_out(const Float3 &scale) {
    const uint32_t mask = (scale.x < 0.0f ? 1u : 0u) | (scale.y < 0.0f ? 2u : 0u) | (scale.z < 0.0f ? 4u : 0u);
    return (__builtin_popcount(mask) & 1u) != 0u;
}

// the math package's short names, kept so a ported file reads like its source
}  // namespace vec3f

TUMBLE_NAMESPACE_END
