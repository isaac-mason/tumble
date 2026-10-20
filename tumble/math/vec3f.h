#pragma once

// the math package's vec3.ts on Vec3f, by value, in snake case. load and store cross to and from a record's
// Float3. each function gives the bits of its scalar reference over Float3 (test/math/storage/vec3f.h):
// component-wise ops are lane ops, horizontal ones keep the scalar order (the dot is (x + y) + z).

#include "math/scalar.h"
#include "math/simd.h"
#include "math/types.h"

namespace vec3f {

/** lanes (x, y, z, z) from a value whose lanes 0..2 are x, y, z. */
inline v128_t fix_w(v128_t v) { return v4f::shuffle<0, 1, 2, 2>(v); }

inline Vec3f load(const Float3 &a) {
    const v128_t xy = wasm_v128_load64_zero(&a.x);
    const v128_t zz = wasm_v128_load32_splat(&a.z);
    return {v4f::shuffle2<0, 1, 4, 4>(xy, zz)};
}
inline Float3 &store(Float3 &out, Vec3f a) {
    v4f::store3(&out.x, a.v);
    return out;
}
/** three floats at buffer[start..start + 2]. */
inline Vec3f load(const float *buffer, uint32_t start) { return load(*reinterpret_cast<const Float3 *>(buffer + start)); }
inline void store(float *buffer, Vec3f a, uint32_t start) { v4f::store3(buffer + start, a.v); }

inline Vec3f make(float x, float y, float z) { return {v4f::make(x, y, z, z)}; }
inline Vec3f splat(float s) { return {v4f::splat(s)}; }
inline Vec3f zero() { return {v4f::zero()}; }
inline Vec3f axis_x() { return make(1.0f, 0.0f, 0.0f); }
inline Vec3f axis_y() { return make(0.0f, 1.0f, 0.0f); }
inline Vec3f axis_z() { return make(0.0f, 0.0f, 1.0f); }

inline float x(Vec3f a) { return v4f::lane<0>(a.v); }
inline float y(Vec3f a) { return v4f::lane<1>(a.v); }
inline float z(Vec3f a) { return v4f::lane<2>(a.v); }
inline Vec3f with_x(Vec3f a, float s) { return {v4f::set_lane<0>(a.v, s)}; }
inline Vec3f with_y(Vec3f a, float s) { return {v4f::set_lane<1>(a.v, s)}; }
inline Vec3f with_z(Vec3f a, float s) { return {v4f::shuffle2<0, 1, 4, 4>(a.v, v4f::splat(s))}; }

inline Vec3f add(Vec3f a, Vec3f b) { return {v4f::add(a.v, b.v)}; }
inline Vec3f add_scalar(Vec3f a, float b) { return {v4f::add(a.v, v4f::splat(b))}; }
inline Vec3f subtract(Vec3f a, Vec3f b) { return {v4f::sub(a.v, b.v)}; }
inline Vec3f subtract_scalar(Vec3f a, float b) { return {v4f::sub(a.v, v4f::splat(b))}; }
inline Vec3f multiply(Vec3f a, Vec3f b) { return {v4f::mul(a.v, b.v)}; }
inline Vec3f divide(Vec3f a, Vec3f b) { return {v4f::div(a.v, b.v)}; }
/** b < a ? b : a per lane, as the storage min. */
inline Vec3f min(Vec3f a, Vec3f b) { return {v4f::min(a.v, b.v)}; }
/** a < b ? b : a per lane, as the storage max. */
inline Vec3f max(Vec3f a, Vec3f b) { return {v4f::max(a.v, b.v)}; }
/** scalar::min per lane (f32.min). */
inline Vec3f fmin(Vec3f a, Vec3f b) { return {v4f::fmin(a.v, b.v)}; }
/** scalar::max per lane (f32.max). */
inline Vec3f fmax(Vec3f a, Vec3f b) { return {v4f::fmax(a.v, b.v)}; }
inline Vec3f negate(Vec3f a) { return {v4f::neg(a.v)}; }
inline Vec3f abs(Vec3f a) { return {v4f::abs(a.v)}; }
inline Vec3f scale(Vec3f a, float b) { return {v4f::mul(a.v, v4f::splat(b))}; }
/** a + b * s. */
inline Vec3f scale_and_add(Vec3f a, Vec3f b, float s) { return {v4f::mul_add(b.v, v4f::splat(s), a.v)}; }

inline Vec3f ceil(Vec3f a) { return {wasm_f32x4_ceil(a.v)}; }
inline Vec3f floor(Vec3f a) { return {wasm_f32x4_floor(a.v)}; }
/** symmetric round per component, as scalar::round. */
inline Vec3f round(Vec3f a) { return make(scalar::round(x(a)), scalar::round(y(a)), scalar::round(z(a))); }

inline Vec3f inverse(Vec3f a) { return {v4f::div(v4f::splat(1.0f), a.v)}; }

inline float dot(Vec3f a, Vec3f b) {
    const v128_t m = v4f::mul(a.v, b.v);
    return v4f::lane<0>(m) + v4f::lane<1>(m) + v4f::lane<2>(m);
}
/** (x + y) + z. */
inline float reduce_sum(Vec3f a) { return x(a) + y(a) + z(a); }
inline float squared_length(Vec3f a) { return dot(a, a); }
inline float length(Vec3f a) { return scalar::sqrt(squared_length(a)); }
inline float squared_distance(Vec3f a, Vec3f b) {
    const Vec3f d = subtract(b, a);
    return dot(d, d);
}
inline float distance(Vec3f a, Vec3f b) { return scalar::sqrt(squared_distance(a, b)); }

/** the zero vector stays zero, as the js does. */
inline Vec3f normalize(Vec3f a) {
    float len = squared_length(a);
    if (len > 0.0f) len = 1.0f / scalar::sqrt(len);
    return scale(a, len);
}

/** a x b over lanes 0..2 of a and b, lane 3 a copy of z. */
inline v128_t cross_lanes(v128_t a, v128_t b) {
    return v4f::sub(v4f::mul(v4f::shuffle<1, 2, 0, 0>(a), v4f::shuffle<2, 0, 1, 1>(b)),
                    v4f::mul(v4f::shuffle<2, 0, 1, 1>(a), v4f::shuffle<1, 2, 0, 0>(b)));
}
inline Vec3f cross(Vec3f a, Vec3f b) { return {cross_lanes(a.v, b.v)}; }

/** a unit vector perpendicular to a; +x for the zero vector, since every direction is. */
inline Vec3f perpendicular(Vec3f a) {
    const float ax = x(a), ay = y(a), az = z(a);
    if (scalar::abs(ax) > scalar::abs(ay)) {
        const float inv_len = 1.0f / scalar::sqrt(ax * ax + az * az);
        return make(az * inv_len, 0.0f, -ax * inv_len);
    }
    const float len = scalar::sqrt(ay * ay + az * az);
    if (len == 0.0f) return axis_x();
    const float inv_len = 1.0f / len;
    return make(0.0f, az * inv_len, -ay * inv_len);
}

/** a unit vector perpendicular to a, divided by its length (jolt's GetNormalizedPerpendicular); a must not be zero. */
inline Vec3f normalized_perpendicular(Vec3f a) {
    const float ax = x(a), ay = y(a), az = z(a);
    if (scalar::abs(ax) > scalar::abs(ay)) {
        const float len = scalar::sqrt(ax * ax + az * az);
        return make(az / len, 0.0f, -ax / len);
    }
    const float len = scalar::sqrt(ay * ay + az * az);
    return make(0.0f, az / len, -ay / len);
}

inline Vec3f lerp(Vec3f a, Vec3f b, float t) { return {v4f::mul_add(v4f::sub(b.v, a.v), v4f::splat(t), a.v)}; }

/** quadratic through a at t=0, b at t=0.5, c at t=1. */
inline Vec3f lagrange(Vec3f a, Vec3f b, Vec3f c, float t) {
    const float c0 = 2.0f * (t - 1.0f) * (t - 0.5f);
    const float c1 = -4.0f * (t - 1.0f) * t;
    const float c2 = 2.0f * (t - 0.5f) * t;
    v128_t r = v4f::mul(a.v, v4f::splat(c0));
    r = v4f::mul_add(b.v, v4f::splat(c1), r);
    r = v4f::mul_add(c.v, v4f::splat(c2), r);
    return {r};
}

/** spherical interpolation between unit vectors. */
inline Vec3f slerp(Vec3f a, Vec3f b, float t) {
    const float angle = scalar::acos(scalar::min(scalar::max(dot(a, b), -1.0f), 1.0f));
    const float sin_total = scalar::sin(angle);
    const float ratio_a = scalar::sin((1.0f - t) * angle) / sin_total;
    const float ratio_b = scalar::sin(t * angle) / sin_total;
    return {v4f::mul_add(b.v, v4f::splat(ratio_b), v4f::mul(a.v, v4f::splat(ratio_a)))};
}

inline Vec3f hermite(Vec3f a, Vec3f b, Vec3f c, Vec3f d, float t) {
    const float t2 = t * t;
    const float f1 = t2 * (2.0f * t - 3.0f) + 1.0f;
    const float f2 = t2 * (t - 2.0f) + t;
    const float f3 = t2 * (t - 1.0f);
    const float f4 = t2 * (3.0f - 2.0f * t);
    v128_t r = v4f::mul(a.v, v4f::splat(f1));
    r = v4f::mul_add(b.v, v4f::splat(f2), r);
    r = v4f::mul_add(c.v, v4f::splat(f3), r);
    r = v4f::mul_add(d.v, v4f::splat(f4), r);
    return {r};
}

inline Vec3f bezier(Vec3f a, Vec3f b, Vec3f c, Vec3f d, float t) {
    const float inv = 1.0f - t;
    const float inv2 = inv * inv;
    const float t2 = t * t;
    const float f1 = inv2 * inv;
    const float f2 = 3.0f * t * inv2;
    const float f3 = 3.0f * t2 * inv;
    const float f4 = t2 * t;
    v128_t r = v4f::mul(a.v, v4f::splat(f1));
    r = v4f::mul_add(b.v, v4f::splat(f2), r);
    r = v4f::mul_add(c.v, v4f::splat(f3), r);
    r = v4f::mul_add(d.v, v4f::splat(f4), r);
    return {r};
}

/** a as a point through m, with the perspective divide; a zero (or nan) w divides by one, as the js `w || 1`. */
inline Vec3f transform_mat4(Vec3f a, const Mat4f &m) {
    v128_t r = v4f::mul(m.c[0], v4f::splat(x(a)));
    r = v4f::mul_add(m.c[1], v4f::splat(y(a)), r);
    r = v4f::mul_add(m.c[2], v4f::splat(z(a)), r);
    r = v4f::add(r, m.c[3]);
    float w = v4f::lane<3>(r);
    if (w == 0.0f || w != w) w = 1.0f;
    return {fix_w(v4f::div(r, v4f::splat(w)))};
}

inline Vec3f transform_mat3(Vec3f a, const Mat3f &m) {
    const float ax = x(a), ay = y(a), az = z(a);
    return make(ax * m.m[0] + ay * m.m[3] + az * m.m[6], ax * m.m[1] + ay * m.m[4] + az * m.m[7],
                ax * m.m[2] + ay * m.m[5] + az * m.m[8]);
}

/** rotates a by q: a + 2w (q x a) + 2 (q x (q x a)). */
inline Vec3f transform_quat(Vec3f a, Quatf q) {
    const v128_t uv = cross_lanes(q.v, a.v);
    const v128_t uuv = cross_lanes(q.v, uv);
    v128_t r = v4f::mul_add(uv, v4f::splat(v4f::lane<3>(q.v) * 2.0f), a.v);
    r = v4f::mul_add(uuv, v4f::splat(2.0f), r);
    return {r};
}

/** rotates a about the x axis through b by rad. */
inline Vec3f rotate_x(Vec3f a, Vec3f b, float rad) {
    const float p1 = y(a) - y(b);
    const float p2 = z(a) - z(b);
    const float c = scalar::cos(rad);
    const float s = scalar::sin(rad);
    return make(x(a), p1 * c - p2 * s + y(b), p1 * s + p2 * c + z(b));
}

inline Vec3f rotate_y(Vec3f a, Vec3f b, float rad) {
    const float p0 = x(a) - x(b);
    const float p2 = z(a) - z(b);
    const float c = scalar::cos(rad);
    const float s = scalar::sin(rad);
    return make(p2 * s + p0 * c + x(b), y(a), p2 * c - p0 * s + z(b));
}

inline Vec3f rotate_z(Vec3f a, Vec3f b, float rad) {
    const float p0 = x(a) - x(b);
    const float p1 = y(a) - y(b);
    const float c = scalar::cos(rad);
    const float s = scalar::sin(rad);
    return make(p0 * c - p1 * s + x(b), p0 * s + p1 * c + y(b), z(a));
}

/** the unsigned angle between a and b in [0, pi]; zero when either is zero. */
inline float angle(Vec3f a, Vec3f b) {
    const float mag = scalar::sqrt(squared_length(a) * squared_length(b));
    const float cosine = mag != 0.0f ? dot(a, b) / mag : 0.0f;
    return scalar::acos(scalar::min(scalar::max(cosine, -1.0f), 1.0f));
}

/** the signed angle from a to b about axis, in (-pi, pi]; the components along the axis carry no angle. */
inline float signed_angle(Vec3f a, Vec3f b, Vec3f axis) {
    const Vec3f pa = subtract(a, scale(axis, dot(a, axis)));
    const Vec3f pb = subtract(b, scale(axis, dot(b, axis)));
    return scalar::atan2(dot(cross(pa, pb), axis), dot(pa, pb));
}

/** turns the unit vector `from` toward the unit vector `to` by at most max_angle (rodrigues). */
inline Vec3f rotate_towards(Vec3f from, Vec3f to, float max_angle) {
    const float cosine = scalar::min(scalar::max(dot(from, to), -1.0f), 1.0f);
    const float limit = max_angle > 0.0f ? max_angle : 0.0f;
    if (scalar::acos(cosine) <= limit) return to;

    Vec3f axis = cross(from, to);
    const float axis_length = length(axis);
    if (axis_length < 1e-8f) {
        // antiparallel: every plane containing `from` is equally valid, so pick one
        axis = perpendicular(from);
    } else {
        axis = scale(axis, 1.0f / axis_length);
    }
    const float c = scalar::cos(limit);
    const float s = scalar::sin(limit);
    const float d = dot(axis, from) * (1.0f - c);
    const float fx = x(from), fy = y(from), fz = z(from);
    const float ax = x(axis), ay = y(axis), az = z(axis);
    return make(fx * c + (ay * fz - az * fy) * s + ax * d, fy * c + (az * fx - ax * fz) * s + ay * d,
                fz * c + (ax * fy - ay * fx) * s + az * d);
}

inline bool exact_equals(Vec3f a, Vec3f b) { return v4f::all_true(v4f::eq(a.v, b.v)); }

inline bool equals(Vec3f a, Vec3f b) {
    return scalar::equals(x(a), x(b)) && scalar::equals(y(a), y(b)) && scalar::equals(z(a), z(b));
}

inline bool finite(Vec3f a) { return scalar::is_finite(x(a)) && scalar::is_finite(y(a)) && scalar::is_finite(z(a)); }

/** an odd number of negative components flips handedness. */
inline bool is_scale_inside_out(Vec3f scale) {
    return (__builtin_popcount(v4f::bitmask(v4f::lt(scale.v, v4f::zero())) & 7u) & 1u) != 0u;
}

inline Vec3f div(Vec3f a, Vec3f b) { return divide(a, b); }
inline float dist(Vec3f a, Vec3f b) { return distance(a, b); }
inline float sqr_dist(Vec3f a, Vec3f b) { return squared_distance(a, b); }
inline float len(Vec3f a) { return length(a); }
inline float sqr_len(Vec3f a) { return squared_length(a); }

}  // namespace vec3f
