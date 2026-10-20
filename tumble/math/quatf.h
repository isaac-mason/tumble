#pragma once

// the math package's quat.ts on Quatf, whole, by value. a quaternion's storage is a Float4. each function gives
// the bits of its scalar reference (test/math/storage/quatf.h); the vec4 arithmetic is vec4f's.

#include "tumble/math/mat3f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/vec4f.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace quatf {

inline Quatf load(const Float4 &a) { return {v4f::load4(&a.x)}; }
inline Float4 &store(Float4 &out, Quatf a) {
    v4f::store4(&out.x, a.v);
    return out;
}

inline Quatf make(float x, float y, float z, float w) { return {v4f::make(x, y, z, w)}; }
inline Quatf identity() { return make(0.0f, 0.0f, 0.0f, 1.0f); }

inline float x(Quatf q) { return v4f::lane<0>(q.v); }
inline float y(Quatf q) { return v4f::lane<1>(q.v); }
inline float z(Quatf q) { return v4f::lane<2>(q.v); }
inline float w(Quatf q) { return v4f::lane<3>(q.v); }
/** the vector part. */
inline Vec3f xyz(Quatf q) { return {v4f::shuffle<0, 1, 2, 2>(q.v)}; }

inline Vec4f as_vec4(Quatf q) { return {q.v}; }
inline Quatf from_vec4(Vec4f v) { return {v.v}; }

inline Quatf add(Quatf a, Quatf b) { return {v4f::add(a.v, b.v)}; }
inline Quatf scale(Quatf a, float b) { return {v4f::mul(a.v, v4f::splat(b))}; }
inline float dot(Quatf a, Quatf b) { return v4f::dot4(a.v, b.v); }
inline Quatf lerp(Quatf a, Quatf b, float t) { return {v4f::mul_add(v4f::sub(b.v, a.v), v4f::splat(t), a.v)}; }
inline float length(Quatf a) { return scalar::sqrt(dot(a, a)); }
inline float squared_length(Quatf a) { return dot(a, a); }
inline Quatf normalize(Quatf a) { return from_vec4(vec4f::normalize(as_vec4(a))); }
inline bool exact_equals(Quatf a, Quatf b) { return v4f::all_true(v4f::eq(a.v, b.v)); }

/** the same rotation within epsilon: |a . b| close to one, so q and -q are equal. */
inline bool equals(Quatf a, Quatf b) { return scalar::abs(dot(a, b)) >= 1.0f - scalar::EPSILON; }

/** rotation of rad about the unit axis. */
inline Quatf set_axis_angle(Vec3f axis, float rad) {
    rad *= 0.5f;
    const float s = scalar::sin(rad);
    return make(s * vec3f::x(axis), s * vec3f::y(axis), s * vec3f::z(axis), scalar::cos(rad));
}

/** the axis into out_axis (x for the identity) and the angle returned. */
inline float get_axis_angle(Vec3f &out_axis, Quatf q) {
    const float rad = scalar::acos(w(q)) * 2.0f;
    const float s = scalar::sin(rad / 2.0f);
    if (s > scalar::EPSILON) {
        out_axis = vec3f::make(x(q) / s, y(q) / s, z(q) / s);
    } else {
        out_axis = vec3f::axis_x();
    }
    return rad;
}

/** the angle between two unit quaternions. */
inline float get_angle(Quatf a, Quatf b) {
    const float d = dot(a, b);
    return scalar::acos(2.0f * d * d - 1.0f);
}

/** a * b: the hamilton product, one lane. */
inline Quatf multiply(Quatf a, Quatf b) {
    const v128_t flip_w = v4f::make(1.0f, 1.0f, 1.0f, -1.0f);
    const v128_t t0 = v4f::mul(v4f::shuffle<3, 3, 3, 3>(a.v), b.v);
    const v128_t t1 = v4f::mul(v4f::mul(v4f::shuffle<0, 1, 2, 0>(a.v), flip_w), v4f::shuffle<3, 3, 3, 0>(b.v));
    const v128_t t2 = v4f::mul(v4f::mul(v4f::shuffle<1, 2, 0, 1>(a.v), flip_w), v4f::shuffle<2, 0, 1, 1>(b.v));
    const v128_t t3 = v4f::mul(v4f::shuffle<2, 0, 1, 2>(a.v), v4f::shuffle<1, 2, 0, 2>(b.v));
    return {v4f::sub(v4f::add(v4f::add(t0, t1), t2), t3)};
}

inline Quatf rotate_x(Quatf a, float rad) {
    rad *= 0.5f;
    const float bx = scalar::sin(rad);
    const float bw = scalar::cos(rad);
    const float ax = x(a), ay = y(a), az = z(a), aw = w(a);
    return make(ax * bw + aw * bx, ay * bw + az * bx, az * bw - ay * bx, aw * bw - ax * bx);
}

inline Quatf rotate_y(Quatf a, float rad) {
    rad *= 0.5f;
    const float by = scalar::sin(rad);
    const float bw = scalar::cos(rad);
    const float ax = x(a), ay = y(a), az = z(a), aw = w(a);
    return make(ax * bw - az * by, ay * bw + aw * by, az * bw + ax * by, aw * bw - ay * by);
}

inline Quatf rotate_z(Quatf a, float rad) {
    rad *= 0.5f;
    const float bz = scalar::sin(rad);
    const float bw = scalar::cos(rad);
    const float ax = x(a), ay = y(a), az = z(a), aw = w(a);
    return make(ax * bw + ay * bz, ay * bw - ax * bz, az * bw + aw * bz, aw * bw - az * bz);
}

/** w from a unit quaternion's xyz. */
inline Quatf calculate_w(Quatf a) {
    const float ax = x(a), ay = y(a), az = z(a);
    return make(ax, ay, az, scalar::sqrt(scalar::abs(1.0f - ax * ax - ay * ay - az * az)));
}

inline Quatf exp(Quatf a) {
    const float ax = x(a), ay = y(a), az = z(a);
    const float r = scalar::sqrt(ax * ax + ay * ay + az * az);
    const float et = scalar::exp(w(a));
    const float s = r > 0.0f ? (et * scalar::sin(r)) / r : 0.0f;
    return make(ax * s, ay * s, az * s, et * scalar::cos(r));
}

inline Quatf ln(Quatf a) {
    const float ax = x(a), ay = y(a), az = z(a), aw = w(a);
    const float r = scalar::sqrt(ax * ax + ay * ay + az * az);
    const float t = r > 0.0f ? scalar::atan2(r, aw) / r : 0.0f;
    return make(ax * t, ay * t, az * t, 0.5f * scalar::log(ax * ax + ay * ay + az * az + aw * aw));
}

inline Quatf pow(Quatf a, float b) { return exp(scale(ln(a), b)); }

/** spherical interpolation along the shorter arc. */
inline Quatf slerp(Quatf a, Quatf b, float t) {
    v128_t vb = b.v;
    float cosom = v4f::dot4(a.v, vb);
    if (cosom < 0.0f) {
        cosom = -cosom;
        vb = v4f::neg(vb);
    }
    float scale0;
    float scale1;
    if (1.0f - cosom > scalar::EPSILON) {
        const float omega = scalar::acos(cosom);
        const float sinom = scalar::sin(omega);
        scale0 = scalar::sin((1.0f - t) * omega) / sinom;
        scale1 = scalar::sin(t * omega) / sinom;
    } else {
        scale0 = 1.0f - t;
        scale1 = t;
    }
    return {v4f::mul_add(vb, v4f::splat(scale1), v4f::mul(a.v, v4f::splat(scale0)))};
}

/** the inverse; zero for the zero quaternion. */
inline Quatf invert(Quatf a) {
    const float d = dot(a, a);
    if (d == 0.0f) return {v4f::zero()};
    const float inv = 1.0f / d;
    return {v4f::mul(v4f::neg_xyz(a.v), v4f::splat(inv))};
}

inline Quatf conjugate(Quatf a) { return {v4f::neg_xyz(a.v)}; }

/** from a rotation matrix; the diagonal picks the stable branch. */
inline Quatf from_mat3(const Mat3f &m3) {
    const float *m = m3.m;
    float o[4];
    const float trace = m[0] + m[4] + m[8];
    float root;
    if (trace > 0.0f) {
        root = scalar::sqrt(trace + 1.0f);  // 2w
        o[3] = 0.5f * root;
        root = 0.5f / root;  // 1/(4w)
        o[0] = (m[5] - m[7]) * root;
        o[1] = (m[6] - m[2]) * root;
        o[2] = (m[1] - m[3]) * root;
    } else {
        int i = 0;
        if (m[4] > m[0]) i = 1;
        if (m[8] > m[i * 3 + i]) i = 2;
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        root = scalar::sqrt(m[i * 3 + i] - m[j * 3 + j] - m[k * 3 + k] + 1.0f);
        o[i] = 0.5f * root;
        root = 0.5f / root;
        o[3] = (m[j * 3 + k] - m[k * 3 + j]) * root;
        o[j] = (m[j * 3 + i] + m[i * 3 + j]) * root;
        o[k] = (m[k * 3 + i] + m[i * 3 + k]) * root;
    }
    return make(o[0], o[1], o[2], o[3]);
}

/** from the rotation part of a mat4. */
inline Quatf from_mat4(const Mat4f &m) {
    const v128_t c0 = m.c[0], c1 = m.c[1], c2 = m.c[2];
    const Mat3f m3 = {{v4f::lane<0>(c0), v4f::lane<1>(c0), v4f::lane<2>(c0), v4f::lane<0>(c1), v4f::lane<1>(c1),
                       v4f::lane<2>(c1), v4f::lane<0>(c2), v4f::lane<1>(c2), v4f::lane<2>(c2)}};
    return from_mat3(m3);
}

inline Quatf from_euler(const Eulerf &euler) {
    const float c1 = scalar::cos(euler.x / 2.0f);
    const float c2 = scalar::cos(euler.y / 2.0f);
    const float c3 = scalar::cos(euler.z / 2.0f);
    const float s1 = scalar::sin(euler.x / 2.0f);
    const float s2 = scalar::sin(euler.y / 2.0f);
    const float s3 = scalar::sin(euler.z / 2.0f);
    switch (euler.order) {
        case EULER_XYZ:
            return make(s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                        c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_YXZ:
            return make(s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                        c1 * c2 * c3 + s1 * s2 * s3);
        case EULER_ZXY:
            return make(s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                        c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_ZYX:
            return make(s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                        c1 * c2 * c3 + s1 * s2 * s3);
        case EULER_YZX:
            return make(s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                        c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_XZY:
            return make(s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                        c1 * c2 * c3 + s1 * s2 * s3);
    }
    return identity();
}

inline Quatf from_degrees(float x, float y, float z, EulerOrder order) {
    const Eulerf euler = {x * scalar::DEGREES_TO_RADIANS, y * scalar::DEGREES_TO_RADIANS, z * scalar::DEGREES_TO_RADIANS, order};
    return from_euler(euler);
}

/** the rotation taking unit vector a onto unit vector b. */
inline Quatf rotation_to(Vec3f a, Vec3f b) {
    const float d = vec3f::dot(a, b);
    if (d < -0.999999f) {
        Vec3f tmp = vec3f::cross(vec3f::axis_x(), a);
        if (vec3f::length(tmp) < 0.000001f) tmp = vec3f::cross(vec3f::axis_y(), a);
        return set_axis_angle(vec3f::normalize(tmp), scalar::PI);
    }
    if (d > 0.999999f) return identity();
    const Vec3f tmp = vec3f::cross(a, b);
    return normalize(make(vec3f::x(tmp), vec3f::y(tmp), vec3f::z(tmp), 1.0f + d));
}

/** spherical cubic interpolation through a, b, c, d. */
inline Quatf sqlerp(Quatf a, Quatf b, Quatf c, Quatf d, float t) {
    return slerp(slerp(a, d, t), slerp(b, c, t), 2.0f * t * (1.0f - t));
}

/** from the three axes of a frame; view is the direction looked along, so it becomes -z. */
inline Quatf set_axes(Vec3f view, Vec3f right, Vec3f up) {
    const Mat3f m = {{vec3f::x(right), vec3f::x(up), -vec3f::x(view), vec3f::y(right), vec3f::y(up), -vec3f::y(view),
                      vec3f::z(right), vec3f::z(up), -vec3f::z(view)}};
    return normalize(from_mat3(m));
}

/** the rotation that looks from eye at target with up, the mirror of mat4's targetTo. */
inline Quatf target_to(Vec3f eye, Vec3f target, Vec3f up) {
    const float ux = vec3f::x(up), uy = vec3f::y(up), uz = vec3f::z(up);
    float z0 = vec3f::x(eye) - vec3f::x(target);
    float z1 = vec3f::y(eye) - vec3f::y(target);
    float z2 = vec3f::z(eye) - vec3f::z(target);
    float len = z0 * z0 + z1 * z1 + z2 * z2;
    if (len > 0.0f) {
        len = 1.0f / scalar::sqrt(len);
        z0 *= len;
        z1 *= len;
        z2 *= len;
    } else {
        z2 = 1.0f;
    }
    float x0 = uy * z2 - uz * z1;
    float x1 = uz * z0 - ux * z2;
    float x2 = ux * z1 - uy * z0;
    len = x0 * x0 + x1 * x1 + x2 * x2;
    if (len == 0.0f) {
        // up is along z: take the axis least aligned with it instead
        const float ax = scalar::abs(ux);
        const float ay = scalar::abs(uy);
        const float az = scalar::abs(uz);
        if (ax <= ay && ax <= az) {
            x0 = 0.0f;
            x1 = uz;
            x2 = -uy;
        } else if (ay <= az) {
            x0 = -uz;
            x1 = 0.0f;
            x2 = ux;
        } else {
            x0 = uy;
            x1 = -ux;
            x2 = 0.0f;
        }
        len = x0 * x0 + x1 * x1 + x2 * x2;
    }
    if (len > 0.0f) {
        len = 1.0f / scalar::sqrt(len);
        x0 *= len;
        x1 *= len;
        x2 *= len;
    }
    float y0 = z1 * x2 - z2 * x1;
    float y1 = z2 * x0 - z0 * x2;
    float y2 = z0 * x1 - z1 * x0;
    float ylen = y0 * y0 + y1 * y1 + y2 * y2;
    if (ylen > 0.0f) {
        ylen = 1.0f / scalar::sqrt(ylen);
        y0 *= ylen;
        y1 *= ylen;
        y2 *= ylen;
    }
    x0 = y1 * z2 - y2 * z1;
    x1 = y2 * z0 - y0 * z2;
    x2 = y0 * z1 - y1 * z0;
    const float trace = x0 + y1 + z2;
    float root;
    if (trace > 0.0f) {
        root = scalar::sqrt(trace + 1.0f);
        const float ow = 0.5f * root;
        root = 0.5f / root;
        return make((y2 - z1) * root, (z0 - x2) * root, (x1 - y0) * root, ow);
    }
    if (z2 > (y1 > x0 ? y1 : x0)) {
        root = scalar::sqrt(z2 - x0 - y1 + 1.0f);
        const float oz = 0.5f * root;
        root = 0.5f / root;
        return make((x2 + z0) * root, (y2 + z1) * root, oz, (x1 - y0) * root);
    }
    if (y1 > x0) {
        root = scalar::sqrt(y1 - z2 - x0 + 1.0f);
        const float oy = 0.5f * root;
        root = 0.5f / root;
        return make((x1 + y0) * root, oy, (z1 + y2) * root, (z0 - x2) * root);
    }
    root = scalar::sqrt(x0 - y1 - z2 + 1.0f);
    const float ox = 0.5f * root;
    root = 0.5f / root;
    return make(ox, (y0 + x1) * root, (z0 + x2) * root, (y2 - z1) * root);
}

inline float len(Quatf a) { return length(a); }
inline float sqr_len(Quatf a) { return squared_length(a); }

}  // namespace quatf

TUMBLE_NAMESPACE_END
