#pragma once

// the scalar functions over the storage types, retired from src when the register types took over: the reference the
// register functions are checked against, bit for bit.

#include "tumble/math/quatf.h"
#include "math/storage/vec3f.h"
#include "math/storage/vec4f.h"

TUMBLE_NAMESPACE_BEGIN

namespace quatf {

inline Float4 create() { return {0.0f, 0.0f, 0.0f, 1.0f}; }
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

inline Float4 &identity(Float4 &out) { return set(out, 0.0f, 0.0f, 0.0f, 1.0f); }

inline Float4 &from_buffer(Float4 &out, const float *buffer, uint32_t start) {
    return set(out, buffer[start], buffer[start + 1], buffer[start + 2], buffer[start + 3]);
}

inline void to_buffer(float *buffer, const Float4 &q, uint32_t start) {
    buffer[start] = q.x;
    buffer[start + 1] = q.y;
    buffer[start + 2] = q.z;
    buffer[start + 3] = q.w;
}

inline Float4 &store(Float4 &out, v128_t v) {
    v4f::store4(&out.x, v);
    return out;
}

// a quaternion is stored as a Float4, so the vec4 arithmetic applies as it is
inline Float4 &add(Float4 &out, const Float4 &a, const Float4 &b) { return vec4f::add(out, a, b); }
inline Float4 &scale(Float4 &out, const Float4 &a, float b) { return vec4f::scale(out, a, b); }
inline float dot(const Float4 &a, const Float4 &b) { return vec4f::dot(a, b); }
inline Float4 &lerp(Float4 &out, const Float4 &a, const Float4 &b, float t) { return vec4f::lerp(out, a, b, t); }
inline float length(const Float4 &a) { return vec4f::length(a); }
inline float squared_length(const Float4 &a) { return vec4f::squared_length(a); }
inline Float4 &normalize(Float4 &out, const Float4 &a) { return vec4f::normalize(out, a); }
inline bool exact_equals(const Float4 &a, const Float4 &b) { return vec4f::exact_equals(a, b); }

/** the same rotation within epsilon: |a . b| close to one, so q and -q are equal. */
inline bool equals(const Float4 &a, const Float4 &b) { return scalar::abs(dot(a, b)) >= 1.0f - scalar::EPSILON; }

/** rotation of rad about the unit axis. */
inline Float4 &set_axis_angle(Float4 &out, const Float3 &axis, float rad) {
    rad *= 0.5f;
    const float s = scalar::sin(rad);
    return set(out, s * axis.x, s * axis.y, s * axis.z, scalar::cos(rad));
}

/** the axis into out_axis (x for the identity) and the angle returned. */
inline float get_axis_angle(Float3 &out_axis, const Float4 &q) {
    const float rad = scalar::acos(q.w) * 2.0f;
    const float s = scalar::sin(rad / 2.0f);
    if (s > scalar::EPSILON) {
        vec3f::set(out_axis, q.x / s, q.y / s, q.z / s);
    } else {
        vec3f::set(out_axis, 1.0f, 0.0f, 0.0f);
    }
    return rad;
}

/** the angle between two unit quaternions. */
inline float get_angle(const Float4 &a, const Float4 &b) {
    const float d = dot(a, b);
    return scalar::acos(2.0f * d * d - 1.0f);
}

/** a * b: the hamilton product, one lane. */
inline Float4 &multiply(Float4 &out, const Float4 &a, const Float4 &b) {
    const v128_t va = v4f::load4(&a.x);
    const v128_t vb = v4f::load4(&b.x);
    const v128_t flip_w = v4f::make(1.0f, 1.0f, 1.0f, -1.0f);
    // lane i of each term, written out: x: aw bx + ax bw + ay bz - az by, and so on round to
    // w: aw bw - ax bx - ay by - az bz
    const v128_t t0 = v4f::mul(v4f::splat(a.w), vb);
    const v128_t t1 = v4f::mul(v4f::mul(v4f::shuffle<0, 1, 2, 0>(va), flip_w), v4f::shuffle<3, 3, 3, 0>(vb));
    const v128_t t2 = v4f::mul(v4f::mul(v4f::shuffle<1, 2, 0, 1>(va), flip_w), v4f::shuffle<2, 0, 1, 1>(vb));
    const v128_t t3 = v4f::mul(v4f::shuffle<2, 0, 1, 2>(va), v4f::shuffle<1, 2, 0, 2>(vb));
    return store(out, v4f::sub(v4f::add(v4f::add(t0, t1), t2), t3));
}

inline Float4 &rotate_x(Float4 &out, const Float4 &a, float rad) {
    rad *= 0.5f;
    const float bx = scalar::sin(rad);
    const float bw = scalar::cos(rad);
    return set(out, a.x * bw + a.w * bx, a.y * bw + a.z * bx, a.z * bw - a.y * bx, a.w * bw - a.x * bx);
}

inline Float4 &rotate_y(Float4 &out, const Float4 &a, float rad) {
    rad *= 0.5f;
    const float by = scalar::sin(rad);
    const float bw = scalar::cos(rad);
    return set(out, a.x * bw - a.z * by, a.y * bw + a.w * by, a.z * bw + a.x * by, a.w * bw - a.y * by);
}

inline Float4 &rotate_z(Float4 &out, const Float4 &a, float rad) {
    rad *= 0.5f;
    const float bz = scalar::sin(rad);
    const float bw = scalar::cos(rad);
    return set(out, a.x * bw + a.y * bz, a.y * bw - a.x * bz, a.z * bw + a.w * bz, a.w * bw - a.z * bz);
}

/** w from a unit quaternion's xyz. */
inline Float4 &calculate_w(Float4 &out, const Float4 &a) {
    return set(out, a.x, a.y, a.z, scalar::sqrt(scalar::abs(1.0f - a.x * a.x - a.y * a.y - a.z * a.z)));
}

inline Float4 &exp(Float4 &out, const Float4 &a) {
    const float r = scalar::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    const float et = scalar::exp(a.w);
    const float s = r > 0.0f ? (et * scalar::sin(r)) / r : 0.0f;
    return set(out, a.x * s, a.y * s, a.z * s, et * scalar::cos(r));
}

inline Float4 &ln(Float4 &out, const Float4 &a) {
    const float r = scalar::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    const float t = r > 0.0f ? scalar::atan2(r, a.w) / r : 0.0f;
    return set(out, a.x * t, a.y * t, a.z * t, 0.5f * scalar::log(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w));
}

inline Float4 &pow(Float4 &out, const Float4 &a, float b) {
    ln(out, a);
    scale(out, out, b);
    return exp(out, out);
}

/** spherical interpolation along the shorter arc. */
inline Float4 &slerp(Float4 &out, const Float4 &a, const Float4 &b, float t) {
    const v128_t va = v4f::load4(&a.x);
    v128_t vb = v4f::load4(&b.x);
    float cosom = v4f::dot4(va, vb);
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
    return store(out, v4f::mul_add(vb, v4f::splat(scale1), v4f::mul(va, v4f::splat(scale0))));
}

/** the inverse; zero for the zero quaternion. */
inline Float4 &invert(Float4 &out, const Float4 &a) {
    const float d = dot(a, a);
    if (d == 0.0f) return set(out, 0.0f, 0.0f, 0.0f, 0.0f);
    const float inv = 1.0f / d;
    return set(out, -a.x * inv, -a.y * inv, -a.z * inv, a.w * inv);
}

inline Float4 &conjugate(Float4 &out, const Float4 &a) { return set(out, -a.x, -a.y, -a.z, a.w); }

/** from a rotation matrix; the diagonal picks the stable branch. */
inline Float4 &from_mat3(Float4 &out, const Mat3f &m3) {
    const float *m = m3.m;
    const float trace = m[0] + m[4] + m[8];
    float root;
    if (trace > 0.0f) {
        root = scalar::sqrt(trace + 1.0f);  // 2w
        out.w = 0.5f * root;
        root = 0.5f / root;  // 1/(4w)
        out.x = (m[5] - m[7]) * root;
        out.y = (m[6] - m[2]) * root;
        out.z = (m[1] - m[3]) * root;
    } else {
        int i = 0;
        if (m[4] > m[0]) i = 1;
        if (m[8] > m[i * 3 + i]) i = 2;
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        float *o = &out.x;
        root = scalar::sqrt(m[i * 3 + i] - m[j * 3 + j] - m[k * 3 + k] + 1.0f);
        o[i] = 0.5f * root;
        root = 0.5f / root;
        out.w = (m[j * 3 + k] - m[k * 3 + j]) * root;
        o[j] = (m[j * 3 + i] + m[i * 3 + j]) * root;
        o[k] = (m[k * 3 + i] + m[i * 3 + k]) * root;
    }
    return out;
}

/** from the rotation part of a mat4. */
inline Float4 &from_mat4(Float4 &out, const Float4x4 &m) {
    Mat3f m3;
    mat3f::from_mat4(m3, m);
    return from_mat3(out, m3);
}

inline Float4 &from_euler(Float4 &out, const Eulerf &euler) {
    const float c1 = scalar::cos(euler.x / 2.0f);
    const float c2 = scalar::cos(euler.y / 2.0f);
    const float c3 = scalar::cos(euler.z / 2.0f);
    const float s1 = scalar::sin(euler.x / 2.0f);
    const float s2 = scalar::sin(euler.y / 2.0f);
    const float s3 = scalar::sin(euler.z / 2.0f);
    switch (euler.order) {
        case EULER_XYZ:
            return set(out, s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                       c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_YXZ:
            return set(out, s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                       c1 * c2 * c3 + s1 * s2 * s3);
        case EULER_ZXY:
            return set(out, s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                       c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_ZYX:
            return set(out, s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                       c1 * c2 * c3 + s1 * s2 * s3);
        case EULER_YZX:
            return set(out, s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                       c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_XZY:
            return set(out, s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                       c1 * c2 * c3 + s1 * s2 * s3);
    }
    return out;
}

inline Float4 &from_degrees(Float4 &out, float x, float y, float z, EulerOrder order) {
    const Eulerf euler = {x * scalar::DEGREES_TO_RADIANS, y * scalar::DEGREES_TO_RADIANS, z * scalar::DEGREES_TO_RADIANS, order};
    return from_euler(out, euler);
}

/** the rotation taking unit vector a onto unit vector b. */
inline Float4 &rotation_to(Float4 &out, const Float3 &a, const Float3 &b) {
    const float d = vec3f::dot(a, b);
    Float3 tmp;
    if (d < -0.999999f) {
        vec3f::cross(tmp, vec3f::from_values(1.0f, 0.0f, 0.0f), a);
        if (vec3f::length(tmp) < 0.000001f) vec3f::cross(tmp, vec3f::from_values(0.0f, 1.0f, 0.0f), a);
        vec3f::normalize(tmp, tmp);
        return set_axis_angle(out, tmp, scalar::PI);
    }
    if (d > 0.999999f) return identity(out);
    vec3f::cross(tmp, a, b);
    set(out, tmp.x, tmp.y, tmp.z, 1.0f + d);
    return normalize(out, out);
}

/** spherical cubic interpolation through a, b, c, d. */
inline Float4 &sqlerp(Float4 &out, const Float4 &a, const Float4 &b, const Float4 &c, const Float4 &d, float t) {
    Float4 temp1;
    Float4 temp2;
    slerp(temp1, a, d, t);
    slerp(temp2, b, c, t);
    return slerp(out, temp1, temp2, 2.0f * t * (1.0f - t));
}

/** from the three axes of a frame; view is the direction looked along, so it becomes -z. */
inline Float4 &set_axes(Float4 &out, const Float3 &view, const Float3 &right, const Float3 &up) {
    const Mat3f m = {{right.x, up.x, -view.x, right.y, up.y, -view.y, right.z, up.z, -view.z}};
    return normalize(out, from_mat3(out, m));
}

/** the rotation that looks from eye at target with up, the mirror of mat4's targetTo. */
inline Float4 &target_to(Float4 &out, const Float3 &eye, const Float3 &target, const Float3 &up) {
    float z0 = eye.x - target.x;
    float z1 = eye.y - target.y;
    float z2 = eye.z - target.z;
    float len = z0 * z0 + z1 * z1 + z2 * z2;
    if (len > 0.0f) {
        len = 1.0f / scalar::sqrt(len);
        z0 *= len;
        z1 *= len;
        z2 *= len;
    } else {
        z2 = 1.0f;
    }
    float x0 = up.y * z2 - up.z * z1;
    float x1 = up.z * z0 - up.x * z2;
    float x2 = up.x * z1 - up.y * z0;
    len = x0 * x0 + x1 * x1 + x2 * x2;
    if (len == 0.0f) {
        // up is along z: take the axis least aligned with it instead
        const float ax = scalar::abs(up.x);
        const float ay = scalar::abs(up.y);
        const float az = scalar::abs(up.z);
        if (ax <= ay && ax <= az) {
            x0 = 0.0f;
            x1 = up.z;
            x2 = -up.y;
        } else if (ay <= az) {
            x0 = -up.z;
            x1 = 0.0f;
            x2 = up.x;
        } else {
            x0 = up.y;
            x1 = -up.x;
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
        out.w = 0.5f * root;
        root = 0.5f / root;
        out.x = (y2 - z1) * root;
        out.y = (z0 - x2) * root;
        out.z = (x1 - y0) * root;
    } else if (z2 > (y1 > x0 ? y1 : x0)) {
        root = scalar::sqrt(z2 - x0 - y1 + 1.0f);
        out.z = 0.5f * root;
        root = 0.5f / root;
        out.w = (x1 - y0) * root;
        out.x = (x2 + z0) * root;
        out.y = (y2 + z1) * root;
    } else if (y1 > x0) {
        root = scalar::sqrt(y1 - z2 - x0 + 1.0f);
        out.y = 0.5f * root;
        root = 0.5f / root;
        out.w = (z0 - x2) * root;
        out.z = (z1 + y2) * root;
        out.x = (x1 + y0) * root;
    } else {
        root = scalar::sqrt(x0 - y1 - z2 + 1.0f);
        out.x = 0.5f * root;
        root = 0.5f / root;
        out.w = (y2 - z1) * root;
        out.y = (y0 + x1) * root;
        out.z = (z0 + x2) * root;
    }
    return out;
}

inline Float4 &mul(Float4 &out, const Float4 &a, const Float4 &b) { return multiply(out, a, b); }
inline float len(const Float4 &a) { return length(a); }
inline float sqr_len(const Float4 &a) { return squared_length(a); }

}  // namespace quatf

TUMBLE_NAMESPACE_END
