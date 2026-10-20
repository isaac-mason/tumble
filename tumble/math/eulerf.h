#pragma once

// the math package's euler.ts over Eulerf. the order is an enum where the js has a string.

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace eulerf {

inline Eulerf create() { return {0.0f, 0.0f, 0.0f, EULER_XYZ}; }
inline Eulerf clone(const Eulerf &a) { return a; }
inline Eulerf from_values(float x, float y, float z, EulerOrder order) { return {x, y, z, order}; }

inline Eulerf &copy(Eulerf &out, const Eulerf &a) {
    out = a;
    return out;
}

inline Eulerf &set(Eulerf &out, float x, float y, float z, EulerOrder order) {
    out.x = x;
    out.y = y;
    out.z = z;
    out.order = order;
    return out;
}

inline Eulerf &from_degrees(Eulerf &out, float x, float y, float z, EulerOrder order) {
    return set(out, x * scalar::DEGREES_TO_RADIANS, y * scalar::DEGREES_TO_RADIANS, z * scalar::DEGREES_TO_RADIANS, order);
}

/** from the nine entries of a rotation matrix, row-major names (m12 is row 1, column 2). */
inline Eulerf &from_rotation_matrix_values(Eulerf &out, float m11, float m12, float m13, float m21, float m22, float m23,
                                           float m31, float m32, float m33, EulerOrder order) {
    switch (order) {
        case EULER_XYZ:
            out.y = scalar::asin(scalar::clamp(m13, -1.0f, 1.0f));
            if (scalar::abs(m13) < 0.9999999f) {
                out.x = scalar::atan2(-m23, m33);
                out.z = scalar::atan2(-m12, m11);
            } else {
                out.x = scalar::atan2(m32, m22);
                out.z = 0.0f;
            }
            break;
        case EULER_YXZ:
            out.x = scalar::asin(-scalar::clamp(m23, -1.0f, 1.0f));
            if (scalar::abs(m23) < 0.9999999f) {
                out.y = scalar::atan2(m13, m33);
                out.z = scalar::atan2(m21, m22);
            } else {
                out.y = scalar::atan2(-m31, m11);
                out.z = 0.0f;
            }
            break;
        case EULER_ZXY:
            out.x = scalar::asin(scalar::clamp(m32, -1.0f, 1.0f));
            if (scalar::abs(m32) < 0.9999999f) {
                out.y = scalar::atan2(-m31, m33);
                out.z = scalar::atan2(-m12, m22);
            } else {
                out.y = 0.0f;
                out.z = scalar::atan2(m21, m11);
            }
            break;
        case EULER_ZYX:
            out.y = scalar::asin(-scalar::clamp(m31, -1.0f, 1.0f));
            if (scalar::abs(m31) < 0.9999999f) {
                out.x = scalar::atan2(m32, m33);
                out.z = scalar::atan2(m21, m11);
            } else {
                out.x = 0.0f;
                out.z = scalar::atan2(-m12, m22);
            }
            break;
        case EULER_YZX:
            out.z = scalar::asin(scalar::clamp(m21, -1.0f, 1.0f));
            if (scalar::abs(m21) < 0.9999999f) {
                out.x = scalar::atan2(-m23, m22);
                out.y = scalar::atan2(-m31, m11);
            } else {
                out.x = 0.0f;
                out.y = scalar::atan2(m13, m33);
            }
            break;
        case EULER_XZY:
            out.z = scalar::asin(-scalar::clamp(m12, -1.0f, 1.0f));
            if (scalar::abs(m12) < 0.9999999f) {
                out.x = scalar::atan2(m32, m22);
                out.y = scalar::atan2(m13, m11);
            } else {
                out.x = scalar::atan2(-m23, m33);
                out.y = 0.0f;
            }
            break;
    }
    out.order = order;
    return out;
}

/** from the rotation part of a mat4. */
inline Eulerf &from_rotation_mat4(Eulerf &out, const Float4x4 &m, EulerOrder order) {
    return from_rotation_matrix_values(out, m.m[0], m.m[4], m.m[8], m.m[1], m.m[5], m.m[9], m.m[2], m.m[6], m.m[10], order);
}

inline Eulerf &from_quat(Eulerf &out, const Float4 &q, EulerOrder order) {
    const float x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
    const float xx = q.x * x2, yx = q.y * x2, yy = q.y * y2;
    const float zx = q.z * x2, zy = q.z * y2, zz = q.z * z2;
    const float wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;
    return from_rotation_matrix_values(out, 1.0f - yy - zz, yx - wz, zx + wy, yx + wz, 1.0f - xx - zz, zy - wx, zx - wy,
                                       zy + wx, 1.0f - xx - yy, order);
}

/** the same rotation expressed in another order. */
inline Eulerf &reorder(Eulerf &out, const Eulerf &a, EulerOrder order) {
    Float4 q;
    quatf::store(q, quatf::from_euler(a));
    return from_quat(out, q, order);
}

inline bool exact_equals(const Eulerf &a, const Eulerf &b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.order == b.order;
}

inline bool equals(const Eulerf &a, const Eulerf &b) {
    return scalar::equals(a.x, b.x) && scalar::equals(a.y, b.y) && scalar::equals(a.z, b.z) && a.order == b.order;
}

}  // namespace eulerf

TUMBLE_NAMESPACE_END
