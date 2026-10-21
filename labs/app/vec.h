#pragma once

#include <math.h>

#include "tumble/math/types.h"

// plain float math for drawing: the library's register types are for the simulation
namespace vec {

using tumble::Float3;
using tumble::Float4;

inline Float3 add(Float3 a, Float3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Float3 subtract(Float3 a, Float3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Float3 multiply(Float3 a, Float3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline Float3 scale(Float3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Float3 scale_and_add(Float3 a, Float3 b, float s) { return {a.x + b.x * s, a.y + b.y * s, a.z + b.z * s}; }
inline float dot(Float3 a, Float3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Float3 cross(Float3 a, Float3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float length(Float3 a) { return sqrtf(dot(a, a)); }
inline Float3 normalize(Float3 a) {
    const float l = length(a);
    return l > 0.0f ? scale(a, 1.0f / l) : a;
}

/** a rotated by the unit quaternion q */
inline Float3 rotate(Float4 q, Float3 a) {
    const Float3 u = {q.x, q.y, q.z};
    const Float3 t = scale(cross(u, a), 2.0f);
    return add(add(a, scale(t, q.w)), cross(u, t));
}

inline Float4 quat_multiply(Float4 a, Float4 b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline Float4 quat_axis_angle(Float3 axis, float angle) {
    const Float3 a = normalize(axis);
    const float s = sinf(angle * 0.5f);
    return {a.x * s, a.y * s, a.z * s, cosf(angle * 0.5f)};
}

/** the shortest rotation taking unit a onto unit b */
inline Float4 quat_from_to(Float3 a, Float3 b) {
    const float d = dot(a, b);
    if (d < -0.999999f) {
        Float3 axis = cross({1.0f, 0.0f, 0.0f}, a);
        if (length(axis) < 1e-6f) axis = cross({0.0f, 1.0f, 0.0f}, a);
        return quat_axis_angle(axis, 3.14159265f);
    }
    const Float3 c = cross(a, b);
    const Float4 q = {c.x, c.y, c.z, 1.0f + d};
    const float l = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return {q.x / l, q.y / l, q.z / l, q.w / l};
}

}
