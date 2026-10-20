#pragma once

// what the tests check with, in the register types: near comparisons, unit vectors, rotations about an axis, poses.

#include "math/aabox.h"
#include "math/mat4f.h"
#include "math/quatf.h"
#include "math/scalar.h"
#include "math/types.h"
#include "math/vec3f.h"

/** each component of a within eps of b's */
inline bool near3(Vec3f a, Vec3f b, float eps) {
    return scalar::abs(vec3f::x(a) - vec3f::x(b)) <= eps && scalar::abs(vec3f::y(a) - vec3f::y(b)) <= eps &&
           scalar::abs(vec3f::z(a) - vec3f::z(b)) <= eps;
}

inline bool near3(Vec3f a, float x, float y, float z, float eps) { return near3(a, vec3f::make(x, y, z), eps); }

/** a stored vector, as a record or a hit holds it */
inline bool near3(const Float3 &a, Vec3f b, float eps) { return near3(vec3f::load(a), b, eps); }

inline bool near3(Vec3f a, const Float3 &b, float eps) { return near3(a, vec3f::load(b), eps); }

inline bool near3(const Float3 &a, const Float3 &b, float eps) { return near3(vec3f::load(a), vec3f::load(b), eps); }

inline bool near3(const Float3 &a, float x, float y, float z, float eps) { return near3(vec3f::load(a), vec3f::make(x, y, z), eps); }

/** two stored boxes equal, component for component */
inline bool equal_boxes(const Box3f &a, const Box3f &b) {
    return vec3f::exact_equals(vec3f::load(a.min), vec3f::load(b.min)) && vec3f::exact_equals(vec3f::load(a.max), vec3f::load(b.max));
}

inline Vec3f unit(Vec3f v) { return vec3f::normalize(v); }

/** a rotation of rad about axis, which need not be unit length */
inline Quatf axis_angle(Vec3f axis, float rad) { return quatf::set_axis_angle(vec3f::normalize(axis), rad); }

/** a register matrix's 16 floats, column-major, to read by index */
inline Float4x4 elements(const Mat4f &m) {
    Float4x4 out;
    return mat4f::store(out, m);
}

/** n register points as flat xyz floats, as a face's vertices are read by index */
inline const float *flatten3(float *out, const Vec3f *points, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) vec3f::store(out, points[i], i * 3);
    return out;
}

/** flat xyz floats as n register points in out */
inline const Vec3f *unflatten3(Vec3f *out, const float *xyz, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) out[i] = vec3f::load(xyz, i * 3);
    return out;
}

/** the same rotation stored, as settings and records hold one */
inline Float4 stored_axis_angle(Vec3f axis, float rad) {
    Float4 out;
    return quatf::store(out, axis_angle(axis, rad));
}

inline Transformf pose(const Float3 &position, const Float4 &quaternion = {0.0f, 0.0f, 0.0f, 1.0f}, const Float3 &scale = {1.0f, 1.0f, 1.0f}) {
    return {position, quaternion, scale};
}

inline Transformf pose(Vec3f position, const Float4 &quaternion = {0.0f, 0.0f, 0.0f, 1.0f}, const Float3 &scale = {1.0f, 1.0f, 1.0f}) {
    Transformf out = {{}, quaternion, scale};
    vec3f::store(out.position, position);
    return out;
}
