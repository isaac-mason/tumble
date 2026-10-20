#pragma once

// the scalar functions over the storage types, retired from src when the register types took over: the reference the
// register functions are checked against, bit for bit.

#include "math/plane3f.h"
#include "math/storage/vec3f.h"
#include "math/storage/mat4f.h"

namespace plane3f {

inline Plane3f create() { return {{0.0f, 1.0f, 0.0f}, 0.0f}; }
inline Plane3f clone(const Plane3f &plane) { return plane; }

inline Plane3f &copy(Plane3f &out, const Plane3f &plane) {
    out = plane;
    return out;
}

inline Plane3f &from_normal_and_constant(Plane3f &out, const Float3 &normal, float constant) {
    out.normal = normal;
    out.constant = constant;
    return out;
}

inline Plane3f &from_normal_and_point(Plane3f &out, const Float3 &normal, const Float3 &point) {
    out.normal = normal;
    out.constant = -vec3f::dot(normal, point);
    return out;
}

/** the plane through a, b, c with normal (b - a) x (c - a). */
inline Plane3f &from_coplanar_points(Plane3f &out, const Float3 &a, const Float3 &b, const Float3 &c) {
    const float v1x = b.x - a.x, v1y = b.y - a.y, v1z = b.z - a.z;
    const float v2x = c.x - a.x, v2y = c.y - a.y, v2z = c.z - a.z;
    float nx = v1y * v2z - v1z * v2y;
    float ny = v1z * v2x - v1x * v2z;
    float nz = v1x * v2y - v1y * v2x;
    float len = nx * nx + ny * ny + nz * nz;
    if (len > 0.0f) len = 1.0f / scalar::sqrt(len);
    nx *= len;
    ny *= len;
    nz *= len;
    vec3f::set(out.normal, nx, ny, nz);
    out.constant = -(nx * a.x + ny * a.y + nz * a.z);
    return out;
}

inline Plane3f &normalize(Plane3f &out, const Plane3f &plane) {
    const float inv = 1.0f / vec3f::length(plane.normal);
    vec3f::scale(out.normal, plane.normal, inv);
    out.constant = plane.constant * inv;
    return out;
}

inline Plane3f &negate(Plane3f &out, const Plane3f &plane) {
    vec3f::negate(out.normal, plane.normal);
    out.constant = -plane.constant;
    return out;
}

/** the plane moved along its normal by distance. */
inline Plane3f &offset(Plane3f &out, const Plane3f &plane, float distance) {
    out.normal = plane.normal;
    out.constant = plane.constant - distance;
    return out;
}

inline float distance_to_point(const Plane3f &plane, const Float3 &point) { return vec3f::dot(plane.normal, point) + plane.constant; }

inline Float3 &project_point(Float3 &out, const Plane3f &plane, const Float3 &point) {
    const float distance = distance_to_point(plane, point);
    return vec3f::scale_and_add(out, point, plane.normal, -distance);
}

/** the plane through matrix: a point on it transforms as a point, the normal as a direction, then renormalise. */
inline Plane3f &transform(Plane3f &out, const Plane3f &plane, const Float4x4 &matrix) {
    const float *m = matrix.m;
    const float inx = plane.normal.x, iny = plane.normal.y, inz = plane.normal.z;
    const float px = inx * -plane.constant;
    const float py = iny * -plane.constant;
    const float pz = inz * -plane.constant;
    float nx = m[0] * inx + m[4] * iny + m[8] * inz;
    float ny = m[1] * inx + m[5] * iny + m[9] * inz;
    float nz = m[2] * inx + m[6] * iny + m[10] * inz;
    float w = m[3] * px + m[7] * py + m[11] * pz + m[15];
    if (w == 0.0f || w != w) w = 1.0f;
    const float tpx = (m[0] * px + m[4] * py + m[8] * pz + m[12]) / w;
    const float tpy = (m[1] * px + m[5] * py + m[9] * pz + m[13]) / w;
    const float tpz = (m[2] * px + m[6] * py + m[10] * pz + m[14]) / w;
    float len = nx * nx + ny * ny + nz * nz;
    if (len > 0.0f) len = 1.0f / scalar::sqrt(len);
    nx *= len;
    ny *= len;
    nz *= len;
    vec3f::set(out.normal, nx, ny, nz);
    out.constant = -(nx * tpx + ny * tpy + nz * tpz);
    return out;
}

inline bool intersects_sphere(const Plane3f &plane, const Spheref &sphere) {
    return scalar::abs(distance_to_point(plane, sphere.center)) <= sphere.radius;
}

/** the point three planes meet at; false when any two are parallel. */
inline bool intersect(Float3 &out, const Plane3f &p1, const Plane3f &p2, const Plane3f &p3) {
    const Float3 &n1 = p1.normal;
    const Float3 &n2 = p2.normal;
    const Float3 &n3 = p3.normal;
    const float c1x = n2.y * n3.z - n2.z * n3.y;
    const float c1y = n2.z * n3.x - n2.x * n3.z;
    const float c1z = n2.x * n3.y - n2.y * n3.x;
    const float denom = n1.x * c1x + n1.y * c1y + n1.z * c1z;
    if (scalar::abs(denom) < 0.000001f) return false;
    const float c2x = n3.y * n1.z - n3.z * n1.y;
    const float c2y = n3.z * n1.x - n3.x * n1.z;
    const float c2z = n3.x * n1.y - n3.y * n1.x;
    const float c3x = n1.y * n2.z - n1.z * n2.y;
    const float c3y = n1.z * n2.x - n1.x * n2.z;
    const float c3z = n1.x * n2.y - n1.y * n2.x;
    const float d1 = p1.constant, d2 = p2.constant, d3 = p3.constant;
    const float s = -1.0f / denom;
    vec3f::set(out, (d1 * c1x + d2 * c2x + d3 * c3x) * s, (d1 * c1y + d2 * c2y + d3 * c3y) * s,
               (d1 * c1z + d2 * c2z + d3 * c3z) * s);
    return true;
}

inline bool exact_equals(const Plane3f &a, const Plane3f &b) {
    return vec3f::exact_equals(a.normal, b.normal) && a.constant == b.constant;
}

inline bool equals(const Plane3f &a, const Plane3f &b) {
    return vec3f::equals(a.normal, b.normal) && scalar::abs(a.constant - b.constant) < 0.000001f;
}

}  // namespace plane3f
