#pragma once

// the math package's frustum.ts over Frustumf: six planes facing inward, left right bottom top near
// far, from a projection and a view matrix. NO is the -1..1 depth range (webgl), ZO is 0..1 (webgpu),
// and the sides variant leaves out near and far for a culler that handles depth itself. the box test
// picks each plane's positive vertex with one lane select.

#include "math/storage/mat4f.h"
#include "math/storage/plane3f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/math/types.h"
#include "math/storage/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace frustumf {

inline Frustumf create() {
    Frustumf f;
    for (int i = 0; i < 6; i++) f.planes[i] = {{0.0f, 1.0f, 0.0f}, 0.0f};
    return f;
}

inline Frustumf clone(const Frustumf &f) { return f; }

inline Frustumf &copy(Frustumf &out, const Frustumf &f) {
    out = f;
    return out;
}

/** a plane from a row combination of the view-projection matrix, normalised. */
inline void set_plane(Plane3f &out, float nx, float ny, float nz, float c) {
    float inv = nx * nx + ny * ny + nz * nz;
    if (inv > 0.0f) inv = 1.0f / scalar::sqrt(inv);
    out.normal = {nx * inv, ny * inv, nz * inv};
    out.constant = c * inv;
}

/** row i of proj * view as (x, y, z, w). */
inline void vp_row(float *out, const Float4x4 &vp, int i) {
    out[0] = vp.m[i];
    out[1] = vp.m[4 + i];
    out[2] = vp.m[8 + i];
    out[3] = vp.m[12 + i];
}

inline void set_sides(Frustumf &out, const Float4x4 &vp) {
    float r0[4], r1[4], r3[4];
    vp_row(r0, vp, 0);
    vp_row(r1, vp, 1);
    vp_row(r3, vp, 3);
    set_plane(out.planes[0], r3[0] + r0[0], r3[1] + r0[1], r3[2] + r0[2], r3[3] + r0[3]);
    set_plane(out.planes[1], r3[0] - r0[0], r3[1] - r0[1], r3[2] - r0[2], r3[3] - r0[3]);
    set_plane(out.planes[2], r3[0] + r1[0], r3[1] + r1[1], r3[2] + r1[2], r3[3] + r1[3]);
    set_plane(out.planes[3], r3[0] - r1[0], r3[1] - r1[1], r3[2] - r1[2], r3[3] - r1[3]);
}

/** -1..1 clip depth: near is w + z, far is w - z. */
inline Frustumf &set_from_view_projection_matrix_no(Frustumf &out, const Float4x4 &proj, const Float4x4 &view) {
    Float4x4 vp;
    mat4f::multiply(vp, proj, view);
    set_sides(out, vp);
    float r2[4], r3[4];
    vp_row(r2, vp, 2);
    vp_row(r3, vp, 3);
    set_plane(out.planes[4], r3[0] + r2[0], r3[1] + r2[1], r3[2] + r2[2], r3[3] + r2[3]);
    set_plane(out.planes[5], r3[0] - r2[0], r3[1] - r2[1], r3[2] - r2[2], r3[3] - r2[3]);
    return out;
}

/** 0..1 clip depth: near is z alone, far is w - z. */
inline Frustumf &set_from_view_projection_matrix_zo(Frustumf &out, const Float4x4 &proj, const Float4x4 &view) {
    Float4x4 vp;
    mat4f::multiply(vp, proj, view);
    set_sides(out, vp);
    float r2[4], r3[4];
    vp_row(r2, vp, 2);
    vp_row(r3, vp, 3);
    set_plane(out.planes[4], r2[0], r2[1], r2[2], r2[3]);
    set_plane(out.planes[5], r3[0] - r2[0], r3[1] - r2[1], r3[2] - r2[2], r3[3] - r2[3]);
    return out;
}

/** only the four side planes; near and far are left as they were. */
inline Frustumf &set_from_view_projection_matrix_sides(Frustumf &out, const Float4x4 &proj, const Float4x4 &view) {
    Float4x4 vp;
    mat4f::multiply(vp, proj, view);
    set_sides(out, vp);
    return out;
}

inline bool planes_intersect_sphere(const Frustumf &f, const Spheref &s, int count) {
    for (int i = 0; i < count; i++) {
        if (plane3f::distance_to_point(f.planes[i], s.center) < -s.radius) return false;
    }
    return true;
}

inline bool intersects_sphere(const Frustumf &f, const Spheref &s) { return planes_intersect_sphere(f, s, 6); }
inline bool sides_intersects_sphere(const Frustumf &f, const Spheref &s) { return planes_intersect_sphere(f, s, 4); }

/** the box is outside when its vertex furthest along a plane's normal is behind that plane. */
inline bool planes_intersect_box3(const Frustumf &f, const Box3f &box, int count) {
    const v128_t lo = v4f::load3(&box.min.x);
    const v128_t hi = v4f::load3(&box.max.x);
    for (int i = 0; i < count; i++) {
        const Plane3f &p = f.planes[i];
        const v128_t n = v4f::load3(&p.normal.x);
        const v128_t positive = v4f::select(hi, lo, v4f::ge(n, v4f::zero()));
        if (v4f::dot3(n, positive) + p.constant < 0.0f) return false;
    }
    return true;
}

inline bool intersects_box3(const Frustumf &f, const Box3f &box) { return planes_intersect_box3(f, box, 6); }
inline bool sides_intersects_box3(const Frustumf &f, const Box3f &box) { return planes_intersect_box3(f, box, 4); }

inline bool planes_contain_point(const Frustumf &f, const Float3 &p, int count) {
    for (int i = 0; i < count; i++) {
        if (plane3f::distance_to_point(f.planes[i], p) < 0.0f) return false;
    }
    return true;
}

inline bool contains_point(const Frustumf &f, const Float3 &p) { return planes_contain_point(f, p, 6); }
inline bool sides_contains_point(const Frustumf &f, const Float3 &p) { return planes_contain_point(f, p, 4); }

/** clips the ray's parameter against each plane's half-space; a ray parallel to a plane must start inside it. */
inline bool planes_intersect_ray(const Frustumf &f, const Float3 &origin, const Float3 &direction, int count) {
    float tmin = 0.0f;
    float tmax = __builtin_inff();
    for (int i = 0; i < count; i++) {
        const Plane3f &p = f.planes[i];
        const float s0 = plane3f::distance_to_point(p, origin);
        const float r = vec3f::dot(p.normal, direction);
        if (scalar::abs(r) < 1e-10f) {
            if (s0 < 0.0f) return false;
        } else {
            const float t = -s0 / r;
            if (r > 0.0f) {
                if (t > tmin) tmin = t;
            } else {
                if (t < tmax) tmax = t;
            }
        }
        if (tmin > tmax) return false;
    }
    return true;
}

inline bool intersects_ray(const Frustumf &f, const Float3 &origin, const Float3 &direction) {
    return planes_intersect_ray(f, origin, direction, 6);
}
inline bool sides_intersects_ray(const Frustumf &f, const Float3 &origin, const Float3 &direction) {
    return planes_intersect_ray(f, origin, direction, 4);
}

/** the eight corners: near then far, each left-bottom, left-top, right-top, right-bottom. */
inline void corners(Float3 *out, const Frustumf &f) {
    const Plane3f *p = f.planes;
    plane3f::intersect(out[0], p[4], p[0], p[2]);
    plane3f::intersect(out[1], p[4], p[0], p[3]);
    plane3f::intersect(out[2], p[4], p[1], p[3]);
    plane3f::intersect(out[3], p[4], p[1], p[2]);
    plane3f::intersect(out[4], p[5], p[0], p[2]);
    plane3f::intersect(out[5], p[5], p[0], p[3]);
    plane3f::intersect(out[6], p[5], p[1], p[3]);
    plane3f::intersect(out[7], p[5], p[1], p[2]);
}

}  // namespace frustumf

TUMBLE_NAMESPACE_END
