#pragma once

// the math package's box3.ts over Box3f. an empty box is +inf min, -inf max, as the js. the min and
// max lanes make expand, union, scale and the overlap tests one op each.

#include "math/storage/mat4f.h"
#include "math/scalar.h"
#include "math/simd.h"
#include "math/types.h"
#include "math/storage/vec3f.h"

namespace box3f {

constexpr float INF = __builtin_inff();

inline Box3f create() { return {{INF, INF, INF}, {-INF, -INF, -INF}}; }
inline Box3f clone(const Box3f &box) { return box; }

inline Box3f &copy(Box3f &out, const Box3f &box) {
    out = box;
    return out;
}

inline Box3f &set(Box3f &out, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z) {
    out.min = {min_x, min_y, min_z};
    out.max = {max_x, max_y, max_z};
    return out;
}

inline Box3f &set_from_vectors(Box3f &out, const Float3 &min, const Float3 &max) {
    out.min = min;
    out.max = max;
    return out;
}

inline Float3 &min(Float3 &out, const Box3f &box) { return vec3f::copy(out, box.min); }
inline Float3 &max(Float3 &out, const Box3f &box) { return vec3f::copy(out, box.max); }

inline Box3f &empty(Box3f &out) { return set(out, INF, INF, INF, -INF, -INF, -INF); }

/** min above max on any axis. */
inline bool is_empty(const Box3f &box) { return box.min.x > box.max.x || box.min.y > box.max.y || box.min.z > box.max.z; }

inline bool exact_equals(const Box3f &a, const Box3f &b) {
    return vec3f::exact_equals(a.min, b.min) && vec3f::exact_equals(a.max, b.max);
}

inline bool equals(const Box3f &a, const Box3f &b) { return vec3f::equals(a.min, b.min) && vec3f::equals(a.max, b.max); }

inline Box3f &set_from_center_and_size(Box3f &out, const Float3 &center, const Float3 &size) {
    const v128_t half = v4f::mul(v4f::load3(&size.x), v4f::splat(0.5f));
    const v128_t c = v4f::load3(&center.x);
    vec3f::store(out.min, v4f::sub(c, half));
    vec3f::store(out.max, v4f::add(c, half));
    return out;
}

inline Box3f &expand_by_point(Box3f &out, const Box3f &box, const Float3 &point) {
    const v128_t p = v4f::load3(&point.x);
    vec3f::store(out.min, v4f::min(v4f::load3(&box.min.x), p));
    vec3f::store(out.max, v4f::max(v4f::load3(&box.max.x), p));
    return out;
}

inline Box3f &expand_by_extents(Box3f &out, const Box3f &box, const Float3 &vector) {
    const v128_t e = v4f::load3(&vector.x);
    vec3f::store(out.min, v4f::sub(v4f::load3(&box.min.x), e));
    vec3f::store(out.max, v4f::add(v4f::load3(&box.max.x), e));
    return out;
}

inline Box3f &expand_by_margin(Box3f &out, const Box3f &box, float margin) {
    const v128_t m = v4f::splat(margin);
    vec3f::store(out.min, v4f::sub(v4f::load3(&box.min.x), m));
    vec3f::store(out.max, v4f::add(v4f::load3(&box.max.x), m));
    return out;
}

inline Box3f &union_(Box3f &out, const Box3f &a, const Box3f &b) {
    vec3f::store(out.min, v4f::min(v4f::load3(&a.min.x), v4f::load3(&b.min.x)));
    vec3f::store(out.max, v4f::max(v4f::load3(&a.max.x), v4f::load3(&b.max.x)));
    return out;
}

inline Float3 &center(Float3 &out, const Box3f &box) {
    return vec3f::store(out, v4f::mul(v4f::add(v4f::load3(&box.min.x), v4f::load3(&box.max.x)), v4f::splat(0.5f)));
}

inline Float3 &extents(Float3 &out, const Box3f &box) {
    return vec3f::store(out, v4f::mul(v4f::sub(v4f::load3(&box.max.x), v4f::load3(&box.min.x)), v4f::splat(0.5f)));
}

inline Float3 &size(Float3 &out, const Box3f &box) {
    return vec3f::store(out, v4f::sub(v4f::load3(&box.max.x), v4f::load3(&box.min.x)));
}

inline float surface_area(const Box3f &box) {
    const float width = box.max.x - box.min.x;
    const float height = box.max.y - box.min.y;
    const float depth = box.max.z - box.min.z;
    return 2.0f * (width * height + width * depth + height * depth);
}

/** scales about the origin; a negative scale swaps the faces back into order. */
inline Box3f &scale(Box3f &out, const Box3f &box, const Float3 &s) {
    const v128_t vs = v4f::load3(&s.x);
    const v128_t lo = v4f::mul(v4f::load3(&box.min.x), vs);
    const v128_t hi = v4f::mul(v4f::load3(&box.max.x), vs);
    vec3f::store(out.min, v4f::min(lo, hi));
    vec3f::store(out.max, v4f::max(lo, hi));
    return out;
}

/** the box of the transformed box: centre through the matrix, extents through its absolute upper 3x3. empty stays empty. */
inline Box3f &transform_mat4(Box3f &out, const Box3f &box, const Float4x4 &mat) {
    if (is_empty(box)) return empty(out);
    const v128_t lo = v4f::load3(&box.min.x);
    const v128_t hi = v4f::load3(&box.max.x);
    const v128_t c = v4f::mul(v4f::add(lo, hi), v4f::splat(0.5f));
    const v128_t e = v4f::mul(v4f::sub(hi, lo), v4f::splat(0.5f));
    const v128_t m0 = mat4f::col(mat, 0);
    const v128_t m1 = mat4f::col(mat, 1);
    const v128_t m2 = mat4f::col(mat, 2);
    v128_t tc = v4f::mul_add(m0, v4f::splat(v4f::lane<0>(c)), mat4f::col(mat, 3));
    tc = v4f::mul_add(m1, v4f::splat(v4f::lane<1>(c)), tc);
    tc = v4f::mul_add(m2, v4f::splat(v4f::lane<2>(c)), tc);
    v128_t te = v4f::mul(v4f::abs(m0), v4f::splat(v4f::lane<0>(e)));
    te = v4f::mul_add(v4f::abs(m1), v4f::splat(v4f::lane<1>(e)), te);
    te = v4f::mul_add(v4f::abs(m2), v4f::splat(v4f::lane<2>(e)), te);
    vec3f::store(out.min, v4f::sub(tc, te));
    vec3f::store(out.max, v4f::add(tc, te));
    return out;
}

inline bool contains_point(const Box3f &box, const Float3 &point) {
    return point.x >= box.min.x && point.x <= box.max.x && point.y >= box.min.y && point.y <= box.max.y &&
           point.z >= box.min.z && point.z <= box.max.z;
}

inline bool contains_box3(const Box3f &container, const Box3f &contained) {
    return contained.min.x >= container.min.x && contained.max.x <= container.max.x && contained.min.y >= container.min.y &&
           contained.max.y <= container.max.y && contained.min.z >= container.min.z && contained.max.z <= container.max.z;
}

/** closed overlap: touching faces count. */
inline bool intersects_box3(const Box3f &a, const Box3f &b) {
    const v128_t lo = v4f::le(v4f::load3(&a.min.x), v4f::load3(&b.max.x));
    const v128_t hi = v4f::ge(v4f::load3(&a.max.x), v4f::load3(&b.min.x));
    return (v4f::bitmask(lo) & v4f::bitmask(hi) & 7u) == 7u;
}

/** the separating axis test over the box's three axes, the triangle normal and the nine edge cross products. */
inline bool intersects_triangle3(const Box3f &box, const Float3 &a, const Float3 &b, const Float3 &c) {
    if (is_empty(box)) return false;
    const float cx = (box.min.x + box.max.x) * 0.5f;
    const float cy = (box.min.y + box.max.y) * 0.5f;
    const float cz = (box.min.z + box.max.z) * 0.5f;
    const float ex = box.max.x - cx;
    const float ey = box.max.y - cy;
    const float ez = box.max.z - cz;
    const float v0x = a.x - cx, v0y = a.y - cy, v0z = a.z - cz;
    const float v1x = b.x - cx, v1y = b.y - cy, v1z = b.z - cz;
    const float v2x = c.x - cx, v2y = c.y - cy, v2z = c.z - cz;
    const auto min3 = [](float p, float q, float r) { return scalar::min(p, scalar::min(q, r)); };
    const auto max3 = [](float p, float q, float r) { return scalar::max(p, scalar::max(q, r)); };
    if (min3(v0x, v1x, v2x) > ex || max3(v0x, v1x, v2x) < -ex) return false;
    if (min3(v0y, v1y, v2y) > ey || max3(v0y, v1y, v2y) < -ey) return false;
    if (min3(v0z, v1z, v2z) > ez || max3(v0z, v1z, v2z) < -ez) return false;
    const float f0x = v1x - v0x, f0y = v1y - v0y, f0z = v1z - v0z;
    const float f1x = v2x - v1x, f1y = v2y - v1y, f1z = v2z - v1z;
    const float f2x = v0x - v2x, f2y = v0y - v2y, f2z = v0z - v2z;
    const float nx = f0y * f1z - f0z * f1y;
    const float ny = f0z * f1x - f0x * f1z;
    const float nz = f0x * f1y - f0y * f1x;
    const float d = v0x * nx + v0y * ny + v0z * nz;
    float r = ex * scalar::abs(nx) + ey * scalar::abs(ny) + ez * scalar::abs(nz);
    if (d > r || d < -r) return false;
    const auto axis = [](float pa, float pb, float rr) { return scalar::min(pa, pb) > rr || scalar::max(pa, pb) < -rr; };
    if (axis(v0z * f0y - v0y * f0z, v2z * f0y - v2y * f0z, ey * scalar::abs(f0z) + ez * scalar::abs(f0y))) return false;
    if (axis(v0z * f1y - v0y * f1z, v1z * f1y - v1y * f1z, ey * scalar::abs(f1z) + ez * scalar::abs(f1y))) return false;
    if (axis(v0z * f2y - v0y * f2z, v1z * f2y - v1y * f2z, ey * scalar::abs(f2z) + ez * scalar::abs(f2y))) return false;
    if (axis(v0x * f0z - v0z * f0x, v2x * f0z - v2z * f0x, ex * scalar::abs(f0z) + ez * scalar::abs(f0x))) return false;
    if (axis(v0x * f1z - v0z * f1x, v1x * f1z - v1z * f1x, ex * scalar::abs(f1z) + ez * scalar::abs(f1x))) return false;
    if (axis(v0x * f2z - v0z * f2x, v1x * f2z - v1z * f2x, ex * scalar::abs(f2z) + ez * scalar::abs(f2x))) return false;
    if (axis(v0y * f0x - v0x * f0y, v2y * f0x - v2x * f0y, ex * scalar::abs(f0y) + ey * scalar::abs(f0x))) return false;
    if (axis(v0y * f1x - v0x * f1y, v1y * f1x - v1x * f1y, ex * scalar::abs(f1y) + ey * scalar::abs(f1x))) return false;
    if (axis(v0y * f2x - v0x * f2y, v1y * f2x - v1x * f2y, ex * scalar::abs(f2y) + ey * scalar::abs(f2x))) return false;
    return true;
}

inline bool intersects_sphere(const Box3f &box, const Spheref &sphere) {
    const float cx = sphere.center.x, cy = sphere.center.y, cz = sphere.center.z;
    const float dx = cx < box.min.x ? box.min.x - cx : (cx > box.max.x ? cx - box.max.x : 0.0f);
    const float dy = cy < box.min.y ? box.min.y - cy : (cy > box.max.y ? cy - box.max.y : 0.0f);
    const float dz = cz < box.min.z ? box.min.z - cz : (cz > box.max.z ? cz - box.max.z : 0.0f);
    return dx * dx + dy * dy + dz * dz <= sphere.radius * sphere.radius;
}

inline bool intersects_plane3(const Box3f &box, const Plane3f &plane) {
    const float nx = plane.normal.x, ny = plane.normal.y, nz = plane.normal.z;
    const float s = nx * (box.min.x + box.max.x) + ny * (box.min.y + box.max.y) + nz * (box.min.z + box.max.z) +
                    2.0f * plane.constant;
    const float r = scalar::abs(nx) * (box.max.x - box.min.x) + scalar::abs(ny) * (box.max.y - box.min.y) +
                    scalar::abs(nz) * (box.max.z - box.min.z);
    return s <= r && s >= -r;
}

}  // namespace box3f
