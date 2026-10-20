#pragma once

#include "math/storage/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/math/types.h"
#include "math/storage/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace box3f {

constexpr float INF = __builtin_inff();

inline Box3f create() { return {{INF, INF, INF}, {-INF, -INF, -INF}}; }
inline Box3f &set(Box3f &out, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z) {
    out.min = {min_x, min_y, min_z};
    out.max = {max_x, max_y, max_z};
    return out;
}

inline Box3f &empty(Box3f &out) { return set(out, INF, INF, INF, -INF, -INF, -INF); }

/** min above max on any axis. */
inline bool is_empty(const Box3f &box) { return box.min.x > box.max.x || box.min.y > box.max.y || box.min.z > box.max.z; }

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

}  // namespace box3f

TUMBLE_NAMESPACE_END
