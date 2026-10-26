#pragma once

#include "tumble/config.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/simd.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace aabox {

constexpr float INF = __builtin_inff();

inline AABox load(const Box3f &box) { return {vec3f::load(box.min), vec3f::load(box.max)}; }
inline Box3f &store(Box3f &out, const AABox &box) {
    vec3f::store(out.min, box.min);
    vec3f::store(out.max, box.max);
    return out;
}

inline AABox make(Vec3f min, Vec3f max) { return {min, max}; }
inline AABox empty() { return {vec3f::splat(INF), vec3f::splat(-INF)}; }

/** min above max on any axis. */
inline bool is_empty(const AABox &box) { return (v4f::bitmask(v4f::gt(box.min.v, box.max.v)) & 7u) != 0u; }

inline AABox from_triangle(Vec3f a, Vec3f b, Vec3f c) {
    return {{v4f::min(a.v, v4f::min(b.v, c.v))}, {v4f::max(a.v, v4f::max(b.v, c.v))}};
}

inline AABox expand_by_point(const AABox &box, Vec3f point) { return {{v4f::min(box.min.v, point.v)}, {v4f::max(box.max.v, point.v)}}; }
inline AABox expand_by_extents(const AABox &box, Vec3f extents) { return {vec3f::subtract(box.min, extents), vec3f::add(box.max, extents)}; }
inline AABox expand_by_margin(const AABox &box, float margin) {
    const Vec3f m = vec3f::splat(margin);
    return {vec3f::subtract(box.min, m), vec3f::add(box.max, m)};
}
inline AABox union_(const AABox &a, const AABox &b) { return {{v4f::min(a.min.v, b.min.v)}, {v4f::max(a.max.v, b.max.v)}}; }

inline Vec3f center(const AABox &box) { return vec3f::scale(vec3f::add(box.min, box.max), 0.5f); }
inline Vec3f extents(const AABox &box) { return vec3f::scale(vec3f::subtract(box.max, box.min), 0.5f); }
inline Vec3f size(const AABox &box) { return vec3f::subtract(box.max, box.min); }

inline float surface_area(const AABox &box) {
    const Vec3f s = size(box);
    const float width = vec3f::x(s), height = vec3f::y(s), depth = vec3f::z(s);
    return 2.0f * (width * height + width * depth + height * depth);
}

/** scales about the origin; a negative scale swaps the faces back into order. */
inline AABox scale(const AABox &box, Vec3f s) {
    const V128 lo = v4f::mul(box.min.v, s.v);
    const V128 hi = v4f::mul(box.max.v, s.v);
    return {{v4f::min(lo, hi)}, {v4f::max(lo, hi)}};
}

/** centre through the matrix, extents through its absolute upper 3x3; empty stays empty */
inline AABox transform_mat4(const AABox &box, const Mat4f &mat) {
    if (is_empty(box)) return empty();
    const V128 c = v4f::mul(v4f::add(box.min.v, box.max.v), v4f::splat(0.5f));
    const V128 e = v4f::mul(v4f::sub(box.max.v, box.min.v), v4f::splat(0.5f));
    V128 tc = v4f::mul_add(mat.c[0], v4f::splat(v4f::lane<0>(c)), mat.c[3]);
    tc = v4f::mul_add(mat.c[1], v4f::splat(v4f::lane<1>(c)), tc);
    tc = v4f::mul_add(mat.c[2], v4f::splat(v4f::lane<2>(c)), tc);
    V128 te = v4f::mul(v4f::abs(mat.c[0]), v4f::splat(v4f::lane<0>(e)));
    te = v4f::mul_add(v4f::abs(mat.c[1]), v4f::splat(v4f::lane<1>(e)), te);
    te = v4f::mul_add(v4f::abs(mat.c[2]), v4f::splat(v4f::lane<2>(e)), te);
    return {{vec3f::fix_w(v4f::sub(tc, te))}, {vec3f::fix_w(v4f::add(tc, te))}};
}

inline bool contains_point(const AABox &box, Vec3f point) {
    const V128 inside = v4f::bit_and(v4f::ge(point.v, box.min.v), v4f::le(point.v, box.max.v));
    return (v4f::bitmask(inside) & 7u) == 7u;
}

inline bool contains(const AABox &container, const AABox &contained) {
    const V128 inside = v4f::bit_and(v4f::ge(contained.min.v, container.min.v), v4f::le(contained.max.v, container.max.v));
    return (v4f::bitmask(inside) & 7u) == 7u;
}

/** closed overlap: touching faces count. */
inline bool intersects(const AABox &a, const AABox &b) {
    const V128 lo = v4f::le(a.min.v, b.max.v);
    const V128 hi = v4f::ge(a.max.v, b.min.v);
    return (v4f::bitmask(lo) & v4f::bitmask(hi) & 7u) == 7u;
}

}  // namespace aabox

TUMBLE_NAMESPACE_END
