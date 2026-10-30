#pragma once

#include "tumble/config.h"
#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace plane3f {

inline Plane make(Vec3f normal, float constant) { return {v4f::set_lane<3>(normal.v, constant)}; }

inline Vec3f normal(const Plane &plane) { return {vec3f::fix_w(plane.v)}; }
inline float constant(const Plane &plane) { return v4f::lane<3>(plane.v); }

inline Plane load(const Plane3f &plane) { return make(vec3f::load(plane.normal), plane.constant); }
inline Plane3f &store(Plane3f &out, const Plane &plane) {
    vec3f::store(out.normal, normal(plane));
    out.constant = constant(plane);
    return out;
}

inline float distance_to_point(const Plane &plane, Vec3f point) { return vec3f::dot(normal(plane), point) + constant(plane); }

inline Vec3f project_point(const Plane &plane, Vec3f point) { return vec3f::scale_and_add(point, normal(plane), -distance_to_point(plane, point)); }

}  // namespace plane3f

TUMBLE_NAMESPACE_END
