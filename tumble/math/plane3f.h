#pragma once

// the register plane (Plane in types.h): normal . p + constant = 0. load and store cross to and from a record's
// Plane3f; each function gives the bits of its scalar reference (test/math/storage/plane3f.h).

#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace plane3f {

inline Plane load(const Plane3f &plane) { return {vec3f::load(plane.normal), plane.constant}; }
inline Plane3f &store(Plane3f &out, const Plane &plane) {
    vec3f::store(out.normal, plane.normal);
    out.constant = plane.constant;
    return out;
}

inline float distance_to_point(const Plane &plane, Vec3f point) { return vec3f::dot(plane.normal, point) + plane.constant; }

inline Vec3f project_point(const Plane &plane, Vec3f point) { return vec3f::scale_and_add(point, plane.normal, -distance_to_point(plane, point)); }

}  // namespace plane3f

TUMBLE_NAMESPACE_END
