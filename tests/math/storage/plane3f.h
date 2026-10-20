#pragma once

#include "tumble/math/plane3f.h"
#include "math/storage/vec3f.h"
#include "math/storage/mat4f.h"

TUMBLE_NAMESPACE_BEGIN

namespace plane3f {

inline float distance_to_point(const Plane3f &plane, const Float3 &point) { return vec3f::dot(plane.normal, point) + plane.constant; }

inline Float3 &project_point(Float3 &out, const Plane3f &plane, const Float3 &point) {
    const float distance = distance_to_point(plane, point);
    return vec3f::scale_and_add(out, point, plane.normal, -distance);
}

}  // namespace plane3f

TUMBLE_NAMESPACE_END
