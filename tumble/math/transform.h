#pragma once

#include "tumble/config.h"
#include "tumble/math/quatf.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace transform {

inline Transform load(const Transformf &a) { return {vec3f::load(a.position), quatf::load(a.quaternion), vec3f::load(a.scale)}; }
inline Transformf &store(Transformf &out, const Transform &a) {
    vec3f::store(out.position, a.position);
    quatf::store(out.quaternion, a.quaternion);
    vec3f::store(out.scale, a.scale);
    return out;
}

inline Transform make(Vec3f position, Quatf quaternion, Vec3f scale) { return {position, quaternion, scale}; }
/** at the position and turned, unscaled */
inline Transform make(Vec3f position, Quatf quaternion) { return {position, quaternion, vec3f::splat(1.0f)}; }
inline Transform identity() { return {vec3f::zero(), quatf::identity(), vec3f::splat(1.0f)}; }

}  // namespace transform

TUMBLE_NAMESPACE_END
