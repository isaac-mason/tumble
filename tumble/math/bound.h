#pragma once

// the math functions bound to js, storage in and out: each loads, runs the register function and stores. included by
// the bindings alone, so nothing else sees storage-typed overloads.

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace vec3f {

inline Float3 &transform_quat(Float3 &out, const Float3 &a, const Float4 &q) { return store(out, transform_quat(load(a), quatf::load(q))); }

inline Float3 &transform_mat4(Float3 &out, const Float3 &a, const Float4x4 &m) { return store(out, transform_mat4(load(a), mat4f::load(m))); }

}  // namespace vec3f

namespace mat4f {

inline Float4 &decompose(Float4 &out_rotation, Float3 &out_translation, Float3 &out_scale, const Float4x4 &mat) {
    Vec3f translation, scale;
    const Quatf rotation = decompose(translation, scale, load(mat));
    vec3f::store(out_translation, translation);
    vec3f::store(out_scale, scale);
    return quatf::store(out_rotation, rotation);
}

}  // namespace mat4f

TUMBLE_NAMESPACE_END
