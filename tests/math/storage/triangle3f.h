#pragma once

#include "tumble/math/simd.h"
#include "tumble/math/types.h"
#include "math/storage/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace triangle3f {

inline Box3f &bounds(Box3f &out, const Float3 &a, const Float3 &b, const Float3 &c) {
    const V128 va = v4f::load3(&a.x);
    const V128 vb = v4f::load3(&b.x);
    const V128 vc = v4f::load3(&c.x);
    vec3f::store(out.min, v4f::min(va, v4f::min(vb, vc)));
    vec3f::store(out.max, v4f::max(va, v4f::max(vb, vc)));
    return out;
}

}  // namespace triangle3f

TUMBLE_NAMESPACE_END
