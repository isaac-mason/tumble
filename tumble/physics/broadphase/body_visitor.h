#pragma once

#include <stdint.h>

#include "tumble/math/scalar.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

namespace body_visitor {

struct BodyVisitor;

/** called when visiting a body. reaches its own state through `visitor->context`. */
typedef void (*Visit)(BodyVisitor *visitor, rigid_body::RigidBody *body);

struct BodyVisitor {
    /** whether the broadphase traversal should exit early */
    bool should_exit;
    /**
     * closest-hit fraction found so far, in [0, 1] of the ray length. cast_ray prunes any node whose
     * fat-aabb entry fraction is >= this. infinity for visitors that don't cast, which turns distance
     * pruning off.
     */
    float early_out_fraction;
    /** a cast's entry fraction into the visited body's box, in [0, 1]; 0 for visitors that don't cast */
    float fraction;
    Visit visit;
    void *context;
};

/** a visitor that doesn't cast: no early exit, no distance pruning. */
inline BodyVisitor create(Visit visit, void *context) { return {false, scalar::INF, 0.0f, visit, context}; }

}  // namespace body_visitor

TUMBLE_NAMESPACE_END
