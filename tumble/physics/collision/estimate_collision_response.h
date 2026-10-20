#pragma once

// crashcat's estimate-collision-response.ts, the types: the predicted outcome of a contact before
// the solver runs, from a few pgs iterations over local copies of the velocities. the estimation is
// estimate_collision_response.cpp.

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/manifold/manifold.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace estimate_collision_response {

struct CollisionEstimationResult {
    /** predicted velocities after the contact */
    Vec3f linear_velocity_1;
    Vec3f angular_velocity_1;
    Vec3f linear_velocity_2;
    Vec3f angular_velocity_2;
    /** the mean of the contact midpoints, relative to the manifold's base offset; friction acts here */
    Vec3f friction_point;
    /** per point normal impulse, kg m/s; num_impulses of them */
    float contact_impulse[manifold::MAX_CLIPPING_CONTACT_POINTS];
    uint32_t num_impulses;
    /** manifold-level linear friction impulse, kg m/s, in the contact plane */
    Vec3f friction_impulse;
    /** manifold-level angular friction impulse around the normal, kg m^2/s */
    float angular_friction_impulse;
};

inline CollisionEstimationResult create_result() {
    CollisionEstimationResult r = {};
    return r;
}

/**
 * predicts the velocities and impulses of a contact with `num_iterations` of pgs over the manifold. restitution
 * applies only above min_velocity_for_restitution along the normal (crashcat's defaults: 1 m/s, 10 iterations).
 */
void estimate_collision_response(CollisionEstimationResult &result, const rigid_body::RigidBody &body_1, const rigid_body::RigidBody &body_2,
                                 const manifold::ContactManifold &manifold, float combined_friction, float combined_restitution,
                                 float min_velocity_for_restitution, uint32_t num_iterations);

}  // namespace estimate_collision_response

TUMBLE_NAMESPACE_END
