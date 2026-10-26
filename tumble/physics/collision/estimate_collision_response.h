#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/constraints/contact_settings.h"
#include "tumble/physics/manifold/manifold.h"

TUMBLE_NAMESPACE_BEGIN

// a contact's predicted outcome before the solver runs, from a few pgs iterations over local copies of the velocities

struct CollisionEstimationResult {
    /** predicted velocities after the contact */
    Vec3f linear_velocity_1;
    Vec3f angular_velocity_1;
    Vec3f linear_velocity_2;
    Vec3f angular_velocity_2;
    /** halfway between the two bodies' friction points (each the mean of its own contact points), relative to the manifold's base offset */
    Vec3f friction_point;
    /** per point normal impulse, kg m/s; num_impulses of them */
    float contact_impulse[manifold::MAX_CLIPPING_CONTACT_POINTS];
    uint32_t num_impulses;
    /** manifold-level linear friction impulse, kg m/s, in the contact plane */
    Vec3f friction_impulse;
    /** manifold-level angular friction impulse around the normal, kg m^2/s */
    float angular_friction_impulse;
};

namespace estimate_collision_response {

inline CollisionEstimationResult create_result() {
    CollisionEstimationResult r = {};
    return r;
}

/**
 * predicts the velocities and impulses of a contact with `num_iterations` of pgs over the manifold, with the contact's
 * settings (its materials, mass and inertia scales and surface velocities; a sensor has no response). accurate for two
 * bodies colliding, not when more collide at once, as it knows only this contact. restitution applies only above
 * min_velocity_for_restitution along the normal; with 1 iteration no friction is calculated
 */
void estimate_collision_response(CollisionEstimationResult &result, const RigidBody &body_1, const RigidBody &body_2,
                                 const ContactManifold &manifold, const ContactSettings &settings,
                                 float min_velocity_for_restitution = 1.0f, uint32_t num_iterations = 10);

}  // namespace estimate_collision_response

TUMBLE_NAMESPACE_END
