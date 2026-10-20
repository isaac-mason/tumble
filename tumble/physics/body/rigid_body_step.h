#pragma once

#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

namespace rigid_body_step {

/**
 * apply a position step (linear velocity * dt) to the body.
 * used in position solver for baumgarte stabilization.
 *
 * the translation dof mask applies to the step, not to the resulting position: a locked axis stops
 * the body moving along it, it does not snap the body to zero there.
 *
 * modifies center_of_mass_position directly (the primary property for physics).
 * derive_transform syncs the derived position and aabb from it.
 */
void add_position_step(rigid_body::RigidBody *body, Vec3f linear_velocity_times_delta_time);

/** subtract a position step (linear velocity * dt) from the body. see add_position_step on the dof mask. */
void sub_position_step(rigid_body::RigidBody *body, Vec3f linear_velocity_times_delta_time);

/**
 * update rotation using an euler step (used during position solver).
 *
 * this uses a proper axis-angle quaternion construction instead of a first-order
 * approximation, which is more accurate for large rotations (important for kinematic bodies).
 */
void add_rotation_step(rigid_body::RigidBody *body, Vec3f angular_velocity_times_delta_time);

/** update rotation using an euler step in the opposite direction (used during position solver). */
void sub_rotation_step(rigid_body::RigidBody *body, Vec3f angular_velocity_times_delta_time);

/**
 * re-derive the cached world transform from the authoritative state the step helpers mutate:
 * `position` from `center_of_mass_position` and `quaternion`, then the world `aabb` from that.
 * whatever moves the centre of mass owes a call to this before the next reader of either.
 *
 * does not publish to the broadphase.
 */
void derive_transform(rigid_body::RigidBody *body);

}  // namespace rigid_body_step

TUMBLE_NAMESPACE_END
