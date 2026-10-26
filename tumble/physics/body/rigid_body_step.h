#pragma once

#include "tumble/config.h"
#include "tumble/math/types.h"

TUMBLE_NAMESPACE_BEGIN

struct RigidBody;

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
void add_position_step(RigidBody *body, Vec3f linear_velocity_times_delta_time);

/** subtract a position step (linear velocity * dt) from the body. see add_position_step on the dof mask. */
void sub_position_step(RigidBody *body, Vec3f linear_velocity_times_delta_time);

/**
 * turns the body by the rotation vector: a rotation of its length about its direction, before the current one,
 * normalized. jolt's Body::AddRotationStep; the position solve's step.
 */
void add_rotation_step(RigidBody *body, Vec3f angular_velocity_times_delta_time);

/** add_rotation_step the other way. */
void sub_rotation_step(RigidBody *body, Vec3f angular_velocity_times_delta_time);

/**
 * re-derive the cached world transform from the authoritative state the step helpers mutate:
 * `position` from `center_of_mass_position` and `quaternion`, then the world `aabb` from that.
 * whatever moves the centre of mass owes a call to this before the next reader of either.
 *
 * does not publish to the broadphase.
 */
void derive_transform(RigidBody *body);

}  // namespace rigid_body_step

TUMBLE_NAMESPACE_END
