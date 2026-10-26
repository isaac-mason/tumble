#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/constraints/constraint_part/point_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/rotation_euler_constraint_part.h"
#include "tumble/physics/constraints/constraints.h"

TUMBLE_NAMESPACE_BEGIN

struct RigidBody;

// two bodies welded to move as one: all six degrees of freedom

/** fixed constraint removes 6 DOF (3 translation + 3 rotation) */
struct FixedConstraint {
    ConstraintBase base;
    /** attachment point on body 1 relative to its center of mass, in body space */
    Vec3f local_space_position1;
    /** attachment point on body 2 relative to its center of mass, in body space */
    Vec3f local_space_position2;
    /** inverse of initial relative orientation from body 1 to body 2 */
    Quatf inv_initial_orientation;
    /** point constraint for translation (3 DOF) */
    PointConstraintPart point_constraint_part;
    /** rotation constraint (3 DOF), keeps orientations locked */
    RotationEulerConstraintPart rotation_constraint_part;
};

struct FixedConstraintSettings {
    /** pivot point on body 1, ignored when auto_detect_point */
    Float3 point_a = {0.0f, 0.0f, 0.0f};
    /** reference axis x for body 1 (for orientation) */
    Float3 axis_x_a = {1.0f, 0.0f, 0.0f};
    /** reference axis y for body 1 (for orientation) */
    Float3 axis_y_a = {0.0f, 1.0f, 0.0f};
    /** pivot point on body 2, ignored when auto_detect_point */
    Float3 point_b = {0.0f, 0.0f, 0.0f};
    /** reference axis x for body 2 (for orientation) */
    Float3 axis_x_b = {1.0f, 0.0f, 0.0f};
    /** reference axis y for body 2 (for orientation) */
    Float3 axis_y_b = {0.0f, 1.0f, 0.0f};
    ConstraintSpace space = ConstraintSpace::WORLD;
    /** world space only: place the attachment point between the bodies' centers of mass, weighted towards the lighter body */
    bool auto_detect_point = false;
    /** higher numbers are more likely to be solved correctly (solved later) */
    uint32_t constraint_priority = 0;
    /** 0 takes the world's iteration counts */
    uint32_t num_velocity_steps_override = 0;
    uint32_t num_position_steps_override = 0;
};

namespace fixed_constraint {

/** a fixed constraint between two live bodies, linked into both bodies' constraint_ids; INVALID_CONSTRAINT_ID (Error::QUOTA) when the pool is at its most */
ConstraintId create(World &world, RigidBody *body_a,
                                   RigidBody *body_b, const FixedConstraintSettings &settings);
/** null when the id is stale or not a fixed constraint */
FixedConstraint *get(World &world, ConstraintId id);

void setup_velocity(FixedConstraint &constraint, Bodies &bodies, float delta_time);
void warm_start_velocity(FixedConstraint &constraint, Bodies &bodies, float warm_start_impulse_ratio);
bool solve_velocity(FixedConstraint &constraint, Bodies &bodies, float delta_time);
bool solve_position(FixedConstraint &constraint, Bodies &bodies, float delta_time, float baumgarte);
/** the body's centre of mass moved by delta_com in its local space: the point on it follows, so the joint holds where it was */
void notify_shape_changed(FixedConstraint &constraint, BodyId body_id, Vec3f delta_com);
void reset_warm_start(FixedConstraint &constraint);
void get_iteration_overrides(Constraints::IterationOverrides &out, const FixedConstraint &constraint);
void get_sort_fields(Constraints::SortFields &out, const FixedConstraint &constraint);

/** total lambda of the position constraint (for debugging/inspection) */
Vec3f get_total_lambda_position(const FixedConstraint &constraint);
/** total lambda of the rotation constraint (for debugging/inspection) */
Vec3f get_total_lambda_rotation(const FixedConstraint &constraint);

}  // namespace fixed_constraint

TUMBLE_NAMESPACE_END
