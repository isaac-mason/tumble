#pragma once

// crashcat's angle-constraint-part.ts: constrains rotation along 1 axis (hinge limits and motors, angular friction).

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/constraints/constraint_part/spring_part.h"
#include "tumble/physics/constraints/constraint_part/spring_settings.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

namespace angle_constraint_part {

/**
 * constrains rotation along 1 axis.
 *
 * based on: "constraints derivation for rigid body simulation in 3d" - daniel chappuis, section 2.4.5
 *
 * constraint equation (eq 108): C = theta(t) - theta_target
 * jacobian (eq 109): J = [0, -a^T, 0, a^T]
 * where a = axis around which rotation is constrained (normalized, world space)
 */
struct AngleConstraintPart {
    /** I1^-1 * axis (cached for velocity/position integration) */
    Vec3f inv_i1_axis;
    /** I2^-1 * axis (cached for velocity/position integration) */
    Vec3f inv_i2_axis;
    /** effective mass: 1 / (J M^-1 J^T) */
    float effective_mass;
    /** spring part for soft constraints */
    spring_part::SpringPart spring;
    /** accumulated impulse (for warm starting) */
    float total_lambda;
};

inline AngleConstraintPart create() {
    AngleConstraintPart part = {};
    return part;
}

/** deactivate this constraint part (zero out effective mass and lambda) */
inline void deactivate(AngleConstraintPart &part) {
    part.effective_mass = 0.0f;
    part.total_lambda = 0.0f;
}

/** check if constraint is active (has non-zero effective mass) */
inline bool is_active(const AngleConstraintPart &part) { return part.effective_mass != 0.0f; }

/** hard constraint, no spring. bias is b in lambda = J v + b */
void calculate_constraint_properties(AngleConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                     Vec3f world_space_axis, float bias = 0.0f);

/** soft constraint from a frequency (hz) and damping ratio. c is the value of the constraint equation (angle error) */
void calculate_constraint_properties_with_frequency_and_damping(AngleConstraintPart &part, float delta_time,
                                                                rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                                                Vec3f world_space_axis, float bias, float c,
                                                                float frequency, float damping);

/** soft constraint from a stiffness k and damping coefficient c */
void calculate_constraint_properties_with_stiffness_and_damping(AngleConstraintPart &part, float delta_time,
                                                                rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                                                Vec3f world_space_axis, float bias, float c,
                                                                float stiffness, float damping);

/** a limit from spring settings, by the spring mode; hard when the spring has no frequency or stiffness */
void calculate_constraint_properties_with_settings(AngleConstraintPart &part, float delta_time, rigid_body::RigidBody &body_a,
                                                   rigid_body::RigidBody &body_b, Vec3f world_space_axis, float bias,
                                                   float c, const spring_settings::SpringSettings &settings);

/** a motor from spring settings, which must have stiffness or damping; damping alone drives the velocity only */
void calculate_constraint_properties_with_settings_for_motor(AngleConstraintPart &part, float delta_time,
                                                             rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                                             Vec3f world_space_axis, float bias, float c,
                                                             const spring_settings::SpringSettings &settings);

/** apply warm start impulse from previous frame */
void warm_start(AngleConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                float warm_start_impulse_ratio);

/** enforces d/dt C = 0, the accumulated impulse clamped to [min_lambda, max_lambda]. true if an impulse was applied */
bool solve_velocity_constraint(AngleConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                               Vec3f world_space_axis, float min_lambda, float max_lambda);

/** baumgarte stabilization of C (angle error in radians), hard constraints only. true if a correction was applied */
bool solve_position_constraint(AngleConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b, float c,
                               float baumgarte);

/** total accumulated lambda (impulse) */
float get_total_lambda(const AngleConstraintPart &part);

/** set total lambda (for warm starting with a specific value) */
void set_total_lambda(AngleConstraintPart &part, float lambda);

}  // namespace angle_constraint_part

TUMBLE_NAMESPACE_END
