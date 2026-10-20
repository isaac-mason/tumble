#pragma once

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/constraints/constraint_part/spring_part.h"
#include "tumble/physics/constraints/constraint_part/spring_settings.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

// one dimension along an axis (a slider, a distance, a six dof translation): its jacobian terms and accumulated impulse
namespace axis_constraint_part {

struct AxisConstraintPart {
    /** (r1 + u) x axis and r2 x axis, the jacobian's angular terms */
    Float3 r1_plus_u_x_axis;
    Float3 r2_x_axis;
    /** the angular terms through each body's inverse inertia */
    Float3 inv_i1_r1_plus_u_x_axis;
    Float3 inv_i2_r2_x_axis;
    /** 1 / k; 0 when inactive */
    float effective_mass;
    /** the impulse applied so far this step */
    float total_lambda;
    spring_part::SpringPart spring;
};

inline AxisConstraintPart create() {
    AxisConstraintPart part = {};
    return part;
}

inline void reset(AxisConstraintPart &part) {
    part.r1_plus_u_x_axis = {0.0f, 0.0f, 0.0f};
    part.r2_x_axis = {0.0f, 0.0f, 0.0f};
    part.inv_i1_r1_plus_u_x_axis = {0.0f, 0.0f, 0.0f};
    part.inv_i2_r2_x_axis = {0.0f, 0.0f, 0.0f};
    part.effective_mass = 0.0f;
    part.total_lambda = 0.0f;
    spring_part::reset(part.spring);
}

inline void deactivate(AxisConstraintPart &part) {
    part.effective_mass = 0.0f;
    part.total_lambda = 0.0f;
}

inline bool is_active(const AxisConstraintPart &part) { return part.effective_mass != 0.0f; }

/**
 * the effective mass and cached jacobian terms of a hard constraint (no spring). inv_mass_a / inv_mass_b
 * are the inverse masses (0 if static), inv_inertia_a / inv_inertia_b the world space inverse inertias,
 * r1_plus_u and r2 the moment arms (contact point - center of mass), axis the normalized constraint axis
 * and bias the velocity bias (restitution, speculative contacts).
 */
void calculate_constraint_properties(AxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                     float inv_mass_a, float inv_mass_b, const Mat4f &inv_inertia_a,
                                     const Mat4f &inv_inertia_b, Vec3f r1_plus_u, Vec3f r2, Vec3f axis,
                                     float bias);

/** a soft constraint from a frequency (hz) and damping ratio; c is the value of the constraint equation */
void calculate_constraint_properties_with_frequency_and_damping(AxisConstraintPart &part, float delta_time,
                                                                rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                                                float inv_mass_a, float inv_mass_b, const Mat4f &inv_inertia_a,
                                                                const Mat4f &inv_inertia_b, Vec3f r1_plus_u,
                                                                Vec3f r2, Vec3f axis, float bias, float c,
                                                                float frequency, float damping);

/** a soft constraint from a stiffness k and damping c; c is the value of the constraint equation */
void calculate_constraint_properties_with_stiffness_and_damping(AxisConstraintPart &part, float delta_time,
                                                                rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                                                float inv_mass_a, float inv_mass_b, const Mat4f &inv_inertia_a,
                                                                const Mat4f &inv_inertia_b, Vec3f r1_plus_u,
                                                                Vec3f r2, Vec3f axis, float bias, float c,
                                                                float stiffness, float damping);

/** a limit from spring settings, by the spring mode; hard when the spring has no frequency or stiffness */
void calculate_constraint_properties_with_settings(AxisConstraintPart &part, float delta_time, rigid_body::RigidBody &body_a,
                                                   rigid_body::RigidBody &body_b, float inv_mass_a, float inv_mass_b,
                                                   const Mat4f &inv_inertia_a, const Mat4f &inv_inertia_b,
                                                   Vec3f r1_plus_u, Vec3f r2, Vec3f axis, float bias,
                                                   float c, const spring_settings::SpringSettings &settings);

/** a motor from spring settings, which must have stiffness or damping; damping alone drives the velocity only */
void calculate_constraint_properties_with_settings_for_motor(AxisConstraintPart &part, float delta_time,
                                                             rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                                             float inv_mass_a, float inv_mass_b, const Mat4f &inv_inertia_a,
                                                             const Mat4f &inv_inertia_b, Vec3f r1_plus_u,
                                                             Vec3f r2, Vec3f axis, float bias, float c,
                                                             const spring_settings::SpringSettings &settings);

/** override the total lagrange multiplier, e.g. to seed warm starting */
void set_total_lambda(AxisConstraintPart &part, float lambda);

/** the current total lagrange multiplier */
float get_total_lambda_value(const AxisConstraintPart &part);

/** apply the previous frame's impulse scaled by warm_start_ratio (dt_new / dt_old), once before the velocity iterations */
void warm_start(AxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b, float inv_mass_a,
                float inv_mass_b, Vec3f axis, float warm_start_ratio);

/**
 * one velocity iteration with the bodies' own inverse masses, the total lambda clamped to
 * [min_lambda, max_lambda]. true if an impulse was applied.
 */
bool solve_velocity_constraint(AxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                               Vec3f axis, float min_lambda, float max_lambda);

/**
 * baumgarte stabilization with the bodies' own inverse masses. c is the constraint error (penetration
 * depth, or 0 if separated). hard constraints only. true if a correction was applied.
 */
bool solve_position_constraint(AxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                               Vec3f axis, float c, float baumgarte);

}  // namespace axis_constraint_part

TUMBLE_NAMESPACE_END
