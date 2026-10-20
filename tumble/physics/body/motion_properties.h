#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/mass_properties.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace motion_properties {

enum MotionQuality : uint32_t {
    /** fast, but can tunnel through thin geometry at high speed */
    MOTION_QUALITY_DISCRETE = 0,
    /** cast along the velocity so a fast body cannot tunnel; dearer than discrete */
    MOTION_QUALITY_LINEAR_CAST = 1,
};

/** the step stamp that bypasses get_world_inverse_inertia's memo */
constexpr uint32_t STEP_STAMP_NONE = 0xffffffffu;

struct MotionProperties {
    Float3 linear_velocity;
    Float3 angular_velocity;

    Float3 inv_inertia_diagonal;
    Float4 inertia_rotation;

    /** accumulated over the step */
    Float3 force;
    /** accumulated over the step */
    Float3 torque;

    float inv_mass;
    float linear_damping;
    float angular_damping;
    float max_linear_velocity;
    float max_angular_velocity;
    /** scales gravity */
    float gravity_factor;

    MotionQuality motion_quality;
    /** a dof mask */
    uint32_t allowed_degrees_of_freedom;
    /** 0 takes the world's */
    uint32_t num_velocity_steps_override;
    /** 0 takes the world's */
    uint32_t num_position_steps_override;

    Spheref sleep_test_spheres[3];
    bool allow_sleeping;
    float sleep_test_timer;

    /** get_world_inverse_inertia's memo, meaningful while world_inverse_inertia_stamp matches the step's stamp */
    Float4x4 world_inverse_inertia;
    /** STEP_STAMP_NONE when the memo holds nothing usable */
    uint32_t world_inverse_inertia_stamp;
};

MotionProperties create();

void add_force(MotionProperties &motion_properties, Vec3f force);
void add_torque(MotionProperties &motion_properties, Vec3f torque);
/** a force and the torque it makes about the centre of mass */
void add_force_at_position(MotionProperties &motion_properties, Vec3f force, Vec3f world_position, Vec3f center_of_mass_position);
/** at the centre of mass */
void add_impulse(MotionProperties &motion_properties, Vec3f impulse);
void add_angular_impulse(MotionProperties &motion_properties, Vec3f impulse, Quatf quaternion);
void add_impulse_at_position(MotionProperties &motion_properties, Vec3f impulse, Vec3f world_position,
                             Vec3f center_of_mass_position, Quatf quaternion);

/**
 * the inertia tensor's principal moments (eigenvalues, descending) and a right handed rotation (eigenvectors as
 * columns), by jacobi's eigenvalue algorithm for 3x3 symmetric matrices; false if it did not converge (numerical
 * recipes 11.1)
 */
bool decompose_principal_moments_of_inertia(const Mat4f &inertia, Mat4f &out_rotation, Vec3f &out_diagonal);

/** the inverse mass and inertia, with the axes `allowed_dofs` locks removed */
void set_mass_properties(MotionProperties &motion_properties, uint32_t allowed_dofs,
                         const mass_properties::MassProperties &mass_properties);

/** R Ri diag(inv_inertia_diagonal) Ri^T R^T, Ri the inertia rotation, the rows and columns of locked rotation axes zeroed */
Mat4f get_inverse_inertia_for_rotation(const MotionProperties &motion_properties, const Mat4f &body_rotation);

/**
 * a dynamic body's world space inverse inertia, memoised per step: with a stamp other than STEP_STAMP_NONE it is
 * computed at most once per body per step, which holds for the velocity constraints because nothing rotates a body
 * between force integration and the end of the velocity solve, and every rotation write outside the step resets the
 * stamp. STEP_STAMP_NONE computes it fresh and leaves the memo alone: for where bodies rotate between calls (the
 * position solver, ccd, getters)
 */
Mat4f get_world_inverse_inertia(MotionProperties &motion_properties, Quatf body_quaternion, uint32_t step_stamp);

void clamp_linear_velocity(MotionProperties &motion_properties);
/** at most max_length long */
Vec3f clamp_length(Vec3f velocity, float max_length);
void clamp_angular_velocity(MotionProperties &motion_properties);
/** clamped */
void set_linear_velocity(MotionProperties &motion_properties, Vec3f velocity);
/** clamped */
void set_angular_velocity(MotionProperties &motion_properties, Vec3f velocity);
/** clamped */
void add_linear_velocity(MotionProperties &motion_properties, Vec3f velocity_delta);
/** clamped */
void add_angular_velocity(MotionProperties &motion_properties, Vec3f velocity_delta);

/** the solver's steps; locked translation axes zeroed */
void add_linear_velocity_step(MotionProperties &motion_properties, Vec3f linear_velocity_change);
void sub_linear_velocity_step(MotionProperties &motion_properties, Vec3f linear_velocity_change);
void add_angular_velocity_step(MotionProperties &motion_properties, Vec3f angular_velocity_change);
void sub_angular_velocity_step(MotionProperties &motion_properties, Vec3f angular_velocity_change);

/** to a new mass (> 0), the inertia kept proportional to mass */
void scale_to_mass(MotionProperties &motion_properties, float new_mass);

/** the velocities that move the body by delta_position and delta_rotation in delta_time; nothing for delta_time <= 0 */
void move_kinematic(MotionProperties &motion_properties, Vec3f delta_position, Quatf delta_rotation, float delta_time);

/** locked translation axes zeroed */
Vec3f apply_translation_dof_constraint(Vec3f velocity, uint32_t allowed_dofs);
/** locked rotation axes zeroed */
Vec3f apply_rotation_dof_constraint(Vec3f angular_velocity, uint32_t allowed_dofs);

/** by the world space inverse inertia without forming it, the dof constraints applied */
Vec3f multiply_world_space_inverse_inertia_by_vector(const MotionProperties &motion_properties, Quatf body_quaternion, Vec3f vector);

/** the torque T = -w x (I w) that makes spinning bodies tumble (the tennis racket theorem) */
void apply_gyroscopic_force(MotionProperties &motion_properties, Quatf body_quaternion, float delta_time);

}  // namespace motion_properties

TUMBLE_NAMESPACE_END
