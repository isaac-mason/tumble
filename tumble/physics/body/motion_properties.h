#pragma once

// crashcat's motion-properties.ts: the velocities, mass, inertia and limits of a moving body.

#include <stdint.h>

#include "math/types.h"
#include "physics/body/mass_properties.h"

namespace motion_properties {

/** motion quality options for collision detection */
enum MotionQuality : uint32_t {
    /**
     * discrete collision detection - standard discrete update.
     * fast but can tunnel through thin geometry at high velocities.
     */
    MOTION_QUALITY_DISCRETE = 0,
    /**
     * continuous collision detection via linear cast.
     * prevents fast-moving objects from tunneling through thin geometry
     * by casting shapes along their velocity vectors.
     * more expensive than DISCRETE.
     */
    MOTION_QUALITY_LINEAR_CAST = 1,
};

/** step stamp that bypasses the per-step memo in get_world_inverse_inertia: compute fresh into `out` */
constexpr uint32_t STEP_STAMP_NONE = 0xffffffffu;

/** motion properties of a rigid body */
struct MotionProperties {
    /** linear velocity of the body */
    Float3 linear_velocity;
    /** angular velocity of the body */
    Float3 angular_velocity;

    /** inverse inertia diagonal */
    Float3 inv_inertia_diagonal;
    /** inertia rotation */
    Float4 inertia_rotation;

    /** force accumulator */
    Float3 force;
    /** torque accumulator */
    Float3 torque;

    /** inverse mass (1/mass) */
    float inv_mass;
    /** linear damping coefficient */
    float linear_damping;
    /** angular damping coefficient */
    float angular_damping;
    /** maximum linear velocity */
    float max_linear_velocity;
    /** maximum angular velocity */
    float max_angular_velocity;
    /** gravity multiplier factor */
    float gravity_factor;

    /** motion quality for collision detection */
    MotionQuality motion_quality;
    /** allowed degrees of freedom bitmask */
    uint32_t allowed_degrees_of_freedom;
    /** velocity solver iterations override (0 = use default) */
    uint32_t num_velocity_steps_override;
    /** position solver iterations override (0 = use default) */
    uint32_t num_position_steps_override;

    /** spheres used for sleeping test */
    Spheref sleep_test_spheres[3];
    /** whether the body is allowed to sleep */
    bool allow_sleeping;
    /** timer for sleeping test */
    float sleep_test_timer;

    /**
     * per-step memo of the world space inverse inertia, see get_world_inverse_inertia. only meaningful
     * while world_inverse_inertia_stamp matches the current step stamp.
     */
    Float4x4 world_inverse_inertia;
    /** step stamp the memo was computed for, STEP_STAMP_NONE when it holds nothing usable */
    uint32_t world_inverse_inertia_stamp;
};

MotionProperties create();

/** adds a force to the force accumulator. */
void add_force(MotionProperties &motion_properties, Vec3f force);
/** adds a torque to the torque accumulator. */
void add_torque(MotionProperties &motion_properties, Vec3f torque);
/** adds a force at a specific world-space position, generating both force and torque. */
void add_force_at_position(MotionProperties &motion_properties, Vec3f force, Vec3f world_position, Vec3f center_of_mass_position);
/** applies an impulse at the center of mass, instantly changing linear velocity. */
void add_impulse(MotionProperties &motion_properties, Vec3f impulse);
/** applies an angular impulse, instantly changing angular velocity. */
void add_angular_impulse(MotionProperties &motion_properties, Vec3f impulse, Quatf quaternion);
/** applies an impulse at a specific world-space position, changing both linear and angular velocity. */
void add_impulse_at_position(MotionProperties &motion_properties, Vec3f impulse, Vec3f world_position,
                             Vec3f center_of_mass_position, Quatf quaternion);

/**
 * decomposes the inertia tensor into principal moments (eigenvalues, descending) and a right handed
 * rotation (eigenvectors as columns), by jacobi's eigenvalue algorithm for 3x3 symmetric matrices.
 * false if it did not converge. based on jolt's EigenValueSymmetric (numerical recipes 11.1).
 */
bool decompose_principal_moments_of_inertia(const Mat4f &inertia, Mat4f &out_rotation, Vec3f &out_diagonal);

/** sets the inverse mass and inertia from mass properties, with the axes `allowed_dofs` locks removed. */
void set_mass_properties(MotionProperties &motion_properties, uint32_t allowed_dofs,
                         const mass_properties::MassProperties &mass_properties);

/**
 * the world space inverse inertia for a body rotation: R Ri diag(inv_inertia_diagonal) Ri^T R^T,
 * where Ri is the inertia rotation, with the rows and columns of locked rotation axes zeroed.
 */
Mat4f get_inverse_inertia_for_rotation(const MotionProperties &motion_properties, const Mat4f &body_rotation);

/**
 * world space inverse inertia of a dynamic body, memoised per step.
 *
 * with a step stamp other than STEP_STAMP_NONE the matrix is computed at most once per body per
 * step: the first call for a new stamp computes it into the body's own storage, later calls load
 * that. this is sound for velocity constraint setup because nothing rotates a body between force
 * integration and the end of the velocity solve, and every rotation write that can happen outside
 * the step resets the stamp.
 *
 * with STEP_STAMP_NONE the matrix is computed fresh and the memo is left alone. use that where
 * bodies rotate between calls: the position solver, ccd, and public getters.
 *
 * the caller checks that the body is dynamic; non-dynamic bodies contribute a zero matrix and never
 * reach this.
 */
Mat4f get_world_inverse_inertia(MotionProperties &motion_properties, Quatf body_quaternion, uint32_t step_stamp);

/** clamps linear velocity to the maximum allowed value */
void clamp_linear_velocity(MotionProperties &motion_properties);
/** a velocity clamped to a length of at most max_length */
Vec3f clamp_length(Vec3f velocity, float max_length);
/** clamps angular velocity to the maximum allowed value */
void clamp_angular_velocity(MotionProperties &motion_properties);
/** sets linear velocity and clamps it to the maximum allowed value */
void set_linear_velocity(MotionProperties &motion_properties, Vec3f velocity);
/** sets angular velocity and clamps it to the maximum allowed value */
void set_angular_velocity(MotionProperties &motion_properties, Vec3f velocity);
/** adds to the linear velocity and clamps to the maximum allowed value */
void add_linear_velocity(MotionProperties &motion_properties, Vec3f velocity_delta);
/** adds to the angular velocity and clamps to the maximum allowed value */
void add_angular_velocity(MotionProperties &motion_properties, Vec3f velocity_delta);

/** add a linear velocity step (used during constraint solving), with locked translation axes zeroed */
void add_linear_velocity_step(MotionProperties &motion_properties, Vec3f linear_velocity_change);
/** subtract a linear velocity step (used during constraint solving), with locked translation axes zeroed */
void sub_linear_velocity_step(MotionProperties &motion_properties, Vec3f linear_velocity_change);
/** add an angular velocity step (used during constraint solving) */
void add_angular_velocity_step(MotionProperties &motion_properties, Vec3f angular_velocity_change);
/** subtract an angular velocity step (used during constraint solving) */
void sub_angular_velocity_step(MotionProperties &motion_properties, Vec3f angular_velocity_change);

/** scales the inverse mass and inverse inertia diagonal to a new mass (> 0), keeping the inertia proportional to mass */
void scale_to_mass(MotionProperties &motion_properties, float new_mass);

/**
 * set velocity of body such that it will rotate/translate by delta_position/delta_rotation in
 * delta_time seconds. a no-op for delta_time <= 0.
 */
void move_kinematic(MotionProperties &motion_properties, Vec3f delta_position, Quatf delta_rotation, float delta_time);

/** the velocity with the components of locked translation axes zeroed */
Vec3f apply_translation_dof_constraint(Vec3f velocity, uint32_t allowed_dofs);
/** the angular velocity with the components of locked rotation axes zeroed */
Vec3f apply_rotation_dof_constraint(Vec3f angular_velocity, uint32_t allowed_dofs);

/**
 * multiplies a vector by the world-space inverse inertia matrix without forming the matrix,
 * applying the dof constraints.
 */
Vec3f multiply_world_space_inverse_inertia_by_vector(const MotionProperties &motion_properties, Quatf body_quaternion, Vec3f vector);

/**
 * apply the gyroscopic force (aka dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
 * this simulates the realistic behavior of spinning bodies by applying torque T = -w x (I w)
 */
void apply_gyroscopic_force(MotionProperties &motion_properties, Quatf body_quaternion, float delta_time);

}  // namespace motion_properties
