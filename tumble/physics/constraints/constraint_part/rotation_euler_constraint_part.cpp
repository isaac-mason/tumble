#include "tumble/physics/constraints/constraint_part/rotation_euler_constraint_part.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/vec4f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/rigid_body_step.h"

TUMBLE_NAMESPACE_BEGIN

namespace rotation_euler_constraint_part {

Quatf get_inv_initial_orientation(Quatf rotation1, Quatf rotation2) { return quatf::multiply(quatf::conjugate(rotation2), rotation1); }

/** the rotation whose matrix has columns x, y and x cross y */
static Quatf rotation_of_axes(Vec3f axis_x, Vec3f axis_y) {
    const Vec3f axis_z = vec3f::cross(axis_x, axis_y);
    return quatf::from_mat4(mat4f::make_columns({axis_x.v}, {axis_y.v}, {axis_z.v}, vec4f::make(0.0f, 0.0f, 0.0f, 1.0f)));
}

Quatf get_inv_initial_orientation_xy(Vec3f axis_x1, Vec3f axis_y1, Vec3f axis_x2, Vec3f axis_y2) {
    // identical axes, identity
    if (vec3f::exact_equals(axis_x1, axis_x2) && vec3f::exact_equals(axis_y1, axis_y2)) return quatf::identity();

    // constraint2 * constraint1^-1
    return quatf::multiply(rotation_of_axes(axis_x2, axis_y2), quatf::conjugate(rotation_of_axes(axis_x1, axis_y1)));
}

void calculate_constraint_properties(RotationEulerConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                     uint32_t step_stamp) {
    // world space inverse inertia of both bodies
    part.inv_i1 = body_a.motion_type == motion_type::MOTION_DYNAMIC
                      ? motion_properties::get_world_inverse_inertia(body_a.motion_properties, quatf::load(body_a.quaternion), step_stamp)
                      : mat4f::zero();
    part.inv_i2 = body_b.motion_type == motion_type::MOTION_DYNAMIC
                      ? motion_properties::get_world_inverse_inertia(body_b.motion_properties, quatf::load(body_b.quaternion), step_stamp)
                      : mat4f::zero();

    // K = I1^-1 + I2^-1 (3x3 only); singular when both bodies are static/kinematic
    Mat4f inertia_sum = mat4f::add(part.inv_i1, part.inv_i2);
    Mat4f effective_mass;
    if (mat4f::invert3x3(effective_mass, inertia_sum)) {
        part.effective_mass = effective_mass;
        return;
    }

    // a zero column is an axis locked on both bodies: identity there, which changes nothing, as every impulse goes through
    // inv_i1 or inv_i2 and so comes to zero on that axis
    for (int axis = 0; axis < 3; axis++) {
        const V128 column = inertia_sum.c[axis];
        if (v4f::lane<0>(column) == 0.0f && v4f::lane<1>(column) == 0.0f && v4f::lane<2>(column) == 0.0f && v4f::lane<3>(column) == 0.0f) {
            inertia_sum.c[axis] = v4f::make(axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f, axis == 2 ? 1.0f : 0.0f, 0.0f);
        }
    }
    if (mat4f::invert3x3(effective_mass, inertia_sum)) {
        part.effective_mass = effective_mass;
    } else {
        deactivate(part);
    }
}

/** an angular impulse on both bodies: dw = I^-1 * lambda */
static bool apply_velocity_step(RotationEulerConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                Vec3f lambda) {
    if (vec3f::squared_length(lambda) == 0.0f) return false;

    // body a: subtract angular impulse
    if (body_a.motion_type == motion_type::MOTION_DYNAMIC) {
        motion_properties::sub_angular_velocity_step(body_a.motion_properties, mat4f::multiply3x3_vec(part.inv_i1, lambda));
    }

    // body b: add angular impulse
    if (body_b.motion_type == motion_type::MOTION_DYNAMIC) {
        motion_properties::add_angular_velocity_step(body_b.motion_properties, mat4f::multiply3x3_vec(part.inv_i2, lambda));
    }

    return true;
}

void warm_start(RotationEulerConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                float warm_start_impulse_ratio) {
    part.total_lambda = vec3f::scale(part.total_lambda, warm_start_impulse_ratio);
    apply_velocity_step(part, body_a, body_b, part.total_lambda);
}

bool solve_velocity_constraint(RotationEulerConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b) {
    // J v = w1 - w2; static bodies contribute zero
    const Vec3f w1 = body_a.motion_type != motion_type::MOTION_STATIC ? vec3f::load(body_a.motion_properties.angular_velocity) : vec3f::zero();
    const Vec3f w2 = body_b.motion_type != motion_type::MOTION_STATIC ? vec3f::load(body_b.motion_properties.angular_velocity) : vec3f::zero();

    // lambda = K * (J v); the negation is the subtraction from body a
    const Vec3f lambda = mat4f::multiply3x3_vec(part.effective_mass, vec3f::subtract(w1, w2));

    part.total_lambda = vec3f::add(part.total_lambda, lambda);

    return apply_velocity_step(part, body_a, body_b, lambda);
}

bool solve_position_constraint(RotationEulerConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                               Quatf inv_initial_orientation, float baumgarte) {
    // the rotation should be q2 = q1 * r0, but with drift q2 = diff * q1 * r0 => diff = q2 * r0^-1 * q1^-1
    Quatf diff = quatf::multiply(quatf::multiply(quatf::load(body_b.quaternion), inv_initial_orientation),
                                 quatf::conjugate(quatf::load(body_a.quaternion)));

    // positive w for consistent angle extraction
    if (quatf::w(diff) < 0.0f) diff = quatf::scale(diff, -1.0f);

    // for small angles: error ~ 2 * [x, y, z]
    const Vec3f error = vec3f::scale(quatf::xyz(diff), 2.0f);

    if (vec3f::squared_length(error) < 1e-12f) return false;

    // lambda = -K * beta * error (3x3 only)
    const Vec3f lambda = vec3f::scale(mat4f::multiply3x3_vec(part.effective_mass, error), -baumgarte);

    // body a: subtract rotation
    if (body_a.motion_type == motion_type::MOTION_DYNAMIC) rigid_body_step::sub_rotation_step(&body_a, mat4f::multiply3x3_vec(part.inv_i1, lambda));

    // body b: add rotation
    if (body_b.motion_type == motion_type::MOTION_DYNAMIC) rigid_body_step::add_rotation_step(&body_b, mat4f::multiply3x3_vec(part.inv_i2, lambda));

    return true;
}

}  // namespace rotation_euler_constraint_part

TUMBLE_NAMESPACE_END
