#include "tumble/physics/body/motion_properties.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/vec4f.h"
#include "tumble/physics/body/dof.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace motion_properties {

MotionProperties create() {
    MotionProperties out;

    out.linear_velocity = {0.0f, 0.0f, 0.0f};
    out.angular_velocity = {0.0f, 0.0f, 0.0f};

    out.inv_inertia_diagonal = {0.0f, 0.0f, 0.0f};
    out.inertia_rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    out.force = {0.0f, 0.0f, 0.0f};
    out.torque = {0.0f, 0.0f, 0.0f};

    out.inv_mass = 0.0f;
    out.linear_damping = 0.0f;
    out.angular_damping = 0.0f;
    out.max_linear_velocity = scalar::INF;
    out.max_angular_velocity = scalar::INF;
    out.gravity_factor = 1.0f;

    out.motion_quality = MOTION_QUALITY_DISCRETE;
    out.allowed_degrees_of_freedom = dof::DOF_ALL;
    out.num_velocity_steps_override = 0;
    out.num_position_steps_override = 0;

    for (int i = 0; i < 3; i++) out.sleep_test_spheres[i] = {{0.0f, 0.0f, 0.0f}, 0.0f};
    out.allow_sleeping = true;
    out.sleep_test_timer = 0.0f;

    mat4f::store(out.world_inverse_inertia, mat4f::zero());
    out.world_inverse_inertia_stamp = STEP_STAMP_NONE;

    return out;
}

void add_force(MotionProperties &motion_properties, Vec3f force) {
    vec3f::store(motion_properties.force, vec3f::add(vec3f::load(motion_properties.force), force));
}

void add_torque(MotionProperties &motion_properties, Vec3f torque) {
    vec3f::store(motion_properties.torque, vec3f::add(vec3f::load(motion_properties.torque), torque));
}

void add_force_at_position(MotionProperties &motion_properties, Vec3f force, Vec3f world_position, Vec3f center_of_mass_position) {
    vec3f::store(motion_properties.force, vec3f::add(vec3f::load(motion_properties.force), force));
    // calculate torque: t = r x F, where r is moment arm from center of mass
    const Vec3f torque = vec3f::cross(vec3f::subtract(world_position, center_of_mass_position), force);
    vec3f::store(motion_properties.torque, vec3f::add(vec3f::load(motion_properties.torque), torque));
}

void add_impulse(MotionProperties &motion_properties, Vec3f impulse) {
    if (motion_properties.inv_mass <= 0.0f) return;
    const Vec3f linear_delta = vec3f::scale(impulse, motion_properties.inv_mass);
    vec3f::store(motion_properties.linear_velocity, vec3f::add(vec3f::load(motion_properties.linear_velocity), linear_delta));
    clamp_linear_velocity(motion_properties);
}

void add_angular_impulse(MotionProperties &motion_properties, Vec3f impulse, Quatf quaternion) {
    const uint32_t allowed_rotation_axis = (motion_properties.allowed_degrees_of_freedom >> 3) & 0b111;
    if (allowed_rotation_axis == 0) return;
    const Vec3f angular_delta = multiply_world_space_inverse_inertia_by_vector(motion_properties, quaternion, impulse);
    vec3f::store(motion_properties.angular_velocity, vec3f::add(vec3f::load(motion_properties.angular_velocity), angular_delta));
    clamp_angular_velocity(motion_properties);
}

void add_impulse_at_position(MotionProperties &motion_properties, Vec3f impulse, Vec3f world_position,
                             Vec3f center_of_mass_position, Quatf quaternion) {
    if (motion_properties.inv_mass > 0.0f) {
        const Vec3f linear_delta = vec3f::scale(impulse, motion_properties.inv_mass);
        vec3f::store(motion_properties.linear_velocity, vec3f::add(vec3f::load(motion_properties.linear_velocity), linear_delta));
        clamp_linear_velocity(motion_properties);
    }

    // apply angular impulse from moment arm: t = r x impulse, dw = I^-1 t
    const uint32_t allowed_rotation_axis = (motion_properties.allowed_degrees_of_freedom >> 3) & 0b111;

    if (allowed_rotation_axis != 0) {
        const Vec3f torque = vec3f::cross(vec3f::subtract(world_position, center_of_mass_position), impulse);
        const Vec3f angular_delta = multiply_world_space_inverse_inertia_by_vector(motion_properties, quaternion, torque);
        vec3f::store(motion_properties.angular_velocity, vec3f::add(vec3f::load(motion_properties.angular_velocity), angular_delta));
        clamp_angular_velocity(motion_properties);
    }
}

static constexpr float EPSILON = 1e-10f;
static constexpr int MAX_JACOBI_ITERATIONS = 50;

bool decompose_principal_moments_of_inertia(const Mat4f &inertia, Mat4f &out_rotation, Vec3f &out_diagonal) {
    // the 3x3 inertia tensor, column-major, so element (row, col) of m is m[col * 3 + row]
    float m[9];
    for (int col = 0; col < 3; col++) {
        const Vec3f axis = mat4f::axis(inertia, col);
        m[col * 3] = vec3f::x(axis);
        m[col * 3 + 1] = vec3f::y(axis);
        m[col * 3 + 2] = vec3f::z(axis);
    }

    float rotation[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float diagonal[3];

    float b[3];
    float z[3];

    for (int ip = 0; ip < 3; ip++) {
        // initialize b and the output to the diagonal of the matrix
        b[ip] = m[ip * 3 + ip];
        diagonal[ip] = m[ip * 3 + ip];
        z[ip] = 0.0f;
    }

    bool converged = false;
    for (int sweep = 0; sweep < MAX_JACOBI_ITERATIONS; sweep++) {
        // sum of the off-diagonal elements of the upper triangle, a(ip, iq) = m[iq * 3 + ip]
        float sm = 0.0f;
        for (int ip = 0; ip < 2; ip++) {
            for (int iq = ip + 1; iq < 3; iq++) {
                sm += scalar::abs(m[iq * 3 + ip]);
            }
        }
        const float avg_sm = sm / (3.0f * 3.0f);

        // normal return, convergence to machine underflow
        if (avg_sm < scalar::FLOAT_MIN_NORMAL) {
            converged = true;
            break;
        }

        // on the first three sweeps use a fraction of the sum of the off diagonal elements as threshold
        const float thresh = sweep < 4 ? 0.2f * avg_sm : scalar::FLOAT_MIN_NORMAL;

        for (int ip = 0; ip < 2; ip++) {
            for (int iq = ip + 1; iq < 3; iq++) {
                const int a_pq_idx = iq * 3 + ip;
                const float abs_a_pq = scalar::abs(m[a_pq_idx]);
                const float g = 100.0f * abs_a_pq;

                // after four sweeps, skip the rotation if the off-diagonal element is small
                if (sweep > 4 && scalar::abs(diagonal[ip]) + g == scalar::abs(diagonal[ip]) &&
                    scalar::abs(diagonal[iq]) + g == scalar::abs(diagonal[iq])) {
                    m[a_pq_idx] = 0.0f;
                } else if (abs_a_pq > thresh) {
                    const float h = diagonal[iq] - diagonal[ip];
                    const float abs_h = scalar::abs(h);

                    float t;
                    if (abs_h + g == abs_h) {
                        t = m[a_pq_idx] / h;
                    } else {
                        const float theta = (0.5f * h) / m[a_pq_idx];
                        t = 1.0f / (scalar::abs(theta) + scalar::sqrt(1.0f + theta * theta));
                        if (theta < 0.0f) t = -t;
                    }

                    const float c = 1.0f / scalar::sqrt(1.0f + t * t);
                    const float s = t * c;
                    const float tau = s / (1.0f + c);
                    const float rotation_h = t * m[a_pq_idx];

                    m[a_pq_idx] = 0.0f;

                    z[ip] -= rotation_h;
                    z[iq] += rotation_h;

                    diagonal[ip] -= rotation_h;
                    diagonal[iq] += rotation_h;

                    // the jacobi rotation of a(i,j) and a(k,l): g = a(i,j), h = a(k,l), a(i,j) = g - s*(h + g*tau), a(k,l) = h + s*(g - h*tau)

                    // for j in [0, ip): rotate a(j, ip) and a(j, iq)
                    for (int j = 0; j < ip; j++) {
                        const int idx_j_ip = ip * 3 + j;
                        const int idx_j_iq = iq * 3 + j;
                        const float g_val = m[idx_j_ip];
                        const float h_val = m[idx_j_iq];
                        m[idx_j_ip] = g_val - s * (h_val + g_val * tau);
                        m[idx_j_iq] = h_val + s * (g_val - h_val * tau);
                    }
                    // for j in [ip+1, iq): rotate a(ip, j) and a(j, iq)
                    for (int j = ip + 1; j < iq; j++) {
                        const int idx_ip_j = j * 3 + ip;
                        const int idx_j_iq = iq * 3 + j;
                        const float g_val = m[idx_ip_j];
                        const float h_val = m[idx_j_iq];
                        m[idx_ip_j] = g_val - s * (h_val + g_val * tau);
                        m[idx_j_iq] = h_val + s * (g_val - h_val * tau);
                    }
                    // for j in [iq+1, n): rotate a(ip, j) and a(iq, j)
                    for (int j = iq + 1; j < 3; j++) {
                        const int idx_ip_j = j * 3 + ip;
                        const int idx_iq_j = j * 3 + iq;
                        const float g_val = m[idx_ip_j];
                        const float h_val = m[idx_iq_j];
                        m[idx_ip_j] = g_val - s * (h_val + g_val * tau);
                        m[idx_iq_j] = h_val + s * (g_val - h_val * tau);
                    }

                    // apply the rotation to the eigenvectors: rotate v(j, ip) and v(j, iq)
                    for (int j = 0; j < 3; j++) {
                        const int idx_j_ip = ip * 3 + j;
                        const int idx_j_iq = iq * 3 + j;
                        const float g_val = rotation[idx_j_ip];
                        const float h_val = rotation[idx_j_iq];
                        rotation[idx_j_ip] = g_val - s * (h_val + g_val * tau);
                        rotation[idx_j_iq] = h_val + s * (g_val - h_val * tau);
                    }
                }
            }
        }

        // update eigenvalues with the sum and reinitialize z
        for (int ip = 0; ip < 3; ip++) {
            b[ip] += z[ip];
            diagonal[ip] = b[ip];
            z[ip] = 0.0f;
        }
    }

    // sort eigenvalues so that the biggest value goes first (descending order)
    int indices[3] = {0, 1, 2};
    if (diagonal[indices[0]] < diagonal[indices[1]]) {
        const int temp = indices[0];
        indices[0] = indices[1];
        indices[1] = temp;
    }
    if (diagonal[indices[1]] < diagonal[indices[2]]) {
        const int temp = indices[1];
        indices[1] = indices[2];
        indices[2] = temp;
    }
    if (diagonal[indices[0]] < diagonal[indices[1]]) {
        const int temp = indices[0];
        indices[0] = indices[1];
        indices[1] = temp;
    }

    // reorder eigenvalues and eigenvectors (columns)
    const float temp_diagonal[3] = {diagonal[0], diagonal[1], diagonal[2]};
    float temp_rotation[9];
    for (int i = 0; i < 9; i++) temp_rotation[i] = rotation[i];
    for (int i = 0; i < 3; i++) {
        diagonal[i] = temp_diagonal[indices[i]];
        for (int j = 0; j < 3; j++) {
            rotation[i * 3 + j] = temp_rotation[indices[i] * 3 + j];
        }
    }

    // make sure that the rotation matrix is a right handed matrix
    const Vec3f axis_x = vec3f::make(rotation[0], rotation[1], rotation[2]);
    const Vec3f axis_y = vec3f::make(rotation[3], rotation[4], rotation[5]);
    const Vec3f axis_z = vec3f::make(rotation[6], rotation[7], rotation[8]);
    if (vec3f::dot(vec3f::cross(axis_x, axis_y), axis_z) < 0.0f) {
        // flip z axis to make it right-handed
        rotation[6] = -rotation[6];
        rotation[7] = -rotation[7];
        rotation[8] = -rotation[8];
    }

    out_rotation = mat4f::make_columns(vec4f::make(rotation[0], rotation[1], rotation[2], 0.0f),
                                       vec4f::make(rotation[3], rotation[4], rotation[5], 0.0f),
                                       vec4f::make(rotation[6], rotation[7], rotation[8], 0.0f), vec4f::make(0.0f, 0.0f, 0.0f, 1.0f));
    out_diagonal = vec3f::make(diagonal[0], diagonal[1], diagonal[2]);
    return converged;
}

void set_mass_properties(MotionProperties &motion_properties, uint32_t allowed_dofs,
                         const mass_properties::MassProperties &mass_properties) {
    // the local inertia is about to change, so a memoised world inverse inertia is stale
    motion_properties.world_inverse_inertia_stamp = STEP_STAMP_NONE;

    motion_properties.allowed_degrees_of_freedom = allowed_dofs;

    // decompose DOFs
    const uint32_t allowed_translation_axis = allowed_dofs & 0b111;
    const uint32_t allowed_rotation_axis = (allowed_dofs >> 3) & 0b111;

    if (allowed_translation_axis == 0) {
        // no translation possible
        motion_properties.inv_mass = 0.0f;
    } else {
        // some shapes like the triangle mesh cannot calculate mass automatically; those need mass properties provided
        TUMBLE_ASSERT(mass_properties.mass > 0.0f);
        motion_properties.inv_mass = 1.0f / mass_properties.mass;
    }

    if (allowed_rotation_axis == 0) {
        // no rotation possible
        vec3f::store(motion_properties.inv_inertia_diagonal, vec3f::zero());
        quatf::store(motion_properties.inertia_rotation, quatf::identity());
    } else {
        Mat4f rotation;
        Vec3f diagonal;

        if (decompose_principal_moments_of_inertia(mat4f::load(mass_properties.inertia), rotation, diagonal)) {
            // check if diagonal is near zero
            const float diagonal_length_sq = vec3f::squared_length(diagonal);

            if (diagonal_length_sq > EPSILON * EPSILON) {
                // calculate reciprocal of diagonal elements
                const float dx = vec3f::x(diagonal), dy = vec3f::y(diagonal), dz = vec3f::z(diagonal);
                vec3f::store(motion_properties.inv_inertia_diagonal,
                             vec3f::make(dx != 0.0f ? 1.0f / dx : 0.0f, dy != 0.0f ? 1.0f / dy : 0.0f, dz != 0.0f ? 1.0f / dz : 0.0f));

                // convert rotation matrix to quaternion
                quatf::store(motion_properties.inertia_rotation, quatf::from_mat4(rotation));
            } else {
                // diagonal is near zero, fall back to sphere
                const float sphere_inertia = 2.5f * motion_properties.inv_mass;
                vec3f::store(motion_properties.inv_inertia_diagonal, vec3f::splat(sphere_inertia));
                quatf::store(motion_properties.inertia_rotation, quatf::identity());
            }
        } else {
            // failed! fall back to inertia tensor of sphere with radius 1.
            const float sphere_inertia = 2.5f * motion_properties.inv_mass;
            vec3f::store(motion_properties.inv_inertia_diagonal, vec3f::splat(sphere_inertia));
            quatf::store(motion_properties.inertia_rotation, quatf::identity());
        }
    }

    // can't lock all axes, use a static body for this; it would divide by zero later
    TUMBLE_ASSERT(motion_properties.inv_mass != 0.0f || vec3f::squared_length(vec3f::load(motion_properties.inv_inertia_diagonal)) != 0.0f);
}

Mat4f get_inverse_inertia_for_rotation(const MotionProperties &motion_properties, const Mat4f &body_rotation) {
    // r = body_rotation * inertia_rotation, scaled column by column by the inverse inertia diagonal, then times r^T
    const Mat4f rotation = mat4f::multiply3x3(body_rotation, mat4f::from_quat(quatf::load(motion_properties.inertia_rotation)));
    const Mat4f scaled = mat4f::scale(rotation, vec3f::load(motion_properties.inv_inertia_diagonal));
    Mat4f out = mat4f::multiply3x3_right_transposed(scaled, rotation);

    // mask out DOFs that are not allowed: the locked axes' columns, then their rows
    const uint32_t allowed_rotation_axis = (motion_properties.allowed_degrees_of_freedom >> 3) & 0b111;
    if (allowed_rotation_axis != 0b111) {
        // 1.0 if allowed, 0.0 if not
        const Vec3f mask = vec3f::make((allowed_rotation_axis & 0b001) ? 1.0f : 0.0f, (allowed_rotation_axis & 0b010) ? 1.0f : 0.0f,
                                       (allowed_rotation_axis & 0b100) ? 1.0f : 0.0f);
        out = mat4f::scale(out, mask);
        const V128 row_mask = v4f::set_lane<3>(mask.v, 1.0f);
        for (uint32_t i = 0; i < 3; i++) out.c[i] = v4f::mul(out.c[i], row_mask);
    }

    return out;
}

Mat4f get_world_inverse_inertia(MotionProperties &motion_properties, Quatf body_quaternion, uint32_t step_stamp) {
    if (step_stamp == STEP_STAMP_NONE) return get_inverse_inertia_for_rotation(motion_properties, mat4f::from_quat(body_quaternion));

    if (motion_properties.world_inverse_inertia_stamp != step_stamp) {
        const Mat4f inverse_inertia = get_inverse_inertia_for_rotation(motion_properties, mat4f::from_quat(body_quaternion));
        mat4f::store(motion_properties.world_inverse_inertia, inverse_inertia);
        motion_properties.world_inverse_inertia_stamp = step_stamp;
        return inverse_inertia;
    }

    return mat4f::load(motion_properties.world_inverse_inertia);
}

Vec3f clamp_length(Vec3f velocity, float max_length) {
    const float len_sq = vec3f::squared_length(velocity);
    if (len_sq > max_length * max_length) return vec3f::scale(vec3f::normalize(velocity), max_length);
    return velocity;
}

void clamp_linear_velocity(MotionProperties &motion_properties) {
    vec3f::store(motion_properties.linear_velocity, clamp_length(vec3f::load(motion_properties.linear_velocity), motion_properties.max_linear_velocity));
}

void clamp_angular_velocity(MotionProperties &motion_properties) {
    vec3f::store(motion_properties.angular_velocity,
                 clamp_length(vec3f::load(motion_properties.angular_velocity), motion_properties.max_angular_velocity));
}

void set_linear_velocity(MotionProperties &motion_properties, Vec3f velocity) {
    vec3f::store(motion_properties.linear_velocity, clamp_length(velocity, motion_properties.max_linear_velocity));
}

void set_angular_velocity(MotionProperties &motion_properties, Vec3f velocity) {
    vec3f::store(motion_properties.angular_velocity, clamp_length(velocity, motion_properties.max_angular_velocity));
}

void add_linear_velocity(MotionProperties &motion_properties, Vec3f velocity_delta) {
    set_linear_velocity(motion_properties, vec3f::add(vec3f::load(motion_properties.linear_velocity), velocity_delta));
}

void add_angular_velocity(MotionProperties &motion_properties, Vec3f velocity_delta) {
    set_angular_velocity(motion_properties, vec3f::add(vec3f::load(motion_properties.angular_velocity), velocity_delta));
}

void add_linear_velocity_step(MotionProperties &motion_properties, Vec3f linear_velocity_change) {
    const Vec3f velocity = vec3f::add(vec3f::load(motion_properties.linear_velocity), linear_velocity_change);
    vec3f::store(motion_properties.linear_velocity, apply_translation_dof_constraint(velocity, motion_properties.allowed_degrees_of_freedom));
}

void sub_linear_velocity_step(MotionProperties &motion_properties, Vec3f linear_velocity_change) {
    const Vec3f velocity = vec3f::subtract(vec3f::load(motion_properties.linear_velocity), linear_velocity_change);
    vec3f::store(motion_properties.linear_velocity, apply_translation_dof_constraint(velocity, motion_properties.allowed_degrees_of_freedom));
}

void add_angular_velocity_step(MotionProperties &motion_properties, Vec3f angular_velocity_change) {
    vec3f::store(motion_properties.angular_velocity, vec3f::add(vec3f::load(motion_properties.angular_velocity), angular_velocity_change));
}

void sub_angular_velocity_step(MotionProperties &motion_properties, Vec3f angular_velocity_change) {
    vec3f::store(motion_properties.angular_velocity,
                 vec3f::subtract(vec3f::load(motion_properties.angular_velocity), angular_velocity_change));
}

void scale_to_mass(MotionProperties &motion_properties, float new_mass) {
    // the body must have finite mass to scale, and the new mass must be positive
    TUMBLE_ASSERT(motion_properties.inv_mass > 0.0f);
    TUMBLE_ASSERT(new_mass > 0.0f);

    const float new_inv_mass = 1.0f / new_mass;
    // scale inverse inertia diagonal: I^-1 *= (new_inv_mass / old_inv_mass)
    vec3f::store(motion_properties.inv_inertia_diagonal,
                 vec3f::scale(vec3f::load(motion_properties.inv_inertia_diagonal), new_inv_mass / motion_properties.inv_mass));
    motion_properties.inv_mass = new_inv_mass;
}

/** the axis and angle (in [0, pi]) of a unit quaternion, x when it is near identity */
static float get_axis_angle_from_quat(Vec3f &out_axis, Quatf q) {
    const float qx = quatf::x(q), qy = quatf::y(q), qz = quatf::z(q), qw = quatf::w(q);
    // q and -q are the same rotation; the w >= 0 twin takes the short way round
    const float sign = qw < 0.0f ? -1.0f : 1.0f;
    const float angle = 2.0f * scalar::acos(scalar::min(1.0f, sign * qw));
    // for a unit quaternion q = [x, y, z, w], the axis is [x, y, z] / sin(angle / 2)
    const float sin_half_angle_sq = qx * qx + qy * qy + qz * qz;
    if (sin_half_angle_sq > 1e-10f) {
        const float sin_half_angle = sign * scalar::sqrt(sin_half_angle_sq);
        out_axis = vec3f::make(qx / sin_half_angle, qy / sin_half_angle, qz / sin_half_angle);
    } else {
        // quaternion is near identity, axis is arbitrary
        out_axis = vec3f::axis_x();
    }
    return angle;
}

void move_kinematic(MotionProperties &motion_properties, Vec3f delta_position, Quatf delta_rotation, float delta_time) {
    // delta time must be positive, otherwise no-op
    if (delta_time <= 0.0f) return;

    // calculate required linear velocity
    const uint32_t allowed_dofs = motion_properties.allowed_degrees_of_freedom;
    vec3f::store(motion_properties.linear_velocity,
                 apply_translation_dof_constraint(vec3f::scale(delta_position, 1.0f / delta_time), allowed_dofs));

    // calculate required angular velocity
    Vec3f axis;
    const float angle = get_axis_angle_from_quat(axis, delta_rotation);
    vec3f::store(motion_properties.angular_velocity, apply_rotation_dof_constraint(vec3f::scale(axis, angle / delta_time), allowed_dofs));
}

/** v with the lanes of the axes not in `axes` (bit 0 x, 1 y, 2 z) set to +0 */
static Vec3f keep_axes(Vec3f v, uint32_t axes) {
    if (axes == 0b111) return v;
    const int32_t x = (axes & 0b001) ? -1 : 0;
    const int32_t y = (axes & 0b010) ? -1 : 0;
    const int32_t z = (axes & 0b100) ? -1 : 0;
    return {v4f::bit_and(v.v, v4i::make(x, y, z, z))};
}

Vec3f apply_translation_dof_constraint(Vec3f velocity, uint32_t allowed_dofs) { return keep_axes(velocity, allowed_dofs & 0b111); }

Vec3f apply_rotation_dof_constraint(Vec3f angular_velocity, uint32_t allowed_dofs) {
    return keep_axes(angular_velocity, (allowed_dofs >> 3) & 0b111);
}

Vec3f multiply_world_space_inverse_inertia_by_vector(const MotionProperties &motion_properties, Quatf body_quaternion, Vec3f vector) {
    // mask out columns of DOFs that are not allowed
    const uint32_t allowed_rotation_axis = (motion_properties.allowed_degrees_of_freedom >> 3) & 0b111;
    const Vec3f v = keep_axes(vector, allowed_rotation_axis);

    // into inertia space, scale by the inverse diagonal, back to world space
    const Mat4f rotation = mat4f::from_quat(quatf::multiply(body_quaternion, quatf::load(motion_properties.inertia_rotation)));
    const Vec3f local = vec3f::multiply(mat4f::multiply3x3_transposed_vec(rotation, v), vec3f::load(motion_properties.inv_inertia_diagonal));

    // mask out rows of DOFs that are not allowed
    return keep_axes(mat4f::multiply3x3_vec(rotation, local), allowed_rotation_axis);
}

void apply_gyroscopic_force(MotionProperties &motion_properties, Quatf body_quaternion, float delta_time) {
    // calculate local space inertia tensor (a diagonal in local space); avoid dividing by zero, inertia in an axis
    // with no inverse inertia is zero
    const Vec3f inv_inertia = vec3f::load(motion_properties.inv_inertia_diagonal);
    const V128 reciprocal = v4f::div(v4f::splat(1.0f), inv_inertia.v);
    const Vec3f local_inertia = {v4f::select(v4f::zero(), reciprocal, v4f::eq(inv_inertia.v, v4f::zero()))};

    // calculate local space angular momentum
    const Quatf inertia_space_to_world_space = quatf::multiply(body_quaternion, quatf::load(motion_properties.inertia_rotation));
    const Vec3f local_angular_velocity =
        vec3f::transform_quat(vec3f::load(motion_properties.angular_velocity), quatf::conjugate(inertia_space_to_world_space));
    const Vec3f local_momentum = vec3f::multiply(local_inertia, local_angular_velocity);  // L = I w

    // the gyroscopic force applies a torque: T = -w x I w = -w x L
    // calculate the new angular momentum by applying the gyroscopic force
    Vec3f new_local_momentum = vec3f::scale_and_add(local_momentum, vec3f::cross(local_angular_velocity, local_momentum), -delta_time);

    // make sure the new magnitude is the same as the old one to avoid introducing energy into the system due to the euler step
    const float new_local_momentum_len_sq = vec3f::squared_length(new_local_momentum);
    if (new_local_momentum_len_sq > 0.0f) {
        const float old_local_momentum_len_sq = vec3f::squared_length(local_momentum);
        new_local_momentum = vec3f::scale(new_local_momentum, scalar::sqrt(old_local_momentum_len_sq / new_local_momentum_len_sq));
    } else {
        new_local_momentum = vec3f::zero();
    }

    // convert back to world space angular velocity: w = I^-1 L
    const Vec3f new_local_angular_velocity = vec3f::multiply(inv_inertia, new_local_momentum);
    vec3f::store(motion_properties.angular_velocity, vec3f::transform_quat(new_local_angular_velocity, inertia_space_to_world_space));
}

}  // namespace motion_properties

TUMBLE_NAMESPACE_END
