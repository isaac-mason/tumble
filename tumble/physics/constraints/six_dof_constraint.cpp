#include "tumble/physics/constraints/six_dof_constraint.h"

#include "tumble/math/eulerf.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/vec4f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/world.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace six_dof_constraint {

using constraint_id::ConstraintId;
using motor_settings::MotorState;
using rigid_body::RigidBody;

static bool is_body_live(const bodies::Bodies &bodies, uint32_t index) {
    return index < bodies.pool.length && bodies.pool[index] != nullptr && bodies.sequences[index] != bodies::SLOT_FREE;
}

static float component(Vec3f v, uint32_t i) { return i == 0 ? vec3f::x(v) : (i == 1 ? vec3f::y(v) : vec3f::z(v)); }

/** the twist of q about a unit axis: normalize(((q.xyz . axis) axis, q.w)) */
static Quatf get_twist_around_axis(Quatf q, Vec3f axis) {
    const float dot = vec3f::dot(quatf::xyz(q), axis);
    const float w = quatf::w(q);
    const float len_sq = dot * dot + w * w;
    if (len_sq != 0.0f) {
        const float inv_len = 1.0f / scalar::sqrt(len_sq);
        const Vec3f xyz = vec3f::scale(vec3f::scale(axis, dot), inv_len);
        return quatf::make(vec3f::x(xyz), vec3f::y(xyz), vec3f::z(xyz), w * inv_len);
    }
    return quatf::identity();
}

/** creates default six dof constraint */
static SixDOFConstraint make_six_dof_constraint() {
    SixDOFConstraint constraint;
    constraint.base = constraints::make_constraint_base();
    constraint.local_space_position1 = vec3f::zero();
    constraint.local_space_position2 = vec3f::zero();
    constraint.constraint_to_body1 = quatf::identity();
    constraint.constraint_to_body2 = quatf::identity();
    for (uint32_t i = 0; i < NUM; i++) {
        const bool rotation = i >= NUM_TRANSLATION;
        constraint.limit_min[i] = rotation ? -scalar::PI : -scalar::FLOAT_MAX;
        constraint.limit_max[i] = rotation ? scalar::PI : scalar::FLOAT_MAX;
        constraint.max_friction[i] = 0.0f;
        constraint.motor_state[i] = motor_settings::OFF;
        constraint.motor_settings[i] = motor_settings::create();
    }
    for (uint32_t i = 0; i < NUM_TRANSLATION; i++) constraint.limits_spring_settings[i] = spring_settings::create();
    constraint.target_velocity = vec3f::zero();
    constraint.target_angular_velocity = vec3f::zero();
    constraint.target_position = vec3f::zero();
    constraint.target_orientation = quatf::identity();
    constraint.swing_type = swing_twist_constraint_part::CONE;
    for (uint32_t i = 0; i < 3; i++) {
        constraint.translation_axis[i] = vec3f::zero();
        constraint.rotation_axis[i] = vec3f::zero();
        constraint.displacement[i] = 0.0f;
        constraint.translation_constraint_part[i] = axis_constraint_part::create();
        constraint.motor_translation_constraint_part[i] = axis_constraint_part::create();
        constraint.motor_rotation_constraint_part[i] = angle_constraint_part::create();
    }
    constraint.free_axis = 0;
    constraint.fixed_axis = 0;
    constraint.translation_motor_active = false;
    constraint.rotation_motor_active = false;
    constraint.rotation_position_motor_active = 0;
    constraint.has_spring_limits = false;
    constraint.point_constraint_part = point_constraint_part::create();
    constraint.swing_twist_constraint_part = swing_twist_constraint_part::create();
    constraint.rotation_constraint_part = rotation_euler_constraint_part::create();
    return constraint;
}

/** reset constraint state for pooling reuse */
static void reset_constraint(SixDOFConstraint &constraint) {
    constraint.base.enabled = true;
    constraint.base.sleeping = false;
    constraint.base.pooled = false;
    point_constraint_part::deactivate(constraint.point_constraint_part);
    rotation_euler_constraint_part::deactivate(constraint.rotation_constraint_part);
    swing_twist_constraint_part::deactivate(constraint.swing_twist_constraint_part);
    for (uint32_t i = 0; i < 3; i++) {
        axis_constraint_part::deactivate(constraint.translation_constraint_part[i]);
        axis_constraint_part::deactivate(constraint.motor_translation_constraint_part[i]);
        angle_constraint_part::deactivate(constraint.motor_rotation_constraint_part[i]);
    }
}

static bool is_fixed_axis(const SixDOFConstraint &constraint, uint32_t axis) { return (constraint.fixed_axis & (1u << axis)) != 0; }

static bool is_free_axis(const SixDOFConstraint &constraint, uint32_t axis) { return (constraint.free_axis & (1u << axis)) != 0; }

static bool is_translation_constrained(const SixDOFConstraint &constraint) { return (constraint.free_axis & 0b111) != 0b111; }

static bool is_translation_fully_constrained(const SixDOFConstraint &constraint) {
    return (constraint.fixed_axis & 0b111) == 0b111 && !constraint.has_spring_limits;
}

static bool is_rotation_constrained(const SixDOFConstraint &constraint) { return (constraint.free_axis & 0b111000) != 0b111000; }

static bool is_rotation_fully_constrained(const SixDOFConstraint &constraint) {
    return (constraint.fixed_axis & 0b111000) == 0b111000;
}

static bool has_friction(const SixDOFConstraint &constraint, uint32_t axis) {
    return !is_fixed_axis(constraint, axis) && constraint.max_friction[axis] > 0.0f;
}

static void update_translation_limits(SixDOFConstraint &constraint) {
    // inverted limits become fixed at 0
    for (uint32_t i = TRANSLATION_X; i <= TRANSLATION_Z; i++) {
        if (constraint.limit_min[i] > constraint.limit_max[i]) constraint.limit_min[i] = constraint.limit_max[i] = 0.0f;
    }
}

static void update_rotation_limits(SixDOFConstraint &constraint) {
    if (constraint.swing_type == swing_twist_constraint_part::CONE) {
        // a cone's swing limits are symmetric with a positive upper limit
        constraint.limit_max[ROTATION_Y] = scalar::max(0.0f, constraint.limit_max[ROTATION_Y]);
        constraint.limit_max[ROTATION_Z] = scalar::max(0.0f, constraint.limit_max[ROTATION_Z]);
        constraint.limit_min[ROTATION_Y] = -constraint.limit_max[ROTATION_Y];
        constraint.limit_min[ROTATION_Z] = -constraint.limit_max[ROTATION_Z];
    }
    for (uint32_t i = ROTATION_X; i <= ROTATION_Z; i++) {
        constraint.limit_min[i] = scalar::max(-scalar::PI, scalar::min(scalar::PI, constraint.limit_min[i]));
        constraint.limit_max[i] = scalar::max(-scalar::PI, scalar::min(scalar::PI, constraint.limit_max[i]));
        if (constraint.limit_min[i] > constraint.limit_max[i]) constraint.limit_min[i] = constraint.limit_max[i] = 0.0f;
    }
    swing_twist_constraint_part::set_limits(constraint.swing_twist_constraint_part, constraint.limit_min[ROTATION_X],
                                            constraint.limit_max[ROTATION_X], constraint.limit_min[ROTATION_Y],
                                            constraint.limit_max[ROTATION_Y], constraint.limit_min[ROTATION_Z],
                                            constraint.limit_max[ROTATION_Z]);
}

static void update_fixed_free_axis(SixDOFConstraint &constraint) {
    const uint32_t old_free_axis = constraint.free_axis;
    const uint32_t old_fixed_axis = constraint.fixed_axis;

    constraint.free_axis = 0;
    constraint.fixed_axis = 0;
    for (uint32_t a = 0; a < NUM; a++) {
        const float limit = a >= ROTATION_X ? scalar::PI : scalar::FLOAT_MAX;
        if (constraint.limit_min[a] >= constraint.limit_max[a]) {
            constraint.fixed_axis |= 1u << a;
        } else if (constraint.limit_min[a] <= -limit && constraint.limit_max[a] >= limit) {
            constraint.free_axis |= 1u << a;
        }
    }

    // on change, deactivate all parts to reset warm starting
    if (old_free_axis != constraint.free_axis || old_fixed_axis != constraint.fixed_axis) {
        for (uint32_t i = 0; i < 3; i++) axis_constraint_part::deactivate(constraint.translation_constraint_part[i]);
        point_constraint_part::deactivate(constraint.point_constraint_part);
        swing_twist_constraint_part::deactivate(constraint.swing_twist_constraint_part);
        rotation_euler_constraint_part::deactivate(constraint.rotation_constraint_part);
        for (uint32_t i = 0; i < 3; i++) axis_constraint_part::deactivate(constraint.motor_translation_constraint_part[i]);
        for (uint32_t i = 0; i < 3; i++) angle_constraint_part::deactivate(constraint.motor_rotation_constraint_part[i]);
    }
}

static void cache_translation_motor_active(SixDOFConstraint &constraint) {
    constraint.translation_motor_active =
        constraint.motor_state[TRANSLATION_X] != motor_settings::OFF || constraint.motor_state[TRANSLATION_Y] != motor_settings::OFF ||
        constraint.motor_state[TRANSLATION_Z] != motor_settings::OFF || has_friction(constraint, TRANSLATION_X) ||
        has_friction(constraint, TRANSLATION_Y) || has_friction(constraint, TRANSLATION_Z);
}

static void cache_rotation_motor_active(SixDOFConstraint &constraint) {
    constraint.rotation_motor_active =
        constraint.motor_state[ROTATION_X] != motor_settings::OFF || constraint.motor_state[ROTATION_Y] != motor_settings::OFF ||
        constraint.motor_state[ROTATION_Z] != motor_settings::OFF || has_friction(constraint, ROTATION_X) ||
        has_friction(constraint, ROTATION_Y) || has_friction(constraint, ROTATION_Z);
}

static void cache_rotation_position_motor_active(SixDOFConstraint &constraint) {
    constraint.rotation_position_motor_active = 0;
    for (uint32_t i = 0; i < 3; i++) {
        if (motor_settings::is_position_motor(constraint.motor_state[ROTATION_X + i])) {
            constraint.rotation_position_motor_active |= 1u << i;
        }
    }
}

static void cache_has_spring_limits(SixDOFConstraint &constraint) {
    constraint.has_spring_limits = constraint.limits_spring_settings[0].frequency_or_stiffness > 0.0f ||
                                   constraint.limits_spring_settings[1].frequency_or_stiffness > 0.0f ||
                                   constraint.limits_spring_settings[2].frequency_or_stiffness > 0.0f;
}

/** the rotation from constraint space (x, y, x cross y) to body space */
static Quatf constraint_to_body(Vec3f axis_x, Vec3f axis_y) {
    const Vec3f axis_z = vec3f::cross(axis_x, axis_y);
    return quatf::from_mat4(mat4f::make_columns({axis_x.v}, {axis_y.v}, {axis_z.v}, vec4f::make(0.0f, 0.0f, 0.0f, 1.0f)));
}

ConstraintId create(world::World &world, RigidBody *body_a, RigidBody *body_b,
                    const SixDOFConstraintSettings &settings) {
    constraints::Constraints &constraints = world.constraints;
    bodies::Bodies &bodies = world.bodies;
    TUMBLE_ASSERT(is_body_live(bodies, body_a->index) && bodies.pool[body_a->index] == body_a);
    TUMBLE_ASSERT(is_body_live(bodies, body_b->index) && bodies.pool[body_b->index] == body_b);
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::SIX_DOF);

    const uint32_t index = constraints::allocate_index(pool);
    if (index == ~0u) return constraint_id::INVALID_CONSTRAINT_ID;
    const uint32_t sequence = constraints::next_sequence(pool, index);
    SixDOFConstraint &constraint = *reinterpret_cast<SixDOFConstraint *>(constraints::base_at(pool, index));
    constraint = make_six_dof_constraint();
    reset_constraint(constraint);

    constraint.base.id = constraint_id::ser(index, constraint_id::SIX_DOF, sequence);
    constraint.base.index = index;
    constraint.base.sequence = sequence;

    constraint.base.constraint_priority = settings.constraint_priority;
    constraint.base.num_velocity_steps_override = settings.num_velocity_steps_override;
    constraint.base.num_position_steps_override = settings.num_position_steps_override;

    constraint.base.body_index_a = body_a->index;
    constraint.base.body_index_b = body_b->index;

    constraint.swing_type = settings.swing_type;
    constraint.swing_twist_constraint_part.swing_type = settings.swing_type;

    // constraint to body rotations from the normalized axes
    const Vec3f axis_x1 = vec3f::normalize(vec3f::load(settings.axis_x_a));
    const Vec3f axis_y1 = vec3f::normalize(vec3f::load(settings.axis_y_a));
    const Vec3f axis_x2 = vec3f::normalize(vec3f::load(settings.axis_x_b));
    const Vec3f axis_y2 = vec3f::normalize(vec3f::load(settings.axis_y_b));
    constraint.constraint_to_body1 = constraint_to_body(axis_x1, axis_y1);
    constraint.constraint_to_body2 = constraint_to_body(axis_x2, axis_y2);

    if (settings.space == constraints::SPACE_WORLD) {
        const Quatf inv_quat_a = quatf::conjugate(quatf::load(body_a->quaternion));
        const Quatf inv_quat_b = quatf::conjugate(quatf::load(body_b->quaternion));

        constraint.local_space_position1 = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point_a), vec3f::load(body_a->center_of_mass_position)), inv_quat_a);

        constraint.local_space_position2 = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point_b), vec3f::load(body_b->center_of_mass_position)), inv_quat_b);

        constraint.constraint_to_body1 = quatf::multiply(inv_quat_a, constraint.constraint_to_body1);
        constraint.constraint_to_body2 = quatf::multiply(inv_quat_b, constraint.constraint_to_body2);
    } else {
        constraint.local_space_position1 = vec3f::load(settings.point_a);
        constraint.local_space_position2 = vec3f::load(settings.point_b);
    }

    for (uint32_t i = 0; i < NUM; i++) {
        constraint.limit_min[i] = settings.limit_min[i];
        constraint.limit_max[i] = settings.limit_max[i];
        constraint.max_friction[i] = settings.max_friction[i];
        constraint.motor_settings[i] = settings.motor_settings[i];
    }
    for (uint32_t i = 0; i < NUM_TRANSLATION; i++) constraint.limits_spring_settings[i] = settings.limits_spring_settings[i];

    update_translation_limits(constraint);
    update_rotation_limits(constraint);
    update_fixed_free_axis(constraint);
    cache_has_spring_limits(constraint);
    cache_translation_motor_active(constraint);
    cache_rotation_motor_active(constraint);
    cache_rotation_position_motor_active(constraint);

    // track constraint on both bodies
    const ConstraintId id = constraint.base.id;
    bool linked = body_a->constraint_ids.push(id);
    if (linked && body_a != body_b) linked = body_b->constraint_ids.push(id);
    if (linked) linked = constraints::add_to_list(constraints, constraint.base);
    if (!linked) {
        constraints::remove_constraint_by_id(constraints, bodies, id);
        return constraint_id::INVALID_CONSTRAINT_ID;
    }

    return id;
}

bool remove(world::World &world, ConstraintId id) {
    SixDOFConstraint *found = get(world, id);
    if (found == nullptr) return false;
    SixDOFConstraint &constraint = *found;
    constraints::Constraints &constraints = world.constraints;
    bodies::Bodies &bodies = world.bodies;
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::SIX_DOF);

    // remove from bodies' constraint_ids
    const uint32_t a = constraint.base.body_index_a;
    const uint32_t b = constraint.base.body_index_b;
    if (is_body_live(bodies, a)) constraints::remove_constraint_id_from_body(*bodies.pool[a], constraint.base.id);
    if (a != b && is_body_live(bodies, b)) constraints::remove_constraint_id_from_body(*bodies.pool[b], constraint.base.id);

    reset_constraint(constraint);

    constraint.base.pooled = true;
    constraint.base.id = constraint_id::INVALID_CONSTRAINT_ID;
    constraints::remove_from_list(constraints, constraint.base);
    pool.free_indices.push(constraint.base.index);
    return true;
}

SixDOFConstraint *get(world::World &world, ConstraintId id) {
    if (constraint_id::type(id) != constraint_id::SIX_DOF) return nullptr;
    return reinterpret_cast<SixDOFConstraint *>(constraints::get(world.constraints, id));
}

/** the body's inverse mass and world space inverse inertia as the axis part takes them: zero unless dynamic */
static void get_inverse_mass_and_inertia(float &out_inv_mass, Mat4f &out_inv_inertia, RigidBody &body, uint32_t step_stamp) {
    if (body.motion_type == motion_type::MOTION_DYNAMIC) {
        out_inv_mass = body.motion_properties.inv_mass;
        out_inv_inertia = motion_properties::get_world_inverse_inertia(body.motion_properties, quatf::load(body.quaternion), step_stamp);
    } else {
        out_inv_mass = 0.0f;
        out_inv_inertia = mat4f::zero();
    }
}

/** r1 + u = p2 - x1, r2 = p2 - x2 and u = p2 - p1, from the bodies' current transforms */
static void get_position_constraint_properties(Vec3f &out_r1_plus_u, Vec3f &out_r2, Vec3f &out_u, const SixDOFConstraint &constraint,
                                               const RigidBody &body_a, const RigidBody &body_b) {
    const Vec3f center_of_mass_a = vec3f::load(body_a.center_of_mass_position);
    const Vec3f center_of_mass_b = vec3f::load(body_b.center_of_mass_position);
    const Vec3f p1 = vec3f::add(vec3f::transform_quat(constraint.local_space_position1, quatf::load(body_a.quaternion)), center_of_mass_a);
    const Vec3f p2 = vec3f::add(vec3f::transform_quat(constraint.local_space_position2, quatf::load(body_b.quaternion)), center_of_mass_b);

    out_r1_plus_u = vec3f::subtract(p2, center_of_mass_a);
    out_r2 = vec3f::subtract(p2, center_of_mass_b);
    out_u = vec3f::subtract(p2, p1);
}

void setup_velocity(SixDOFConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    // rotates from body 1's constraint space to world space
    const Quatf constraint_body1_to_world = quatf::multiply(quatf::load(body_a.quaternion), constraint.constraint_to_body1);

    // world space axes of constraint space
    const Mat4f translation_axes = mat4f::from_quat(constraint_body1_to_world);
    for (int i = 0; i < 3; i++) constraint.translation_axis[i] = mat4f::axis(translation_axes, i);

    if (is_translation_fully_constrained(constraint)) {
        // all translation locked: the point part
        point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                               constraint.local_space_position1, body_b, rotation_b,
                                                               constraint.local_space_position2, bodies.step_stamp);
    } else if (is_translation_constrained(constraint) || constraint.translation_motor_active) {
        Vec3f r1_plus_u;
        Vec3f r2;
        Vec3f u;
        get_position_constraint_properties(r1_plus_u, r2, u, constraint, body_a, body_b);

        float inv_mass_a;
        float inv_mass_b;
        Mat4f inv_inertia_a;
        Mat4f inv_inertia_b;
        get_inverse_mass_and_inertia(inv_mass_a, inv_inertia_a, body_a, bodies.step_stamp);
        get_inverse_mass_and_inertia(inv_mass_b, inv_inertia_b, body_b, bodies.step_stamp);

        for (uint32_t i = 0; i < 3; i++) {
            const uint32_t axis = TRANSLATION_X + i;
            const Vec3f translation_axis = constraint.translation_axis[i];

            const float d = vec3f::dot(translation_axis, u);
            constraint.displacement[i] = d;

            // a fixed axis is always active, a limited one only outside its range
            bool constraint_active = false;
            float constraint_value = 0.0f;
            if (is_fixed_axis(constraint, axis)) {
                constraint_value = d - constraint.limit_min[i];
                constraint_active = true;
            } else if (!is_free_axis(constraint, axis)) {
                if (d <= constraint.limit_min[i]) {
                    constraint_value = d - constraint.limit_min[i];
                    constraint_active = true;
                } else if (d >= constraint.limit_max[i]) {
                    constraint_value = d - constraint.limit_max[i];
                    constraint_active = true;
                }
            }

            if (constraint_active) {
                axis_constraint_part::calculate_constraint_properties_with_settings(
                    constraint.translation_constraint_part[i], delta_time, body_a, body_b, inv_mass_a, inv_mass_b, inv_inertia_a,
                    inv_inertia_b, r1_plus_u, r2, translation_axis, 0.0f, constraint_value, constraint.limits_spring_settings[i]);
            } else {
                axis_constraint_part::deactivate(constraint.translation_constraint_part[i]);
            }

            switch (constraint.motor_state[i]) {
                case motor_settings::OFF:
                    if (has_friction(constraint, axis)) {
                        axis_constraint_part::calculate_constraint_properties(constraint.motor_translation_constraint_part[i],
                                                                              body_a, body_b, inv_mass_a, inv_mass_b, inv_inertia_a,
                                                                              inv_inertia_b, r1_plus_u, r2, translation_axis, 0.0f);
                    } else {
                        axis_constraint_part::deactivate(constraint.motor_translation_constraint_part[i]);
                    }
                    break;
                case motor_settings::VELOCITY:
                    axis_constraint_part::calculate_constraint_properties(
                        constraint.motor_translation_constraint_part[i], body_a, body_b, inv_mass_a, inv_mass_b, inv_inertia_a,
                        inv_inertia_b, r1_plus_u, r2, translation_axis, -component(constraint.target_velocity, i));
                    break;
                case motor_settings::POSITION: {
                    const spring_settings::SpringSettings &motor_spring = constraint.motor_settings[i].spring;
                    if (spring_settings::has_stiffness(motor_spring)) {
                        axis_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                            constraint.motor_translation_constraint_part[i], delta_time, body_a, body_b, inv_mass_a, inv_mass_b,
                            inv_inertia_a, inv_inertia_b, r1_plus_u, r2, translation_axis, 0.0f,
                            d - component(constraint.target_position, i), motor_spring);
                    } else {
                        axis_constraint_part::deactivate(constraint.motor_translation_constraint_part[i]);
                    }
                    break;
                }
                case motor_settings::POSITION_AND_VELOCITY: {
                    const spring_settings::SpringSettings &motor_spring = constraint.motor_settings[i].spring;
                    if (spring_settings::has_stiffness_or_damping(motor_spring)) {
                        axis_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                            constraint.motor_translation_constraint_part[i], delta_time, body_a, body_b, inv_mass_a, inv_mass_b,
                            inv_inertia_a, inv_inertia_b, r1_plus_u, r2, translation_axis, -component(constraint.target_velocity, i),
                            d - component(constraint.target_position, i), motor_spring);
                    } else {
                        axis_constraint_part::deactivate(constraint.motor_translation_constraint_part[i]);
                    }
                    break;
                }
            }
        }
    }

    // rotation constraints
    const Quatf constraint_body2_to_world = quatf::multiply(quatf::load(body_b.quaternion), constraint.constraint_to_body2);
    const Quatf q = quatf::multiply(quatf::conjugate(constraint_body1_to_world), constraint_body2_to_world);

    if (is_rotation_fully_constrained(constraint)) {
        // all rotation locked: the rotation euler part
        rotation_euler_constraint_part::calculate_constraint_properties(constraint.rotation_constraint_part, body_a, body_b,
                                                                        bodies.step_stamp);
    } else if (is_rotation_constrained(constraint) || constraint.rotation_motor_active) {
        if (is_rotation_constrained(constraint)) {
            swing_twist_constraint_part::calculate_constraint_properties(constraint.swing_twist_constraint_part, body_a, body_b, q,
                                                                         constraint_body1_to_world);
        } else {
            swing_twist_constraint_part::deactivate(constraint.swing_twist_constraint_part);
        }

        if (constraint.rotation_motor_active) {
            // rotation motor axes
            const Mat4f rotation_axes = mat4f::from_quat(constraint_body2_to_world);
            for (int i = 0; i < 3; i++) constraint.rotation_axis[i] = mat4f::axis(rotation_axes, i);

            // the target orientation along the shortest path from q
            Quatf target_orientation = constraint.target_orientation;
            if (quatf::dot(q, target_orientation) < 0.0f) target_orientation = quatf::scale(target_orientation, -1.0f);

            // target_orientation = q * diff, so diff = q^-1 * target_orientation
            const Quatf diff = quatf::multiply(quatf::conjugate(q), target_orientation);

            // keep only the rotation about the axes with a position motor
            Quatf projected_diff;
            switch (constraint.rotation_position_motor_active) {
                case 0b001:
                    projected_diff = get_twist_around_axis(diff, vec3f::axis_x());
                    break;
                case 0b010:
                    projected_diff = get_twist_around_axis(diff, vec3f::axis_y());
                    break;
                case 0b100:
                    projected_diff = get_twist_around_axis(diff, vec3f::axis_z());
                    break;
                case 0b011:
                    // q = swing_xy * twist_z, so swing_xy = q * twist_z^-1
                    projected_diff = quatf::multiply(diff, quatf::conjugate(get_twist_around_axis(diff, vec3f::axis_z())));
                    break;
                case 0b101:
                    // swing_xz = q * twist_y^-1
                    projected_diff = quatf::multiply(diff, quatf::conjugate(get_twist_around_axis(diff, vec3f::axis_y())));
                    break;
                case 0b110:
                    // swing_yz = q * twist_x^-1
                    projected_diff = quatf::multiply(diff, quatf::conjugate(get_twist_around_axis(diff, vec3f::axis_x())));
                    break;
                default:
                    // all three (or none, unused): the whole rotation
                    projected_diff = diff;
                    break;
            }

            // the imaginary part is axis * sin(angle / 2), so for small angles angle ~ 2 * axis
            const Vec3f rotation_error = vec3f::scale(quatf::xyz(projected_diff), -2.0f);

            for (uint32_t i = 0; i < 3; i++) {
                const uint32_t axis = ROTATION_X + i;
                const Vec3f rotation_axis = constraint.rotation_axis[i];

                switch (constraint.motor_state[axis]) {
                    case motor_settings::OFF:
                        if (has_friction(constraint, axis)) {
                            angle_constraint_part::calculate_constraint_properties(constraint.motor_rotation_constraint_part[i],
                                                                                   body_a, body_b, rotation_axis, 0.0f);
                        } else {
                            angle_constraint_part::deactivate(constraint.motor_rotation_constraint_part[i]);
                        }
                        break;
                    case motor_settings::VELOCITY:
                        angle_constraint_part::calculate_constraint_properties(constraint.motor_rotation_constraint_part[i], body_a,
                                                                               body_b, rotation_axis,
                                                                               -component(constraint.target_angular_velocity, i));
                        break;
                    case motor_settings::POSITION: {
                        const spring_settings::SpringSettings &motor_spring = constraint.motor_settings[axis].spring;
                        if (spring_settings::has_stiffness(motor_spring)) {
                            angle_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                                constraint.motor_rotation_constraint_part[i], delta_time, body_a, body_b, rotation_axis, 0.0f,
                                component(rotation_error, i), motor_spring);
                        } else {
                            angle_constraint_part::deactivate(constraint.motor_rotation_constraint_part[i]);
                        }
                        break;
                    }
                    case motor_settings::POSITION_AND_VELOCITY: {
                        const spring_settings::SpringSettings &motor_spring = constraint.motor_settings[axis].spring;
                        if (spring_settings::has_stiffness_or_damping(motor_spring)) {
                            angle_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                                constraint.motor_rotation_constraint_part[i], delta_time, body_a, body_b, rotation_axis,
                                -component(constraint.target_angular_velocity, i), component(rotation_error, i), motor_spring);
                        } else {
                            angle_constraint_part::deactivate(constraint.motor_rotation_constraint_part[i]);
                        }
                        break;
                    }
                }
            }
        }
    }
}

void warm_start_velocity(SixDOFConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const float inv_mass_a = body_a.motion_type == motion_type::MOTION_DYNAMIC ? body_a.motion_properties.inv_mass : 0.0f;
    const float inv_mass_b = body_b.motion_type == motion_type::MOTION_DYNAMIC ? body_b.motion_properties.inv_mass : 0.0f;

    if (constraint.translation_motor_active) {
        for (uint32_t i = 0; i < 3; i++) {
            if (axis_constraint_part::is_active(constraint.motor_translation_constraint_part[i])) {
                axis_constraint_part::warm_start(constraint.motor_translation_constraint_part[i], body_a, body_b, inv_mass_a,
                                                 inv_mass_b, constraint.translation_axis[i], warm_start_impulse_ratio);
            }
        }
    }

    if (constraint.rotation_motor_active) {
        for (uint32_t i = 0; i < 3; i++) {
            if (angle_constraint_part::is_active(constraint.motor_rotation_constraint_part[i])) {
                angle_constraint_part::warm_start(constraint.motor_rotation_constraint_part[i], body_a, body_b,
                                                  warm_start_impulse_ratio);
            }
        }
    }

    if (is_rotation_fully_constrained(constraint)) {
        rotation_euler_constraint_part::warm_start(constraint.rotation_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    } else if (is_rotation_constrained(constraint)) {
        swing_twist_constraint_part::warm_start(constraint.swing_twist_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    }

    if (is_translation_fully_constrained(constraint)) {
        point_constraint_part::warm_start(constraint.point_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    } else if (is_translation_constrained(constraint)) {
        for (uint32_t i = 0; i < 3; i++) {
            if (axis_constraint_part::is_active(constraint.translation_constraint_part[i])) {
                axis_constraint_part::warm_start(constraint.translation_constraint_part[i], body_a, body_b, inv_mass_a, inv_mass_b,
                                                 constraint.translation_axis[i], warm_start_impulse_ratio);
            }
        }
    }
}

void notify_shape_changed(SixDOFConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com) {
    // separate tests, not an else-if: a constraint between a body and itself moves both points
    const uint32_t body_index = body_id::index(body_id);
    if (constraint.base.body_index_a == body_index)
        constraint.local_space_position1 = vec3f::subtract(constraint.local_space_position1, delta_com);
    if (constraint.base.body_index_b == body_index)
        constraint.local_space_position2 = vec3f::subtract(constraint.local_space_position2, delta_com);
}

void reset_warm_start(SixDOFConstraint &constraint) {
    for (uint32_t i = 0; i < 3; i++) axis_constraint_part::deactivate(constraint.motor_translation_constraint_part[i]);
    for (uint32_t i = 0; i < 3; i++) angle_constraint_part::deactivate(constraint.motor_rotation_constraint_part[i]);
    rotation_euler_constraint_part::deactivate(constraint.rotation_constraint_part);
    swing_twist_constraint_part::deactivate(constraint.swing_twist_constraint_part);
    point_constraint_part::deactivate(constraint.point_constraint_part);
    for (uint32_t i = 0; i < 3; i++) axis_constraint_part::deactivate(constraint.translation_constraint_part[i]);
}

bool solve_velocity(SixDOFConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    bool impulse = false;

    if (constraint.translation_motor_active) {
        for (uint32_t i = 0; i < 3; i++) {
            if (!axis_constraint_part::is_active(constraint.motor_translation_constraint_part[i])) continue;
            switch (constraint.motor_state[i]) {
                case motor_settings::OFF: {
                    // friction only
                    const float max_lambda = constraint.max_friction[i] * delta_time;
                    impulse |= axis_constraint_part::solve_velocity_constraint(constraint.motor_translation_constraint_part[i],
                                                                               body_a, body_b, constraint.translation_axis[i],
                                                                               -max_lambda, max_lambda);
                    break;
                }
                case motor_settings::VELOCITY:
                case motor_settings::POSITION:
                case motor_settings::POSITION_AND_VELOCITY:
                    impulse |= axis_constraint_part::solve_velocity_constraint(
                        constraint.motor_translation_constraint_part[i], body_a, body_b, constraint.translation_axis[i],
                        delta_time * constraint.motor_settings[i].min_force_limit,
                        delta_time * constraint.motor_settings[i].max_force_limit);
                    break;
            }
        }
    }

    if (constraint.rotation_motor_active) {
        for (uint32_t i = 0; i < 3; i++) {
            const uint32_t axis = ROTATION_X + i;
            if (!angle_constraint_part::is_active(constraint.motor_rotation_constraint_part[i])) continue;
            switch (constraint.motor_state[axis]) {
                case motor_settings::OFF: {
                    // friction only
                    const float max_lambda = constraint.max_friction[axis] * delta_time;
                    impulse |= angle_constraint_part::solve_velocity_constraint(constraint.motor_rotation_constraint_part[i],
                                                                                body_a, body_b, constraint.rotation_axis[i],
                                                                                -max_lambda, max_lambda);
                    break;
                }
                case motor_settings::VELOCITY:
                case motor_settings::POSITION:
                case motor_settings::POSITION_AND_VELOCITY:
                    impulse |= angle_constraint_part::solve_velocity_constraint(
                        constraint.motor_rotation_constraint_part[i], body_a, body_b, constraint.rotation_axis[i],
                        delta_time * constraint.motor_settings[axis].min_torque_limit,
                        delta_time * constraint.motor_settings[axis].max_torque_limit);
                    break;
            }
        }
    }

    if (is_rotation_fully_constrained(constraint)) {
        impulse |= rotation_euler_constraint_part::solve_velocity_constraint(constraint.rotation_constraint_part, body_a, body_b);
    } else if (is_rotation_constrained(constraint)) {
        impulse |= swing_twist_constraint_part::solve_velocity_constraint(constraint.swing_twist_constraint_part, body_a, body_b);
    }

    if (is_translation_fully_constrained(constraint)) {
        impulse |= point_constraint_part::solve_velocity_constraint(constraint.point_constraint_part, body_a, body_b);
    } else if (is_translation_constrained(constraint)) {
        for (uint32_t i = 0; i < 3; i++) {
            if (!axis_constraint_part::is_active(constraint.translation_constraint_part[i])) continue;
            // a limited axis pushes from the side it is limited on only
            float limit_min = -scalar::FLOAT_MAX;
            float limit_max = scalar::FLOAT_MAX;
            if (!is_fixed_axis(constraint, i)) {
                if (constraint.displacement[i] <= constraint.limit_min[i]) {
                    limit_min = 0.0f;
                } else if (constraint.displacement[i] >= constraint.limit_max[i]) {
                    limit_max = 0.0f;
                }
            }
            impulse |= axis_constraint_part::solve_velocity_constraint(constraint.translation_constraint_part[i], body_a, body_b,
                                                                       constraint.translation_axis[i], limit_min, limit_max);
        }
    }

    return impulse;
}

bool solve_position(SixDOFConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte) {
    (void)delta_time;
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    bool impulse = false;

    if (is_rotation_fully_constrained(constraint)) {
        // inverse of the initial rotation from body 1 to body 2 in body 1 space, with the fixed angles applied:
        // r0^-1 = c2 * c1^-1
        Eulerf rotation_limits_min_euler;
        eulerf::set(rotation_limits_min_euler, constraint.limit_min[ROTATION_X], constraint.limit_min[ROTATION_Y],
                    constraint.limit_min[ROTATION_Z], EULER_ZYX);
        const Quatf constraint_to_body1 = quatf::multiply(constraint.constraint_to_body1, quatf::from_euler(rotation_limits_min_euler));
        const Quatf inv_initial_orientation = quatf::multiply(constraint.constraint_to_body2, quatf::conjugate(constraint_to_body1));

        rotation_euler_constraint_part::calculate_constraint_properties(constraint.rotation_constraint_part, body_a, body_b,
                                                                        motion_properties::STEP_STAMP_NONE);
        impulse = rotation_euler_constraint_part::solve_position_constraint(constraint.rotation_constraint_part, body_a, body_b,
                                                                            inv_initial_orientation, baumgarte) ||
                  impulse;
    } else if (is_rotation_constrained(constraint)) {
        const Quatf constraint_body1_to_world = quatf::multiply(quatf::load(body_a.quaternion), constraint.constraint_to_body1);
        const Quatf constraint_body2_to_world = quatf::multiply(quatf::load(body_b.quaternion), constraint.constraint_to_body2);
        const Quatf q = quatf::multiply(quatf::conjugate(constraint_body1_to_world), constraint_body2_to_world);

        impulse = swing_twist_constraint_part::solve_position_constraint(constraint.swing_twist_constraint_part, body_a, body_b, q,
                                                                         constraint.constraint_to_body1,
                                                                         constraint.constraint_to_body2, baumgarte) ||
                  impulse;
    }

    // the translation solve takes the rotations as the rotation solve left them
    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    if (is_translation_fully_constrained(constraint)) {
        // the point part, at the fixed offsets
        const Vec3f translation_limits_min =
            vec3f::make(constraint.limit_min[TRANSLATION_X], constraint.limit_min[TRANSLATION_Y], constraint.limit_min[TRANSLATION_Z]);
        const Vec3f local_space_position1 =
            vec3f::add(constraint.local_space_position1, vec3f::transform_quat(translation_limits_min, constraint.constraint_to_body1));

        point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                               local_space_position1, body_b, rotation_b,
                                                               constraint.local_space_position2, motion_properties::STEP_STAMP_NONE);
        impulse = point_constraint_part::solve_position_constraint(constraint.point_constraint_part, body_a, body_b, baumgarte) ||
                  impulse;
    } else if (is_translation_constrained(constraint)) {
        // per axis, hard limits only
        for (uint32_t i = 0; i < 3; i++) {
            if (constraint.limits_spring_settings[i].frequency_or_stiffness > 0.0f) continue;

            // the bodies may have moved
            Vec3f r1_plus_u;
            Vec3f r2;
            Vec3f u;
            get_position_constraint_properties(r1_plus_u, r2, u, constraint, body_a, body_b);
            const Quatf constraint_body1_to_world = quatf::multiply(quatf::load(body_a.quaternion), constraint.constraint_to_body1);

            const Vec3f unit_axis = vec3f::make(i == 0 ? 1.0f : 0.0f, i == 1 ? 1.0f : 0.0f, i == 2 ? 1.0f : 0.0f);
            const Vec3f translation_axis = vec3f::transform_quat(unit_axis, constraint_body1_to_world);

            float error = 0.0f;
            const uint32_t axis = TRANSLATION_X + i;
            if (is_fixed_axis(constraint, axis)) {
                error = vec3f::dot(u, translation_axis) - constraint.limit_min[axis];
            } else if (!is_free_axis(constraint, axis)) {
                const float displacement = vec3f::dot(u, translation_axis);
                if (displacement <= constraint.limit_min[axis]) {
                    error = displacement - constraint.limit_min[axis];
                } else if (displacement >= constraint.limit_max[axis]) {
                    error = displacement - constraint.limit_max[axis];
                }
            }

            if (error != 0.0f) {
                float inv_mass_a;
                float inv_mass_b;
                Mat4f inv_inertia_a;
                Mat4f inv_inertia_b;
                get_inverse_mass_and_inertia(inv_mass_a, inv_inertia_a, body_a, motion_properties::STEP_STAMP_NONE);
                get_inverse_mass_and_inertia(inv_mass_b, inv_inertia_b, body_b, motion_properties::STEP_STAMP_NONE);

                axis_constraint_part::calculate_constraint_properties(constraint.translation_constraint_part[i], body_a, body_b,
                                                                      inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b,
                                                                      r1_plus_u, r2, translation_axis, 0.0f);
                impulse = axis_constraint_part::solve_position_constraint(constraint.translation_constraint_part[i], body_a,
                                                                          body_b, translation_axis, error, baumgarte) ||
                          impulse;
            }
        }
    }

    return impulse;
}

void set_translation_limits(SixDOFConstraint &constraint, Vec3f limit_min, Vec3f limit_max) {
    constraint.limit_min[TRANSLATION_X] = vec3f::x(limit_min);
    constraint.limit_min[TRANSLATION_Y] = vec3f::y(limit_min);
    constraint.limit_min[TRANSLATION_Z] = vec3f::z(limit_min);
    constraint.limit_max[TRANSLATION_X] = vec3f::x(limit_max);
    constraint.limit_max[TRANSLATION_Y] = vec3f::y(limit_max);
    constraint.limit_max[TRANSLATION_Z] = vec3f::z(limit_max);
    update_translation_limits(constraint);
    update_fixed_free_axis(constraint);
}

void set_rotation_limits(SixDOFConstraint &constraint, Vec3f limit_min, Vec3f limit_max) {
    constraint.limit_min[ROTATION_X] = vec3f::x(limit_min);
    constraint.limit_min[ROTATION_Y] = vec3f::y(limit_min);
    constraint.limit_min[ROTATION_Z] = vec3f::z(limit_min);
    constraint.limit_max[ROTATION_X] = vec3f::x(limit_max);
    constraint.limit_max[ROTATION_Y] = vec3f::y(limit_max);
    constraint.limit_max[ROTATION_Z] = vec3f::z(limit_max);
    update_rotation_limits(constraint);
    update_fixed_free_axis(constraint);
}

void set_motor_state(SixDOFConstraint &constraint, SixDOFAxis axis, MotorState state) {
    TUMBLE_ASSERT(state == motor_settings::OFF || motor_settings::is_valid(constraint.motor_settings[axis]));
    if (constraint.motor_state[axis] == state) return;
    constraint.motor_state[axis] = state;
    // the motor parts are repurposed between modes: no warm start into the new one
    if (axis <= TRANSLATION_Z) {
        axis_constraint_part::deactivate(constraint.motor_translation_constraint_part[axis]);
        cache_translation_motor_active(constraint);
    } else {
        angle_constraint_part::deactivate(constraint.motor_rotation_constraint_part[axis - ROTATION_X]);
        cache_rotation_motor_active(constraint);
        cache_rotation_position_motor_active(constraint);
    }
}

void set_target_velocity_cs(SixDOFConstraint &constraint, Vec3f velocity) {
    constraint.target_velocity = velocity;
}

void set_target_angular_velocity_cs(SixDOFConstraint &constraint, Vec3f angular_velocity) {
    constraint.target_angular_velocity = angular_velocity;
}

void set_target_position_cs(SixDOFConstraint &constraint, Vec3f position) {
    constraint.target_position = position;
}

void set_target_orientation_cs(SixDOFConstraint &constraint, Quatf orientation) {
    // clamp to the limits
    Quatf q_swing;
    Quatf q_twist;
    swing_twist_constraint_part::get_swing_twist(q_swing, q_twist, orientation);
    const uint32_t clamped_axis = swing_twist_constraint_part::clamp_swing_twist(constraint.swing_twist_constraint_part, q_swing, q_twist);
    constraint.target_orientation = clamped_axis != 0 ? quatf::multiply(q_swing, q_twist) : orientation;
}

void set_max_friction(SixDOFConstraint &constraint, uint32_t axis, float friction) {
    constraint.max_friction[axis] = friction;
    if (axis < NUM_TRANSLATION) {
        cache_translation_motor_active(constraint);
    } else {
        cache_rotation_motor_active(constraint);
    }
}

void make_free_axis(SixDOFConstraint &constraint, uint32_t axis) { set_limited_axis(constraint, axis, -scalar::FLOAT_MAX, scalar::FLOAT_MAX); }

void make_fixed_axis(SixDOFConstraint &constraint, uint32_t axis) { set_limited_axis(constraint, axis, scalar::FLOAT_MAX, -scalar::FLOAT_MAX); }

void set_limited_axis(SixDOFConstraint &constraint, uint32_t axis, float min, float max) {
    constraint.limit_min[axis] = min;
    constraint.limit_max[axis] = max;
    if (axis < NUM_TRANSLATION) {
        update_translation_limits(constraint);
    } else {
        update_rotation_limits(constraint);
    }
    update_fixed_free_axis(constraint);
}

void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const SixDOFConstraint &constraint) {
    out.velocity = constraint.base.num_velocity_steps_override;
    out.position = constraint.base.num_position_steps_override;
}

void get_sort_fields(constraints::ConstraintSortFields &out, const SixDOFConstraint &constraint) {
    out.priority = constraint.base.constraint_priority;
    out.index = constraint.base.list_index;
}

}  // namespace six_dof_constraint

TUMBLE_NAMESPACE_END
