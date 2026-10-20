#include "tumble/physics/constraints/cone_constraint.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace cone_constraint {

using constraint_id::ConstraintId;
using rigid_body::RigidBody;

static bool is_body_live(const bodies::Bodies &bodies, uint32_t index) {
    return index < bodies.pool.length && bodies.pool[index] != nullptr && bodies.sequences[index] != bodies::SLOT_FREE;
}

/** creates default cone constraint */
static ConeConstraint make_cone_constraint() {
    ConeConstraint constraint;
    constraint.base = constraints::make_constraint_base();
    constraint.local_space_position1 = vec3f::zero();
    constraint.local_space_position2 = vec3f::zero();
    constraint.local_space_twist_axis1 = vec3f::axis_x();
    constraint.local_space_twist_axis2 = vec3f::axis_x();
    // 0 degrees: no swing allowed by default
    constraint.cos_half_cone_angle = 1.0f;
    constraint.world_space_rotation_axis = vec3f::axis_y();
    constraint.cos_theta = 1.0f;
    constraint.point_constraint_part = point_constraint_part::create();
    constraint.angle_constraint_part = angle_constraint_part::create();
    return constraint;
}

/** reset constraint state for pooling reuse */
static void reset_constraint(ConeConstraint &constraint) {
    constraint.base.enabled = true;
    constraint.base.sleeping = false;
    constraint.base.pooled = false;
    point_constraint_part::deactivate(constraint.point_constraint_part);
    angle_constraint_part::deactivate(constraint.angle_constraint_part);
}

ConstraintId create(constraints::Constraints &constraints, bodies::Bodies &bodies, RigidBody *body_a, RigidBody *body_b,
                    const ConeConstraintSettings &settings) {
    CC_ASSERT(is_body_live(bodies, body_a->index) && bodies.pool[body_a->index] == body_a);
    CC_ASSERT(is_body_live(bodies, body_b->index) && bodies.pool[body_b->index] == body_b);
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::CONE);

    const uint32_t index = constraints::allocate_index(pool);
    if (index == ~0u) return constraint_id::INVALID_CONSTRAINT_ID;
    const uint32_t sequence = constraints::next_sequence(pool, index);
    ConeConstraint &constraint = *reinterpret_cast<ConeConstraint *>(constraints::base_at(pool, index));
    constraint = make_cone_constraint();
    reset_constraint(constraint);

    constraint.base.id = constraint_id::ser(index, constraint_id::CONE, sequence);
    constraint.base.index = index;
    constraint.base.sequence = sequence;

    constraint.base.constraint_priority = settings.constraint_priority;
    constraint.base.num_velocity_steps_override = settings.num_velocity_steps_override;
    constraint.base.num_position_steps_override = settings.num_position_steps_override;

    constraint.base.body_index_a = body_a->index;
    constraint.base.body_index_b = body_b->index;

    const Vec3f twist_axis1 = vec3f::normalize(vec3f::load(settings.twist_axis_a));
    const Vec3f twist_axis2 = vec3f::normalize(vec3f::load(settings.twist_axis_b));

    set_half_cone_angle(constraint, settings.half_cone_angle);

    // the rotation axis starts perpendicular to the twist axis, for when the twist axes align in the first step
    constraint.world_space_rotation_axis = vec3f::perpendicular(twist_axis1);

    if (settings.space == constraints::SPACE_WORLD) {
        const Quatf inv_quat_a = quatf::conjugate(quatf::load(body_a->quaternion));
        const Quatf inv_quat_b = quatf::conjugate(quatf::load(body_b->quaternion));

        constraint.local_space_position1 =
            vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point_a), vec3f::load(body_a->center_of_mass_position)), inv_quat_a);
        constraint.local_space_position2 =
            vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point_b), vec3f::load(body_b->center_of_mass_position)), inv_quat_b);

        constraint.local_space_twist_axis1 = vec3f::transform_quat(twist_axis1, inv_quat_a);
        constraint.local_space_twist_axis2 = vec3f::transform_quat(twist_axis2, inv_quat_b);
    } else {
        constraint.local_space_position1 = vec3f::load(settings.point_a);
        constraint.local_space_position2 = vec3f::load(settings.point_b);
        constraint.local_space_twist_axis1 = twist_axis1;
        constraint.local_space_twist_axis2 = twist_axis2;

        // the initial rotation axis came from a body space twist axis: take it to world space
        constraint.world_space_rotation_axis = vec3f::transform_quat(constraint.world_space_rotation_axis, quatf::load(body_a->quaternion));
    }

    // track constraint on both bodies
    const ConstraintId id = constraint.base.id;
    bool linked = body_a->constraint_ids.push(id);
    if (linked && body_a != body_b) linked = body_b->constraint_ids.push(id);
    if (!linked) {
        constraints::remove_constraint_by_id(constraints, bodies, id);
        return constraint_id::INVALID_CONSTRAINT_ID;
    }

    return id;
}

void remove(constraints::Constraints &constraints, bodies::Bodies &bodies, ConeConstraint &constraint) {
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::CONE);

    // remove from bodies' constraint_ids
    const uint32_t a = constraint.base.body_index_a;
    const uint32_t b = constraint.base.body_index_b;
    if (is_body_live(bodies, a)) constraints::remove_constraint_id_from_body(*bodies.pool[a], constraint.base.id);
    if (a != b && is_body_live(bodies, b)) constraints::remove_constraint_id_from_body(*bodies.pool[b], constraint.base.id);

    reset_constraint(constraint);

    constraint.base.pooled = true;
    constraint.base.id = constraint_id::INVALID_CONSTRAINT_ID;
    pool.free_indices.push(constraint.base.index);
}

ConeConstraint *get(constraints::Constraints &constraints, ConstraintId id) {
    if (constraint_id::type(id) != constraint_id::CONE) return nullptr;
    return reinterpret_cast<ConeConstraint *>(constraints::get(constraints, id));
}

void set_half_cone_angle(ConeConstraint &constraint, float half_cone_angle) {
    CC_ASSERT(half_cone_angle >= 0.0f && half_cone_angle <= scalar::PI);
    constraint.cos_half_cone_angle = scalar::cos(half_cone_angle);
}

float get_cos_half_cone_angle(const ConeConstraint &constraint) { return constraint.cos_half_cone_angle; }

float get_half_cone_angle(const ConeConstraint &constraint) { return scalar::acos(constraint.cos_half_cone_angle); }

/** whether the cone limit is violated, and if so the angle part set up about the axis that closes the swing */
static void calculate_rotation_constraint_properties(ConeConstraint &constraint, RigidBody &body_a, RigidBody &body_b) {
    // twist axes in world space
    const Vec3f twist1 = vec3f::transform_quat(constraint.local_space_twist_axis1, quatf::load(body_a.quaternion));
    const Vec3f twist2 = vec3f::transform_quat(constraint.local_space_twist_axis2, quatf::load(body_b.quaternion));

    constraint.cos_theta = vec3f::dot(twist1, twist2);

    if (constraint.cos_theta < constraint.cos_half_cone_angle) {
        // rotation axis is t2 x t1
        const Vec3f rotation_axis = vec3f::cross(twist2, twist1);

        // twist axes parallel, no axis of their own: keep last step's
        const float len = vec3f::length(rotation_axis);
        if (len > 0.0f) constraint.world_space_rotation_axis = vec3f::scale(rotation_axis, 1.0f / len);

        angle_constraint_part::calculate_constraint_properties(constraint.angle_constraint_part, body_a, body_b,
                                                               constraint.world_space_rotation_axis);
    } else {
        // inside the cone: no angular limit
        angle_constraint_part::deactivate(constraint.angle_constraint_part);
    }
}

void setup_velocity(ConeConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    (void)delta_time;
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));
    point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                           constraint.local_space_position1, body_b, rotation_b,
                                                           constraint.local_space_position2, bodies.step_stamp);

    calculate_rotation_constraint_properties(constraint, body_a, body_b);
}

void warm_start_velocity(ConeConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    point_constraint_part::warm_start(constraint.point_constraint_part, body_a, body_b, warm_start_impulse_ratio);

    if (angle_constraint_part::is_active(constraint.angle_constraint_part)) {
        angle_constraint_part::warm_start(constraint.angle_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    }
}

bool solve_velocity(ConeConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    (void)delta_time;
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const bool pos = point_constraint_part::solve_velocity_constraint(constraint.point_constraint_part, body_a, body_b);

    // the limit only pushes the axes back into the cone: lambda >= 0
    bool rot = false;
    if (angle_constraint_part::is_active(constraint.angle_constraint_part)) {
        rot = angle_constraint_part::solve_velocity_constraint(constraint.angle_constraint_part, body_a, body_b,
                                                               constraint.world_space_rotation_axis, 0.0f, scalar::FLOAT_MAX);
    }

    return pos || rot;
}

bool solve_position(ConeConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte) {
    (void)delta_time;
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));
    point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                           constraint.local_space_position1, body_b, rotation_b,
                                                           constraint.local_space_position2, motion_properties::STEP_STAMP_NONE);
    const bool pos = point_constraint_part::solve_position_constraint(constraint.point_constraint_part, body_a, body_b, baumgarte);

    calculate_rotation_constraint_properties(constraint, body_a, body_b);
    bool rot = false;
    if (angle_constraint_part::is_active(constraint.angle_constraint_part)) {
        // cos theta - cos half cone angle, negative while violated
        rot = angle_constraint_part::solve_position_constraint(constraint.angle_constraint_part, body_a, body_b,
                                                               constraint.cos_theta - constraint.cos_half_cone_angle, baumgarte);
    }

    return pos || rot;
}

void notify_shape_changed(ConeConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com) {
    // separate tests, not jolt's else-if: a constraint between a body and itself moves both points
    const uint32_t body_index = body_id::index(body_id);
    if (constraint.base.body_index_a == body_index)
        constraint.local_space_position1 = vec3f::subtract(constraint.local_space_position1, delta_com);
    if (constraint.base.body_index_b == body_index)
        constraint.local_space_position2 = vec3f::subtract(constraint.local_space_position2, delta_com);
}

void reset_warm_start(ConeConstraint &constraint) {
    point_constraint_part::deactivate(constraint.point_constraint_part);
    angle_constraint_part::deactivate(constraint.angle_constraint_part);
}

void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const ConeConstraint &constraint) {
    out.velocity = constraint.base.num_velocity_steps_override;
    out.position = constraint.base.num_position_steps_override;
}

void get_sort_fields(constraints::ConstraintSortFields &out, const ConeConstraint &constraint) {
    out.priority = constraint.base.constraint_priority;
    out.index = constraint.base.index;
}

Vec3f get_total_lambda_position(const ConeConstraint &constraint) { return constraint.point_constraint_part.total_lambda; }

float get_total_lambda_rotation(const ConeConstraint &constraint) { return constraint.angle_constraint_part.total_lambda; }

}  // namespace cone_constraint

TUMBLE_NAMESPACE_END
