#include "tumble/physics/constraints/fixed_constraint.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace fixed_constraint {

using constraint_id::ConstraintId;
using rigid_body::RigidBody;

static bool is_body_live(const bodies::Bodies &bodies, uint32_t index) {
    return index < bodies.pool.length && bodies.pool[index] != nullptr && bodies.sequences[index] != bodies::SLOT_FREE;
}

/** creates default fixed constraint */
static FixedConstraint make_fixed_constraint() {
    FixedConstraint constraint;
    constraint.base = constraints::make_constraint_base();
    constraint.local_space_position1 = vec3f::zero();
    constraint.local_space_position2 = vec3f::zero();
    constraint.inv_initial_orientation = quatf::identity();
    constraint.point_constraint_part = point_constraint_part::create();
    constraint.rotation_constraint_part = rotation_euler_constraint_part::create();
    return constraint;
}

/** reset constraint state for pooling reuse */
static void reset_constraint(FixedConstraint &constraint) {
    constraint.base.enabled = true;
    constraint.base.sleeping = false;
    constraint.base.pooled = false;
    rotation_euler_constraint_part::deactivate(constraint.rotation_constraint_part);
    point_constraint_part::deactivate(constraint.point_constraint_part);
}

ConstraintId create(constraints::Constraints &constraints, bodies::Bodies &bodies, RigidBody *body_a, RigidBody *body_b,
                    const FixedConstraintSettings &settings) {
    TUMBLE_ASSERT(is_body_live(bodies, body_a->index) && bodies.pool[body_a->index] == body_a);
    TUMBLE_ASSERT(is_body_live(bodies, body_b->index) && bodies.pool[body_b->index] == body_b);
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::FIXED);

    const uint32_t index = constraints::allocate_index(pool);
    if (index == ~0u) return constraint_id::INVALID_CONSTRAINT_ID;
    const uint32_t sequence = constraints::next_sequence(pool, index);
    FixedConstraint &constraint = *reinterpret_cast<FixedConstraint *>(constraints::base_at(pool, index));
    constraint = make_fixed_constraint();
    reset_constraint(constraint);

    constraint.base.id = constraint_id::ser(index, constraint_id::FIXED, sequence);
    constraint.base.index = index;
    constraint.base.sequence = sequence;

    constraint.base.constraint_priority = settings.constraint_priority;
    constraint.base.num_velocity_steps_override = settings.num_velocity_steps_override;
    constraint.base.num_position_steps_override = settings.num_position_steps_override;

    constraint.base.body_index_a = body_a->index;
    constraint.base.body_index_b = body_b->index;

    // normalize axes
    const Vec3f axis_x1 = vec3f::normalize(vec3f::load(settings.axis_x1));
    const Vec3f axis_y1 = vec3f::normalize(vec3f::load(settings.axis_y1));
    const Vec3f axis_x2 = vec3f::normalize(vec3f::load(settings.axis_x2));
    const Vec3f axis_y2 = vec3f::normalize(vec3f::load(settings.axis_y2));

    // initial orientation from the reference axes
    const Quatf r0 = rotation_euler_constraint_part::get_inv_initial_orientation_xy(axis_x1, axis_y1, axis_x2, axis_y2);

    if (settings.space == constraints::SPACE_WORLD) {
        const Quatf inv_quat_a = quatf::conjugate(quatf::load(body_a->quaternion));
        const Quatf inv_quat_b = quatf::conjugate(quatf::load(body_b->quaternion));

        if (settings.auto_detect_point) {
            // if either body is static, anchor on the other body's center of mass.
            // otherwise weight the anchor towards the lighter body (higher inv mass = more influence)
            const float inv_m1 = body_a->motion_properties.inv_mass;
            const float inv_m2 = body_b->motion_properties.inv_mass;
            const float total_inv_mass = inv_m1 + inv_m2;

            const Vec3f center_of_mass_a = vec3f::load(body_a->center_of_mass_position);
            const Vec3f center_of_mass_b = vec3f::load(body_b->center_of_mass_position);
            Vec3f anchor;
            if (body_a->motion_type == motion_type::MOTION_STATIC) {
                anchor = center_of_mass_b;
            } else if (body_b->motion_type == motion_type::MOTION_STATIC) {
                anchor = center_of_mass_a;
            } else if (total_inv_mass > 0.0f) {
                anchor = vec3f::scale_and_add(vec3f::zero(), center_of_mass_a, inv_m1 / total_inv_mass);
                anchor = vec3f::scale_and_add(anchor, center_of_mass_b, inv_m2 / total_inv_mass);
            } else {
                // both infinite mass: body a's position
                anchor = center_of_mass_a;
            }

            constraint.local_space_position1 = vec3f::transform_quat(vec3f::subtract(anchor, center_of_mass_a), inv_quat_a);
            constraint.local_space_position2 = vec3f::transform_quat(vec3f::subtract(anchor, center_of_mass_b), inv_quat_b);
        } else {
            constraint.local_space_position1 = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point1), vec3f::load(body_a->center_of_mass_position)), inv_quat_a);

            constraint.local_space_position2 = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point2), vec3f::load(body_b->center_of_mass_position)), inv_quat_b);
        }

        // the axes were given in world space, so c1 becomes q10^-1 c1 and c2 becomes q20^-1 c2:
        // r0^-1 = (q20^-1 c2) (q10^-1 c1)^-1 = q20^-1 (c2 c1^-1) q10
        constraint.inv_initial_orientation = quatf::multiply(quatf::multiply(inv_quat_b, r0), quatf::load(body_a->quaternion));
    } else {
        constraint.local_space_position1 = vec3f::load(settings.point1);
        constraint.local_space_position2 = vec3f::load(settings.point2);
        constraint.inv_initial_orientation = r0;
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

void remove(constraints::Constraints &constraints, bodies::Bodies &bodies, FixedConstraint &constraint) {
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::FIXED);

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

FixedConstraint *get(constraints::Constraints &constraints, ConstraintId id) {
    if (constraint_id::type(id) != constraint_id::FIXED) return nullptr;
    return reinterpret_cast<FixedConstraint *>(constraints::get(constraints, id));
}

void setup_velocity(FixedConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    (void)delta_time;
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    // rotation (3 DOF, keeps orientations locked)
    rotation_euler_constraint_part::calculate_constraint_properties(constraint.rotation_constraint_part, body_a, rotation_a,
                                                                    body_b, rotation_b, bodies.step_stamp);

    // translation (3 DOF, keeps positions together)
    point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                           constraint.local_space_position1, body_b, rotation_b,
                                                           constraint.local_space_position2, bodies.step_stamp);
}

void warm_start_velocity(FixedConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    rotation_euler_constraint_part::warm_start(constraint.rotation_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    point_constraint_part::warm_start(constraint.point_constraint_part, body_a, body_b, warm_start_impulse_ratio);
}

bool solve_velocity(FixedConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    (void)delta_time;
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    // rotation first, so translation is not disturbed by rotation errors
    const bool rot = rotation_euler_constraint_part::solve_velocity_constraint(constraint.rotation_constraint_part, body_a, body_b);
    const bool pos = point_constraint_part::solve_velocity_constraint(constraint.point_constraint_part, body_a, body_b);

    return rot || pos;
}

bool solve_position(FixedConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte) {
    (void)delta_time;
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    // recalculate constraint properties (the bodies may have moved)
    Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    rotation_euler_constraint_part::calculate_constraint_properties(constraint.rotation_constraint_part, body_a, rotation_a,
                                                                    body_b, rotation_b, motion_properties::STEP_STAMP_NONE);
    const bool rot = rotation_euler_constraint_part::solve_position_constraint(
        constraint.rotation_constraint_part, body_a, body_b, constraint.inv_initial_orientation, baumgarte);

    // the rotation solve moved the bodies
    rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                           constraint.local_space_position1, body_b, rotation_b,
                                                           constraint.local_space_position2, motion_properties::STEP_STAMP_NONE);
    const bool pos = point_constraint_part::solve_position_constraint(constraint.point_constraint_part, body_a, body_b, baumgarte);

    return rot || pos;
}

void notify_shape_changed(FixedConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com) {
    // separate tests, not jolt's else-if: a constraint between a body and itself moves both points
    const uint32_t body_index = body_id::index(body_id);
    if (constraint.base.body_index_a == body_index)
        constraint.local_space_position1 = vec3f::subtract(constraint.local_space_position1, delta_com);
    if (constraint.base.body_index_b == body_index)
        constraint.local_space_position2 = vec3f::subtract(constraint.local_space_position2, delta_com);
}

void reset_warm_start(FixedConstraint &constraint) {
    rotation_euler_constraint_part::deactivate(constraint.rotation_constraint_part);
    point_constraint_part::deactivate(constraint.point_constraint_part);
}

void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const FixedConstraint &constraint) {
    out.velocity = constraint.base.num_velocity_steps_override;
    out.position = constraint.base.num_position_steps_override;
}

void get_sort_fields(constraints::ConstraintSortFields &out, const FixedConstraint &constraint) {
    out.priority = constraint.base.constraint_priority;
    out.index = constraint.base.index;
}

Vec3f get_total_lambda_position(const FixedConstraint &constraint) { return constraint.point_constraint_part.total_lambda; }

Vec3f get_total_lambda_rotation(const FixedConstraint &constraint) { return constraint.rotation_constraint_part.total_lambda; }

}  // namespace fixed_constraint

TUMBLE_NAMESPACE_END
