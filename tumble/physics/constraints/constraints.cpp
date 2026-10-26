#include "tumble/physics/constraints/constraints.h"

#include "tumble/util/error.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/constraints/cone_constraint.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/constraints/fixed_constraint.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/constraints/six_dof_constraint.h"
#include "tumble/physics/constraints/slider_constraint.h"
#include "tumble/physics/constraints/swing_twist_constraint.h"
#include "tumble/physics/world.h"
#include "tumble/util/assert.h"
#include "tumble/util/mem.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

namespace constraints {

template <typename T>
static constexpr uint32_t record_stride() {
    return (uint32_t)((sizeof(T) + alignof(T) - 1) & ~(alignof(T) - 1));
}

template <typename T>
static T &record_at(Constraints::Pool &pool, uint32_t index) {
    return *reinterpret_cast<T *>(base_at(pool, index));
}

template <typename T>
static const T &record_at(const Constraints::Pool &pool, uint32_t index) {
    return *reinterpret_cast<const T *>(base_at(pool, index));
}

static bool is_body_live(const Bodies &bodies, uint32_t index) {
    return index < bodies.pool.length && bodies.pool[index] != nullptr && bodies.sequences[index] != bodies::SLOT_FREE;
}

Constraints create() {
    Constraints constraints = {};
    for (uint32_t type = 0; type < constraint_id::CONSTRAINT_TYPE_COUNT; type++) {
        constraints.pools[(uint32_t)type].type = (ConstraintType)type;
    }
    return constraints;
}

void add_to_list(Constraints &constraints, ConstraintBase &constraint) {
    constraints.list.push(constraint.id);
    constraint.list_index = constraints.list.length - 1;
}

void remove_from_list(Constraints &constraints, ConstraintBase &constraint) {
    const uint32_t list_index = constraint.list_index;
    if (list_index == ~0u) return;
    const uint32_t last_index = constraints.list.length - 1;
    if (list_index != last_index) {
        const ConstraintId last_id = constraints.list[last_index];
        constraints.list[list_index] = last_id;
        get(constraints, last_id)->list_index = list_index;
    }
    constraints.list.length = last_index;
    constraint.list_index = ~0u;
}

Constraints::Pool &ensure_pool(Constraints &constraints, ConstraintType type) {
    TUMBLE_ASSERT((uint32_t)type < constraint_id::CONSTRAINT_TYPE_COUNT);
    Constraints::Pool &pool = constraints.pools[(uint32_t)type];
    if (pool.stride != 0) return pool;
    pool.type = type;
    switch (type) {
        case ConstraintType::POINT:
            pool.stride = record_stride<PointConstraint>();
            break;
        case ConstraintType::DISTANCE:
            pool.stride = record_stride<DistanceConstraint>();
            break;
        case ConstraintType::FIXED:
            pool.stride = record_stride<FixedConstraint>();
            break;
        case ConstraintType::CONE:
            pool.stride = record_stride<ConeConstraint>();
            break;
        case ConstraintType::HINGE:
            pool.stride = record_stride<HingeConstraint>();
            break;
        case ConstraintType::SLIDER:
            pool.stride = record_stride<SliderConstraint>();
            break;
        case ConstraintType::SWING_TWIST:
            pool.stride = record_stride<SwingTwistConstraint>();
            break;
        case ConstraintType::SIX_DOF:
            pool.stride = record_stride<SixDOFConstraint>();
            break;
        // there are no user constraint types
        default:
            TUMBLE_ASSERT(false);
            break;
    }
    return pool;
}

ConstraintBase *get(Constraints &constraints, ConstraintId id) {
    const ConstraintType type = constraint_id::type(id);
    if ((uint32_t)type >= constraint_id::CONSTRAINT_TYPE_COUNT) return nullptr;
    Constraints::Pool &pool = constraints.pools[(uint32_t)type];
    const uint32_t index = constraint_id::index(id);
    if (pool.stride == 0 || index >= pool.count) return nullptr;
    ConstraintBase *constraint = base_at(pool, index);
    if (constraint->pooled || constraint->sequence != constraint_id::sequence(id)) return nullptr;
    return constraint;
}

/** unlinks a live constraint from both bodies and the list, and frees its record */
static void free_constraint(Constraints &constraints, Bodies &bodies, ConstraintBase &constraint) {
    const uint32_t a = constraint.body_index_a;
    const uint32_t b = constraint.body_index_b;
    if (is_body_live(bodies, a)) remove_constraint_id_from_body(*bodies.pool[a], constraint.id);
    if (a != b && is_body_live(bodies, b)) remove_constraint_id_from_body(*bodies.pool[b], constraint.id);
    remove_from_list(constraints, constraint);
    constraints.pools[(uint32_t)constraint_id::type(constraint.id)].free_indices.push(constraint.index);
    constraint.pooled = true;
}

bool remove(World &world, ConstraintId id) {
    ConstraintBase *constraint = get(world.constraints, id);
    if (constraint == nullptr) return false;
    free_constraint(world.constraints, world.bodies, *constraint);
    return true;
}

void destroy_body_constraints(Constraints &constraints, Bodies &bodies, RigidBody &body) {
    // backwards, since removing swap-removes from this body's list
    for (uint32_t i = body.constraint_ids.length; i-- > 0;) {
        if (i >= body.constraint_ids.length) continue;
        ConstraintBase *constraint = get(constraints, body.constraint_ids[i]);
        if (constraint != nullptr) free_constraint(constraints, bodies, *constraint);
    }
    body.constraint_ids.length = 0;
}

void notify_shape_changed(Constraints &constraints, const RigidBody &body, Vec3f delta_com) {
    for (uint32_t i = 0; i < body.constraint_ids.length; i++) {
        const ConstraintId id = body.constraint_ids[i];
        Constraints::Pool &pool = constraints.pools[(uint32_t)constraint_id::type(id)];
        const uint32_t index = constraint_id::index(id);
        switch (constraint_id::type(id)) {
            case ConstraintType::POINT:
                point_constraint::notify_shape_changed(record_at<PointConstraint>(pool, index), body.id, delta_com);
                break;
            case ConstraintType::DISTANCE:
                distance_constraint::notify_shape_changed(record_at<DistanceConstraint>(pool, index), body.id, delta_com);
                break;
            case ConstraintType::FIXED:
                fixed_constraint::notify_shape_changed(record_at<FixedConstraint>(pool, index), body.id, delta_com);
                break;
            case ConstraintType::CONE:
                cone_constraint::notify_shape_changed(record_at<ConeConstraint>(pool, index), body.id, delta_com);
                break;
            case ConstraintType::HINGE:
                hinge_constraint::notify_shape_changed(record_at<HingeConstraint>(pool, index), body.id, delta_com);
                break;
            case ConstraintType::SLIDER:
                slider_constraint::notify_shape_changed(record_at<SliderConstraint>(pool, index), body.id, delta_com);
                break;
            case ConstraintType::SWING_TWIST:
                swing_twist_constraint::notify_shape_changed(record_at<SwingTwistConstraint>(pool, index), body.id,
                                                             delta_com);
                break;
            case ConstraintType::SIX_DOF:
                six_dof_constraint::notify_shape_changed(record_at<SixDOFConstraint>(pool, index), body.id, delta_com);
                break;
            default:
                TUMBLE_ASSERT(false);
                break;
        }
    }
}

void get_constraint_iteration_overrides(Constraints::IterationOverrides &out, const Constraints &constraints, ConstraintId id) {
    out.velocity = 0;
    out.position = 0;
    const ConstraintType type = constraint_id::type(id);
    if ((uint32_t)type >= constraint_id::CONSTRAINT_TYPE_COUNT) return;
    const Constraints::Pool &pool = constraints.pools[(uint32_t)type];
    if (pool.stride == 0) return;
    const uint32_t index = constraint_id::index(id);
    TUMBLE_ASSERT(index < pool.count);
    switch (type) {
        case ConstraintType::POINT:
            point_constraint::get_iteration_overrides(out, record_at<PointConstraint>(pool, index));
            break;
        case ConstraintType::DISTANCE:
            distance_constraint::get_iteration_overrides(out, record_at<DistanceConstraint>(pool, index));
            break;
        case ConstraintType::FIXED:
            fixed_constraint::get_iteration_overrides(out, record_at<FixedConstraint>(pool, index));
            break;
        case ConstraintType::CONE:
            cone_constraint::get_iteration_overrides(out, record_at<ConeConstraint>(pool, index));
            break;
        case ConstraintType::HINGE:
            hinge_constraint::get_iteration_overrides(out, record_at<HingeConstraint>(pool, index));
            break;
        case ConstraintType::SLIDER:
            slider_constraint::get_iteration_overrides(out, record_at<SliderConstraint>(pool, index));
            break;
        case ConstraintType::SWING_TWIST:
            swing_twist_constraint::get_iteration_overrides(out, record_at<SwingTwistConstraint>(pool, index));
            break;
        case ConstraintType::SIX_DOF:
            six_dof_constraint::get_iteration_overrides(out, record_at<SixDOFConstraint>(pool, index));
            break;
        default:
            break;
    }
}

void get_constraint_sort_fields(Constraints::SortFields &out, const Constraints &constraints, ConstraintId id) {
    out.priority = 0;
    out.index = 0;
    const ConstraintType type = constraint_id::type(id);
    if ((uint32_t)type >= constraint_id::CONSTRAINT_TYPE_COUNT) return;
    const Constraints::Pool &pool = constraints.pools[(uint32_t)type];
    if (pool.stride == 0) return;
    const uint32_t index = constraint_id::index(id);
    TUMBLE_ASSERT(index < pool.count);
    switch (type) {
        case ConstraintType::POINT:
            point_constraint::get_sort_fields(out, record_at<PointConstraint>(pool, index));
            break;
        case ConstraintType::DISTANCE:
            distance_constraint::get_sort_fields(out, record_at<DistanceConstraint>(pool, index));
            break;
        case ConstraintType::FIXED:
            fixed_constraint::get_sort_fields(out, record_at<FixedConstraint>(pool, index));
            break;
        case ConstraintType::CONE:
            cone_constraint::get_sort_fields(out, record_at<ConeConstraint>(pool, index));
            break;
        case ConstraintType::HINGE:
            hinge_constraint::get_sort_fields(out, record_at<HingeConstraint>(pool, index));
            break;
        case ConstraintType::SLIDER:
            slider_constraint::get_sort_fields(out, record_at<SliderConstraint>(pool, index));
            break;
        case ConstraintType::SWING_TWIST:
            swing_twist_constraint::get_sort_fields(out, record_at<SwingTwistConstraint>(pool, index));
            break;
        case ConstraintType::SIX_DOF:
            six_dof_constraint::get_sort_fields(out, record_at<SixDOFConstraint>(pool, index));
            break;
        default:
            break;
    }
}

/** solve order: lower priority first, so the highest priority is solved last and most likely satisfied; then by index */
static bool sorts_before(const Constraints &constraints, ConstraintId a, ConstraintId b) {
    Constraints::SortFields fields_a;
    Constraints::SortFields fields_b;
    get_constraint_sort_fields(fields_a, constraints, a);
    get_constraint_sort_fields(fields_b, constraints, b);
    if (fields_a.priority != fields_b.priority) return fields_a.priority < fields_b.priority;
    return fields_a.index < fields_b.index;
}

void sort_constraint_ids(const Constraints &constraints, collections::Array<ConstraintId> &ids) {
    // an island's constraint list is short
    sort::insertion_sort(ids.data, ids.data + ids.length, [&](ConstraintId a, ConstraintId b) { return sorts_before(constraints, a, b); });
}

void update_sleeping(Constraints &constraints, const Bodies &bodies) {
    for (uint32_t type = 0; type < constraint_id::CONSTRAINT_TYPE_COUNT; type++) {
        Constraints::Pool &pool = constraints.pools[(uint32_t)type];
        for (uint32_t i = 0; i < pool.count; i++) {
            ConstraintBase *constraint = base_at(pool, i);
            if (constraint->pooled) continue;
            const RigidBody *body_a = bodies.pool[constraint->body_index_a];
            const RigidBody *body_b = bodies.pool[constraint->body_index_b];
            // asleep unless a body is active; a static body never is
            constraint->sleeping = !rigid_body::is_active(*body_a) && !rigid_body::is_active(*body_b);
        }
    }
}

static bool is_solvable(const ConstraintBase *constraint) {
    return !constraint->pooled && constraint->enabled && !constraint->sleeping;
}

void setup_velocity_constraints(Constraints &constraints, Bodies &bodies, float delta_time) {
    for (uint32_t type = 0; type < constraint_id::CONSTRAINT_TYPE_COUNT; type++) {
        Constraints::Pool &pool = constraints.pools[(uint32_t)type];
        for (uint32_t i = 0; i < pool.count; i++) {
            if (!is_solvable(base_at(pool, i))) continue;
            switch (pool.type) {
                case ConstraintType::POINT:
                    point_constraint::setup_velocity(record_at<PointConstraint>(pool, i), bodies, delta_time);
                    break;
                case ConstraintType::DISTANCE:
                    distance_constraint::setup_velocity(record_at<DistanceConstraint>(pool, i), bodies,
                                                        delta_time);
                    break;
                case ConstraintType::FIXED:
                    fixed_constraint::setup_velocity(record_at<FixedConstraint>(pool, i), bodies, delta_time);
                    break;
                case ConstraintType::CONE:
                    cone_constraint::setup_velocity(record_at<ConeConstraint>(pool, i), bodies, delta_time);
                    break;
                case ConstraintType::HINGE:
                    hinge_constraint::setup_velocity(record_at<HingeConstraint>(pool, i), bodies, delta_time);
                    break;
                case ConstraintType::SLIDER:
                    slider_constraint::setup_velocity(record_at<SliderConstraint>(pool, i), bodies, delta_time);
                    break;
                case ConstraintType::SWING_TWIST:
                    swing_twist_constraint::setup_velocity(record_at<SwingTwistConstraint>(pool, i), bodies,
                                                           delta_time);
                    break;
                case ConstraintType::SIX_DOF:
                    six_dof_constraint::setup_velocity(record_at<SixDOFConstraint>(pool, i), bodies, delta_time);
                    break;
                default:
                    break;
            }
        }
    }
}

void warm_start_velocity_constraints(Constraints &constraints, Bodies &bodies, const collections::Array<ConstraintId> &ids, float warm_start_impulse_ratio) {
    for (uint32_t i = 0; i < ids.length; i++) {
        const ConstraintId id = ids[i];
        const ConstraintType type = constraint_id::type(id);
        if ((uint32_t)type >= constraint_id::CONSTRAINT_TYPE_COUNT) continue;
        Constraints::Pool &pool = constraints.pools[(uint32_t)type];
        if (pool.stride == 0) continue;
        const uint32_t index = constraint_id::index(id);
        TUMBLE_ASSERT(index < pool.count);

        switch (type) {
            case ConstraintType::POINT:
                point_constraint::warm_start_velocity(record_at<PointConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case ConstraintType::DISTANCE:
                distance_constraint::warm_start_velocity(record_at<DistanceConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case ConstraintType::FIXED:
                fixed_constraint::warm_start_velocity(record_at<FixedConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case ConstraintType::CONE:
                cone_constraint::warm_start_velocity(record_at<ConeConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case ConstraintType::HINGE:
                hinge_constraint::warm_start_velocity(record_at<HingeConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case ConstraintType::SLIDER:
                slider_constraint::warm_start_velocity(record_at<SliderConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case ConstraintType::SWING_TWIST:
                swing_twist_constraint::warm_start_velocity(record_at<SwingTwistConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            case ConstraintType::SIX_DOF:
                six_dof_constraint::warm_start_velocity(record_at<SixDOFConstraint>(pool, index), bodies, warm_start_impulse_ratio);
                break;
            default:
                break;
        }
    }
}

bool solve_velocity_constraints_for_island(Constraints &constraints, Bodies &bodies, const collections::Array<ConstraintId> &ids,
                                           float delta_time) {
    bool any_impulse_applied = false;

    for (uint32_t i = 0; i < ids.length; i++) {
        const ConstraintId id = ids[i];
        const ConstraintType type = constraint_id::type(id);
        if ((uint32_t)type >= constraint_id::CONSTRAINT_TYPE_COUNT) continue;
        Constraints::Pool &pool = constraints.pools[(uint32_t)type];
        if (pool.stride == 0) continue;
        const uint32_t index = constraint_id::index(id);
        TUMBLE_ASSERT(index < pool.count);

        bool applied = false;
        switch (type) {
            case ConstraintType::POINT:
                applied = point_constraint::solve_velocity(record_at<PointConstraint>(pool, index), bodies,
                                                           delta_time);
                break;
            case ConstraintType::DISTANCE:
                applied = distance_constraint::solve_velocity(record_at<DistanceConstraint>(pool, index),
                                                              bodies, delta_time);
                break;
            case ConstraintType::FIXED:
                applied = fixed_constraint::solve_velocity(record_at<FixedConstraint>(pool, index), bodies,
                                                           delta_time);
                break;
            case ConstraintType::CONE:
                applied = cone_constraint::solve_velocity(record_at<ConeConstraint>(pool, index), bodies,
                                                          delta_time);
                break;
            case ConstraintType::HINGE:
                applied = hinge_constraint::solve_velocity(record_at<HingeConstraint>(pool, index), bodies,
                                                           delta_time);
                break;
            case ConstraintType::SLIDER:
                applied = slider_constraint::solve_velocity(record_at<SliderConstraint>(pool, index), bodies,
                                                            delta_time);
                break;
            case ConstraintType::SWING_TWIST:
                applied = swing_twist_constraint::solve_velocity(
                    record_at<SwingTwistConstraint>(pool, index), bodies, delta_time);
                break;
            case ConstraintType::SIX_DOF:
                applied = six_dof_constraint::solve_velocity(record_at<SixDOFConstraint>(pool, index), bodies,
                                                             delta_time);
                break;
            default:
                break;
        }
        any_impulse_applied = any_impulse_applied || applied;
    }

    return any_impulse_applied;
}

bool solve_position_constraints_for_island(Constraints &constraints, Bodies &bodies, const collections::Array<ConstraintId> &ids,
                                           float baumgarte_factor, float delta_time) {
    bool applied_impulse = false;

    for (uint32_t i = 0; i < ids.length; i++) {
        const ConstraintId id = ids[i];
        const ConstraintType type = constraint_id::type(id);
        if ((uint32_t)type >= constraint_id::CONSTRAINT_TYPE_COUNT) continue;
        Constraints::Pool &pool = constraints.pools[(uint32_t)type];
        if (pool.stride == 0) continue;
        const uint32_t index = constraint_id::index(id);
        TUMBLE_ASSERT(index < pool.count);

        bool applied = false;
        switch (type) {
            case ConstraintType::POINT:
                applied = point_constraint::solve_position(record_at<PointConstraint>(pool, index), bodies,
                                                           delta_time, baumgarte_factor);
                break;
            case ConstraintType::DISTANCE:
                applied = distance_constraint::solve_position(record_at<DistanceConstraint>(pool, index),
                                                              bodies, delta_time, baumgarte_factor);
                break;
            case ConstraintType::FIXED:
                applied = fixed_constraint::solve_position(record_at<FixedConstraint>(pool, index), bodies,
                                                           delta_time, baumgarte_factor);
                break;
            case ConstraintType::CONE:
                applied = cone_constraint::solve_position(record_at<ConeConstraint>(pool, index), bodies,
                                                          delta_time, baumgarte_factor);
                break;
            case ConstraintType::HINGE:
                applied = hinge_constraint::solve_position(record_at<HingeConstraint>(pool, index), bodies,
                                                           delta_time, baumgarte_factor);
                break;
            case ConstraintType::SLIDER:
                applied = slider_constraint::solve_position(record_at<SliderConstraint>(pool, index), bodies,
                                                            delta_time, baumgarte_factor);
                break;
            case ConstraintType::SWING_TWIST:
                applied = swing_twist_constraint::solve_position(
                    record_at<SwingTwistConstraint>(pool, index), bodies, delta_time, baumgarte_factor);
                break;
            case ConstraintType::SIX_DOF:
                applied = six_dof_constraint::solve_position(record_at<SixDOFConstraint>(pool, index), bodies,
                                                             delta_time, baumgarte_factor);
                break;
            default:
                break;
        }
        applied_impulse = applied_impulse || applied;
    }

    return applied_impulse;
}

void remove_constraint_id_from_body(RigidBody &body, ConstraintId id) {
    for (uint32_t i = 0; i < body.constraint_ids.length; i++) {
        if (body.constraint_ids[i] == id) {
            body.constraint_ids.swap_remove(i);
            return;
        }
    }
}

uint32_t allocate_index(Constraints::Pool &pool) {
    if (pool.free_indices.length > 0) return pool.free_indices.data[--pool.free_indices.length];
    // grow only as far as the id has index bits for
    if (pool.count > constraint_id::MAX_CONSTRAINT_INDEX) {
        util::set_error(Error::QUOTA);
        return ~0u;
    }
    const uint32_t bytes = (pool.count + 1) * pool.stride;
    if (bytes > pool.storage.capacity) {
        const uint32_t doubled = pool.storage.capacity * 2;
        pool.storage.reserve(doubled > bytes ? doubled : bytes);
    }
    pool.storage.length = bytes;
    base_at(pool, pool.count)->sequence = 0;
    return pool.count++;
}

}  // namespace constraints

TUMBLE_NAMESPACE_END
