#include "tumble/physics/body/rigid_body.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/util/assert.h"
#include "tumble/util/object.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {

/** a new body with default properties */
/** the one empty shape every shapeless or removed body of the pool holds, made on first use and freed with the pool */
static shapes::Shape *shared_empty_shape(bodies::Bodies &bodies) {
    if (bodies.pooled_body_shape == nullptr) bodies.pooled_body_shape = empty::create();
    return bodies.pooled_body_shape;
}

static void make_rigid_body(RigidBody *body) {
    body->id = body_id::INVALID_BODY_ID;
    body->user_data = 0;
    body->position = {0.0f, 0.0f, 0.0f};
    body->quaternion = {0.0f, 0.0f, 0.0f, 1.0f};
    body->center_of_mass_position = {0.0f, 0.0f, 0.0f};
    body->mass_properties = mass_properties::create();
    body->mass_properties_override = CALCULATE_MASS_AND_INERTIA;
    body->motion_properties = motion_properties::create();
    body->friction = 0.0f;
    body->restitution = 0.0f;
    body->friction_combine_mode = combine_material::AVERAGE;
    body->restitution_combine_mode = combine_material::AVERAGE;
    body->collision_groups = 0;
    body->collision_mask = 0;
    body->use_manifold_reduction = true;
    body->motion_type = motion_type::MOTION_STATIC;
    body->sleeping = false;
    body->sensor = false;
    body->enhanced_internal_edge_removal = false;
    body->collide_kinematic_vs_non_dynamic = false;
    body->observes_contacts = false;
    body->shape = nullptr;
    aabox::store(body->aabb, aabox::empty());
    body->object_layer = 0;
    body->broadphase_layer = -1;
    body->broadphase_node = -1;
    body->in_move_set = false;
    body->head_pair_key = -1;
    body->island_index = -1;
    body->active_index = sleep::INACTIVE_BODY_INDEX;
    body->ccd_body_index = -1;
}

/** initializes a body with the given settings */
static void set_rigid_body(RigidBody *body, bodies::Bodies &bodies, const RigidBodySettings &o) {
    body->user_data = o.user_data;

    body->position = o.position;
    body->quaternion = o.quaternion;
    body->center_of_mass_position = {0.0f, 0.0f, 0.0f};

    // handle mass properties with three-tier precedence:
    // 1. mass_properties_override (full control)
    // 2. mass (calculate and scale inertia)
    // 3. auto-calculate from shape (default)
    if (o.mass_properties_override != nullptr) {
        // full override - use provided mass and inertia as-is
        mass_properties::copy(body->mass_properties, *o.mass_properties_override);
        body->mass_properties_override = MASS_AND_INERTIA_PROVIDED;
    } else if (o.mass >= 0.0f) {
        // mass-only override - will calculate inertia from shape and scale it
        // this is deferred to update_shape() where we have access to the shape
        body->mass_properties.mass = o.mass;
        body->mass_properties_override = CALCULATE_INERTIA;
    } else {
        // auto-calculate from shape
        mass_properties::reset(body->mass_properties);
        body->mass_properties_override = CALCULATE_MASS_AND_INERTIA;
    }

    motion_properties::MotionProperties &mp = body->motion_properties;

    // reset velocities and accumulators (important for pooled bodies)
    mp.linear_velocity = {0.0f, 0.0f, 0.0f};
    mp.angular_velocity = {0.0f, 0.0f, 0.0f};
    mp.force = {0.0f, 0.0f, 0.0f};
    mp.torque = {0.0f, 0.0f, 0.0f};

    // reset mass state and memos a previous body in this slot may have left behind.
    // update_shape only sets mass for non-static bodies, so statics rely on this
    mp.inv_mass = 0.0f;
    mp.inv_inertia_diagonal = {0.0f, 0.0f, 0.0f};
    mp.inertia_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
    mp.sleep_test_timer = 0.0f;
    mp.world_inverse_inertia_stamp = motion_properties::STEP_STAMP_NONE;

    mp.gravity_factor = o.gravity_factor;
    mp.linear_damping = o.linear_damping;
    mp.angular_damping = o.angular_damping;
    mp.max_linear_velocity = o.max_linear_velocity;
    mp.max_angular_velocity = o.max_angular_velocity;
    mp.num_velocity_steps_override = o.num_velocity_steps_override;
    mp.num_position_steps_override = o.num_position_steps_override;
    CC_ASSERT(mp.num_velocity_steps_override < 256);
    CC_ASSERT(mp.num_position_steps_override < 256);
    mp.allowed_degrees_of_freedom = o.allowed_degrees_of_freedom;
    mp.allow_sleeping = o.allow_sleeping;
    mp.motion_quality = o.motion_quality;

    body->friction = o.friction;
    body->restitution = o.restitution;

    body->friction_combine_mode = o.friction_combine_mode;
    body->restitution_combine_mode = o.restitution_combine_mode;

    body->collision_groups = o.collision_groups;
    body->collision_mask = o.collision_mask;

    body->use_manifold_reduction = o.use_manifold_reduction;

    body->motion_type = o.motion_type;

    body->sleeping = false;

    body->sensor = o.sensor;
    body->enhanced_internal_edge_removal = o.enhanced_internal_edge_removal;
    body->collide_kinematic_vs_non_dynamic = o.collide_kinematic_vs_non_dynamic;
    body->observes_contacts = o.observes_contacts;

    // a body with no shape holds the pool's shared empty shape, as a removed body does (nothing per body to free)
    body->shape = o.shape != nullptr ? o.shape : shared_empty_shape(bodies);

    aabox::store(body->aabb, aabox::empty());

    body->object_layer = o.object_layer;
    body->broadphase_layer = -1;
    body->broadphase_node = -1;
    body->in_move_set = false;

    body->active_index = sleep::INACTIVE_BODY_INDEX;

    body->head_pair_key = -1;
    body->island_index = -1;
    body->ccd_body_index = -1;

    body->constraint_ids.length = 0;
}

RigidBody *create(bodies::Bodies &bodies, const layers::Layers &layers, const RigidBodySettings &settings) {
    CC_ASSERT(settings.object_layer < layers.object_layers);
    (void)layers;

    // claim a slot: the last freed one, or a new one at the end
    const body_id::BodyId id = bodies::add_slot(bodies);
    if (id == body_id::INVALID_BODY_ID) return nullptr;
    const uint32_t index = body_id::index(id);

    // reuse the body a freed slot kept, or make one
    RigidBody *body = bodies.pool[index];
    if (body == nullptr) {
        body = util::create<RigidBody>();
        if (body == nullptr) {
            bodies::remove_slot(bodies, id);
            return nullptr;
        }
        make_rigid_body(body);
        bodies.pool[index] = body;
    }

    // set body id, index, sequence
    body->id = id;
    body->index = index;
    body->sequence = body_id::sequence(id);

    // initialize rigid body with settings
    set_rigid_body(body, bodies, settings);

    // update shape-related properties
    update_shape(body);

    // initialize sleep test spheres
    if (body->motion_type == motion_type::MOTION_DYNAMIC) {
        sleep::reset_sleep_timer(body);
    }

    // add to active bodies list if dynamic/kinematic and not sleeping
    if (body->motion_type != motion_type::MOTION_STATIC && !body->sleeping) {
        sleep::add_body_to_active_bodies(bodies, body);
    }

    return body;
}

bool remove(bodies::Bodies &bodies, RigidBody *body) {
    // return false if already removed
    if (!bodies::is_valid(bodies, body->id)) {
        return false;
    }

    // remove from active bodies list
    sleep::remove_body_from_active_bodies(bodies, body);

    // release references to user owned objects
    body->user_data = 0;
    body->shape = shared_empty_shape(bodies);

    // return index to pool
    bodies::remove_slot(bodies, body->id);

    return true;
}

RigidBody *get(const bodies::Bodies &bodies, body_id::BodyId body_id) { return bodies::get(bodies, body_id); }

Float4x4 &get_inverse_inertia(Float4x4 &out, RigidBody *body) {
    if (body->motion_type != motion_type::MOTION_DYNAMIC) {
        return mat4f::store(out, mat4f::zero());
    }
    return mat4f::store(out, motion_properties::get_world_inverse_inertia(body->motion_properties, quatf::load(body->quaternion),
                                                                          motion_properties::STEP_STAMP_NONE));
}

void update_center_of_mass_position(RigidBody *body) {
    // the shape-local center of mass, rotated to world space and translated to the body's position
    const Vec3f rotated = vec3f::transform_quat(vec3f::load(body->shape->center_of_mass), quatf::load(body->quaternion));
    vec3f::store(body->center_of_mass_position, vec3f::add(rotated, vec3f::load(body->position)));
}

void update_aabb(RigidBody *body) {
    const AABox shape_aabb = aabox::load(body->shape->aabb);
    Box3f &out = body->aabb;

    // empty input -> empty output (preserve the sentinel rather than producing a bogus transformed
    // box from negative extents)
    if (aabox::is_empty(shape_aabb)) {
        aabox::store(out, aabox::empty());
        return;
    }

    // centre and half-extents through the body's rotation and translation: the transformed extent of an axis is the sum
    // of the absolute contributions of each source axis
    const Mat4f transform = mat4f::from_rotation_translation(quatf::load(body->quaternion), vec3f::load(body->position));
    const Mat4f absolute_rotation = {{v4f::abs(transform.c[0]), v4f::abs(transform.c[1]), v4f::abs(transform.c[2]), transform.c[3]}};
    const Vec3f center = mat4f::multiply_vec3(transform, aabox::center(shape_aabb));
    const Vec3f extents = mat4f::multiply3x3_vec(absolute_rotation, aabox::extents(shape_aabb));
    aabox::store(out, aabox::make(vec3f::subtract(center, extents), vec3f::add(center, extents)));
}

void update_shape(RigidBody *body) {
    // handle mass properties based on mode
    if (body->mass_properties_override == CALCULATE_MASS_AND_INERTIA) {
        // auto-calculate from shape
        shapes::compute_mass_properties(body->mass_properties, body->shape);
    } else if (body->mass_properties_override == CALCULATE_INERTIA) {
        // mass-only override: calculate inertia from shape and scale to match mass
        const float override_mass = body->mass_properties.mass;

        // calculate mass properties from shape
        mass_properties::MassProperties shape_mass_properties = mass_properties::create();
        shapes::compute_mass_properties(shape_mass_properties, body->shape);

        // copy to body and scale to the override mass
        mass_properties::copy(body->mass_properties, shape_mass_properties);
        mass_properties::scale_to_mass(body->mass_properties, override_mass);
    }
    // else: MASS_AND_INERTIA_PROVIDED - use as-is, already set in set_rigid_body

    // only initialize motion properties for dynamic/kinematic bodies
    if (body->motion_type != motion_type::MOTION_STATIC) {
        motion_properties::set_mass_properties(body->motion_properties, body->motion_properties.allowed_degrees_of_freedom,
                                               body->mass_properties);
    }

    // recompute world-space center of mass after shape changes
    update_center_of_mass_position(body);

    // recompute world-space aabb after shape changes
    update_aabb(body);
}

void set_position(bodies::Bodies &bodies, RigidBody *body, Vec3f position, bool wake) {
    // set position
    vec3f::store(body->position, position);

    // update body properties
    update_center_of_mass_position(body);
    update_aabb(body);

    // optionally wake body
    if (wake) {
        sleep::wake(bodies, body);
    }
}

void set_quaternion(bodies::Bodies &bodies, RigidBody *body, Quatf quaternion, bool wake) {
    // set quaternion
    quatf::store(body->quaternion, quaternion);

    // update body properties
    body->motion_properties.world_inverse_inertia_stamp = motion_properties::STEP_STAMP_NONE;
    update_center_of_mass_position(body);
    update_aabb(body);

    // optionally wake body
    if (wake) {
        sleep::wake(bodies, body);
    }
}

void set_transform(bodies::Bodies &bodies, RigidBody *body, Vec3f position, Quatf quaternion, bool wake) {
    // set transform
    vec3f::store(body->position, position);
    quatf::store(body->quaternion, quaternion);

    // update body properties
    body->motion_properties.world_inverse_inertia_stamp = motion_properties::STEP_STAMP_NONE;
    update_center_of_mass_position(body);
    update_aabb(body);

    // optionally wake body
    if (wake) {
        sleep::wake(bodies, body);
    }
}

void set_object_layer(const layers::Layers &layers, RigidBody *body, uint32_t object_layer) {
    CC_ASSERT(object_layer < layers.object_layers);
    (void)layers;
    body->object_layer = object_layer;
}

void set_motion_type(bodies::Bodies &bodies, RigidBody *body, motion_type::MotionType motion_type, bool wake) {
    const motion_type::MotionType old_motion_type = body->motion_type;
    if (old_motion_type == motion_type) return;

    // update motion type
    body->motion_type = motion_type;

    // handle active bodies list transitions
    if (motion_type == motion_type::MOTION_STATIC) {
        // becoming static: remove from active list, zero velocities
        sleep::remove_body_from_active_bodies(bodies, body);
        body->motion_properties.linear_velocity = {0.0f, 0.0f, 0.0f};
        body->motion_properties.angular_velocity = {0.0f, 0.0f, 0.0f};
    } else if (old_motion_type == motion_type::MOTION_STATIC) {
        // was static, now dynamic/kinematic: add to active list if not sleeping
        if (!body->sleeping) {
            sleep::add_body_to_active_bodies(bodies, body);
        }
        // reset sleep timer for dynamic bodies
        if (motion_type == motion_type::MOTION_DYNAMIC) {
            sleep::reset_sleep_timer(body);
        }
    }

    // optionally wake (only relevant for dynamic/kinematic)
    if (wake && motion_type != motion_type::MOTION_STATIC) {
        sleep::wake(bodies, body);
    }

    // reinitialize motion properties if becoming dynamic/kinematic
    if (motion_type != motion_type::MOTION_STATIC && old_motion_type == motion_type::MOTION_STATIC) {
        motion_properties::set_mass_properties(body->motion_properties, body->motion_properties.allowed_degrees_of_freedom,
                                               body->mass_properties);
    }

    // a memoised world inverse inertia from the previous motion type is stale
    body->motion_properties.world_inverse_inertia_stamp = motion_properties::STEP_STAMP_NONE;
}

void add_force(bodies::Bodies &bodies, RigidBody *body, Vec3f force, bool wake) {
    if (body->motion_type != motion_type::MOTION_DYNAMIC) return;
    motion_properties::add_force(body->motion_properties, force);
    if (wake) sleep::wake(bodies, body);
}

void add_torque(bodies::Bodies &bodies, RigidBody *body, Vec3f torque, bool wake) {
    if (body->motion_type != motion_type::MOTION_DYNAMIC) return;
    motion_properties::add_torque(body->motion_properties, torque);
    if (wake) sleep::wake(bodies, body);
}

void add_force_at_position(bodies::Bodies &bodies, RigidBody *body, Vec3f force, Vec3f world_position, bool wake) {
    if (body->motion_type != motion_type::MOTION_DYNAMIC) return;
    motion_properties::add_force_at_position(body->motion_properties, force, world_position,
                                             vec3f::load(body->center_of_mass_position));
    if (wake) sleep::wake(bodies, body);
}

void add_impulse(bodies::Bodies &bodies, RigidBody *body, Vec3f impulse) {
    if (body->motion_type != motion_type::MOTION_DYNAMIC) return;
    if (body->motion_properties.inv_mass <= 0.0f) return;
    motion_properties::add_impulse(body->motion_properties, impulse);
    sleep::wake(bodies, body);
}

void add_impulse_at_position(bodies::Bodies &bodies, RigidBody *body, Vec3f impulse, Vec3f world_position) {
    if (body->motion_type != motion_type::MOTION_DYNAMIC) return;
    motion_properties::add_impulse_at_position(body->motion_properties, impulse, world_position,
                                               vec3f::load(body->center_of_mass_position), quatf::load(body->quaternion));
    sleep::wake(bodies, body);
}

void add_angular_impulse(bodies::Bodies &bodies, RigidBody *body, Vec3f impulse) {
    if (body->motion_type != motion_type::MOTION_DYNAMIC) return;
    motion_properties::add_angular_impulse(body->motion_properties, impulse, quatf::load(body->quaternion));
    sleep::wake(bodies, body);
}

void set_linear_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity) {
    if (body->motion_type == motion_type::MOTION_STATIC) return;
    motion_properties::set_linear_velocity(body->motion_properties, velocity);
    sleep::wake(bodies, body);
}

void add_linear_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity_delta) {
    if (body->motion_type == motion_type::MOTION_STATIC) return;
    motion_properties::add_linear_velocity(body->motion_properties, velocity_delta);
    sleep::wake(bodies, body);
}

void set_angular_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity) {
    if (body->motion_type == motion_type::MOTION_STATIC) return;
    motion_properties::set_angular_velocity(body->motion_properties, velocity);
    sleep::wake(bodies, body);
}

void add_angular_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity_delta) {
    if (body->motion_type == motion_type::MOTION_STATIC) return;
    motion_properties::add_angular_velocity(body->motion_properties, velocity_delta);
    sleep::wake(bodies, body);
}

void clear_forces(RigidBody *body) {
    body->motion_properties.force = {0.0f, 0.0f, 0.0f};
    body->motion_properties.torque = {0.0f, 0.0f, 0.0f};
}

void move_kinematic(RigidBody *body, Vec3f target_position, Quatf target_quaternion, float delta_time) {
    // calculate center of mass at target situation
    const Quatf target_rotation = target_quaternion;
    const Vec3f new_com = vec3f::add(target_position, vec3f::transform_quat(vec3f::load(body->shape->center_of_mass), target_rotation));

    // calculate delta position (from current com to new com)
    const Vec3f delta_position = vec3f::subtract(new_com, vec3f::load(body->center_of_mass_position));

    // calculate delta rotation: delta_rotation = target_rotation * current_rotation^-1
    const Quatf delta_rotation = quatf::multiply(target_rotation, quatf::conjugate(quatf::load(body->quaternion)));

    motion_properties::move_kinematic(body->motion_properties, delta_position, delta_rotation, delta_time);
}

Vec3f get_world_space_surface_normal(const RigidBody *body, sub_shape::SubShapeId sub_shape_id, Vec3f world_position) {
    // transform world position to local space (relative to shape origin)
    const Quatf rotation = quatf::load(body->quaternion);
    const Vec3f local_position = vec3f::transform_quat(vec3f::subtract(world_position, vec3f::load(body->position)), quatf::conjugate(rotation));

    // get normal in local space
    const Vec3f normal = shapes::get_shape_surface_normal(body->shape, local_position, sub_shape_id);

    // transform normal to world space (rotation only, no translation)
    return vec3f::transform_quat(normal, rotation);
}

Float3 &get_surface_normal(Float3 &out, const RigidBody *body, const Float3 &world_position, sub_shape::SubShapeId sub_shape_id) {
    return vec3f::store(out, get_world_space_surface_normal(body, sub_shape_id, vec3f::load(world_position)));
}

shapes::GetLeafShapeResult create_get_leaf_shape_result() { return {nullptr, sub_shape::EMPTY_SUB_SHAPE_ID}; }

void get_leaf_shape(shapes::GetLeafShapeResult &out, const shapes::Shape *root_shape, sub_shape::SubShapeId sub_shape_id) {
    shapes::get_leaf_shape(out, root_shape, sub_shape_id);
}

shapes::GetSubShapeTransformedShapeResult create_get_sub_shape_transformed_shape_result() {
    return {nullptr, vec3f::zero(), quatf::identity(), vec3f::zero(), sub_shape::EMPTY_SUB_SHAPE_ID};
}

void get_sub_shape_transformed_shape(shapes::GetSubShapeTransformedShapeResult &out, const shapes::Shape *root_shape,
                                     sub_shape::SubShapeId sub_shape_id, Vec3f position, Quatf rotation,
                                     Vec3f scale) {
    // initialize output with input transforms
    out.position = position;
    out.rotation = rotation;
    out.scale = scale;

    // delegate to shape implementation
    shapes::get_sub_shape_transformed_shape(out, root_shape, sub_shape_id);
}

Vec3f get_velocity_at_point_com(const RigidBody *body, Vec3f point_relative_to_com) {
    if (body->motion_type != motion_type::MOTION_STATIC) {
        const motion_properties::MotionProperties &mp = body->motion_properties;

        // v_point = v_linear + w x r
        return vec3f::add(vec3f::load(mp.linear_velocity), vec3f::cross(vec3f::load(mp.angular_velocity), point_relative_to_com));
    }

    return vec3f::zero();
}

Vec3f get_velocity_at_point(const RigidBody *body, Vec3f world_point) {
    return get_velocity_at_point_com(body, vec3f::subtract(world_point, vec3f::load(body->center_of_mass_position)));
}

bool bodies_share_constraint(const RigidBody *body_a, const RigidBody *body_b) {
    // fast path: if either has no constraints, they can't be connected
    if (body_a->constraint_ids.length == 0 || body_b->constraint_ids.length == 0) {
        return false;
    }

    // iterate the shorter list and check if any id exists in the longer list
    const bool a_shorter = body_a->constraint_ids.length <= body_b->constraint_ids.length;
    const collections::Vec<constraint_id::ConstraintId> &shorter = a_shorter ? body_a->constraint_ids : body_b->constraint_ids;
    const collections::Vec<constraint_id::ConstraintId> &longer = a_shorter ? body_b->constraint_ids : body_a->constraint_ids;

    for (uint32_t i = 0; i < shorter.length; i++) {
        for (uint32_t j = 0; j < longer.length; j++) {
            if (shorter[i] == longer[j]) return true;
        }
    }
    return false;
}

}  // namespace rigid_body

TUMBLE_NAMESPACE_END
