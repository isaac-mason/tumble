#pragma once

// crashcat's rigid-body.ts: a body in the world, its settings, and the calls that move, push and
// query it. a body lives in a slot of `bodies::Bodies`; the broadphase (`broadphase::add_body` /
// `remove_body` and the bounds refresh) is the caller's, so nothing here touches it.

#include <stdint.h>

#include "math/scalar.h"
#include "math/types.h"
#include "physics/body/bodies.h"
#include "physics/body/body_id.h"
#include "physics/body/mass_properties.h"
#include "physics/body/motion_properties.h"
#include "physics/body/motion_type.h"
#include "physics/body/sleep.h"
#include "physics/body/sub_shape.h"
#include "physics/constraints/combine_material.h"
#include "physics/constraints/constraint_id.h"
#include "physics/filter.h"
#include "physics/layers.h"
#include "physics/shapes/shapes.h"
#include "util/collections.h"

namespace rigid_body {

/** describes how mass properties are calculated for a rigid body */
enum MassPropertiesOverride : uint32_t {
    /** calculate both mass and inertia from shape density */
    CALCULATE_MASS_AND_INERTIA = 0,
    /** override mass, calculate and scale inertia from shape */
    CALCULATE_INERTIA = 1,
    /** use provided mass and inertia as-is */
    MASS_AND_INERTIA_PROVIDED = 2,
};

/** settings for creating a new rigid body */
struct RigidBodySettings {
    /** shape for the body. null creates a body with an empty shape */
    shapes::Shape *shape = nullptr;

    /** object layer the body belongs to */
    uint32_t object_layer;

    /** motion type (static, kinematic, dynamic) */
    motion_type::MotionType motion_type;

    /** the user's own value for the body, reported back with it in events */
    uint64_t user_data = 0;

    /** starting position */
    Float3 position = {0.0f, 0.0f, 0.0f};

    /** starting quaternion */
    Float4 quaternion = {0.0f, 0.0f, 0.0f, 1.0f};

    /** motion quality */
    motion_properties::MotionQuality motion_quality = motion_properties::MOTION_QUALITY_DISCRETE;

    /** allowed degrees of freedom */
    uint32_t allowed_degrees_of_freedom = 0b111111;

    /** factor for gravity applied to this body */
    float gravity_factor = 1.0f;
    float linear_damping = 0.05f;
    float angular_damping = 0.05f;
    float max_linear_velocity = 500.0f;
    float max_angular_velocity = 0.25f * scalar::PI * 60.0f;

    /**
     * used only when this body is dynamic and colliding. override for the number of solver velocity iterations, 0 uses
     * the world default. an island runs the max over all of its contacts and constraints
     */
    uint32_t num_velocity_steps_override = 0;
    /**
     * used only when this body is dynamic and colliding. override for the number of solver position iterations, 0 uses
     * the world default. an island runs the max over all of its contacts and constraints
     */
    uint32_t num_position_steps_override = 0;

    float friction = 0.2f;
    float restitution = 0.0f;

    /** the material combine mode for friction */
    combine_material::MaterialCombineMode friction_combine_mode = combine_material::GEOMETRIC_MEAN;
    /** the material combine mode for restitution */
    combine_material::MaterialCombineMode restitution_combine_mode = combine_material::MAX;

    /** groups this body is part of */
    uint32_t collision_groups = 0xffffffffu;
    /** groups this body collides with */
    uint32_t collision_mask = 0xffffffffu;

    /** whether to use manifold reduction for this body, see RigidBody::use_manifold_reduction */
    bool use_manifold_reduction = true;

    /** whether this body is allowed to sleep */
    bool allow_sleeping = true;

    bool sensor = false;

    /** enable enhanced internal edge removal to eliminate ghost collisions on internal edges */
    bool enhanced_internal_edge_removal = false;

    /** if true, and the motion type is kinematic, this body will collide with non-dynamic bodies */
    bool collide_kinematic_vs_non_dynamic = false;

    /** the world's contact pairs record the pairs this body is a side of */
    bool observes_contacts = false;

    /**
     * override mass only, negative for none. inertia will be calculated from the shape and scaled
     * to match this mass. ignored if mass_properties_override is provided.
     */
    float mass = -1.0f;

    /**
     * override mass properties for this body (both mass and inertia), null for none.
     * useful for triangle meshes and other complex shapes where mass properties cannot be derived
     * from the shape alone, or to use custom values. copied into the body, not referenced.
     * takes precedence over the mass field.
     */
    const mass_properties::MassProperties *mass_properties_override = nullptr;
};

/** physics body in a physics world */
struct RigidBody {
    /** unique body identifier */
    body_id::BodyId id;

    /** index from body id (index into Bodies::pool) */
    uint32_t index;

    /** sequence number from body id (for fast stale reference detection) */
    uint32_t sequence;

    /** the user's own value for the body */
    uint64_t user_data;

    /** the body's shape */
    shapes::Shape *shape;

    /** the body's world-space position at the shape origin, see center_of_mass_position for the center of mass position */
    Float3 position;

    /** the body's orientation */
    Float4 quaternion;

    /** world-space center of mass position */
    Float3 center_of_mass_position;

    /** the body's motion type */
    motion_type::MotionType motion_type;

    /** mass properties (mass, inertia tensor, etc.) */
    mass_properties::MassProperties mass_properties;

    /** how mass properties are calculated/overridden */
    MassPropertiesOverride mass_properties_override;

    /** motion properties (velocity, forces, damping, etc.) */
    motion_properties::MotionProperties motion_properties;

    /** coefficient of friction */
    float friction;

    /** coefficient of restitution (bounciness) */
    float restitution;

    /** how to combine friction when this body collides with another */
    combine_material::MaterialCombineMode friction_combine_mode;

    /** how to combine restitution when this body collides with another */
    combine_material::MaterialCombineMode restitution_combine_mode;

    /** collision group bitfield - "i am in these groups" */
    uint32_t collision_groups;

    /** collision mask bitfield - "i collide with these groups" */
    uint32_t collision_mask;

    /**
     * whether to use manifold reduction for this body.
     * if you require tracking exactly which sub shape ids are in contact, you can turn off manifold reduction. note that this comes at a performance cost.
     * manifold reduction by default will combine contacts with similar normals that come from different sub shape ids (e.g. different triangles in a mesh shape or different compound shapes).
     */
    bool use_manifold_reduction;

    /** whether this body is a sensor (detects collisions but does not respond physically) */
    bool sensor;

    /** enable enhanced internal edge removal to eliminate ghost collisions on internal edges */
    bool enhanced_internal_edge_removal;

    /** whether kinematic bodies can collide with static/other kinematic bodies */
    bool collide_kinematic_vs_non_dynamic;

    /** the world's contact pairs record the pairs this body is a side of */
    bool observes_contacts;

    /** read only: whether the body is currently sleeping */
    bool sleeping;

    /** world-space aabb */
    Box3f aabb;

    /** object layer this body is in */
    uint32_t object_layer;

    /** broadphase layer this body is in, -1 if none */
    int32_t broadphase_layer;

    /** the broadphase tree's handle for this body (a quad tree node and child), -1 if none */
    int32_t broadphase_node;

    /** true iff this body is in the broadphase move set, awaiting pair discovery */
    bool in_move_set;

    /**
     * head of the intrusive doubly-linked list of persistent broadphase pairs involving this body.
     * packed key: pair_edge_key(record_index, side). -1 for an empty list.
     */
    int32_t head_pair_key;

    /** island index this body belongs to (set during island building, -1 if not in an island) */
    int32_t island_index;

    /** active body index for island building (sequential index 0,1,2... for dynamic+kinematic bodies, INACTIVE_BODY_INDEX otherwise) */
    uint32_t active_index;

    /** ccd body index (index into the ccd bodies array, -1 if not using ccd this frame) */
    int32_t ccd_body_index;

    /** constraint ids referencing this body (for cleanup on body removal) */
    collections::Vec<constraint_id::ConstraintId> constraint_ids;
};

/**
 * creates a new body in a slot of `bodies`, with its mass, centre of mass and aabb from its shape,
 * and on the active list unless static. it is not in the broadphase: `broadphase::add_body` puts
 * it there. null (error set) when the pool cannot grow. `layers` holds `settings.object_layer`.
 */
RigidBody *create(bodies::Bodies &bodies, const layers::Layers &layers, const RigidBodySettings &settings);

/**
 * removes a body: off the active list and its slot freed, so its id stops resolving; the body
 * record stays in the slot for the next create to reuse. the caller removes it from the broadphase,
 * its constraints and its pairs first. false if the body was already removed.
 */
bool remove(bodies::Bodies &bodies, RigidBody *body);

/** the body `body_id` names, or null if it was removed (a stale id). */
RigidBody *get(const bodies::Bodies &bodies, body_id::BodyId body_id);

/** the world-space inverse inertia; zero for non-dynamic bodies. */
Float4x4 &get_inverse_inertia(Float4x4 &out, RigidBody *body);

/** the world-space center of mass from the transform and the shape. call whenever position, quaternion or shape changes. */
void update_center_of_mass_position(RigidBody *body);

/** the world-space aabb from the transform and the shape's aabb. call whenever position, quaternion or shape changes. */
void update_aabb(RigidBody *body);

/** updates mass, center of mass and aabb from the shape; call whenever the shape changes. the caller refreshes the broadphase. */
void update_shape(RigidBody *body);

/** sets the position and recomputes center of mass and aabb, optionally waking. the caller refreshes the broadphase. */
void set_position(bodies::Bodies &bodies, RigidBody *body, Vec3f position, bool wake);

/** sets the orientation and recomputes center of mass and aabb, optionally waking. the caller refreshes the broadphase. */
void set_quaternion(bodies::Bodies &bodies, RigidBody *body, Quatf quaternion, bool wake);

/** sets position and orientation in one go, optionally waking. the caller refreshes the broadphase. */
void set_transform(bodies::Bodies &bodies, RigidBody *body, Vec3f position, Quatf quaternion, bool wake);

/** sets the object layer. the caller reinserts the body in the broadphase and marks it moved. */
void set_object_layer(const layers::Layers &layers, RigidBody *body, uint32_t object_layer);

/**
 * sets the motion type and updates active body tracking; `wake` wakes a body that is not becoming
 * static. a body becoming static stops. the caller refreshes the broadphase.
 */
void set_motion_type(bodies::Bodies &bodies, RigidBody *body, motion_type::MotionType motion_type, bool wake);

/** adds a force at the center of mass, integrated over the next step. dynamic bodies only. */
void add_force(bodies::Bodies &bodies, RigidBody *body, Vec3f force, bool wake);

/** adds a torque, integrated over the next step. dynamic bodies only. */
void add_torque(bodies::Bodies &bodies, RigidBody *body, Vec3f torque, bool wake);

/** adds a force at a world-space position: the force, and the torque r x f about the center of mass. dynamic bodies only. */
void add_force_at_position(bodies::Bodies &bodies, RigidBody *body, Vec3f force, Vec3f world_position, bool wake);

/** applies an impulse at the center of mass: dv = impulse / mass. dynamic bodies only, always wakes. */
void add_impulse(bodies::Bodies &bodies, RigidBody *body, Vec3f impulse);

/** applies an impulse at a world-space position, changing linear and angular velocity. dynamic bodies only, always wakes. */
void add_impulse_at_position(bodies::Bodies &bodies, RigidBody *body, Vec3f impulse, Vec3f world_position);

/** applies an angular impulse: dw = I^-1 impulse. dynamic bodies only, always wakes. */
void add_angular_impulse(bodies::Bodies &bodies, RigidBody *body, Vec3f impulse);

/** sets the linear velocity. always wakes. */
void set_linear_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity);

/** adds to the linear velocity. always wakes. */
void add_linear_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity_delta);

/** sets the angular velocity. always wakes. */
void set_angular_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity);

/** adds to the angular velocity. always wakes. */
void add_angular_velocity(bodies::Bodies &bodies, RigidBody *body, Vec3f velocity_delta);

/** clears the accumulated force and torque. */
void clear_forces(RigidBody *body);

/** sets the velocities of a kinematic body so it reaches the target position and rotation in `delta_time`. */
void move_kinematic(RigidBody *body, Vec3f target_position, Quatf target_quaternion, float delta_time);

/** the surface normal of the sub shape at a world position, world space, as jolt's Body::GetWorldSpaceSurfaceNormal */
Vec3f get_world_space_surface_normal(const RigidBody *body, sub_shape::SubShapeId sub_shape_id, Vec3f world_position);

/**
 * the surface normal at a world-space position on the body's shape.
 * the query is performed relative to the shape's local coordinate system (shape origin, not center of mass).
 */
Float3 &get_surface_normal(Float3 &out, const RigidBody *body, const Float3 &world_position, sub_shape::SubShapeId sub_shape_id);

shapes::GetLeafShapeResult create_get_leaf_shape_result();

/** navigate to a leaf shape following the sub shape id hierarchy. */
void get_leaf_shape(shapes::GetLeafShapeResult &out, const shapes::Shape *root_shape, sub_shape::SubShapeId sub_shape_id);

shapes::GetSubShapeTransformedShapeResult create_get_sub_shape_transformed_shape_result();

/** the transformed shape for a sub shape id path, starting from the given transform. */
void get_sub_shape_transformed_shape(shapes::GetSubShapeTransformedShapeResult &out, const shapes::Shape *root_shape,
                                     sub_shape::SubShapeId sub_shape_id, Vec3f position, Quatf rotation,
                                     Vec3f scale);

/** velocity of a point relative to the center of mass; zero for static bodies. */
Vec3f get_velocity_at_point_com(const RigidBody *body, Vec3f point_relative_to_com);

/** velocity of a point in world space; zero for static bodies. */
Vec3f get_velocity_at_point(const RigidBody *body, Vec3f world_point);

/** true if two bodies share a constraint (are connected) */
bool bodies_share_constraint(const RigidBody *body_a, const RigidBody *body_b);

/** see sleep::sleep */
inline void sleep(bodies::Bodies &bodies, RigidBody *body) { sleep::sleep(bodies, body); }

/** see sleep::wake */
inline void wake(bodies::Bodies &bodies, RigidBody *body) { sleep::wake(bodies, body); }

}  // namespace rigid_body

namespace filter {

/** the layers that collide with the body's object layer, and its groups and mask. */
inline void set_from_body(Filter &filter, const layers::Layers &layers, const rigid_body::RigidBody *body) {
    set_from_layer(filter, layers, body->object_layer, body->collision_groups, body->collision_mask);
}

}  // namespace filter
