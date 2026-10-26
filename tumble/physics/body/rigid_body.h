#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/dof.h"
#include "tumble/physics/body/mass_properties.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/sleep.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/constraints/combine_material.h"
#include "tumble/physics/constraints/constraint_id.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/layers.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct World;

struct RigidBodySettings {
    /** null makes a body with an empty shape */
    Shape *shape = nullptr;

    uint32_t object_layer;

    MotionType motion_type;

    /** the user's own value for the body, reported back with it in events */
    uint64_t user_data = 0;

    Float3 position = {0.0f, 0.0f, 0.0f};

    Float4 quaternion = {0.0f, 0.0f, 0.0f, 1.0f};

    MotionQuality motion_quality = MotionQuality::DISCRETE;

    /** a dof:: mask: dof::DOF_ALL, or the axes it may move and turn about */
    uint32_t allowed_degrees_of_freedom = dof::DOF_ALL;

    /** scales gravity */
    float gravity_factor = 1.0f;
    float linear_damping = 0.05f;
    float angular_damping = 0.05f;
    float max_linear_velocity = 500.0f;
    float max_angular_velocity = 0.25f * scalar::PI * 60.0f;

    /** for a dynamic body in contact; 0 takes the world's. an island runs the most of its contacts and constraints */
    uint32_t num_velocity_steps_override = 0;
    /** for a dynamic body in contact; 0 takes the world's. an island runs the most of its contacts and constraints */
    uint32_t num_position_steps_override = 0;

    float friction = 0.2f;
    float restitution = 0.0f;

    MaterialCombineMode friction_combine_mode = MaterialCombineMode::GEOMETRIC_MEAN;
    MaterialCombineMode restitution_combine_mode = MaterialCombineMode::MAX;

    /** the groups it is in */
    uint32_t collision_groups = 0xffffffffu;
    /** the groups it collides with */
    uint32_t collision_mask = 0xffffffffu;

    /** see RigidBody::use_manifold_reduction */
    bool use_manifold_reduction = true;

    bool allow_sleeping = true;

    bool sensor = false;

    /** no ghost collisions on internal edges */
    bool enhanced_internal_edge_removal = false;

    /** a kinematic body that also collides with static and kinematic bodies */
    bool collide_kinematic_vs_non_dynamic = false;

    /** the world's contact pairs record the pairs this body is a side of */
    bool observes_contacts = false;

    /** the mass alone, negative for none: the inertia from the shape scaled to it. ignored with mass_properties_override */
    float mass = -1.0f;

    /**
     * mass and inertia, null for none: for shapes they cannot be derived from, such as triangle meshes, or custom values.
     * copied, not referenced; wins over mass
     */
    const MassProperties *mass_properties_override = nullptr;
};

// a body in a world: everything that changes it takes the world, which keeps the broadphase, the pairs and the
// constraints in step with it

struct RigidBody {
    enum class MassPropertiesOverride : uint32_t {
        /** both from the shape's density */
        CALCULATE_MASS_AND_INERTIA = 0,
        /** the mass given, the inertia from the shape scaled to it */
        CALCULATE_INERTIA = 1,
        /** both given */
        MASS_AND_INERTIA_PROVIDED = 2,
    };

    BodyId id;

    /** the id's index: the slot in Bodies::pool */
    uint32_t index;

    /** the id's sequence */
    uint32_t sequence;

    uint64_t user_data;

    /** held; set by set_shape */
    Ref<Shape> shape;

    /** at the shape's origin, world space */
    Float3 position;

    Float4 quaternion;

    /** world space */
    Float3 center_of_mass_position;

    /**
     * the centre of mass its shape had when the body last took it, so set_shape with the same shape after the shape
     * changed in place keeps the constraints' anchors where they were on the body
     */
    Float3 shape_center_of_mass;

    MotionType motion_type;

    MassProperties mass_properties;

    MassPropertiesOverride mass_properties_override;

    MotionProperties motion_properties;

    float friction;

    float restitution;

    MaterialCombineMode friction_combine_mode;

    MaterialCombineMode restitution_combine_mode;

    /** the groups it is in */
    uint32_t collision_groups;

    /** the groups it collides with */
    uint32_t collision_mask;

    /**
     * contacts of different sub shapes (a mesh's triangles, a compound's children) with similar normals merge. off when
     * exactly which sub shapes touch matters, at a cost
     */
    bool use_manifold_reduction;

    /** detects collisions without responding to them */
    bool sensor;

    /** no ghost collisions on internal edges */
    bool enhanced_internal_edge_removal;

    /** a kinematic body that also collides with static and kinematic bodies */
    bool collide_kinematic_vs_non_dynamic;

    /** the world's contact pairs record the pairs this body is a side of */
    bool observes_contacts;

    /** read only: shapes::caches_pairs of its shape, worked out as the shape is set */
    bool caches_pairs;

    /** world space */
    Box3f aabb;

    uint32_t object_layer;

    /** -1 if none */
    int32_t broadphase_layer;

    /** its place in the broadphase tree (a node and child), -1 if none */
    int32_t broadphase_node;

    /** in the broadphase move set, awaiting pair discovery */
    bool in_move_set;

    /** the head of its list of persistent pairs, a pairs::pair_edge_key; -1 for none */
    int32_t head_pair_key;

    /** set by island building, -1 when in none */
    int32_t island_index;

    /** its place in the active list (dynamic and kinematic bodies); INACTIVE_BODY_INDEX otherwise */
    uint32_t active_index;

    /** its place in the step's ccd bodies, -1 when it has none */
    int32_t ccd_body_index;

    /** the constraints that reference it, removed with it */
    collections::Array<ConstraintId> constraint_ids;
};

namespace rigid_body {

/** on the active list: a dynamic or kinematic body the step simulates */
inline bool is_active(const RigidBody &body) { return body.active_index != sleep::INACTIVE_BODY_INDEX; }

/** a dynamic or kinematic body off the active list, out of the simulation until something activates it */
inline bool is_sleeping(const RigidBody &body) { return body.motion_type != MotionType::STATIC && !is_active(body); }

/**
 * a new body in the world: its mass, centre of mass and aabb from its shape, in the broadphase, on the active list
 * unless static, and found by the next find_pairs. null (Error::QUOTA) when the pool is full
 */
RigidBody *create(World &world, const RigidBodySettings &settings);

/**
 * out of the world: its constraints destroyed, its pairs and contacts dropped (their removal events come with the next
 * step), out of the broadphase, its id no longer resolving. the record stays for the next create to reuse. false if
 * it was already removed
 */
bool remove(World &world, RigidBody *body);

/** null for a removed body's id */
RigidBody *get(const World &world, BodyId body_id);

/** world space; zero for a body that is not dynamic */
Float4x4 &get_inverse_inertia(Float4x4 &out, RigidBody *body);

/** from the transform and the shape */
void update_center_of_mass_position(RigidBody *body);

/** from the transform and the shape */
void update_aabb(RigidBody *body);

/**
 * the body takes the shape: its mass, centre of mass and bounds from it, its cached contacts dropped and its constraints'
 * anchors kept where they were on it. a new shape is swapped in; the body's own shape again refreshes the body after the
 * shape changed in place (its type's update)
 */
void set_shape(World &world, RigidBody *body, Shape *shape, bool wake);

void set_position(World &world, RigidBody *body, Vec3f position, bool wake);

void set_quaternion(World &world, RigidBody *body, Quatf quaternion, bool wake);

void set_transform(World &world, RigidBody *body, Vec3f position, Quatf quaternion, bool wake);

/** the body moves to the layer's broadphase layer, and finds its pairs again */
void set_object_layer(World &world, RigidBody *body, uint32_t object_layer);

/** the active list follows; `wake` wakes one not becoming static, and one becoming static stops */
void set_motion_type(World &world, RigidBody *body, MotionType motion_type, bool wake);

/** at the centre of mass, integrated over the next step; dynamic bodies only */
void add_force(World &world, RigidBody *body, Vec3f force, bool wake);

/** integrated over the next step; dynamic bodies only */
void add_torque(World &world, RigidBody *body, Vec3f torque, bool wake);

/** the force, and the torque r x f about the centre of mass; dynamic bodies only */
void add_force_at_position(World &world, RigidBody *body, Vec3f force, Vec3f world_position, bool wake);

/** at the centre of mass, dv = impulse / mass; dynamic bodies only, always wakes */
void add_impulse(World &world, RigidBody *body, Vec3f impulse);

/** dynamic bodies only, always wakes */
void add_impulse_at_position(World &world, RigidBody *body, Vec3f impulse, Vec3f world_position);

/** dw = I^-1 impulse; dynamic bodies only, always wakes */
void add_angular_impulse(World &world, RigidBody *body, Vec3f impulse);

/** always wakes */
void set_linear_velocity(World &world, RigidBody *body, Vec3f velocity);

/** always wakes */
void add_linear_velocity(World &world, RigidBody *body, Vec3f velocity_delta);

/** always wakes */
void set_angular_velocity(World &world, RigidBody *body, Vec3f velocity);

/** always wakes */
void add_angular_velocity(World &world, RigidBody *body, Vec3f velocity_delta);

void clear_forces(RigidBody *body);

/** the velocities that bring a kinematic body to the target in `delta_time`, and the body woken so the next step moves it */
void move_kinematic(World &world, RigidBody *body, Vec3f target_position, Quatf target_quaternion, float delta_time);

/** world space */
Vec3f get_world_space_surface_normal(const RigidBody *body, SubShapeId sub_shape_id, Vec3f world_position);

/** the material id of the part of the body's shape a sub shape id names, as a hit or a contact gives it; -1 for none */
int32_t get_material(const RigidBody *body, SubShapeId sub_shape_id);

/** queried relative to the shape's origin, not the centre of mass */
Float3 &get_surface_normal(Float3 &out, const RigidBody *body, const Float3 &world_position, SubShapeId sub_shape_id);

GetLeafShapeResult create_get_leaf_shape_result();

/** the leaf the sub shape id's path leads to */
void get_leaf_shape(GetLeafShapeResult &out, const Shape *root_shape, SubShapeId sub_shape_id);

GetSubShapeTransformedShapeResult create_get_sub_shape_transformed_shape_result();

/** the shape the sub shape id's path leads to, transformed from the given pose */
void get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *root_shape,
                                     SubShapeId sub_shape_id, Vec3f position, Quatf rotation,
                                     Vec3f scale);

/** of a point relative to the centre of mass; zero for a static body */
Vec3f get_velocity_at_point_com(const RigidBody *body, Vec3f point_relative_to_com);

/** of a world space point; zero for a static body */
Vec3f get_velocity_at_point(const RigidBody *body, Vec3f world_point);

bool bodies_share_constraint(const RigidBody *body_a, const RigidBody *body_b);

/** a dynamic body out of the simulation until something wakes it */
void sleep(World &world, RigidBody *body);

void wake(World &world, RigidBody *body);

}  // namespace rigid_body

namespace filter {

/** the layers that collide with the body's object layer, and its groups and mask */
inline void set_from_body(Filter &filter, const Layers &layers, const RigidBody *body) {
    set_from_layer(filter, layers, body->object_layer, body->collision_groups, body->collision_mask);
}

}  // namespace filter

TUMBLE_NAMESPACE_END
