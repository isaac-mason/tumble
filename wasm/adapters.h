#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/dof.h"
#include "tumble/physics/body/mass_properties.h"
#include "tumble/physics/character/kcc.h"
#include "tumble/physics/character/kcc_policy.h"
#include "tumble/physics/constraints/cone_constraint.h"
#include "tumble/physics/contact_pairs.h"
#include "tumble/physics/constraints/contact_settings.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/constraints/fixed_constraint.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/constraints/six_dof_constraint.h"
#include "tumble/physics/constraints/slider_constraint.h"
#include "tumble/physics/constraints/swing_twist_constraint.h"
#include "tumble/physics/debug.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/manifold/manifold.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/offset_center_of_mass.h"
#include "tumble/physics/shapes/plane.h"
#include "tumble/physics/shapes/scaled.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/static_compound.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"
#include "tumble/physics/world.h"
#include "tumble/physics/world_settings.h"

// what js calls where the C++ api does not cross as it is: bodies by id (an id naming no live body sets
// ERROR_BAD_HANDLE and the call does nothing), settings holding pointers, the world's settings built call by call
namespace adapters {

namespace wasm {

/** the heap from heap_begin, and every built-in shape type registered */
bool init(uint32_t heap_begin);
uint32_t abi_version();
uint32_t schema_hash();
float simd_check();
uint32_t error();
void error_clear();
void *alloc_bytes(uint32_t bytes);
void free_bytes(void *block, uint32_t bytes);
uint32_t allocated_bytes();

}  // namespace wasm

namespace world_settings {

/** the defaults, on the heap, for the layer calls to fill */
tumble::world_settings::WorldSettings *create();
void destroy(tumble::world_settings::WorldSettings *settings);
uint32_t add_broadphase_layer(tumble::world_settings::WorldSettings *settings);
uint32_t add_object_layer(tumble::world_settings::WorldSettings *settings, uint32_t broadphase_layer);
void enable_collision(tumble::world_settings::WorldSettings *settings, uint32_t object_layer_a, uint32_t object_layer_b);

}  // namespace world_settings

namespace world {

tumble::world::World *create(const tumble::world_settings::WorldSettings *settings);
void destroy(tumble::world::World *world);
void step(tumble::world::World *world, float delta_time);

// the step's phases, as update.h has them
void step_begin(tumble::world::World *world, float delta_time);
void find_pairs(tumble::world::World *world);
void narrowphase(tumble::world::World *world);
void solve(tumble::world::World *world);
void step_end(tumble::world::World *world);
/** the bodies the last narrowphase woke, whose pairs are not found yet */
uint32_t woken(const tumble::world::World *world);

/** the surfaces material ids name, as friction and restitution pairs; false (error set) when the allocator fails */
bool set_materials(tumble::world::World *world, const float *friction_restitution, uint32_t count);

void get_gravity(tumble::Float3 &out, const tumble::world::World *world);
/** from the next step on */
void set_gravity(tumble::world::World *world, const tumble::Float3 &gravity);

}  // namespace world

/** events::PairEvent as js reads it: bodies by id, user data in halves */
struct PairEvent {
    tumble::body_id::BodyId body_a;
    tumble::body_id::BodyId body_b;
    uint32_t user_data_a_low;
    uint32_t user_data_a_high;
    uint32_t user_data_b_low;
    uint32_t user_data_b_high;
};

/** events::ContactEvent as js reads it */
struct ContactEvent {
    tumble::body_id::BodyId body_a;
    tumble::body_id::BodyId body_b;
    uint32_t user_data_a_low;
    uint32_t user_data_a_high;
    uint32_t user_data_b_low;
    uint32_t user_data_b_high;
    uint32_t sub_shape_a;
    uint32_t sub_shape_b;
    int32_t material_a;
    int32_t material_b;
    tumble::Float3 normal;
    float penetration;
    uint32_t num_points;
    tumble::Float3 points_a[tumble::manifold::MAX_CONTACT_POINTS];
    tumble::Float3 points_b[tumble::manifold::MAX_CONTACT_POINTS];
    tumble::contact_constraints::ContactSettings settings;
};

struct ContactRemovedEvent {
    tumble::body_id::BodyId body_a;
    tumble::body_id::BodyId body_b;
    uint32_t sub_shape_a;
    uint32_t sub_shape_b;
};

/**
 * a renderer's view of bodies: each body it draws in a slot, with the slot's pose in flat arrays here (positions xyz,
 * quaternions xyzw), refreshed by read
 */
struct Poses {
    tumble::world::World *world;
    /** by slot; INVALID_BODY_ID for a free slot */
    tumble::collections::Vec<tumble::body_id::BodyId> bodies;
    tumble::collections::Vec<uint32_t> free_slots;
    tumble::collections::Vec<float> positions;
    tumble::collections::Vec<float> quaternions;
    /** by slot: added since the last read, so it reads as changed */
    tumble::collections::Vec<uint8_t> fresh;
    /** the slots the last read changed */
    tumble::collections::Vec<uint32_t> changed;
};

namespace poses {

/** null when the allocator fails */
Poses *create(tumble::world::World *world);
void destroy(Poses *poses);
/** a slot for the body, its pose filled now; a free slot is reused. UINT32_MAX (error set) for a bad id or out of memory */
uint32_t add(Poses *poses, tumble::body_id::BodyId body);
/** the slot freed, its pose left as it was; false for a free or out of range slot */
bool remove(Poses *poses, uint32_t slot);
/**
 * every live slot's pose brought to its body's, comparing with what is there: exact whatever stepped or teleported
 * since. a slot whose body is gone keeps its pose. how many slots changed, listed in changed
 */
uint32_t read(Poses *poses);
/** slots in use or freed: the length of the arrays, over 3 and 4 */
uint32_t slots(const Poses *poses);
tumble::body_id::BodyId body(const Poses *poses, uint32_t slot);
const float *positions(const Poses *poses);
const float *quaternions(const Poses *poses);
const uint32_t *changed(const Poses *poses);

}  // namespace poses

// a step's events: a count, and a reader of one by index (false past the count)
namespace events {

uint32_t pairs(const tumble::world::World *world);
bool read_pair(PairEvent &out, const tumble::world::World *world, uint32_t index);
uint32_t contacts_added(const tumble::world::World *world);
bool read_contact_added(ContactEvent &out, const tumble::world::World *world, uint32_t index);
uint32_t contacts_persisted(const tumble::world::World *world);
bool read_contact_persisted(ContactEvent &out, const tumble::world::World *world, uint32_t index);
uint32_t contacts_removed(const tumble::world::World *world);
bool read_contact_removed(ContactRemovedEvent &out, const tumble::world::World *world, uint32_t index);
/** the bodies the step moved */
uint32_t moved(const tumble::world::World *world);
tumble::body_id::BodyId moved_body(const tumble::world::World *world, uint32_t index);

}  // namespace events

// answers to this step's events, by the index the reader took
namespace answers {

/** no narrowphase for pairs entry `index`, and its contacts go; false past the count */
bool reject_pair(tumble::world::World *world, uint32_t index);
/** contact event `index` (added, then persisted) solved with these settings; false past the count */
bool override_contact(tumble::world::World *world, uint32_t index, const tumble::contact_constraints::ContactSettings &settings);
/** contact event `index` dropped before the solve */
bool reject_contact(tumble::world::World *world, uint32_t index);

}  // namespace answers

/** RigidBodySettings as js gives it: the mass properties inline, used when has_mass_properties, for the pointer */
struct BodySettings {
    tumble::shapes::Shape *shape;
    uint32_t object_layer;
    uint32_t motion_type;
    uint32_t user_data_low;
    uint32_t user_data_high;
    tumble::Float3 position;
    tumble::Float4 quaternion;
    uint32_t motion_quality;
    uint32_t allowed_degrees_of_freedom;
    float gravity_factor;
    float linear_damping;
    float angular_damping;
    float max_linear_velocity;
    float max_angular_velocity;
    uint32_t num_velocity_steps_override;
    uint32_t num_position_steps_override;
    float friction;
    float restitution;
    uint32_t friction_combine_mode;
    uint32_t restitution_combine_mode;
    uint32_t collision_groups;
    uint32_t collision_mask;
    bool use_manifold_reduction;
    bool allow_sleeping;
    bool sensor;
    bool enhanced_internal_edge_removal;
    bool collide_kinematic_vs_non_dynamic;
    bool observes_contacts;
    float mass;
    /** mass_properties in place of the shape's, for shapes they cannot be derived from */
    bool has_mass_properties;
    tumble::mass_properties::MassProperties mass_properties;
};

namespace rigid_body {

/** the C++ defaults, for js to start a body's settings from */
void default_settings(BodySettings &out);
/** the new body's id, or INVALID_BODY_ID (error set) when the allocator fails */
tumble::body_id::BodyId create(tumble::world::World *world, const BodySettings &settings);
bool remove(tumble::world::World *world, tumble::body_id::BodyId id);
/** whether the id names a live body */
bool exists(tumble::world::World *world, tumble::body_id::BodyId id);

// readers: false (out unchanged) or 0 when the id names no body
bool get_position(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_quaternion(tumble::Float4 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_transform(tumble::Float3 &out_position, tumble::Float4 &out_quaternion, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_center_of_mass_position(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_linear_velocity(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_angular_velocity(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id);
bool get_velocity_at_point(tumble::Float3 &out, tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &point);
bool get_aabb(tumble::Box3f &out, tumble::world::World *world, tumble::body_id::BodyId id);
float get_mass(tumble::world::World *world, tumble::body_id::BodyId id);
uint32_t get_motion_type(tumble::world::World *world, tumble::body_id::BodyId id);
uint32_t get_object_layer(tumble::world::World *world, tumble::body_id::BodyId id);
uint32_t get_user_data_low(tumble::world::World *world, tumble::body_id::BodyId id);
uint32_t get_user_data_high(tumble::world::World *world, tumble::body_id::BodyId id);
tumble::shapes::Shape *get_shape(tumble::world::World *world, tumble::body_id::BodyId id);
bool is_sleeping(tumble::world::World *world, tumble::body_id::BodyId id);

// writers: false when the id names no body
bool set_position(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &position, bool wake);
bool set_quaternion(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float4 &quaternion, bool wake);
bool set_transform(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &position, const tumble::Float4 &quaternion,
                   bool wake);
bool set_linear_velocity(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &velocity);
bool set_angular_velocity(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &velocity);
bool add_linear_velocity(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &velocity);
bool add_angular_velocity(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &velocity);
bool add_force(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &force, bool wake);
bool add_torque(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &torque, bool wake);
bool add_force_at_position(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &force, const tumble::Float3 &position,
                           bool wake);
bool add_impulse(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &impulse);
bool add_impulse_at_position(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &impulse, const tumble::Float3 &position);
bool add_angular_impulse(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &impulse);
bool set_motion_type(tumble::world::World *world, tumble::body_id::BodyId id, uint32_t motion_type, bool wake);
bool set_object_layer(tumble::world::World *world, tumble::body_id::BodyId id, uint32_t object_layer);
bool set_shape(tumble::world::World *world, tumble::body_id::BodyId id, tumble::shapes::Shape *shape);
bool move_kinematic(tumble::world::World *world, tumble::body_id::BodyId id, const tumble::Float3 &position, const tumble::Float4 &quaternion,
                    float delta_time);
bool sleep(tumble::world::World *world, tumble::body_id::BodyId id);
bool wake(tumble::world::World *world, tumble::body_id::BodyId id);
bool set_user_data(tumble::world::World *world, tumble::body_id::BodyId id, uint32_t low, uint32_t high);
bool set_friction(tumble::world::World *world, tumble::body_id::BodyId id, float friction);
bool set_restitution(tumble::world::World *world, tumble::body_id::BodyId id, float restitution);

}  // namespace rigid_body

// what every shape has
namespace shape {

uint32_t get_type(const tumble::shapes::Shape *shape);
/** local space */
void get_local_bounds(tumble::Box3f &out, const tumble::shapes::Shape *shape);
void get_center_of_mass(tumble::Float3 &out, const tumble::shapes::Shape *shape);
float get_volume(const tumble::shapes::Shape *shape);
/** at the shape's density, about its centre of mass */
void get_mass_properties(tumble::mass_properties::MassProperties &out, const tumble::shapes::Shape *shape);

}  // namespace shape

// each shape's create, null when the allocator fails; the defaults are the C++'s

namespace box {

void default_settings(tumble::box::BoxShapeSettings &out);
tumble::shapes::Shape *create(const tumble::box::BoxShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace box

namespace sphere {

void default_settings(tumble::sphere::SphereShapeSettings &out);
tumble::shapes::Shape *create(const tumble::sphere::SphereShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace sphere

namespace capsule {

void default_settings(tumble::capsule::CapsuleShapeSettings &out);
tumble::shapes::Shape *create(const tumble::capsule::CapsuleShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace capsule

namespace cylinder {

void default_settings(tumble::cylinder::CylinderShapeSettings &out);
tumble::shapes::Shape *create(const tumble::cylinder::CylinderShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace cylinder

namespace plane {

void default_settings(tumble::plane::PlaneShapeSettings &out);
tumble::shapes::Shape *create(const tumble::plane::PlaneShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace plane

namespace empty {

tumble::shapes::Shape *create();
void destroy(tumble::shapes::Shape *shape);

}  // namespace empty

/** a hull build's outcome: the data, or null and why */
struct ConvexHullBuild {
    tumble::convex_hull::ConvexHullData *data;
    int32_t result;
};

namespace convex_hull_data {

void default_settings(tumble::convex_hull::ConvexHullBuildSettings &out);
/** false when the points make no hull or the allocator fails, out.result saying which */
bool build(ConvexHullBuild &out, const tumble::convex_hull::ConvexHullBuildSettings &settings);
void release(tumble::convex_hull::ConvexHullData *data);

}  // namespace convex_hull_data

namespace convex_hull {

void default_settings(tumble::convex_hull::ConvexHullShapeSettings &out);
tumble::shapes::Shape *create(const tumble::convex_hull::ConvexHullShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace convex_hull

namespace triangle_mesh_data {

void default_settings(tumble::triangle_mesh_builder::TriangleMeshBuilderSettings &out);
/** null when the allocator fails */
tumble::triangle_mesh_data::TriangleMeshData *build(const tumble::triangle_mesh_builder::TriangleMeshBuilderSettings &settings);
void release(tumble::triangle_mesh_data::TriangleMeshData *data);

}  // namespace triangle_mesh_data

namespace triangle_mesh {

void default_settings(tumble::triangle_mesh::TriangleMeshShapeSettings &out);
tumble::shapes::Shape *create(const tumble::triangle_mesh::TriangleMeshShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace triangle_mesh

namespace compound {

/** `children` is the address of `count` CompoundShapeChild values, read during the call */
tumble::shapes::Shape *create(const uint8_t *children, uint32_t count);
void destroy(tumble::shapes::Shape *shape);

}  // namespace compound

namespace static_compound {

uint32_t default_bvh_max_leaf_children();
tumble::shapes::Shape *create(const uint8_t *children, uint32_t count, uint32_t bvh_max_leaf_children);
void destroy(tumble::shapes::Shape *shape);

}  // namespace static_compound

namespace transformed {

void default_settings(tumble::transformed::TransformedShapeSettings &out);
tumble::shapes::Shape *create(const tumble::transformed::TransformedShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace transformed

namespace scaled {

void default_settings(tumble::scaled::ScaledShapeSettings &out);
tumble::shapes::Shape *create(const tumble::scaled::ScaledShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace scaled

namespace offset_center_of_mass {

void default_settings(tumble::offset_center_of_mass::OffsetCenterOfMassShapeSettings &out);
tumble::shapes::Shape *create(const tumble::offset_center_of_mass::OffsetCenterOfMassShapeSettings &settings);
void destroy(tumble::shapes::Shape *shape);

}  // namespace offset_center_of_mass

// constraints by id: an id naming no live constraint of the type sets ERROR_BAD_HANDLE and the call does nothing,
// returning false, or 0 from a reader of one number. each type's create takes its bodies by id, and gives
// INVALID_CONSTRAINT_ID (error set) for an id naming no body or when the allocator fails. the setters past the C++
// api's write the record's field, as C++ does

// what every constraint has
namespace constraint {

/** the type the id encodes, whether or not it names a live constraint */
uint32_t get_type(tumble::constraint_id::ConstraintId id);
/** whether the id names a live constraint */
bool exists(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
/** a disabled constraint is skipped by the solve and does not join its bodies' islands */
bool set_enabled(tumble::world::World *world, tumble::constraint_id::ConstraintId id, bool enabled);
bool is_enabled(tumble::world::World *world, tumble::constraint_id::ConstraintId id);

}  // namespace constraint

namespace point_constraint {

void default_settings(tumble::point_constraint::PointConstraintSettings &out);
tumble::constraint_id::ConstraintId create(tumble::world::World *world, tumble::body_id::BodyId body_a, tumble::body_id::BodyId body_b,
                                           const tumble::point_constraint::PointConstraintSettings &settings);
bool remove(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool get_total_lambda(tumble::Float3 &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);

}  // namespace point_constraint

namespace distance_constraint {

void default_settings(tumble::distance_constraint::DistanceConstraintSettings &out);
tumble::constraint_id::ConstraintId create(tumble::world::World *world, tumble::body_id::BodyId body_a, tumble::body_id::BodyId body_b,
                                           const tumble::distance_constraint::DistanceConstraintSettings &settings);
bool remove(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
float get_total_lambda(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool set_distance(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float min_distance, float max_distance);
bool set_limits_spring_settings(tumble::world::World *world, tumble::constraint_id::ConstraintId id,
                                const tumble::spring_settings::SpringSettings &settings);

}  // namespace distance_constraint

namespace hinge_constraint {

void default_settings(tumble::hinge_constraint::HingeConstraintSettings &out);
tumble::constraint_id::ConstraintId create(tumble::world::World *world, tumble::body_id::BodyId body_a, tumble::body_id::BodyId body_b,
                                           const tumble::hinge_constraint::HingeConstraintSettings &settings);
bool remove(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool set_limits(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float min, float max);
bool set_limits_spring_settings(tumble::world::World *world, tumble::constraint_id::ConstraintId id,
                                const tumble::spring_settings::SpringSettings &settings);
bool set_max_friction_torque(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float torque);
bool set_motor_settings(tumble::world::World *world, tumble::constraint_id::ConstraintId id, const tumble::motor_settings::MotorSettings &settings);
bool get_motor_settings(tumble::motor_settings::MotorSettings &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool set_motor_state(tumble::world::World *world, tumble::constraint_id::ConstraintId id, uint32_t state);
bool set_target_angular_velocity(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float velocity);
bool set_target_angle(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float angle);
float get_current_angle(tumble::world::World *world, tumble::constraint_id::ConstraintId id);

}  // namespace hinge_constraint

namespace slider_constraint {

void default_settings(tumble::slider_constraint::SliderConstraintSettings &out);
tumble::constraint_id::ConstraintId create(tumble::world::World *world, tumble::body_id::BodyId body_a, tumble::body_id::BodyId body_b,
                                           const tumble::slider_constraint::SliderConstraintSettings &settings);
bool remove(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
float get_current_position(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool set_limits(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float min, float max);
bool set_limits_spring_settings(tumble::world::World *world, tumble::constraint_id::ConstraintId id,
                                const tumble::spring_settings::SpringSettings &settings);
bool set_motor_settings(tumble::world::World *world, tumble::constraint_id::ConstraintId id, const tumble::motor_settings::MotorSettings &settings);
bool get_motor_settings(tumble::motor_settings::MotorSettings &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool set_motor_state(tumble::world::World *world, tumble::constraint_id::ConstraintId id, uint32_t state);
bool set_target_velocity(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float velocity);
bool set_target_position(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float position);
bool set_max_friction_force(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float force);
float get_max_friction_force(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool get_total_lambda_position(tumble::Vec2f &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);
float get_total_lambda_position_limits(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool get_total_lambda_rotation(tumble::Float3 &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);
float get_total_lambda_motor(tumble::world::World *world, tumble::constraint_id::ConstraintId id);

}  // namespace slider_constraint

namespace fixed_constraint {

void default_settings(tumble::fixed_constraint::FixedConstraintSettings &out);
tumble::constraint_id::ConstraintId create(tumble::world::World *world, tumble::body_id::BodyId body_a, tumble::body_id::BodyId body_b,
                                           const tumble::fixed_constraint::FixedConstraintSettings &settings);
bool remove(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool get_total_lambda_position(tumble::Float3 &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool get_total_lambda_rotation(tumble::Float3 &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);

}  // namespace fixed_constraint

namespace cone_constraint {

void default_settings(tumble::cone_constraint::ConeConstraintSettings &out);
tumble::constraint_id::ConstraintId create(tumble::world::World *world, tumble::body_id::BodyId body_a, tumble::body_id::BodyId body_b,
                                           const tumble::cone_constraint::ConeConstraintSettings &settings);
bool remove(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool set_half_cone_angle(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float half_cone_angle);
float get_half_cone_angle(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
float get_cos_half_cone_angle(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool get_total_lambda_position(tumble::Float3 &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);
float get_total_lambda_rotation(tumble::world::World *world, tumble::constraint_id::ConstraintId id);

}  // namespace cone_constraint

namespace swing_twist_constraint {

void default_settings(tumble::swing_twist_constraint::SwingTwistConstraintSettings &out);
tumble::constraint_id::ConstraintId create(tumble::world::World *world, tumble::body_id::BodyId body_a, tumble::body_id::BodyId body_b,
                                           const tumble::swing_twist_constraint::SwingTwistConstraintSettings &settings);
bool remove(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool set_normal_half_cone_angle(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float angle);
bool set_plane_half_cone_angle(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float angle);
bool set_twist_min_angle(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float angle);
bool set_twist_max_angle(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float angle);
bool set_max_friction_torque(tumble::world::World *world, tumble::constraint_id::ConstraintId id, float torque);
bool set_swing_motor_settings(tumble::world::World *world, tumble::constraint_id::ConstraintId id,
                              const tumble::motor_settings::MotorSettings &settings);
bool set_twist_motor_settings(tumble::world::World *world, tumble::constraint_id::ConstraintId id,
                              const tumble::motor_settings::MotorSettings &settings);
bool get_swing_motor_settings(tumble::motor_settings::MotorSettings &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool get_twist_motor_settings(tumble::motor_settings::MotorSettings &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool set_swing_motor_state(tumble::world::World *world, tumble::constraint_id::ConstraintId id, uint32_t state);
bool set_twist_motor_state(tumble::world::World *world, tumble::constraint_id::ConstraintId id, uint32_t state);
bool set_target_angular_velocity_cs(tumble::world::World *world, tumble::constraint_id::ConstraintId id, const tumble::Float3 &velocity);
bool set_target_orientation_cs(tumble::world::World *world, tumble::constraint_id::ConstraintId id, const tumble::Float4 &orientation);
bool set_target_orientation_bs(tumble::world::World *world, tumble::constraint_id::ConstraintId id, const tumble::Float4 &orientation);
bool get_rotation_in_constraint_space(tumble::Float4 &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool get_total_lambda_position(tumble::Float3 &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);
float get_total_lambda_twist(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
float get_total_lambda_swing_y(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
float get_total_lambda_swing_z(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool get_total_lambda_motor(tumble::Float3 &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id);

}  // namespace swing_twist_constraint

namespace six_dof_constraint {

void default_settings(tumble::six_dof_constraint::SixDOFConstraintSettings &out);
tumble::constraint_id::ConstraintId create(tumble::world::World *world, tumble::body_id::BodyId body_a, tumble::body_id::BodyId body_b,
                                           const tumble::six_dof_constraint::SixDOFConstraintSettings &settings);
bool remove(tumble::world::World *world, tumble::constraint_id::ConstraintId id);
bool set_translation_limits(tumble::world::World *world, tumble::constraint_id::ConstraintId id, const tumble::Float3 &limit_min,
                            const tumble::Float3 &limit_max);
bool set_rotation_limits(tumble::world::World *world, tumble::constraint_id::ConstraintId id, const tumble::Float3 &limit_min,
                         const tumble::Float3 &limit_max);
/** translation axes only */
bool set_limits_spring_settings(tumble::world::World *world, tumble::constraint_id::ConstraintId id, uint32_t axis,
                                const tumble::spring_settings::SpringSettings &settings);
bool set_motor_settings(tumble::world::World *world, tumble::constraint_id::ConstraintId id, uint32_t axis,
                        const tumble::motor_settings::MotorSettings &settings);
bool get_motor_settings(tumble::motor_settings::MotorSettings &out, tumble::world::World *world, tumble::constraint_id::ConstraintId id,
                        uint32_t axis);
bool set_motor_state(tumble::world::World *world, tumble::constraint_id::ConstraintId id, uint32_t axis, uint32_t state);
bool set_target_velocity_cs(tumble::world::World *world, tumble::constraint_id::ConstraintId id, const tumble::Float3 &velocity);
bool set_target_angular_velocity_cs(tumble::world::World *world, tumble::constraint_id::ConstraintId id, const tumble::Float3 &velocity);
bool set_target_position_cs(tumble::world::World *world, tumble::constraint_id::ConstraintId id, const tumble::Float3 &position);
bool set_target_orientation_cs(tumble::world::World *world, tumble::constraint_id::ConstraintId id, const tumble::Float4 &orientation);
bool set_max_friction(tumble::world::World *world, tumble::constraint_id::ConstraintId id, uint32_t axis, float friction);
bool make_free_axis(tumble::world::World *world, tumble::constraint_id::ConstraintId id, uint32_t axis);
bool make_fixed_axis(tumble::world::World *world, tumble::constraint_id::ConstraintId id, uint32_t axis);
bool set_limited_axis(tumble::world::World *world, tumble::constraint_id::ConstraintId id, uint32_t axis, float min, float max);

}  // namespace six_dof_constraint

// a query filter's layers, against a world's; its group and mask cross with each query
namespace filter {

/** every layer of the world's enabled; null when the allocator fails */
tumble::filter::Filter *create(tumble::world::World *world);
void destroy(tumble::filter::Filter *filter);
void enable_all_layers(tumble::filter::Filter *filter, tumble::world::World *world);
void disable_all_layers(tumble::filter::Filter *filter, tumble::world::World *world);
// false for a layer the world does not have
bool enable_object_layer(tumble::filter::Filter *filter, tumble::world::World *world, uint32_t object_layer);
bool disable_object_layer(tumble::filter::Filter *filter, tumble::world::World *world, uint32_t object_layer);
bool enable_broadphase_layer(tumble::filter::Filter *filter, tumble::world::World *world, uint32_t broadphase_layer);
bool disable_broadphase_layer(tumble::filter::Filter *filter, tumble::world::World *world, uint32_t broadphase_layer);
bool is_object_layer_enabled(const tumble::filter::Filter *filter, uint32_t object_layer);
bool is_broadphase_layer_enabled(const tumble::filter::Filter *filter, uint32_t broadphase_layer);
/** the layers the body's object layer collides with, and its group and mask, read back with the getters; false (error set) for a bad id */
bool set_from_body(tumble::filter::Filter *filter, tumble::world::World *world, tumble::body_id::BodyId body);
uint32_t get_collision_groups(const tumble::filter::Filter *filter);
uint32_t get_collision_mask(const tumble::filter::Filter *filter);
/** false (error set) when the allocator fails */
bool copy(tumble::filter::Filter *out, const tumble::filter::Filter *source);

}  // namespace filter

/** cast_shape::CastShapeHit as js reads it: no faces */
struct CastShapeHit {
    uint32_t status;
    float fraction;
    tumble::Float3 point_a;
    tumble::Float3 point_b;
    float penetration_depth;
    tumble::Float3 penetration_axis;
    tumble::Float3 normal;
    uint32_t sub_shape_id_a;
    uint32_t sub_shape_id_b;
    int32_t material_id_a;
    int32_t material_id_b;
    tumble::body_id::BodyId body_id_b;
};

/** cast_shape::CastShapeSettings as js gives it: no faces to collect */
struct CastShapeSettings {
    bool collide_with_backfaces_triangles;
    bool collide_with_backfaces_convex;
    float collision_tolerance;
    float penetration_tolerance;
    bool return_deepest_point;
    bool use_shrunken_shape_and_convex_radius;
    bool collide_only_with_active_edges;
    tumble::Float3 active_edge_movement_direction;
};

/** collide_shape::CollideShapeHit as js reads it: no faces */
struct CollideShapeHit {
    tumble::Float3 point_a;
    tumble::Float3 point_b;
    tumble::Float3 penetration_axis;
    float penetration;
    uint32_t sub_shape_id_a;
    uint32_t sub_shape_id_b;
    int32_t material_id_a;
    int32_t material_id_b;
    tumble::body_id::BodyId body_id_b;
};

/** collide_shape::CollideShapeSettings as js gives it: no faces to collect */
struct CollideShapeSettings {
    float max_separation_distance;
    float collision_tolerance;
    float penetration_tolerance;
    bool collide_with_backfaces;
    bool collide_only_with_active_edges;
    tumble::Float3 active_edge_movement_direction;
    float internal_edge_removal_vertex_tolerance_sq;
};

// each query into the adapters' own collector of its type, of `kind` (a collector::CollectorKind but CUSTOM) and
// starting at early_out_fraction, under the filter with the group and mask given (written into it): how many hits it
// holds, read by index (false past the count) until the next query of the type; 0 for a CUSTOM kind
namespace query {

/** a cast collector's early-out fraction before any hit */
float initial_early_out_fraction();
/** a collide collector's before any hit: no bound on the penetration */
float unbounded_early_out();

void default_cast_ray_settings(tumble::cast_ray::CastRaySettings &out);
uint32_t cast_ray(tumble::world::World *world, uint32_t kind, float early_out_fraction, const tumble::cast_ray::CastRaySettings &settings,
                  const tumble::Float3 &origin, const tumble::Float3 &direction, float length, tumble::filter::Filter *filter,
                  uint32_t collision_groups, uint32_t collision_mask);
bool read_cast_ray_hit(tumble::cast_ray::CastRayHit &out, uint32_t index);

void default_cast_shape_settings(CastShapeSettings &out);
uint32_t cast_shape(tumble::world::World *world, uint32_t kind, float early_out_fraction, const CastShapeSettings &settings,
                    const tumble::shapes::Shape *shape, const tumble::Float3 &position, const tumble::Float4 &quaternion,
                    const tumble::Float3 &scale, const tumble::Float3 &displacement, tumble::filter::Filter *filter,
                    uint32_t collision_groups, uint32_t collision_mask);
bool read_cast_shape_hit(CastShapeHit &out, uint32_t index);

void default_collide_shape_settings(CollideShapeSettings &out);
/** with internal_edge_removal, through the internal edge removing wrapper, flushed per body */
uint32_t collide_shape(tumble::world::World *world, uint32_t kind, float early_out_fraction, const CollideShapeSettings &settings,
                       const tumble::shapes::Shape *shape, const tumble::Float3 &position, const tumble::Float4 &quaternion,
                       const tumble::Float3 &scale, bool internal_edge_removal, tumble::filter::Filter *filter,
                       uint32_t collision_groups, uint32_t collision_mask);
bool read_collide_shape_hit(CollideShapeHit &out, uint32_t index);

void default_collide_point_settings(tumble::collide_point::CollidePointSettings &out);
uint32_t collide_point(tumble::world::World *world, uint32_t kind, const tumble::collide_point::CollidePointSettings &settings,
                       const tumble::Float3 &point, tumble::filter::Filter *filter, uint32_t collision_groups,
                       uint32_t collision_mask);
bool read_collide_point_hit(tumble::collide_point::CollidePointHit &out, uint32_t index);

}  // namespace query

namespace kcc_policy {

/** null when the allocator fails */
void default_body_policy(tumble::kcc_policy::BodyPolicy &out);
tumble::kcc_policy::Policy *create();
void destroy(tumble::kcc_policy::Policy *policy);
void get_defaults(tumble::kcc_policy::BodyPolicy &out, const tumble::kcc_policy::Policy *policy);
void set_defaults(tumble::kcc_policy::Policy *policy, const tumble::kcc_policy::BodyPolicy &body_policy);
/** false (error set) when the allocator fails */
bool set(tumble::kcc_policy::Policy *policy, tumble::body_id::BodyId body, const tumble::kcc_policy::BodyPolicy &body_policy);
bool remove(tumble::kcc_policy::Policy *policy, tumble::body_id::BodyId body);
/** the body's, or the defaults */
void get(tumble::kcc_policy::BodyPolicy &out, const tumble::kcc_policy::Policy *policy, tumble::body_id::BodyId body);

}  // namespace kcc_policy

/** kcc::KCCSettings as js gives it: the user data in halves */
struct KCCSettings {
    tumble::shapes::Shape *shape;
    tumble::Float3 shape_offset;
    float mass;
    float max_strength;
    tumble::Float3 up;
    float max_slope_angle;
    float predictive_contact_distance;
    float character_padding;
    uint32_t max_collision_iterations;
    uint32_t max_constraint_iterations;
    float collision_tolerance;
    float penetration_recovery_speed;
    float min_time_remaining;
    uint32_t max_num_hits;
    float hit_reduction_cos_max_angle;
    uint32_t back_face_mode;
    bool enhanced_internal_edge_removal;
    bool has_supporting_volume_plane;
    tumble::Plane3f supporting_volume_plane;
    uint32_t user_data_low;
    uint32_t user_data_high;
    bool observes_contacts;
    tumble::shapes::Shape *inner_rigid_body_shape;
    uint32_t inner_rigid_body_object_layer;
};

/** what js mirrors of a character after a call that moves it */
struct KCCState {
    tumble::Float3 position;
    tumble::Float4 quaternion;
    tumble::Float3 linear_velocity;
    tumble::Float3 up;
    uint32_t ground_state;
    tumble::Float3 ground_normal;
    tumble::Float3 ground_velocity;
    tumble::body_id::BodyId ground_body_id;
    uint32_t ground_sub_shape_id;
    tumble::Float3 ground_position;
    tumble::shapes::Shape *shape;
    tumble::body_id::BodyId inner_rigid_body_id;
    uint32_t contact_count;
    uint32_t contact_event_count;
};

/** kcc::ContactEvent as js reads it */
struct KCCContactEvent {
    uint32_t kind;
    tumble::body_id::BodyId body_id;
    uint32_t sub_shape_id;
    tumble::Float3 position;
    tumble::Float3 normal;
};

/** a kcc::CharacterContact as js reads one that is not a row: a sweep's */
struct KCCContact {
    tumble::Float3 position;
    tumble::Float3 linear_velocity;
    tumble::Float3 contact_normal;
    tumble::Float3 surface_normal;
    float distance;
    float fraction;
    tumble::body_id::BodyId body_id;
    uint32_t sub_shape_id;
    int32_t material_id;
    uint32_t motion_type;
    bool is_sensor;
};

/** a solve's results */
struct KCCSolve {
    tumble::Float3 velocity;
    tumble::Float3 displacement;
    float time_simulated;
};

/** what a character's settings can change after create: the shape has set_shape; the inner body's are fixed when added */
struct KCCTuning {
    tumble::Float3 shape_offset;
    float mass;
    float max_strength;
    /** normalized when set */
    tumble::Float3 up;
    float max_slope_angle;
    float predictive_contact_distance;
    float character_padding;
    uint32_t max_collision_iterations;
    uint32_t max_constraint_iterations;
    float collision_tolerance;
    float penetration_recovery_speed;
    float min_time_remaining;
    uint32_t max_num_hits;
    float hit_reduction_cos_max_angle;
    uint32_t back_face_mode;
    bool enhanced_internal_edge_removal;
    /** normal xyz and constant w in the character's frame; create's default, for none, is up at -1e10 */
    tumble::Float4 supporting_volume_plane;
};

// characters by handle, bodies by id, filters with the query's group and mask written into them; the character's
// contacts are rows in its memory at contacts(), contact_count of them, CharacterContact's layout under the KCC_CONTACT
// constants, until the next call that moves it
namespace kcc {

void default_settings(KCCSettings &out);
void default_update_settings(tumble::kcc::UpdateSettings &out);
/** null when the allocator fails */
tumble::kcc::KCC *create(const KCCSettings &settings, const tumble::Float3 &position, const tumble::Float4 &quaternion);
void destroy(tumble::kcc::KCC *character);
void add(tumble::world::World *world, tumble::kcc::KCC *character);
void remove(tumble::world::World *world, tumble::kcc::KCC *character);

void read_state(KCCState &out, const tumble::kcc::KCC *character);
void set_linear_velocity(tumble::kcc::KCC *character, const tumble::Float3 &velocity);
void get_tuning(KCCTuning &out, const tumble::kcc::KCC *character);
/** as create sets them: up normalized, the slope's cosine with its angle */
void set_tuning(tumble::kcc::KCC *character, const KCCTuning &tuning);
uint8_t *contacts(tumble::kcc::KCC *character);
bool read_contact_event(KCCContactEvent &out, const tumble::kcc::KCC *character, uint32_t index);

void set_position(tumble::world::World *world, tumble::kcc::KCC *character, const tumble::Float3 &position);
void set_quaternion(tumble::world::World *world, tumble::kcc::KCC *character, const tumble::Float4 &quaternion);
bool is_slope_too_steep(const tumble::kcc::KCC *character, const tumble::Float3 &normal);
void get_center_of_mass_transform(tumble::Float3 &out_position, tumble::Float4 &out_quaternion, const tumble::kcc::KCC *character);
bool has_collided_with(const tumble::kcc::KCC *character, tumble::body_id::BodyId body);
bool can_walk_stairs(const tumble::kcc::KCC *character, const tumble::Float3 &velocity);
void cancel_velocity_towards_steep_slopes(tumble::Float3 &out, const tumble::kcc::KCC *character, const tumble::Float3 &velocity);

void update_ground_velocity(tumble::world::World *world, tumble::kcc::KCC *character, const tumble::kcc_policy::Policy *policy);
void refresh_contacts(tumble::world::World *world, tumble::kcc::KCC *character, tumble::filter::Filter *filter, uint32_t collision_groups,
                      uint32_t collision_mask, const tumble::kcc_policy::Policy *policy);
bool set_shape(tumble::world::World *world, tumble::kcc::KCC *character, tumble::shapes::Shape *shape, tumble::filter::Filter *filter,
               uint32_t collision_groups, uint32_t collision_mask, const tumble::kcc_policy::Policy *policy, float max_penetration_depth);
void move(tumble::world::World *world, tumble::kcc::KCC *character, float delta_time, const tumble::Float3 &gravity,
          const tumble::kcc_policy::Policy *policy, tumble::filter::Filter *filter, uint32_t collision_groups, uint32_t collision_mask);
bool stick_to_floor(tumble::world::World *world, tumble::kcc::KCC *character, const tumble::Float3 &step_down, tumble::filter::Filter *filter,
                    uint32_t collision_groups, uint32_t collision_mask, const tumble::kcc_policy::Policy *policy);
bool walk_stairs(tumble::world::World *world, tumble::kcc::KCC *character, float delta_time, const tumble::Float3 &step_up,
                 const tumble::Float3 &step_forward, const tumble::Float3 &step_forward_test, const tumble::Float3 &step_down_extra,
                 tumble::filter::Filter *filter, uint32_t collision_groups, uint32_t collision_mask, const tumble::kcc_policy::Policy *policy);
void update(tumble::world::World *world, tumble::kcc::KCC *character, float delta_time, const tumble::Float3 &gravity,
            const tumble::kcc::UpdateSettings &settings, const tumble::kcc_policy::Policy *policy, tumble::filter::Filter *filter,
            uint32_t collision_groups, uint32_t collision_mask);

// the toolkit
void collide(tumble::world::World *world, tumble::kcc::KCC *character, const tumble::Float3 &position, const tumble::Float3 &movement_direction,
             tumble::filter::Filter *filter, uint32_t collision_groups, uint32_t collision_mask, const tumble::kcc_policy::Policy *policy);
void solve(KCCSolve &out, tumble::world::World *world, tumble::kcc::KCC *character, const tumble::Float3 &velocity, float delta_time,
           float time_remaining, const tumble::kcc_policy::Policy *policy);
bool cast(KCCContact &out, tumble::world::World *world, tumble::kcc::KCC *character, const tumble::Float3 &position,
          const tumble::Float3 &displacement, tumble::filter::Filter *filter, uint32_t collision_groups, uint32_t collision_mask,
          const tumble::kcc_policy::Policy *policy);
void update_ground(tumble::world::World *world, tumble::kcc::KCC *character, float delta_time, bool skip_contact_velocity_check,
                   const tumble::kcc_policy::Policy *policy);
void apply_weight(tumble::world::World *world, tumble::kcc::KCC *character, const tumble::Float3 &gravity, float delta_time);
void begin_contact_tracking(tumble::kcc::KCC *character);
void end_contact_tracking(tumble::kcc::KCC *character);

}  // namespace kcc

// wireframes into buffers js views in place: lines and triangles as x y z r g b per vertex, valid until the next draw
// or clear, a draw moving them when it grows them
namespace debug {

/** null when the allocator fails */
tumble::debug::DebugBuffers *create();
void destroy(tumble::debug::DebugBuffers *buffers);
void clear(tumble::debug::DebugBuffers *buffers);
const float *lines(const tumble::debug::DebugBuffers *buffers);
uint32_t line_count(const tumble::debug::DebugBuffers *buffers);
const float *triangles(const tumble::debug::DebugBuffers *buffers);
uint32_t triangle_count(const tumble::debug::DebugBuffers *buffers);

void default_bodies_options(tumble::debug::BodiesOptions &out);
void default_body_options(tumble::debug::BodyOptions &out);
void default_shape_options(tumble::debug::ShapeOptions &out);
void default_joints_options(tumble::debug::JointsOptions &out);

// each draw appends; false (error set) for a body id naming no body
void bodies(tumble::debug::DebugBuffers *out, const tumble::world::World *world, const tumble::debug::BodiesOptions &options);
bool body(tumble::debug::DebugBuffers *out, tumble::world::World *world, tumble::body_id::BodyId body,
          const tumble::debug::BodyOptions &options);
void line(tumble::debug::DebugBuffers *out, const tumble::Float3 &a, const tumble::Float3 &b, const tumble::Float3 &color);
void shape(tumble::debug::DebugBuffers *out, const tumble::shapes::Shape *shape, const tumble::Float3 &position, const tumble::Float4 &quaternion,
           const tumble::debug::ShapeOptions &options);
void contacts(tumble::debug::DebugBuffers *out, const tumble::world::World *world);
void character_contacts(tumble::debug::DebugBuffers *out, const tumble::kcc::KCC *character);
void contact_constraints(tumble::debug::DebugBuffers *out, const tumble::world::World *world);
void joints(tumble::debug::DebugBuffers *out, const tumble::world::World *world, const tumble::debug::JointsOptions &options);

}  // namespace debug

/** a contact_pairs::ContactPair as js reads it: user data in halves */
struct ContactPair {
    /** stable while the pair lasts; reused the frame after it is removed */
    uint32_t handle;
    tumble::body_id::BodyId body_a;
    uint32_t sub_shape_a;
    uint32_t user_data_a_low;
    uint32_t user_data_a_high;
    bool sensor_a;
    tumble::body_id::BodyId body_b;
    uint32_t sub_shape_b;
    uint32_t user_data_b_low;
    uint32_t user_data_b_high;
    bool sensor_b;
    tumble::Float3 point;
    tumble::Float3 normal;
    float penetration_depth;
    tumble::Float3 relative_velocity;
    bool added;
};

// the pairs of body sub shapes in contact that a body observing contacts is a side of, kept between frames
namespace contact_pairs {

void begin(tumble::world::World *world);
/** the last step's contact events from these indices of its added and persisted contacts on */
void record_world_contacts(tumble::world::World *world, uint32_t first_added, uint32_t first_persisted);
void record_character_contacts(tumble::world::World *world, const tumble::kcc::KCC *character);
void end(tumble::world::World *world);
/** this frame's pairs, in the order first recorded */
uint32_t active_count(const tumble::world::World *world);
bool read_active(ContactPair &out, const tumble::world::World *world, uint32_t index);
/** live last frame and not recorded in this one */
uint32_t removed_count(const tumble::world::World *world);
bool read_removed(ContactPair &out, const tumble::world::World *world, uint32_t index);

}  // namespace contact_pairs

}  // namespace adapters
