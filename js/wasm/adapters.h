#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/body_activation.h"
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
#include "tumble/physics/world.h"
#include "tumble/physics/world_settings.h"

// what js calls where the C++ api does not cross as it is: bodies by id (an id naming no live body sets
// Error::BAD_HANDLE and the call does nothing), settings holding pointers, the world's settings built call by call
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
tumble::WorldSettings *create();
void destroy(tumble::WorldSettings *settings);
uint32_t add_broadphase_layer(tumble::WorldSettings *settings);
uint32_t add_object_layer(tumble::WorldSettings *settings, uint32_t broadphase_layer);
void enable_collision(tumble::WorldSettings *settings, uint32_t object_layer_a, uint32_t object_layer_b);

}  // namespace world_settings

namespace world {

/** the world, its WorldState made and its recorder set as its activation listener */
tumble::World *create(const tumble::WorldSettings *settings);
void destroy(tumble::World *world);
void step(tumble::World *world, float delta_time);

// the step's phases, as update.h has them
void step_begin(tumble::World *world, float delta_time);
void find_pairs(tumble::World *world);
void narrowphase(tumble::World *world);
void solve(tumble::World *world);
void step_end(tumble::World *world);
/** the bodies the last narrowphase woke, whose pairs are not found yet */
uint32_t woken(const tumble::World *world);

/** the surfaces material ids name, as friction and restitution pairs */
void set_materials(tumble::World *world, const float *friction_restitution, uint32_t count);

void get_gravity(tumble::Float3 &out, const tumble::World *world);
/** from the next step on */
void set_gravity(tumble::World *world, const tumble::Float3 &gravity);

}  // namespace world

/** tumble::PairEvent as js reads it: bodies by id, user data in halves */
struct PairEvent {
    tumble::BodyId body_a;
    tumble::BodyId body_b;
    uint32_t user_data_a_low;
    uint32_t user_data_a_high;
    uint32_t user_data_b_low;
    uint32_t user_data_b_high;
};

/** tumble::ContactEvent as js reads it */
struct ContactEvent {
    tumble::BodyId body_a;
    tumble::BodyId body_b;
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
    tumble::ContactSettings settings;
};

struct ContactRemovedEvent {
    tumble::BodyId body_a;
    tumble::BodyId body_b;
    uint32_t sub_shape_a;
    uint32_t sub_shape_b;
};

/** one body of the active list as js reads it */
struct ActiveBodyRow {
    tumble::BodyId body;
    tumble::Float3 position;
    tumble::Float4 quaternion;
    tumble::Float3 center_of_mass_position;
    tumble::Float3 linear_velocity;
    tumble::Float3 angular_velocity;
};

/** BodyActivationEvent as js reads it */
struct ActivationEvent {
    tumble::BodyId body;
    uint32_t user_data_low;
    uint32_t user_data_high;
    /** 0 for a deactivation */
    uint32_t activated;
};

/**
 * what js keeps beside each world, made with it by world::create: a recorder as the world's activation listener, so
 * js hears every change to the active list after the step rather than during it, and the rows active_bodies writes
 */
struct WorldState {
    /** first, so the world's listener's context finds the state */
    tumble::BodyActivationRecorder recorder;
    /** a bit by body index for the bodies active_bodies writes a row for, so it writes them in index order */
    tumble::collections::Array<uint32_t> marks;
    /** in body index order, which keeps whatever js copies them into visited in the order it was made */
    tumble::collections::Array<ActiveBodyRow> rows;
};

namespace activation {

/** the changes to the active list recorded since the last clear, in order */
uint32_t events(const tumble::World *world);
bool read_event(ActivationEvent &out, const tumble::World *world, uint32_t index);
void clear(tumble::World *world);

}  // namespace activation

namespace active_bodies {

/** how many bodies are on the active list */
uint32_t count(const tumble::World *world);
/**
 * a row for every body on the active list, in body index order, at rows: how many. valid until the next read
 */
uint32_t read(tumble::World *world);
/** the first row, the rest ACTIVE_BODY_ROW_STRIDE bytes apart */
const uint32_t *rows(const tumble::World *world);

}  // namespace active_bodies

// a step's events: a count, and a reader of one by index (false past the count)
namespace events {

uint32_t pairs(const tumble::World *world);
bool read_pair(PairEvent &out, const tumble::World *world, uint32_t index);
uint32_t contacts_added(const tumble::World *world);
bool read_contact_added(ContactEvent &out, const tumble::World *world, uint32_t index);
uint32_t contacts_persisted(const tumble::World *world);
bool read_contact_persisted(ContactEvent &out, const tumble::World *world, uint32_t index);
uint32_t contacts_removed(const tumble::World *world);
bool read_contact_removed(ContactRemovedEvent &out, const tumble::World *world, uint32_t index);

}  // namespace events

// answers to this step's events, by the index the reader took
namespace answers {

/** no narrowphase for pairs entry `index`, and its contacts go; false past the count */
bool reject_pair(tumble::World *world, uint32_t index);
/** contact event `index` (added, then persisted) solved with these settings; false past the count */
bool override_contact(tumble::World *world, uint32_t index, const tumble::ContactSettings &settings);
/** contact event `index` dropped before the solve */
bool reject_contact(tumble::World *world, uint32_t index);

}  // namespace answers

/** RigidBodySettings as js gives it: the mass properties inline, used when has_mass_properties, for the pointer */
struct BodySettings {
    tumble::Shape *shape;
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
    tumble::MassProperties mass_properties;
};

namespace rigid_body {

/** the C++ defaults, for js to start a body's settings from */
void default_settings(BodySettings &out);
/** the new body's id, or INVALID_BODY_ID (Error::QUOTA) when the pool is full */
tumble::BodyId create(tumble::World *world, const BodySettings &settings);
bool remove(tumble::World *world, tumble::BodyId id);
/** whether the id names a live body */
bool exists(tumble::World *world, tumble::BodyId id);

// readers: false (out unchanged) or 0 when the id names no body
bool get_position(tumble::Float3 &out, tumble::World *world, tumble::BodyId id);
bool get_quaternion(tumble::Float4 &out, tumble::World *world, tumble::BodyId id);
bool get_transform(tumble::Float3 &out_position, tumble::Float4 &out_quaternion, tumble::World *world, tumble::BodyId id);
bool get_center_of_mass_position(tumble::Float3 &out, tumble::World *world, tumble::BodyId id);
bool get_linear_velocity(tumble::Float3 &out, tumble::World *world, tumble::BodyId id);
bool get_angular_velocity(tumble::Float3 &out, tumble::World *world, tumble::BodyId id);
bool get_velocity_at_point(tumble::Float3 &out, tumble::World *world, tumble::BodyId id, const tumble::Float3 &point);
bool get_aabb(tumble::Box3f &out, tumble::World *world, tumble::BodyId id);
float get_mass(tumble::World *world, tumble::BodyId id);
uint32_t get_motion_type(tumble::World *world, tumble::BodyId id);
uint32_t get_object_layer(tumble::World *world, tumble::BodyId id);
uint32_t get_user_data_low(tumble::World *world, tumble::BodyId id);
uint32_t get_user_data_high(tumble::World *world, tumble::BodyId id);
tumble::Shape *get_shape(tumble::World *world, tumble::BodyId id);
/** the material id of the part of the body's shape the sub shape id names; -1 for none or a body id naming no body */
int32_t get_material(tumble::World *world, tumble::BodyId id, uint32_t sub_shape_id);
bool is_sleeping(tumble::World *world, tumble::BodyId id);

// writers: false when the id names no body
bool set_position(tumble::World *world, tumble::BodyId id, const tumble::Float3 &position, bool wake);
bool set_quaternion(tumble::World *world, tumble::BodyId id, const tumble::Float4 &quaternion, bool wake);
bool set_transform(tumble::World *world, tumble::BodyId id, const tumble::Float3 &position, const tumble::Float4 &quaternion,
                   bool wake);
bool set_linear_velocity(tumble::World *world, tumble::BodyId id, const tumble::Float3 &velocity);
bool set_angular_velocity(tumble::World *world, tumble::BodyId id, const tumble::Float3 &velocity);
bool add_linear_velocity(tumble::World *world, tumble::BodyId id, const tumble::Float3 &velocity);
bool add_angular_velocity(tumble::World *world, tumble::BodyId id, const tumble::Float3 &velocity);
bool add_force(tumble::World *world, tumble::BodyId id, const tumble::Float3 &force, bool wake);
bool add_torque(tumble::World *world, tumble::BodyId id, const tumble::Float3 &torque, bool wake);
bool add_force_at_position(tumble::World *world, tumble::BodyId id, const tumble::Float3 &force, const tumble::Float3 &position,
                           bool wake);
bool add_impulse(tumble::World *world, tumble::BodyId id, const tumble::Float3 &impulse);
bool add_impulse_at_position(tumble::World *world, tumble::BodyId id, const tumble::Float3 &impulse, const tumble::Float3 &position);
bool add_angular_impulse(tumble::World *world, tumble::BodyId id, const tumble::Float3 &impulse);
bool set_motion_type(tumble::World *world, tumble::BodyId id, uint32_t motion_type, bool wake);
bool set_object_layer(tumble::World *world, tumble::BodyId id, uint32_t object_layer);
bool set_shape(tumble::World *world, tumble::BodyId id, tumble::Shape *shape, bool wake);
bool move_kinematic(tumble::World *world, tumble::BodyId id, const tumble::Float3 &position, const tumble::Float4 &quaternion,
                    float delta_time);
bool sleep(tumble::World *world, tumble::BodyId id);
bool wake(tumble::World *world, tumble::BodyId id);
bool set_user_data(tumble::World *world, tumble::BodyId id, uint32_t low, uint32_t high);
bool set_friction(tumble::World *world, tumble::BodyId id, float friction);
bool set_restitution(tumble::World *world, tumble::BodyId id, float restitution);

}  // namespace rigid_body

// what every shape has
namespace shape {

uint32_t get_type(const tumble::Shape *shape);
/** local space */
void get_local_bounds(tumble::Box3f &out, const tumble::Shape *shape);
void get_center_of_mass(tumble::Float3 &out, const tumble::Shape *shape);
float get_volume(const tumble::Shape *shape);
/** at the shape's density, about its centre of mass */
void get_mass_properties(tumble::MassProperties &out, const tumble::Shape *shape);
/** how many references hold the shape: bodies', characters', the shapes built over it, and js's own */
uint32_t get_ref_count(const tumble::Shape *shape);
/** js lets go of the shape it made; the last reference to go frees it */
void release(const tumble::Shape *shape);

}  // namespace shape

// each shape's create, held for js until shape::release; the defaults are the C++'s

namespace box {

void default_settings(tumble::BoxShapeSettings &out);
tumble::Shape *create(const tumble::BoxShapeSettings &settings);

/** the shape's fields from the settings, as create takes them, its bounds and volume after; bodies using it take it again with set_shape */
void update(tumble::Shape *shape, const tumble::BoxShapeSettings &settings);

}  // namespace box

namespace sphere {

void default_settings(tumble::SphereShapeSettings &out);
tumble::Shape *create(const tumble::SphereShapeSettings &settings);

/** the shape's fields from the settings, as create takes them, its bounds and volume after; bodies using it take it again with set_shape */
void update(tumble::Shape *shape, const tumble::SphereShapeSettings &settings);

}  // namespace sphere

namespace capsule {

void default_settings(tumble::CapsuleShapeSettings &out);
tumble::Shape *create(const tumble::CapsuleShapeSettings &settings);

/** the shape's fields from the settings, as create takes them, its bounds and volume after; bodies using it take it again with set_shape */
void update(tumble::Shape *shape, const tumble::CapsuleShapeSettings &settings);

}  // namespace capsule

namespace cylinder {

void default_settings(tumble::CylinderShapeSettings &out);
tumble::Shape *create(const tumble::CylinderShapeSettings &settings);

/** the shape's fields from the settings, as create takes them, its bounds and volume after; bodies using it take it again with set_shape */
void update(tumble::Shape *shape, const tumble::CylinderShapeSettings &settings);

}  // namespace cylinder

namespace plane {

void default_settings(tumble::PlaneShapeSettings &out);
tumble::Shape *create(const tumble::PlaneShapeSettings &settings);

}  // namespace plane

namespace empty {

tumble::Shape *create();

}  // namespace empty

/** a hull's creation: the shape, or null and why */
struct ConvexHullCreate {
    tumble::Shape *shape;
    int32_t result;
};

namespace convex_hull {

void default_settings(tumble::ConvexHullShapeSettings &out);
/** false when the points make no hull, out.result saying why */
bool create(ConvexHullCreate &out, const tumble::ConvexHullShapeSettings &settings);

}  // namespace convex_hull

namespace triangle_mesh {

void default_settings(tumble::TriangleMeshShapeSettings &out);
tumble::Shape *create(const tumble::TriangleMeshShapeSettings &settings);

}  // namespace triangle_mesh

namespace compound {

/** `children` is the address of `count` CompoundShapeSettings::Child values, read during the call */
tumble::Shape *create(const uint8_t *children, uint32_t count);

/**
 * the compound's children replaced by `count` CompoundShapeSettings::Child values at `children`, holding the new and letting go of
 * the old; bodies using it take it again with set_shape
 */
void set_children(tumble::Shape *shape, const uint8_t *children, uint32_t count);

}  // namespace compound

namespace static_compound {

uint32_t default_bvh_max_leaf_children();
tumble::Shape *create(const uint8_t *children, uint32_t count, uint32_t bvh_max_leaf_children);

}  // namespace static_compound

namespace transformed {

void default_settings(tumble::TransformedShapeSettings &out);
tumble::Shape *create(const tumble::TransformedShapeSettings &settings);

}  // namespace transformed

namespace scaled {

void default_settings(tumble::ScaledShapeSettings &out);
tumble::Shape *create(const tumble::ScaledShapeSettings &settings);

}  // namespace scaled

namespace offset_center_of_mass {

void default_settings(tumble::OffsetCenterOfMassShapeSettings &out);
tumble::Shape *create(const tumble::OffsetCenterOfMassShapeSettings &settings);

}  // namespace offset_center_of_mass

// constraints by id: an id naming no live constraint of the type sets Error::BAD_HANDLE and the call does nothing,
// returning false, or 0 from a reader of one number. each type's create takes its bodies by id, and gives
// INVALID_CONSTRAINT_ID (error set) for an id naming no body or when the pool is at its most. the setters past the C++
// api's write the record's field, as C++ does

// what every constraint has
namespace constraint {

/** the type the id encodes, whether or not it names a live constraint */
uint32_t get_type(tumble::ConstraintId id);
/** whether the id names a live constraint */
bool exists(tumble::World *world, tumble::ConstraintId id);
/** a disabled constraint is skipped by the solve and does not join its bodies' islands */
bool set_enabled(tumble::World *world, tumble::ConstraintId id, bool enabled);
bool is_enabled(tumble::World *world, tumble::ConstraintId id);
/** any type's; false, with the error set, when the id names no constraint */
bool remove(tumble::World *world, tumble::ConstraintId id);

}  // namespace constraint

namespace point_constraint {

void default_settings(tumble::PointConstraintSettings &out);
tumble::ConstraintId create(tumble::World *world, tumble::BodyId body_a, tumble::BodyId body_b,
                                           const tumble::PointConstraintSettings &settings);
bool get_total_lambda(tumble::Float3 &out, tumble::World *world, tumble::ConstraintId id);

}  // namespace point_constraint

namespace distance_constraint {

void default_settings(tumble::DistanceConstraintSettings &out);
tumble::ConstraintId create(tumble::World *world, tumble::BodyId body_a, tumble::BodyId body_b,
                                           const tumble::DistanceConstraintSettings &settings);
float get_total_lambda(tumble::World *world, tumble::ConstraintId id);
bool set_distance(tumble::World *world, tumble::ConstraintId id, float min_distance, float max_distance);
bool set_limits_spring_settings(tumble::World *world, tumble::ConstraintId id,
                                const tumble::SpringSettings &settings);

}  // namespace distance_constraint

namespace hinge_constraint {

void default_settings(tumble::HingeConstraintSettings &out);
tumble::ConstraintId create(tumble::World *world, tumble::BodyId body_a, tumble::BodyId body_b,
                                           const tumble::HingeConstraintSettings &settings);
bool set_limits(tumble::World *world, tumble::ConstraintId id, float min, float max);
bool set_limits_spring_settings(tumble::World *world, tumble::ConstraintId id,
                                const tumble::SpringSettings &settings);
bool set_max_friction_torque(tumble::World *world, tumble::ConstraintId id, float torque);
bool set_motor_settings(tumble::World *world, tumble::ConstraintId id, const tumble::MotorSettings &settings);
bool get_motor_settings(tumble::MotorSettings &out, tumble::World *world, tumble::ConstraintId id);
bool set_motor_state(tumble::World *world, tumble::ConstraintId id, uint32_t state);
bool set_target_angular_velocity(tumble::World *world, tumble::ConstraintId id, float velocity);
bool set_target_angle(tumble::World *world, tumble::ConstraintId id, float angle);
float get_current_angle(tumble::World *world, tumble::ConstraintId id);

}  // namespace hinge_constraint

namespace slider_constraint {

void default_settings(tumble::SliderConstraintSettings &out);
tumble::ConstraintId create(tumble::World *world, tumble::BodyId body_a, tumble::BodyId body_b,
                                           const tumble::SliderConstraintSettings &settings);
float get_current_position(tumble::World *world, tumble::ConstraintId id);
bool set_limits(tumble::World *world, tumble::ConstraintId id, float min, float max);
bool set_limits_spring_settings(tumble::World *world, tumble::ConstraintId id,
                                const tumble::SpringSettings &settings);
bool set_motor_settings(tumble::World *world, tumble::ConstraintId id, const tumble::MotorSettings &settings);
bool get_motor_settings(tumble::MotorSettings &out, tumble::World *world, tumble::ConstraintId id);
bool set_motor_state(tumble::World *world, tumble::ConstraintId id, uint32_t state);
bool set_target_velocity(tumble::World *world, tumble::ConstraintId id, float velocity);
bool set_target_position(tumble::World *world, tumble::ConstraintId id, float position);
bool set_max_friction_force(tumble::World *world, tumble::ConstraintId id, float force);
float get_max_friction_force(tumble::World *world, tumble::ConstraintId id);
bool get_total_lambda_position(tumble::Vec2f &out, tumble::World *world, tumble::ConstraintId id);
float get_total_lambda_position_limits(tumble::World *world, tumble::ConstraintId id);
bool get_total_lambda_rotation(tumble::Float3 &out, tumble::World *world, tumble::ConstraintId id);
float get_total_lambda_motor(tumble::World *world, tumble::ConstraintId id);

}  // namespace slider_constraint

namespace fixed_constraint {

void default_settings(tumble::FixedConstraintSettings &out);
tumble::ConstraintId create(tumble::World *world, tumble::BodyId body_a, tumble::BodyId body_b,
                                           const tumble::FixedConstraintSettings &settings);
bool get_total_lambda_position(tumble::Float3 &out, tumble::World *world, tumble::ConstraintId id);
bool get_total_lambda_rotation(tumble::Float3 &out, tumble::World *world, tumble::ConstraintId id);

}  // namespace fixed_constraint

namespace cone_constraint {

void default_settings(tumble::ConeConstraintSettings &out);
tumble::ConstraintId create(tumble::World *world, tumble::BodyId body_a, tumble::BodyId body_b,
                                           const tumble::ConeConstraintSettings &settings);
bool set_half_cone_angle(tumble::World *world, tumble::ConstraintId id, float half_cone_angle);
float get_half_cone_angle(tumble::World *world, tumble::ConstraintId id);
float get_cos_half_cone_angle(tumble::World *world, tumble::ConstraintId id);
bool get_total_lambda_position(tumble::Float3 &out, tumble::World *world, tumble::ConstraintId id);
float get_total_lambda_rotation(tumble::World *world, tumble::ConstraintId id);

}  // namespace cone_constraint

namespace swing_twist_constraint {

void default_settings(tumble::SwingTwistConstraintSettings &out);
tumble::ConstraintId create(tumble::World *world, tumble::BodyId body_a, tumble::BodyId body_b,
                                           const tumble::SwingTwistConstraintSettings &settings);
bool set_normal_half_cone_angle(tumble::World *world, tumble::ConstraintId id, float angle);
bool set_plane_half_cone_angle(tumble::World *world, tumble::ConstraintId id, float angle);
bool set_twist_min_angle(tumble::World *world, tumble::ConstraintId id, float angle);
bool set_twist_max_angle(tumble::World *world, tumble::ConstraintId id, float angle);
bool set_max_friction_torque(tumble::World *world, tumble::ConstraintId id, float torque);
bool set_swing_motor_settings(tumble::World *world, tumble::ConstraintId id,
                              const tumble::MotorSettings &settings);
bool set_twist_motor_settings(tumble::World *world, tumble::ConstraintId id,
                              const tumble::MotorSettings &settings);
bool get_swing_motor_settings(tumble::MotorSettings &out, tumble::World *world, tumble::ConstraintId id);
bool get_twist_motor_settings(tumble::MotorSettings &out, tumble::World *world, tumble::ConstraintId id);
bool set_swing_motor_state(tumble::World *world, tumble::ConstraintId id, uint32_t state);
bool set_twist_motor_state(tumble::World *world, tumble::ConstraintId id, uint32_t state);
bool set_target_angular_velocity_cs(tumble::World *world, tumble::ConstraintId id, const tumble::Float3 &velocity);
bool set_target_orientation_cs(tumble::World *world, tumble::ConstraintId id, const tumble::Float4 &orientation);
bool set_target_orientation_bs(tumble::World *world, tumble::ConstraintId id, const tumble::Float4 &orientation);
bool get_rotation_in_constraint_space(tumble::Float4 &out, tumble::World *world, tumble::ConstraintId id);
bool get_total_lambda_position(tumble::Float3 &out, tumble::World *world, tumble::ConstraintId id);
float get_total_lambda_twist(tumble::World *world, tumble::ConstraintId id);
float get_total_lambda_swing_y(tumble::World *world, tumble::ConstraintId id);
float get_total_lambda_swing_z(tumble::World *world, tumble::ConstraintId id);
bool get_total_lambda_motor(tumble::Float3 &out, tumble::World *world, tumble::ConstraintId id);

}  // namespace swing_twist_constraint

namespace six_dof_constraint {

void default_settings(tumble::SixDOFConstraintSettings &out);
tumble::ConstraintId create(tumble::World *world, tumble::BodyId body_a, tumble::BodyId body_b,
                                           const tumble::SixDOFConstraintSettings &settings);
bool set_translation_limits(tumble::World *world, tumble::ConstraintId id, const tumble::Float3 &limit_min,
                            const tumble::Float3 &limit_max);
bool set_rotation_limits(tumble::World *world, tumble::ConstraintId id, const tumble::Float3 &limit_min,
                         const tumble::Float3 &limit_max);
/** translation axes only */
bool set_limits_spring_settings(tumble::World *world, tumble::ConstraintId id, uint32_t axis,
                                const tumble::SpringSettings &settings);
bool set_motor_settings(tumble::World *world, tumble::ConstraintId id, uint32_t axis,
                        const tumble::MotorSettings &settings);
bool get_motor_settings(tumble::MotorSettings &out, tumble::World *world, tumble::ConstraintId id,
                        uint32_t axis);
bool set_motor_state(tumble::World *world, tumble::ConstraintId id, uint32_t axis, uint32_t state);
bool set_target_velocity_cs(tumble::World *world, tumble::ConstraintId id, const tumble::Float3 &velocity);
bool set_target_angular_velocity_cs(tumble::World *world, tumble::ConstraintId id, const tumble::Float3 &velocity);
bool set_target_position_cs(tumble::World *world, tumble::ConstraintId id, const tumble::Float3 &position);
bool set_target_orientation_cs(tumble::World *world, tumble::ConstraintId id, const tumble::Float4 &orientation);
bool set_max_friction(tumble::World *world, tumble::ConstraintId id, uint32_t axis, float friction);
bool make_free_axis(tumble::World *world, tumble::ConstraintId id, uint32_t axis);
bool make_fixed_axis(tumble::World *world, tumble::ConstraintId id, uint32_t axis);
bool set_limited_axis(tumble::World *world, tumble::ConstraintId id, uint32_t axis, float min, float max);

}  // namespace six_dof_constraint

// a query filter's layers, against a world's; its group and mask cross with each query
namespace filter {

/** every layer of the world's enabled */
tumble::Filter *create(tumble::World *world);
void destroy(tumble::Filter *filter);
void enable_all_layers(tumble::Filter *filter, tumble::World *world);
void disable_all_layers(tumble::Filter *filter, tumble::World *world);
// false for a layer the world does not have
bool enable_object_layer(tumble::Filter *filter, tumble::World *world, uint32_t object_layer);
bool disable_object_layer(tumble::Filter *filter, tumble::World *world, uint32_t object_layer);
bool enable_broadphase_layer(tumble::Filter *filter, tumble::World *world, uint32_t broadphase_layer);
bool disable_broadphase_layer(tumble::Filter *filter, tumble::World *world, uint32_t broadphase_layer);
bool is_object_layer_enabled(const tumble::Filter *filter, uint32_t object_layer);
bool is_broadphase_layer_enabled(const tumble::Filter *filter, uint32_t broadphase_layer);
/** the layers the body's object layer collides with, and its group and mask, read back with the getters; false (error set) for a bad id */
bool set_from_body(tumble::Filter *filter, tumble::World *world, tumble::BodyId body);
uint32_t get_collision_groups(const tumble::Filter *filter);
uint32_t get_collision_mask(const tumble::Filter *filter);
void copy(tumble::Filter *out, const tumble::Filter *source);

}  // namespace filter

/** tumble::CastShapeHit as js reads it: no faces */
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
    tumble::BodyId body_id_b;
};

/** tumble::CastShapeSettings as js gives it: no faces to collect */
struct CastShapeSettings {
    bool collide_with_backfaces_triangles;
    bool collide_with_backfaces_convex;
    float collision_tolerance;
    float penetration_tolerance;
    bool return_deepest_point;
    bool use_shrunken_shape_and_convex_radius;
    bool collide_only_with_active_edges;
    tumble::Float3 active_edge_movement_direction;
    float extra_convex_radius;
};

/** tumble::CollideShapeHit as js reads it: no faces */
struct CollideShapeHit {
    tumble::Float3 point_a;
    tumble::Float3 point_b;
    tumble::Float3 penetration_axis;
    float penetration;
    uint32_t sub_shape_id_a;
    uint32_t sub_shape_id_b;
    tumble::BodyId body_id_b;
};

/** tumble::CollideShapeSettings as js gives it: no faces to collect */
struct CollideShapeSettings {
    float max_separation_distance;
    float collision_tolerance;
    float penetration_tolerance;
    bool collide_with_backfaces;
    bool collide_only_with_active_edges;
    tumble::Float3 active_edge_movement_direction;
    float internal_edge_removal_vertex_tolerance_sq;
};

// each query into the adapters' own collector of its type, of `kind` (a CollectorKind but CUSTOM) and
// starting at early_out_fraction, under the filter with the group and mask given (written into it): how many hits it
// holds, read by index (false past the count) until the next query of the type; 0 for a CUSTOM kind
namespace query {

void default_cast_ray_settings(tumble::CastRaySettings &out);
uint32_t cast_ray(tumble::World *world, uint32_t kind, float early_out_fraction, const tumble::CastRaySettings &settings,
                  const tumble::Float3 &origin, const tumble::Float3 &direction, float length, tumble::Filter *filter,
                  uint32_t collision_groups, uint32_t collision_mask);
bool read_cast_ray_hit(tumble::CastRayHit &out, uint32_t index);

void default_cast_shape_settings(CastShapeSettings &out);
uint32_t cast_shape(tumble::World *world, uint32_t kind, float early_out_fraction, const CastShapeSettings &settings,
                    const tumble::Shape *shape, const tumble::Float3 &position, const tumble::Float4 &quaternion,
                    const tumble::Float3 &scale, const tumble::Float3 &displacement, tumble::Filter *filter,
                    uint32_t collision_groups, uint32_t collision_mask);
bool read_cast_shape_hit(CastShapeHit &out, uint32_t index);

void default_collide_shape_settings(CollideShapeSettings &out);
/** with internal_edge_removal, through the internal edge removing wrapper, flushed per body */
uint32_t collide_shape(tumble::World *world, uint32_t kind, float early_out_fraction, const CollideShapeSettings &settings,
                       const tumble::Shape *shape, const tumble::Float3 &position, const tumble::Float4 &quaternion,
                       const tumble::Float3 &scale, bool internal_edge_removal, tumble::Filter *filter,
                       uint32_t collision_groups, uint32_t collision_mask);
bool read_collide_shape_hit(CollideShapeHit &out, uint32_t index);

void default_collide_point_settings(tumble::CollidePointSettings &out);
uint32_t collide_point(tumble::World *world, uint32_t kind, const tumble::CollidePointSettings &settings,
                       const tumble::Float3 &point, tumble::Filter *filter, uint32_t collision_groups,
                       uint32_t collision_mask);
bool read_collide_point_hit(tumble::CollidePointHit &out, uint32_t index);

}  // namespace query

namespace kcc_policy {

void default_body_policy(tumble::KCC::BodyPolicy &out);
tumble::KCC::Policy *create();
void destroy(tumble::KCC::Policy *policy);
void get_defaults(tumble::KCC::BodyPolicy &out, const tumble::KCC::Policy *policy);
void set_defaults(tumble::KCC::Policy *policy, const tumble::KCC::BodyPolicy &body_policy);
void set(tumble::KCC::Policy *policy, tumble::BodyId body, const tumble::KCC::BodyPolicy &body_policy);
bool remove(tumble::KCC::Policy *policy, tumble::BodyId body);
/** the body's, or the defaults */
void get(tumble::KCC::BodyPolicy &out, const tumble::KCC::Policy *policy, tumble::BodyId body);

}  // namespace kcc_policy

/** tumble::KCCSettings as js gives it: the user data in halves */
struct KCCSettings {
    tumble::Shape *shape;
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
    tumble::Shape *inner_rigid_body_shape;
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
    tumble::BodyId ground_body_id;
    uint32_t ground_sub_shape_id;
    tumble::Float3 ground_position;
    tumble::Shape *shape;
    tumble::BodyId inner_rigid_body_id;
    uint32_t contact_count;
    uint32_t contact_event_count;
};

/** KCC::ContactEvent as js reads it */
struct KCCContactEvent {
    uint32_t kind;
    tumble::BodyId body_id;
    uint32_t sub_shape_id;
    tumble::Float3 position;
    tumble::Float3 normal;
};

/** a KCC::Contact as js reads one that is not a row: a sweep's */
struct KCCContact {
    tumble::Float3 position;
    tumble::Float3 linear_velocity;
    tumble::Float3 contact_normal;
    tumble::Float3 surface_normal;
    float distance;
    float fraction;
    tumble::BodyId body_id;
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
// contacts are rows in its memory at contacts(), contact_count of them, KCC::Contact's layout under the KCC_CONTACT
// constants, until the next call that moves it
namespace kcc {

void default_settings(KCCSettings &out);
void default_update_settings(tumble::KCCUpdateSettings &out);
/** a character js holds one reference to until release */
tumble::KCC *create(const KCCSettings &settings, const tumble::Float3 &position, const tumble::Float4 &quaternion);
/** js lets go of the character it made; the last reference to go frees it, taking its inner body out of the world */
void release(const tumble::KCC *character);
void add(tumble::World *world, tumble::KCC *character);
void remove(tumble::World *world, tumble::KCC *character);

void read_state(KCCState &out, const tumble::KCC *character);
void set_linear_velocity(tumble::KCC *character, const tumble::Float3 &velocity);
void get_tuning(KCCTuning &out, const tumble::KCC *character);
/** as create sets them: up normalized, the slope's cosine with its angle */
void set_tuning(tumble::KCC *character, const KCCTuning &tuning);
uint8_t *contacts(tumble::KCC *character);
bool read_contact_event(KCCContactEvent &out, const tumble::KCC *character, uint32_t index);

void set_position(tumble::World *world, tumble::KCC *character, const tumble::Float3 &position);
void set_quaternion(tumble::World *world, tumble::KCC *character, const tumble::Float4 &quaternion);
bool is_slope_too_steep(const tumble::KCC *character, const tumble::Float3 &normal);
void get_center_of_mass_transform(tumble::Float3 &out_position, tumble::Float4 &out_quaternion, const tumble::KCC *character);
bool has_collided_with(const tumble::KCC *character, tumble::BodyId body);
bool can_walk_stairs(const tumble::KCC *character, const tumble::Float3 &velocity);
void cancel_velocity_towards_steep_slopes(tumble::Float3 &out, const tumble::KCC *character, const tumble::Float3 &velocity);

void update_ground_velocity(tumble::World *world, tumble::KCC *character, const tumble::KCC::Policy *policy);
void refresh_contacts(tumble::World *world, tumble::KCC *character, tumble::Filter *filter, uint32_t collision_groups,
                      uint32_t collision_mask, const tumble::KCC::Policy *policy);
bool set_shape(tumble::World *world, tumble::KCC *character, tumble::Shape *shape, tumble::Filter *filter,
               uint32_t collision_groups, uint32_t collision_mask, const tumble::KCC::Policy *policy, float max_penetration_depth);
void move(tumble::World *world, tumble::KCC *character, float delta_time, const tumble::Float3 &gravity,
          const tumble::KCC::Policy *policy, tumble::Filter *filter, uint32_t collision_groups, uint32_t collision_mask);
bool stick_to_floor(tumble::World *world, tumble::KCC *character, const tumble::Float3 &step_down, tumble::Filter *filter,
                    uint32_t collision_groups, uint32_t collision_mask, const tumble::KCC::Policy *policy);
bool walk_stairs(tumble::World *world, tumble::KCC *character, float delta_time, const tumble::Float3 &step_up,
                 const tumble::Float3 &step_forward, const tumble::Float3 &step_forward_test, const tumble::Float3 &step_down_extra,
                 tumble::Filter *filter, uint32_t collision_groups, uint32_t collision_mask, const tumble::KCC::Policy *policy);
void update(tumble::World *world, tumble::KCC *character, float delta_time, const tumble::Float3 &gravity,
            const tumble::KCCUpdateSettings &settings, const tumble::KCC::Policy *policy, tumble::Filter *filter,
            uint32_t collision_groups, uint32_t collision_mask);

// the toolkit
void collide(tumble::World *world, tumble::KCC *character, const tumble::Float3 &position, const tumble::Float3 &movement_direction,
             tumble::Filter *filter, uint32_t collision_groups, uint32_t collision_mask, const tumble::KCC::Policy *policy);
void solve(KCCSolve &out, tumble::World *world, tumble::KCC *character, const tumble::Float3 &velocity, float delta_time,
           float time_remaining, const tumble::KCC::Policy *policy);
bool cast(KCCContact &out, tumble::World *world, tumble::KCC *character, const tumble::Float3 &position,
          const tumble::Float3 &displacement, tumble::Filter *filter, uint32_t collision_groups, uint32_t collision_mask,
          const tumble::KCC::Policy *policy);
void update_ground(tumble::World *world, tumble::KCC *character, float delta_time, bool skip_contact_velocity_check,
                   const tumble::KCC::Policy *policy);
void apply_weight(tumble::World *world, tumble::KCC *character, const tumble::Float3 &gravity, float delta_time);
void begin_contact_tracking(tumble::KCC *character);
void end_contact_tracking(tumble::KCC *character);

}  // namespace kcc

// wireframes into buffers js views in place: lines and triangles as x y z r g b per vertex, valid until the next draw
// or clear, a draw moving them when it grows them
namespace debug {

tumble::DebugDraw *create();
void destroy(tumble::DebugDraw *buffers);
void clear(tumble::DebugDraw *buffers);
const float *lines(const tumble::DebugDraw *buffers);
uint32_t line_count(const tumble::DebugDraw *buffers);
const float *triangles(const tumble::DebugDraw *buffers);
uint32_t triangle_count(const tumble::DebugDraw *buffers);

void default_bodies_options(tumble::DebugDraw::BodiesOptions &out);
void default_body_options(tumble::DebugDraw::BodyOptions &out);
void default_shape_options(tumble::DebugDraw::ShapeOptions &out);
void default_joints_options(tumble::DebugDraw::JointsOptions &out);

// each draw appends; false (error set) for a body id naming no body
void bodies(tumble::DebugDraw *out, const tumble::World *world, const tumble::DebugDraw::BodiesOptions &options);
bool body(tumble::DebugDraw *out, tumble::World *world, tumble::BodyId body,
          const tumble::DebugDraw::BodyOptions &options);
void line(tumble::DebugDraw *out, const tumble::Float3 &a, const tumble::Float3 &b, const tumble::Float3 &color);
void shape(tumble::DebugDraw *out, const tumble::Shape *shape, const tumble::Float3 &position, const tumble::Float4 &quaternion,
           const tumble::DebugDraw::ShapeOptions &options);
void contacts(tumble::DebugDraw *out, const tumble::World *world);
void character_contacts(tumble::DebugDraw *out, const tumble::KCC *character);
void contact_constraints(tumble::DebugDraw *out, const tumble::World *world);
void joints(tumble::DebugDraw *out, const tumble::World *world, const tumble::DebugDraw::JointsOptions &options);

}  // namespace debug

/** a tumble::ContactPair as js reads it: user data in halves */
struct ContactPair {
    /** stable while the pair lasts; reused the frame after it is removed */
    uint32_t handle;
    tumble::BodyId body_a;
    uint32_t sub_shape_a;
    uint32_t user_data_a_low;
    uint32_t user_data_a_high;
    bool sensor_a;
    tumble::BodyId body_b;
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

void begin(tumble::World *world);
/** the last step's contact events from these indices of its added and persisted contacts on */
void record_world_contacts(tumble::World *world, uint32_t first_added, uint32_t first_persisted);
void record_character_contacts(tumble::World *world, const tumble::KCC *character);
void end(tumble::World *world);
/** this frame's pairs, in the order first recorded */
uint32_t active_count(const tumble::World *world);
bool read_active(ContactPair &out, const tumble::World *world, uint32_t index);
/** live last frame and not recorded in this one */
uint32_t removed_count(const tumble::World *world);
bool read_removed(ContactPair &out, const tumble::World *world, uint32_t index);

}  // namespace contact_pairs

}  // namespace adapters
