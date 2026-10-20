// what js calls where the C++ api does not cross as it is: bodies by id, settings holding pointers, the world's
// settings built call by call. wasm/bindings.cpp binds them; the generated extern "C" layer calls them.

#include "wasm/adapters.h"

#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/util/alloc.h"
#include "tumble/util/error.h"
#include "tumble/util/object.h"
#include "wasm/runtime/instance.h"

using namespace tumble;

namespace adapters {

namespace wasm {

bool init(uint32_t heap_begin) {
    if (!runtime::init(heap_begin)) return false;
    dispatch::init();
    return true;
}

uint32_t abi_version() { return runtime::abi_version(); }
uint32_t schema_hash() { return runtime::schema_hash(); }
float simd_check() { return runtime::simd_check(); }
uint32_t error() { return util::error(); }
void error_clear() { util::error_clear(); }
void *alloc_bytes(uint32_t bytes) { return util::alloc_bytes(bytes); }
void free_bytes(void *block, uint32_t bytes) { util::free_bytes(block, bytes); }
uint32_t allocated_bytes() { return (uint32_t)util::allocated_bytes(); }

}  // namespace wasm

namespace world_settings {

tumble::world_settings::WorldSettings *create() {
    tumble::world_settings::WorldSettings *settings = util::create<tumble::world_settings::WorldSettings>();
    if (settings != nullptr) *settings = tumble::world_settings::create();
    return settings;
}

void destroy(tumble::world_settings::WorldSettings *settings) {
    tumble::world_settings::release(*settings);
    util::destroy(settings);
}

uint32_t add_broadphase_layer(tumble::world_settings::WorldSettings *settings) { return tumble::world_settings::add_broadphase_layer(*settings); }

uint32_t add_object_layer(tumble::world_settings::WorldSettings *settings, uint32_t broadphase_layer) {
    return tumble::world_settings::add_object_layer(*settings, broadphase_layer);
}

void enable_collision(tumble::world_settings::WorldSettings *settings, uint32_t object_layer_a, uint32_t object_layer_b) {
    tumble::world_settings::enable_collision(*settings, object_layer_a, object_layer_b);
}

}  // namespace world_settings

namespace world {

tumble::world::World *create(const tumble::world_settings::WorldSettings *settings) { return tumble::world::create(*settings); }
void destroy(tumble::world::World *world) { tumble::world::destroy(world); }
void step(tumble::world::World *world, float delta_time) { tumble::world::step(*world, delta_time); }

}  // namespace world

namespace rigid_body {

void default_settings(BodySettings &out) {
    const tumble::rigid_body::RigidBodySettings d = {};
    out = {d.shape, d.object_layer, d.motion_type, (uint32_t)d.user_data, (uint32_t)(d.user_data >> 32), d.position, d.quaternion,
           d.motion_quality, d.allowed_degrees_of_freedom, d.gravity_factor, d.linear_damping, d.angular_damping, d.max_linear_velocity,
           d.max_angular_velocity, d.num_velocity_steps_override, d.num_position_steps_override, d.friction, d.restitution,
           d.friction_combine_mode, d.restitution_combine_mode, d.collision_groups, d.collision_mask, d.use_manifold_reduction,
           d.allow_sleeping, d.sensor, d.enhanced_internal_edge_removal, d.collide_kinematic_vs_non_dynamic, d.observes_contacts,
           d.mass};
}

body_id::BodyId create(tumble::world::World *world, const BodySettings &s) {
    tumble::rigid_body::RigidBodySettings settings;
    settings.shape = s.shape;
    settings.object_layer = s.object_layer;
    settings.motion_type = (motion_type::MotionType)s.motion_type;
    settings.user_data = (uint64_t)s.user_data_high << 32 | s.user_data_low;
    settings.position = s.position;
    settings.quaternion = s.quaternion;
    settings.motion_quality = (motion_properties::MotionQuality)s.motion_quality;
    settings.allowed_degrees_of_freedom = s.allowed_degrees_of_freedom;
    settings.gravity_factor = s.gravity_factor;
    settings.linear_damping = s.linear_damping;
    settings.angular_damping = s.angular_damping;
    settings.max_linear_velocity = s.max_linear_velocity;
    settings.max_angular_velocity = s.max_angular_velocity;
    settings.num_velocity_steps_override = s.num_velocity_steps_override;
    settings.num_position_steps_override = s.num_position_steps_override;
    settings.friction = s.friction;
    settings.restitution = s.restitution;
    settings.friction_combine_mode = (combine_material::MaterialCombineMode)s.friction_combine_mode;
    settings.restitution_combine_mode = (combine_material::MaterialCombineMode)s.restitution_combine_mode;
    settings.collision_groups = s.collision_groups;
    settings.collision_mask = s.collision_mask;
    settings.use_manifold_reduction = s.use_manifold_reduction;
    settings.allow_sleeping = s.allow_sleeping;
    settings.sensor = s.sensor;
    settings.enhanced_internal_edge_removal = s.enhanced_internal_edge_removal;
    settings.collide_kinematic_vs_non_dynamic = s.collide_kinematic_vs_non_dynamic;
    settings.observes_contacts = s.observes_contacts;
    settings.mass = s.mass;
    tumble::rigid_body::RigidBody *body = tumble::rigid_body::create(*world, settings);
    return body == nullptr ? body_id::INVALID_BODY_ID : body->id;
}

/** the live body an id names, or null with the error set */
static tumble::rigid_body::RigidBody *resolve(tumble::world::World *world, body_id::BodyId id) {
    tumble::rigid_body::RigidBody *body = tumble::rigid_body::get(*world, id);
    if (body == nullptr) util::set_error(util::ERROR_BAD_HANDLE);
    return body;
}

bool remove(tumble::world::World *world, body_id::BodyId id) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body != nullptr && tumble::rigid_body::remove(*world, body);
}

bool get_position(Float3 &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->position;
    return true;
}

bool get_quaternion(Float4 &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->quaternion;
    return true;
}

bool get_linear_velocity(Float3 &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->motion_properties.linear_velocity;
    return true;
}

bool set_linear_velocity(tumble::world::World *world, body_id::BodyId id, const Float3 &velocity) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_linear_velocity(*world, body, vec3f::load(velocity));
    return true;
}

}  // namespace rigid_body

namespace box {

shapes::Shape *create(const tumble::box::BoxShapeSettings &settings) { return tumble::box::create(settings); }
void destroy(shapes::Shape *shape) { tumble::box::destroy(shape); }

}  // namespace box

namespace sphere {

shapes::Shape *create(const tumble::sphere::SphereShapeSettings &settings) { return tumble::sphere::create(settings); }
void destroy(shapes::Shape *shape) { tumble::sphere::destroy(shape); }

}  // namespace sphere

}  // namespace adapters
