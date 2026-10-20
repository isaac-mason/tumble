// what js calls where the C++ api does not cross as it is: bodies by id, settings holding pointers, the world's
// settings built call by call. wasm/bindings.cpp binds them; the generated extern "C" layer calls them.

#include "wasm/adapters.h"

#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/events.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/update.h"
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

void step_begin(tumble::world::World *world, float delta_time) { update::step_begin(*world, delta_time); }
void find_pairs(tumble::world::World *world) { update::find_pairs(*world); }
void narrowphase(tumble::world::World *world) { update::narrowphase(*world); }
void solve(tumble::world::World *world) { update::solve(*world); }
void step_end(tumble::world::World *world) { update::step_end(*world); }
uint32_t woken(const tumble::world::World *world) { return world->events.woken; }

bool set_materials(tumble::world::World *world, const float *friction_restitution, uint32_t count) {
    collections::Owned<contact_constraints::Material> materials;
    if (!materials.v.reserve(count)) return false;
    for (uint32_t i = 0; i < count; i++) materials.v.push({friction_restitution[i * 2], friction_restitution[i * 2 + 1]});
    return tumble::world::set_materials(*world, materials.v.data, count);
}

}  // namespace world

namespace events {

static void split(uint32_t &low, uint32_t &high, uint64_t value) {
    low = (uint32_t)value;
    high = (uint32_t)(value >> 32);
}

uint32_t pairs(const tumble::world::World *world) { return world->events.pairs.length; }

bool read_pair(PairEvent &out, const tumble::world::World *world, uint32_t index) {
    if (index >= world->events.pairs.length) return false;
    const tumble::events::PairEvent &event = world->events.pairs[index];
    out.body_a = event.body_id_a;
    out.body_b = event.body_id_b;
    split(out.user_data_a_low, out.user_data_a_high, event.user_data_a);
    split(out.user_data_b_low, out.user_data_b_high, event.user_data_b);
    return true;
}

static void copy_contact(ContactEvent &out, const tumble::events::ContactEvent &event) {
    out.body_a = event.body_id_a;
    out.body_b = event.body_id_b;
    split(out.user_data_a_low, out.user_data_a_high, event.user_data_a);
    split(out.user_data_b_low, out.user_data_b_high, event.user_data_b);
    out.sub_shape_a = event.sub_shape_id_a;
    out.sub_shape_b = event.sub_shape_id_b;
    out.material_a = event.material_id_a;
    out.material_b = event.material_id_b;
    out.normal = event.normal;
    out.penetration = event.penetration;
    out.num_points = event.num_points;
    for (uint32_t i = 0; i < manifold::MAX_CONTACT_POINTS; i++) {
        out.points_a[i] = event.points_a[i];
        out.points_b[i] = event.points_b[i];
    }
    out.settings = event.settings;
}

uint32_t contacts_added(const tumble::world::World *world) { return world->events.contacts_added.length; }

bool read_contact_added(ContactEvent &out, const tumble::world::World *world, uint32_t index) {
    if (index >= world->events.contacts_added.length) return false;
    copy_contact(out, world->events.contacts_added[index]);
    return true;
}

uint32_t contacts_persisted(const tumble::world::World *world) { return world->events.contacts_persisted.length; }

bool read_contact_persisted(ContactEvent &out, const tumble::world::World *world, uint32_t index) {
    if (index >= world->events.contacts_persisted.length) return false;
    copy_contact(out, world->events.contacts_persisted[index]);
    return true;
}

uint32_t contacts_removed(const tumble::world::World *world) { return world->events.contacts_removed.length; }

bool read_contact_removed(ContactRemovedEvent &out, const tumble::world::World *world, uint32_t index) {
    if (index >= world->events.contacts_removed.length) return false;
    const tumble::events::ContactRemovedEvent &event = world->events.contacts_removed[index];
    out = {event.body_id_a, event.body_id_b, event.sub_shape_id_a, event.sub_shape_id_b};
    return true;
}

uint32_t moved(const tumble::world::World *world) { return world->events.moved.length; }

body_id::BodyId moved_body(const tumble::world::World *world, uint32_t index) {
    if (index >= world->events.moved.length) return body_id::INVALID_BODY_ID;
    return world->bodies.pool[world->events.moved[index]]->id;
}

}  // namespace events

namespace answers {

bool reject_pair(tumble::world::World *world, uint32_t index) {
    if (index >= world->events.pairs.length) return false;
    tumble::events::reject_pair(world->answers, index);
    return true;
}

bool override_contact(tumble::world::World *world, uint32_t index, const contact_constraints::ContactSettings &settings) {
    if (index >= world->answers.contact_override.length) return false;
    tumble::events::override_contact(world->answers, world->events, index).settings = settings;
    return true;
}

bool reject_contact(tumble::world::World *world, uint32_t index) {
    if (index >= world->answers.contact_override.length) return false;
    tumble::events::override_contact(world->answers, world->events, index).reject = true;
    return true;
}

}  // namespace answers

namespace rigid_body {

void default_settings(BodySettings &out) {
    const tumble::rigid_body::RigidBodySettings d = {};
    out = {d.shape, d.object_layer, d.motion_type, (uint32_t)d.user_data, (uint32_t)(d.user_data >> 32), d.position, d.quaternion,
           d.motion_quality, d.allowed_degrees_of_freedom, d.gravity_factor, d.linear_damping, d.angular_damping, d.max_linear_velocity,
           d.max_angular_velocity, d.num_velocity_steps_override, d.num_position_steps_override, d.friction, d.restitution,
           d.friction_combine_mode, d.restitution_combine_mode, d.collision_groups, d.collision_mask, d.use_manifold_reduction,
           d.allow_sleeping, d.sensor, d.enhanced_internal_edge_removal, d.collide_kinematic_vs_non_dynamic, d.observes_contacts,
           d.mass, false, mass_properties::create()};
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
    settings.mass_properties_override = s.has_mass_properties ? &s.mass_properties : nullptr;
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

bool exists(tumble::world::World *world, body_id::BodyId id) { return tumble::rigid_body::get(*world, id) != nullptr; }

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

bool get_transform(Float3 &out_position, Float4 &out_quaternion, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out_position = body->position;
    out_quaternion = body->quaternion;
    return true;
}

bool get_center_of_mass_position(Float3 &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->center_of_mass_position;
    return true;
}

bool get_linear_velocity(Float3 &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->motion_properties.linear_velocity;
    return true;
}

bool get_angular_velocity(Float3 &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->motion_properties.angular_velocity;
    return true;
}

bool get_velocity_at_point(Float3 &out, tumble::world::World *world, body_id::BodyId id, const Float3 &point) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    vec3f::store(out, tumble::rigid_body::get_velocity_at_point(body, vec3f::load(point)));
    return true;
}

bool get_aabb(Box3f &out, tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->aabb;
    return true;
}

float get_mass(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr || body->motion_properties.inv_mass <= 0.0f) return 0.0f;
    return 1.0f / body->motion_properties.inv_mass;
}

uint32_t get_motion_type(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : (uint32_t)body->motion_type;
}

uint32_t get_object_layer(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : body->object_layer;
}

uint32_t get_user_data_low(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : (uint32_t)body->user_data;
}

uint32_t get_user_data_high(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : (uint32_t)(body->user_data >> 32);
}

shapes::Shape *get_shape(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body == nullptr ? nullptr : body->shape;
}

bool is_sleeping(tumble::world::World *world, body_id::BodyId id) {
    const tumble::rigid_body::RigidBody *body = resolve(world, id);
    return body != nullptr && body->sleeping;
}

uint32_t read_transforms(tumble::world::World *world, const uint32_t *ids, uint32_t count, float *out) {
    uint32_t live = 0;
    for (uint32_t i = 0; i < count; i++) {
        const tumble::rigid_body::RigidBody *body = tumble::rigid_body::get(*world, ids[i]);
        if (body == nullptr) continue;
        float *at = out + i * 7;
        at[0] = body->position.x;
        at[1] = body->position.y;
        at[2] = body->position.z;
        at[3] = body->quaternion.x;
        at[4] = body->quaternion.y;
        at[5] = body->quaternion.z;
        at[6] = body->quaternion.w;
        live++;
    }
    return live;
}

bool set_position(tumble::world::World *world, body_id::BodyId id, const Float3 &position, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_position(*world, body, vec3f::load(position), wake);
    return true;
}

bool set_quaternion(tumble::world::World *world, body_id::BodyId id, const Float4 &quaternion, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_quaternion(*world, body, quatf::load(quaternion), wake);
    return true;
}

bool set_transform(tumble::world::World *world, body_id::BodyId id, const Float3 &position, const Float4 &quaternion, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_transform(*world, body, vec3f::load(position), quatf::load(quaternion), wake);
    return true;
}

bool set_linear_velocity(tumble::world::World *world, body_id::BodyId id, const Float3 &velocity) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_linear_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool set_angular_velocity(tumble::world::World *world, body_id::BodyId id, const Float3 &velocity) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_angular_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool add_linear_velocity(tumble::world::World *world, body_id::BodyId id, const Float3 &velocity) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_linear_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool add_angular_velocity(tumble::world::World *world, body_id::BodyId id, const Float3 &velocity) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_angular_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool add_force(tumble::world::World *world, body_id::BodyId id, const Float3 &force, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_force(*world, body, vec3f::load(force), wake);
    return true;
}

bool add_torque(tumble::world::World *world, body_id::BodyId id, const Float3 &torque, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_torque(*world, body, vec3f::load(torque), wake);
    return true;
}

bool add_force_at_position(tumble::world::World *world, body_id::BodyId id, const Float3 &force, const Float3 &position, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_force_at_position(*world, body, vec3f::load(force), vec3f::load(position), wake);
    return true;
}

bool add_impulse(tumble::world::World *world, body_id::BodyId id, const Float3 &impulse) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_impulse(*world, body, vec3f::load(impulse));
    return true;
}

bool add_impulse_at_position(tumble::world::World *world, body_id::BodyId id, const Float3 &impulse, const Float3 &position) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_impulse_at_position(*world, body, vec3f::load(impulse), vec3f::load(position));
    return true;
}

bool add_angular_impulse(tumble::world::World *world, body_id::BodyId id, const Float3 &impulse) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_angular_impulse(*world, body, vec3f::load(impulse));
    return true;
}

bool set_motion_type(tumble::world::World *world, body_id::BodyId id, uint32_t motion_type, bool wake) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_motion_type(*world, body, (motion_type::MotionType)motion_type, wake);
    return true;
}

bool set_object_layer(tumble::world::World *world, body_id::BodyId id, uint32_t object_layer) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_object_layer(*world, body, object_layer);
    return true;
}

bool set_shape(tumble::world::World *world, body_id::BodyId id, shapes::Shape *shape) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_shape(*world, body, shape);
    return true;
}

bool move_kinematic(tumble::world::World *world, body_id::BodyId id, const Float3 &position, const Float4 &quaternion, float delta_time) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::move_kinematic(*world, body, vec3f::load(position), quatf::load(quaternion), delta_time);
    return true;
}

bool sleep(tumble::world::World *world, body_id::BodyId id) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::sleep(*world, body);
    return true;
}

bool wake(tumble::world::World *world, body_id::BodyId id) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::wake(*world, body);
    return true;
}

bool set_user_data(tumble::world::World *world, body_id::BodyId id, uint32_t low, uint32_t high) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    body->user_data = (uint64_t)high << 32 | low;
    return true;
}

bool set_friction(tumble::world::World *world, body_id::BodyId id, float friction) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    body->friction = friction;
    return true;
}

bool set_restitution(tumble::world::World *world, body_id::BodyId id, float restitution) {
    tumble::rigid_body::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    body->restitution = restitution;
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
