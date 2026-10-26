// what js calls where the C++ api does not cross as it is: bodies by id, settings holding pointers, the world's
// settings built call by call. js/wasm/bindings.cpp binds them; the generated extern "C" layer calls them.

#include "wasm/adapters.h"

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/vec4f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/sleep.h"
#include "tumble/physics/events.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/physics/shapes/utils/triangle_mesh_data.h"
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
uint32_t error() { return (uint32_t)util::error(); }
void error_clear() { util::error_clear(); }
void *alloc_bytes(uint32_t bytes) { return util::alloc_bytes(bytes); }
void free_bytes(void *block, uint32_t bytes) { util::free_bytes(block, bytes); }
uint32_t allocated_bytes() { return (uint32_t)util::allocated_bytes(); }

}  // namespace wasm

namespace world_settings {

tumble::WorldSettings *create() {
    tumble::WorldSettings *settings = util::create<tumble::WorldSettings>();
    if (settings != nullptr) *settings = tumble::world_settings::create();
    return settings;
}

void destroy(tumble::WorldSettings *settings) {
    util::destroy(settings);
}

uint32_t add_broadphase_layer(tumble::WorldSettings *settings) { return tumble::world_settings::add_broadphase_layer(*settings); }

uint32_t add_object_layer(tumble::WorldSettings *settings, uint32_t broadphase_layer) {
    return tumble::world_settings::add_object_layer(*settings, broadphase_layer);
}

void enable_collision(tumble::WorldSettings *settings, uint32_t object_layer_a, uint32_t object_layer_b) {
    tumble::world_settings::enable_collision(*settings, object_layer_a, object_layer_b);
}

}  // namespace world_settings

static WorldState *state_of(const tumble::World *world) {
    return reinterpret_cast<WorldState *>(world->bodies.activation_listener->context);
}

namespace world {

tumble::World *create(const tumble::WorldSettings *settings) {
    WorldState *state = util::create<WorldState>();
    tumble::World *world = tumble::world::create(*settings);
    body_activation::init_recorder(state->recorder);
    tumble::world::set_body_activation_listener(*world, &state->recorder.listener);
    return world;
}

void destroy(tumble::World *world) {
    WorldState *state = state_of(world);
    tumble::world::destroy(world);
    util::destroy(state);
}
void step(tumble::World *world, float delta_time) { tumble::world::step(*world, delta_time); }

void step_begin(tumble::World *world, float delta_time) { update::step_begin(*world, delta_time); }
void find_pairs(tumble::World *world) { update::find_pairs(*world); }
void narrowphase(tumble::World *world) { update::narrowphase(*world); }
void solve(tumble::World *world) { update::solve(*world); }
void step_end(tumble::World *world) { update::step_end(*world); }
uint32_t woken(const tumble::World *world) { return world->events.woken; }

void set_materials(tumble::World *world, const float *friction_restitution, uint32_t count) {
    collections::Array<WorldMaterial> materials;
    materials.reserve(count);
    for (uint32_t i = 0; i < count; i++) materials.push({friction_restitution[i * 2], friction_restitution[i * 2 + 1]});
    tumble::world::set_materials(*world, materials.data, count);
}

void get_gravity(Float3 &out, const tumble::World *world) { out = world->settings.gravity; }

void set_gravity(tumble::World *world, const Float3 &gravity) { world->settings.gravity = gravity; }

}  // namespace world

namespace events {

static void split(uint32_t &low, uint32_t &high, uint64_t value) {
    low = (uint32_t)value;
    high = (uint32_t)(value >> 32);
}

uint32_t pairs(const tumble::World *world) { return world->events.pairs.length; }

bool read_pair(PairEvent &out, const tumble::World *world, uint32_t index) {
    if (index >= world->events.pairs.length) return false;
    const tumble::PairEvent &event = world->events.pairs[index];
    out.body_a = event.body_id_a;
    out.body_b = event.body_id_b;
    split(out.user_data_a_low, out.user_data_a_high, event.user_data_a);
    split(out.user_data_b_low, out.user_data_b_high, event.user_data_b);
    return true;
}

static void copy_contact(ContactEvent &out, const tumble::ContactEvent &event) {
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

uint32_t contacts_added(const tumble::World *world) { return world->events.contacts_added.length; }

bool read_contact_added(ContactEvent &out, const tumble::World *world, uint32_t index) {
    if (index >= world->events.contacts_added.length) return false;
    copy_contact(out, world->events.contacts_added[index]);
    return true;
}

uint32_t contacts_persisted(const tumble::World *world) { return world->events.contacts_persisted.length; }

bool read_contact_persisted(ContactEvent &out, const tumble::World *world, uint32_t index) {
    if (index >= world->events.contacts_persisted.length) return false;
    copy_contact(out, world->events.contacts_persisted[index]);
    return true;
}

uint32_t contacts_removed(const tumble::World *world) { return world->events.contacts_removed.length; }

bool read_contact_removed(ContactRemovedEvent &out, const tumble::World *world, uint32_t index) {
    if (index >= world->events.contacts_removed.length) return false;
    const tumble::ContactRemovedEvent &event = world->events.contacts_removed[index];
    out = {event.body_id_a, event.body_id_b, event.sub_shape_id_a, event.sub_shape_id_b};
    return true;
}

}  // namespace events

namespace activation {

uint32_t events(const tumble::World *world) { return state_of(world)->recorder.events.length; }

bool read_event(ActivationEvent &out, const tumble::World *world, uint32_t index) {
    const collections::Array<BodyActivationEvent> &events = state_of(world)->recorder.events;
    if (index >= events.length) return false;
    const BodyActivationEvent &event = events[index];
    out.body = event.body_id;
    out.user_data_low = (uint32_t)event.user_data;
    out.user_data_high = (uint32_t)(event.user_data >> 32);
    out.activated = event.activated ? 1 : 0;
    return true;
}

void clear(tumble::World *world) { body_activation::flush(state_of(world)->recorder, nullptr); }

}  // namespace activation

namespace active_bodies {

uint32_t count(const tumble::World *world) { return tumble::world::get_num_active_bodies(*world); }

uint32_t read(tumble::World *world) {
    WorldState &state = *state_of(world);
    const tumble::Bodies &bodies = world->bodies;
    const uint32_t active = tumble::world::get_num_active_bodies(*world);
    const uint32_t *indices = tumble::world::get_active_bodies(*world);
    const uint32_t words = (bodies.pool.length + 31) >> 5;
    state.rows.length = 0;
    while (state.marks.length < words) state.marks.push(0);
    state.rows.reserve(active);
    for (uint32_t i = 0; i < active; i++) state.marks[indices[i] >> 5] |= 1u << (indices[i] & 31);
    for (uint32_t word = 0; word < words; word++) {
        uint32_t bits = state.marks[word];
        if (bits == 0) continue;
        state.marks[word] = 0;
        while (bits != 0) {
            const uint32_t index = (word << 5) | static_cast<uint32_t>(__builtin_ctz(bits));
            bits &= bits - 1;
            const tumble::RigidBody &body = *bodies.pool[index];
            ActiveBodyRow &row = state.rows.data[state.rows.length++];
            row.body = body.id;
            row.position = body.position;
            row.quaternion = body.quaternion;
            row.center_of_mass_position = body.center_of_mass_position;
            row.linear_velocity = body.motion_properties.linear_velocity;
            row.angular_velocity = body.motion_properties.angular_velocity;
        }
    }
    return state.rows.length;
}

const uint32_t *rows(const tumble::World *world) { return reinterpret_cast<const uint32_t *>(state_of(world)->rows.data); }

}  // namespace active_bodies

namespace answers {

bool reject_pair(tumble::World *world, uint32_t index) {
    if (index >= world->events.pairs.length) return false;
    tumble::events::reject_pair(world->answers, index);
    return true;
}

bool override_contact(tumble::World *world, uint32_t index, const ContactSettings &settings) {
    if (index >= world->answers.contact_override.length) return false;
    tumble::events::override_contact(world->answers, world->events, index).settings = settings;
    return true;
}

bool reject_contact(tumble::World *world, uint32_t index) {
    if (index >= world->answers.contact_override.length) return false;
    tumble::events::override_contact(world->answers, world->events, index).reject = true;
    return true;
}

}  // namespace answers

namespace rigid_body {

void default_settings(BodySettings &out) {
    const tumble::RigidBodySettings d = {};
    out = {d.shape, d.object_layer, (uint32_t)d.motion_type, (uint32_t)d.user_data, (uint32_t)(d.user_data >> 32), d.position, d.quaternion,
           (uint32_t)d.motion_quality, d.allowed_degrees_of_freedom, d.gravity_factor, d.linear_damping, d.angular_damping, d.max_linear_velocity,
           d.max_angular_velocity, d.num_velocity_steps_override, d.num_position_steps_override, d.friction, d.restitution,
           (uint32_t)d.friction_combine_mode, (uint32_t)d.restitution_combine_mode, d.collision_groups, d.collision_mask, d.use_manifold_reduction,
           d.allow_sleeping, d.sensor, d.enhanced_internal_edge_removal, d.collide_kinematic_vs_non_dynamic, d.observes_contacts,
           d.mass, false, mass_properties::create()};
}

BodyId create(tumble::World *world, const BodySettings &s) {
    tumble::RigidBodySettings settings;
    settings.shape = s.shape;
    settings.object_layer = s.object_layer;
    settings.motion_type = (MotionType)s.motion_type;
    settings.user_data = (uint64_t)s.user_data_high << 32 | s.user_data_low;
    settings.position = s.position;
    settings.quaternion = s.quaternion;
    settings.motion_quality = (MotionQuality)s.motion_quality;
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
    settings.friction_combine_mode = (MaterialCombineMode)s.friction_combine_mode;
    settings.restitution_combine_mode = (MaterialCombineMode)s.restitution_combine_mode;
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
    tumble::RigidBody *body = tumble::rigid_body::create(*world, settings);
    return body == nullptr ? body_id::INVALID_BODY_ID : body->id;
}

/** the live body an id names, or null with the error set */
static tumble::RigidBody *resolve(tumble::World *world, BodyId id) {
    tumble::RigidBody *body = tumble::rigid_body::get(*world, id);
    if (body == nullptr) util::set_error(Error::BAD_HANDLE);
    return body;
}

bool remove(tumble::World *world, BodyId id) {
    tumble::RigidBody *body = resolve(world, id);
    return body != nullptr && tumble::rigid_body::remove(*world, body);
}

bool exists(tumble::World *world, BodyId id) { return tumble::rigid_body::get(*world, id) != nullptr; }

bool get_position(Float3 &out, tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->position;
    return true;
}

bool get_quaternion(Float4 &out, tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->quaternion;
    return true;
}

bool get_transform(Float3 &out_position, Float4 &out_quaternion, tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out_position = body->position;
    out_quaternion = body->quaternion;
    return true;
}

bool get_center_of_mass_position(Float3 &out, tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->center_of_mass_position;
    return true;
}

bool get_linear_velocity(Float3 &out, tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->motion_properties.linear_velocity;
    return true;
}

bool get_angular_velocity(Float3 &out, tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->motion_properties.angular_velocity;
    return true;
}

bool get_velocity_at_point(Float3 &out, tumble::World *world, BodyId id, const Float3 &point) {
    const tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    vec3f::store(out, tumble::rigid_body::get_velocity_at_point(body, vec3f::load(point)));
    return true;
}

bool get_aabb(Box3f &out, tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    out = body->aabb;
    return true;
}

float get_mass(tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr || body->motion_properties.inv_mass <= 0.0f) return 0.0f;
    return 1.0f / body->motion_properties.inv_mass;
}

uint32_t get_motion_type(tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : (uint32_t)body->motion_type;
}

uint32_t get_object_layer(tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : body->object_layer;
}

uint32_t get_user_data_low(tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : (uint32_t)body->user_data;
}

uint32_t get_user_data_high(tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    return body == nullptr ? 0 : (uint32_t)(body->user_data >> 32);
}

Shape *get_shape(tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    return body == nullptr ? nullptr : body->shape;
}

int32_t get_material(tumble::World *world, BodyId id, uint32_t sub_shape_id) {
    const tumble::RigidBody *body = resolve(world, id);
    return body == nullptr ? -1 : tumble::rigid_body::get_material(body, sub_shape_id);
}

bool is_sleeping(tumble::World *world, BodyId id) {
    const tumble::RigidBody *body = resolve(world, id);
    return body != nullptr && tumble::rigid_body::is_sleeping(*body);
}

bool set_position(tumble::World *world, BodyId id, const Float3 &position, bool wake) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_position(*world, body, vec3f::load(position), wake);
    return true;
}

bool set_quaternion(tumble::World *world, BodyId id, const Float4 &quaternion, bool wake) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_quaternion(*world, body, quatf::load(quaternion), wake);
    return true;
}

bool set_transform(tumble::World *world, BodyId id, const Float3 &position, const Float4 &quaternion, bool wake) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_transform(*world, body, vec3f::load(position), quatf::load(quaternion), wake);
    return true;
}

bool set_linear_velocity(tumble::World *world, BodyId id, const Float3 &velocity) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_linear_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool set_angular_velocity(tumble::World *world, BodyId id, const Float3 &velocity) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_angular_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool add_linear_velocity(tumble::World *world, BodyId id, const Float3 &velocity) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_linear_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool add_angular_velocity(tumble::World *world, BodyId id, const Float3 &velocity) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_angular_velocity(*world, body, vec3f::load(velocity));
    return true;
}

bool add_force(tumble::World *world, BodyId id, const Float3 &force, bool wake) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_force(*world, body, vec3f::load(force), wake);
    return true;
}

bool add_torque(tumble::World *world, BodyId id, const Float3 &torque, bool wake) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_torque(*world, body, vec3f::load(torque), wake);
    return true;
}

bool add_force_at_position(tumble::World *world, BodyId id, const Float3 &force, const Float3 &position, bool wake) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_force_at_position(*world, body, vec3f::load(force), vec3f::load(position), wake);
    return true;
}

bool add_impulse(tumble::World *world, BodyId id, const Float3 &impulse) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_impulse(*world, body, vec3f::load(impulse));
    return true;
}

bool add_impulse_at_position(tumble::World *world, BodyId id, const Float3 &impulse, const Float3 &position) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_impulse_at_position(*world, body, vec3f::load(impulse), vec3f::load(position));
    return true;
}

bool add_angular_impulse(tumble::World *world, BodyId id, const Float3 &impulse) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::add_angular_impulse(*world, body, vec3f::load(impulse));
    return true;
}

bool set_motion_type(tumble::World *world, BodyId id, uint32_t motion_type, bool wake) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_motion_type(*world, body, (MotionType)motion_type, wake);
    return true;
}

bool set_object_layer(tumble::World *world, BodyId id, uint32_t object_layer) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_object_layer(*world, body, object_layer);
    return true;
}

bool set_shape(tumble::World *world, BodyId id, Shape *shape, bool wake) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::set_shape(*world, body, shape, wake);
    return true;
}

bool move_kinematic(tumble::World *world, BodyId id, const Float3 &position, const Float4 &quaternion, float delta_time) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::move_kinematic(*world, body, vec3f::load(position), quatf::load(quaternion), delta_time);
    return true;
}

bool sleep(tumble::World *world, BodyId id) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::sleep(*world, body);
    return true;
}

bool wake(tumble::World *world, BodyId id) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    tumble::rigid_body::wake(*world, body);
    return true;
}

bool set_user_data(tumble::World *world, BodyId id, uint32_t low, uint32_t high) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    body->user_data = (uint64_t)high << 32 | low;
    return true;
}

bool set_friction(tumble::World *world, BodyId id, float friction) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    body->friction = friction;
    return true;
}

bool set_restitution(tumble::World *world, BodyId id, float restitution) {
    tumble::RigidBody *body = resolve(world, id);
    if (body == nullptr) return false;
    body->restitution = restitution;
    return true;
}

}  // namespace rigid_body

/** a shape made for js, which holds one reference to it until shape::release */
static Shape *held_by_js(Shape *shape) {
    if (shape != nullptr) add_ref(shape);
    return shape;
}

namespace shape {

uint32_t get_type(const Shape *shape) { return (uint32_t)shape->type; }
void get_local_bounds(Box3f &out, const Shape *shape) { out = shape->aabb; }
void get_center_of_mass(Float3 &out, const Shape *shape) { out = shape->center_of_mass; }
float get_volume(const Shape *shape) { return shape->volume; }
void get_mass_properties(MassProperties &out, const Shape *shape) { shapes::compute_mass_properties(out, shape); }

uint32_t get_ref_count(const Shape *shape) { return shape->ref_count; }

void release(const Shape *shape) { release_ref(shape); }

}  // namespace shape

namespace box {

void default_settings(tumble::BoxShapeSettings &out) { out = {}; }
Shape *create(const tumble::BoxShapeSettings &settings) { return held_by_js(tumble::box::create(settings)); }

void update(Shape *shape, const tumble::BoxShapeSettings &settings) {
    tumble::BoxShape &typed = shapes::as<tumble::BoxShape>(shape);
    typed.half_extents = settings.half_extents;
    typed.convex_radius = settings.convex_radius;
    typed.density = settings.density;
    shape->material_id = settings.material_id;
    tumble::box::update(shape);
}

}  // namespace box

namespace sphere {

void default_settings(tumble::SphereShapeSettings &out) { out = {}; }
Shape *create(const tumble::SphereShapeSettings &settings) { return held_by_js(tumble::sphere::create(settings)); }

void update(Shape *shape, const tumble::SphereShapeSettings &settings) {
    tumble::SphereShape &typed = shapes::as<tumble::SphereShape>(shape);
    typed.radius = settings.radius;
    typed.density = settings.density;
    shape->material_id = settings.material_id;
    tumble::sphere::update(shape);
}

}  // namespace sphere

namespace capsule {

void default_settings(tumble::CapsuleShapeSettings &out) { out = {}; }
Shape *create(const tumble::CapsuleShapeSettings &settings) { return held_by_js(tumble::capsule::create(settings)); }

void update(Shape *shape, const tumble::CapsuleShapeSettings &settings) {
    tumble::CapsuleShape &typed = shapes::as<tumble::CapsuleShape>(shape);
    typed.half_height_of_cylinder = settings.half_height_of_cylinder;
    typed.radius = settings.radius;
    typed.density = settings.density;
    shape->material_id = settings.material_id;
    tumble::capsule::update(shape);
}

}  // namespace capsule

namespace cylinder {

void default_settings(tumble::CylinderShapeSettings &out) { out = {}; }
Shape *create(const tumble::CylinderShapeSettings &settings) { return held_by_js(tumble::cylinder::create(settings)); }

void update(Shape *shape, const tumble::CylinderShapeSettings &settings) {
    tumble::CylinderShape &typed = shapes::as<tumble::CylinderShape>(shape);
    typed.half_height = settings.half_height;
    typed.radius = settings.radius;
    typed.convex_radius = settings.convex_radius;
    typed.density = settings.density;
    shape->material_id = settings.material_id;
    tumble::cylinder::update(shape);
}

}  // namespace cylinder

namespace plane {

void default_settings(tumble::PlaneShapeSettings &out) { out = {}; }
Shape *create(const tumble::PlaneShapeSettings &settings) { return held_by_js(tumble::plane::create(settings)); }

}  // namespace plane

namespace empty {

Shape *create() { return held_by_js(tumble::empty::create()); }

}  // namespace empty

namespace convex_hull {

void default_settings(tumble::ConvexHullShapeSettings &out) { out = {}; }

bool create(ConvexHullCreate &out, const tumble::ConvexHullShapeSettings &settings) {
    tumble::ConvexHullShape::BuildResult result = tumble::ConvexHullShape::BuildResult::SUCCESS;
    out.shape = held_by_js(tumble::convex_hull::create(settings, &result));
    out.result = (int32_t)result;
    return out.shape != nullptr;
}


}  // namespace convex_hull

namespace triangle_mesh {

void default_settings(tumble::TriangleMeshShapeSettings &out) { out = {}; }
Shape *create(const tumble::TriangleMeshShapeSettings &settings) { return held_by_js(tumble::triangle_mesh::create(settings)); }

}  // namespace triangle_mesh

namespace compound {

Shape *create(const uint8_t *children, uint32_t count) {
    return held_by_js(tumble::compound::create({.children = (const tumble::CompoundShapeSettings::Child *)children, .num_children = count}));
}


void set_children(Shape *shape, const uint8_t *children, uint32_t count) {
    tumble::CompoundShape::Data &data = *shapes::as<tumble::CompoundShape>(shape).data;
    const tumble::CompoundShapeSettings::Child *given = (const tumble::CompoundShapeSettings::Child *)children;
    // the new list made whole first, so a shape both old and new stays held throughout
    tumble::collections::Array<tumble::CompoundShape::Child> list;
    list.reserve(count);
    data.child_bounds.reserve(count);
    for (uint32_t i = 0; i < count; i++) list.push(tumble::compound::held_child(given[i]));
    data.children = util::move(list);
    tumble::compound::update(shape);
}

}  // namespace compound

namespace static_compound {

uint32_t default_bvh_max_leaf_children() { return tumble::StaticCompoundShapeSettings{}.bvh_max_leaf_children; }

Shape *create(const uint8_t *children, uint32_t count, uint32_t bvh_max_leaf_children) {
    return held_by_js(tumble::static_compound::create({.children = (const tumble::CompoundShapeSettings::Child *)children,
                                            .num_children = count,
                                            .bvh_max_leaf_children = bvh_max_leaf_children}));
}


}  // namespace static_compound

namespace transformed {

void default_settings(tumble::TransformedShapeSettings &out) { out = {}; }
Shape *create(const tumble::TransformedShapeSettings &settings) { return held_by_js(tumble::transformed::create(settings)); }

}  // namespace transformed

namespace scaled {

void default_settings(tumble::ScaledShapeSettings &out) { out = {}; }
Shape *create(const tumble::ScaledShapeSettings &settings) { return held_by_js(tumble::scaled::create(settings)); }

}  // namespace scaled

namespace offset_center_of_mass {

void default_settings(tumble::OffsetCenterOfMassShapeSettings &out) { out = {}; }
Shape *create(const tumble::OffsetCenterOfMassShapeSettings &settings) { return held_by_js(tumble::offset_center_of_mass::create(settings)); }

}  // namespace offset_center_of_mass

using tumble::ConstraintId;

/** the constraint record a get found, or null with the error set */
template <typename Record>
static Record *found(Record *record) {
    if (record == nullptr) util::set_error(Error::BAD_HANDLE);
    return record;
}

template <typename Settings>
static ConstraintId create_between(tumble::World *world, BodyId body_a, BodyId body_b, const Settings &settings,
                                   ConstraintId (*create)(tumble::World &, tumble::RigidBody *,
                                                          tumble::RigidBody *, const Settings &)) {
    tumble::RigidBody *a = rigid_body::resolve(world, body_a);
    tumble::RigidBody *b = rigid_body::resolve(world, body_b);
    if (a == nullptr || b == nullptr) return tumble::constraint_id::INVALID_CONSTRAINT_ID;
    return create(*world, a, b, settings);
}

namespace constraint {

uint32_t get_type(ConstraintId id) { return (uint32_t)tumble::constraint_id::type(id); }

bool exists(tumble::World *world, ConstraintId id) { return constraints::get(world->constraints, id) != nullptr; }

bool set_enabled(tumble::World *world, ConstraintId id, bool enabled) {
    ConstraintBase *base = found(constraints::get(world->constraints, id));
    if (base == nullptr) return false;
    base->enabled = enabled;
    return true;
}

bool is_enabled(tumble::World *world, ConstraintId id) {
    const ConstraintBase *base = found(constraints::get(world->constraints, id));
    return base != nullptr && base->enabled;
}

bool remove(tumble::World *world, ConstraintId id) { return found(constraints::get(world->constraints, id)) != nullptr && constraints::remove(*world, id); }

}  // namespace constraint

namespace point_constraint {

void default_settings(tumble::PointConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::World *world, BodyId body_a, BodyId body_b,
                    const tumble::PointConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::point_constraint::create);
}


bool get_total_lambda(Float3 &out, tumble::World *world, ConstraintId id) {
    const tumble::PointConstraint *constraint = found(tumble::point_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::point_constraint::get_total_lambda(*constraint));
    return true;
}

}  // namespace point_constraint

namespace distance_constraint {

void default_settings(tumble::DistanceConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::World *world, BodyId body_a, BodyId body_b,
                    const tumble::DistanceConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::distance_constraint::create);
}


float get_total_lambda(tumble::World *world, ConstraintId id) {
    const tumble::DistanceConstraint *constraint = found(tumble::distance_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::distance_constraint::get_total_lambda(*constraint);
}

bool set_distance(tumble::World *world, ConstraintId id, float min_distance, float max_distance) {
    tumble::DistanceConstraint *constraint = found(tumble::distance_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::distance_constraint::set_distance(*constraint, min_distance, max_distance);
    return true;
}

bool set_limits_spring_settings(tumble::World *world, ConstraintId id, const SpringSettings &settings) {
    tumble::DistanceConstraint *constraint = found(tumble::distance_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->limits_spring_settings = settings;
    return true;
}

}  // namespace distance_constraint

namespace hinge_constraint {

void default_settings(tumble::HingeConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::World *world, BodyId body_a, BodyId body_b,
                    const tumble::HingeConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::hinge_constraint::create);
}


bool set_limits(tumble::World *world, ConstraintId id, float min, float max) {
    tumble::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::hinge_constraint::set_limits(*constraint, min, max);
    return true;
}

bool set_limits_spring_settings(tumble::World *world, ConstraintId id, const SpringSettings &settings) {
    tumble::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->limits_spring_settings = settings;
    return true;
}

bool set_max_friction_torque(tumble::World *world, ConstraintId id, float torque) {
    tumble::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->max_friction_torque = torque;
    return true;
}

bool set_motor_settings(tumble::World *world, ConstraintId id, const MotorSettings &settings) {
    tumble::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->motor_settings = settings;
    return true;
}

bool get_motor_settings(MotorSettings &out, tumble::World *world, ConstraintId id) {
    const tumble::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    out = constraint->motor_settings;
    return true;
}

bool set_motor_state(tumble::World *world, ConstraintId id, uint32_t state) {
    tumble::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::hinge_constraint::set_motor_state(*constraint, (MotorState)state);
    return true;
}

bool set_target_angular_velocity(tumble::World *world, ConstraintId id, float velocity) {
    tumble::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::hinge_constraint::set_target_angular_velocity(*constraint, velocity);
    return true;
}

bool set_target_angle(tumble::World *world, ConstraintId id, float angle) {
    tumble::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::hinge_constraint::set_target_angle(*constraint, angle);
    return true;
}

float get_current_angle(tumble::World *world, ConstraintId id) {
    const tumble::HingeConstraint *constraint = found(tumble::hinge_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::hinge_constraint::get_current_angle(*constraint);
}

}  // namespace hinge_constraint

namespace slider_constraint {

void default_settings(tumble::SliderConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::World *world, BodyId body_a, BodyId body_b,
                    const tumble::SliderConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::slider_constraint::create);
}


float get_current_position(tumble::World *world, ConstraintId id) {
    const tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::slider_constraint::get_current_position(*world, *constraint);
}

bool set_limits(tumble::World *world, ConstraintId id, float min, float max) {
    tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::set_limits(*constraint, min, max);
    return true;
}

bool set_limits_spring_settings(tumble::World *world, ConstraintId id, const SpringSettings &settings) {
    tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->limits_spring_settings = settings;
    return true;
}

bool set_motor_settings(tumble::World *world, ConstraintId id, const MotorSettings &settings) {
    tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->motor_settings = settings;
    return true;
}

bool get_motor_settings(MotorSettings &out, tumble::World *world, ConstraintId id) {
    const tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    out = constraint->motor_settings;
    return true;
}

bool set_motor_state(tumble::World *world, ConstraintId id, uint32_t state) {
    tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::set_motor_state(*constraint, (MotorState)state);
    return true;
}

bool set_target_velocity(tumble::World *world, ConstraintId id, float velocity) {
    tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::set_target_velocity(*constraint, velocity);
    return true;
}

bool set_target_position(tumble::World *world, ConstraintId id, float position) {
    tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::set_target_position(*constraint, position);
    return true;
}

bool set_max_friction_force(tumble::World *world, ConstraintId id, float force) {
    tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::set_max_friction_force(*constraint, force);
    return true;
}

float get_max_friction_force(tumble::World *world, ConstraintId id) {
    const tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::slider_constraint::get_max_friction_force(*constraint);
}

bool get_total_lambda_position(Vec2f &out, tumble::World *world, ConstraintId id) {
    const tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::slider_constraint::get_total_lambda_position(out, *constraint);
    return true;
}

float get_total_lambda_position_limits(tumble::World *world, ConstraintId id) {
    const tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::slider_constraint::get_total_lambda_position_limits(*constraint);
}

bool get_total_lambda_rotation(Float3 &out, tumble::World *world, ConstraintId id) {
    const tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::slider_constraint::get_total_lambda_rotation(*constraint));
    return true;
}

float get_total_lambda_motor(tumble::World *world, ConstraintId id) {
    const tumble::SliderConstraint *constraint = found(tumble::slider_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::slider_constraint::get_total_lambda_motor(*constraint);
}

}  // namespace slider_constraint

namespace fixed_constraint {

void default_settings(tumble::FixedConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::World *world, BodyId body_a, BodyId body_b,
                    const tumble::FixedConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::fixed_constraint::create);
}


bool get_total_lambda_position(Float3 &out, tumble::World *world, ConstraintId id) {
    const tumble::FixedConstraint *constraint = found(tumble::fixed_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::fixed_constraint::get_total_lambda_position(*constraint));
    return true;
}

bool get_total_lambda_rotation(Float3 &out, tumble::World *world, ConstraintId id) {
    const tumble::FixedConstraint *constraint = found(tumble::fixed_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::fixed_constraint::get_total_lambda_rotation(*constraint));
    return true;
}

}  // namespace fixed_constraint

namespace cone_constraint {

void default_settings(tumble::ConeConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::World *world, BodyId body_a, BodyId body_b,
                    const tumble::ConeConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::cone_constraint::create);
}


bool set_half_cone_angle(tumble::World *world, ConstraintId id, float half_cone_angle) {
    tumble::ConeConstraint *constraint = found(tumble::cone_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::cone_constraint::set_half_cone_angle(*constraint, half_cone_angle);
    return true;
}

float get_half_cone_angle(tumble::World *world, ConstraintId id) {
    const tumble::ConeConstraint *constraint = found(tumble::cone_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::cone_constraint::get_half_cone_angle(*constraint);
}

float get_cos_half_cone_angle(tumble::World *world, ConstraintId id) {
    const tumble::ConeConstraint *constraint = found(tumble::cone_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::cone_constraint::get_cos_half_cone_angle(*constraint);
}

bool get_total_lambda_position(Float3 &out, tumble::World *world, ConstraintId id) {
    const tumble::ConeConstraint *constraint = found(tumble::cone_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::cone_constraint::get_total_lambda_position(*constraint));
    return true;
}

float get_total_lambda_rotation(tumble::World *world, ConstraintId id) {
    const tumble::ConeConstraint *constraint = found(tumble::cone_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::cone_constraint::get_total_lambda_rotation(*constraint);
}

}  // namespace cone_constraint

namespace swing_twist_constraint {

void default_settings(tumble::SwingTwistConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::World *world, BodyId body_a, BodyId body_b,
                    const tumble::SwingTwistConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::swing_twist_constraint::create);
}


bool set_normal_half_cone_angle(tumble::World *world, ConstraintId id, float angle) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_normal_half_cone_angle(*constraint, angle);
    return true;
}

bool set_plane_half_cone_angle(tumble::World *world, ConstraintId id, float angle) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_plane_half_cone_angle(*constraint, angle);
    return true;
}

bool set_twist_min_angle(tumble::World *world, ConstraintId id, float angle) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_twist_min_angle(*constraint, angle);
    return true;
}

bool set_twist_max_angle(tumble::World *world, ConstraintId id, float angle) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_twist_max_angle(*constraint, angle);
    return true;
}

bool set_max_friction_torque(tumble::World *world, ConstraintId id, float torque) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_max_friction_torque(*constraint, torque);
    return true;
}

bool set_swing_motor_settings(tumble::World *world, ConstraintId id, const MotorSettings &settings) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->swing_motor_settings = settings;
    return true;
}

bool set_twist_motor_settings(tumble::World *world, ConstraintId id, const MotorSettings &settings) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    constraint->twist_motor_settings = settings;
    return true;
}

bool get_swing_motor_settings(MotorSettings &out, tumble::World *world, ConstraintId id) {
    const tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    out = constraint->swing_motor_settings;
    return true;
}

bool get_twist_motor_settings(MotorSettings &out, tumble::World *world, ConstraintId id) {
    const tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    out = constraint->twist_motor_settings;
    return true;
}

bool set_swing_motor_state(tumble::World *world, ConstraintId id, uint32_t state) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_swing_motor_state(*constraint, (MotorState)state);
    return true;
}

bool set_twist_motor_state(tumble::World *world, ConstraintId id, uint32_t state) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_twist_motor_state(*constraint, (MotorState)state);
    return true;
}

bool set_target_angular_velocity_cs(tumble::World *world, ConstraintId id, const Float3 &velocity) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_target_angular_velocity_cs(*constraint, vec3f::load(velocity));
    return true;
}

bool set_target_orientation_cs(tumble::World *world, ConstraintId id, const Float4 &orientation) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_target_orientation_cs(*constraint, quatf::load(orientation));
    return true;
}

bool set_target_orientation_bs(tumble::World *world, ConstraintId id, const Float4 &orientation) {
    tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::swing_twist_constraint::set_target_orientation_bs(*constraint, quatf::load(orientation));
    return true;
}

bool get_rotation_in_constraint_space(Float4 &out, tumble::World *world, ConstraintId id) {
    const tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    quatf::store(out, tumble::swing_twist_constraint::get_rotation_in_constraint_space(*constraint, world->bodies));
    return true;
}

bool get_total_lambda_position(Float3 &out, tumble::World *world, ConstraintId id) {
    const tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::swing_twist_constraint::get_total_lambda_position(*constraint));
    return true;
}

float get_total_lambda_twist(tumble::World *world, ConstraintId id) {
    const tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::swing_twist_constraint::get_total_lambda_twist(*constraint);
}

float get_total_lambda_swing_y(tumble::World *world, ConstraintId id) {
    const tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::swing_twist_constraint::get_total_lambda_swing_y(*constraint);
}

float get_total_lambda_swing_z(tumble::World *world, ConstraintId id) {
    const tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    return constraint == nullptr ? 0.0f : tumble::swing_twist_constraint::get_total_lambda_swing_z(*constraint);
}

bool get_total_lambda_motor(Float3 &out, tumble::World *world, ConstraintId id) {
    const tumble::SwingTwistConstraint *constraint = found(tumble::swing_twist_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    vec3f::store(out, tumble::swing_twist_constraint::get_total_lambda_motor(*constraint));
    return true;
}

}  // namespace swing_twist_constraint

namespace six_dof_constraint {

using tumble::SixDOFConstraint;

/** the constraint, or null: with the error set for a bad id, without it for an axis past the last */
static SixDOFConstraint *found_axis(tumble::World *world, ConstraintId id, uint32_t axis, uint32_t axes) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    return axis < axes ? constraint : nullptr;
}

void default_settings(tumble::SixDOFConstraintSettings &out) { out = {}; }

ConstraintId create(tumble::World *world, BodyId body_a, BodyId body_b,
                    const tumble::SixDOFConstraintSettings &settings) {
    return create_between(world, body_a, body_b, settings, tumble::six_dof_constraint::create);
}


bool set_translation_limits(tumble::World *world, ConstraintId id, const Float3 &limit_min, const Float3 &limit_max) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_translation_limits(*constraint, vec3f::load(limit_min), vec3f::load(limit_max));
    return true;
}

bool set_rotation_limits(tumble::World *world, ConstraintId id, const Float3 &limit_min, const Float3 &limit_max) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_rotation_limits(*constraint, vec3f::load(limit_min), vec3f::load(limit_max));
    return true;
}

bool set_limits_spring_settings(tumble::World *world, ConstraintId id, uint32_t axis, const SpringSettings &settings) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::SixDOFConstraint::Axis::NUM_TRANSLATION);
    if (constraint == nullptr) return false;
    constraint->limits_spring_settings[axis] = settings;
    return true;
}

bool set_motor_settings(tumble::World *world, ConstraintId id, uint32_t axis, const MotorSettings &settings) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::SixDOFConstraint::Axis::NUM);
    if (constraint == nullptr) return false;
    constraint->motor_settings[axis] = settings;
    return true;
}

bool get_motor_settings(MotorSettings &out, tumble::World *world, ConstraintId id, uint32_t axis) {
    const SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::SixDOFConstraint::Axis::NUM);
    if (constraint == nullptr) return false;
    out = constraint->motor_settings[axis];
    return true;
}

bool set_motor_state(tumble::World *world, ConstraintId id, uint32_t axis, uint32_t state) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::SixDOFConstraint::Axis::NUM);
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_motor_state(*constraint, (tumble::SixDOFConstraint::Axis)axis, (MotorState)state);
    return true;
}

bool set_target_velocity_cs(tumble::World *world, ConstraintId id, const Float3 &velocity) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_target_velocity_cs(*constraint, vec3f::load(velocity));
    return true;
}

bool set_target_angular_velocity_cs(tumble::World *world, ConstraintId id, const Float3 &velocity) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_target_angular_velocity_cs(*constraint, vec3f::load(velocity));
    return true;
}

bool set_target_position_cs(tumble::World *world, ConstraintId id, const Float3 &position) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_target_position_cs(*constraint, vec3f::load(position));
    return true;
}

bool set_target_orientation_cs(tumble::World *world, ConstraintId id, const Float4 &orientation) {
    SixDOFConstraint *constraint = found(tumble::six_dof_constraint::get(*world, id));
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_target_orientation_cs(*constraint, quatf::load(orientation));
    return true;
}

bool set_max_friction(tumble::World *world, ConstraintId id, uint32_t axis, float friction) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::SixDOFConstraint::Axis::NUM);
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_max_friction(*constraint, axis, friction);
    return true;
}

bool make_free_axis(tumble::World *world, ConstraintId id, uint32_t axis) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::SixDOFConstraint::Axis::NUM);
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::make_free_axis(*constraint, axis);
    return true;
}

bool make_fixed_axis(tumble::World *world, ConstraintId id, uint32_t axis) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::SixDOFConstraint::Axis::NUM);
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::make_fixed_axis(*constraint, axis);
    return true;
}

bool set_limited_axis(tumble::World *world, ConstraintId id, uint32_t axis, float min, float max) {
    SixDOFConstraint *constraint = found_axis(world, id, axis, tumble::SixDOFConstraint::Axis::NUM);
    if (constraint == nullptr) return false;
    tumble::six_dof_constraint::set_limited_axis(*constraint, axis, min, max);
    return true;
}

}  // namespace six_dof_constraint

namespace filter {

/** whether the tables hold every layer: a fill that could not grow leaves them short, with the error set */
static bool whole(const tumble::Filter &filter, const Layers &layers) {
    return filter.enabled_object_layers.length == layers.object_layers && filter.enabled_broadphase_layers.length == layers.broadphase_layers;
}

tumble::Filter *create(tumble::World *world) {
    tumble::Filter *filter = util::create<tumble::Filter>();
    *filter = tumble::filter::create(world->settings.layers);
    if (!whole(*filter, world->settings.layers)) {
        destroy(filter);
        return nullptr;
    }
    return filter;
}

void destroy(tumble::Filter *filter) {
    util::destroy(filter);
}

void enable_all_layers(tumble::Filter *filter, tumble::World *world) {
    tumble::filter::enable_all_layers(*filter, world->settings.layers);
}

void disable_all_layers(tumble::Filter *filter, tumble::World *world) {
    tumble::filter::disable_all_layers(*filter, world->settings.layers);
}

/** the filter's tables are the world's size, and the layer is one of them */
static bool fits(const tumble::Filter *filter, const tumble::World *world, uint32_t layer, uint32_t layers) {
    return whole(*filter, world->settings.layers) && layer < layers;
}

bool enable_object_layer(tumble::Filter *filter, tumble::World *world, uint32_t object_layer) {
    if (!fits(filter, world, object_layer, world->settings.layers.object_layers)) return false;
    tumble::filter::enable_object_layer(*filter, world->settings.layers, object_layer);
    return true;
}

bool disable_object_layer(tumble::Filter *filter, tumble::World *world, uint32_t object_layer) {
    if (!fits(filter, world, object_layer, world->settings.layers.object_layers)) return false;
    tumble::filter::disable_object_layer(*filter, world->settings.layers, object_layer);
    return true;
}

bool enable_broadphase_layer(tumble::Filter *filter, tumble::World *world, uint32_t broadphase_layer) {
    if (!fits(filter, world, broadphase_layer, world->settings.layers.broadphase_layers)) return false;
    tumble::filter::enable_broadphase_layer(*filter, world->settings.layers, broadphase_layer);
    return true;
}

bool disable_broadphase_layer(tumble::Filter *filter, tumble::World *world, uint32_t broadphase_layer) {
    if (!fits(filter, world, broadphase_layer, world->settings.layers.broadphase_layers)) return false;
    tumble::filter::disable_broadphase_layer(*filter, world->settings.layers, broadphase_layer);
    return true;
}

bool is_object_layer_enabled(const tumble::Filter *filter, uint32_t object_layer) {
    return tumble::filter::filter_object_layer(*filter, object_layer);
}

bool is_broadphase_layer_enabled(const tumble::Filter *filter, uint32_t broadphase_layer) {
    return tumble::filter::filter_broadphase_layer(*filter, broadphase_layer);
}

bool set_from_body(tumble::Filter *filter, tumble::World *world, BodyId body) {
    const tumble::RigidBody *found = rigid_body::resolve(world, body);
    if (found == nullptr || !whole(*filter, world->settings.layers)) return false;
    tumble::filter::set_from_body(*filter, world->settings.layers, found);
    return true;
}

uint32_t get_collision_groups(const tumble::Filter *filter) { return filter->collision_groups; }
uint32_t get_collision_mask(const tumble::Filter *filter) { return filter->collision_mask; }

void copy(tumble::Filter *out, const tumble::Filter *source) { tumble::filter::copy(*out, *source); }

}  // namespace filter

namespace query {

namespace {

/**
 * one collector per query type, reused, its hits growing to the most a query has held; each query sets its kind and
 * resets it. with what a query without a filter of its own hits, refilled for the world each time
 */
struct Scratch {
    CastRayCollector ray_collector;
    CastShapeCollector cast_shape_collector;
    CollideShapeCollector collide_shape_collector;
    CollidePointCollector collide_point_collector;
    tumble::Filter every_layer;
};

}  // namespace

// made on the first query and kept for the module's life: a static of the struct itself would need a destructor run at
// exit, which a module without an entry point never does
static Scratch *the_scratch = nullptr;

static Scratch &scratch() {
    if (the_scratch == nullptr) the_scratch = util::create<Scratch>();
    return *the_scratch;
}

/** ALL, ANY or CLOSEST: a CUSTOM collector's function pointers are not js's to give */
static bool built_in(uint32_t kind) { return kind <= (uint32_t)CollectorKind::CLOSEST; }

/** the filter with the query's group and mask; a null filter enables every layer of the world */
static const tumble::Filter &with_groups(const tumble::World &world, tumble::Filter *filter, uint32_t collision_groups,
                                                 uint32_t collision_mask) {
    if (filter == nullptr) {
        tumble::filter::set_all_enabled(scratch().every_layer, world.settings.layers);
        filter = &scratch().every_layer;
    }
    filter->collision_groups = collision_groups;
    filter->collision_mask = collision_mask;
    return *filter;
}

static Transformf transform_of(const Float3 &position, const Float4 &quaternion, const Float3 &scale) { return {position, quaternion, scale}; }

void default_cast_ray_settings(CastRaySettings &out) { out = cast_ray::default_settings(); }

uint32_t cast_ray(tumble::World *world, uint32_t kind, float early_out_fraction, const CastRaySettings &settings,
                  const Float3 &origin, const Float3 &direction, float length, tumble::Filter *filter, uint32_t collision_groups,
                  uint32_t collision_mask) {
    if (!built_in(kind)) return 0;
    scratch().ray_collector.kind = (CollectorKind)kind;
    cast_ray::reset(scratch().ray_collector);
    scratch().ray_collector.early_out_fraction = early_out_fraction;
    const Rayf ray = {vec3f::load(origin), vec3f::load(direction), length};
    tumble::query::cast_ray(*world, scratch().ray_collector, settings, ray, with_groups(*world, filter, collision_groups, collision_mask));
    return scratch().ray_collector.hits.length;
}

bool read_cast_ray_hit(CastRayHit &out, uint32_t index) {
    if (index >= scratch().ray_collector.hits.length) return false;
    out = scratch().ray_collector.hits[index];
    return true;
}

void default_cast_shape_settings(CastShapeSettings &out) {
    const tumble::CastShapeSettings d = cast_shape::default_settings();
    out = {d.collide_with_backfaces_triangles,
           d.collide_with_backfaces_convex,
           d.collision_tolerance,
           d.penetration_tolerance,
           d.return_deepest_point,
           d.use_shrunken_shape_and_convex_radius,
           d.collide_only_with_active_edges,
           {0.0f, 0.0f, 0.0f},
           d.extra_convex_radius};
    vec3f::store(out.active_edge_movement_direction, d.active_edge_movement_direction);
}

uint32_t cast_shape(tumble::World *world, uint32_t kind, float early_out_fraction, const CastShapeSettings &s,
                    const Shape *shape, const Float3 &position, const Float4 &quaternion, const Float3 &scale,
                    const Float3 &displacement, tumble::Filter *filter, uint32_t collision_groups, uint32_t collision_mask) {
    if (!built_in(kind)) return 0;
    tumble::CastShapeSettings settings = cast_shape::default_settings();
    settings.collide_with_backfaces_triangles = s.collide_with_backfaces_triangles;
    settings.collide_with_backfaces_convex = s.collide_with_backfaces_convex;
    settings.collect_faces = false;
    settings.collision_tolerance = s.collision_tolerance;
    settings.penetration_tolerance = s.penetration_tolerance;
    settings.return_deepest_point = s.return_deepest_point;
    settings.use_shrunken_shape_and_convex_radius = s.use_shrunken_shape_and_convex_radius;
    settings.collide_only_with_active_edges = s.collide_only_with_active_edges;
    settings.active_edge_movement_direction = vec3f::load(s.active_edge_movement_direction);
    settings.extra_convex_radius = s.extra_convex_radius;
    scratch().cast_shape_collector.kind = (CollectorKind)kind;
    cast_shape::reset(scratch().cast_shape_collector);
    scratch().cast_shape_collector.early_out_fraction = early_out_fraction;
    tumble::query::cast_shape(*world, scratch().cast_shape_collector, settings, shape, transform_of(position, quaternion, scale), vec3f::load(displacement),
                              with_groups(*world, filter, collision_groups, collision_mask));
    return scratch().cast_shape_collector.hits.length;
}

bool read_cast_shape_hit(CastShapeHit &out, uint32_t index) {
    if (index >= scratch().cast_shape_collector.hits.length) return false;
    const tumble::CastShapeHit &hit = scratch().cast_shape_collector.hits[index];
    out.status = (uint32_t)hit.status;
    out.fraction = hit.fraction;
    vec3f::store(out.point_a, hit.point_a);
    vec3f::store(out.point_b, hit.point_b);
    out.penetration_depth = hit.penetration_depth;
    vec3f::store(out.penetration_axis, hit.penetration_axis);
    vec3f::store(out.normal, hit.normal);
    out.sub_shape_id_a = hit.sub_shape_id_a;
    out.sub_shape_id_b = hit.sub_shape_id_b;
    out.body_id_b = hit.body_id_b;
    return true;
}

void default_collide_shape_settings(CollideShapeSettings &out) {
    const tumble::CollideShapeSettings d = collide_shape::default_settings();
    out = {d.max_separation_distance,
           d.collision_tolerance,
           d.penetration_tolerance,
           d.collide_with_backfaces,
           d.collide_only_with_active_edges,
           {0.0f, 0.0f, 0.0f},
           d.internal_edge_removal_vertex_tolerance_sq};
    vec3f::store(out.active_edge_movement_direction, d.active_edge_movement_direction);
}

uint32_t collide_shape(tumble::World *world, uint32_t kind, float early_out_fraction, const CollideShapeSettings &s,
                       const Shape *shape, const Float3 &position, const Float4 &quaternion, const Float3 &scale,
                       bool internal_edge_removal, tumble::Filter *filter, uint32_t collision_groups, uint32_t collision_mask) {
    if (!built_in(kind)) return 0;
    tumble::CollideShapeSettings settings = collide_shape::default_settings();
    settings.max_separation_distance = s.max_separation_distance;
    settings.collision_tolerance = s.collision_tolerance;
    settings.penetration_tolerance = s.penetration_tolerance;
    settings.collide_with_backfaces = s.collide_with_backfaces;
    settings.collide_only_with_active_edges = s.collide_only_with_active_edges;
    settings.active_edge_movement_direction = vec3f::load(s.active_edge_movement_direction);
    settings.collect_faces = false;
    settings.internal_edge_removal_vertex_tolerance_sq = s.internal_edge_removal_vertex_tolerance_sq;
    scratch().collide_shape_collector.kind = (CollectorKind)kind;
    collide_shape::reset(scratch().collide_shape_collector);
    scratch().collide_shape_collector.early_out_fraction = early_out_fraction;
    const Transformf transform = transform_of(position, quaternion, scale);
    const tumble::Filter &query_filter = with_groups(*world, filter, collision_groups, collision_mask);
    if (internal_edge_removal) {
        tumble::query::collide_shape_with_internal_edge_removal(*world, scratch().collide_shape_collector, settings, shape, transform, query_filter);
    } else {
        tumble::query::collide_shape(*world, scratch().collide_shape_collector, settings, shape, transform, query_filter);
    }
    return scratch().collide_shape_collector.hits.length;
}

bool read_collide_shape_hit(CollideShapeHit &out, uint32_t index) {
    if (index >= scratch().collide_shape_collector.hits.length) return false;
    const tumble::CollideShapeHit &hit = scratch().collide_shape_collector.hits[index];
    vec3f::store(out.point_a, hit.point_a);
    vec3f::store(out.point_b, hit.point_b);
    vec3f::store(out.penetration_axis, hit.penetration_axis);
    out.penetration = hit.penetration;
    out.sub_shape_id_a = hit.sub_shape_id_a;
    out.sub_shape_id_b = hit.sub_shape_id_b;
    out.body_id_b = hit.body_id_b;
    return true;
}

void default_collide_point_settings(CollidePointSettings &out) { out = collide_point::default_settings(); }

uint32_t collide_point(tumble::World *world, uint32_t kind, const CollidePointSettings &settings, const Float3 &point,
                       tumble::Filter *filter, uint32_t collision_groups, uint32_t collision_mask) {
    if (!built_in(kind)) return 0;
    scratch().collide_point_collector.kind = (CollectorKind)kind;
    collide_point::reset(scratch().collide_point_collector);
    tumble::query::collide_point(*world, scratch().collide_point_collector, settings, vec3f::load(point), with_groups(*world, filter, collision_groups, collision_mask));
    return scratch().collide_point_collector.hits.length;
}

bool read_collide_point_hit(CollidePointHit &out, uint32_t index) {
    if (index >= scratch().collide_point_collector.hits.length) return false;
    out = scratch().collide_point_collector.hits[index];
    return true;
}

}  // namespace query

namespace kcc_policy {

void default_body_policy(tumble::KCC::BodyPolicy &out) { out = {}; }

tumble::KCC::Policy *create() {
    tumble::KCC::Policy *policy = util::create<tumble::KCC::Policy>();
    if (policy != nullptr) *policy = tumble::kcc_policy::create();
    return policy;
}

void destroy(tumble::KCC::Policy *policy) {
    util::destroy(policy);
}

void get_defaults(tumble::KCC::BodyPolicy &out, const tumble::KCC::Policy *policy) { out = policy->defaults; }
void set_defaults(tumble::KCC::Policy *policy, const tumble::KCC::BodyPolicy &body_policy) { policy->defaults = body_policy; }

void set(tumble::KCC::Policy *policy, BodyId body, const tumble::KCC::BodyPolicy &body_policy) {
    tumble::kcc_policy::set(*policy, body, body_policy);
}

bool remove(tumble::KCC::Policy *policy, BodyId body) { return tumble::kcc_policy::remove(*policy, body); }

void get(tumble::KCC::BodyPolicy &out, const tumble::KCC::Policy *policy, BodyId body) {
    out = tumble::kcc_policy::get(policy, body);
}

}  // namespace kcc_policy

namespace kcc {

using tumble::KCC;

static Float3 float3_of(Vec3f v) {
    Float3 out;
    return vec3f::store(out, v);
}

/** the policy's listener for the length of a call; none for no policy, which treats every body as the defaults do */
struct PolicyListener {
    tumble::KCC::Listener listener;
    const tumble::KCC::Listener *pointer;
    explicit PolicyListener(const tumble::KCC::Policy *policy)
        : listener(policy != nullptr ? tumble::kcc_policy::listener(*policy) : tumble::KCC::Listener{}),
          pointer(policy != nullptr ? &listener : nullptr) {}
    PolicyListener(const PolicyListener &) = delete;
};

void default_settings(KCCSettings &out) {
    const tumble::KCCSettings d;
    out = {d.shape, d.shape_offset, d.mass, d.max_strength, d.up, d.max_slope_angle, d.predictive_contact_distance, d.character_padding,
           d.max_collision_iterations, d.max_constraint_iterations, d.collision_tolerance, d.penetration_recovery_speed, d.min_time_remaining,
           d.max_num_hits, d.hit_reduction_cos_max_angle, (uint32_t)d.back_face_mode, d.enhanced_internal_edge_removal, d.has_supporting_volume_plane,
           d.supporting_volume_plane, (uint32_t)d.user_data, (uint32_t)(d.user_data >> 32), d.observes_contacts, d.inner_rigid_body_shape,
           d.inner_rigid_body_object_layer};
}

void default_update_settings(tumble::KCCUpdateSettings &out) { out = {}; }

KCC *create(const KCCSettings &s, const Float3 &position, const Float4 &quaternion) {
    tumble::KCCSettings settings;
    settings.shape = s.shape;
    settings.shape_offset = s.shape_offset;
    settings.mass = s.mass;
    settings.max_strength = s.max_strength;
    settings.up = s.up;
    settings.max_slope_angle = s.max_slope_angle;
    settings.predictive_contact_distance = s.predictive_contact_distance;
    settings.character_padding = s.character_padding;
    settings.max_collision_iterations = s.max_collision_iterations;
    settings.max_constraint_iterations = s.max_constraint_iterations;
    settings.collision_tolerance = s.collision_tolerance;
    settings.penetration_recovery_speed = s.penetration_recovery_speed;
    settings.min_time_remaining = s.min_time_remaining;
    settings.max_num_hits = s.max_num_hits;
    settings.hit_reduction_cos_max_angle = s.hit_reduction_cos_max_angle;
    settings.back_face_mode = (tumble::KCC::BackFaceMode)s.back_face_mode;
    settings.enhanced_internal_edge_removal = s.enhanced_internal_edge_removal;
    settings.has_supporting_volume_plane = s.has_supporting_volume_plane;
    settings.supporting_volume_plane = s.supporting_volume_plane;
    settings.user_data = (uint64_t)s.user_data_high << 32 | s.user_data_low;
    settings.observes_contacts = s.observes_contacts;
    settings.inner_rigid_body_shape = s.inner_rigid_body_shape;
    settings.inner_rigid_body_object_layer = s.inner_rigid_body_object_layer;
    KCC *character = tumble::kcc::create(settings, vec3f::load(position), quatf::load(quaternion));
    tumble::add_ref(character);
    return character;
}

void release(const KCC *character) { tumble::release_ref(character); }
void add(tumble::World *world, KCC *character) { tumble::kcc::add(*world, *character); }
void remove(tumble::World *world, KCC *character) { tumble::kcc::remove(*world, *character); }

void read_state(KCCState &out, const KCC *character) {
    const tumble::KCC::GroundInfo &ground = character->ground;
    out.position = float3_of(character->position);
    quatf::store(out.quaternion, character->quaternion);
    out.linear_velocity = float3_of(character->linear_velocity);
    out.up = float3_of(character->up);
    out.ground_state = (uint32_t)ground.state;
    out.ground_normal = float3_of(ground.normal);
    out.ground_velocity = float3_of(ground.velocity);
    out.ground_body_id = ground.body_id;
    out.ground_sub_shape_id = ground.sub_shape_id;
    out.ground_position = float3_of(ground.position);
    out.shape = character->shape;
    out.inner_rigid_body_id = character->inner_rigid_body_id;
    out.contact_count = character->contacts.length;
    out.contact_event_count = character->contact_events.length;
}

void set_linear_velocity(KCC *character, const Float3 &velocity) { character->linear_velocity = vec3f::load(velocity); }

void get_tuning(KCCTuning &out, const KCC *character) {
    const KCC &c = *character;
    out = {float3_of(c.shape_offset), c.mass, c.max_strength, float3_of(c.up), c.max_slope_angle, c.predictive_contact_distance,
           c.character_padding, c.max_collision_iterations, c.max_constraint_iterations, c.collision_tolerance, c.penetration_recovery_speed,
           c.min_time_remaining, c.max_num_hits, c.hit_reduction_cos_max_angle, (uint32_t)c.back_face_mode, c.enhanced_internal_edge_removal,
           {vec4f::x(c.supporting_volume_plane), vec4f::y(c.supporting_volume_plane), vec4f::z(c.supporting_volume_plane),
            vec4f::w(c.supporting_volume_plane)}};
}

void set_tuning(KCC *character, const KCCTuning &t) {
    KCC &c = *character;
    c.shape_offset = vec3f::load(t.shape_offset);
    c.mass = t.mass;
    c.max_strength = t.max_strength;
    c.up = vec3f::normalize(vec3f::load(t.up));
    c.max_slope_angle = t.max_slope_angle;
    c.cos_max_slope_angle = scalar::cos(t.max_slope_angle);
    c.predictive_contact_distance = t.predictive_contact_distance;
    c.character_padding = t.character_padding;
    c.max_collision_iterations = t.max_collision_iterations;
    c.max_constraint_iterations = t.max_constraint_iterations;
    c.collision_tolerance = t.collision_tolerance;
    c.penetration_recovery_speed = t.penetration_recovery_speed;
    c.min_time_remaining = t.min_time_remaining;
    c.max_num_hits = t.max_num_hits;
    c.hit_reduction_cos_max_angle = t.hit_reduction_cos_max_angle;
    c.back_face_mode = (tumble::KCC::BackFaceMode)t.back_face_mode;
    c.enhanced_internal_edge_removal = t.enhanced_internal_edge_removal;
    c.supporting_volume_plane = vec4f::make(t.supporting_volume_plane.x, t.supporting_volume_plane.y, t.supporting_volume_plane.z,
                                            t.supporting_volume_plane.w);
}

uint8_t *contacts(KCC *character) { return reinterpret_cast<uint8_t *>(character->contacts.data); }

bool read_contact_event(KCCContactEvent &out, const KCC *character, uint32_t index) {
    if (index >= character->contact_events.length) return false;
    const tumble::KCC::ContactEvent &event = character->contact_events[index];
    out = {(uint32_t)event.kind, event.body_id, event.sub_shape_id, float3_of(event.position), float3_of(event.normal)};
    return true;
}

void set_position(tumble::World *world, KCC *character, const Float3 &position) {
    tumble::kcc::set_position(*world, *character, vec3f::load(position));
}

void set_quaternion(tumble::World *world, KCC *character, const Float4 &quaternion) {
    tumble::kcc::set_quaternion(*world, *character, quatf::load(quaternion));
}

bool is_slope_too_steep(const KCC *character, const Float3 &normal) { return tumble::kcc::is_slope_too_steep(*character, vec3f::load(normal)); }

void get_center_of_mass_transform(Float3 &out_position, Float4 &out_quaternion, const KCC *character) {
    Vec3f position;
    Quatf quaternion;
    tumble::kcc::get_center_of_mass_transform(*character, position, quaternion);
    vec3f::store(out_position, position);
    quatf::store(out_quaternion, quaternion);
}

bool has_collided_with(const KCC *character, BodyId body) { return tumble::kcc::has_collided_with(*character, body); }

bool can_walk_stairs(const KCC *character, const Float3 &velocity) { return tumble::kcc::can_walk_stairs(*character, vec3f::load(velocity)); }

void cancel_velocity_towards_steep_slopes(Float3 &out, const KCC *character, const Float3 &velocity) {
    out = float3_of(tumble::kcc::cancel_velocity_towards_steep_slopes(*character, vec3f::load(velocity)));
}

void update_ground_velocity(tumble::World *world, KCC *character, const tumble::KCC::Policy *policy) {
    tumble::kcc::update_ground_velocity(*world, *character, PolicyListener(policy).pointer);
}

void refresh_contacts(tumble::World *world, KCC *character, tumble::Filter *filter, uint32_t collision_groups,
                      uint32_t collision_mask, const tumble::KCC::Policy *policy) {
    tumble::kcc::refresh_contacts(*world, *character, query::with_groups(*world, filter, collision_groups, collision_mask), PolicyListener(policy).pointer);
}

bool set_shape(tumble::World *world, KCC *character, Shape *shape, tumble::Filter *filter, uint32_t collision_groups,
               uint32_t collision_mask, const tumble::KCC::Policy *policy, float max_penetration_depth) {
    return tumble::kcc::set_shape(*world, *character, shape, query::with_groups(*world, filter, collision_groups, collision_mask), PolicyListener(policy).pointer,
                                  max_penetration_depth);
}

void move(tumble::World *world, KCC *character, float delta_time, const Float3 &gravity, const tumble::KCC::Policy *policy,
          tumble::Filter *filter, uint32_t collision_groups, uint32_t collision_mask) {
    tumble::kcc::move(*world, *character, delta_time, vec3f::load(gravity), PolicyListener(policy).pointer, query::with_groups(*world, filter, collision_groups, collision_mask));
}

bool stick_to_floor(tumble::World *world, KCC *character, const Float3 &step_down, tumble::Filter *filter,
                    uint32_t collision_groups, uint32_t collision_mask, const tumble::KCC::Policy *policy) {
    return tumble::kcc::stick_to_floor(*world, *character, vec3f::load(step_down), query::with_groups(*world, filter, collision_groups, collision_mask),
                                       PolicyListener(policy).pointer);
}

bool walk_stairs(tumble::World *world, KCC *character, float delta_time, const Float3 &step_up, const Float3 &step_forward,
                 const Float3 &step_forward_test, const Float3 &step_down_extra, tumble::Filter *filter, uint32_t collision_groups,
                 uint32_t collision_mask, const tumble::KCC::Policy *policy) {
    return tumble::kcc::walk_stairs(*world, *character, delta_time, vec3f::load(step_up), vec3f::load(step_forward), vec3f::load(step_forward_test),
                                    vec3f::load(step_down_extra), query::with_groups(*world, filter, collision_groups, collision_mask), PolicyListener(policy).pointer);
}

void update(tumble::World *world, KCC *character, float delta_time, const Float3 &gravity, const tumble::KCCUpdateSettings &settings,
            const tumble::KCC::Policy *policy, tumble::Filter *filter, uint32_t collision_groups, uint32_t collision_mask) {
    tumble::kcc::update(*world, *character, delta_time, vec3f::load(gravity), settings, PolicyListener(policy).pointer,
                        query::with_groups(*world, filter, collision_groups, collision_mask));
}

void collide(tumble::World *world, KCC *character, const Float3 &position, const Float3 &movement_direction, tumble::Filter *filter,
             uint32_t collision_groups, uint32_t collision_mask, const tumble::KCC::Policy *policy) {
    tumble::kcc::collide(*world, *character, vec3f::load(position), vec3f::load(movement_direction),
                         query::with_groups(*world, filter, collision_groups, collision_mask), PolicyListener(policy).pointer);
}

void solve(KCCSolve &out, tumble::World *world, KCC *character, const Float3 &velocity, float delta_time, float time_remaining,
           const tumble::KCC::Policy *policy) {
    Vec3f solved = vec3f::load(velocity);
    Vec3f displacement;
    out.time_simulated = tumble::kcc::solve(*world, *character, solved, delta_time, time_remaining, PolicyListener(policy).pointer, displacement);
    out.velocity = float3_of(solved);
    out.displacement = float3_of(displacement);
}

bool cast(KCCContact &out, tumble::World *world, KCC *character, const Float3 &position, const Float3 &displacement,
          tumble::Filter *filter, uint32_t collision_groups, uint32_t collision_mask, const tumble::KCC::Policy *policy) {
    tumble::KCC::Contact hit;
    if (!tumble::kcc::cast(hit, *world, *character, vec3f::load(position), vec3f::load(displacement),
                           query::with_groups(*world, filter, collision_groups, collision_mask), PolicyListener(policy).pointer)) {
        return false;
    }
    out = {float3_of(hit.position), float3_of(hit.linear_velocity), float3_of(hit.contact_normal), float3_of(hit.surface_normal), hit.distance,
           hit.fraction, hit.body_id, hit.sub_shape_id, hit.material_id, (uint32_t)hit.motion_type, hit.is_sensor};
    return true;
}

void update_ground(tumble::World *world, KCC *character, float delta_time, bool skip_contact_velocity_check,
                   const tumble::KCC::Policy *policy) {
    tumble::kcc::update_ground(*world, *character, delta_time, skip_contact_velocity_check, PolicyListener(policy).pointer);
}

void apply_weight(tumble::World *world, KCC *character, const Float3 &gravity, float delta_time) {
    tumble::kcc::apply_weight(*world, *character, vec3f::load(gravity), delta_time);
}

void begin_contact_tracking(KCC *character) { tumble::kcc::begin_contact_tracking(*character); }
void end_contact_tracking(KCC *character) { tumble::kcc::end_contact_tracking(*character, nullptr); }

}  // namespace kcc

namespace debug {

tumble::DebugDraw *create() { return util::create<tumble::DebugDraw>(); }
void destroy(tumble::DebugDraw *buffers) { util::destroy(buffers); }
void clear(tumble::DebugDraw *buffers) { tumble::debug::clear(*buffers); }
const float *lines(const tumble::DebugDraw *buffers) { return buffers->lines.data; }
uint32_t line_count(const tumble::DebugDraw *buffers) { return buffers->line_count; }
const float *triangles(const tumble::DebugDraw *buffers) { return buffers->triangles.data; }
uint32_t triangle_count(const tumble::DebugDraw *buffers) { return buffers->triangle_count; }

void default_bodies_options(tumble::DebugDraw::BodiesOptions &out) { out = {}; }
void default_body_options(tumble::DebugDraw::BodyOptions &out) { out = {}; }
void default_shape_options(tumble::DebugDraw::ShapeOptions &out) { out = {}; }
void default_joints_options(tumble::DebugDraw::JointsOptions &out) { out = {}; }

void bodies(tumble::DebugDraw *out, const tumble::World *world, const tumble::DebugDraw::BodiesOptions &options) {
    tumble::debug::bodies(*out, *world, options);
}

bool body(tumble::DebugDraw *out, tumble::World *world, BodyId body, const tumble::DebugDraw::BodyOptions &options) {
    const tumble::RigidBody *found = rigid_body::resolve(world, body);
    if (found == nullptr) return false;
    tumble::debug::body(*out, *found, options);
    return true;
}

void line(tumble::DebugDraw *out, const Float3 &a, const Float3 &b, const Float3 &color) {
    tumble::debug::line(*out, vec3f::load(a), vec3f::load(b), vec3f::load(color));
}

void shape(tumble::DebugDraw *out, const Shape *shape, const Float3 &position, const Float4 &quaternion,
           const tumble::DebugDraw::ShapeOptions &options) {
    tumble::debug::shape(*out, shape, vec3f::load(position), quatf::load(quaternion), options);
}

void contacts(tumble::DebugDraw *out, const tumble::World *world) { tumble::debug::contacts(*out, *world); }
void character_contacts(tumble::DebugDraw *out, const tumble::KCC *character) { tumble::debug::character_contacts(*out, *character); }
void contact_constraints(tumble::DebugDraw *out, const tumble::World *world) { tumble::debug::contact_constraints(*out, *world); }

void joints(tumble::DebugDraw *out, const tumble::World *world, const tumble::DebugDraw::JointsOptions &options) {
    tumble::debug::joints(*out, *world, options);
}

}  // namespace debug

namespace contact_pairs {

void begin(tumble::World *world) { tumble::contact_pairs::begin(*world); }

void record_world_contacts(tumble::World *world, uint32_t first_added, uint32_t first_persisted) {
    tumble::contact_pairs::record_world_contacts(*world, first_added, first_persisted);
}

void record_character_contacts(tumble::World *world, const tumble::KCC *character) {
    tumble::contact_pairs::record_character_contacts(*world, *character);
}

void end(tumble::World *world) { tumble::contact_pairs::end(*world); }

static void read_pair(ContactPair &out, const tumble::World *world, uint32_t handle) {
    const tumble::ContactPair &pair = world->contact_pairs.pairs[handle];
    out = {handle,
           pair.a.body_id,
           pair.a.sub_shape_id,
           (uint32_t)pair.a.user_data,
           (uint32_t)(pair.a.user_data >> 32),
           pair.a.is_sensor,
           pair.b.body_id,
           pair.b.sub_shape_id,
           (uint32_t)pair.b.user_data,
           (uint32_t)(pair.b.user_data >> 32),
           pair.b.is_sensor,
           pair.point,
           pair.normal,
           pair.penetration_depth,
           pair.relative_velocity,
           pair.added};
}

uint32_t active_count(const tumble::World *world) { return world->contact_pairs.active.length; }

bool read_active(ContactPair &out, const tumble::World *world, uint32_t index) {
    if (index >= world->contact_pairs.active.length) return false;
    read_pair(out, world, world->contact_pairs.active[index]);
    return true;
}

uint32_t removed_count(const tumble::World *world) { return world->contact_pairs.removed.length; }

bool read_removed(ContactPair &out, const tumble::World *world, uint32_t index) {
    if (index >= world->contact_pairs.removed.length) return false;
    read_pair(out, world, world->contact_pairs.removed[index]);
    return true;
}

}  // namespace contact_pairs

}  // namespace adapters
