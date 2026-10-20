// what the wasm exposes to js: the manifest the schema build turns into js/src/generated/tumble.ts and
// wasm/generated/exports.cpp (see wasm/bind/bind.h). every field of a value that crosses is exposed: the bytes of one
// that is not would be what the scratch held before.

#include "tumble/util/error.h"
#include "wasm/adapters.h"
#include "wasm/bind/bind.h"

using namespace tumble;
using bind::GROWS;
using bind::names;


TUMBLE_BINDINGS {
    bind::include(b, "wasm/adapters.h");

    bind::enumeration<uint32_t>(b, "WasmError")
        .doc("what error() reports: the first failure since the last clear.")
        .value("NONE", util::ERROR_NONE)
        .value("OUT_OF_MEMORY", util::ERROR_OUT_OF_MEMORY, "the memory could not grow for an allocation.")
        .value("BAD_HANDLE", util::ERROR_BAD_HANDLE, "an id or handle named nothing live.")
        .value("QUOTA", util::ERROR_QUOTA, "a pool is at its most.");

    bind::fn(b, BIND(adapters::wasm::init), names("heapBegin"), GROWS);
    bind::fn(b, BIND(adapters::wasm::abi_version));
    bind::fn(b, BIND(adapters::wasm::schema_hash));
    bind::fn(b, BIND(adapters::wasm::simd_check));
    bind::fn(b, BIND(adapters::wasm::error));
    bind::fn(b, BIND(adapters::wasm::error_clear));
    bind::fn(b, BIND(adapters::wasm::alloc_bytes), names("bytes"), GROWS);
    bind::fn(b, BIND(adapters::wasm::free_bytes), names("block", "bytes"));
    bind::fn(b, BIND(adapters::wasm::allocated_bytes));

    TUPLE_AS(Float3, "Vec3").field(FIELD(Float3, x)).field(FIELD(Float3, y)).field(FIELD(Float3, z));
    TUPLE_AS(Float4, "Quat").field(FIELD(Float4, x)).field(FIELD(Float4, y)).field(FIELD(Float4, z)).field(FIELD(Float4, w));

    RECORD(world_settings::WorldSettings).field(FIELD(world_settings::WorldSettings, gravity), bind::WRITABLE);
    RECORD(world::World);
    RECORD_AS(shapes::Shape, "Shape");

    bind::fn(b, BIND(adapters::world_settings::create), GROWS);
    bind::fn(b, BIND(adapters::world_settings::destroy));
    bind::fn(b, BIND(adapters::world_settings::add_broadphase_layer), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::world_settings::add_object_layer), names("settings", "broadphaseLayer"), GROWS);
    bind::fn(b, BIND(adapters::world_settings::enable_collision), names("settings", "objectLayerA", "objectLayerB"));

    bind::fn(b, BIND(adapters::world::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::world::destroy));
    bind::fn(b, BIND(adapters::world::step), names("world", "deltaTime"), GROWS);

    VALUE_AS(adapters::BodySettings, "BodySettings")
        .field(FIELD(adapters::BodySettings, shape))
        .field(FIELD(adapters::BodySettings, object_layer))
        .field(FIELD(adapters::BodySettings, motion_type))
        .field(FIELD(adapters::BodySettings, user_data_low))
        .field(FIELD(adapters::BodySettings, user_data_high))
        .field(FIELD(adapters::BodySettings, position))
        .field(FIELD(adapters::BodySettings, quaternion))
        .field(FIELD(adapters::BodySettings, motion_quality))
        .field(FIELD(adapters::BodySettings, allowed_degrees_of_freedom))
        .field(FIELD(adapters::BodySettings, gravity_factor))
        .field(FIELD(adapters::BodySettings, linear_damping))
        .field(FIELD(adapters::BodySettings, angular_damping))
        .field(FIELD(adapters::BodySettings, max_linear_velocity))
        .field(FIELD(adapters::BodySettings, max_angular_velocity))
        .field(FIELD(adapters::BodySettings, num_velocity_steps_override))
        .field(FIELD(adapters::BodySettings, num_position_steps_override))
        .field(FIELD(adapters::BodySettings, friction))
        .field(FIELD(adapters::BodySettings, restitution))
        .field(FIELD(adapters::BodySettings, friction_combine_mode))
        .field(FIELD(adapters::BodySettings, restitution_combine_mode))
        .field(FIELD(adapters::BodySettings, collision_groups))
        .field(FIELD(adapters::BodySettings, collision_mask))
        .field(FIELD(adapters::BodySettings, use_manifold_reduction))
        .field(FIELD(adapters::BodySettings, allow_sleeping))
        .field(FIELD(adapters::BodySettings, sensor))
        .field(FIELD(adapters::BodySettings, enhanced_internal_edge_removal))
        .field(FIELD(adapters::BodySettings, collide_kinematic_vs_non_dynamic))
        .field(FIELD(adapters::BodySettings, observes_contacts))
        .field(FIELD(adapters::BodySettings, mass));

    bind::fn(b, BIND(adapters::rigid_body::default_settings), names("out"));
    bind::fn(b, BIND(adapters::rigid_body::create), names("world", "settings"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::remove), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_position), names("out", "world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_quaternion), names("out", "world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_linear_velocity), names("out", "world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::set_linear_velocity), names("world", "body", "velocity"));

    VALUE_AS(box::BoxShapeSettings, "BoxSettings")
        .field(FIELD(box::BoxShapeSettings, half_extents))
        .field(FIELD(box::BoxShapeSettings, convex_radius))
        .field(FIELD(box::BoxShapeSettings, density))
        .field(FIELD(box::BoxShapeSettings, material_id));
    VALUE_AS(sphere::SphereShapeSettings, "SphereSettings")
        .field(FIELD(sphere::SphereShapeSettings, radius))
        .field(FIELD(sphere::SphereShapeSettings, density))
        .field(FIELD(sphere::SphereShapeSettings, material_id));
    bind::fn(b, BIND(adapters::box::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::box::destroy), names("shape"));
    bind::fn(b, BIND(adapters::sphere::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::sphere::destroy), names("shape"));
}
