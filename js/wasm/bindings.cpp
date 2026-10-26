// what the wasm exposes to js: the manifest the schema build turns into js/src/generated/tumble.ts and
// js/wasm/generated/exports.cpp (see js/wasm/bind/bind.h). every field of a value that crosses is exposed: the bytes of one
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
        .value("NONE", Error::NONE)
        .value("OUT_OF_MEMORY", Error::OUT_OF_MEMORY, "the memory could not grow for an allocation.")
        .value("BAD_HANDLE", Error::BAD_HANDLE, "an id or handle named nothing live.")
        .value("QUOTA", Error::QUOTA, "a pool is at its most.");

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
    TUPLE_AS(Float4x4, "Mat4").field(FIELD(Float4x4, m));
    VALUE_AS(Box3f, "Box3").field(FIELD(Box3f, min)).field(FIELD(Box3f, max));
    VALUE_AS(MassProperties, "MassProperties")
        .field(FIELD(MassProperties, mass), "kg.")
        .field(FIELD(MassProperties, inertia), "kg m^2, the upper 3x3 of a column-major matrix.");

    VALUE_AS(WorldSettings::Contacts, "ContactsSettings")
        .field(FIELD(WorldSettings::Contacts, contact_point_preserve_lambda_max_dist_sq));
    VALUE_AS(WorldSettings::Narrowphase, "NarrowphaseSettings")
        .field(FIELD(WorldSettings::Narrowphase, collision_tolerance))
        .field(FIELD(WorldSettings::Narrowphase, penetration_tolerance))
        .field(FIELD(WorldSettings::Narrowphase, return_deepest_point))
        .field(FIELD(WorldSettings::Narrowphase, collide_only_with_active_edges))
        .field(FIELD(WorldSettings::Narrowphase, speculative_contact_distance))
        .field(FIELD(WorldSettings::Narrowphase, manifold_tolerance))
        .field(FIELD(WorldSettings::Narrowphase, collide_with_backfaces))
        .field(FIELD(WorldSettings::Narrowphase, use_manifold_reduction))
        .field(FIELD(WorldSettings::Narrowphase, normal_cos_max_delta_rotation))
        .field(FIELD(WorldSettings::Narrowphase, use_body_pair_contact_cache))
        .field(FIELD(WorldSettings::Narrowphase, body_pair_cache_max_delta_position_sq))
        .field(FIELD(WorldSettings::Narrowphase, body_pair_cache_cos_max_delta_rotation_div_2))
        .field(FIELD(WorldSettings::Narrowphase, internal_edge_removal_vertex_tolerance_sq));
    VALUE_AS(WorldSettings::Solver, "SolverSettings")
        .field(FIELD(WorldSettings::Solver, min_velocity_for_restitution))
        .field(FIELD(WorldSettings::Solver, velocity_iterations))
        .field(FIELD(WorldSettings::Solver, position_iterations))
        .field(FIELD(WorldSettings::Solver, warm_starting))
        .field(FIELD(WorldSettings::Solver, penetration_slop))
        .field(FIELD(WorldSettings::Solver, baumgarte_factor))
        .field(FIELD(WorldSettings::Solver, warm_start_impulse_ratio))
        .field(FIELD(WorldSettings::Solver, max_penetration_distance));
    VALUE_AS(WorldSettings::Sleeping, "SleepingSettings")
        .field(FIELD(WorldSettings::Sleeping, allow_sleeping))
        .field(FIELD(WorldSettings::Sleeping, time_before_sleep))
        .field(FIELD(WorldSettings::Sleeping, point_velocity_sleep_threshold));
    VALUE_AS(WorldSettings::Ccd, "CcdSettings")
        .field(FIELD(WorldSettings::Ccd, linear_cast_threshold))
        .field(FIELD(WorldSettings::Ccd, linear_cast_max_penetration))
        .field(FIELD(WorldSettings::Ccd, min_velocity_for_restitution));
    RECORD(WorldSettings)
        .field(FIELD(WorldSettings, gravity), bind::WRITABLE)
        .field(FIELD(WorldSettings, gravity_enabled), bind::WRITABLE)
        .field(FIELD(WorldSettings, contacts), bind::WRITABLE)
        .field(FIELD(WorldSettings, narrowphase), bind::WRITABLE)
        .field(FIELD(WorldSettings, solver), bind::WRITABLE)
        .field(FIELD(WorldSettings, sleeping), bind::WRITABLE)
        .field(FIELD(WorldSettings, ccd), bind::WRITABLE);
    RECORD(World);
    RECORD_AS(Shape, "Shape");

    bind::fn(b, BIND(adapters::world_settings::create), GROWS);
    bind::fn(b, BIND(adapters::world_settings::destroy));
    bind::fn(b, BIND(adapters::world_settings::add_broadphase_layer), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::world_settings::add_object_layer), names("settings", "broadphaseLayer"), GROWS);
    bind::fn(b, BIND(adapters::world_settings::enable_collision), names("settings", "objectLayerA", "objectLayerB"));

    bind::fn(b, BIND(adapters::world::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::world::destroy));
    bind::fn(b, BIND(adapters::world::step), names("world", "deltaTime"), GROWS);
    bind::fn(b, BIND(adapters::world::step_begin), names("world", "deltaTime"), GROWS);
    bind::fn(b, BIND(adapters::world::find_pairs), names("world"), GROWS);
    bind::fn(b, BIND(adapters::world::narrowphase), names("world"), GROWS);
    bind::fn(b, BIND(adapters::world::solve), names("world"), GROWS);
    bind::fn(b, BIND(adapters::world::step_end), names("world"), GROWS);
    bind::fn(b, BIND(adapters::world::woken), names("world"));
    bind::fn(b, BIND(adapters::world::set_materials), names("world", "frictionRestitution", "count"), GROWS);
    bind::fn(b, BIND(adapters::world::get_gravity), names("out", "world"));
    bind::fn(b, BIND(adapters::world::set_gravity), names("world", "gravity"));

    VALUE_AS(ContactSettings, "ContactSettings")
        .field(FIELD(ContactSettings, combined_friction))
        .field(FIELD(ContactSettings, combined_restitution))
        .field(FIELD(ContactSettings, is_sensor))
        .field(FIELD(ContactSettings, inv_mass_scale_1))
        .field(FIELD(ContactSettings, inv_mass_scale_2))
        .field(FIELD(ContactSettings, inv_inertia_scale_1))
        .field(FIELD(ContactSettings, inv_inertia_scale_2))
        .field(FIELD(ContactSettings, relative_linear_surface_velocity))
        .field(FIELD(ContactSettings, relative_angular_surface_velocity));
    VALUE_AS(adapters::PairEvent, "PairEvent")
        .field(FIELD(adapters::PairEvent, body_a))
        .field(FIELD(adapters::PairEvent, body_b))
        .field(FIELD(adapters::PairEvent, user_data_a_low))
        .field(FIELD(adapters::PairEvent, user_data_a_high))
        .field(FIELD(adapters::PairEvent, user_data_b_low))
        .field(FIELD(adapters::PairEvent, user_data_b_high));
    VALUE_AS(adapters::ContactEvent, "ContactEvent")
        .field(FIELD(adapters::ContactEvent, body_a))
        .field(FIELD(adapters::ContactEvent, body_b))
        .field(FIELD(adapters::ContactEvent, user_data_a_low))
        .field(FIELD(adapters::ContactEvent, user_data_a_high))
        .field(FIELD(adapters::ContactEvent, user_data_b_low))
        .field(FIELD(adapters::ContactEvent, user_data_b_high))
        .field(FIELD(adapters::ContactEvent, sub_shape_a))
        .field(FIELD(adapters::ContactEvent, sub_shape_b))
        .field(FIELD(adapters::ContactEvent, material_a))
        .field(FIELD(adapters::ContactEvent, material_b))
        .field(FIELD(adapters::ContactEvent, normal))
        .field(FIELD(adapters::ContactEvent, penetration))
        .field(FIELD(adapters::ContactEvent, num_points))
        .field(FIELD(adapters::ContactEvent, points_a))
        .field(FIELD(adapters::ContactEvent, points_b))
        .field(FIELD(adapters::ContactEvent, settings));
    VALUE_AS(adapters::ContactRemovedEvent, "ContactRemovedEvent")
        .field(FIELD(adapters::ContactRemovedEvent, body_a))
        .field(FIELD(adapters::ContactRemovedEvent, body_b))
        .field(FIELD(adapters::ContactRemovedEvent, sub_shape_a))
        .field(FIELD(adapters::ContactRemovedEvent, sub_shape_b));
    bind::fn(b, BIND(adapters::events::pairs), names("world"));
    bind::fn(b, BIND(adapters::events::read_pair), names("out", "world", "index"));
    bind::fn(b, BIND(adapters::events::contacts_added), names("world"));
    bind::fn(b, BIND(adapters::events::read_contact_added), names("out", "world", "index"));
    bind::fn(b, BIND(adapters::events::contacts_persisted), names("world"));
    bind::fn(b, BIND(adapters::events::read_contact_persisted), names("out", "world", "index"));
    bind::fn(b, BIND(adapters::events::contacts_removed), names("world"));
    bind::fn(b, BIND(adapters::events::read_contact_removed), names("out", "world", "index"));
    bind::fn(b, BIND(adapters::events::reject_pair), names("world", "index"));
    bind::fn(b, BIND(adapters::events::reject_contact_added), names("world", "index"));
    bind::fn(b, BIND(adapters::events::reject_contact_persisted), names("world", "index"));
    bind::fn(b, BIND(adapters::events::change_contact_added), names("world", "index", "settings"));
    bind::fn(b, BIND(adapters::events::change_contact_persisted), names("world", "index", "settings"));

    VALUE_AS(adapters::ActivationEvent, "ActivationEvent")
        .field(FIELD(adapters::ActivationEvent, body))
        .field(FIELD(adapters::ActivationEvent, user_data_low))
        .field(FIELD(adapters::ActivationEvent, user_data_high))
        .field(FIELD(adapters::ActivationEvent, activated));
    bind::fn(b, BIND(adapters::activation::events), names("world"));
    bind::fn(b, BIND(adapters::activation::read_event), names("out", "world", "index"));
    bind::fn(b, BIND(adapters::activation::clear), names("world"));

    bind::fn(b, BIND(adapters::active_bodies::count), names("world"));
    bind::fn(b, BIND(adapters::active_bodies::read), names("world"), GROWS);
    bind::fn(b, BIND(adapters::active_bodies::rows), names("world"));
    bind::constant(b, "ACTIVE_BODY_ROW_STRIDE", sizeof(adapters::ActiveBodyRow));
    bind::constant(b, "ACTIVE_BODY_ROW_BODY", offsetof(adapters::ActiveBodyRow, body));
    bind::constant(b, "ACTIVE_BODY_ROW_POSITION", offsetof(adapters::ActiveBodyRow, position));
    bind::constant(b, "ACTIVE_BODY_ROW_QUATERNION", offsetof(adapters::ActiveBodyRow, quaternion));
    bind::constant(b, "ACTIVE_BODY_ROW_CENTER_OF_MASS_POSITION", offsetof(adapters::ActiveBodyRow, center_of_mass_position));
    bind::constant(b, "ACTIVE_BODY_ROW_LINEAR_VELOCITY", offsetof(adapters::ActiveBodyRow, linear_velocity));
    bind::constant(b, "ACTIVE_BODY_ROW_ANGULAR_VELOCITY", offsetof(adapters::ActiveBodyRow, angular_velocity));

    bind::enumeration<uint32_t>(b, "MotionType")
        .value("STATIC", MotionType::STATIC, "never moves.")
        .value("KINEMATIC", MotionType::KINEMATIC, "moves by its velocity, pushing dynamic bodies aside.")
        .value("DYNAMIC", MotionType::DYNAMIC, "moved by gravity, forces and contacts.");
    bind::enumeration<uint32_t>(b, "MotionQuality")
        .value("DISCRETE", MotionQuality::DISCRETE, "fast, but can tunnel through thin geometry at high speed.")
        .value("LINEAR_CAST", MotionQuality::LINEAR_CAST,
               "cast along the velocity so a fast body cannot tunnel; dearer than discrete.");
    bind::enumeration<uint32_t>(b, "MaterialCombineMode")
        .doc("how two bodies' friction or restitution combine; when the modes differ the more restrictive wins.")
        .value("AVERAGE", MaterialCombineMode::AVERAGE)
        .value("MULTIPLY", MaterialCombineMode::MULTIPLY)
        .value("MIN", MaterialCombineMode::MIN)
        .value("MAX", MaterialCombineMode::MAX)
        .value("GEOMETRIC_MEAN", MaterialCombineMode::GEOMETRIC_MEAN);
    bind::enumeration<uint32_t>(b, "Dof")
        .doc("bits of allowedDegreesOfFreedom: the axes a body may move along and turn about.")
        .value("TRANSLATION_X", dof::TRANSLATION_X)
        .value("TRANSLATION_Y", dof::TRANSLATION_Y)
        .value("TRANSLATION_Z", dof::TRANSLATION_Z)
        .value("ROTATION_X", dof::ROTATION_X)
        .value("ROTATION_Y", dof::ROTATION_Y)
        .value("ROTATION_Z", dof::ROTATION_Z)
        .value("ALL", dof::DOF_ALL)
        .value("TRANSLATION_ONLY", dof::DOF_TRANSLATION_ONLY)
        .value("ROTATION_ONLY", dof::DOF_ROTATION_ONLY);
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
        .field(FIELD(adapters::BodySettings, mass))
        .field(FIELD(adapters::BodySettings, has_mass_properties))
        .field(FIELD(adapters::BodySettings, mass_properties));

    bind::fn(b, BIND(adapters::rigid_body::default_settings), names("out"));
    bind::fn(b, BIND(adapters::rigid_body::create), names("world", "settings"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::remove), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::exists), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_position), names("out", "world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_quaternion), names("out", "world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_transform), names("outPosition", "outQuaternion", "world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_center_of_mass_position), names("out", "world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_linear_velocity), names("out", "world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_angular_velocity), names("out", "world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_velocity_at_point), names("out", "world", "body", "point"));
    bind::fn(b, BIND(adapters::rigid_body::get_aabb), names("out", "world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_mass), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_motion_type), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_object_layer), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_user_data_low), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_user_data_high), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_shape), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::get_material), names("world", "body", "subShapeId"));
    bind::fn(b, BIND(adapters::rigid_body::is_sleeping), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::set_position), names("world", "body", "position", "wake"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::set_quaternion), names("world", "body", "quaternion", "wake"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::set_transform), names("world", "body", "position", "quaternion", "wake"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::set_linear_velocity), names("world", "body", "velocity"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::set_angular_velocity), names("world", "body", "velocity"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::add_linear_velocity), names("world", "body", "velocity"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::add_angular_velocity), names("world", "body", "velocity"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::add_force), names("world", "body", "force", "wake"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::add_torque), names("world", "body", "torque", "wake"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::add_force_at_position), names("world", "body", "force", "position", "wake"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::add_impulse), names("world", "body", "impulse"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::add_impulse_at_position), names("world", "body", "impulse", "position"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::add_angular_impulse), names("world", "body", "impulse"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::set_motion_type), names("world", "body", "motionType", "wake"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::set_object_layer), names("world", "body", "objectLayer"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::set_shape), names("world", "body", "shape", "wake"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::move_kinematic), names("world", "body", "position", "quaternion", "deltaTime"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::sleep), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::wake), names("world", "body"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::set_user_data), names("world", "body", "low", "high"));
    bind::fn(b, BIND(adapters::rigid_body::set_friction), names("world", "body", "friction"));
    bind::fn(b, BIND(adapters::rigid_body::set_restitution), names("world", "body", "restitution"));

    bind::enumeration<uint32_t>(b, "ShapeType")
        .value("SPHERE", ShapeType::SPHERE)
        .value("BOX", ShapeType::BOX)
        .value("CAPSULE", ShapeType::CAPSULE)
        .value("CONVEX_HULL", ShapeType::CONVEX_HULL)
        .value("TRIANGLE_MESH", ShapeType::TRIANGLE_MESH)
        .value("COMPOUND", ShapeType::COMPOUND)
        .value("TRANSFORMED", ShapeType::TRANSFORMED)
        .value("SCALED", ShapeType::SCALED)
        .value("EMPTY", ShapeType::EMPTY)
        .value("CYLINDER", ShapeType::CYLINDER)
        .value("OFFSET_CENTER_OF_MASS", ShapeType::OFFSET_CENTER_OF_MASS)
        .value("PLANE", ShapeType::PLANE)
        .value("STATIC_COMPOUND", ShapeType::STATIC_COMPOUND);
    bind::fn(b, BIND(adapters::shape::get_type), names("shape"));
    bind::fn(b, BIND(adapters::shape::get_local_bounds), names("out", "shape"));
    bind::fn(b, BIND(adapters::shape::get_center_of_mass), names("out", "shape"));
    bind::fn(b, BIND(adapters::shape::get_volume), names("shape"));
    bind::fn(b, BIND(adapters::shape::get_mass_properties), names("out", "shape"));
    bind::fn(b, BIND(adapters::shape::get_ref_count), names("shape"));
    bind::fn(b, BIND(adapters::shape::release), names("shape"));

    VALUE_AS(BoxShapeSettings, "BoxSettings")
        .field(FIELD(BoxShapeSettings, half_extents))
        .field(FIELD(BoxShapeSettings, convex_radius))
        .field(FIELD(BoxShapeSettings, density))
        .field(FIELD(BoxShapeSettings, material_id));
    bind::fn(b, BIND(adapters::box::default_settings), names("out"));
    bind::fn(b, BIND(adapters::box::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::box::update), names("shape", "settings"));
    VALUE_AS(SphereShapeSettings, "SphereSettings")
        .field(FIELD(SphereShapeSettings, radius))
        .field(FIELD(SphereShapeSettings, density))
        .field(FIELD(SphereShapeSettings, material_id));
    bind::fn(b, BIND(adapters::sphere::default_settings), names("out"));
    bind::fn(b, BIND(adapters::sphere::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::sphere::update), names("shape", "settings"));
    VALUE_AS(CapsuleShapeSettings, "CapsuleSettings")
        .field(FIELD(CapsuleShapeSettings, half_height_of_cylinder))
        .field(FIELD(CapsuleShapeSettings, radius))
        .field(FIELD(CapsuleShapeSettings, density))
        .field(FIELD(CapsuleShapeSettings, material_id));
    bind::fn(b, BIND(adapters::capsule::default_settings), names("out"));
    bind::fn(b, BIND(adapters::capsule::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::capsule::update), names("shape", "settings"));
    VALUE_AS(CylinderShapeSettings, "CylinderSettings")
        .field(FIELD(CylinderShapeSettings, half_height))
        .field(FIELD(CylinderShapeSettings, radius))
        .field(FIELD(CylinderShapeSettings, convex_radius))
        .field(FIELD(CylinderShapeSettings, density))
        .field(FIELD(CylinderShapeSettings, material_id));
    bind::fn(b, BIND(adapters::cylinder::default_settings), names("out"));
    bind::fn(b, BIND(adapters::cylinder::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::cylinder::update), names("shape", "settings"));
    VALUE_AS(Plane3f, "Plane3").field(FIELD(Plane3f, normal)).field(FIELD(Plane3f, constant));
    VALUE_AS(PlaneShapeSettings, "PlaneSettings")
        .field(FIELD(PlaneShapeSettings, plane))
        .field(FIELD(PlaneShapeSettings, half_extent))
        .field(FIELD(PlaneShapeSettings, material_id));
    bind::fn(b, BIND(adapters::plane::default_settings), names("out"));
    bind::fn(b, BIND(adapters::plane::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::empty::create), names(), GROWS);

    bind::enumeration<uint32_t>(b, "ConvexHullBuildResult")
        .doc("why a hull build failed.")
        .value("SUCCESS", ConvexHullShape::BuildResult::SUCCESS)
        .value("TOO_FEW_POINTS", ConvexHullShape::BuildResult::TOO_FEW_POINTS, "fewer than 3 points.")
        .value("TOO_FEW_FACES", ConvexHullShape::BuildResult::TOO_FEW_FACES, "the points were nearly coplanar and the hull collapsed.")
        .value("DEGENERATE", ConvexHullShape::BuildResult::DEGENERATE, "the points coincide or lie on a line.")
        .value("INVALID_CONVEX_RADIUS", ConvexHullShape::BuildResult::INVALID_CONVEX_RADIUS, "a negative convex radius.")
        .value("HULL_ERROR_TOO_LARGE", ConvexHullShape::BuildResult::HULL_ERROR_TOO_LARGE,
               "a point lies outside the built hull by more than 4 * max(coplanar distance, hull tolerance).")
        .value("TOO_MANY_POINTS", ConvexHullShape::BuildResult::TOO_MANY_POINTS,
               "more than 256 vertices survived; a larger hull tolerance gives fewer.")
        .value("POINT_WITH_TOO_FEW_FACES", ConvexHullShape::BuildResult::POINT_WITH_TOO_FEW_FACES, "a hull vertex on fewer than 2 faces.");
    bind::enumeration<uint32_t>(b, "BakeSupportAdjacency")
        .doc("whether a hull keeps its vertex neighbours, for faster support queries on big hulls.")
        .value("AUTO", ConvexHullShape::BakeSupportAdjacency::AUTO, "when the hull has many vertices.")
        .value("NEVER", ConvexHullShape::BakeSupportAdjacency::NEVER)
        .value("ALWAYS", ConvexHullShape::BakeSupportAdjacency::ALWAYS);
    VALUE_AS(ConvexHullShapeSettings, "ConvexHullSettings")
        .field(FIELD(ConvexHullShapeSettings, positions))
        .field(FIELD(ConvexHullShapeSettings, num_positions))
        .field(FIELD(ConvexHullShapeSettings, convex_radius))
        .field(FIELD(ConvexHullShapeSettings, hull_tolerance))
        .field(FIELD(ConvexHullShapeSettings, max_error_convex_radius))
        .field(FIELD(ConvexHullShapeSettings, bake_support_adjacency))
        .field(FIELD(ConvexHullShapeSettings, density))
        .field(FIELD(ConvexHullShapeSettings, material_id));
    VALUE_AS(adapters::ConvexHullCreate, "ConvexHullCreate")
        .field(FIELD(adapters::ConvexHullCreate, shape))
        .field(FIELD(adapters::ConvexHullCreate, result));
    bind::fn(b, BIND(adapters::convex_hull::default_settings), names("out"));
    bind::fn(b, BIND(adapters::convex_hull::create), names("out", "settings"), GROWS);

    bind::enumeration<uint32_t>(b, "BvhSplitStrategy")
        .doc("how a mesh's bvh is split while it is built.")
        .value("CENTER", TriangleMeshShape::BvhSplitStrategy::CENTER, "at the midpoint of the centres' bounds: fast, a good balance.")
        .value("AVERAGE", TriangleMeshShape::BvhSplitStrategy::AVERAGE, "at the mean of the centres: fast, can unbalance skewed meshes.")
        .value("SAH", TriangleMeshShape::BvhSplitStrategy::SAH, "by surface area: slow to build, fastest to query, for static meshes.");
    VALUE_AS(TriangleMeshShapeSettings, "TriangleMeshSettings")
        .field(FIELD(TriangleMeshShapeSettings, positions))
        .field(FIELD(TriangleMeshShapeSettings, num_positions))
        .field(FIELD(TriangleMeshShapeSettings, indices))
        .field(FIELD(TriangleMeshShapeSettings, num_indices))
        .field(FIELD(TriangleMeshShapeSettings, material_ids))
        .field(FIELD(TriangleMeshShapeSettings, degenerate_tolerance))
        .field(FIELD(TriangleMeshShapeSettings, bvh_split_strategy))
        .field(FIELD(TriangleMeshShapeSettings, bvh_max_leaf_tris))
        .field(FIELD(TriangleMeshShapeSettings, active_edge_cos_threshold_angle));
    bind::fn(b, BIND(adapters::triangle_mesh::default_settings), names("out"));
    bind::fn(b, BIND(adapters::triangle_mesh::create), names("settings"), GROWS);

    VALUE_AS(CompoundShapeSettings::Child, "CompoundChild")
        .field(FIELD(CompoundShapeSettings::Child, position))
        .field(FIELD(CompoundShapeSettings::Child, quaternion))
        .field(FIELD(CompoundShapeSettings::Child, shape));
    bind::fn(b, BIND(adapters::compound::create), names("children", "count"), GROWS);
    bind::fn(b, BIND(adapters::compound::set_children), names("shape", "children", "count"), GROWS);
    bind::fn(b, BIND(adapters::static_compound::default_bvh_max_leaf_children), names());
    bind::fn(b, BIND(adapters::static_compound::create), names("children", "count", "bvhMaxLeafChildren"), GROWS);

    VALUE_AS(TransformedShapeSettings, "TransformedSettings")
        .field(FIELD(TransformedShapeSettings, shape))
        .field(FIELD(TransformedShapeSettings, position))
        .field(FIELD(TransformedShapeSettings, quaternion));
    bind::fn(b, BIND(adapters::transformed::default_settings), names("out"));
    bind::fn(b, BIND(adapters::transformed::create), names("settings"), GROWS);
    VALUE_AS(ScaledShapeSettings, "ScaledSettings")
        .field(FIELD(ScaledShapeSettings, shape))
        .field(FIELD(ScaledShapeSettings, scale));
    bind::fn(b, BIND(adapters::scaled::default_settings), names("out"));
    bind::fn(b, BIND(adapters::scaled::create), names("settings"), GROWS);
    VALUE_AS(OffsetCenterOfMassShapeSettings, "OffsetCenterOfMassSettings")
        .field(FIELD(OffsetCenterOfMassShapeSettings, shape))
        .field(FIELD(OffsetCenterOfMassShapeSettings, offset));
    bind::fn(b, BIND(adapters::offset_center_of_mass::default_settings), names("out"));
    bind::fn(b, BIND(adapters::offset_center_of_mass::create), names("settings"), GROWS);

    TUPLE_AS(Vec2f, "Vec2").field(FIELD(Vec2f, x)).field(FIELD(Vec2f, y));
    bind::enumeration<uint32_t>(b, "ConstraintType")
        .doc("a constraint's type, encoded in its id.")
        .value("POINT", ConstraintType::POINT)
        .value("DISTANCE", ConstraintType::DISTANCE)
        .value("HINGE", ConstraintType::HINGE)
        .value("SLIDER", ConstraintType::SLIDER)
        .value("FIXED", ConstraintType::FIXED)
        .value("CONE", ConstraintType::CONE)
        .value("SWING_TWIST", ConstraintType::SWING_TWIST)
        .value("SIX_DOF", ConstraintType::SIX_DOF);
    bind::enumeration<uint32_t>(b, "ConstraintSpace")
        .doc("the space a constraint's points and axes are given in.")
        .value("WORLD", ConstraintSpace::WORLD)
        .value("LOCAL", ConstraintSpace::LOCAL, "relative to each body: its centre of mass and orientation.");
    bind::enumeration<uint32_t>(b, "SpringMode")
        .value("FREQUENCY_AND_DAMPING", SpringMode::FREQUENCY_AND_DAMPING,
               "frequencyOrStiffness is the oscillation frequency in hz, damping the damping ratio (0 none, 1 critical).")
        .value("STIFFNESS_AND_DAMPING", SpringMode::STIFFNESS_AND_DAMPING, "frequencyOrStiffness is k and damping is c in F = -k x - c v.");
    bind::enumeration<uint32_t>(b, "MotorState")
        .value("OFF", MotorState::OFF, "free, apart from any friction.")
        .value("VELOCITY", MotorState::VELOCITY, "drives to the target velocity.")
        .value("POSITION", MotorState::POSITION, "drives to the target position through the motor's spring.")
        .value("POSITION_AND_VELOCITY", MotorState::POSITION_AND_VELOCITY, "drives to both targets.");
    bind::enumeration<uint32_t>(b, "SwingType")
        .doc("the shape that limits a swing.")
        .value("CONE", SwingType::CONE, "symmetric about 0.")
        .value("PYRAMID", SwingType::PYRAMID, "asymmetric limits allowed.");
    bind::enumeration<uint32_t>(b, "SixDOFAxis")
        .doc("a six dof constraint's axes, in its constraint space.")
        .value("TRANSLATION_X", SixDOFConstraint::Axis::TRANSLATION_X)
        .value("TRANSLATION_Y", SixDOFConstraint::Axis::TRANSLATION_Y)
        .value("TRANSLATION_Z", SixDOFConstraint::Axis::TRANSLATION_Z)
        .value("ROTATION_X", SixDOFConstraint::Axis::ROTATION_X)
        .value("ROTATION_Y", SixDOFConstraint::Axis::ROTATION_Y)
        .value("ROTATION_Z", SixDOFConstraint::Axis::ROTATION_Z);
    VALUE_AS(SpringSettings, "SpringSettings")
        .field(FIELD(SpringSettings, mode))
        .field(FIELD(SpringSettings, frequency_or_stiffness))
        .field(FIELD(SpringSettings, damping));
    VALUE_AS(MotorSettings, "MotorSettings")
        .field(FIELD(MotorSettings, spring))
        .field(FIELD(MotorSettings, min_force_limit))
        .field(FIELD(MotorSettings, max_force_limit))
        .field(FIELD(MotorSettings, min_torque_limit))
        .field(FIELD(MotorSettings, max_torque_limit));

    bind::fn(b, BIND(adapters::constraint::get_type), names("constraint"));
    bind::fn(b, BIND(adapters::constraint::exists), names("world", "constraint"));
    bind::fn(b, BIND(adapters::constraint::set_enabled), names("world", "constraint", "enabled"));
    bind::fn(b, BIND(adapters::constraint::is_enabled), names("world", "constraint"));
    bind::fn(b, BIND(adapters::constraint::remove), names("world", "constraint"));

    VALUE_AS(PointConstraintSettings, "PointConstraintSettings")
        .field(FIELD(PointConstraintSettings, point_a))
        .field(FIELD(PointConstraintSettings, point_b))
        .field(FIELD(PointConstraintSettings, space))
        .field(FIELD(PointConstraintSettings, constraint_priority))
        .field(FIELD(PointConstraintSettings, num_velocity_steps_override))
        .field(FIELD(PointConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::point_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::point_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::point_constraint::get_total_lambda), names("out", "world", "constraint"));

    VALUE_AS(DistanceConstraintSettings, "DistanceConstraintSettings")
        .field(FIELD(DistanceConstraintSettings, point_a))
        .field(FIELD(DistanceConstraintSettings, point_b))
        .field(FIELD(DistanceConstraintSettings, min_distance))
        .field(FIELD(DistanceConstraintSettings, max_distance))
        .field(FIELD(DistanceConstraintSettings, spring_settings))
        .field(FIELD(DistanceConstraintSettings, space))
        .field(FIELD(DistanceConstraintSettings, constraint_priority))
        .field(FIELD(DistanceConstraintSettings, num_velocity_steps_override))
        .field(FIELD(DistanceConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::distance_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::distance_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::distance_constraint::get_total_lambda), names("world", "constraint"));
    bind::fn(b, BIND(adapters::distance_constraint::set_distance), names("world", "constraint", "minDistance", "maxDistance"));
    bind::fn(b, BIND(adapters::distance_constraint::set_limits_spring_settings), names("world", "constraint", "settings"));

    VALUE_AS(HingeConstraintSettings, "HingeConstraintSettings")
        .field(FIELD(HingeConstraintSettings, point_a))
        .field(FIELD(HingeConstraintSettings, point_b))
        .field(FIELD(HingeConstraintSettings, hinge_axis_a))
        .field(FIELD(HingeConstraintSettings, hinge_axis_b))
        .field(FIELD(HingeConstraintSettings, normal_axis_a))
        .field(FIELD(HingeConstraintSettings, normal_axis_b))
        .field(FIELD(HingeConstraintSettings, space))
        .field(FIELD(HingeConstraintSettings, limits_min))
        .field(FIELD(HingeConstraintSettings, limits_max))
        .field(FIELD(HingeConstraintSettings, limits_spring_settings))
        .field(FIELD(HingeConstraintSettings, max_friction_torque))
        .field(FIELD(HingeConstraintSettings, motor_settings))
        .field(FIELD(HingeConstraintSettings, constraint_priority))
        .field(FIELD(HingeConstraintSettings, num_velocity_steps_override))
        .field(FIELD(HingeConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::hinge_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::hinge_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::hinge_constraint::set_limits), names("world", "constraint", "min", "max"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_limits_spring_settings), names("world", "constraint", "settings"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_max_friction_torque), names("world", "constraint", "torque"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_motor_settings), names("world", "constraint", "settings"));
    bind::fn(b, BIND(adapters::hinge_constraint::get_motor_settings), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_motor_state), names("world", "constraint", "state"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_target_angular_velocity), names("world", "constraint", "velocity"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_target_angle), names("world", "constraint", "angle"));
    bind::fn(b, BIND(adapters::hinge_constraint::get_current_angle), names("world", "constraint"));

    VALUE_AS(SliderConstraintSettings, "SliderConstraintSettings")
        .field(FIELD(SliderConstraintSettings, point_a))
        .field(FIELD(SliderConstraintSettings, point_b))
        .field(FIELD(SliderConstraintSettings, slider_axis_a))
        .field(FIELD(SliderConstraintSettings, slider_axis_b))
        .field(FIELD(SliderConstraintSettings, normal_axis_a))
        .field(FIELD(SliderConstraintSettings, normal_axis_b))
        .field(FIELD(SliderConstraintSettings, space))
        .field(FIELD(SliderConstraintSettings, auto_detect_point))
        .field(FIELD(SliderConstraintSettings, limits_min))
        .field(FIELD(SliderConstraintSettings, limits_max))
        .field(FIELD(SliderConstraintSettings, limits_spring_settings))
        .field(FIELD(SliderConstraintSettings, max_friction_force))
        .field(FIELD(SliderConstraintSettings, motor_settings))
        .field(FIELD(SliderConstraintSettings, constraint_priority))
        .field(FIELD(SliderConstraintSettings, num_velocity_steps_override))
        .field(FIELD(SliderConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::slider_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::slider_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::slider_constraint::get_current_position), names("world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::set_limits), names("world", "constraint", "min", "max"));
    bind::fn(b, BIND(adapters::slider_constraint::set_limits_spring_settings), names("world", "constraint", "settings"));
    bind::fn(b, BIND(adapters::slider_constraint::set_motor_settings), names("world", "constraint", "settings"));
    bind::fn(b, BIND(adapters::slider_constraint::get_motor_settings), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::set_motor_state), names("world", "constraint", "state"));
    bind::fn(b, BIND(adapters::slider_constraint::set_target_velocity), names("world", "constraint", "velocity"));
    bind::fn(b, BIND(adapters::slider_constraint::set_target_position), names("world", "constraint", "position"));
    bind::fn(b, BIND(adapters::slider_constraint::set_max_friction_force), names("world", "constraint", "force"));
    bind::fn(b, BIND(adapters::slider_constraint::get_max_friction_force), names("world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::get_total_lambda_position), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::get_total_lambda_position_limits), names("world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::get_total_lambda_rotation), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::get_total_lambda_motor), names("world", "constraint"));

    VALUE_AS(FixedConstraintSettings, "FixedConstraintSettings")
        .field(FIELD(FixedConstraintSettings, point_a))
        .field(FIELD(FixedConstraintSettings, axis_x_a))
        .field(FIELD(FixedConstraintSettings, axis_y_a))
        .field(FIELD(FixedConstraintSettings, point_b))
        .field(FIELD(FixedConstraintSettings, axis_x_b))
        .field(FIELD(FixedConstraintSettings, axis_y_b))
        .field(FIELD(FixedConstraintSettings, space))
        .field(FIELD(FixedConstraintSettings, auto_detect_point))
        .field(FIELD(FixedConstraintSettings, constraint_priority))
        .field(FIELD(FixedConstraintSettings, num_velocity_steps_override))
        .field(FIELD(FixedConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::fixed_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::fixed_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::fixed_constraint::get_total_lambda_position), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::fixed_constraint::get_total_lambda_rotation), names("out", "world", "constraint"));

    VALUE_AS(ConeConstraintSettings, "ConeConstraintSettings")
        .field(FIELD(ConeConstraintSettings, point_a))
        .field(FIELD(ConeConstraintSettings, twist_axis_a))
        .field(FIELD(ConeConstraintSettings, point_b))
        .field(FIELD(ConeConstraintSettings, twist_axis_b))
        .field(FIELD(ConeConstraintSettings, half_cone_angle))
        .field(FIELD(ConeConstraintSettings, space))
        .field(FIELD(ConeConstraintSettings, constraint_priority))
        .field(FIELD(ConeConstraintSettings, num_velocity_steps_override))
        .field(FIELD(ConeConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::cone_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::cone_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::cone_constraint::set_half_cone_angle), names("world", "constraint", "halfConeAngle"));
    bind::fn(b, BIND(adapters::cone_constraint::get_half_cone_angle), names("world", "constraint"));
    bind::fn(b, BIND(adapters::cone_constraint::get_cos_half_cone_angle), names("world", "constraint"));
    bind::fn(b, BIND(adapters::cone_constraint::get_total_lambda_position), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::cone_constraint::get_total_lambda_rotation), names("world", "constraint"));

    VALUE_AS(SwingTwistConstraintSettings, "SwingTwistConstraintSettings")
        .field(FIELD(SwingTwistConstraintSettings, point_a))
        .field(FIELD(SwingTwistConstraintSettings, point_b))
        .field(FIELD(SwingTwistConstraintSettings, twist_axis_a))
        .field(FIELD(SwingTwistConstraintSettings, plane_axis_a))
        .field(FIELD(SwingTwistConstraintSettings, twist_axis_b))
        .field(FIELD(SwingTwistConstraintSettings, plane_axis_b))
        .field(FIELD(SwingTwistConstraintSettings, space))
        .field(FIELD(SwingTwistConstraintSettings, swing_type))
        .field(FIELD(SwingTwistConstraintSettings, normal_half_cone_angle))
        .field(FIELD(SwingTwistConstraintSettings, plane_half_cone_angle))
        .field(FIELD(SwingTwistConstraintSettings, twist_min_angle))
        .field(FIELD(SwingTwistConstraintSettings, twist_max_angle))
        .field(FIELD(SwingTwistConstraintSettings, max_friction_torque))
        .field(FIELD(SwingTwistConstraintSettings, swing_motor_settings))
        .field(FIELD(SwingTwistConstraintSettings, twist_motor_settings))
        .field(FIELD(SwingTwistConstraintSettings, constraint_priority))
        .field(FIELD(SwingTwistConstraintSettings, num_velocity_steps_override))
        .field(FIELD(SwingTwistConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::swing_twist_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_normal_half_cone_angle), names("world", "constraint", "angle"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_plane_half_cone_angle), names("world", "constraint", "angle"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_twist_min_angle), names("world", "constraint", "angle"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_twist_max_angle), names("world", "constraint", "angle"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_max_friction_torque), names("world", "constraint", "torque"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_swing_motor_settings), names("world", "constraint", "settings"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_twist_motor_settings), names("world", "constraint", "settings"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::get_swing_motor_settings), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::get_twist_motor_settings), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_swing_motor_state), names("world", "constraint", "state"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_twist_motor_state), names("world", "constraint", "state"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_target_angular_velocity_cs), names("world", "constraint", "velocity"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_target_orientation_cs), names("world", "constraint", "orientation"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_target_orientation_bs), names("world", "constraint", "orientation"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::get_rotation_in_constraint_space), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::get_total_lambda_position), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::get_total_lambda_twist), names("world", "constraint"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::get_total_lambda_swing_y), names("world", "constraint"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::get_total_lambda_swing_z), names("world", "constraint"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::get_total_lambda_motor), names("out", "world", "constraint"));

    VALUE_AS(SixDOFConstraintSettings, "SixDOFConstraintSettings")
        .field(FIELD(SixDOFConstraintSettings, point_a))
        .field(FIELD(SixDOFConstraintSettings, point_b))
        .field(FIELD(SixDOFConstraintSettings, axis_x_a))
        .field(FIELD(SixDOFConstraintSettings, axis_y_a))
        .field(FIELD(SixDOFConstraintSettings, axis_x_b))
        .field(FIELD(SixDOFConstraintSettings, axis_y_b))
        .field(FIELD(SixDOFConstraintSettings, space))
        .field(FIELD(SixDOFConstraintSettings, swing_type))
        .field(FIELD(SixDOFConstraintSettings, limit_min))
        .field(FIELD(SixDOFConstraintSettings, limit_max))
        .field(FIELD(SixDOFConstraintSettings, max_friction))
        .field(FIELD(SixDOFConstraintSettings, limits_spring_settings))
        .field(FIELD(SixDOFConstraintSettings, motor_settings))
        .field(FIELD(SixDOFConstraintSettings, constraint_priority))
        .field(FIELD(SixDOFConstraintSettings, num_velocity_steps_override))
        .field(FIELD(SixDOFConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::six_dof_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::six_dof_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::six_dof_constraint::set_translation_limits), names("world", "constraint", "limitMin", "limitMax"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_rotation_limits), names("world", "constraint", "limitMin", "limitMax"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_limits_spring_settings), names("world", "constraint", "axis", "settings"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_motor_settings), names("world", "constraint", "axis", "settings"));
    bind::fn(b, BIND(adapters::six_dof_constraint::get_motor_settings), names("out", "world", "constraint", "axis"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_motor_state), names("world", "constraint", "axis", "state"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_target_velocity_cs), names("world", "constraint", "velocity"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_target_angular_velocity_cs), names("world", "constraint", "velocity"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_target_position_cs), names("world", "constraint", "position"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_target_orientation_cs), names("world", "constraint", "orientation"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_max_friction), names("world", "constraint", "axis", "friction"));
    bind::fn(b, BIND(adapters::six_dof_constraint::make_free_axis), names("world", "constraint", "axis"));
    bind::fn(b, BIND(adapters::six_dof_constraint::make_fixed_axis), names("world", "constraint", "axis"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_limited_axis), names("world", "constraint", "axis", "min", "max"));

    RECORD(Filter);
    bind::fn(b, BIND(adapters::filter::create), names("world"), GROWS);
    bind::fn(b, BIND(adapters::filter::destroy), names("filter"));
    bind::fn(b, BIND(adapters::filter::enable_all_layers), names("filter", "world"));
    bind::fn(b, BIND(adapters::filter::disable_all_layers), names("filter", "world"));
    bind::fn(b, BIND(adapters::filter::enable_object_layer), names("filter", "world", "objectLayer"));
    bind::fn(b, BIND(adapters::filter::disable_object_layer), names("filter", "world", "objectLayer"));
    bind::fn(b, BIND(adapters::filter::enable_broadphase_layer), names("filter", "world", "broadphaseLayer"));
    bind::fn(b, BIND(adapters::filter::disable_broadphase_layer), names("filter", "world", "broadphaseLayer"));
    bind::fn(b, BIND(adapters::filter::is_object_layer_enabled), names("filter", "objectLayer"));
    bind::fn(b, BIND(adapters::filter::is_broadphase_layer_enabled), names("filter", "broadphaseLayer"));
    bind::fn(b, BIND(adapters::filter::set_from_body), names("filter", "world", "body"));
    bind::fn(b, BIND(adapters::filter::get_collision_groups), names("filter"));
    bind::fn(b, BIND(adapters::filter::get_collision_mask), names("filter"));
    bind::fn(b, BIND(adapters::filter::copy), names("out", "source"), GROWS);

    bind::enumeration<uint32_t>(b, "CollectorKind")
        .value("ALL", CollectorKind::ALL)
        .value("ANY", CollectorKind::ANY)
        .value("CLOSEST", CollectorKind::CLOSEST);
    bind::enumeration<uint32_t>(b, "CastRayStatus")
        .value("NOT_COLLIDING", CastRayStatus::NOT_COLLIDING)
        .value("COLLIDING", CastRayStatus::COLLIDING);
    bind::enumeration<uint32_t>(b, "CastShapeStatus")
        .value("NOT_COLLIDING", CastShapeStatus::NOT_COLLIDING)
        .value("COLLIDING", CastShapeStatus::COLLIDING);

    VALUE_AS(CastRaySettings, "CastRaySettings")
        .field(FIELD(CastRaySettings, collide_with_backfaces_triangles))
        .field(FIELD(CastRaySettings, collide_with_backfaces_convex))
        .field(FIELD(CastRaySettings, treat_convex_as_solid));
    VALUE_AS(CastRayHit, "CastRayHit")
        .field(FIELD(CastRayHit, status))
        .field(FIELD(CastRayHit, fraction))
        .field(FIELD(CastRayHit, sub_shape_id))
        .field(FIELD(CastRayHit, body_id_b));
    bind::float_constant(b, "INITIAL_EARLY_OUT_FRACTION", collector::INITIAL_EARLY_OUT_FRACTION,
                   "a cast collector's early out fraction before any hit");
    bind::float_constant(b, "UNBOUNDED_EARLY_OUT", collector::UNBOUNDED, "a collide collector's before any hit: no bound on the penetration");
    bind::float_constant(b, "SHOULD_EARLY_OUT_FRACTION", collector::SHOULD_EARLY_OUT_FRACTION,
                         "the bound at or below which a ray stops; a shape cast, a collide and a point stop at -UNBOUNDED_EARLY_OUT");
    bind::fn(b, BIND(adapters::query::default_cast_ray_settings), names("out"));
    bind::fn(b, BIND(adapters::query::cast_ray),
             names("world", "kind", "earlyOutFraction", "settings", "origin", "direction", "length", "filter", "collisionGroups", "collisionMask"),
             GROWS);
    bind::fn(b, BIND(adapters::query::read_cast_ray_hit), names("out", "index"));

    VALUE_AS(adapters::CastShapeSettings, "CastShapeSettings")
        .field(FIELD(adapters::CastShapeSettings, collide_with_backfaces_triangles))
        .field(FIELD(adapters::CastShapeSettings, collide_with_backfaces_convex))
        .field(FIELD(adapters::CastShapeSettings, collision_tolerance))
        .field(FIELD(adapters::CastShapeSettings, penetration_tolerance))
        .field(FIELD(adapters::CastShapeSettings, return_deepest_point))
        .field(FIELD(adapters::CastShapeSettings, use_shrunken_shape_and_convex_radius))
        .field(FIELD(adapters::CastShapeSettings, collide_only_with_active_edges))
        .field(FIELD(adapters::CastShapeSettings, active_edge_movement_direction))
        .field(FIELD(adapters::CastShapeSettings, extra_convex_radius));
    VALUE_AS(adapters::CastShapeHit, "CastShapeHit")
        .field(FIELD(adapters::CastShapeHit, status))
        .field(FIELD(adapters::CastShapeHit, fraction))
        .field(FIELD(adapters::CastShapeHit, point_a))
        .field(FIELD(adapters::CastShapeHit, point_b))
        .field(FIELD(adapters::CastShapeHit, penetration_depth))
        .field(FIELD(adapters::CastShapeHit, penetration_axis))
        .field(FIELD(adapters::CastShapeHit, normal))
        .field(FIELD(adapters::CastShapeHit, sub_shape_id_a))
        .field(FIELD(adapters::CastShapeHit, sub_shape_id_b))
        .field(FIELD(adapters::CastShapeHit, body_id_b));
    bind::fn(b, BIND(adapters::query::default_cast_shape_settings), names("out"));
    bind::fn(b, BIND(adapters::query::cast_shape),
             names("world", "kind", "earlyOutFraction", "settings", "shape", "position", "quaternion", "scale", "displacement", "filter",
                   "collisionGroups", "collisionMask"),
             GROWS);
    bind::fn(b, BIND(adapters::query::read_cast_shape_hit), names("out", "index"));

    VALUE_AS(adapters::CollideShapeSettings, "CollideShapeSettings")
        .field(FIELD(adapters::CollideShapeSettings, max_separation_distance))
        .field(FIELD(adapters::CollideShapeSettings, collision_tolerance))
        .field(FIELD(adapters::CollideShapeSettings, penetration_tolerance))
        .field(FIELD(adapters::CollideShapeSettings, collide_with_backfaces))
        .field(FIELD(adapters::CollideShapeSettings, collide_only_with_active_edges))
        .field(FIELD(adapters::CollideShapeSettings, active_edge_movement_direction))
        .field(FIELD(adapters::CollideShapeSettings, internal_edge_removal_vertex_tolerance_sq));
    VALUE_AS(adapters::CollideShapeHit, "CollideShapeHit")
        .field(FIELD(adapters::CollideShapeHit, point_a))
        .field(FIELD(adapters::CollideShapeHit, point_b))
        .field(FIELD(adapters::CollideShapeHit, penetration_axis))
        .field(FIELD(adapters::CollideShapeHit, penetration))
        .field(FIELD(adapters::CollideShapeHit, sub_shape_id_a))
        .field(FIELD(adapters::CollideShapeHit, sub_shape_id_b))
        .field(FIELD(adapters::CollideShapeHit, body_id_b));
    bind::fn(b, BIND(adapters::query::default_collide_shape_settings), names("out"));
    bind::fn(b, BIND(adapters::query::collide_shape),
             names("world", "kind", "earlyOutFraction", "settings", "shape", "position", "quaternion", "scale", "internalEdgeRemoval", "filter",
                   "collisionGroups", "collisionMask"),
             GROWS);
    bind::fn(b, BIND(adapters::query::read_collide_shape_hit), names("out", "index"));

    VALUE_AS(CollidePointSettings, "CollidePointSettings").field(FIELD(CollidePointSettings, collision_tolerance));
    VALUE_AS(CollidePointHit, "CollidePointHit")
        .field(FIELD(CollidePointHit, sub_shape_id_b))
        .field(FIELD(CollidePointHit, body_id_b));
    bind::fn(b, BIND(adapters::query::default_collide_point_settings), names("out"));
    bind::fn(b, BIND(adapters::query::collide_point), names("world", "kind", "settings", "point", "filter", "collisionGroups", "collisionMask"),
             GROWS);
    bind::fn(b, BIND(adapters::query::read_collide_point_hit), names("out", "index"));

    RECORD(KCC::Policy);
    VALUE_AS(KCC::BodyPolicy, "KCCBodyPolicy")
        .field(FIELD(KCC::BodyPolicy, ignore))
        .field(FIELD(KCC::BodyPolicy, can_push_character))
        .field(FIELD(KCC::BodyPolicy, can_receive_impulses))
        .field(FIELD(KCC::BodyPolicy, slide_on_walkable))
        .field(FIELD(KCC::BodyPolicy, surface_linear_velocity))
        .field(FIELD(KCC::BodyPolicy, surface_angular_velocity));
    bind::fn(b, BIND(adapters::kcc_policy::default_body_policy), names("out"));
    bind::fn(b, BIND(adapters::kcc_policy::create), names(), GROWS);
    bind::fn(b, BIND(adapters::kcc_policy::destroy), names("policy"));
    bind::fn(b, BIND(adapters::kcc_policy::get_defaults), names("out", "policy"));
    bind::fn(b, BIND(adapters::kcc_policy::set_defaults), names("policy", "bodyPolicy"));
    bind::fn(b, BIND(adapters::kcc_policy::set), names("policy", "body", "bodyPolicy"), GROWS);
    bind::fn(b, BIND(adapters::kcc_policy::remove), names("policy", "body"));
    bind::fn(b, BIND(adapters::kcc_policy::get), names("out", "policy", "body"));

    RECORD(KCC);
    bind::enumeration<uint32_t>(b, "GroundState")
        .value("ON_GROUND", KCC::GroundState::ON_GROUND, "walkable.")
        .value("ON_STEEP_GROUND", KCC::GroundState::ON_STEEP_GROUND, "too steep to climb.")
        .value("NOT_SUPPORTED", KCC::GroundState::NOT_SUPPORTED, "touching something, but not supported by it.")
        .value("IN_AIR", KCC::GroundState::IN_AIR);
    bind::enumeration<uint32_t>(b, "BackFaceMode")
        .doc("triangle mesh back faces.")
        .value("IGNORE", KCC::BackFaceMode::IGNORE)
        .value("COLLIDE", KCC::BackFaceMode::COLLIDE);
    bind::enumeration<uint32_t>(b, "KCCContactEventKind")
        .value("ADDED", KCC::ContactEventKind::ADDED, "first met this frame, not touching last frame.")
        .value("PERSISTED", KCC::ContactEventKind::PERSISTED, "first met this frame, touching last frame too.")
        .value("REMOVED", KCC::ContactEventKind::REMOVED, "touching last frame, not at the end of this one; the body may have been removed.");
    VALUE_AS(adapters::KCCSettings, "KCCSettings")
        .field(FIELD(adapters::KCCSettings, shape))
        .field(FIELD(adapters::KCCSettings, shape_offset))
        .field(FIELD(adapters::KCCSettings, mass))
        .field(FIELD(adapters::KCCSettings, max_strength))
        .field(FIELD(adapters::KCCSettings, up))
        .field(FIELD(adapters::KCCSettings, max_slope_angle))
        .field(FIELD(adapters::KCCSettings, predictive_contact_distance))
        .field(FIELD(adapters::KCCSettings, character_padding))
        .field(FIELD(adapters::KCCSettings, max_collision_iterations))
        .field(FIELD(adapters::KCCSettings, max_constraint_iterations))
        .field(FIELD(adapters::KCCSettings, collision_tolerance))
        .field(FIELD(adapters::KCCSettings, penetration_recovery_speed))
        .field(FIELD(adapters::KCCSettings, min_time_remaining))
        .field(FIELD(adapters::KCCSettings, max_num_hits))
        .field(FIELD(adapters::KCCSettings, hit_reduction_cos_max_angle))
        .field(FIELD(adapters::KCCSettings, back_face_mode))
        .field(FIELD(adapters::KCCSettings, enhanced_internal_edge_removal))
        .field(FIELD(adapters::KCCSettings, has_supporting_volume_plane))
        .field(FIELD(adapters::KCCSettings, supporting_volume_plane))
        .field(FIELD(adapters::KCCSettings, user_data_low))
        .field(FIELD(adapters::KCCSettings, user_data_high))
        .field(FIELD(adapters::KCCSettings, observes_contacts))
        .field(FIELD(adapters::KCCSettings, inner_rigid_body_shape))
        .field(FIELD(adapters::KCCSettings, inner_rigid_body_object_layer));
    VALUE_AS(KCCUpdateSettings, "KCCUpdateSettings")
        .field(FIELD(KCCUpdateSettings, stick_to_floor_step_down))
        .field(FIELD(KCCUpdateSettings, walk_stairs_step_up))
        .field(FIELD(KCCUpdateSettings, walk_stairs_min_step_forward))
        .field(FIELD(KCCUpdateSettings, walk_stairs_step_forward_test))
        .field(FIELD(KCCUpdateSettings, walk_stairs_cos_angle_forward_contact))
        .field(FIELD(KCCUpdateSettings, walk_stairs_step_down_extra));
    VALUE_AS(adapters::KCCTuning, "KCCTuning")
        .field(FIELD(adapters::KCCTuning, shape_offset))
        .field(FIELD(adapters::KCCTuning, mass))
        .field(FIELD(adapters::KCCTuning, max_strength))
        .field(FIELD(adapters::KCCTuning, up))
        .field(FIELD(adapters::KCCTuning, max_slope_angle))
        .field(FIELD(adapters::KCCTuning, predictive_contact_distance))
        .field(FIELD(adapters::KCCTuning, character_padding))
        .field(FIELD(adapters::KCCTuning, max_collision_iterations))
        .field(FIELD(adapters::KCCTuning, max_constraint_iterations))
        .field(FIELD(adapters::KCCTuning, collision_tolerance))
        .field(FIELD(adapters::KCCTuning, penetration_recovery_speed))
        .field(FIELD(adapters::KCCTuning, min_time_remaining))
        .field(FIELD(adapters::KCCTuning, max_num_hits))
        .field(FIELD(adapters::KCCTuning, hit_reduction_cos_max_angle))
        .field(FIELD(adapters::KCCTuning, back_face_mode))
        .field(FIELD(adapters::KCCTuning, enhanced_internal_edge_removal))
        .field(FIELD(adapters::KCCTuning, supporting_volume_plane));
    VALUE_AS(adapters::KCCState, "KCCState")
        .field(FIELD(adapters::KCCState, position))
        .field(FIELD(adapters::KCCState, quaternion))
        .field(FIELD(adapters::KCCState, linear_velocity))
        .field(FIELD(adapters::KCCState, up))
        .field(FIELD(adapters::KCCState, ground_state))
        .field(FIELD(adapters::KCCState, ground_normal))
        .field(FIELD(adapters::KCCState, ground_velocity))
        .field(FIELD(adapters::KCCState, ground_body_id))
        .field(FIELD(adapters::KCCState, ground_sub_shape_id))
        .field(FIELD(adapters::KCCState, ground_position))
        .field(FIELD(adapters::KCCState, shape))
        .field(FIELD(adapters::KCCState, inner_rigid_body_id))
        .field(FIELD(adapters::KCCState, contact_count))
        .field(FIELD(adapters::KCCState, contact_event_count));
    VALUE_AS(adapters::KCCContactEvent, "KCCContactEvent")
        .field(FIELD(adapters::KCCContactEvent, kind))
        .field(FIELD(adapters::KCCContactEvent, body_id))
        .field(FIELD(adapters::KCCContactEvent, sub_shape_id))
        .field(FIELD(adapters::KCCContactEvent, position))
        .field(FIELD(adapters::KCCContactEvent, normal));
    VALUE_AS(adapters::KCCContact, "KCCContact")
        .field(FIELD(adapters::KCCContact, position))
        .field(FIELD(adapters::KCCContact, linear_velocity))
        .field(FIELD(adapters::KCCContact, contact_normal))
        .field(FIELD(adapters::KCCContact, surface_normal))
        .field(FIELD(adapters::KCCContact, distance))
        .field(FIELD(adapters::KCCContact, fraction))
        .field(FIELD(adapters::KCCContact, body_id))
        .field(FIELD(adapters::KCCContact, sub_shape_id))
        .field(FIELD(adapters::KCCContact, material_id))
        .field(FIELD(adapters::KCCContact, motion_type))
        .field(FIELD(adapters::KCCContact, is_sensor));
    VALUE_AS(adapters::KCCSolve, "KCCSolve")
        .field(FIELD(adapters::KCCSolve, velocity))
        .field(FIELD(adapters::KCCSolve, displacement))
        .field(FIELD(adapters::KCCSolve, time_simulated));

    // a character's contact rows, for js to read and edit in place
    bind::constant(b, "KCC_CONTACT_STRIDE", sizeof(KCC::Contact));
    bind::constant(b, "KCC_CONTACT_POSITION", offsetof(KCC::Contact, position));
    bind::constant(b, "KCC_CONTACT_LINEAR_VELOCITY", offsetof(KCC::Contact, linear_velocity));
    bind::constant(b, "KCC_CONTACT_CONTACT_NORMAL", offsetof(KCC::Contact, contact_normal));
    bind::constant(b, "KCC_CONTACT_SURFACE_NORMAL", offsetof(KCC::Contact, surface_normal));
    bind::constant(b, "KCC_CONTACT_DISTANCE", offsetof(KCC::Contact, distance));
    bind::constant(b, "KCC_CONTACT_FRACTION", offsetof(KCC::Contact, fraction));
    bind::constant(b, "KCC_CONTACT_BODY_ID", offsetof(KCC::Contact, body_id));
    bind::constant(b, "KCC_CONTACT_SUB_SHAPE_ID", offsetof(KCC::Contact, sub_shape_id));
    bind::constant(b, "KCC_CONTACT_MATERIAL_ID", offsetof(KCC::Contact, material_id));
    bind::constant(b, "KCC_CONTACT_MOTION_TYPE", offsetof(KCC::Contact, motion_type));
    bind::constant(b, "KCC_CONTACT_IS_SENSOR", offsetof(KCC::Contact, is_sensor));
    bind::constant(b, "KCC_CONTACT_HAD_COLLISION", offsetof(KCC::Contact, had_collision));
    bind::constant(b, "KCC_CONTACT_WAS_DISCARDED", offsetof(KCC::Contact, was_discarded));
    bind::constant(b, "KCC_CONTACT_CAN_PUSH_CHARACTER", offsetof(KCC::Contact, can_push_character));
    bind::constant(b, "KCC_CONTACT_CAN_RECEIVE_IMPULSES", offsetof(KCC::Contact, can_receive_impulses));

    bind::fn(b, BIND(adapters::kcc::default_settings), names("out"));
    bind::fn(b, BIND(adapters::kcc::default_update_settings), names("out"));
    bind::fn(b, BIND(adapters::kcc::create), names("settings", "position", "quaternion"), GROWS);
    bind::fn(b, BIND(adapters::kcc::release), names("character"));
    bind::fn(b, BIND(adapters::kcc::add), names("world", "character"), GROWS);
    bind::fn(b, BIND(adapters::kcc::remove), names("world", "character"));
    bind::fn(b, BIND(adapters::kcc::read_state), names("out", "character"));
    bind::fn(b, BIND(adapters::kcc::set_linear_velocity), names("character", "velocity"));
    bind::fn(b, BIND(adapters::kcc::get_tuning), names("out", "character"));
    bind::fn(b, BIND(adapters::kcc::set_tuning), names("character", "tuning"));
    bind::fn(b, BIND(adapters::kcc::contacts), names("character"));
    bind::fn(b, BIND(adapters::kcc::read_contact_event), names("out", "character", "index"));
    bind::fn(b, BIND(adapters::kcc::set_position), names("world", "character", "position"), GROWS);
    bind::fn(b, BIND(adapters::kcc::set_quaternion), names("world", "character", "quaternion"), GROWS);
    bind::fn(b, BIND(adapters::kcc::is_slope_too_steep), names("character", "normal"));
    bind::fn(b, BIND(adapters::kcc::get_center_of_mass_transform), names("outPosition", "outQuaternion", "character"));
    bind::fn(b, BIND(adapters::kcc::has_collided_with), names("character", "body"));
    bind::fn(b, BIND(adapters::kcc::can_walk_stairs), names("character", "velocity"));
    bind::fn(b, BIND(adapters::kcc::cancel_velocity_towards_steep_slopes), names("out", "character", "velocity"));
    bind::fn(b, BIND(adapters::kcc::update_ground_velocity), names("world", "character", "policy"));
    bind::fn(b, BIND(adapters::kcc::refresh_contacts), names("world", "character", "filter", "collisionGroups", "collisionMask", "policy"), GROWS);
    bind::fn(b, BIND(adapters::kcc::set_shape),
             names("world", "character", "shape", "filter", "collisionGroups", "collisionMask", "policy", "maxPenetrationDepth"), GROWS);
    bind::fn(b, BIND(adapters::kcc::move),
             names("world", "character", "deltaTime", "gravity", "policy", "filter", "collisionGroups", "collisionMask"), GROWS);
    bind::fn(b, BIND(adapters::kcc::stick_to_floor),
             names("world", "character", "stepDown", "filter", "collisionGroups", "collisionMask", "policy"), GROWS);
    bind::fn(b, BIND(adapters::kcc::walk_stairs),
             names("world", "character", "deltaTime", "stepUp", "stepForward", "stepForwardTest", "stepDownExtra", "filter", "collisionGroups",
                   "collisionMask", "policy"),
             GROWS);
    bind::fn(b, BIND(adapters::kcc::update),
             names("world", "character", "deltaTime", "gravity", "settings", "policy", "filter", "collisionGroups", "collisionMask"), GROWS);
    bind::fn(b, BIND(adapters::kcc::collide),
             names("world", "character", "position", "movementDirection", "filter", "collisionGroups", "collisionMask", "policy"), GROWS);
    bind::fn(b, BIND(adapters::kcc::solve), names("out", "world", "character", "velocity", "deltaTime", "timeRemaining", "policy"), GROWS);
    bind::fn(b, BIND(adapters::kcc::cast),
             names("out", "world", "character", "position", "displacement", "filter", "collisionGroups", "collisionMask", "policy"), GROWS);
    bind::fn(b, BIND(adapters::kcc::update_ground), names("world", "character", "deltaTime", "skipContactVelocityCheck", "policy"), GROWS);
    bind::fn(b, BIND(adapters::kcc::apply_weight), names("world", "character", "gravity", "deltaTime"), GROWS);
    bind::fn(b, BIND(adapters::kcc::begin_contact_tracking), names("character"), GROWS);
    bind::fn(b, BIND(adapters::kcc::end_contact_tracking), names("character"), GROWS);

    RECORD(DebugDraw);
    bind::enumeration<uint32_t>(b, "BodyColorMode")
        .value("MOTION_TYPE", DebugDraw::BodyColorMode::MOTION_TYPE)
        .value("INSTANCE", DebugDraw::BodyColorMode::INSTANCE, "a colour of its own per body.")
        .value("SLEEPING", DebugDraw::BodyColorMode::SLEEPING)
        .value("ISLAND", DebugDraw::BodyColorMode::ISLAND);
    VALUE_AS(DebugDraw::BodiesOptions, "DebugBodiesOptions")
        .field(FIELD(DebugDraw::BodiesOptions, color_mode))
        .field(FIELD(DebugDraw::BodiesOptions, show_linear_velocity))
        .field(FIELD(DebugDraw::BodiesOptions, show_angular_velocity))
        .field(FIELD(DebugDraw::BodiesOptions, excluded_object_layers));
    VALUE_AS(DebugDraw::BodyOptions, "DebugBodyOptions")
        .field(FIELD(DebugDraw::BodyOptions, has_color))
        .field(FIELD(DebugDraw::BodyOptions, color))
        .field(FIELD(DebugDraw::BodyOptions, color_mode))
        .field(FIELD(DebugDraw::BodyOptions, show_linear_velocity))
        .field(FIELD(DebugDraw::BodyOptions, show_angular_velocity));
    VALUE_AS(DebugDraw::ShapeOptions, "DebugShapeOptions").field(FIELD(DebugDraw::ShapeOptions, color));
    VALUE_AS(DebugDraw::JointsOptions, "DebugJointsOptions")
        .field(FIELD(DebugDraw::JointsOptions, size))
        .field(FIELD(DebugDraw::JointsOptions, draw_limits));
    bind::constant(b, "DEBUG_VERTEX_STRIDE", debug::VERTEX_STRIDE, "floats per vertex: x, y, z, r, g, b.");
    bind::fn(b, BIND(adapters::debug::create), names(), GROWS);
    bind::fn(b, BIND(adapters::debug::destroy), names("buffers"));
    bind::fn(b, BIND(adapters::debug::clear), names("buffers"));
    bind::fn(b, BIND(adapters::debug::lines), names("buffers"));
    bind::fn(b, BIND(adapters::debug::line_count), names("buffers"));
    bind::fn(b, BIND(adapters::debug::triangles), names("buffers"));
    bind::fn(b, BIND(adapters::debug::triangle_count), names("buffers"));
    bind::fn(b, BIND(adapters::debug::default_bodies_options), names("out"));
    bind::fn(b, BIND(adapters::debug::default_body_options), names("out"));
    bind::fn(b, BIND(adapters::debug::default_shape_options), names("out"));
    bind::fn(b, BIND(adapters::debug::default_joints_options), names("out"));
    bind::fn(b, BIND(adapters::debug::bodies), names("out", "world", "options"), GROWS);
    bind::fn(b, BIND(adapters::debug::body), names("out", "world", "body", "options"), GROWS);
    bind::fn(b, BIND(adapters::debug::line), names("out", "a", "b", "color"), GROWS);
    bind::fn(b, BIND(adapters::debug::shape), names("out", "shape", "position", "quaternion", "options"), GROWS);
    bind::fn(b, BIND(adapters::debug::contacts), names("out", "world"), GROWS);
    bind::fn(b, BIND(adapters::debug::character_contacts), names("out", "character"), GROWS);
    bind::fn(b, BIND(adapters::debug::contact_constraints), names("out", "world"), GROWS);
    bind::fn(b, BIND(adapters::debug::joints), names("out", "world", "options"), GROWS);

    VALUE_AS(adapters::ContactPair, "ContactPair")
        .field(FIELD(adapters::ContactPair, handle))
        .field(FIELD(adapters::ContactPair, body_a))
        .field(FIELD(adapters::ContactPair, sub_shape_a))
        .field(FIELD(adapters::ContactPair, user_data_a_low))
        .field(FIELD(adapters::ContactPair, user_data_a_high))
        .field(FIELD(adapters::ContactPair, sensor_a))
        .field(FIELD(adapters::ContactPair, body_b))
        .field(FIELD(adapters::ContactPair, sub_shape_b))
        .field(FIELD(adapters::ContactPair, user_data_b_low))
        .field(FIELD(adapters::ContactPair, user_data_b_high))
        .field(FIELD(adapters::ContactPair, sensor_b))
        .field(FIELD(adapters::ContactPair, point))
        .field(FIELD(adapters::ContactPair, normal))
        .field(FIELD(adapters::ContactPair, penetration_depth))
        .field(FIELD(adapters::ContactPair, relative_velocity))
        .field(FIELD(adapters::ContactPair, added));
    bind::fn(b, BIND(adapters::contact_pairs::begin), names("world"), GROWS);
    bind::fn(b, BIND(adapters::contact_pairs::record_world_contacts), names("world", "firstAdded", "firstPersisted"), GROWS);
    bind::fn(b, BIND(adapters::contact_pairs::record_character_contacts), names("world", "character"), GROWS);
    bind::fn(b, BIND(adapters::contact_pairs::end), names("world"), GROWS);
    bind::fn(b, BIND(adapters::contact_pairs::active_count), names("world"));
    bind::fn(b, BIND(adapters::contact_pairs::read_active), names("out", "world", "index"));
    bind::fn(b, BIND(adapters::contact_pairs::removed_count), names("world"));
    bind::fn(b, BIND(adapters::contact_pairs::read_removed), names("out", "world", "index"));
}
