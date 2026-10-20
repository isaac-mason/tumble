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
    TUPLE_AS(Float4x4, "Mat4").field(FIELD(Float4x4, m));
    VALUE_AS(Box3f, "Box3").field(FIELD(Box3f, min)).field(FIELD(Box3f, max));
    VALUE_AS(mass_properties::MassProperties, "MassProperties")
        .field(FIELD(mass_properties::MassProperties, mass), "kg.")
        .field(FIELD(mass_properties::MassProperties, inertia), "kg m^2, the upper 3x3 of a column-major matrix.");

    VALUE_AS(world_settings::ContactsSettings, "ContactsSettings")
        .field(FIELD(world_settings::ContactsSettings, contact_point_preserve_lambda_max_dist_sq));
    VALUE_AS(world_settings::NarrowphaseSettings, "NarrowphaseSettings")
        .field(FIELD(world_settings::NarrowphaseSettings, collision_tolerance))
        .field(FIELD(world_settings::NarrowphaseSettings, penetration_tolerance))
        .field(FIELD(world_settings::NarrowphaseSettings, return_deepest_point))
        .field(FIELD(world_settings::NarrowphaseSettings, collide_only_with_active_edges))
        .field(FIELD(world_settings::NarrowphaseSettings, speculative_contact_distance))
        .field(FIELD(world_settings::NarrowphaseSettings, manifold_tolerance))
        .field(FIELD(world_settings::NarrowphaseSettings, collide_with_backfaces))
        .field(FIELD(world_settings::NarrowphaseSettings, use_manifold_reduction))
        .field(FIELD(world_settings::NarrowphaseSettings, normal_cos_max_delta_rotation))
        .field(FIELD(world_settings::NarrowphaseSettings, use_body_pair_contact_cache))
        .field(FIELD(world_settings::NarrowphaseSettings, body_pair_cache_max_delta_position_sq))
        .field(FIELD(world_settings::NarrowphaseSettings, body_pair_cache_cos_max_delta_rotation_div_2))
        .field(FIELD(world_settings::NarrowphaseSettings, internal_edge_removal_vertex_tolerance_sq));
    VALUE_AS(world_settings::SolverSettings, "SolverSettings")
        .field(FIELD(world_settings::SolverSettings, min_velocity_for_restitution))
        .field(FIELD(world_settings::SolverSettings, velocity_iterations))
        .field(FIELD(world_settings::SolverSettings, position_iterations))
        .field(FIELD(world_settings::SolverSettings, warm_starting))
        .field(FIELD(world_settings::SolverSettings, penetration_slop))
        .field(FIELD(world_settings::SolverSettings, baumgarte_factor))
        .field(FIELD(world_settings::SolverSettings, warm_start_impulse_ratio))
        .field(FIELD(world_settings::SolverSettings, max_penetration_distance));
    VALUE_AS(world_settings::SleepingSettings, "SleepingSettings")
        .field(FIELD(world_settings::SleepingSettings, allow_sleeping))
        .field(FIELD(world_settings::SleepingSettings, time_before_sleep))
        .field(FIELD(world_settings::SleepingSettings, point_velocity_sleep_threshold));
    VALUE_AS(world_settings::CcdSettings, "CcdSettings")
        .field(FIELD(world_settings::CcdSettings, linear_cast_threshold))
        .field(FIELD(world_settings::CcdSettings, linear_cast_max_penetration))
        .field(FIELD(world_settings::CcdSettings, min_velocity_for_restitution));
    RECORD(world_settings::WorldSettings)
        .field(FIELD(world_settings::WorldSettings, gravity), bind::WRITABLE)
        .field(FIELD(world_settings::WorldSettings, gravity_enabled), bind::WRITABLE)
        .field(FIELD(world_settings::WorldSettings, contacts), bind::WRITABLE)
        .field(FIELD(world_settings::WorldSettings, narrowphase), bind::WRITABLE)
        .field(FIELD(world_settings::WorldSettings, solver), bind::WRITABLE)
        .field(FIELD(world_settings::WorldSettings, sleeping), bind::WRITABLE)
        .field(FIELD(world_settings::WorldSettings, ccd), bind::WRITABLE);
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
    bind::fn(b, BIND(adapters::world::step_begin), names("world", "deltaTime"), GROWS);
    bind::fn(b, BIND(adapters::world::find_pairs), names("world"), GROWS);
    bind::fn(b, BIND(adapters::world::narrowphase), names("world"), GROWS);
    bind::fn(b, BIND(adapters::world::solve), names("world"), GROWS);
    bind::fn(b, BIND(adapters::world::step_end), names("world"), GROWS);
    bind::fn(b, BIND(adapters::world::woken), names("world"));
    bind::fn(b, BIND(adapters::world::set_materials), names("world", "frictionRestitution", "count"), GROWS);

    VALUE_AS(contact_constraints::ContactSettings, "ContactSettings")
        .field(FIELD(contact_constraints::ContactSettings, combined_friction))
        .field(FIELD(contact_constraints::ContactSettings, combined_restitution))
        .field(FIELD(contact_constraints::ContactSettings, is_sensor))
        .field(FIELD(contact_constraints::ContactSettings, inv_mass_scale_1))
        .field(FIELD(contact_constraints::ContactSettings, inv_mass_scale_2))
        .field(FIELD(contact_constraints::ContactSettings, inv_inertia_scale_1))
        .field(FIELD(contact_constraints::ContactSettings, inv_inertia_scale_2))
        .field(FIELD(contact_constraints::ContactSettings, relative_linear_surface_velocity))
        .field(FIELD(contact_constraints::ContactSettings, relative_angular_surface_velocity));
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
    bind::fn(b, BIND(adapters::events::moved), names("world"));
    bind::fn(b, BIND(adapters::events::moved_body), names("world", "index"));
    RECORD(adapters::Poses);
    bind::fn(b, BIND(adapters::poses::create), names("world"), GROWS);
    bind::fn(b, BIND(adapters::poses::destroy), names("poses"));
    bind::fn(b, BIND(adapters::poses::add), names("poses", "body"), GROWS);
    bind::fn(b, BIND(adapters::poses::remove), names("poses", "slot"), GROWS);
    bind::fn(b, BIND(adapters::poses::read), names("poses"));
    bind::fn(b, BIND(adapters::poses::slots), names("poses"));
    bind::fn(b, BIND(adapters::poses::body), names("poses", "slot"));
    bind::fn(b, BIND(adapters::poses::positions), names("poses"));
    bind::fn(b, BIND(adapters::poses::quaternions), names("poses"));
    bind::fn(b, BIND(adapters::poses::changed), names("poses"));
    bind::fn(b, BIND(adapters::answers::reject_pair), names("world", "index"));
    bind::fn(b, BIND(adapters::answers::override_contact), names("world", "index", "settings"));
    bind::fn(b, BIND(adapters::answers::reject_contact), names("world", "index"));

    bind::enumeration<uint32_t>(b, "MotionType")
        .value("STATIC", motion_type::MOTION_STATIC, "never moves.")
        .value("KINEMATIC", motion_type::MOTION_KINEMATIC, "moves by its velocity, pushing dynamic bodies aside.")
        .value("DYNAMIC", motion_type::MOTION_DYNAMIC, "moved by gravity, forces and contacts.");
    bind::enumeration<uint32_t>(b, "MotionQuality")
        .value("DISCRETE", motion_properties::MOTION_QUALITY_DISCRETE, "fast, but can tunnel through thin geometry at high speed.")
        .value("LINEAR_CAST", motion_properties::MOTION_QUALITY_LINEAR_CAST,
               "cast along the velocity so a fast body cannot tunnel; dearer than discrete.");
    bind::enumeration<uint32_t>(b, "MaterialCombineMode")
        .doc("how two bodies' friction or restitution combine; when the modes differ the more restrictive wins.")
        .value("AVERAGE", combine_material::AVERAGE)
        .value("MULTIPLY", combine_material::MULTIPLY)
        .value("MIN", combine_material::MIN)
        .value("MAX", combine_material::MAX)
        .value("GEOMETRIC_MEAN", combine_material::GEOMETRIC_MEAN);
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
    bind::fn(b, BIND(adapters::rigid_body::set_shape), names("world", "body", "shape"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::move_kinematic), names("world", "body", "position", "quaternion", "deltaTime"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::sleep), names("world", "body"));
    bind::fn(b, BIND(adapters::rigid_body::wake), names("world", "body"), GROWS);
    bind::fn(b, BIND(adapters::rigid_body::set_user_data), names("world", "body", "low", "high"));
    bind::fn(b, BIND(adapters::rigid_body::set_friction), names("world", "body", "friction"));
    bind::fn(b, BIND(adapters::rigid_body::set_restitution), names("world", "body", "restitution"));

    bind::enumeration<uint32_t>(b, "ShapeType")
        .value("SPHERE", shapes::SHAPE_SPHERE)
        .value("BOX", shapes::SHAPE_BOX)
        .value("CAPSULE", shapes::SHAPE_CAPSULE)
        .value("CONVEX_HULL", shapes::SHAPE_CONVEX_HULL)
        .value("TRIANGLE_MESH", shapes::SHAPE_TRIANGLE_MESH)
        .value("COMPOUND", shapes::SHAPE_COMPOUND)
        .value("TRANSFORMED", shapes::SHAPE_TRANSFORMED)
        .value("SCALED", shapes::SHAPE_SCALED)
        .value("EMPTY", shapes::SHAPE_EMPTY)
        .value("CYLINDER", shapes::SHAPE_CYLINDER)
        .value("OFFSET_CENTER_OF_MASS", shapes::SHAPE_OFFSET_CENTER_OF_MASS)
        .value("PLANE", shapes::SHAPE_PLANE)
        .value("STATIC_COMPOUND", shapes::SHAPE_STATIC_COMPOUND);
    bind::fn(b, BIND(adapters::shape::get_type), names("shape"));
    bind::fn(b, BIND(adapters::shape::get_local_bounds), names("out", "shape"));
    bind::fn(b, BIND(adapters::shape::get_center_of_mass), names("out", "shape"));
    bind::fn(b, BIND(adapters::shape::get_volume), names("shape"));
    bind::fn(b, BIND(adapters::shape::get_mass_properties), names("out", "shape"));

    VALUE_AS(box::BoxShapeSettings, "BoxSettings")
        .field(FIELD(box::BoxShapeSettings, half_extents))
        .field(FIELD(box::BoxShapeSettings, convex_radius))
        .field(FIELD(box::BoxShapeSettings, density))
        .field(FIELD(box::BoxShapeSettings, material_id));
    bind::fn(b, BIND(adapters::box::default_settings), names("out"));
    bind::fn(b, BIND(adapters::box::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::box::destroy), names("shape"));
    VALUE_AS(sphere::SphereShapeSettings, "SphereSettings")
        .field(FIELD(sphere::SphereShapeSettings, radius))
        .field(FIELD(sphere::SphereShapeSettings, density))
        .field(FIELD(sphere::SphereShapeSettings, material_id));
    bind::fn(b, BIND(adapters::sphere::default_settings), names("out"));
    bind::fn(b, BIND(adapters::sphere::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::sphere::destroy), names("shape"));
    VALUE_AS(capsule::CapsuleShapeSettings, "CapsuleSettings")
        .field(FIELD(capsule::CapsuleShapeSettings, half_height_of_cylinder))
        .field(FIELD(capsule::CapsuleShapeSettings, radius))
        .field(FIELD(capsule::CapsuleShapeSettings, density))
        .field(FIELD(capsule::CapsuleShapeSettings, material_id));
    bind::fn(b, BIND(adapters::capsule::default_settings), names("out"));
    bind::fn(b, BIND(adapters::capsule::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::capsule::destroy), names("shape"));
    VALUE_AS(cylinder::CylinderShapeSettings, "CylinderSettings")
        .field(FIELD(cylinder::CylinderShapeSettings, half_height))
        .field(FIELD(cylinder::CylinderShapeSettings, radius))
        .field(FIELD(cylinder::CylinderShapeSettings, convex_radius))
        .field(FIELD(cylinder::CylinderShapeSettings, density))
        .field(FIELD(cylinder::CylinderShapeSettings, material_id));
    bind::fn(b, BIND(adapters::cylinder::default_settings), names("out"));
    bind::fn(b, BIND(adapters::cylinder::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::cylinder::destroy), names("shape"));
    VALUE_AS(Plane3f, "Plane3").field(FIELD(Plane3f, normal)).field(FIELD(Plane3f, constant));
    VALUE_AS(plane::PlaneShapeSettings, "PlaneSettings")
        .field(FIELD(plane::PlaneShapeSettings, plane))
        .field(FIELD(plane::PlaneShapeSettings, half_extent))
        .field(FIELD(plane::PlaneShapeSettings, material_id));
    bind::fn(b, BIND(adapters::plane::default_settings), names("out"));
    bind::fn(b, BIND(adapters::plane::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::plane::destroy), names("shape"));
    bind::fn(b, BIND(adapters::empty::create), names(), GROWS);
    bind::fn(b, BIND(adapters::empty::destroy), names("shape"));

    RECORD(convex_hull::ConvexHullData);
    bind::enumeration<uint32_t>(b, "ConvexHullBuildResult")
        .doc("why a hull build failed.")
        .value("SUCCESS", convex_hull::BUILD_SUCCESS)
        .value("TOO_FEW_POINTS", convex_hull::BUILD_TOO_FEW_POINTS, "fewer than 3 points.")
        .value("TOO_FEW_FACES", convex_hull::BUILD_TOO_FEW_FACES, "the points were nearly coplanar and the hull collapsed.")
        .value("DEGENERATE", convex_hull::BUILD_DEGENERATE, "the points coincide or lie on a line.")
        .value("INVALID_CONVEX_RADIUS", convex_hull::BUILD_INVALID_CONVEX_RADIUS, "a negative convex radius.")
        .value("HULL_ERROR_TOO_LARGE", convex_hull::BUILD_HULL_ERROR_TOO_LARGE,
               "a point lies outside the built hull by more than 4 * max(coplanar distance, hull tolerance).")
        .value("TOO_MANY_POINTS", convex_hull::BUILD_TOO_MANY_POINTS,
               "more than 256 vertices survived; a larger hull tolerance gives fewer.")
        .value("POINT_WITH_TOO_FEW_FACES", convex_hull::BUILD_POINT_WITH_TOO_FEW_FACES, "a hull vertex on fewer than 2 faces.")
        .value("OUT_OF_MEMORY", convex_hull::BUILD_OUT_OF_MEMORY);
    bind::enumeration<uint32_t>(b, "BakeSupportAdjacency")
        .doc("whether a hull keeps its vertex neighbours, for faster support queries on big hulls.")
        .value("AUTO", convex_hull::BAKE_SUPPORT_ADJACENCY_AUTO, "when the hull has many vertices.")
        .value("NEVER", convex_hull::BAKE_SUPPORT_ADJACENCY_NEVER)
        .value("ALWAYS", convex_hull::BAKE_SUPPORT_ADJACENCY_ALWAYS);
    VALUE_AS(convex_hull::ConvexHullBuildSettings, "ConvexHullBuildSettings")
        .field(FIELD(convex_hull::ConvexHullBuildSettings, positions))
        .field(FIELD(convex_hull::ConvexHullBuildSettings, num_positions))
        .field(FIELD(convex_hull::ConvexHullBuildSettings, convex_radius))
        .field(FIELD(convex_hull::ConvexHullBuildSettings, hull_tolerance))
        .field(FIELD(convex_hull::ConvexHullBuildSettings, max_error_convex_radius))
        .field(FIELD(convex_hull::ConvexHullBuildSettings, bake_support_adjacency));
    VALUE_AS(adapters::ConvexHullBuild, "ConvexHullBuild")
        .field(FIELD(adapters::ConvexHullBuild, data))
        .field(FIELD(adapters::ConvexHullBuild, result));
    bind::fn(b, BIND(adapters::convex_hull_data::default_settings), names("out"));
    bind::fn(b, BIND(adapters::convex_hull_data::build), names("out", "settings"), GROWS);
    bind::fn(b, BIND(adapters::convex_hull_data::release), names("data"));
    VALUE_AS(convex_hull::ConvexHullShapeSettings, "ConvexHullSettings")
        .field(FIELD(convex_hull::ConvexHullShapeSettings, data))
        .field(FIELD(convex_hull::ConvexHullShapeSettings, density))
        .field(FIELD(convex_hull::ConvexHullShapeSettings, material_id));
    bind::fn(b, BIND(adapters::convex_hull::default_settings), names("out"));
    bind::fn(b, BIND(adapters::convex_hull::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::convex_hull::destroy), names("shape"));

    RECORD(triangle_mesh_data::TriangleMeshData);
    bind::enumeration<uint32_t>(b, "BvhSplitStrategy")
        .doc("how a mesh's bvh is split while it is built.")
        .value("CENTER", triangle_mesh_bvh::BVH_SPLIT_STRATEGY_CENTER, "at the midpoint of the centres' bounds: fast, a good balance.")
        .value("AVERAGE", triangle_mesh_bvh::BVH_SPLIT_STRATEGY_AVERAGE, "at the mean of the centres: fast, can unbalance skewed meshes.")
        .value("SAH", triangle_mesh_bvh::BVH_SPLIT_STRATEGY_SAH, "by surface area: slow to build, fastest to query, for static meshes.");
    VALUE_AS(triangle_mesh_builder::TriangleMeshBuilderSettings, "TriangleMeshBuildSettings")
        .field(FIELD(triangle_mesh_builder::TriangleMeshBuilderSettings, positions))
        .field(FIELD(triangle_mesh_builder::TriangleMeshBuilderSettings, num_positions))
        .field(FIELD(triangle_mesh_builder::TriangleMeshBuilderSettings, indices))
        .field(FIELD(triangle_mesh_builder::TriangleMeshBuilderSettings, num_indices))
        .field(FIELD(triangle_mesh_builder::TriangleMeshBuilderSettings, material_ids))
        .field(FIELD(triangle_mesh_builder::TriangleMeshBuilderSettings, degenerate_tolerance))
        .field(FIELD(triangle_mesh_builder::TriangleMeshBuilderSettings, bvh_split_strategy))
        .field(FIELD(triangle_mesh_builder::TriangleMeshBuilderSettings, bvh_max_leaf_tris))
        .field(FIELD(triangle_mesh_builder::TriangleMeshBuilderSettings, active_edge_cos_threshold_angle));
    bind::fn(b, BIND(adapters::triangle_mesh_data::default_settings), names("out"));
    bind::fn(b, BIND(adapters::triangle_mesh_data::build), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::triangle_mesh_data::release), names("data"));
    VALUE_AS(triangle_mesh::TriangleMeshShapeSettings, "TriangleMeshSettings")
        .field(FIELD(triangle_mesh::TriangleMeshShapeSettings, data))
        .field(FIELD(triangle_mesh::TriangleMeshShapeSettings, material_id));
    bind::fn(b, BIND(adapters::triangle_mesh::default_settings), names("out"));
    bind::fn(b, BIND(adapters::triangle_mesh::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::triangle_mesh::destroy), names("shape"));

    VALUE_AS(compound::CompoundShapeChild, "CompoundChild")
        .field(FIELD(compound::CompoundShapeChild, position))
        .field(FIELD(compound::CompoundShapeChild, quaternion))
        .field(FIELD(compound::CompoundShapeChild, shape));
    bind::fn(b, BIND(adapters::compound::create), names("children", "count"), GROWS);
    bind::fn(b, BIND(adapters::compound::destroy), names("shape"));
    bind::fn(b, BIND(adapters::static_compound::default_bvh_max_leaf_children), names());
    bind::fn(b, BIND(adapters::static_compound::create), names("children", "count", "bvhMaxLeafChildren"), GROWS);
    bind::fn(b, BIND(adapters::static_compound::destroy), names("shape"));

    VALUE_AS(transformed::TransformedShapeSettings, "TransformedSettings")
        .field(FIELD(transformed::TransformedShapeSettings, shape))
        .field(FIELD(transformed::TransformedShapeSettings, position))
        .field(FIELD(transformed::TransformedShapeSettings, quaternion));
    bind::fn(b, BIND(adapters::transformed::default_settings), names("out"));
    bind::fn(b, BIND(adapters::transformed::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::transformed::destroy), names("shape"));
    VALUE_AS(scaled::ScaledShapeSettings, "ScaledSettings")
        .field(FIELD(scaled::ScaledShapeSettings, shape))
        .field(FIELD(scaled::ScaledShapeSettings, scale));
    bind::fn(b, BIND(adapters::scaled::default_settings), names("out"));
    bind::fn(b, BIND(adapters::scaled::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::scaled::destroy), names("shape"));
    VALUE_AS(offset_center_of_mass::OffsetCenterOfMassShapeSettings, "OffsetCenterOfMassSettings")
        .field(FIELD(offset_center_of_mass::OffsetCenterOfMassShapeSettings, shape))
        .field(FIELD(offset_center_of_mass::OffsetCenterOfMassShapeSettings, offset));
    bind::fn(b, BIND(adapters::offset_center_of_mass::default_settings), names("out"));
    bind::fn(b, BIND(adapters::offset_center_of_mass::create), names("settings"), GROWS);
    bind::fn(b, BIND(adapters::offset_center_of_mass::destroy), names("shape"));
}
