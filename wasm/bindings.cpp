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

    TUPLE_AS(Vec2f, "Vec2").field(FIELD(Vec2f, x)).field(FIELD(Vec2f, y));
    bind::enumeration<uint32_t>(b, "ConstraintType")
        .doc("a constraint's type, encoded in its id.")
        .value("POINT", constraint_id::POINT)
        .value("DISTANCE", constraint_id::DISTANCE)
        .value("HINGE", constraint_id::HINGE)
        .value("SLIDER", constraint_id::SLIDER)
        .value("FIXED", constraint_id::FIXED)
        .value("CONE", constraint_id::CONE)
        .value("SWING_TWIST", constraint_id::SWING_TWIST)
        .value("SIX_DOF", constraint_id::SIX_DOF);
    bind::enumeration<uint32_t>(b, "ConstraintSpace")
        .doc("the space a constraint's points and axes are given in.")
        .value("WORLD", constraints::SPACE_WORLD)
        .value("LOCAL", constraints::SPACE_LOCAL, "relative to each body: its centre of mass and orientation.");
    bind::enumeration<uint32_t>(b, "SpringMode")
        .value("FREQUENCY_AND_DAMPING", spring_settings::FREQUENCY_AND_DAMPING,
               "frequencyOrStiffness is the oscillation frequency in hz, damping the damping ratio (0 none, 1 critical).")
        .value("STIFFNESS_AND_DAMPING", spring_settings::STIFFNESS_AND_DAMPING, "frequencyOrStiffness is k and damping is c in F = -k x - c v.");
    bind::enumeration<uint32_t>(b, "MotorState")
        .value("OFF", motor_settings::OFF, "free, apart from any friction.")
        .value("VELOCITY", motor_settings::VELOCITY, "drives to the target velocity.")
        .value("POSITION", motor_settings::POSITION, "drives to the target position through the motor's spring.")
        .value("POSITION_AND_VELOCITY", motor_settings::POSITION_AND_VELOCITY, "drives to both targets.");
    bind::enumeration<uint32_t>(b, "SwingType")
        .doc("the shape that limits a swing.")
        .value("CONE", swing_twist_constraint_part::CONE, "symmetric about 0.")
        .value("PYRAMID", swing_twist_constraint_part::PYRAMID, "asymmetric limits allowed.");
    bind::enumeration<uint32_t>(b, "SixDOFAxis")
        .doc("a six dof constraint's axes, in its constraint space.")
        .value("TRANSLATION_X", six_dof_constraint::TRANSLATION_X)
        .value("TRANSLATION_Y", six_dof_constraint::TRANSLATION_Y)
        .value("TRANSLATION_Z", six_dof_constraint::TRANSLATION_Z)
        .value("ROTATION_X", six_dof_constraint::ROTATION_X)
        .value("ROTATION_Y", six_dof_constraint::ROTATION_Y)
        .value("ROTATION_Z", six_dof_constraint::ROTATION_Z);
    VALUE_AS(spring_settings::SpringSettings, "SpringSettings")
        .field(FIELD(spring_settings::SpringSettings, mode))
        .field(FIELD(spring_settings::SpringSettings, frequency_or_stiffness))
        .field(FIELD(spring_settings::SpringSettings, damping));
    VALUE_AS(motor_settings::MotorSettings, "MotorSettings")
        .field(FIELD(motor_settings::MotorSettings, spring))
        .field(FIELD(motor_settings::MotorSettings, min_force_limit))
        .field(FIELD(motor_settings::MotorSettings, max_force_limit))
        .field(FIELD(motor_settings::MotorSettings, min_torque_limit))
        .field(FIELD(motor_settings::MotorSettings, max_torque_limit));

    bind::fn(b, BIND(adapters::constraint::get_type), names("constraint"));
    bind::fn(b, BIND(adapters::constraint::exists), names("world", "constraint"));
    bind::fn(b, BIND(adapters::constraint::set_enabled), names("world", "constraint", "enabled"));
    bind::fn(b, BIND(adapters::constraint::is_enabled), names("world", "constraint"));

    VALUE_AS(point_constraint::PointConstraintSettings, "PointConstraintSettings")
        .field(FIELD(point_constraint::PointConstraintSettings, point_a))
        .field(FIELD(point_constraint::PointConstraintSettings, point_b))
        .field(FIELD(point_constraint::PointConstraintSettings, space))
        .field(FIELD(point_constraint::PointConstraintSettings, constraint_priority))
        .field(FIELD(point_constraint::PointConstraintSettings, num_velocity_steps_override))
        .field(FIELD(point_constraint::PointConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::point_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::point_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::point_constraint::remove), names("world", "constraint"));
    bind::fn(b, BIND(adapters::point_constraint::get_total_lambda), names("out", "world", "constraint"));

    VALUE_AS(distance_constraint::DistanceConstraintSettings, "DistanceConstraintSettings")
        .field(FIELD(distance_constraint::DistanceConstraintSettings, point_a))
        .field(FIELD(distance_constraint::DistanceConstraintSettings, point_b))
        .field(FIELD(distance_constraint::DistanceConstraintSettings, min_distance))
        .field(FIELD(distance_constraint::DistanceConstraintSettings, max_distance))
        .field(FIELD(distance_constraint::DistanceConstraintSettings, spring_settings))
        .field(FIELD(distance_constraint::DistanceConstraintSettings, space))
        .field(FIELD(distance_constraint::DistanceConstraintSettings, constraint_priority))
        .field(FIELD(distance_constraint::DistanceConstraintSettings, num_velocity_steps_override))
        .field(FIELD(distance_constraint::DistanceConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::distance_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::distance_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::distance_constraint::remove), names("world", "constraint"));
    bind::fn(b, BIND(adapters::distance_constraint::get_total_lambda), names("world", "constraint"));
    bind::fn(b, BIND(adapters::distance_constraint::set_distance), names("world", "constraint", "minDistance", "maxDistance"));
    bind::fn(b, BIND(adapters::distance_constraint::set_limits_spring_settings), names("world", "constraint", "settings"));

    VALUE_AS(hinge_constraint::HingeConstraintSettings, "HingeConstraintSettings")
        .field(FIELD(hinge_constraint::HingeConstraintSettings, point_a))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, point_b))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, hinge_axis_a))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, hinge_axis_b))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, normal_axis_a))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, normal_axis_b))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, space))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, limits_min))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, limits_max))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, limits_spring_settings))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, max_friction_torque))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, motor_settings))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, constraint_priority))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, num_velocity_steps_override))
        .field(FIELD(hinge_constraint::HingeConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::hinge_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::hinge_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::hinge_constraint::remove), names("world", "constraint"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_limits), names("world", "constraint", "min", "max"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_limits_spring_settings), names("world", "constraint", "settings"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_max_friction_torque), names("world", "constraint", "torque"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_motor_settings), names("world", "constraint", "settings"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_motor_state), names("world", "constraint", "state"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_target_angular_velocity), names("world", "constraint", "velocity"));
    bind::fn(b, BIND(adapters::hinge_constraint::set_target_angle), names("world", "constraint", "angle"));
    bind::fn(b, BIND(adapters::hinge_constraint::get_current_angle), names("world", "constraint"));

    VALUE_AS(slider_constraint::SliderConstraintSettings, "SliderConstraintSettings")
        .field(FIELD(slider_constraint::SliderConstraintSettings, point_a))
        .field(FIELD(slider_constraint::SliderConstraintSettings, point_b))
        .field(FIELD(slider_constraint::SliderConstraintSettings, slider_axis_a))
        .field(FIELD(slider_constraint::SliderConstraintSettings, slider_axis_b))
        .field(FIELD(slider_constraint::SliderConstraintSettings, normal_axis_a))
        .field(FIELD(slider_constraint::SliderConstraintSettings, normal_axis_b))
        .field(FIELD(slider_constraint::SliderConstraintSettings, space))
        .field(FIELD(slider_constraint::SliderConstraintSettings, auto_detect_point))
        .field(FIELD(slider_constraint::SliderConstraintSettings, limits_min))
        .field(FIELD(slider_constraint::SliderConstraintSettings, limits_max))
        .field(FIELD(slider_constraint::SliderConstraintSettings, limits_spring_settings))
        .field(FIELD(slider_constraint::SliderConstraintSettings, max_friction_force))
        .field(FIELD(slider_constraint::SliderConstraintSettings, motor_settings))
        .field(FIELD(slider_constraint::SliderConstraintSettings, constraint_priority))
        .field(FIELD(slider_constraint::SliderConstraintSettings, num_velocity_steps_override))
        .field(FIELD(slider_constraint::SliderConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::slider_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::slider_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::slider_constraint::remove), names("world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::get_current_position), names("world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::set_limits), names("world", "constraint", "min", "max"));
    bind::fn(b, BIND(adapters::slider_constraint::set_limits_spring_settings), names("world", "constraint", "settings"));
    bind::fn(b, BIND(adapters::slider_constraint::set_motor_settings), names("world", "constraint", "settings"));
    bind::fn(b, BIND(adapters::slider_constraint::set_motor_state), names("world", "constraint", "state"));
    bind::fn(b, BIND(adapters::slider_constraint::set_target_velocity), names("world", "constraint", "velocity"));
    bind::fn(b, BIND(adapters::slider_constraint::set_target_position), names("world", "constraint", "position"));
    bind::fn(b, BIND(adapters::slider_constraint::set_max_friction_force), names("world", "constraint", "force"));
    bind::fn(b, BIND(adapters::slider_constraint::get_max_friction_force), names("world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::get_total_lambda_position), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::get_total_lambda_position_limits), names("world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::get_total_lambda_rotation), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::slider_constraint::get_total_lambda_motor), names("world", "constraint"));

    VALUE_AS(fixed_constraint::FixedConstraintSettings, "FixedConstraintSettings")
        .field(FIELD(fixed_constraint::FixedConstraintSettings, point_a))
        .field(FIELD(fixed_constraint::FixedConstraintSettings, axis_x_a))
        .field(FIELD(fixed_constraint::FixedConstraintSettings, axis_y_a))
        .field(FIELD(fixed_constraint::FixedConstraintSettings, point_b))
        .field(FIELD(fixed_constraint::FixedConstraintSettings, axis_x_b))
        .field(FIELD(fixed_constraint::FixedConstraintSettings, axis_y_b))
        .field(FIELD(fixed_constraint::FixedConstraintSettings, space))
        .field(FIELD(fixed_constraint::FixedConstraintSettings, auto_detect_point))
        .field(FIELD(fixed_constraint::FixedConstraintSettings, constraint_priority))
        .field(FIELD(fixed_constraint::FixedConstraintSettings, num_velocity_steps_override))
        .field(FIELD(fixed_constraint::FixedConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::fixed_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::fixed_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::fixed_constraint::remove), names("world", "constraint"));
    bind::fn(b, BIND(adapters::fixed_constraint::get_total_lambda_position), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::fixed_constraint::get_total_lambda_rotation), names("out", "world", "constraint"));

    VALUE_AS(cone_constraint::ConeConstraintSettings, "ConeConstraintSettings")
        .field(FIELD(cone_constraint::ConeConstraintSettings, point_a))
        .field(FIELD(cone_constraint::ConeConstraintSettings, twist_axis_a))
        .field(FIELD(cone_constraint::ConeConstraintSettings, point_b))
        .field(FIELD(cone_constraint::ConeConstraintSettings, twist_axis_b))
        .field(FIELD(cone_constraint::ConeConstraintSettings, half_cone_angle))
        .field(FIELD(cone_constraint::ConeConstraintSettings, space))
        .field(FIELD(cone_constraint::ConeConstraintSettings, constraint_priority))
        .field(FIELD(cone_constraint::ConeConstraintSettings, num_velocity_steps_override))
        .field(FIELD(cone_constraint::ConeConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::cone_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::cone_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::cone_constraint::remove), names("world", "constraint"));
    bind::fn(b, BIND(adapters::cone_constraint::set_half_cone_angle), names("world", "constraint", "halfConeAngle"));
    bind::fn(b, BIND(adapters::cone_constraint::get_half_cone_angle), names("world", "constraint"));
    bind::fn(b, BIND(adapters::cone_constraint::get_cos_half_cone_angle), names("world", "constraint"));
    bind::fn(b, BIND(adapters::cone_constraint::get_total_lambda_position), names("out", "world", "constraint"));
    bind::fn(b, BIND(adapters::cone_constraint::get_total_lambda_rotation), names("world", "constraint"));

    VALUE_AS(swing_twist_constraint::SwingTwistConstraintSettings, "SwingTwistConstraintSettings")
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, point_a))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, point_b))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, twist_axis_a))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, plane_axis_a))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, twist_axis_b))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, plane_axis_b))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, space))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, swing_type))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, normal_half_cone_angle))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, plane_half_cone_angle))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, twist_min_angle))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, twist_max_angle))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, max_friction_torque))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, swing_motor_settings))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, twist_motor_settings))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, constraint_priority))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, num_velocity_steps_override))
        .field(FIELD(swing_twist_constraint::SwingTwistConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::swing_twist_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::swing_twist_constraint::remove), names("world", "constraint"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_normal_half_cone_angle), names("world", "constraint", "angle"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_plane_half_cone_angle), names("world", "constraint", "angle"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_twist_min_angle), names("world", "constraint", "angle"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_twist_max_angle), names("world", "constraint", "angle"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_max_friction_torque), names("world", "constraint", "torque"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_swing_motor_settings), names("world", "constraint", "settings"));
    bind::fn(b, BIND(adapters::swing_twist_constraint::set_twist_motor_settings), names("world", "constraint", "settings"));
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

    VALUE_AS(six_dof_constraint::SixDOFConstraintSettings, "SixDOFConstraintSettings")
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, point_a))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, point_b))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, axis_x_a))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, axis_y_a))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, axis_x_b))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, axis_y_b))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, space))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, swing_type))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, limit_min))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, limit_max))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, max_friction))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, limits_spring_settings))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, motor_settings))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, constraint_priority))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, num_velocity_steps_override))
        .field(FIELD(six_dof_constraint::SixDOFConstraintSettings, num_position_steps_override));
    bind::fn(b, BIND(adapters::six_dof_constraint::default_settings), names("out"));
    bind::fn(b, BIND(adapters::six_dof_constraint::create), names("world", "bodyA", "bodyB", "settings"), GROWS);
    bind::fn(b, BIND(adapters::six_dof_constraint::remove), names("world", "constraint"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_translation_limits), names("world", "constraint", "limitMin", "limitMax"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_rotation_limits), names("world", "constraint", "limitMin", "limitMax"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_limits_spring_settings), names("world", "constraint", "axis", "settings"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_motor_settings), names("world", "constraint", "axis", "settings"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_motor_state), names("world", "constraint", "axis", "state"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_target_velocity_cs), names("world", "constraint", "velocity"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_target_angular_velocity_cs), names("world", "constraint", "velocity"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_target_position_cs), names("world", "constraint", "position"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_target_orientation_cs), names("world", "constraint", "orientation"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_max_friction), names("world", "constraint", "axis", "friction"));
    bind::fn(b, BIND(adapters::six_dof_constraint::make_free_axis), names("world", "constraint", "axis"));
    bind::fn(b, BIND(adapters::six_dof_constraint::make_fixed_axis), names("world", "constraint", "axis"));
    bind::fn(b, BIND(adapters::six_dof_constraint::set_limited_axis), names("world", "constraint", "axis", "min", "max"));

    RECORD(filter::Filter);
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
        .value("ALL", collector::COLLECT_ALL)
        .value("ANY", collector::COLLECT_ANY)
        .value("CLOSEST", collector::COLLECT_CLOSEST);
    bind::enumeration<uint32_t>(b, "CastRayStatus")
        .value("NOT_COLLIDING", cast_ray::NOT_COLLIDING)
        .value("COLLIDING", cast_ray::COLLIDING);
    bind::enumeration<uint32_t>(b, "CastShapeStatus")
        .value("NOT_COLLIDING", cast_shape::NOT_COLLIDING)
        .value("COLLIDING", cast_shape::COLLIDING);

    VALUE_AS(cast_ray::CastRaySettings, "CastRaySettings")
        .field(FIELD(cast_ray::CastRaySettings, collide_with_backfaces_triangles))
        .field(FIELD(cast_ray::CastRaySettings, collide_with_backfaces_convex))
        .field(FIELD(cast_ray::CastRaySettings, treat_convex_as_solid));
    VALUE_AS(cast_ray::CastRayHit, "CastRayHit")
        .field(FIELD(cast_ray::CastRayHit, status))
        .field(FIELD(cast_ray::CastRayHit, fraction))
        .field(FIELD(cast_ray::CastRayHit, sub_shape_id))
        .field(FIELD(cast_ray::CastRayHit, body_id_b))
        .field(FIELD(cast_ray::CastRayHit, material_id));
    bind::fn(b, BIND(adapters::query::initial_early_out_fraction), names());
    bind::fn(b, BIND(adapters::query::unbounded_early_out), names());
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
        .field(FIELD(adapters::CastShapeSettings, active_edge_movement_direction));
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
        .field(FIELD(adapters::CastShapeHit, material_id_a))
        .field(FIELD(adapters::CastShapeHit, material_id_b))
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
        .field(FIELD(adapters::CollideShapeHit, material_id_a))
        .field(FIELD(adapters::CollideShapeHit, material_id_b))
        .field(FIELD(adapters::CollideShapeHit, body_id_b));
    bind::fn(b, BIND(adapters::query::default_collide_shape_settings), names("out"));
    bind::fn(b, BIND(adapters::query::collide_shape),
             names("world", "kind", "earlyOutFraction", "settings", "shape", "position", "quaternion", "scale", "internalEdgeRemoval", "filter",
                   "collisionGroups", "collisionMask"),
             GROWS);
    bind::fn(b, BIND(adapters::query::read_collide_shape_hit), names("out", "index"));

    VALUE_AS(collide_point::CollidePointSettings, "CollidePointSettings").field(FIELD(collide_point::CollidePointSettings, collision_tolerance));
    VALUE_AS(collide_point::CollidePointHit, "CollidePointHit")
        .field(FIELD(collide_point::CollidePointHit, sub_shape_id_b))
        .field(FIELD(collide_point::CollidePointHit, body_id_b))
        .field(FIELD(collide_point::CollidePointHit, material_id));
    bind::fn(b, BIND(adapters::query::default_collide_point_settings), names("out"));
    bind::fn(b, BIND(adapters::query::collide_point), names("world", "kind", "settings", "point", "filter", "collisionGroups", "collisionMask"),
             GROWS);
    bind::fn(b, BIND(adapters::query::read_collide_point_hit), names("out", "index"));

    RECORD(kcc_policy::Policy);
    VALUE_AS(kcc_policy::BodyPolicy, "KCCBodyPolicy")
        .field(FIELD(kcc_policy::BodyPolicy, ignore))
        .field(FIELD(kcc_policy::BodyPolicy, can_push_character))
        .field(FIELD(kcc_policy::BodyPolicy, can_receive_impulses))
        .field(FIELD(kcc_policy::BodyPolicy, slide_on_walkable))
        .field(FIELD(kcc_policy::BodyPolicy, surface_linear_velocity))
        .field(FIELD(kcc_policy::BodyPolicy, surface_angular_velocity));
    bind::fn(b, BIND(adapters::kcc_policy::default_body_policy), names("out"));
    bind::fn(b, BIND(adapters::kcc_policy::create), names(), GROWS);
    bind::fn(b, BIND(adapters::kcc_policy::destroy), names("policy"));
    bind::fn(b, BIND(adapters::kcc_policy::get_defaults), names("out", "policy"));
    bind::fn(b, BIND(adapters::kcc_policy::set_defaults), names("policy", "bodyPolicy"));
    bind::fn(b, BIND(adapters::kcc_policy::set), names("policy", "body", "bodyPolicy"), GROWS);
    bind::fn(b, BIND(adapters::kcc_policy::remove), names("policy", "body"));
    bind::fn(b, BIND(adapters::kcc_policy::get), names("out", "policy", "body"));

    RECORD(kcc::KCC);
    bind::enumeration<uint32_t>(b, "GroundState")
        .value("ON_GROUND", kcc::ON_GROUND, "walkable.")
        .value("ON_STEEP_GROUND", kcc::ON_STEEP_GROUND, "too steep to climb.")
        .value("NOT_SUPPORTED", kcc::NOT_SUPPORTED, "touching something, but not supported by it.")
        .value("IN_AIR", kcc::IN_AIR);
    bind::enumeration<uint32_t>(b, "BackFaceMode")
        .doc("triangle mesh back faces.")
        .value("IGNORE", kcc::BACK_FACE_IGNORE)
        .value("COLLIDE", kcc::BACK_FACE_COLLIDE);
    bind::enumeration<uint32_t>(b, "KCCContactEventKind")
        .value("ADDED", kcc::CONTACT_ADDED, "first met this frame, not touching last frame.")
        .value("PERSISTED", kcc::CONTACT_PERSISTED, "first met this frame, touching last frame too.")
        .value("REMOVED", kcc::CONTACT_REMOVED, "touching last frame, not at the end of this one; the body may have been removed.");
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
    VALUE_AS(kcc::UpdateSettings, "KCCUpdateSettings")
        .field(FIELD(kcc::UpdateSettings, stick_to_floor_step_down))
        .field(FIELD(kcc::UpdateSettings, walk_stairs_step_up))
        .field(FIELD(kcc::UpdateSettings, walk_stairs_min_step_forward))
        .field(FIELD(kcc::UpdateSettings, walk_stairs_step_forward_test))
        .field(FIELD(kcc::UpdateSettings, walk_stairs_cos_angle_forward_contact))
        .field(FIELD(kcc::UpdateSettings, walk_stairs_step_down_extra));
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
    bind::constant(b, "KCC_CONTACT_STRIDE", sizeof(kcc::CharacterContact));
    bind::constant(b, "KCC_CONTACT_POSITION", offsetof(kcc::CharacterContact, position));
    bind::constant(b, "KCC_CONTACT_LINEAR_VELOCITY", offsetof(kcc::CharacterContact, linear_velocity));
    bind::constant(b, "KCC_CONTACT_CONTACT_NORMAL", offsetof(kcc::CharacterContact, contact_normal));
    bind::constant(b, "KCC_CONTACT_SURFACE_NORMAL", offsetof(kcc::CharacterContact, surface_normal));
    bind::constant(b, "KCC_CONTACT_DISTANCE", offsetof(kcc::CharacterContact, distance));
    bind::constant(b, "KCC_CONTACT_FRACTION", offsetof(kcc::CharacterContact, fraction));
    bind::constant(b, "KCC_CONTACT_BODY_ID", offsetof(kcc::CharacterContact, body_id));
    bind::constant(b, "KCC_CONTACT_SUB_SHAPE_ID", offsetof(kcc::CharacterContact, sub_shape_id));
    bind::constant(b, "KCC_CONTACT_MATERIAL_ID", offsetof(kcc::CharacterContact, material_id));
    bind::constant(b, "KCC_CONTACT_MOTION_TYPE", offsetof(kcc::CharacterContact, motion_type));
    bind::constant(b, "KCC_CONTACT_IS_SENSOR", offsetof(kcc::CharacterContact, is_sensor));
    bind::constant(b, "KCC_CONTACT_HAD_COLLISION", offsetof(kcc::CharacterContact, had_collision));
    bind::constant(b, "KCC_CONTACT_WAS_DISCARDED", offsetof(kcc::CharacterContact, was_discarded));
    bind::constant(b, "KCC_CONTACT_CAN_PUSH_CHARACTER", offsetof(kcc::CharacterContact, can_push_character));
    bind::constant(b, "KCC_CONTACT_CAN_RECEIVE_IMPULSES", offsetof(kcc::CharacterContact, can_receive_impulses));

    bind::fn(b, BIND(adapters::kcc::default_settings), names("out"));
    bind::fn(b, BIND(adapters::kcc::default_update_settings), names("out"));
    bind::fn(b, BIND(adapters::kcc::create), names("settings", "position", "quaternion"), GROWS);
    bind::fn(b, BIND(adapters::kcc::destroy), names("character"));
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

    RECORD(debug::DebugBuffers);
    bind::enumeration<uint32_t>(b, "BodyColorMode")
        .value("MOTION_TYPE", debug::COLOR_MOTION_TYPE)
        .value("INSTANCE", debug::COLOR_INSTANCE, "a colour of its own per body.")
        .value("SLEEPING", debug::COLOR_SLEEPING)
        .value("ISLAND", debug::COLOR_ISLAND);
    VALUE_AS(debug::BodiesOptions, "DebugBodiesOptions")
        .field(FIELD(debug::BodiesOptions, color_mode))
        .field(FIELD(debug::BodiesOptions, show_linear_velocity))
        .field(FIELD(debug::BodiesOptions, show_angular_velocity))
        .field(FIELD(debug::BodiesOptions, excluded_object_layers));
    VALUE_AS(debug::BodyOptions, "DebugBodyOptions")
        .field(FIELD(debug::BodyOptions, has_color))
        .field(FIELD(debug::BodyOptions, color))
        .field(FIELD(debug::BodyOptions, color_mode))
        .field(FIELD(debug::BodyOptions, show_linear_velocity))
        .field(FIELD(debug::BodyOptions, show_angular_velocity));
    VALUE_AS(debug::ShapeOptions, "DebugShapeOptions").field(FIELD(debug::ShapeOptions, color));
    VALUE_AS(debug::JointsOptions, "DebugJointsOptions")
        .field(FIELD(debug::JointsOptions, size))
        .field(FIELD(debug::JointsOptions, draw_limits));
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
