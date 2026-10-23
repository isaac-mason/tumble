#include "app/app.h"
#include "app/examples.h"
#include "app/model.h"
#include "app/random.h"
#include "app/vec.h"
#include "imgui.h"
#include "sokol_app.h"
#include "tumble/physics/constraints/cone_constraint.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/constraints/fixed_constraint.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/constraints/six_dof_constraint.h"
#include "tumble/physics/constraints/slider_constraint.h"
#include "tumble/physics/constraints/swing_twist_constraint.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/offset_center_of_mass.h"
#include "tumble/physics/shapes/plane.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"
#include "tumble/physics/update.h"

using namespace tumble;

static constexpr float PI = 3.14159265f;
static constexpr Float3 NO_GRAVITY = {0.0f, 0.0f, 0.0f};
static constexpr Float3 EARTH_GRAVITY = {0.0f, -9.81f, 0.0f};
static constexpr Float3 TEN_GRAVITY = {0.0f, -10.0f, 0.0f};
static constexpr Float4 IDENTITY = {0.0f, 0.0f, 0.0f, 1.0f};

struct Scene {
    const char *name;
    void (*build)(App &app);
};

static struct {
    uint32_t moving;
    uint32_t not_moving;
    /** contacts between bodies that share a constraint are skipped, as crashcat's constraint scenes' listener does */
    bool skip_constrained_pairs;
    /** a scene to load at the next update, -1 for none */
    int pending_scene;
    bool previous_left;
    bool previous_right;
    bool previous_space;
    Random random;
} state;

/** the scene shown, kept over the app's restart as crashcat keeps it in the url */
static int current_scene = 0;

static Float4 rotation_x(float angle) { return vec::quat_axis_angle({1.0f, 0.0f, 0.0f}, angle); }
static Float4 rotation_y(float angle) { return vec::quat_axis_angle({0.0f, 1.0f, 0.0f}, angle); }
static Float4 rotation_z(float angle) { return vec::quat_axis_angle({0.0f, 0.0f, 1.0f}, angle); }

/** moving bodies on one layer, and not moving ones on another when `not_moving_layer` */
static void make_world(App &app, Float3 gravity, bool not_moving_layer, bool moving_vs_moving = true) {
    world_settings::WorldSettings settings = world_settings::create();
    state.moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    state.not_moving = not_moving_layer ? world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings)) : state.moving;
    if (moving_vs_moving) world_settings::enable_collision(settings, state.moving, state.moving);
    if (not_moving_layer) world_settings::enable_collision(settings, state.moving, state.not_moving);
    settings.gravity = gravity;
    app.world = world::create(settings);
    world_settings::release(settings);
}

static rigid_body::RigidBodySettings dynamic_body(shapes::Shape *shape, Float3 position, Float4 quaternion = IDENTITY) {
    rigid_body::RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = state.moving;
    settings.motion_type = motion_type::MOTION_DYNAMIC;
    settings.position = position;
    settings.quaternion = quaternion;
    return settings;
}

static rigid_body::RigidBodySettings undamped_body(shapes::Shape *shape, Float3 position, Float4 quaternion = IDENTITY) {
    rigid_body::RigidBodySettings settings = dynamic_body(shape, position, quaternion);
    settings.linear_damping = 0.0f;
    settings.angular_damping = 0.0f;
    return settings;
}

static rigid_body::RigidBodySettings static_body(shapes::Shape *shape, Float3 position, Float4 quaternion = IDENTITY) {
    rigid_body::RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = state.not_moving;
    settings.motion_type = motion_type::MOTION_STATIC;
    settings.position = position;
    settings.quaternion = quaternion;
    return settings;
}

static rigid_body::RigidBody *add(App &app, const rigid_body::RigidBodySettings &settings) { return rigid_body::create(*app.world, settings); }

static shapes::Shape *box_shape(App &app, Float3 half_extents, float convex_radius = support::DEFAULT_CONVEX_RADIUS) {
    return app::keep(app, box::create({.half_extents = half_extents, .convex_radius = convex_radius}));
}
static shapes::Shape *sphere_shape(App &app, float radius) { return app::keep(app, sphere::create({.radius = radius})); }
static shapes::Shape *capsule_shape(App &app, float half_height, float radius) {
    return app::keep(app, capsule::create({.half_height_of_cylinder = half_height, .radius = radius}));
}
static shapes::Shape *cylinder_shape(App &app, float half_height, float radius) {
    return app::keep(app, cylinder::create({.half_height = half_height, .radius = radius}));
}
static shapes::Shape *mesh_shape(App &app, const float *positions, uint32_t num_positions, const uint32_t *indices, uint32_t num_indices,
                                 float active_edge_cos_threshold_angle = 0.996195f) {
    triangle_mesh_data::TriangleMeshData *data = app::keep(app, triangle_mesh_builder::build({.positions = positions,
                                                                                              .num_positions = num_positions,
                                                                                              .indices = indices,
                                                                                              .num_indices = num_indices,
                                                                                              .active_edge_cos_threshold_angle = active_edge_cos_threshold_angle}));
    return app::keep(app, triangle_mesh::create({.data = data}));
}

/** the ground most scenes stand on: a static box, its top at y = 0 */
static rigid_body::RigidBodySettings ground_body(App &app, Float3 half_extents, float convex_radius = support::DEFAULT_CONVEX_RADIUS) {
    return static_body(box_shape(app, half_extents, convex_radius), {0.0f, -0.5f, 0.0f});
}

/* contact scenes */

/** two bodies without gravity or damping, flying at each other along x from 5 m either side */
static void head_on(App &app, shapes::Shape *shape_a, Float4 quaternion_a, shapes::Shape *shape_b, Float4 quaternion_b, float speed) {
    make_world(app, NO_GRAVITY, false);
    rigid_body::RigidBody *a = add(app, undamped_body(shape_a, {-5.0f, 0.0f, 0.0f}, quaternion_a));
    rigid_body::set_linear_velocity(*app.world, a, vec3f::make(speed, 0.0f, 0.0f));
    rigid_body::RigidBody *b = add(app, undamped_body(shape_b, {5.0f, 0.0f, 0.0f}, quaternion_b));
    rigid_body::set_linear_velocity(*app.world, b, vec3f::make(-speed, 0.0f, 0.0f));
}

static void sphere_x_sphere(App &app) {
    shapes::Shape *ball = sphere_shape(app, 1.0f);
    head_on(app, ball, IDENTITY, ball, IDENTITY, 5.0f);
}

static void plane_x_dynamic_bodies(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    const Float3 normal = vec::normalize({0.1f, 1.0f, 0.0f});
    add(app, static_body(app::keep(app, plane::create({.plane = {normal, 1.0f}, .half_extent = 100.0f})), {0.0f, 0.0f, 0.0f}));
    add(app, dynamic_body(sphere_shape(app, 0.5f), {0.0f, 1.0f, 0.0f}));
    add(app, dynamic_body(app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}})), {2.0f, 1.0f, 0.0f}));
}

static void box_x_triangle_mesh_floor(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    // every body linear casts, however slowly it moves
    app.world->settings.ccd.linear_cast_threshold = 0.0f;
    const float size = 10.0f;
    const float positions[] = {-size, 0.0f, -size, size, 0.0f, -size, size, 0.0f, size, -size, 0.0f, size};
    const uint32_t indices[] = {0, 2, 1, 0, 3, 2};
    rigid_body::RigidBodySettings floor = static_body(mesh_shape(app, positions, 4, indices, 6), {0.0f, 0.0f, 0.0f});
    floor.friction = 0.5f;
    add(app, floor);
    rigid_body::RigidBodySettings cube = dynamic_body(box_shape(app, {1.0f, 1.0f, 1.0f}), {0.0f, 5.0f, 0.0f});
    cube.friction = 0.5f;
    cube.motion_quality = motion_properties::MOTION_QUALITY_LINEAR_CAST;
    add(app, cube);
}

static void sphere_x_box_side(App &app) { head_on(app, box_shape(app, {1.0f, 1.0f, 1.0f}), IDENTITY, sphere_shape(app, 1.0f), IDENTITY, 5.0f); }

static void sphere_x_box_edge(App &app) {
    head_on(app, box_shape(app, {1.0f, 1.0f, 1.0f}), rotation_y(0.25f * PI), sphere_shape(app, 1.0f), IDENTITY, 5.0f);
}

static void sphere_x_box_corner(App &app) {
    const Float4 corner = vec::quat_multiply(rotation_y(0.25f * PI), rotation_z(0.25f * PI));
    head_on(app, box_shape(app, {1.0f, 1.0f, 1.0f}), corner, sphere_shape(app, 1.0f), IDENTITY, 5.0f);
}

static void box_x_box_face(App &app) {
    shapes::Shape *cube = box_shape(app, {1.0f, 1.0f, 1.0f});
    head_on(app, cube, IDENTITY, cube, IDENTITY, 5.0f);
}

static void box_x_box_edge(App &app) {
    shapes::Shape *cube = box_shape(app, {1.0f, 1.0f, 1.0f});
    head_on(app, cube, rotation_y(0.25f * PI), cube, IDENTITY, 5.0f);
}

static void box_x_box_corner(App &app) {
    shapes::Shape *cube = box_shape(app, {1.0f, 1.0f, 1.0f});
    const Float4 corner_a = vec::quat_multiply(rotation_y(0.25f * PI), rotation_z(0.25f * PI));
    const Float4 corner_b = vec::quat_multiply(rotation_y(-0.25f * PI), rotation_z(-0.25f * PI));
    head_on(app, cube, corner_a, cube, corner_b, 5.0f);
}

static void capsule_x_sphere(App &app) { head_on(app, capsule_shape(app, 1.0f, 1.0f), IDENTITY, sphere_shape(app, 1.0f), IDENTITY, 5.0f); }

static void capsule_x_box(App &app) { head_on(app, capsule_shape(app, 1.0f, 1.0f), IDENTITY, box_shape(app, {1.0f, 1.0f, 1.0f}), IDENTITY, 5.0f); }

/** an upright capsule coming down at 2 m/s onto a static body at the origin, without gravity */
static void capsule_down_onto(App &app, shapes::Shape *target) {
    make_world(app, NO_GRAVITY, false);
    add(app, static_body(target, {0.0f, 0.0f, 0.0f}));
    rigid_body::RigidBody *pill = add(app, undamped_body(capsule_shape(app, 1.0f, 1.0f), {0.0f, 4.0f, 0.0f}));
    rigid_body::set_linear_velocity(*app.world, pill, vec3f::make(0.0f, -2.0f, 0.0f));
}

static void capsule_top_x_box(App &app) { capsule_down_onto(app, box_shape(app, {1.0f, 1.0f, 1.0f})); }

static void capsule_top_x_sphere(App &app) { capsule_down_onto(app, sphere_shape(app, 1.0f)); }

static void capsule_edge_x_sphere(App &app) {
    make_world(app, NO_GRAVITY, false);
    add(app, static_body(sphere_shape(app, 1.0f), {0.0f, 0.0f, 0.0f}));
    rigid_body::RigidBody *pill = add(app, undamped_body(capsule_shape(app, 1.0f, 1.0f), {5.0f, 0.0f, 0.0f}, rotation_z(0.5f * PI)));
    rigid_body::set_linear_velocity(*app.world, pill, vec3f::make(-5.0f, 0.0f, 0.0f));
}

static void capsule_top_x_box_edge(App &app) {
    make_world(app, NO_GRAVITY, false);
    add(app, static_body(box_shape(app, {1.0f, 1.0f, 1.0f}), {3.0f, 1.0f, 0.0f}, rotation_y(0.25f * PI)));
    rigid_body::RigidBody *pill = add(app, undamped_body(capsule_shape(app, 1.0f, 1.0f), {-3.0f, 2.0f, 0.0f}));
    rigid_body::set_linear_velocity(*app.world, pill, vec3f::make(4.0f, 0.0f, 0.0f));
}

/** a 10 by 10 m static box floor, its top at y = 0, under 10 m/s2 of gravity */
static void box_floor_world(App &app) {
    make_world(app, TEN_GRAVITY, true);
    add(app, ground_body(app, {5.0f, 0.5f, 5.0f}));
}

static void tilted_capsule_x_static_box_floor(App &app) {
    box_floor_world(app);
    add(app, undamped_body(capsule_shape(app, 1.0f, 1.0f), {0.0f, 3.0f, 0.0f}, rotation_z(0.25f * PI)));
}

static void tilted_box_x_static_box_floor(App &app) {
    box_floor_world(app);
    add(app, undamped_body(box_shape(app, {1.0f, 1.0f, 1.0f}), {0.0f, 3.0f, 0.0f}, rotation_z(PI / 3.0f)));
}

static void box_with_convex_radius_x_static_box_floor(App &app) {
    box_floor_world(app);
    rigid_body::RigidBodySettings rounded = dynamic_body(box_shape(app, {1.0f, 1.0f, 1.0f}, 0.1f), {0.0f, 3.0f, 0.0f});
    rounded.linear_damping = 0.05f;
    rounded.angular_damping = 0.05f;
    add(app, rounded);
}

static void box_stack(App &app) {
    box_floor_world(app);
    shapes::Shape *cube = box_shape(app, {1.0f, 1.0f, 1.0f});
    const float box_height = 2.0f;
    const float start_height = box_height / 2.0f + 0.5f;
    for (int i = 0; i < 5; i++) add(app, undamped_body(cube, {0.0f, start_height + (float)i * box_height, 0.0f}));
}

static void tilted_box_falls_onto_dynamic_box(App &app) {
    box_floor_world(app);
    shapes::Shape *cube = box_shape(app, {1.0f, 1.0f, 1.0f});
    add(app, undamped_body(cube, {0.0f, 1.0f, 0.0f}));
    add(app, undamped_body(cube, {0.0f, 5.0f, 0.0f}, rotation_z(PI / 3.0f)));
}

static void tilted_box_falls_onto_static_box(App &app) {
    make_world(app, TEN_GRAVITY, true);
    shapes::Shape *cube = box_shape(app, {1.0f, 1.0f, 1.0f});
    // static, though on the moving layer, as crashcat has it
    rigid_body::RigidBodySettings pedestal = static_body(cube, {0.0f, 1.0f, 0.0f});
    pedestal.object_layer = state.moving;
    add(app, pedestal);
    add(app, undamped_body(cube, {0.0f, 5.0f, 0.0f}, rotation_z(PI / 3.0f)));
}

/* cylinder scenes */

static void cylinder_x_sphere(App &app) { head_on(app, cylinder_shape(app, 1.5f, 0.8f), IDENTITY, sphere_shape(app, 1.0f), IDENTITY, 3.0f); }

static void cylinder_x_box_edge(App &app) {
    make_world(app, NO_GRAVITY, false);
    rigid_body::RigidBody *can = add(app, undamped_body(cylinder_shape(app, 1.0f, 0.5f), {0.0f, 3.0f, 0.0f}, rotation_z(0.5f * PI)));
    rigid_body::set_linear_velocity(*app.world, can, vec3f::make(0.0f, -2.0f, 0.0f));
    add(app, static_body(box_shape(app, {2.0f, 0.2f, 0.2f}), {0.0f, 0.0f, 0.0f}));
}

static void rolling_cylinder_x_static_floor(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    rigid_body::RigidBodySettings floor = ground_body(app, {10.0f, 0.5f, 5.0f});
    floor.friction = 0.8f;
    add(app, floor);
    rigid_body::RigidBodySettings roller = dynamic_body(cylinder_shape(app, 1.0f, 0.5f), {-5.0f, 2.0f, 0.0f}, rotation_z(0.5f * PI));
    roller.friction = 0.8f;
    rigid_body::RigidBody *can = add(app, roller);
    rigid_body::set_angular_velocity(*app.world, can, vec3f::make(0.0f, 0.0f, 5.0f));
    rigid_body::set_linear_velocity(*app.world, can, vec3f::make(2.5f, 0.0f, 0.0f));
}

static void cylinder_stack(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    add(app, ground_body(app, {10.0f, 0.5f, 5.0f}));
    shapes::Shape *can = cylinder_shape(app, 0.5f, 0.4f);
    for (int i = 0; i < 6; i++) {
        rigid_body::RigidBodySettings settings = dynamic_body(can, {0.0f, (float)i + 0.5f, 0.0f});
        settings.friction = 0.5f;
        add(app, settings);
    }
}

static void tilted_cylinder_x_static_floor(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    add(app, ground_body(app, {10.0f, 0.5f, 5.0f}));
    add(app, dynamic_body(cylinder_shape(app, 1.5f, 0.5f), {0.0f, 3.0f, 0.0f}, rotation_z(PI / 6.0f)));
}

static void cylinder_x_cylinder_t_bone(App &app) {
    make_world(app, NO_GRAVITY, false);
    shapes::Shape *can = cylinder_shape(app, 1.5f, 0.4f);
    rigid_body::RigidBody *across = add(app, undamped_body(can, {0.0f, 4.0f, 0.0f}, rotation_z(0.5f * PI)));
    rigid_body::set_linear_velocity(*app.world, across, vec3f::make(0.0f, -3.0f, 0.0f));
    add(app, undamped_body(can, {0.0f, 0.0f, 0.0f}));
}

/* stacking scenes */

static void jenga(App &app) {
    make_world(app, {0.0f, -5.0f, 0.0f}, true);
    add(app, ground_body(app, {50.0f, 0.5f, 50.0f}));
    const float size = 0.5f;
    const float gap = 0.02f;
    shapes::Shape *along_z = box_shape(app, {size, size, size * 3.0f});
    shapes::Shape *along_x = box_shape(app, {size * 3.0f, size, size});
    for (int i = 0; i < 10; i++) {
        const bool even = i % 2 == 0;
        const float dx = even ? 1.0f : 0.0f;
        const float dz = even ? 0.0f : 1.0f;
        for (int j = 0; j < 3; j++) {
            const float offset = 2.0f * (size + gap) * (float)(j - 1);
            add(app, dynamic_body(even ? along_z : along_x, {offset * dx, 2.0f * (size + gap) * (float)(i + 1), offset * dz}));
        }
    }
}

static void pyramid_stress_test(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    rigid_body::RigidBodySettings floor = ground_body(app, {20.0f, 0.5f, 20.0f});
    floor.restitution = 0.3f;
    add(app, floor);
    const int tower_height = 6;
    const float box_size = 1.0f;
    shapes::Shape *cube = box_shape(app, {box_size * 0.5f, box_size * 0.5f, box_size * 0.5f}, 0.2f);
    for (int y = 0; y < tower_height; y++) {
        const int base_offset = tower_height - 1 - y;
        for (int x = -base_offset; x <= base_offset; x++) {
            for (int z = -base_offset; z <= base_offset; z++) {
                rigid_body::RigidBodySettings settings = dynamic_body(cube, {(float)x * box_size, (float)y * box_size + box_size * 0.5f, (float)z * box_size});
                settings.restitution = 0.0f;
                settings.friction = 0.5f;
                add(app, settings);
            }
        }
    }
    rigid_body::RigidBodySettings falling = dynamic_body(box_shape(app, {1.0f, 1.0f, 1.0f}), {0.0f, 20.0f, 0.0f});
    falling.restitution = 0.3f;
    add(app, falling);
}

static void box_stack_pyramid(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    rigid_body::RigidBodySettings floor = ground_body(app, {20.0f, 0.5f, 20.0f});
    floor.friction = 1.0f;
    add(app, floor);
    const int size = 10;
    shapes::Shape *cube = box_shape(app, {1.0f, 1.0f, 1.0f});
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size - i; j++) {
            for (int k = 0; k < size - i; k++) {
                const Float3 position = {2.0f * (float)j * 1.3f - (float)size + (float)i * 1.2f, (float)i * 2.2f + 1.0f,
                                         2.0f * (float)k * 1.3f - (float)size + (float)i * 1.2f};
                rigid_body::RigidBodySettings settings = dynamic_body(cube, position);
                settings.friction = 1.0f;
                add(app, settings);
            }
        }
    }
}

static void spheres_x_suzanne(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    rigid_body::RigidBodySettings floor = ground_body(app, {20.0f, 0.5f, 20.0f});
    floor.restitution = 0.3f;
    floor.friction = 0.5f;
    add(app, floor);

    Model suzanne = model::load("low-poly-suzi.glb");
    for (uint32_t i = 0; i < suzanne.positions.length; i++) suzanne.positions[i] *= 3.0f;
    rigid_body::RigidBodySettings monkey = static_body(
        mesh_shape(app, suzanne.positions.data, suzanne.positions.length / 3, suzanne.indices.data, suzanne.indices.length), {0.0f, 3.0f, 0.0f});
    model::release(suzanne);
    monkey.restitution = 0.3f;
    monkey.friction = 0.5f;
    add(app, monkey);

    shapes::Shape *ball = sphere_shape(app, 0.1f);
    for (int i = 0; i < 100; i++) {
        const float x = 0.4f + seeded::next(state.random) * 0.3f;
        const float y = 10.0f + seeded::next(state.random);
        const float z = 0.4f + seeded::next(state.random) * 0.3f;
        rigid_body::RigidBodySettings settings = dynamic_body(ball, {x * 10.0f - 5.0f, y, z * 10.0f - 5.0f});
        settings.restitution = 0.3f;
        settings.friction = 0.5f;
        add(app, settings);
    }
}

static void ccd_test(App &app) {
    make_world(app, NO_GRAVITY, true);
    add(app, static_body(box_shape(app, {0.5f, 5.0f, 5.0f}), {10.0f, 0.0f, 0.0f}));
    shapes::Shape *ball = sphere_shape(app, 0.5f);
    // the same speed: discrete tunnels through the wall, linear cast stops at it
    rigid_body::RigidBody *discrete = add(app, undamped_body(ball, {-10.0f, 2.0f, 0.0f}));
    rigid_body::set_linear_velocity(*app.world, discrete, vec3f::make(300.0f, 0.0f, 0.0f));
    rigid_body::RigidBodySettings cast = undamped_body(ball, {-10.0f, -2.0f, 0.0f});
    cast.motion_quality = motion_properties::MOTION_QUALITY_LINEAR_CAST;
    rigid_body::RigidBody *swept = add(app, cast);
    rigid_body::set_linear_velocity(*app.world, swept, vec3f::make(300.0f, 0.0f, 0.0f));
}

/* constraint scenes */

/** a 30 by 30 m static box ground, its top at y = 0 */
static void chain_ground(App &app) {
    rigid_body::RigidBodySettings ground = ground_body(app, {15.0f, 0.5f, 15.0f});
    ground.restitution = 0.3f;
    ground.friction = 0.5f;
    add(app, ground);
}

static void point_constraint_chain(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    chain_ground(app);
    const float link_spacing = 1.5f;
    rigid_body::RigidBody *previous = add(app, static_body(sphere_shape(app, 0.2f), {0.0f, 12.0f, 0.0f}));
    shapes::Shape *link_shape = box_shape(app, {0.3f, 0.3f, 0.3f});
    for (int i = 0; i < 5; i++) {
        rigid_body::RigidBodySettings settings = dynamic_body(link_shape, {0.0f, 12.0f - (float)(i + 1) * link_spacing, 0.0f});
        settings.restitution = 0.1f;
        settings.friction = 0.5f;
        rigid_body::RigidBody *link = add(app, settings);
        const float connection_y = 12.0f - ((float)i + 0.5f) * link_spacing;
        point_constraint::create(*app.world, previous, link, {.point_a = {0.0f, connection_y, 0.0f}, .point_b = {0.0f, connection_y, 0.0f}});
        // a push to start it swinging
        rigid_body::set_linear_velocity(*app.world, link, vec3f::make(1.0f, 0.0f, 0.0f));
        previous = link;
    }
    state.skip_constrained_pairs = true;
}

static void distance_constraint_chain(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    chain_ground(app);
    rigid_body::RigidBody *previous = add(app, static_body(sphere_shape(app, 0.15f), {-4.0f, 10.0f, 0.0f}));
    shapes::Shape *link_shape = sphere_shape(app, 0.25f);
    for (int i = 0; i < 8; i++) {
        rigid_body::RigidBodySettings settings = dynamic_body(link_shape, {-4.0f + (float)(i + 1), 10.0f, 0.0f});
        settings.restitution = 0.1f;
        settings.friction = 0.5f;
        rigid_body::RigidBody *link = add(app, settings);
        // min and max distance default to the distance they start at
        distance_constraint::create(*app.world, previous, link, {.point_a = previous->center_of_mass_position, .point_b = link->center_of_mass_position});
        previous = link;
    }
    state.skip_constrained_pairs = true;
}

static void distance_constraint_rope_bridge(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    chain_ground(app);
    const int bridge_length = 12;
    const float link_spacing = 0.8f;
    const float bridge_y = 8.0f;
    const float bridge_start_x = -5.0f;
    shapes::Shape *anchor_shape = box_shape(app, {0.3f, 0.3f, 0.5f});
    rigid_body::RigidBody *left_anchor = add(app, static_body(anchor_shape, {bridge_start_x, bridge_y, 0.0f}));
    rigid_body::RigidBody *right_anchor =
        add(app, static_body(anchor_shape, {bridge_start_x + (float)(bridge_length + 1) * link_spacing, bridge_y, 0.0f}));
    shapes::Shape *plank_shape = box_shape(app, {0.3f, 0.1f, 0.4f}, 0.02f);
    rigid_body::RigidBody *previous = left_anchor;
    for (int i = 0; i < bridge_length; i++) {
        rigid_body::RigidBodySettings settings = dynamic_body(plank_shape, {bridge_start_x + (float)(i + 1) * link_spacing, bridge_y, 0.0f});
        settings.restitution = 0.1f;
        settings.friction = 0.8f;
        rigid_body::RigidBody *plank = add(app, settings);
        distance_constraint::create(*app.world, previous, plank, {.point_a = previous->center_of_mass_position, .point_b = plank->center_of_mass_position});
        previous = plank;
    }
    distance_constraint::create(*app.world, previous, right_anchor,
                                {.point_a = previous->center_of_mass_position, .point_b = right_anchor->center_of_mass_position});
    rigid_body::RigidBodySettings ball = dynamic_body(sphere_shape(app, 0.5f), {0.0f, 12.0f, 0.0f});
    ball.restitution = 0.3f;
    ball.friction = 0.5f;
    add(app, ball);
    state.skip_constrained_pairs = true;
}

static void distance_constraint_spring(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    const float bar_y = 25.0f;
    rigid_body::RigidBody *bar = add(app, static_body(box_shape(app, {50.0f, 0.5f, 0.5f}), {15.0f, bar_y, 0.0f}));
    shapes::Shape *weight = box_shape(app, {0.5f, 0.5f, 0.5f});
    // a weight hung by a spring, then lifted 4 m below the bar to set it bouncing
    const auto hang = [&](float x, float length, float frequency, float damping) {
        const float y = bar_y - length;
        rigid_body::RigidBody *body = add(app, undamped_body(weight, {x, y, 0.0f}));
        distance_constraint::create(
            *app.world, bar, body,
            {.point_a = {x, bar_y, 0.0f},
             .point_b = {x, y, 0.0f},
             .spring_settings = {.mode = spring_settings::FREQUENCY_AND_DAMPING, .frequency_or_stiffness = frequency, .damping = damping}});
        rigid_body::set_position(*app.world, body, vec3f::make(x, bar_y - 4.0f, 0.0f), false);
        app::snap(app, *body);
    };
    // the same frequency, lengths of 8 to 16 m
    for (int i = 0; i < 5; i++) hang(-25.0f + (float)i * 5.0f, 8.0f + (float)i * 2.0f, 0.33f, 0.0f);
    // frequencies of 0.2 to 1 hz
    for (int i = 0; i < 5; i++) hang(5.0f + (float)i * 5.0f, 12.0f, 0.2f + (float)i * 0.2f, 0.0f);
    // damping ratios of 0 to 1
    for (int i = 0; i < 5; i++) hang(35.0f + (float)i * 5.0f, 12.0f, 0.5f, (float)i * 0.25f);
    state.skip_constrained_pairs = true;
}

static void hinge_constraint_scene(App &app) {
    make_world(app, EARTH_GRAVITY, true, false);
    shapes::Shape *cube = box_shape(app, {1.0f, 1.0f, 1.0f});
    rigid_body::RigidBody *anchor = add(app, static_body(cube, {0.0f, 10.0f, 0.0f}));
    rigid_body::RigidBody *pendulum = add(app, dynamic_body(cube, {3.0f, 10.0f, 0.0f}));
    // about z, swinging in the xy plane
    hinge_constraint::create(*app.world, anchor, pendulum,
                             {.point_a = {1.0f, 10.0f, 0.0f},
                              .point_b = {2.0f, 10.0f, 0.0f},
                              .hinge_axis_a = {0.0f, 0.0f, 1.0f},
                              .hinge_axis_b = {0.0f, 0.0f, 1.0f},
                              .normal_axis_a = {1.0f, 0.0f, 0.0f},
                              .normal_axis_b = {1.0f, 0.0f, 0.0f}});
    state.skip_constrained_pairs = true;
}

static void fixed_constraint_scene(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    add(app, ground_body(app, {50.0f, 0.5f, 50.0f}));
    shapes::Shape *cube = box_shape(app, {1.0f, 1.0f, 1.0f});
    rigid_body::RigidBody *a = add(app, dynamic_body(cube, {-1.5f, 10.0f, 0.0f}));
    rigid_body::RigidBody *b = add(app, dynamic_body(cube, {1.5f, 11.0f, 0.0f}));
    // welded: they fall and tumble as one
    fixed_constraint::create(*app.world, a, b, {.auto_detect_point = true});
    state.skip_constrained_pairs = true;
}

static void swing_twist_constraint_chain(App &app) {
    // moving bodies do not collide, so neighbouring capsules do not
    make_world(app, EARTH_GRAVITY, true, false);
    add(app, ground_body(app, {50.0f, 0.5f, 50.0f}));
    const float half_capsule_height = 1.5f;
    shapes::Shape *segment = capsule_shape(app, half_capsule_height, 0.5f);
    Float3 position = {0.0f, 25.0f, 0.0f};
    rigid_body::RigidBody *previous = nullptr;
    for (int i = 0; i < 10; i++) {
        position.x += 2.0f * half_capsule_height;
        // along x, each twisted an eighth turn more than the last
        const Float4 rotation = vec::quat_multiply(rotation_x(0.25f * PI * (float)i), rotation_z(0.5f * PI));
        rigid_body::RigidBodySettings settings = i == 0 ? static_body(segment, position, rotation) : dynamic_body(segment, position, rotation);
        settings.allow_sleeping = i == 0;
        rigid_body::RigidBody *body = add(app, settings);
        if (previous != nullptr) {
            const Float3 joint = {position.x - half_capsule_height, position.y, position.z};
            swing_twist_constraint::create(*app.world, previous, body,
                                           {.point_a = joint,
                                            .point_b = joint,
                                            .twist_axis_a = {1.0f, 0.0f, 0.0f},
                                            .plane_axis_a = {0.0f, 1.0f, 0.0f},
                                            .twist_axis_b = {1.0f, 0.0f, 0.0f},
                                            .plane_axis_b = {0.0f, 1.0f, 0.0f},
                                            .normal_half_cone_angle = PI / 4.0f,
                                            .plane_half_cone_angle = PI / 6.0f,
                                            .twist_min_angle = -PI / 4.0f,
                                            .twist_max_angle = PI / 4.0f,
                                            .max_friction_torque = 0.5f});
        }
        previous = body;
    }
    state.skip_constrained_pairs = true;
}

static void swing_twist_ragdoll_shoulder(App &app) {
    make_world(app, EARTH_GRAVITY, true, false);
    add(app, ground_body(app, {50.0f, 0.5f, 50.0f}));
    rigid_body::RigidBody *torso = add(app, static_body(box_shape(app, {2.0f, 1.5f, 1.0f}), {0.0f, 10.0f, 0.0f}));
    rigid_body::RigidBody *upper_arm = add(app, dynamic_body(capsule_shape(app, 1.5f, 0.3f), {4.0f, 10.0f, 0.0f}, rotation_z(0.5f * PI)));
    // twisting about the arm, x; swinging 90 degrees up and down and 60 forward and back
    swing_twist_constraint::create(*app.world, torso, upper_arm,
                                   {.point_a = {2.0f, 10.0f, 0.0f},
                                    .point_b = {2.5f, 10.0f, 0.0f},
                                    .twist_axis_a = {1.0f, 0.0f, 0.0f},
                                    .plane_axis_a = {0.0f, 0.0f, 1.0f},
                                    .twist_axis_b = {1.0f, 0.0f, 0.0f},
                                    .plane_axis_b = {0.0f, 0.0f, 1.0f},
                                    .normal_half_cone_angle = PI / 2.0f,
                                    .plane_half_cone_angle = PI / 3.0f,
                                    .twist_min_angle = -PI / 3.0f,
                                    .twist_max_angle = PI / 2.0f,
                                    .max_friction_torque = 1.0f});
    state.skip_constrained_pairs = true;
}

static void slider_constraint_scene(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    add(app, ground_body(app, {50.0f, 0.5f, 50.0f}));
    rigid_body::RigidBody *rail = add(app, static_body(box_shape(app, {5.0f, 0.2f, 0.2f}), {0.0f, 5.0f, 0.0f}));
    rigid_body::RigidBody *slider = add(app, dynamic_body(box_shape(app, {0.5f, 0.5f, 0.5f}), {-3.0f, 5.0f, 0.0f}));
    rigid_body::set_linear_velocity(*app.world, slider, vec3f::make(10.0f, 0.0f, 0.0f));
    slider_constraint::create(*app.world, rail, slider,
                              {.point_a = {0.0f, 5.0f, 0.0f},
                               .point_b = {-3.0f, 5.0f, 0.0f},
                               .slider_axis_a = {1.0f, 0.0f, 0.0f},
                               .slider_axis_b = {1.0f, 0.0f, 0.0f},
                               .normal_axis_a = {0.0f, 1.0f, 0.0f},
                               .normal_axis_b = {0.0f, 1.0f, 0.0f},
                               .limits_min = -4.0f,
                               .limits_max = 4.0f});
    state.skip_constrained_pairs = true;
}

static void cone_constraint_scene(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    add(app, ground_body(app, {50.0f, 0.5f, 50.0f}));
    rigid_body::RigidBody *anchor = add(app, static_body(sphere_shape(app, 0.2f), {0.0f, 5.0f, 0.0f}));
    rigid_body::RigidBody *arm = add(app, dynamic_body(capsule_shape(app, 1.0f, 0.5f), {0.0f, 3.0f, 0.0f}));
    rigid_body::set_linear_velocity(*app.world, arm, vec3f::make(10.0f, 0.0f, 0.0f));
    // swinging within 45 degrees of straight down
    cone_constraint::create(*app.world, anchor, arm,
                            {.point_a = {0.0f, 5.0f, 0.0f},
                             .twist_axis_a = {0.0f, -1.0f, 0.0f},
                             .point_b = {0.0f, 4.5f, 0.0f},
                             .twist_axis_b = {0.0f, -1.0f, 0.0f},
                             .half_cone_angle = PI / 4.0f});
    rigid_body::set_angular_velocity(*app.world, arm, vec3f::make(2.0f, 0.0f, 1.0f));
    state.skip_constrained_pairs = true;
}

/** limits per axis, translation x y z then rotation x y z */
static six_dof_constraint::SixDOFConstraintSettings six_dof(Float3 point_a, Float3 point_b, const float (&limit_min)[6], const float (&limit_max)[6]) {
    six_dof_constraint::SixDOFConstraintSettings settings;
    settings.point_a = point_a;
    settings.point_b = point_b;
    for (uint32_t axis = 0; axis < six_dof_constraint::NUM; axis++) {
        settings.limit_min[axis] = limit_min[axis];
        settings.limit_max[axis] = limit_max[axis];
    }
    return settings;
}

static void six_dof_constraint_scene(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    add(app, ground_body(app, {50.0f, 0.5f, 50.0f}));
    const float spacing = 4.0f;

    // every axis fixed: a weld
    rigid_body::RigidBody *anchor_1 = add(app, static_body(sphere_shape(app, 0.3f), {-2.0f * spacing, 5.0f, 0.0f}));
    rigid_body::RigidBody *body_1 = add(app, dynamic_body(box_shape(app, {0.5f, 0.5f, 0.5f}), {-2.0f * spacing, 4.0f, 0.0f}));
    six_dof_constraint::create(*app.world, anchor_1, body_1, six_dof({-2.0f * spacing, 4.5f, 0.0f}, {-2.0f * spacing, 4.5f, 0.0f}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}));

    // a slider: x within 2.5 m either way, the rest fixed
    rigid_body::RigidBody *anchor_2 = add(app, static_body(box_shape(app, {3.0f, 0.1f, 0.1f}), {-1.0f * spacing, 5.0f, 0.0f}));
    rigid_body::RigidBody *body_2 = add(app, dynamic_body(box_shape(app, {0.4f, 0.4f, 0.4f}), {-1.0f * spacing - 1.0f, 5.0f, 0.0f}));
    rigid_body::set_linear_velocity(*app.world, body_2, vec3f::make(5.0f, 0.0f, 0.0f));
    six_dof_constraint::create(*app.world, anchor_2, body_2,
                               six_dof({-1.0f * spacing, 5.0f, 0.0f}, {-1.0f * spacing - 1.0f, 5.0f, 0.0f}, {-2.5f, 0, 0, 0, 0, 0}, {2.5f, 0, 0, 0, 0, 0}));

    // a hinge: free about z, the rest fixed
    rigid_body::RigidBody *anchor_3 = add(app, static_body(sphere_shape(app, 0.2f), {0.0f, 5.0f, 0.0f}));
    rigid_body::RigidBody *body_3 = add(app, dynamic_body(capsule_shape(app, 1.0f, 0.2f), {0.0f, 3.5f, 0.0f}, rotation_z(0.5f * PI)));
    six_dof_constraint::SixDOFConstraintSettings hinge = six_dof({0.0f, 5.0f, 0.0f}, {-1.2f, 3.5f, 0.0f}, {0, 0, 0, 0, 0, -PI}, {0, 0, 0, 0, 0, PI});
    hinge.axis_x_b = {0.0f, 1.0f, 0.0f};
    hinge.axis_y_b = {-1.0f, 0.0f, 0.0f};
    six_dof_constraint::create(*app.world, anchor_3, body_3, hinge);

    // a ball and socket: every rotation free
    rigid_body::RigidBody *anchor_4 = add(app, static_body(sphere_shape(app, 0.3f), {1.0f * spacing, 5.0f, 0.0f}));
    rigid_body::RigidBody *body_4 = add(app, dynamic_body(capsule_shape(app, 1.0f, 0.3f), {1.0f * spacing, 3.0f, 0.0f}));
    rigid_body::set_angular_velocity(*app.world, body_4, vec3f::make(3.0f, 0.0f, 2.0f));
    six_dof_constraint::create(*app.world, anchor_4, body_4,
                               six_dof({1.0f * spacing, 5.0f, 0.0f}, {1.0f * spacing, 4.3f, 0.0f}, {0, 0, 0, -PI, -PI, -PI}, {0, 0, 0, PI, PI, PI}));

    // a joystick: tilting 45 degrees about x and z, no twist
    const float max_swing = PI / 4.0f;
    rigid_body::RigidBody *anchor_5 = add(app, static_body(sphere_shape(app, 0.3f), {2.0f * spacing, 5.0f, 0.0f}));
    rigid_body::RigidBody *body_5 = add(app, dynamic_body(capsule_shape(app, 1.0f, 0.2f), {2.0f * spacing, 3.5f, 0.0f}));
    rigid_body::set_linear_velocity(*app.world, body_5, vec3f::make(5.0f, 0.0f, 0.0f));
    six_dof_constraint::create(*app.world, anchor_5, body_5,
                               six_dof({2.0f * spacing, 5.0f, 0.0f}, {2.0f * spacing, 4.5f, 0.0f}, {0, 0, 0, -max_swing, 0, -max_swing},
                                       {0, 0, 0, max_swing, 0, max_swing}));
    state.skip_constrained_pairs = true;
}

/* body scenes */

static void offset_center_of_mass_scene(App &app) {
    make_world(app, EARTH_GRAVITY, true);
    rigid_body::RigidBodySettings floor = ground_body(app, {50.0f, 0.5f, 50.0f});
    floor.friction = 1.0f;
    add(app, floor);
    shapes::Shape *ball = sphere_shape(app, 1.0f);
    const auto offset = [&](float x) { return app::keep(app, offset_center_of_mass::create({.shape = ball, .offset = {x, 0.0f, 0.0f}})); };

    // centres of mass to the left, in the middle and to the right: the outer two roll that way
    const float xs[] = {-5.0f, 0.0f, 5.0f};
    shapes::Shape *rolling[] = {offset(-1.0f), ball, offset(1.0f)};
    for (int i = 0; i < 3; i++) {
        rigid_body::RigidBodySettings settings = dynamic_body(rolling[i], {xs[i], 5.0f, 0.0f});
        settings.friction = 1.0f;
        add(app, settings);
    }

    // floating, spun about the centre of mass 3 m to the side: once by an impulse, once by a torque over one step
    shapes::Shape *spun = offset(-3.0f);
    rigid_body::RigidBodySettings floating = undamped_body(spun, {-5.0f, 5.0f, 10.0f});
    floating.gravity_factor = 0.0f;
    rigid_body::RigidBody *by_impulse = add(app, floating);
    rigid_body::add_angular_impulse(*app.world, by_impulse, vec3f::make(0.0f, 1.0e6f, 0.0f));
    floating.position = {5.0f, 5.0f, 10.0f};
    rigid_body::RigidBody *by_torque = add(app, floating);
    rigid_body::add_torque(*app.world, by_torque, vec3f::make(0.0f, 1.0e6f * 60.0f, 0.0f), true);
}

/** square cells of `cell` metres, two triangles each, x from x_begin and z from z_begin, all on y = 0 */
static shapes::Shape *grid_mesh(App &app, float cell, int x_begin, int x_end, int z_begin, int z_end) {
    collections::Vec<float> positions = {};
    collections::Vec<uint32_t> indices = {};
    uint32_t vertex = 0;
    for (int x = x_begin; x < x_end; x++) {
        for (int z = z_begin; z < z_end; z++) {
            const float x1 = cell * (float)x;
            const float z1 = cell * (float)z;
            const float x2 = x1 + cell;
            const float z2 = z1 + cell;
            const float quad[] = {x1, 0.0f, z1, x1, 0.0f, z2, x2, 0.0f, z2, x2, 0.0f, z1};
            for (float value : quad) positions.push(value);
            const uint32_t triangles[] = {vertex, vertex + 1, vertex + 2, vertex, vertex + 2, vertex + 3};
            for (uint32_t index : triangles) indices.push(index);
            vertex += 4;
        }
    }
    // no active edges: only enhanced internal edge removal keeps bodies from catching on them
    shapes::Shape *shape = mesh_shape(app, positions.data, positions.length / 3, indices.data, indices.length, -1.0f);
    positions.release();
    indices.release();
    return shape;
}

/** a box, a sphere and eight small boxes at a big box's corners sliding along x, without and then with enhanced removal */
static void slide_across(App &app, Float3 start, bool with_corners) {
    shapes::Shape *big_box = box_shape(app, {2.0f, 2.0f, 2.0f});
    shapes::Shape *big_ball = sphere_shape(app, 2.0f);
    shapes::Shape *small_box = box_shape(app, {0.1f, 0.1f, 0.1f});
    for (int enhanced = 0; enhanced < 2; enhanced++) {
        Float3 position = {start.x, start.y, start.z - 12.0f};
        const auto slide = [&](shapes::Shape *shape, Float3 at) {
            rigid_body::RigidBodySettings settings = dynamic_body(shape, at);
            settings.enhanced_internal_edge_removal = enhanced == 1;
            rigid_body::RigidBody *body = add(app, settings);
            rigid_body::set_linear_velocity(*app.world, body, vec3f::make(20.0f, 0.0f, 0.0f));
        };
        slide(big_box, position);
        position.z += 5.0f;
        slide(big_ball, position);
        position.z += 5.0f;
        if (with_corners) {
            for (int corner = 0; corner < 8; corner++) {
                slide(small_box, {position.x + ((corner & 1) ? 1.9f : -1.9f), position.y + ((corner & 2) ? 1.9f : -1.9f),
                                  position.z + ((corner & 4) ? 1.9f : -1.9f)});
            }
        }
        start.z += 5.0f;
    }
}

static void enhanced_internal_edge_removal(App &app) {
    make_world(app, EARTH_GRAVITY, true);

    // a 20 by 20 grid of static boxes, slid across
    shapes::Shape *tile = box_shape(app, {1.0f, 1.0f, 1.0f});
    for (int x = -10; x < 10; x++) {
        for (int z = -10; z < 10; z++) add(app, static_body(tile, {2.0f * (float)x, -1.0f, 2.0f * (float)z - 40.0f}));
    }
    slide_across(app, {-18.0f, 1.9f, -40.0f}, true);

    // a 20 by 20 grid of triangle pairs, slid across
    add(app, static_body(grid_mesh(app, 2.0f, -10, 10, -10, 10), {0.0f, 0.0f, 0.0f}));
    slide_across(app, {-18.0f, 1.9f, 0.0f}, false);

    // an l: a floor and a step up at its edge, which must stay an edge
    {
        const float height = 0.5f;
        const float half_width = 5.0f;
        const float half_length = 2.0f;
        const float positions[] = {
            -half_length, 0.0f,   half_width, half_length, 0.0f,   -half_width, -half_length, 0.0f, -half_width, half_length, 0.0f, half_width,
            half_length,  height, half_width, half_length, height, -half_width, half_length,  0.0f, half_width,  half_length, 0.0f, -half_width,
        };
        const uint32_t indices[] = {0, 1, 2, 0, 3, 1, 4, 5, 6, 6, 5, 7};
        add(app, static_body(mesh_shape(app, positions, 8, indices, 12), {0.0f, 0.0f, 30.0f}));
        shapes::Shape *ball = sphere_shape(app, 1.0f);
        float z = 28.0f;
        for (int enhanced = 0; enhanced < 2; enhanced++) {
            rigid_body::RigidBodySettings settings = dynamic_body(ball, {0.0f, 1.0f, z});
            settings.enhanced_internal_edge_removal = enhanced == 1;
            rigid_body::RigidBody *body = add(app, settings);
            rigid_body::set_linear_velocity(*app.world, body, vec3f::make(20.0f, 0.0f, 0.0f));
            z += 4.0f;
        }
    }

    // balls rolling on a flat square and down the same square tilted 45 degrees
    {
        const float positions[] = {-10.0f, 0.0f, -10.0f, -10.0f, 0.0f, 10.0f, 10.0f, 0.0f, 10.0f, 10.0f, 0.0f, -10.0f};
        const uint32_t indices[] = {0, 1, 2, 0, 2, 3};
        shapes::Shape *square = mesh_shape(app, positions, 4, indices, 6);
        shapes::Shape *ball = sphere_shape(app, 0.5f);
        const Float4 tilts[] = {IDENTITY, rotation_x(0.25f * PI)};
        const Float3 squares[] = {{-10.0f, 0.0f, 50.0f}, {10.0f, 0.0f, 50.0f}};
        const Float3 balls[] = {{-10.0f, 1.0f, 41.0f}, {10.0f, 8.0f, 44.0f}};
        for (int i = 0; i < 2; i++) {
            rigid_body::RigidBodySettings ground = static_body(square, squares[i], tilts[i]);
            ground.friction = 1.0f;
            add(app, ground);
            rigid_body::RigidBodySettings rolling = dynamic_body(ball, balls[i]);
            rolling.enhanced_internal_edge_removal = true;
            rolling.friction = 1.0f;
            add(app, rolling);
        }
    }

    // a long thin box and a plate at its end, two bodies as crashcat has them, dropped onto a static box
    {
        add(app, static_body(box_shape(app, {2.5f, 2.5f, 2.5f}), {0.0f, 0.0f, 70.0f}));
        rigid_body::RigidBodySettings bar = dynamic_body(box_shape(app, {2.5f, 0.1f, 0.1f}), {2.0f - 2.5f, 5.0f, 70.0f});
        bar.enhanced_internal_edge_removal = true;
        add(app, bar);
        rigid_body::RigidBodySettings plate = dynamic_body(box_shape(app, {0.1f, 1.0f, 1.0f}), {2.0f + 0.1f, 5.0f, 70.0f});
        plate.enhanced_internal_edge_removal = true;
        add(app, plate);
    }

    // a 200 by 10 grid of small triangle pairs, a box sliding fast across it
    {
        add(app, static_body(grid_mesh(app, 0.25f, -100, 100, -5, 5), {0.0f, 0.0f, 80.0f}));
        rigid_body::RigidBodySettings fast = dynamic_body(box_shape(app, {1.0f, 1.0f, 1.0f}), {-24.0f, 0.9f, 80.0f});
        fast.enhanced_internal_edge_removal = true;
        rigid_body::RigidBody *body = add(app, fast);
        rigid_body::set_linear_velocity(*app.world, body, vec3f::make(20.0f, 0.0f, 0.0f));
    }
}

static const Scene SCENES[] = {
    {"Sphere x Sphere", sphere_x_sphere},
    {"Plane x Dynamic Bodies", plane_x_dynamic_bodies},
    {"Box x Triangle Mesh Floor", box_x_triangle_mesh_floor},
    {"Sphere x Box Side", sphere_x_box_side},
    {"Sphere x Box Edge", sphere_x_box_edge},
    {"Sphere x Box Corner", sphere_x_box_corner},
    {"Box x Box Face", box_x_box_face},
    {"Box x Box Edge", box_x_box_edge},
    {"Box x Box Corner", box_x_box_corner},
    {"Capsule x Sphere", capsule_x_sphere},
    {"Capsule x Box", capsule_x_box},
    {"Capsule Top x Box", capsule_top_x_box},
    {"Capsule Top x Sphere", capsule_top_x_sphere},
    {"Capsule Edge x Sphere", capsule_edge_x_sphere},
    {"Capsule Top x Box Edge", capsule_top_x_box_edge},
    {"Tilted Capsule x Static Box Floor", tilted_capsule_x_static_box_floor},
    {"Tilted Box x Static Box Floor", tilted_box_x_static_box_floor},
    {"Box with Convex Radius x Static Box Floor", box_with_convex_radius_x_static_box_floor},
    {"Box Stack", box_stack},
    {"Tilted Box Falls onto Dynamic Box", tilted_box_falls_onto_dynamic_box},
    {"Tilted Box Falls onto Static Box", tilted_box_falls_onto_static_box},
    {"Cylinder x Sphere", cylinder_x_sphere},
    {"Cylinder x Box Edge", cylinder_x_box_edge},
    {"Rolling Cylinder x Static Floor", rolling_cylinder_x_static_floor},
    {"Cylinder Stack", cylinder_stack},
    {"Tilted Cylinder x Static Floor", tilted_cylinder_x_static_floor},
    {"Cylinder x Cylinder T-Bone", cylinder_x_cylinder_t_bone},
    {"Jenga", jenga},
    {"Pyramid Stress Test", pyramid_stress_test},
    {"Box Stack Pyramid", box_stack_pyramid},
    {"100 Spheres x Suzanne", spheres_x_suzanne},
    {"CCD Test", ccd_test},
    {"Point Constraint - Chain", point_constraint_chain},
    {"Distance Constraint - Chain", distance_constraint_chain},
    {"Distance Constraint - Rope Bridge", distance_constraint_rope_bridge},
    {"Distance Constraint - Spring", distance_constraint_spring},
    {"Hinge Constraint", hinge_constraint_scene},
    {"Fixed Constraint", fixed_constraint_scene},
    {"SwingTwist Constraint Chain", swing_twist_constraint_chain},
    {"SwingTwist Ragdoll Shoulder", swing_twist_ragdoll_shoulder},
    {"Slider Constraint", slider_constraint_scene},
    {"Cone Constraint", cone_constraint_scene},
    {"SixDOF Constraint", six_dof_constraint_scene},
    {"Offset Center of Mass", offset_center_of_mass_scene},
    {"Enhanced Internal Edge Removal", enhanced_internal_edge_removal},
};
static constexpr int SCENE_COUNT = (int)(sizeof(SCENES) / sizeof(SCENES[0]));

static void load_scene(App &app, int index) {
    current_scene = index;
    state.skip_constrained_pairs = false;
    state.random = {1};
    SCENES[index].build(app);
}

static void init(App &app) {
    state.pending_scene = -1;
    state.previous_left = false;
    state.previous_right = false;
    state.previous_space = false;
    // crashcat draws no grid, and many scenes are in the air around the origin
    app.drawing.grid = false;
    load_scene(app, current_scene);
    camera::look_at(app.camera, {0.0f, 5.0f, 15.0f}, {0.0f, 0.0f, 0.0f});
}

static void step_world(App &app, float delta_time) {
    world::World &world = *app.world;
    update::step_begin(world, delta_time);
    do {
        update::find_pairs(world);
        if (state.skip_constrained_pairs) {
            for (uint32_t i = 0; i < world.events.pairs.length; i++) {
                const events::PairEvent &pair = world.events.pairs[i];
                if (rigid_body::bodies_share_constraint(world.bodies.pool[pair.body_index_a], world.bodies.pool[pair.body_index_b])) {
                    events::reject_pair(world.answers, i);
                }
            }
        }
        update::narrowphase(world);
    } while (world.events.woken > 0);
    update::solve(world);
    update::step_end(world);
}

/** true the frame a key goes down */
static bool pressed(App &app, int key, bool &previous) {
    const bool down = app.keys[key];
    const bool went_down = down && !previous;
    previous = down;
    return went_down;
}

static void switch_scenes(App &app, float) {
    if (pressed(app, SAPP_KEYCODE_RIGHT, state.previous_right)) state.pending_scene = (current_scene + 1) % SCENE_COUNT;
    if (pressed(app, SAPP_KEYCODE_LEFT, state.previous_left)) state.pending_scene = (current_scene - 1 + SCENE_COUNT) % SCENE_COUNT;
    if (pressed(app, SAPP_KEYCODE_SPACE, state.previous_space)) state.pending_scene = current_scene;
    // here, before the frame draws: ui runs after its meshes are queued
    if (state.pending_scene < 0) return;
    app::clear_world(app);
    load_scene(app, state.pending_scene);
    state.pending_scene = -1;
}

static void ui(App &) {
    if (ImGui::CollapsingHeader("Scenes", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (int i = 0; i < SCENE_COUNT; i++) {
            if (ImGui::Selectable(SCENES[i].name, i == current_scene)) state.pending_scene = i;
        }
    }
    if (ImGui::Button("previous (left)")) state.pending_scene = (current_scene - 1 + SCENE_COUNT) % SCENE_COUNT;
    ImGui::SameLine();
    if (ImGui::Button("next (right)")) state.pending_scene = (current_scene + 1) % SCENE_COUNT;
    ImGui::SameLine();
    if (ImGui::Button("restart (space)")) state.pending_scene = current_scene;
}

static const Example EXAMPLE_TEST_SCENARIOS = {"Test Scenarios", init, nullptr, switch_scenes, ui, nullptr, step_world};
static const examples::Registration REGISTERED(EXAMPLE_TEST_SCENARIOS);
