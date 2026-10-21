#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/vec.h"
#include "imgui.h"
#include "sokol_app.h"
#include "tumble/physics/character/kcc.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/math/quatf.h"
#include "tumble/math/vec4f.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"

using namespace tumble;

static constexpr float PI = 3.14159265f;
static constexpr float HEIGHT_STANDING = 2.0f;
static constexpr float RADIUS_STANDING = 1.0f;
static constexpr float HEIGHT_CROUCHING = 1.0f;
static constexpr float RADIUS_CROUCHING = 0.8f;
static constexpr float SPEED = 10.0f;
static constexpr float JUMP_SPEED = 15.0f;
static constexpr bool CONTROL_MOVEMENT_DURING_JUMP = true;
static constexpr bool ENABLE_INERTIA = true;
static constexpr Float3 GRAVITY = {0.0f, -25.0f, 0.0f};
static constexpr Float3 SPAWN = {0.0f, 10.0f, 0.0f};

enum ShapeType { SHAPE_CAPSULE = 0, SHAPE_BOX = 1 };

static struct {
    app::Layers layers;
    filter::Filter filter;
    kcc::CharacterListener listener;
    kcc::UpdateSettings update_settings;
    kcc::KCC *character;
    shapes::Shape *standing[2];
    shapes::Shape *crouching[2];
    int shape_type;
    bool crouched;
    Float3 desired_velocity;
    rigid_body::RigidBody *lava;
    rigid_body::RigidBody *conveyor;
    /** the input asks to move: the character may slide on walkable slopes */
    bool allow_sliding;
    bool in_lava;
    rigid_body::RigidBody *spinning;
    rigid_body::RigidBody *sliding;
    rigid_body::RigidBody *diagonal;
    uint32_t steps;
    bool walk_stairs;
    bool stick_to_floor;
    bool show_contacts;
    bool show_ground_normal;
    bool show_ground_velocity;
    bool show_ground_position;
    bool show_velocity;
    bool show_supporting_plane;
} state;

static Float3 f3(Vec3f v) { return {vec3f::x(v), vec3f::y(v), vec3f::z(v)}; }
static Vec3f v3(Float3 f) { return vec3f::make(f.x, f.y, f.z); }

static rigid_body::RigidBody *add_static(App &app, shapes::Shape *shape, Float3 position, Float4 quaternion = {0, 0, 0, 1}) {
    rigid_body::RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = state.layers.not_moving;
    settings.motion_type = motion_type::MOTION_STATIC;
    settings.position = position;
    settings.quaternion = quaternion;
    return rigid_body::create(*app.world, settings);
}

static rigid_body::RigidBody *add_moving(App &app, shapes::Shape *shape, Float3 position, motion_type::MotionType motion) {
    rigid_body::RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = state.layers.moving;
    settings.motion_type = motion;
    settings.position = position;
    return rigid_body::create(*app.world, settings);
}

static shapes::Shape *box_of(App &app, Float3 half_extents) { return app::keep(app, box::create({.half_extents = half_extents})); }

/** the character's shapes stand on its feet: the character's position is the bottom of the shape */
static void make_character_shapes(App &app) {
    const Float3 standing_offset = {0.0f, 0.5f * HEIGHT_STANDING + RADIUS_STANDING, 0.0f};
    const Float3 crouching_offset = {0.0f, 0.5f * HEIGHT_CROUCHING + RADIUS_CROUCHING, 0.0f};
    shapes::Shape *standing_capsule = app::keep(app, capsule::create({.half_height_of_cylinder = HEIGHT_STANDING / 2, .radius = RADIUS_STANDING}));
    shapes::Shape *crouching_capsule = app::keep(app, capsule::create({.half_height_of_cylinder = HEIGHT_CROUCHING / 2, .radius = RADIUS_CROUCHING}));
    shapes::Shape *standing_box = box_of(app, {RADIUS_STANDING, HEIGHT_STANDING / 2 + RADIUS_STANDING, RADIUS_STANDING});
    shapes::Shape *crouching_box = box_of(app, {RADIUS_CROUCHING, HEIGHT_CROUCHING / 2 + RADIUS_CROUCHING, RADIUS_CROUCHING});
    const auto raised = [&](shapes::Shape *shape, Float3 offset) {
        return app::keep(app, transformed::create({.shape = shape, .position = offset, .quaternion = {0, 0, 0, 1}}));
    };
    state.standing[SHAPE_CAPSULE] = raised(standing_capsule, standing_offset);
    state.crouching[SHAPE_CAPSULE] = raised(crouching_capsule, crouching_offset);
    state.standing[SHAPE_BOX] = raised(standing_box, standing_offset);
    state.crouching[SHAPE_BOX] = raised(crouching_box, crouching_offset);
}

static void make_character(App &app) {
    kcc::KCCSettings settings;
    settings.shape = state.standing[state.shape_type];
    settings.inner_rigid_body_shape = state.standing[state.shape_type];
    settings.inner_rigid_body_object_layer = state.layers.moving;
    settings.mass = 1000.0f;
    settings.max_slope_angle = 45.0f * PI / 180.0f;
    settings.max_strength = 100.0f;
    settings.character_padding = 0.02f;
    settings.penetration_recovery_speed = 1.0f;
    settings.predictive_contact_distance = 0.1f;
    // through the bottom hemisphere's centre: only contacts below it support the character
    settings.has_supporting_volume_plane = true;
    settings.supporting_volume_plane = {{0.0f, 1.0f, 0.0f}, -RADIUS_STANDING};
    state.character = kcc::create(settings, v3(SPAWN), quatf::identity());
    state.crouched = false;
    state.desired_velocity = {0, 0, 0};
    kcc::add(*app.world, *state.character);
    app::set_color(app, rigid_body::get(*app.world, state.character->inner_rigid_body_id), {0.95f, 0.75f, 0.3f, 1.0f});
}

static void release_character(App &app) {
    kcc::remove(*app.world, *state.character);
    kcc::destroy(state.character);
    state.character = nullptr;
}

/** rolling hills, a 16 by 16 grid of 2 m cells */
static shapes::Shape *make_terrain(App &app) {
    const int grid = 16;
    const float cell = 2.0f, height = 2.0f;
    const auto height_at = [&](float x, float z) {
        return sinf(x * 0.3f) * cosf(z * 0.3f) * height * 0.5f + sinf(x * 0.7f + 1.0f) * sinf(z * 0.7f) * height * 0.3f +
               cosf((x + z) * 0.15f) * height * 0.2f;
    };
    collections::Owned<float> positions;
    collections::Owned<uint32_t> indices;
    for (int iz = 0; iz <= grid; iz++) {
        for (int ix = 0; ix <= grid; ix++) {
            const float x = (float)(ix - grid / 2) * cell, z = (float)(iz - grid / 2) * cell;
            positions.v.push(x);
            positions.v.push(height_at(x, z));
            positions.v.push(z);
        }
    }
    const uint32_t row = grid + 1;
    for (uint32_t iz = 0; iz < (uint32_t)grid; iz++) {
        for (uint32_t ix = 0; ix < (uint32_t)grid; ix++) {
            const uint32_t bl = iz * row + ix, br = bl + 1, tl = bl + row, tr = tl + 1;
            const uint32_t quad[6] = {bl, tl, br, br, tl, tr};
            for (uint32_t index : quad) indices.v.push(index);
        }
    }
    triangle_mesh_data::TriangleMeshData *data = app::keep(
        app, triangle_mesh_builder::build({.positions = positions.v.data, .num_positions = positions.v.length / 3, .indices = indices.v.data,
                                           .num_indices = indices.v.length}));
    return app::keep(app, triangle_mesh::create({.data = data}));
}

static void make_level(App &app) {
    // lava far below: falling in sends the character back to the start
    state.lava = add_static(app, box_of(app, {1000, 2, 1000}), {0, -50, 0});
    app::set_color(app, state.lava, {0.9f, 0.3f, 0.1f, 1.0f});
    state.conveyor = add_static(app, box_of(app, {10, 0.25f, 2}), {0, 0, -10});
    app::set_color(app, state.conveyor, {0.3f, 0.3f, 0.75f, 1.0f});

    add_static(app, box_of(app, {50, 0.5f, 50}), {0, -0.5f, 0});
    shapes::Shape *wall = box_of(app, {0.5f, 2, 45});
    add_static(app, wall, {-45, 1, 0});
    add_static(app, wall, {45, 1, 0});

    // five flights of stairs, their steps 0.3 to 0.7 m
    for (int j = 0; j < 5; j++) {
        const float step_height = 0.3f + 0.1f * (float)j;
        shapes::Shape *step = box_of(app, {2, step_height / 2, 2});
        for (int i = 1; i < 10; i++) {
            add_static(app, step, {15.0f + 5.0f * (float)j, (float)i * step_height - 0.5f + step_height / 2, -20.0f - (float)i * 3});
        }
    }

    // slopes from 70 down to 25 degrees
    shapes::Shape *slope = box_of(app, {2.5f, 0.6f, 8});
    for (int i = 0; i < 10; i++) {
        add_static(app, slope, {-40.0f + 5.0f * (float)i, 2, -25}, vec::quat_axis_angle({1, 0, 0}, (70.0f - (float)i * 5.0f) * PI / 180.0f));
    }

    // something to push
    rigid_body::RigidBodySettings pushable;
    pushable.object_layer = state.layers.moving;
    pushable.motion_type = motion_type::MOTION_DYNAMIC;
    pushable.friction = 0.1f;
    pushable.mass = 1.0f;
    pushable.shape = box_of(app, {0.75f, 0.75f, 0.75f});
    pushable.position = {-10, 5, 10};
    rigid_body::create(*app.world, pushable);
    pushable.shape = app::keep(app, sphere::create({.radius = 0.75f}));
    pushable.position = {-8, 2, 10};
    rigid_body::create(*app.world, pushable);

    state.spinning = add_moving(app, box_of(app, {4, 0.2f, 4}), {0, 2, 15}, motion_type::MOTION_KINEMATIC);
    state.sliding = add_moving(app, box_of(app, {3, 0.2f, 3}), {20, 2, -5}, motion_type::MOTION_KINEMATIC);
    state.diagonal = add_moving(app, box_of(app, {3, 0.2f, 3}), {20, 3, 5}, motion_type::MOTION_KINEMATIC);

    add_static(app, make_terrain(app), {-30, 2, 30});

    // a bumpy floor: spheres sunk into the ground, their tops showing
    const float bumps[][3] = {
        {25, 28, 0.8f},   {29, 32, 0.9f},  {23, 34, 0.7f},  {31, 28, 0.85f}, {27, 30, 1.0f},  {22, 30, 0.5f},  {32, 30, 0.55f}, {25, 26, 0.45f},
        {29, 34, 0.6f},   {21, 32, 0.5f},  {33, 32, 0.48f}, {27, 35, 0.52f}, {27, 25, 0.55f}, {24, 31, 0.35f}, {30, 29, 0.3f},  {26, 33, 0.38f},
        {28, 27, 0.32f},  {22, 28, 0.28f}, {32, 34, 0.34f}, {20, 30, 0.4f},  {34, 30, 0.36f}, {23, 36, 0.33f}, {31, 26, 0.31f},
    };
    for (const float *bump : bumps) add_static(app, app::keep(app, sphere::create({.radius = bump[2]})), {bump[0], bump[2] * -0.5f, bump[1]});

    // boxes doubling in size, for stepping up onto
    float x = 5;
    for (int i = 0; i < 5; i++) {
        const float half = 0.25f * (float)(1 << i);
        add_static(app, box_of(app, {half, half, half}), {x, half, 30});
        x += half * 3;
    }

    // a platform hung from four springs
    const Float3 center = {-10, 5, 35}, half = {3, 0.3f, 3};
    rigid_body::RigidBodySettings hung;
    hung.shape = box_of(app, half);
    hung.object_layer = state.layers.moving;
    hung.motion_type = motion_type::MOTION_DYNAMIC;
    hung.position = center;
    hung.linear_damping = 0.5f;
    hung.angular_damping = 0.5f;
    rigid_body::RigidBody *platform = rigid_body::create(*app.world, hung);
    shapes::Shape *nothing = app::keep(app, empty::create());
    const float corners[4][2] = {{-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
    for (const float *corner : corners) {
        const Float3 anchor_at = {center.x + corner[0] * half.x, 12, center.z + corner[1] * half.z};
        rigid_body::RigidBody *anchor = add_static(app, nothing, anchor_at);
        distance_constraint::DistanceConstraintSettings spring;
        spring.point_a = anchor_at;
        spring.point_b = {anchor_at.x, center.y + half.y, anchor_at.z};
        spring.spring_settings = {spring_settings::FREQUENCY_AND_DAMPING, 0.8f, 0.1f};
        spring.min_distance = 2;
        spring.max_distance = 8;
        distance_constraint::create(*app.world, anchor, platform, spring);
    }
}

// crashcat's listener: the conveyor's surface, the lava, and no sliding while standing still

static void on_adjust_body_velocity(const kcc::KCC &, const rigid_body::RigidBody &body, Vec3f &io_linear_velocity, Vec3f &, void *) {
    // the belt seems to move: 5 m/s along x
    if (body.id == state.conveyor->id) io_linear_velocity = vec3f::add(io_linear_velocity, vec3f::make(5.0f, 0.0f, 0.0f));
}

static bool on_contact_validate(const kcc::KCC &, const rigid_body::RigidBody &body, sub_shape::SubShapeId, Vec3f, Vec3f, void *) {
    if (body.id == state.lava->id) state.in_lava = true;
    return true;
}

static void on_contact_solve(const kcc::KCC &character, const rigid_body::RigidBody &, sub_shape::SubShapeId, Vec3f, Vec3f contact_normal,
                             Vec3f contact_velocity, Vec3f, Vec3f &io_new_character_velocity, void *) {
    if (!state.allow_sliding && vec3f::squared_length(contact_velocity) < 1e-6f && !kcc::is_slope_too_steep(character, contact_normal)) {
        io_new_character_velocity = vec3f::zero();
    }
}

static void init(App &app) {
    state = {};
    state.walk_stairs = true;
    state.stick_to_floor = true;
    app.world = app::create_world(state.layers, GRAVITY);
    state.filter = filter::create(app.world->settings.layers);
    filter::set_from_layer(state.filter, app.world->settings.layers, state.layers.moving, 0xffffffffu, 0xffffffffu);
    state.listener = {};
    state.listener.on_adjust_body_velocity = on_adjust_body_velocity;
    state.listener.on_contact_validate = on_contact_validate;
    state.listener.on_contact_solve = on_contact_solve;
    make_level(app);
    make_character_shapes(app);
    make_character(app);
    camera::look_at(app.camera, {0, 20, 20}, SPAWN);
}

static void set_crouched(App &app, bool crouched) {
    if (crouched == state.crouched) return;
    shapes::Shape *shape = crouched ? state.crouching[state.shape_type] : state.standing[state.shape_type];
    // only where it fits
    if (!kcc::set_shape(*app.world, *state.character, shape, state.filter, &state.listener, 0.1f)) return;
    state.crouched = crouched;
    state.character->supporting_volume_plane = vec4f::make(0.0f, 1.0f, 0.0f, -(crouched ? RADIUS_CROUCHING : RADIUS_STANDING));
}

static void move_platforms(App &app, float delta_time) {
    const float time = (float)state.steps * delta_time;
    rigid_body::RigidBody *spinning = state.spinning;
    const Float4 turned = vec::quat_multiply(spinning->quaternion, vec::quat_axis_angle({0, 1, 0}, delta_time));
    rigid_body::move_kinematic(*app.world, spinning, v3(spinning->position), quatf::load(turned), delta_time);
    const float slide = sinf(time * 0.15f * 2 * PI) * 8;
    rigid_body::move_kinematic(*app.world, state.sliding, vec3f::make(20 + slide, 2, -5), quatf::load(state.sliding->quaternion), delta_time);
    const float phase = time * 0.12f * 2 * PI;
    rigid_body::move_kinematic(*app.world, state.diagonal, vec3f::make(20 + sinf(phase) * 6, 3 + sinf(phase * 2) * 2, 5),
                               quatf::load(state.diagonal->quaternion), delta_time);
}

/** the velocity from the input, gravity and the ground, as crashcat's and Jolt's character samples make it */
static void steer(App &app, float delta_time) {
    kcc::KCC &character = *state.character;
    const float forward = app.keys[SAPP_KEYCODE_W] ? 1.0f : (app.keys[SAPP_KEYCODE_S] ? -1.0f : 0.0f);
    const float right = app.keys[SAPP_KEYCODE_D] ? 1.0f : (app.keys[SAPP_KEYCODE_A] ? -1.0f : 0.0f);
    const Float3 camera_forward = vec::normalize({app.camera.forward.x, 0, app.camera.forward.z});
    const Float3 camera_side = vec::normalize({app.camera.side.x, 0, app.camera.side.z});
    Float3 direction = vec::add(vec::scale(camera_forward, forward), vec::scale(camera_side, right));
    const float length = vec::length(direction);
    if (length > 1e-6f) direction = vec::scale(direction, 1.0f / length);

    const bool controls_horizontal = CONTROL_MOVEMENT_DURING_JUMP || kcc::is_supported(character);
    // no sliding down walkable slopes while standing still; in the air, slide
    state.allow_sliding = true;
    if (controls_horizontal) {
        state.allow_sliding = length > 1e-6f;
        if (ENABLE_INERTIA) {
            state.desired_velocity = vec::add(vec::scale(state.desired_velocity, 0.75f), vec::scale(direction, 0.25f * SPEED));
        } else {
            state.desired_velocity = vec::scale(direction, SPEED);
        }
    }

    kcc::update_ground_velocity(*app.world, character, &state.listener);
    const Float3 up = f3(character.up);
    const Float3 velocity = f3(character.linear_velocity);
    const Float3 vertical = vec::scale(up, vec::dot(velocity, up));
    const Float3 ground_velocity = f3(character.ground.velocity);
    const bool towards_ground = vec::dot(vec::subtract(vertical, ground_velocity), up) < 0.1f;

    Float3 next = vertical;
    if (character.ground.state == kcc::ON_GROUND) {
        const bool stick = ENABLE_INERTIA ? towards_ground : !kcc::is_slope_too_steep(character, character.ground.normal);
        if (stick) {
            next = ground_velocity;
            if (app.keys[SAPP_KEYCODE_SPACE] && towards_ground) next = vec::scale_and_add(next, up, JUMP_SPEED);
        }
    }
    next = vec::scale_and_add(next, GRAVITY, delta_time);
    next = controls_horizontal ? vec::add(next, state.desired_velocity) : vec::add(next, vec::subtract(velocity, vertical));
    character.linear_velocity = v3(next);
}

static void step(App &app, float delta_time) {
    kcc::KCC &character = *state.character;
    if (state.in_lava) {
        kcc::set_position(*app.world, character, v3(SPAWN));
        character.linear_velocity = vec3f::zero();
        app.camera.target = SPAWN;
        state.in_lava = false;
    }
    const Float3 up = f3(character.up);
    state.update_settings.stick_to_floor_step_down = state.stick_to_floor ? vec::scale(up, -0.5f) : Float3{0, 0, 0};
    state.update_settings.walk_stairs_step_up = state.walk_stairs ? vec::scale(up, 0.4f) : Float3{0, 0, 0};

    set_crouched(app, app.keys[SAPP_KEYCODE_LEFT_SHIFT] || app.keys[SAPP_KEYCODE_RIGHT_SHIFT]);
    steer(app, delta_time);
    move_platforms(app, delta_time);

    const Float3 before = f3(character.position);
    kcc::update(*app.world, character, delta_time, v3(GRAVITY), state.update_settings, &state.listener, state.filter);
    // the camera follows
    const Float3 moved = vec::subtract(f3(character.position), before);
    app.camera.target = vec::add(app.camera.target, moved);

    state.steps++;
}

static void update(App &app, float) {
    const kcc::KCC &character = *state.character;
    DrawList &frame = app::frame(app);
    const Float3 position = f3(character.position);
    if (state.show_contacts) {
        for (uint32_t i = 0; i < character.contacts.length; i++) {
            const kcc::CharacterContact &contact = character.contacts[i];
            const Float3 at = f3(contact.position);
            renderer::point(frame, at, 0.1f, {1, 1, 0, 1});
            renderer::arrow(frame, at, vec::add(at, f3(contact.contact_normal)), {0, 1, 1, 1});
            if (fabsf(vec3f::dot(contact.contact_normal, contact.surface_normal) - 1.0f) > 0.01f) {
                renderer::arrow(frame, at, vec::add(at, f3(contact.surface_normal)), {1, 0, 0, 1});
            }
        }
    }
    if (character.ground.state != kcc::IN_AIR) {
        if (state.show_ground_normal) renderer::arrow(frame, position, vec::scale_and_add(position, f3(character.ground.normal), 2), {0, 1, 0, 1});
        if (state.show_ground_velocity) renderer::arrow(frame, position, vec::add(position, f3(character.ground.velocity)), {0, 0, 1, 1});
        if (state.show_ground_position) renderer::point(frame, f3(character.ground.position), 0.15f, {1, 0.53f, 0, 1});
    }
    if (state.show_velocity) renderer::arrow(frame, position, vec::add(position, f3(character.linear_velocity)), {1, 0, 1, 1});
    if (state.show_supporting_plane) {
        const Vec4f plane = character.supporting_volume_plane;
        const Float3 normal = vec::rotate({quatf::x(character.quaternion), quatf::y(character.quaternion), quatf::z(character.quaternion),
                                           quatf::w(character.quaternion)},
                                          {vec4f::x(plane), vec4f::y(plane), vec4f::z(plane)});
        const Float3 center = vec::scale_and_add(position, normal, -vec4f::w(plane));
        const Float4 turn = vec::quat_from_to({0, 1, 0}, normal);
        const Float3 u = vec::rotate(turn, {2, 0, 0}), w = vec::rotate(turn, {0, 0, 2});
        const Float3 a = vec::subtract(vec::subtract(center, u), w), b = vec::add(vec::subtract(center, u), w);
        const Float3 c = vec::add(vec::add(center, u), w), d = vec::subtract(vec::add(center, u), w);
        renderer::triangle(frame, a, b, c, {1, 0.67f, 0, 0.3f});
        renderer::triangle(frame, a, c, d, {1, 0.67f, 0, 0.3f});
    }
}

static void ui(App &app) {
    ImGui::TextWrapped("WASD to move, space to jump, shift to crouch. Drag to look around.");
    const char *types[] = {"capsule", "box"};
    if (ImGui::Combo("shape", &state.shape_type, types, 2)) {
        release_character(app);
        make_character(app);
        app.camera.target = SPAWN;
    }
    ImGui::Checkbox("walk stairs", &state.walk_stairs);
    ImGui::Checkbox("stick to floor", &state.stick_to_floor);
    ImGui::Checkbox("enhanced internal edge removal", &state.character->enhanced_internal_edge_removal);
    const char *grounds[] = {"on ground", "on steep ground", "not supported", "in air"};
    ImGui::Text("ground: %s", grounds[state.character->ground.state]);
    if (ImGui::CollapsingHeader("character debug")) {
        ImGui::Checkbox("contacts", &state.show_contacts);
        ImGui::Checkbox("ground normal", &state.show_ground_normal);
        ImGui::Checkbox("ground velocity", &state.show_ground_velocity);
        ImGui::Checkbox("ground position", &state.show_ground_position);
        ImGui::Checkbox("velocity", &state.show_velocity);
        ImGui::Checkbox("supporting plane", &state.show_supporting_plane);
    }
}

static void shutdown(App &app) {
    release_character(app);
    filter::release(state.filter);
}

static const Example EXAMPLE_KCC = {"Kinematic Character Controller", init, step, update, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_KCC);
