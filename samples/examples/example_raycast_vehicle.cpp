#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/model.h"
#include "app/vec.h"
#include "imgui.h"
#include "sokol_app.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"

using namespace tumble;

static constexpr int WHEEL_COUNT = 4;
static constexpr float CHASSIS_HALF_WIDTH = 1.0f;
static constexpr float CHASSIS_HALF_HEIGHT = 0.55f;
static constexpr float CHASSIS_HALF_LENGTH = 2.35f;
static constexpr float WHEEL_RADIUS = 0.38f;
static constexpr float WHEEL_WIDTH = 0.3f;
static constexpr float VEHICLE_WIDTH = 2.2f;
static constexpr float VEHICLE_HEIGHT = -0.3f;
static constexpr float VEHICLE_FRONT = -1.35f;
static constexpr float VEHICLE_BACK = 1.3f;
static constexpr int WHEEL_SEGMENTS = 24;

struct WheelOptions {
    float radius;
    Vec3f direction_local;
    Vec3f axle_local;
    /** the suspension's natural frequency in Hz, and its damping ratio: 0 undamped, 1 critically damped */
    float suspension_frequency;
    float suspension_damping;
    float suspension_rest_length;
    float max_suspension_force;
    float max_suspension_travel;
    float side_friction_stiffness;
    float friction_slip;
    float roll_influence;
    float custom_sliding_rotational_speed;
    bool use_custom_sliding_rotational_speed;
    float forward_acceleration;
    float side_acceleration;
    Vec3f chassis_connection_point_local;
};

struct Wheel {
    WheelOptions options;
    float suspension_length;
    float suspension_relative_velocity;
    float suspension_force;
    float clipped_inv_contact_dot_suspension;
    bool in_contact_with_ground;
    Vec3f hit_point_world;
    Vec3f hit_normal_world;
    Vec3f direction_world;
    Vec3f axle_world;
    Vec3f chassis_connection_point_world;
    float side_impulse;
    float forward_impulse;
    Vec3f forward_world;
    Vec3f axle;
    Vec3f world_position;
    Quatf world_quaternion;
    /** its pose as the last step began, which drawing blends on from */
    Float3 before_position;
    Float4 before_quaternion;
    float engine_force;
    float brake_force;
    float steering;
    float rotation;
    float delta_rotation;
    rigid_body::RigidBody *ground_body;
    float skid_info;
    bool sliding;
};

struct AntiRollBar {
    int left_wheel;
    int right_wheel;
    float stiffness;
};

struct Tuning {
    float max_force;
    float max_steer;
    float max_brake;
    float suspension_frequency;
    float suspension_damping;
    float max_suspension_travel;
    float max_suspension_force;
    float friction_slip;
    float side_friction_stiffness;
    float roll_influence;
};

static struct {
    app::Layers layers;
    rigid_body::RigidBody *chassis;
    Wheel wheels[WHEEL_COUNT];
    AntiRollBar anti_roll_bars[2];
    bool sliding;
    filter::Filter filter;
    cast_ray::CastRayCollector collector;
    shapes::Shape *wheel_shape;
    Tuning tuning;
    bool show_suspension;
    bool show_rays;
    bool camera_rig;
    Float3 camera_position;
    Float3 camera_look_at;
} state;

static Color hex(uint32_t rgb) { return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, 1.0f}; }

static Float3 stored(Vec3f v) {
    Float3 out;
    return vec3f::store(out, v);
}

static bool not_chassis(const rigid_body::RigidBody *body, void *chassis) { return body != static_cast<const rigid_body::RigidBody *>(chassis); }

static Vec3f chassis_position() { return vec3f::load(state.chassis->position); }
static Quatf chassis_quaternion() { return quatf::load(state.chassis->quaternion); }

/** the impulse along the normal that takes out a fifth of the bodies' relative velocity there */
static float resolve_single_bilateral_constraint(rigid_body::RigidBody *body_1, Vec3f position_1, rigid_body::RigidBody *body_2, Vec3f position_2,
                                                 Vec3f normal) {
    if (vec3f::squared_length(normal) > 1.1f) return 0.0f;
    const Vec3f velocity = vec3f::subtract(rigid_body::get_velocity_at_point(body_1, position_1), rigid_body::get_velocity_at_point(body_2, position_2));
    const float relative_velocity = vec3f::dot(normal, velocity);
    const float contact_damping = 0.2f;
    const float mass_term = 1.0f / (body_1->motion_properties.inv_mass + body_2->motion_properties.inv_mass);
    return -contact_damping * relative_velocity * mass_term;
}

static float compute_impulse_denominator(rigid_body::RigidBody *body, Vec3f position, Vec3f normal) {
    Float4x4 inverse_inertia;
    rigid_body::get_inverse_inertia(inverse_inertia, body);
    const Vec3f r0 = vec3f::subtract(position, vec3f::load(body->position));
    const Vec3f c0 = vec3f::cross(r0, normal);
    const Vec3f m = mat4f::multiply3x3_vec(mat4f::load(inverse_inertia), c0);
    return body->motion_properties.inv_mass + vec3f::dot(normal, vec3f::cross(m, r0));
}

/** the impulse along the direction that stops the bodies sliding there, at most max_impulse either way */
static float calc_rolling_friction(rigid_body::RigidBody *body_0, rigid_body::RigidBody *body_1, Vec3f position, Vec3f direction, float max_impulse) {
    const Vec3f velocity = vec3f::subtract(rigid_body::get_velocity_at_point(body_0, position), rigid_body::get_velocity_at_point(body_1, position));
    const float relative_velocity = vec3f::dot(direction, velocity);
    const float denominator_0 = compute_impulse_denominator(body_0, position, direction);
    const float denominator_1 = compute_impulse_denominator(body_1, position, direction);
    const float impulse = -relative_velocity / (denominator_0 + denominator_1);
    return scalar::clamp(impulse, -max_impulse, max_impulse);
}

static void update_wheel_transform_world(Wheel &wheel) {
    const Quatf rotation = chassis_quaternion();
    wheel.chassis_connection_point_world = vec3f::add(chassis_position(), vec3f::transform_quat(wheel.options.chassis_connection_point_local, rotation));
    wheel.direction_world = vec3f::transform_quat(wheel.options.direction_local, rotation);
    wheel.axle_world = vec3f::transform_quat(wheel.options.axle_local, rotation);
}

/** each wheel's pose: on its suspension, steered, and turned about its axle */
static void update_wheel_transforms() {
    for (Wheel &wheel : state.wheels) {
        update_wheel_transform_world(wheel);
        const Vec3f up = vec3f::negate(wheel.options.direction_local);
        const Vec3f right = vec3f::normalize(wheel.options.axle_local);
        const Quatf steering = quatf::set_axis_angle(up, wheel.steering);
        const Quatf rotating = quatf::set_axis_angle(right, wheel.rotation);
        wheel.world_quaternion = quatf::normalize(quatf::multiply(quatf::multiply(chassis_quaternion(), steering), rotating));
        wheel.world_position = vec3f::scale_and_add(wheel.chassis_connection_point_world, wheel.direction_world, wheel.suspension_length);
    }
}

/** each wheel's pose where the last step left the chassis, kept to blend from */
static void keep_wheel_poses() {
    update_wheel_transforms();
    for (Wheel &wheel : state.wheels) {
        wheel.before_position = stored(wheel.world_position);
        quatf::store(wheel.before_quaternion, wheel.world_quaternion);
    }
}

static void update_wheel_suspension(world::World &world) {
    for (Wheel &wheel : state.wheels) {
        update_wheel_transform_world(wheel);
        const WheelOptions &options = wheel.options;

        // the ray covers the suspension's full travel and the wheel
        const float max_suspension_length = options.suspension_rest_length + options.max_suspension_travel;
        const float ray_length = options.radius + max_suspension_length;
        const Vec3f origin = wheel.chassis_connection_point_world;
        const Vec3f direction = vec3f::normalize(wheel.direction_world);
        cast_ray::reset(state.collector);
        query::cast_ray(world, state.collector, cast_ray::default_settings(), {origin, direction, ray_length}, state.filter);
        const cast_ray::CastRayHit *hit = cast_ray::hit(state.collector);
        rigid_body::RigidBody *hit_body = hit != nullptr ? rigid_body::get(world, hit->body_id_b) : nullptr;

        if (hit_body != nullptr) {
            wheel.ground_body = hit_body;
            wheel.in_contact_with_ground = true;
            const float hit_distance = hit->fraction * ray_length;
            wheel.hit_point_world = vec3f::scale_and_add(origin, direction, hit_distance);
            wheel.hit_normal_world = rigid_body::get_world_space_surface_normal(hit_body, hit->sub_shape_id, wheel.hit_point_world);
            wheel.suspension_length = hit_distance - options.radius;

            // clamped to the suspension's travel; past it, the wheel hangs free
            const float min_length = options.suspension_rest_length - options.max_suspension_travel;
            if (wheel.suspension_length < min_length) wheel.suspension_length = min_length;
            if (wheel.suspension_length > max_suspension_length) {
                wheel.suspension_length = max_suspension_length;
                wheel.ground_body = nullptr;
                wheel.in_contact_with_ground = false;
                wheel.hit_normal_world = vec3f::zero();
                wheel.hit_point_world = vec3f::zero();
            }

            const float denominator = vec3f::dot(wheel.hit_normal_world, wheel.direction_world);
            const Vec3f chassis_velocity = rigid_body::get_velocity_at_point(state.chassis, wheel.hit_point_world);
            const float projected_velocity = vec3f::dot(wheel.hit_normal_world, chassis_velocity);
            if (denominator >= -0.1f) {
                wheel.suspension_relative_velocity = 0.0f;
                wheel.clipped_inv_contact_dot_suspension = 1.0f / 0.1f;
            } else {
                const float inverse = -1.0f / denominator;
                wheel.suspension_relative_velocity = projected_velocity * inverse;
                wheel.clipped_inv_contact_dot_suspension = inverse;
            }
        } else {
            // at rest length
            wheel.suspension_length = options.suspension_rest_length;
            wheel.suspension_relative_velocity = 0.0f;
            wheel.hit_normal_world = vec3f::negate(wheel.direction_world);
            wheel.clipped_inv_contact_dot_suspension = 1.0f;
        }

        wheel.suspension_force = 0.0f;
        if (!wheel.in_contact_with_ground) continue;

        // stiffness and damping from the frequency and the chassis' effective mass at the mid-suspension point, so the
        // ride does not depend on how the chassis' mass is spread
        const Vec3f force_point = vec3f::scale_and_add(options.chassis_connection_point_local, options.direction_local,
                                                       0.5f * (options.suspension_rest_length + options.max_suspension_travel));
        const motion_properties::MotionProperties &motion = state.chassis->motion_properties;
        const Vec3f r_cross_up = vec3f::cross(force_point, vec3f::negate(options.direction_local));
        const float angular = vec3f::dot(r_cross_up, vec3f::multiply(r_cross_up, vec3f::load(motion.inv_inertia_diagonal)));
        const float effective_mass = 1.0f / (motion.inv_mass + angular);
        const float omega = 2.0f * scalar::PI * options.suspension_frequency;
        // stiffer on a surface at an angle to the suspension, up to ten times
        const float stiffness = effective_mass * omega * omega * wheel.clipped_inv_contact_dot_suspension;
        const float damping = 2.0f * effective_mass * options.suspension_damping * omega * wheel.clipped_inv_contact_dot_suspension;
        const float force = stiffness * (options.suspension_rest_length - wheel.suspension_length) - damping * wheel.suspension_relative_velocity;
        wheel.suspension_force = force < 0.0f ? 0.0f : force;
    }
}

static void apply_wheel_suspension_force(world::World &world, float delta_time) {
    for (const Wheel &wheel : state.wheels) {
        const float force = scalar::min(wheel.suspension_force, wheel.options.max_suspension_force);
        rigid_body::add_impulse_at_position(world, state.chassis, vec3f::scale(wheel.hit_normal_world, force * delta_time), wheel.hit_point_world);
    }

    // anti-roll bars push the more compressed side of an axle down and the other up
    for (const AntiRollBar &bar : state.anti_roll_bars) {
        if (bar.stiffness <= 0.0f) continue;
        const Wheel &left = state.wheels[bar.left_wheel];
        const Wheel &right = state.wheels[bar.right_wheel];
        if (!left.in_contact_with_ground || !right.in_contact_with_ground) continue;
        const float impulse = (right.suspension_length - left.suspension_length) * bar.stiffness * delta_time;
        rigid_body::add_impulse_at_position(world, state.chassis, vec3f::scale(left.hit_normal_world, -impulse), left.hit_point_world);
        rigid_body::add_impulse_at_position(world, state.chassis, vec3f::scale(right.hit_normal_world, impulse), right.hit_point_world);
    }
}

static void update_friction(world::World &world, float delta_time) {
    for (Wheel &wheel : state.wheels) {
        wheel.side_impulse = 0.0f;
        wheel.forward_impulse = 0.0f;
        if (!wheel.in_contact_with_ground || wheel.ground_body == nullptr) continue;
        // the axle and the forward direction in the ground's plane
        const Vec3f normal = wheel.hit_normal_world;
        const Vec3f axle = vec3f::transform_quat(vec3f::axis_x(), wheel.world_quaternion);
        wheel.axle = vec3f::normalize(vec3f::scale_and_add(axle, normal, -vec3f::dot(axle, normal)));
        wheel.forward_world = vec3f::normalize(vec3f::cross(normal, wheel.axle));
        wheel.side_impulse = resolve_single_bilateral_constraint(state.chassis, wheel.hit_point_world, wheel.ground_body, wheel.hit_point_world, wheel.axle) *
                             wheel.options.side_friction_stiffness;
    }

    const float side_factor = 1.0f;
    const float forward_factor = 0.5f;
    state.sliding = false;
    for (Wheel &wheel : state.wheels) {
        wheel.forward_impulse = 0.0f;
        wheel.skid_info = 1.0f;
        if (wheel.ground_body == nullptr) continue;

        // brake, then accelerate
        const float max_brake_impulse = wheel.brake_force != 0.0f ? wheel.brake_force : 0.0f;
        float rolling_friction = calc_rolling_friction(state.chassis, wheel.ground_body, wheel.hit_point_world, wheel.forward_world, max_brake_impulse);
        rolling_friction += wheel.engine_force * delta_time;

        // past what the suspension's load allows, the wheel skids
        const float max_impulse = wheel.suspension_force * delta_time * wheel.options.friction_slip;
        wheel.forward_impulse = rolling_friction;
        const float x = wheel.forward_impulse * forward_factor / wheel.options.forward_acceleration;
        const float y = wheel.side_impulse * side_factor / wheel.options.side_acceleration;
        const float impulse_squared = x * x + y * y;
        wheel.sliding = false;
        if (impulse_squared > max_impulse * max_impulse) {
            state.sliding = true;
            wheel.sliding = true;
            wheel.skid_info *= max_impulse / scalar::sqrt(impulse_squared);
        }
    }

    if (state.sliding) {
        for (Wheel &wheel : state.wheels) {
            if (wheel.side_impulse != 0.0f && wheel.skid_info < 1.0f) {
                wheel.forward_impulse *= wheel.skid_info;
                wheel.side_impulse *= wheel.skid_info;
            }
        }
    }

    const Vec3f position = chassis_position();
    const Quatf rotation = chassis_quaternion();
    for (Wheel &wheel : state.wheels) {
        if (wheel.forward_impulse != 0.0f) {
            rigid_body::add_impulse_at_position(world, state.chassis, vec3f::scale(wheel.forward_world, wheel.forward_impulse), wheel.hit_point_world);
        }
        if (wheel.side_impulse != 0.0f) {
            const Vec3f side_impulse = vec3f::scale(wheel.axle, wheel.side_impulse);
            // the side impulse applied lower on the chassis by the roll influence, so it rolls less in a turn
            Vec3f local = vec3f::transform_quat(vec3f::subtract(wheel.hit_point_world, position), quatf::conjugate(rotation));
            local = vec3f::with_y(local, vec3f::y(local) * wheel.options.roll_influence);
            const Vec3f at = vec3f::add(vec3f::transform_quat(local, rotation), position);
            rigid_body::add_impulse_at_position(world, state.chassis, side_impulse, at);
            rigid_body::add_impulse_at_position(world, wheel.ground_body, vec3f::negate(side_impulse), wheel.hit_point_world);
        }
    }
}

static void update_wheel_rotation(float delta_time) {
    for (Wheel &wheel : state.wheels) {
        const Vec3f velocity = rigid_body::get_velocity_at_point(state.chassis, wheel.chassis_connection_point_world);
        if (wheel.in_contact_with_ground) {
            // rolling with the ground: the chassis' velocity along its forward axis in the ground's plane
            Vec3f forward = vec3f::transform_quat(vec3f::axis_z(), chassis_quaternion());
            forward = vec3f::scale_and_add(forward, wheel.hit_normal_world, -vec3f::dot(forward, wheel.hit_normal_world));
            wheel.delta_rotation = vec3f::dot(forward, velocity) * delta_time / wheel.options.radius;
        }
        // spinning when it slides or is off the ground under power
        if ((wheel.sliding || !wheel.in_contact_with_ground) && wheel.engine_force != 0.0f && wheel.options.use_custom_sliding_rotational_speed) {
            wheel.delta_rotation = (wheel.engine_force > 0.0f ? 1.0f : -1.0f) * wheel.options.custom_sliding_rotational_speed * delta_time;
        }
        // locked under the brake
        if (scalar::abs(wheel.brake_force) > scalar::abs(wheel.engine_force)) wheel.delta_rotation = 0.0f;
        wheel.rotation += wheel.delta_rotation;
        wheel.delta_rotation *= 0.99f;
    }
}

static void update_vehicle(world::World &world, float delta_time) {
    state.sliding = false;
    for (Wheel &wheel : state.wheels) {
        wheel.in_contact_with_ground = false;
        wheel.ground_body = nullptr;
    }
    update_wheel_transforms();
    update_wheel_suspension(world);
    apply_wheel_suspension_force(world, delta_time);
    update_friction(world, delta_time);
    update_wheel_rotation(delta_time);
}

static void make_track(App &app) {
    Model track = model::load("track.glb");
    if (!track.loaded) return;
    triangle_mesh_data::TriangleMeshData *data = app::keep(app, triangle_mesh_builder::build({.positions = track.positions.data,
                                                                                               .num_positions = track.positions.length / 3,
                                                                                               .indices = track.indices.data,
                                                                                               .num_indices = track.indices.length}));
    model::release(track);
    rigid_body::RigidBodySettings settings;
    settings.shape = app::keep(app, triangle_mesh::create({.data = data}));
    settings.object_layer = state.layers.not_moving;
    settings.motion_type = motion_type::MOTION_STATIC;
    settings.restitution = 0.0f;
    settings.friction = 0.5f;
    rigid_body::create(*app.world, settings);
}

static void add_wheel(int index, Vec3f connection_point) {
    Wheel &wheel = state.wheels[index];
    wheel = {};
    wheel.options = {
        .radius = WHEEL_RADIUS,
        .direction_local = vec3f::make(0.0f, -1.0f, 0.0f),
        .axle_local = vec3f::make(1.0f, 0.0f, 0.0f),
        .suspension_frequency = 1.5f,
        .suspension_damping = 0.5f,
        .suspension_rest_length = 0.3f,
        .max_suspension_force = 100000.0f,
        .max_suspension_travel = 0.3f,
        .side_friction_stiffness = 1.5f,
        .friction_slip = 1.6f,
        .roll_influence = 0.2f,
        .custom_sliding_rotational_speed = -30.0f,
        .use_custom_sliding_rotational_speed = true,
        .forward_acceleration = 1.0f,
        .side_acceleration = 1.0f,
        .chassis_connection_point_local = connection_point,
    };
    wheel.clipped_inv_contact_dot_suspension = 1.0f;
    wheel.world_quaternion = quatf::identity();
}

/** where the camera wants to be, behind and above the chassis, and what it looks at */
static void camera_ideal(const App &app, Float3 &position, Float3 &look_at) {
    const BodyPose drawn = app::pose(app, *state.chassis);
    const Float3 chassis = drawn.position;
    const Float4 rotation = drawn.quaternion;
    position = vec::add(chassis, vec::rotate(rotation, {0.0f, 3.0f, 10.0f}));
    look_at = vec::add(chassis, vec::rotate(rotation, {0.0f, 1.0f, 0.0f}));
}

static void init(App &app) {
    app.world = app::create_world(state.layers);
    make_track(app);

    rigid_body::RigidBodySettings chassis;
    chassis.shape = app::keep(app, box::create({.half_extents = {CHASSIS_HALF_WIDTH, CHASSIS_HALF_HEIGHT, CHASSIS_HALF_LENGTH}, .convex_radius = 0.05f}));
    chassis.object_layer = state.layers.moving;
    chassis.motion_type = motion_type::MOTION_DYNAMIC;
    chassis.position = {-16.0f, 10.0f, 90.0f};
    quatf::store(chassis.quaternion, quatf::set_axis_angle(vec3f::axis_y(), -60.0f * scalar::DEGREES_TO_RADIANS));
    chassis.mass = 150.0f;
    chassis.restitution = 0.0f;
    chassis.friction = 0.5f;
    state.chassis = rigid_body::create(*app.world, chassis);
    app::set_color(app, state.chassis, hex(0x2244aa));

    // the wheels' rays see everything but the chassis
    state.filter = filter::create(app.world->settings.layers);
    filter::enable_all_layers(state.filter, app.world->settings.layers);
    state.filter.body_filter = not_chassis;
    state.filter.body_filter_context = state.chassis;
    state.collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);

    // front left, front right, back left, back right
    add_wheel(0, vec3f::make(-VEHICLE_WIDTH * 0.5f, VEHICLE_HEIGHT, VEHICLE_FRONT));
    add_wheel(1, vec3f::make(VEHICLE_WIDTH * 0.5f, VEHICLE_HEIGHT, VEHICLE_FRONT));
    add_wheel(2, vec3f::make(-VEHICLE_WIDTH * 0.5f, VEHICLE_HEIGHT, VEHICLE_BACK));
    add_wheel(3, vec3f::make(VEHICLE_WIDTH * 0.5f, VEHICLE_HEIGHT, VEHICLE_BACK));
    state.anti_roll_bars[0] = {0, 1, 500.0f};
    state.anti_roll_bars[1] = {2, 3, 500.0f};
    state.sliding = false;

    // drawn only: a wheel is a ray, not a body
    state.wheel_shape = app::keep(app, cylinder::create({.half_height = WHEEL_WIDTH / 2.0f, .radius = WHEEL_RADIUS}));

    const WheelOptions &options = state.wheels[0].options;
    state.tuning = {
        .max_force = 500.0f,
        .max_steer = 0.5f,
        .max_brake = 4.0f,
        .suspension_frequency = options.suspension_frequency,
        .suspension_damping = options.suspension_damping,
        .max_suspension_travel = options.max_suspension_travel,
        .max_suspension_force = options.max_suspension_force,
        .friction_slip = options.friction_slip,
        .side_friction_stiffness = options.side_friction_stiffness,
        .roll_influence = options.roll_influence,
    };
    state.show_suspension = true;
    state.show_rays = true;
    state.camera_rig = true;

    app.drawing.grid = false;
    app.camera.fov_y = 60.0f * scalar::DEGREES_TO_RADIANS;
    camera_ideal(app, state.camera_position, state.camera_look_at);
    camera::look_at(app.camera, state.camera_position, state.camera_look_at);
    keep_wheel_poses();
}

static void step(App &app, float delta_time) {
    keep_wheel_poses();
    const bool *keys = app.keys;
    float engine_force = 0.0f;
    float steering = 0.0f;
    if (keys[SAPP_KEYCODE_W] || keys[SAPP_KEYCODE_UP]) engine_force += state.tuning.max_force;
    if (keys[SAPP_KEYCODE_S] || keys[SAPP_KEYCODE_DOWN]) engine_force -= state.tuning.max_force;
    if (keys[SAPP_KEYCODE_A] || keys[SAPP_KEYCODE_LEFT]) steering += state.tuning.max_steer;
    if (keys[SAPP_KEYCODE_D] || keys[SAPP_KEYCODE_RIGHT]) steering -= state.tuning.max_steer;
    const float brake_force = keys[SAPP_KEYCODE_SPACE] ? state.tuning.max_brake : 0.0f;

    for (Wheel &wheel : state.wheels) {
        WheelOptions &options = wheel.options;
        options.suspension_frequency = state.tuning.suspension_frequency;
        options.suspension_damping = state.tuning.suspension_damping;
        options.max_suspension_travel = state.tuning.max_suspension_travel;
        options.max_suspension_force = state.tuning.max_suspension_force;
        options.friction_slip = state.tuning.friction_slip;
        options.side_friction_stiffness = state.tuning.side_friction_stiffness;
        options.roll_influence = state.tuning.roll_influence;
        wheel.brake_force = brake_force;
    }
    // front wheels steer, back wheels drive
    state.wheels[0].steering = steering;
    state.wheels[1].steering = steering;
    state.wheels[2].engine_force = engine_force;
    state.wheels[3].engine_force = engine_force;

    update_vehicle(*app.world, delta_time);
}

static void draw_wheel(App &app, const Wheel &wheel) {
    Float4 now;
    quatf::store(now, wheel.world_quaternion);
    const Float3 position = app::blend(app, wheel.before_position, stored(wheel.world_position));
    const Float4 rotation = app::blend(app, wheel.before_quaternion, now);
    // the cylinder's axis, y, along the axle, x
    const Float4 upright = vec::quat_multiply(rotation, vec::quat_axis_angle({0.0f, 0.0f, 1.0f}, 0.5f * scalar::PI));
    renderer::mesh(app.renderer, renderer::mesh_of(app.renderer, state.wheel_shape), matrix::compose(position, upright), hex(0x333333));

    // its edges: both rims, and the seams between the side's faces
    DrawList &frame = app::frame(app);
    const Color white = {1.0f, 1.0f, 1.0f, 1.0f};
    const auto rim = [&](float side, int segment) {
        const float angle = 2.0f * scalar::PI * (float)segment / (float)WHEEL_SEGMENTS;
        const Float3 local = {side * WHEEL_WIDTH * 0.5f, WHEEL_RADIUS * cosf(angle), WHEEL_RADIUS * sinf(angle)};
        return vec::add(position, vec::rotate(rotation, local));
    };
    for (int segment = 0; segment < WHEEL_SEGMENTS; segment++) {
        renderer::line(frame, rim(-1.0f, segment), rim(-1.0f, segment + 1), white);
        renderer::line(frame, rim(1.0f, segment), rim(1.0f, segment + 1), white);
        renderer::line(frame, rim(-1.0f, segment), rim(1.0f, segment), white);
    }
}

static void update(App &app, float delta_time) {
    // the wheels where the step left the chassis
    update_wheel_transforms();

    DrawList &frame = app::frame(app);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const Wheel &wheel = state.wheels[i];
        draw_wheel(app, wheel);

        const Float3 origin = stored(wheel.chassis_connection_point_world);
        const Float3 direction = stored(wheel.direction_world);
        const float ray_length = wheel.options.radius + wheel.options.suspension_rest_length + wheel.options.max_suspension_travel;
        const float hit_distance = wheel.in_contact_with_ground ? wheel.suspension_length + wheel.options.radius : ray_length;
        if (state.show_rays) {
            // green to the hit, red past it
            const Float3 hit = vec::scale_and_add(origin, direction, hit_distance);
            renderer::line(frame, origin, hit, {0.0f, 1.0f, 0.0f, 1.0f});
            renderer::line(frame, wheel.in_contact_with_ground ? hit : origin, vec::scale_and_add(origin, direction, ray_length), {1.0f, 0.0f, 0.0f, 0.5f});
        }
        if (state.show_suspension) {
            // beside the ray, outboard
            const Float3 offset = vec::rotate(state.chassis->quaternion, {i % 2 == 0 ? -0.1f : 0.1f, 0.0f, 0.0f});
            const Float3 top = vec::add(origin, offset);
            renderer::line(frame, top, vec::scale_and_add(top, direction, fmaxf(0.0f, wheel.suspension_length)), {0.0f, 1.0f, 1.0f, 1.0f});
        }
    }

    if (state.camera_rig) {
        Float3 ideal_position, ideal_look_at;
        camera_ideal(app, ideal_position, ideal_look_at);
        ideal_position.y = fmaxf(ideal_position.y, 0.5f);
        const float t = 1.0f - powf(0.01f, delta_time);
        state.camera_position = vec::add(state.camera_position, vec::scale(vec::subtract(ideal_position, state.camera_position), t));
        state.camera_look_at = vec::add(state.camera_look_at, vec::scale(vec::subtract(ideal_look_at, state.camera_look_at), t));
        camera::look_at(app.camera, state.camera_position, state.camera_look_at);
    } else {
        // orbiting the chassis
        app.camera.target = app::pose(app, *state.chassis).position;
    }
}

static void ui(App &) {
    if (ImGui::CollapsingHeader("vehicle debug", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("suspension lines", &state.show_suspension);
        ImGui::Checkbox("ray lines (green hit, red miss)", &state.show_rays);
        ImGui::Checkbox("camera rig", &state.camera_rig);
    }
    if (ImGui::CollapsingHeader("vehicle tuning", ImGuiTreeNodeFlags_DefaultOpen)) {
        Tuning &tuning = state.tuning;
        ImGui::SliderFloat("max force", &tuning.max_force, 0.0f, 1000.0f);
        ImGui::SliderFloat("max steer", &tuning.max_steer, 0.0f, 1.0f);
        ImGui::SliderFloat("max brake", &tuning.max_brake, 0.0f, 10.0f);
        ImGui::SliderFloat("suspension freq (hz)", &tuning.suspension_frequency, 0.5f, 5.0f);
        ImGui::SliderFloat("suspension damping", &tuning.suspension_damping, 0.0f, 1.0f);
        ImGui::SliderFloat("max suspension travel", &tuning.max_suspension_travel, 0.1f, 1.0f);
        ImGui::SliderFloat("max suspension force", &tuning.max_suspension_force, 1000.0f, 200000.0f);
        ImGui::SliderFloat("friction slip", &tuning.friction_slip, 0.1f, 5.0f);
        ImGui::SliderFloat("side friction stiffness", &tuning.side_friction_stiffness, 0.1f, 3.0f);
        ImGui::SliderFloat("roll influence", &tuning.roll_influence, 0.0f, 1.0f);
        ImGui::SliderFloat("front arb stiffness", &state.anti_roll_bars[0].stiffness, 0.0f, 2000.0f);
        ImGui::SliderFloat("rear arb stiffness", &state.anti_roll_bars[1].stiffness, 0.0f, 2000.0f);
    }
}

static void shutdown(App &) {
    filter::release(state.filter);
    cast_ray::release(state.collector);
}

static const Example EXAMPLE_RAYCAST_VEHICLE = {"Raycast Vehicle", init, step, update, ui, shutdown, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_RAYCAST_VEHICLE);
