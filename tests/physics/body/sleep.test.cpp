#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/sleep.h"
#include "test.h"
#include "test_math.h"

using rigid_body::RigidBody;

// crashcat's default sleep settings: time_before_sleep 0.5 s, point velocity threshold 0.03 m/s,
// so a test point may wander max_movement = 0.03 * 0.5 m before the timer restarts
static constexpr float TIME_BEFORE_SLEEP = 0.5f;
static constexpr float MAX_MOVEMENT = 0.03f * TIME_BEFORE_SLEEP;
static constexpr float DT = 1.0f / 60.0f;

/** the stored point within 1e-5 of (x, y, z) */
static bool near_point(const Float3 &a, float x, float y, float z) { return vec3f::distance(vec3f::load(a), vec3f::make(x, y, z)) < 1e-5f; }

/** a dynamic body over `shape` (whose aabb the test sets) at `com`, unrotated, allowed to sleep */
static RigidBody make_body(shapes::Shape &shape, const Box3f &aabb, const Float3 &com) {
    shape = {};
    shape.type = shapes::SHAPE_BOX;
    shape.aabb = aabb;
    RigidBody body = {};
    body.shape = &shape;
    body.motion_type = motion_type::MOTION_DYNAMIC;
    body.center_of_mass_position = com;
    body.quaternion = {0.0f, 0.0f, 0.0f, 1.0f};
    body.motion_properties = motion_properties::create();
    return body;
}

void test_sleep_test_points(TestCtx &ctx) {
    // half extents 0.5, 1, 2: x is smallest, so the points sit on the y and z axes
    const Box3f aabb = {{-0.5f, -1.0f, -2.0f}, {0.5f, 1.0f, 2.0f}};
    shapes::Shape shape;
    RigidBody body = make_body(shape, aabb, {1.0f, 2.0f, 3.0f});
    Float3 points[3];
    sleep::get_sleep_test_points(points, &body);
    CHECK(ctx, near_point(points[0], 1.0f, 2.0f, 3.0f));
    CHECK(ctx, near_point(points[1], 1.0f, 3.0f, 3.0f));
    CHECK(ctx, near_point(points[2], 1.0f, 2.0f, 5.0f));

    // a quarter turn about x carries y to z and z to -y
    body.quaternion = stored_axis_angle(vec3f::make(1.0f, 0.0f, 0.0f), scalar::PI * 0.5f);
    sleep::get_sleep_test_points(points, &body);
    CHECK(ctx, near_point(points[1], 1.0f, 2.0f, 4.0f));
    CHECK(ctx, near_point(points[2], 1.0f, 0.0f, 3.0f));

    // y smallest: x and z axes; z smallest: x and y axes
    body.quaternion = {0.0f, 0.0f, 0.0f, 1.0f};
    shape.aabb = {{-2.0f, -0.1f, -1.0f}, {2.0f, 0.1f, 1.0f}};
    sleep::get_sleep_test_points(points, &body);
    CHECK(ctx, near_point(points[1], 3.0f, 2.0f, 3.0f));
    CHECK(ctx, near_point(points[2], 1.0f, 2.0f, 4.0f));
    shape.aabb = {{-2.0f, -1.0f, -0.1f}, {2.0f, 1.0f, 0.1f}};
    sleep::get_sleep_test_points(points, &body);
    CHECK(ctx, near_point(points[1], 3.0f, 2.0f, 3.0f));
    CHECK(ctx, near_point(points[2], 1.0f, 3.0f, 3.0f));

    // equal extents: a cube tests its x and z axes
    shape.aabb = {{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};
    sleep::get_sleep_test_points(points, &body);
    CHECK(ctx, near_point(points[1], 1.5f, 2.0f, 3.0f));
    CHECK(ctx, near_point(points[2], 1.0f, 2.0f, 3.5f));
}

/** steps a body moving at `velocity` and spinning at `spin` rad/s about y; the step on which it may first sleep, or -1. */
static int steps_until_sleep(TestCtx &ctx, const Float3 &velocity, float spin, int max_steps) {
    shapes::Shape shape;
    RigidBody body = make_body(shape, {{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}}, {0.0f, 1.0f, 0.0f});
    sleep::reset_sleep_timer(&body);
    CHECK(ctx, body.motion_properties.sleep_test_timer == 0.0f && body.motion_properties.sleep_test_spheres[0].radius == 0.0f);
    for (int step = 1; step <= max_steps; step++) {
        vec3f::store(body.center_of_mass_position, vec3f::scale_and_add(vec3f::load(body.center_of_mass_position), vec3f::load(velocity), DT));
        quatf::store(body.quaternion, quatf::multiply(axis_angle(vec3f::axis_y(), spin * DT), quatf::load(body.quaternion)));
        if (sleep::update_sleep_state(&body, DT, MAX_MOVEMENT, TIME_BEFORE_SLEEP)) {
            return step;
        }
    }
    return -1;
}

void test_sleep_update_sleep_state(TestCtx &ctx) {
    // at rest the timer runs out after time_before_sleep: 30 steps of 1/60 s, give or take float rounding
    const int at_rest = steps_until_sleep(ctx, {0.0f, 0.0f, 0.0f}, 0.0f, 120);
    CHECK(ctx, at_rest >= 30 && at_rest <= 31);

    // creeping below the threshold (0.02 m/s covers 0.01 m in 0.5 s) still sleeps on time
    const int creeping = steps_until_sleep(ctx, {0.02f, 0.0f, 0.0f}, 0.0f, 120);
    CHECK(ctx, creeping == at_rest);

    // each sphere grows to just hold its point's path, so a straight drift is measured by half its length: 0.05 m/s
    // covers 0.025 m in 0.5 s, a 0.0125 m sphere, inside the 0.015 m allowed, and sleeps on time
    CHECK(ctx, steps_until_sleep(ctx, {0.05f, 0.0f, 0.0f}, 0.0f, 120) == at_rest);

    // 0.07 m/s spans the 0.03 m diameter in 0.43 s, so the timer keeps restarting
    CHECK(ctx, steps_until_sleep(ctx, {0.07f, 0.0f, 0.0f}, 0.0f, 600) == -1);

    // spinning in place: the centre of mass stays but the axis points sweep, so no sleep
    CHECK(ctx, steps_until_sleep(ctx, {0.0f, 0.0f, 0.0f}, 1.0f, 600) == -1);
}

void test_sleep_reset_and_exclusions(TestCtx &ctx) {
    shapes::Shape shape;
    RigidBody body = make_body(shape, {{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}}, {0.0f, 0.0f, 0.0f});
    motion_properties::MotionProperties &mp = body.motion_properties;
    sleep::reset_sleep_timer(&body);

    // a sensor, or a body not allowed to sleep, never may and its timer does not run
    body.sensor = true;
    CHECK(ctx, !sleep::update_sleep_state(&body, 10.0f, MAX_MOVEMENT, TIME_BEFORE_SLEEP));
    body.sensor = false;
    mp.allow_sleeping = false;
    CHECK(ctx, !sleep::update_sleep_state(&body, 10.0f, MAX_MOVEMENT, TIME_BEFORE_SLEEP));
    mp.allow_sleeping = true;
    CHECK(ctx, mp.sleep_test_timer == 0.0f);

    // a jump past max_movement recentres every sphere on the new points and zeroes the timer
    mp.sleep_test_timer = 0.4f;
    body.center_of_mass_position = {1.0f, 0.0f, 0.0f};
    CHECK(ctx, !sleep::update_sleep_state(&body, DT, MAX_MOVEMENT, TIME_BEFORE_SLEEP));
    CHECK(ctx, mp.sleep_test_timer == 0.0f);
    CHECK(ctx, near_point(mp.sleep_test_spheres[0].center, 1.0f, 0.0f, 0.0f) && mp.sleep_test_spheres[0].radius == 0.0f);
    CHECK(ctx, mp.sleep_test_spheres[1].radius == 0.0f && mp.sleep_test_spheres[2].radius == 0.0f);

    // resetting the timer only touches dynamic bodies
    body.center_of_mass_position = {0.0f, 0.0f, 0.0f};
    mp.sleep_test_timer = 0.3f;
    body.motion_type = motion_type::MOTION_KINEMATIC;
    sleep::reset_sleep_timer(&body);
    CHECK(ctx, mp.sleep_test_timer == 0.3f);
    CHECK(ctx, near_point(mp.sleep_test_spheres[0].center, 1.0f, 0.0f, 0.0f));
    body.motion_type = motion_type::MOTION_DYNAMIC;
    sleep::reset_sleep_timer(&body);
    CHECK(ctx, mp.sleep_test_timer == 0.0f && near_point(mp.sleep_test_spheres[0].center, 0.0f, 0.0f, 0.0f));

    CHECK(ctx, sleep::INACTIVE_BODY_INDEX == 0xffffffffu);
}
