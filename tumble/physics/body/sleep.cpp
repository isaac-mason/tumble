#include "tumble/physics/body/sleep.h"

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"

TUMBLE_NAMESPACE_BEGIN

namespace sleep {

/** column i of the rotation matrix of q, straight from the quaternion */
static Vec3f rotation_axis(Quatf q, int i) {
    const float qx = quatf::x(q), qy = quatf::y(q), qz = quatf::z(q), qw = quatf::w(q);
    if (i == 0) return vec3f::make(1.0f - 2.0f * (qy * qy + qz * qz), 2.0f * (qx * qy + qw * qz), 2.0f * (qx * qz - qw * qy));
    if (i == 1) return vec3f::make(2.0f * (qx * qy - qw * qz), 1.0f - 2.0f * (qx * qx + qz * qz), 2.0f * (qy * qz + qw * qx));
    return vec3f::make(2.0f * (qx * qz + qw * qy), 2.0f * (qy * qz - qw * qx), 1.0f - 2.0f * (qx * qx + qy * qy));
}

void get_sleep_test_points(Vec3f out_points[3], const RigidBody *body) {
    // center of mass is the first position
    const Vec3f com = body->center_of_mass_position;
    out_points[0] = com;

    // half-sizes of shape AABB
    const AABox &shape_aabb = body->shape->aabb;
    const float extents[3] = {(vec3f::x(shape_aabb.max) - vec3f::x(shape_aabb.min)) * 0.5f, (vec3f::y(shape_aabb.max) - vec3f::y(shape_aabb.min)) * 0.5f,
                              (vec3f::z(shape_aabb.max) - vec3f::z(shape_aabb.min)) * 0.5f};

    // the two largest extents pick the two rotated axes to test, along the rotation's columns. the smallest extent's
    // index, ties going to the later axis but z only when strictly smallest (a cube tests x and z)
    const float ex = extents[0], ey = extents[1], ez = extents[2];
    const int lowest = ex < ey ? (ez < ex ? 2 : 0) : (ez < ey ? 2 : 1);
    const int first = lowest == 0 ? 1 : 0;
    const int second = lowest == 2 ? 1 : 2;
    out_points[1] = vec3f::scale_and_add(com, rotation_axis(body->quaternion, first), extents[first]);
    out_points[2] = vec3f::scale_and_add(com, rotation_axis(body->quaternion, second), extents[second]);
}

void reset_sleep_test_spheres(MotionProperties &motion_properties, const Vec3f points[3]) {
    for (int i = 0; i < 3; i++) {
        vec3f::store(motion_properties.sleep_test_spheres[i].center, points[i]);
        motion_properties.sleep_test_spheres[i].radius = 0.0f;
    }
    motion_properties.sleep_test_timer = 0.0f;
}

/**
 * grows the sphere just enough to hold the point: its diameter becomes the old radius plus the point's distance, and the
 * centre moves toward the point, so a body drifting one way is measured by half its travel
 */
static void encapsulate_point(Spheref &sphere, Vec3f point) {
    const Vec3f center = vec3f::load(sphere.center);
    const Vec3f to_point = vec3f::subtract(point, center);
    const float distance_sq = vec3f::squared_length(to_point);
    if (distance_sq > sphere.radius * sphere.radius) {
        const float distance = scalar::sqrt(distance_sq);
        const float radius = 0.5f * (sphere.radius + distance);
        vec3f::store(sphere.center, vec3f::scale_and_add(center, to_point, (radius - sphere.radius) / distance));
        sphere.radius = radius;
    }
}

bool update_sleep_state(RigidBody *body, float delta_time, float max_movement, float time_before_sleep) {
    MotionProperties &mp = body->motion_properties;

    // sensors and bodies with allow_sleeping=false never sleep
    if (!mp.allow_sleeping || body->sensor) {
        return false;  // cannot sleep
    }

    // get current test points
    Vec3f points[3];
    get_sleep_test_points(points, body);

    // check if any sphere exceeds max movement
    for (int i = 0; i < 3; i++) {
        Spheref &sphere = mp.sleep_test_spheres[i];

        encapsulate_point(sphere, points[i]);

        // if exceeded threshold, reset and return cannot sleep
        if (sphere.radius > max_movement) {
            reset_sleep_test_spheres(mp, points);
            return false;
        }
    }

    // accumulate sleep time
    mp.sleep_test_timer += delta_time;
    return mp.sleep_test_timer >= time_before_sleep;  // can sleep
}

void reset_sleep_timer(RigidBody *body) {
    if (body->motion_type != MotionType::DYNAMIC) {
        return;
    }

    Vec3f points[3];
    get_sleep_test_points(points, body);
    reset_sleep_test_spheres(body->motion_properties, points);
}

static void add_body_to_active_bodies(Bodies &bodies, RigidBody *body) {
    body->active_index = bodies.active_body_count;
    bodies.active_body_indices[bodies.active_body_count] = body->index;
    bodies.active_body_count++;
}

static void remove_body_from_active_bodies(Bodies &bodies, RigidBody *body) {
    const uint32_t last_index = bodies.active_body_count - 1;

    if (body->active_index != last_index) {
        // swap with last body to fill the hole
        const uint32_t last_body_index = bodies.active_body_indices[last_index];
        bodies.active_body_indices[body->active_index] = last_body_index;

        // update swapped body's active_index
        RigidBody *last_body = bodies.pool[last_body_index];
        last_body->active_index = body->active_index;
    }

    // islands are only built over active bodies, so the island index goes stale here; clearing it on exit keeps
    // islands::prepare from resetting the whole body pool every step
    body->active_index = INACTIVE_BODY_INDEX;
    body->island_index = -1;
    bodies.active_body_count--;
}

void wake(Bodies &bodies, RigidBody *body) {
    if (body->motion_type == MotionType::STATIC) return;

    // so it does not go straight back to sleep
    reset_sleep_timer(body);

    if (body->active_index != INACTIVE_BODY_INDEX) return;
    add_body_to_active_bodies(bodies, body);

    BodyActivationListener *listener = bodies.activation_listener;
    if (listener != nullptr && listener->on_body_activated != nullptr) listener->on_body_activated(*listener, body->id, body->user_data);
}

void sleep(Bodies &bodies, RigidBody *body) {
    if (body->active_index == INACTIVE_BODY_INDEX) return;
    remove_body_from_active_bodies(bodies, body);

    body->motion_properties.linear_velocity = vec3f::zero();
    body->motion_properties.angular_velocity = vec3f::zero();

    BodyActivationListener *listener = bodies.activation_listener;
    if (listener != nullptr && listener->on_body_deactivated != nullptr) listener->on_body_deactivated(*listener, body->id, body->user_data);
}

}  // namespace sleep

TUMBLE_NAMESPACE_END
