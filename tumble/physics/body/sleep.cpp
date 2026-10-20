#include "tumble/physics/body/sleep.h"

#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"

TUMBLE_NAMESPACE_BEGIN

namespace sleep {

void get_sleep_test_points(Float3 out_points[3], const rigid_body::RigidBody *body) {
    const Float3 &com = body->center_of_mass_position;

    // center of mass is the first position
    out_points[0] = com;

    // half-sizes of shape AABB
    const Box3f &shape_aabb = body->shape->aabb;
    const float ex = (shape_aabb.max.x - shape_aabb.min.x) * 0.5f;
    const float ey = (shape_aabb.max.y - shape_aabb.min.y) * 0.5f;
    const float ez = (shape_aabb.max.z - shape_aabb.min.z) * 0.5f;

    // the two largest extents pick the two rotated axes to test; a rotation matrix column comes
    // straight from the quaternion, so only those two are built
    const float qx = body->quaternion.x;
    const float qy = body->quaternion.y;
    const float qz = body->quaternion.z;
    const float qw = body->quaternion.w;
    Float3 &p1 = out_points[1];
    Float3 &p2 = out_points[2];
    // the smallest extent's index, ties going to the later axis but z only when strictly smallest (a cube tests x and z)
    const int lowest = ex < ey ? (ez < ex ? 2 : 0) : (ez < ey ? 2 : 1);
    if (lowest == 0) {
        // x is smallest: y and z axes
        p1.x = com.x + 2.0f * (qx * qy - qw * qz) * ey;
        p1.y = com.y + (1.0f - 2.0f * (qx * qx + qz * qz)) * ey;
        p1.z = com.z + 2.0f * (qy * qz + qw * qx) * ey;
        p2.x = com.x + 2.0f * (qx * qz + qw * qy) * ez;
        p2.y = com.y + 2.0f * (qy * qz - qw * qx) * ez;
        p2.z = com.z + (1.0f - 2.0f * (qx * qx + qy * qy)) * ez;
    } else if (lowest == 1) {
        // y is smallest: x and z axes
        p1.x = com.x + (1.0f - 2.0f * (qy * qy + qz * qz)) * ex;
        p1.y = com.y + 2.0f * (qx * qy + qw * qz) * ex;
        p1.z = com.z + 2.0f * (qx * qz - qw * qy) * ex;
        p2.x = com.x + 2.0f * (qx * qz + qw * qy) * ez;
        p2.y = com.y + 2.0f * (qy * qz - qw * qx) * ez;
        p2.z = com.z + (1.0f - 2.0f * (qx * qx + qy * qy)) * ez;
    } else {
        // z is smallest: x and y axes
        p1.x = com.x + (1.0f - 2.0f * (qy * qy + qz * qz)) * ex;
        p1.y = com.y + 2.0f * (qx * qy + qw * qz) * ex;
        p1.z = com.z + 2.0f * (qx * qz - qw * qy) * ex;
        p2.x = com.x + 2.0f * (qx * qy - qw * qz) * ey;
        p2.y = com.y + (1.0f - 2.0f * (qx * qx + qz * qz)) * ey;
        p2.z = com.z + 2.0f * (qy * qz + qw * qx) * ey;
    }
}

void reset_sleep_test_spheres(motion_properties::MotionProperties &motion_properties, const Float3 points[3]) {
    for (int i = 0; i < 3; i++) {
        motion_properties.sleep_test_spheres[i].center = points[i];
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

bool update_sleep_state(rigid_body::RigidBody *body, float delta_time, float max_movement, float time_before_sleep) {
    motion_properties::MotionProperties &mp = body->motion_properties;

    // sensors and bodies with allow_sleeping=false never sleep
    if (!mp.allow_sleeping || body->sensor) {
        return false;  // cannot sleep
    }

    // get current test points
    Float3 points[3];
    get_sleep_test_points(points, body);

    // check if any sphere exceeds max movement
    for (int i = 0; i < 3; i++) {
        Spheref &sphere = mp.sleep_test_spheres[i];

        encapsulate_point(sphere, vec3f::load(points[i]));

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

void reset_sleep_timer(rigid_body::RigidBody *body) {
    if (body->motion_type != motion_type::MOTION_DYNAMIC) {
        return;
    }

    Float3 points[3];
    get_sleep_test_points(points, body);
    reset_sleep_test_spheres(body->motion_properties, points);
}

void add_body_to_active_bodies(bodies::Bodies &bodies, rigid_body::RigidBody *body) {
    if (body->active_index != INACTIVE_BODY_INDEX) return;

    // assign new index at end of array
    body->active_index = bodies.active_body_count;
    bodies.active_body_indices[bodies.active_body_count] = body->index;
    bodies.active_body_count++;
}

void remove_body_from_active_bodies(bodies::Bodies &bodies, rigid_body::RigidBody *body) {
    if (body->active_index == INACTIVE_BODY_INDEX) return;

    const uint32_t last_index = bodies.active_body_count - 1;

    if (body->active_index != last_index) {
        // swap with last body to fill the hole
        const uint32_t last_body_index = bodies.active_body_indices[last_index];
        bodies.active_body_indices[body->active_index] = last_body_index;

        // update swapped body's active_index
        rigid_body::RigidBody *last_body = bodies.pool[last_body_index];
        last_body->active_index = body->active_index;
    }

    // mark as inactive. islands are only built over active bodies, so the island index goes stale
    // here; clearing it on exit keeps islands::prepare from resetting the whole body pool every step
    body->active_index = INACTIVE_BODY_INDEX;
    body->island_index = -1;
    bodies.active_body_count--;
}

void sleep(bodies::Bodies &bodies, rigid_body::RigidBody *body) {
    if (body->motion_type == motion_type::MOTION_STATIC) return;

    // exit if already sleeping
    if (body->sleeping) return;

    // remove from active list
    remove_body_from_active_bodies(bodies, body);

    body->sleeping = true;

    body->motion_properties.linear_velocity = {0.0f, 0.0f, 0.0f};
    body->motion_properties.angular_velocity = {0.0f, 0.0f, 0.0f};
}

void wake(bodies::Bodies &bodies, rigid_body::RigidBody *body) {
    if (body->motion_type == motion_type::MOTION_STATIC) return;

    // always reset sleep timer
    reset_sleep_timer(body);

    if (!body->sleeping) return;

    body->sleeping = false;

    add_body_to_active_bodies(bodies, body);
}

}  // namespace sleep

TUMBLE_NAMESPACE_END
