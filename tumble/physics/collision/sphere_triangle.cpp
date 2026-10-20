#include "physics/collision/sphere_triangle.h"

#include "math/scalar.h"
#include "math/vec3f.h"

namespace sphere_triangle {

float ray_cylinder(Vec3f direction, Vec3f cylinder_a, Vec3f cylinder_b, float radius) {
    // calculate cylinder axis
    const Vec3f axis = vec3f::subtract(cylinder_b, cylinder_a);

    // make ray start relative to cylinder side A (moving cylinder A to the origin)
    // since ray origin is at (0,0,0), start = -cylinder_a
    Vec3f start = vec3f::negate(cylinder_a);

    // test if segment is fully on the A side of the cylinder
    const float start_dot_axis = vec3f::dot(start, axis);
    const float direction_dot_axis = vec3f::dot(direction, axis);
    const float end_dot_axis = start_dot_axis + direction_dot_axis;

    if (start_dot_axis < 0.0f && end_dot_axis < 0.0f) return scalar::INF;

    // test if segment is fully on the B side of the cylinder
    const float axis_len_sq = vec3f::squared_length(axis);
    if (start_dot_axis > axis_len_sq && end_dot_axis > axis_len_sq) return scalar::INF;

    // calculate a, b and c, the factors for quadratic equation
    // we're solving: |ray(t) - closest_point_on_axis|^2 = radius^2
    const float dir_len_sq = vec3f::squared_length(direction);
    const float a = axis_len_sq * dir_len_sq - direction_dot_axis * direction_dot_axis;

    if (scalar::abs(a) < 1e-6f) {
        // segment runs parallel to cylinder axis
        return scalar::INF;
    }

    const float start_dot_dir = vec3f::dot(start, direction);
    const float start_len_sq = vec3f::squared_length(start);

    // note: b should be multiplied by 2, but we'll divide a and c by 2 when solving
    const float b = axis_len_sq * start_dot_dir - direction_dot_axis * start_dot_axis;
    const float c = axis_len_sq * (start_len_sq - radius * radius) - start_dot_axis * start_dot_axis;

    // discriminant (normally 4*a*c but since both a and c are divided by 2, we lose the 4)
    const float det = b * b - a * c;
    if (det < 0.0f) {
        // no solution to quadratic equation
        return scalar::INF;
    }

    // solve fraction t where the ray hits the cylinder
    // normally divided by 2*a but since a should be divided by 2, we lose the 2
    const float t = -(b + scalar::sqrt(det)) / a;

    if (t < 0.0f || t > 1.0f) {
        // intersection lies outside segment [0, 1]
        return scalar::INF;
    }

    // check if intersection is within cylinder segment (not past the caps)
    const float hit_dot_axis = start_dot_axis + t * direction_dot_axis;
    if (hit_dot_axis < 0.0f || hit_dot_axis > axis_len_sq) {
        // intersection outside the cylinder segment
        return scalar::INF;
    }

    return t;
}

/** the smaller root of a t^2 + b t + c in [0, 1], 0 when the ray starts inside, infinity otherwise. */
static float solve_ray_sphere(float a, float b, float c) {
    const float discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f) {
        // no intersection - but check if we're inside the sphere
        return c <= 0.0f ? 0.0f : scalar::INF;
    }

    if (a < 1e-10f) {
        // degenerate case: direction is zero-length
        return c <= 0.0f ? 0.0f : scalar::INF;
    }

    const float sqrt_d = scalar::sqrt(discriminant);
    float t1 = (-b - sqrt_d) / (2.0f * a);
    float t2 = (-b + sqrt_d) / (2.0f * a);

    // sort so t1 <= t2
    if (t1 > t2) {
        const float tmp = t1;
        t1 = t2;
        t2 = tmp;
    }

    // test solution with lowest fraction (ray entering sphere)
    if (t1 >= 0.0f && t1 <= 1.0f) return t1;

    // test solution with highest fraction (ray leaving sphere)
    if (t2 >= 0.0f && t2 <= 1.0f) {
        // we start inside the sphere
        return 0.0f;
    }

    // check if entire ray segment is inside sphere
    if (t1 < 0.0f && t2 > 1.0f) return 0.0f;

    return scalar::INF;
}

float ray_sphere(Vec3f origin, Vec3f direction, Vec3f sphere_center, float sphere_radius) {
    // solve: |origin + t * direction - center|^2 = radius^2
    const Vec3f center_origin = vec3f::subtract(origin, sphere_center);

    const float a = vec3f::squared_length(direction);
    const float b = 2.0f * vec3f::dot(direction, center_origin);
    const float c = vec3f::squared_length(center_origin) - sphere_radius * sphere_radius;
    return solve_ray_sphere(a, b, c);
}

float ray_sphere_from_origin(Vec3f direction, Vec3f sphere_center, float sphere_radius) {
    // solve: |t * direction - center|^2 = radius^2
    const float a = vec3f::squared_length(direction);
    const float b = -2.0f * vec3f::dot(direction, sphere_center);
    const float c = vec3f::squared_length(sphere_center) - sphere_radius * sphere_radius;
    return solve_ray_sphere(a, b, c);
}

}  // namespace sphere_triangle
