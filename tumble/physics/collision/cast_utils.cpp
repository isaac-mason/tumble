#include "tumble/physics/collision/cast_utils.h"

#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace cast_utils {

static constexpr float MISS = scalar::FLOAT_MAX;

static void no_hit(RayIntersectsTriangleResult &out) {
    out.hit = false;
    out.fraction = 0.0f;
    out.front_facing = false;
}

void ray_intersects_triangle(RayIntersectsTriangleResult &out, const cast_ray::Rayf &ray, float max_t, Vec3f a, Vec3f b,
                             Vec3f c, bool backface_culling) {
    const Vec3f origin = ray.origin;
    const Vec3f direction = ray.direction;

    const Vec3f edge1 = vec3f::subtract(b, a);
    const Vec3f edge2 = vec3f::subtract(c, a);
    const Vec3f normal = vec3f::cross(edge1, edge2);

    // determine front vs back facing
    float d_dot_n = vec3f::dot(direction, normal);
    float sign;
    if (d_dot_n > 0.0f) {
        // backface
        if (backface_culling) {
            no_hit(out);
            return;
        }
        sign = 1.0f;
    } else if (d_dot_n < 0.0f) {
        // frontface
        sign = -1.0f;
        d_dot_n = -d_dot_n;
    } else {
        // ray is parallel to triangle
        no_hit(out);
        return;
    }

    const Vec3f diff = vec3f::subtract(origin, a);

    // barycentric coordinate b1: sign * D . (diff x edge2)
    const float d_dot_q_cross_e2 = sign * vec3f::dot(direction, vec3f::cross(diff, edge2));
    if (d_dot_q_cross_e2 < 0.0f) {
        no_hit(out);
        return;
    }

    // barycentric coordinate b2: sign * D . (edge1 x diff)
    const float d_dot_e1_cross_q = sign * vec3f::dot(direction, vec3f::cross(edge1, diff));
    if (d_dot_e1_cross_q < 0.0f) {
        no_hit(out);
        return;
    }

    // b1 + b2 must not exceed 1
    if (d_dot_q_cross_e2 + d_dot_e1_cross_q > d_dot_n) {
        no_hit(out);
        return;
    }

    // intersection distance along the ray
    const float q_dot_n = -sign * vec3f::dot(diff, normal);
    if (q_dot_n < 0.0f) {
        no_hit(out);
        return;
    }

    // t = q_dot_n / d_dot_n; reject against the caller's bound before dividing (d_dot_n > 0 here)
    if (q_dot_n > d_dot_n * max_t) {
        no_hit(out);
        return;
    }

    out.hit = true;
    out.fraction = q_dot_n / d_dot_n / ray.length;
    out.front_facing = sign < 0.0f;
}

float safe_reciprocal(float d) { return d > 1e-30f || d < -1e-30f ? 1.0f / d : 0.0f; }

float ray_fraction_to_box3(Vec3f origin, Vec3f inv_displacement, const AABox &box) {
    float t_min = 0.0f;
    float t_max = 1.0f;

    const float origins[3] = {vec3f::x(origin), vec3f::y(origin), vec3f::z(origin)};
    const float invs[3] = {vec3f::x(inv_displacement), vec3f::y(inv_displacement), vec3f::z(inv_displacement)};
    const float mins[3] = {vec3f::x(box.min), vec3f::y(box.min), vec3f::z(box.min)};
    const float maxs[3] = {vec3f::x(box.max), vec3f::y(box.max), vec3f::z(box.max)};

    for (int axis = 0; axis < 3; axis++) {
        if (invs[axis] == 0.0f) {
            // parallel to the slab: the segment stays at the origin there, which must lie inside it
            if (origins[axis] < mins[axis] || origins[axis] > maxs[axis]) return MISS;
        } else {
            const float t0 = (mins[axis] - origins[axis]) * invs[axis];
            const float t1 = (maxs[axis] - origins[axis]) * invs[axis];
            if (t0 < t1) {
                if (t0 > t_min) t_min = t0;
                if (t1 < t_max) t_max = t1;
            } else {
                if (t1 > t_min) t_min = t1;
                if (t0 < t_max) t_max = t0;
            }
            if (t_max < t_min) return MISS;
        }
    }

    return t_min;
}

float ray_distance_to_box3(const cast_ray::Rayf &ray, const AABox &box) {
    const float length = ray.length;
    float t_min = 0.0f;
    float t_max = length;

    const float origins[3] = {vec3f::x(ray.origin), vec3f::y(ray.origin), vec3f::z(ray.origin)};
    const float directions[3] = {vec3f::x(ray.direction), vec3f::y(ray.direction), vec3f::z(ray.direction)};
    const float mins[3] = {vec3f::x(box.min), vec3f::y(box.min), vec3f::z(box.min)};
    const float maxs[3] = {vec3f::x(box.max), vec3f::y(box.max), vec3f::z(box.max)};

    for (int axis = 0; axis < 3; axis++) {
        if (scalar::abs(directions[axis]) < 1e-10f) {
            // ray is parallel to slab - check if origin is within slab
            if (origins[axis] < mins[axis] || origins[axis] > maxs[axis]) return MISS;
        } else {
            const float inv_d = 1.0f / directions[axis];
            const float t0 = (mins[axis] - origins[axis]) * inv_d;
            const float t1 = (maxs[axis] - origins[axis]) * inv_d;

            const float t_near = t0 < t1 ? t0 : t1;
            const float t_far = t0 < t1 ? t1 : t0;

            t_min = t_near > t_min ? t_near : t_min;
            t_max = t_far < t_max ? t_far : t_max;

            if (t_max < t_min) return MISS;
        }
    }

    // a zero-length ray reaching here starts inside the box, the overlap limit of a cast
    if (length == 0.0f) return 0.0f;

    return t_min >= 0.0f ? t_min / length : MISS;
}

bool ray_hits_box3(const cast_ray::Rayf &ray, const AABox &box) {
    float t_near = 0.0f;
    float t_far = ray.length;

    const float origins[3] = {vec3f::x(ray.origin), vec3f::y(ray.origin), vec3f::z(ray.origin)};
    const float directions[3] = {vec3f::x(ray.direction), vec3f::y(ray.direction), vec3f::z(ray.direction)};
    const float mins[3] = {vec3f::x(box.min), vec3f::y(box.min), vec3f::z(box.min)};
    const float maxs[3] = {vec3f::x(box.max), vec3f::y(box.max), vec3f::z(box.max)};

    for (int axis = 0; axis < 3; axis++) {
        if (scalar::abs(directions[axis]) < 1e-10f) {
            if (origins[axis] < mins[axis] || origins[axis] > maxs[axis]) return false;
        } else {
            const float inv = 1.0f / directions[axis];
            float t_enter = (mins[axis] - origins[axis]) * inv;
            float t_exit = (maxs[axis] - origins[axis]) * inv;
            if (inv < 0.0f) {
                const float tmp = t_enter;
                t_enter = t_exit;
                t_exit = tmp;
            }
            if (t_enter > t_near) t_near = t_enter;
            if (t_exit < t_far) t_far = t_exit;
            if (t_far < t_near) return false;
        }
    }

    return true;
}

}  // namespace cast_utils

TUMBLE_NAMESPACE_END
