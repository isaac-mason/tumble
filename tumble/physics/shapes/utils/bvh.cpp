#include "tumble/physics/shapes/utils/bvh.h"

TUMBLE_NAMESPACE_BEGIN

namespace bvh {

static inline bool is_nan(float x) { return x != x; }

bool node_intersects_ray(const BvhNode *nodes, int32_t node, Vec3f origin, Vec3f direction, float near, float far) {
    float tmin, tmax, tymin, tymax, tzmin, tzmax;

    const float invdirx = 1.0f / vec3f::x(direction);
    const float invdiry = 1.0f / vec3f::y(direction);
    const float invdirz = 1.0f / vec3f::z(direction);

    const Box3f &bounds = nodes[node].bounds;

    if (invdirx >= 0.0f) {
        tmin = (bounds.min.x - vec3f::x(origin)) * invdirx;
        tmax = (bounds.max.x - vec3f::x(origin)) * invdirx;
    } else {
        tmin = (bounds.max.x - vec3f::x(origin)) * invdirx;
        tmax = (bounds.min.x - vec3f::x(origin)) * invdirx;
    }

    if (invdiry >= 0.0f) {
        tymin = (bounds.min.y - vec3f::y(origin)) * invdiry;
        tymax = (bounds.max.y - vec3f::y(origin)) * invdiry;
    } else {
        tymin = (bounds.max.y - vec3f::y(origin)) * invdiry;
        tymax = (bounds.min.y - vec3f::y(origin)) * invdiry;
    }

    if (tmin > tymax || tymin > tmax) return false;

    if (tymin > tmin || is_nan(tmin)) tmin = tymin;
    if (tymax < tmax || is_nan(tmax)) tmax = tymax;

    if (invdirz >= 0.0f) {
        tzmin = (bounds.min.z - vec3f::z(origin)) * invdirz;
        tzmax = (bounds.max.z - vec3f::z(origin)) * invdirz;
    } else {
        tzmin = (bounds.max.z - vec3f::z(origin)) * invdirz;
        tzmax = (bounds.min.z - vec3f::z(origin)) * invdirz;
    }

    if (tmin > tzmax || tzmin > tmax) return false;

    if (tzmin > tmin || is_nan(tmin)) tmin = tzmin;
    if (tzmax < tmax || is_nan(tmax)) tmax = tzmax;

    return tmin <= far && tmax >= near;
}

}  // namespace bvh

TUMBLE_NAMESPACE_END
