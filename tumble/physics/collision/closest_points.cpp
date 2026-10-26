#include "tumble/physics/collision/closest_points.h"

#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace closest_points {

void compute_barycentric_coordinates_2d(BarycentricCoordinatesResult &out, Vec3f a, Vec3f b, float squared_tolerance) {
    const Vec3f ab = vec3f::subtract(b, a);

    const float denominator = vec3f::dot(ab, ab);

    if (denominator < squared_tolerance) {
        // degenerate line segment, fallback to points
        if (vec3f::dot(a, a) < vec3f::dot(b, b)) {
            // a closest
            out.u = 1.0f;
            out.v = 0.0f;
        } else {
            // b closest
            out.u = 0.0f;
            out.v = 1.0f;
        }
        out.is_valid = false;
        return;
    }

    out.v = -vec3f::dot(a, ab) / denominator;
    out.u = 1.0f - out.v;
    out.is_valid = true;
}

void compute_barycentric_coordinates_3d(BarycentricCoordinatesResult &out, Vec3f a, Vec3f b, Vec3f c,
                                        float squared_tolerance) {
    const Vec3f ab = vec3f::subtract(b, a);
    const Vec3f ac = vec3f::subtract(c, a);
    const Vec3f bc = vec3f::subtract(c, b);

    const float d00 = vec3f::dot(ab, ab);
    const float d11 = vec3f::dot(ac, ac);
    const float d22 = vec3f::dot(bc, bc);

    BarycentricCoordinatesResult other;

    if (d00 <= d22) {
        // use v0 and v1 to calculate barycentric coordinates
        const float d01 = vec3f::dot(ab, ac);

        // |v0|^2 |v1|^2 - (v0 . v1)^2 = |v0|^2 |v1|^2 (1 - cos(angle)^2): below 1e-6 of |v0|^2 |v1|^2 the angle is about
        // 0.06 degrees, so the triangle is degenerate and we fall back to the closest point on the longest edge
        const float denominator = d00 * d11 - d01 * d01;
        if (denominator <= 1.0e-6f * d00 * d11) {
            // degenerate triangle, return coordinates along longest edge
            if (d00 > d11) {
                compute_barycentric_coordinates_2d(other, a, b, squared_tolerance);
                out.u = other.u;
                out.v = other.v;
                out.w = 0.0f;
            } else {
                compute_barycentric_coordinates_2d(other, a, c, squared_tolerance);
                out.u = other.u;
                out.w = other.v;
                out.v = 0.0f;
            }
            out.is_valid = false;
            return;
        }

        const float a0 = vec3f::dot(a, ab);
        const float a1 = vec3f::dot(a, ac);

        out.v = (d01 * a1 - d11 * a0) / denominator;
        out.w = (d01 * a0 - d00 * a1) / denominator;
        out.u = 1.0f - out.v - out.w;
    } else {
        // use v1 and v2 to calculate barycentric coordinates
        const float d12 = vec3f::dot(ac, bc);

        const float denominator = d11 * d22 - d12 * d12;
        if (denominator <= 1.0e-6f * d11 * d22) {
            // degenerate triangle, return coordinates along longest edge
            if (d11 > d22) {
                compute_barycentric_coordinates_2d(other, a, c, squared_tolerance);
                out.u = other.u;
                out.w = other.v;
                out.v = 0.0f;
            } else {
                compute_barycentric_coordinates_2d(other, b, c, squared_tolerance);
                out.v = other.u;
                out.w = other.v;
                out.u = 0.0f;
            }
            out.is_valid = false;
            return;
        }

        const float c1 = vec3f::dot(c, ac);
        const float c2 = vec3f::dot(c, bc);

        out.u = (d22 * c1 - d12 * c2) / denominator;
        out.v = (d11 * c2 - d12 * c1) / denominator;
        out.w = 1.0f - out.u - out.v;
    }
    out.is_valid = true;
}

}  // namespace closest_points

TUMBLE_NAMESPACE_END
