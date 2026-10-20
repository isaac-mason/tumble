#include "tumble/physics/collision/active_edges.h"

#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/closest_points.h"

TUMBLE_NAMESPACE_BEGIN

namespace active_edges {

bool is_edge_active(Vec3f normal1, Vec3f normal2, Vec3f edge_direction, float cos_threshold_angle) {
    // if normals are opposite the edges are active (the triangles are back to back)
    const float cos_angle_normals = vec3f::dot(normal1, normal2);
    if (cos_angle_normals < COS_179_DEGREES) return true;

    // check if concave edge, if so we are not active
    const Vec3f cross = vec3f::cross(normal1, normal2);
    if (vec3f::dot(cross, edge_direction) < 0.0f) return false;

    // convex edge, active when angle bigger than threshold
    return cos_angle_normals < cos_threshold_angle;
}

Vec3f fix_normal(Vec3f v0, Vec3f v1, Vec3f v2, Vec3f triangle_normal, uint32_t active_edges,
                  Vec3f point, Vec3f normal, Vec3f movement_direction) {
    // all of the edges are active, we have the correct normal already
    if (active_edges == 0b111) return normal;

    // if normal would affect movement less than triangle_normal use normal.
    // this is done since it is really hard to make a distinction between sliding over a horizontal
    // triangulated grid and hitting an edge (in this case you want to use the triangle normal)
    // and sliding over a triangulated grid and grazing a vertical triangle with an inactive edge
    // (in this case using the triangle normal will cause the object to bounce back so we want to use
    // the calculated normal).
    // to solve this we take a movement hint to give an indication of what direction our object is moving.
    // if the edge normal results in less motion difference than the triangle normal we use the edge normal.
    const float normal_length = vec3f::length(normal);
    const float triangle_normal_length = vec3f::length(triangle_normal);
    if (vec3f::dot(movement_direction, normal) * triangle_normal_length <
        vec3f::dot(movement_direction, triangle_normal) * normal_length) {
        return normal;
    }

    // none of the edges are active, we need to use the triangle normal
    if (active_edges == 0) return triangle_normal;

    // some edges are active.
    // if normal is parallel to the triangle normal we don't need to check the active edges.
    if (vec3f::dot(triangle_normal, normal) > COS_1_DEGREE * normal_length * triangle_normal_length) return normal;

    // test where the contact point is in the triangle: vertices relative to the collision point
    const Vec3f v0_shifted = vec3f::subtract(v0, point);
    const Vec3f v1_shifted = vec3f::subtract(v1, point);
    const Vec3f v2_shifted = vec3f::subtract(v2, point);

    closest_points::BarycentricCoordinatesResult barycentric;
    closest_points::compute_barycentric_coordinates_3d(barycentric, v0_shifted, v1_shifted, v2_shifted, 1e-10f);
    const float u = barycentric.u;
    const float v = barycentric.v;
    const float w = barycentric.w;

    // determine which edge(s) the collision point is near
    uint32_t colliding_edge;
    if (u > BARYCENTRIC_ONE_MINUS_EPSILON) {
        // colliding with v0, edge 0 or 2 needs to be active
        colliding_edge = 0b101;
    } else if (v > BARYCENTRIC_ONE_MINUS_EPSILON) {
        // colliding with v1, edge 0 or 1 needs to be active
        colliding_edge = 0b011;
    } else if (w > BARYCENTRIC_ONE_MINUS_EPSILON) {
        // colliding with v2, edge 1 or 2 needs to be active
        colliding_edge = 0b110;
    } else if (u < BARYCENTRIC_EPSILON) {
        // colliding with edge v1, v2, edge 1 needs to be active
        colliding_edge = 0b010;
    } else if (v < BARYCENTRIC_EPSILON) {
        // colliding with edge v0, v2, edge 2 needs to be active
        colliding_edge = 0b100;
    } else if (w < BARYCENTRIC_EPSILON) {
        // colliding with edge v0, v1, edge 0 needs to be active
        colliding_edge = 0b001;
    } else {
        // interior hit
        return triangle_normal;
    }

    // if this edge is active, use the provided normal instead of the triangle normal
    return (active_edges & colliding_edge) != 0 ? normal : triangle_normal;
}

}  // namespace active_edges

TUMBLE_NAMESPACE_END
