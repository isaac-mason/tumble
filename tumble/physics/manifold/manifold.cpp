#include "tumble/physics/manifold/manifold.h"

#include "tumble/math/scalar.h"
#include "tumble/physics/manifold/clip.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace manifold {

void swap_shapes(ContactManifold &m) {
    m.world_space_normal = vec3f::negate(m.world_space_normal);

    const sub_shape::SubShapeId sub_shape_id = m.sub_shape_id_a;
    m.sub_shape_id_a = m.sub_shape_id_b;
    m.sub_shape_id_b = sub_shape_id;

    const int32_t material_id = m.material_id_a;
    m.material_id_a = m.material_id_b;
    m.material_id_b = material_id;

    for (uint32_t i = 0; i < m.num_contact_points; i++) {
        const Vec3f point = m.relative_contact_points_on_a[i];
        m.relative_contact_points_on_a[i] = m.relative_contact_points_on_b[i];
        m.relative_contact_points_on_b[i] = point;
    }
}

// 1 mm^2
static constexpr float MIN_DISTANCE_SQ = 1.0e-6f;

void prune_contact_points(ContactManifold &m, Vec3f penetration_axis) {
    TUMBLE_ASSERT(m.num_contact_points > MAX_CONTACT_POINTS);
    if (m.num_contact_points <= MAX_CONTACT_POINTS) return;

    TUMBLE_ASSERT(scalar::abs(vec3f::squared_length(penetration_axis) - 1.0f) <= 1.0e-6f);

    const uint32_t num_points = m.num_contact_points;
    Vec3f projected[MAX_CLIPPING_CONTACT_POINTS];
    float penetration_depth_sq[MAX_CLIPPING_CONTACT_POINTS];

    // step 1: project all contact points onto plane perpendicular to penetration_axis
    for (uint32_t i = 0; i < num_points; i++) {
        const Vec3f v1 = m.relative_contact_points_on_a[i];
        const Vec3f v2 = m.relative_contact_points_on_b[i];

        // projected = v1 - (v1 . axis) * axis
        projected[i] = vec3f::scale_and_add(v1, penetration_axis, -vec3f::dot(v1, penetration_axis));

        // penetration depth = distance between points on shapes
        penetration_depth_sq[i] = scalar::max(MIN_DISTANCE_SQ, vec3f::squared_distance(v2, v1));
    }

    // step 2: select point 1 (maximum metric in plane)
    uint32_t point1 = 0;
    float max_metric = scalar::max(MIN_DISTANCE_SQ, vec3f::squared_length(projected[0])) * penetration_depth_sq[0];
    for (uint32_t i = 1; i < num_points; i++) {
        const float metric = scalar::max(MIN_DISTANCE_SQ, vec3f::squared_length(projected[i])) * penetration_depth_sq[i];
        if (metric > max_metric) {
            max_metric = metric;
            point1 = i;
        }
    }
    const Vec3f point1_projected = projected[point1];

    // step 3: select point 2 (furthest from point 1)
    int32_t point2 = -1;
    max_metric = -scalar::INF;
    for (uint32_t i = 0; i < num_points; i++) {
        if (i == point1) continue;
        const float metric = scalar::max(MIN_DISTANCE_SQ, vec3f::squared_distance(projected[i], point1_projected)) * penetration_depth_sq[i];
        if (metric > max_metric) {
            max_metric = metric;
            point2 = (int32_t)i;
        }
    }

    TUMBLE_ASSERT(point2 != -1);

    // step 4: select points 3 & 4 (opposite sides of the p1-p2 edge)
    int32_t point3 = -1;
    int32_t point4 = -1;
    // furthest in negative and positive perpendicular direction
    float min_dot = 0.0f;
    float max_dot = 0.0f;

    // perp = (p2 - p1) x axis
    const Vec3f perpendicular = vec3f::cross(vec3f::subtract(projected[point2], point1_projected), penetration_axis);

    for (uint32_t i = 0; i < num_points; i++) {
        if (i == point1 || (int32_t)i == point2) continue;
        const float d = vec3f::dot(vec3f::subtract(projected[i], point1_projected), perpendicular);
        if (d < min_dot) {
            min_dot = d;
            point3 = (int32_t)i;
        } else if (d > max_dot) {
            max_dot = d;
            point4 = (int32_t)i;
        }
    }

    // step 5: gather the kept points in winding order [p1, p3, p2, p4] so they form a polygon, then copy them back
    Vec3f kept_on_a[MAX_CONTACT_POINTS];
    Vec3f kept_on_b[MAX_CONTACT_POINTS];
    uint32_t num_kept = 0;
    const auto keep = [&](uint32_t index) {
        kept_on_a[num_kept] = m.relative_contact_points_on_a[index];
        kept_on_b[num_kept] = m.relative_contact_points_on_b[index];
        num_kept++;
    };
    keep(point1);
    if (point3 != -1) keep((uint32_t)point3);
    keep((uint32_t)point2);
    if (point4 != -1) {
        TUMBLE_ASSERT(point3 != point4);
        keep((uint32_t)point4);
    }

    for (uint32_t i = 0; i < num_kept; i++) {
        m.relative_contact_points_on_a[i] = kept_on_a[i];
        m.relative_contact_points_on_b[i] = kept_on_b[i];
    }
    m.num_contact_points = num_kept;
}

static void set_single_point(ContactManifold &out, Vec3f contact_point_1, Vec3f contact_point_2) {
    out.relative_contact_points_on_a[0] = vec3f::subtract(contact_point_1, out.base_offset);
    out.relative_contact_points_on_b[0] = vec3f::subtract(contact_point_2, out.base_offset);
    out.num_contact_points = 1;
}

void manifold_between_two_faces(ContactManifold &out, Vec3f contact_point_1, Vec3f contact_point_2, Vec3f penetration_axis,
                                float max_contact_distance, const face::Face &shape_1_face,
                                const face::Face &shape_2_face) {
    TUMBLE_ASSERT(max_contact_distance > 0.0f);

    out.num_contact_points = 0;

    // both faces need at least 2 vertices or else there can never be more than 1 contact point.
    // at least one face needs at least 3 vertices (when both have 2, only exactly matching edges give 2 contact
    // points, and that situation is unstable anyhow)
    const uint32_t num_vertices_1 = shape_1_face.num_vertices, num_vertices_2 = shape_2_face.num_vertices;
    if ((num_vertices_1 < num_vertices_2 ? num_vertices_1 : num_vertices_2) >= 2 &&
        (num_vertices_1 > num_vertices_2 ? num_vertices_1 : num_vertices_2) >= 3) {
        // swap the shapes if the 2nd face doesn't have enough vertices
        const bool swapped = num_vertices_2 < 3;
        const face::Face &face_1 = swapped ? shape_2_face : shape_1_face;
        const face::Face &face_2 = swapped ? shape_1_face : shape_2_face;
        Vec3f *contact_points_1 = swapped ? out.relative_contact_points_on_b : out.relative_contact_points_on_a;
        Vec3f *contact_points_2 = swapped ? out.relative_contact_points_on_a : out.relative_contact_points_on_b;
        const Vec3f axis = swapped ? vec3f::negate(penetration_axis) : penetration_axis;

        // plane origin and first edge from face 1
        const Vec3f plane_origin = face_1.vertices[0];
        const Vec3f first_edge = vec3f::subtract(face_1.vertices[1], plane_origin);

        Vec3f plane_normal;
        face::Face clipped_face;
        clipped_face.num_vertices = 0;
        if (face_1.num_vertices >= 3) {
            // clip the polygon of face 2 against that of 1
            clip::clip_poly_vs_poly(clipped_face, face_2, face_1, axis);

            // three vertices, can just calculate the normal
            const Vec3f second_edge = vec3f::subtract(face_1.vertices[2], plane_origin);
            plane_normal = vec3f::cross(first_edge, second_edge);
        } else {
            // clip the polygon of face 2 against the edge of 1
            clip::clip_poly_vs_edge(clipped_face, face_2, plane_origin, face_1.vertices[1], axis);

            // two vertices: a perpendicular to the edge and penetration axis, crossed with the edge, forms the normal
            const Vec3f perpendicular = vec3f::cross(first_edge, axis);
            plane_normal = vec3f::cross(perpendicular, first_edge);
        }

        // if penetration axis and plane normal are perpendicular, fall back to the contact points
        const float penetration_axis_dot_plane_normal = vec3f::dot(axis, plane_normal);
        if (penetration_axis_dot_plane_normal != 0.0f) {
            const float penetration_axis_len = vec3f::length(axis);

            for (uint32_t i = 0; i < clipped_face.num_vertices; i++) {
                const Vec3f p2 = clipped_face.vertices[i];

                // project p2 back onto the plane of face 1, solving:
                // p1 = p2 + distance * penetration_axis / |penetration_axis|
                // (p1 - plane_origin) . plane_normal = 0
                // distance = -|penetration_axis| * (p2 - plane_origin) . plane_normal / penetration_axis . plane_normal
                // (the -|penetration_axis| term is left out here and folded in below)
                const float distance = vec3f::dot(vec3f::subtract(p2, plane_origin), plane_normal) / penetration_axis_dot_plane_normal;

                // keep the point if it is less than max_contact_distance in front of the plane of face 1
                if (distance * penetration_axis_len < max_contact_distance) {
                    const Vec3f p1 = vec3f::scale_and_add(p2, axis, -distance);
                    const uint32_t index = out.num_contact_points;
                    contact_points_1[index] = vec3f::subtract(p1, out.base_offset);
                    contact_points_2[index] = vec3f::subtract(p2, out.base_offset);
                    out.num_contact_points++;

                    if (out.num_contact_points >= MAX_CLIPPING_CONTACT_POINTS) break;
                }
            }
        }
    }

    // if clipping produced no points, use original contact points
    if (out.num_contact_points == 0) set_single_point(out, contact_point_1, contact_point_2);
}

}  // namespace manifold

TUMBLE_NAMESPACE_END
