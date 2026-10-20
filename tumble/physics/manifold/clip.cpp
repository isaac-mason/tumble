#include "tumble/physics/manifold/clip.h"

#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace clip {

static void push_vertex(face::Face &out, Vec3f v) { out.vertices[out.num_vertices++] = v; }

void clip_poly_vs_plane(face::Face &out, const face::Face &polygon, Vec3f plane_origin, Vec3f plane_normal) {
    // reset output
    out.num_vertices = 0;

    if (polygon.num_vertices < 2) return;

    // get last vertex (wrapping loop)
    Vec3f e1 = polygon.vertices[polygon.num_vertices - 1];

    // determine state of last point
    float prev_num = vec3f::dot(vec3f::subtract(plane_origin, e1), plane_normal);
    bool prev_inside = prev_num < 0.0f;

    // loop through all vertices
    for (uint32_t j = 0; j < polygon.num_vertices; j++) {
        // check if second point is inside
        const Vec3f e2 = polygon.vertices[j];
        const float num = vec3f::dot(vec3f::subtract(plane_origin, e2), plane_normal);
        bool cur_inside = num < 0.0f;

        // in -> out or out -> in: add point on clipping plane
        if (cur_inside != prev_inside) {
            // solve: (X - plane_origin) . plane_normal = 0 and X = e1 + t * (e2 - e1) for X
            const Vec3f e12 = vec3f::subtract(e2, e1);
            const float denom = vec3f::dot(e12, plane_normal);
            if (denom != 0.0f) {
                push_vertex(out, vec3f::scale_and_add(e1, e12, prev_num / denom));
            } else {
                // edge is parallel to plane, treat point as if it were on the same side as the last point
                cur_inside = prev_inside;
            }
        }

        // point inside, add it
        if (cur_inside) push_vertex(out, e2);

        // update previous state
        prev_num = num;
        prev_inside = cur_inside;
        e1 = e2;
    }
}

void clip_poly_vs_poly(face::Face &out, const face::Face &polygon_to_clip, const face::Face &clipping_polygon, Vec3f clipping_polygon_normal) {
    // reset output
    out.num_vertices = 0;

    if (polygon_to_clip.num_vertices < 2) return;
    if (clipping_polygon.num_vertices < 3) return;

    face::Face tmp_faces[2];
    uint32_t tmp_index = 0;

    // loop through edges of clipping polygon
    for (uint32_t i = 0; i < clipping_polygon.num_vertices; i++) {
        // get edge to clip against
        const Vec3f clip_e1 = clipping_polygon.vertices[i];
        const Vec3f clip_e2 = clipping_polygon.vertices[(i + 1) % clipping_polygon.num_vertices];

        // pointing inward to the clipping polygon
        const Vec3f clip_normal = vec3f::cross(clipping_polygon_normal, vec3f::subtract(clip_e2, clip_e1));

        // get source and target faces
        const face::Face &source = i == 0 ? polygon_to_clip : tmp_faces[tmp_index];
        tmp_index ^= 1;
        face::Face &target = i == clipping_polygon.num_vertices - 1 ? out : tmp_faces[tmp_index];

        // clip against the edge
        clip_poly_vs_plane(target, source, clip_e1, clip_normal);

        // break out if no polygon left
        if (target.num_vertices < 3) {
            out.num_vertices = 0;
            break;
        }
    }
}

void clip_poly_vs_edge(face::Face &out, const face::Face &polygon, Vec3f edge_vertex_1, Vec3f edge_vertex_2, Vec3f clipping_edge_normal) {
    // reset output
    out.num_vertices = 0;

    if (polygon.num_vertices < 3) return;

    // get normal that is perpendicular to the edge and the clipping edge normal
    const Vec3f edge_normal = vec3f::cross(clipping_edge_normal, vec3f::subtract(edge_vertex_2, edge_vertex_1));

    // project vertices of edge on polygon
    const Vec3f p0 = polygon.vertices[0];
    const Vec3f p1 = polygon.vertices[1];
    const Vec3f p2 = polygon.vertices[2];
    const Vec3f polygon_normal = vec3f::cross(vec3f::subtract(p2, p0), vec3f::subtract(p1, p0));
    const float polygon_normal_len_sq = vec3f::squared_length(polygon_normal);

    const Vec3f v1 =
        vec3f::scale_and_add(edge_vertex_1, polygon_normal, vec3f::dot(vec3f::subtract(p0, edge_vertex_1), polygon_normal) / polygon_normal_len_sq);
    const Vec3f v2 =
        vec3f::scale_and_add(edge_vertex_2, polygon_normal, vec3f::dot(vec3f::subtract(p0, edge_vertex_2), polygon_normal) / polygon_normal_len_sq);
    const Vec3f v12 = vec3f::subtract(v2, v1);
    const float v12_len_sq = vec3f::squared_length(v12);

    // determine state of last point
    Vec3f e1 = polygon.vertices[polygon.num_vertices - 1];
    float prev_num = vec3f::dot(vec3f::subtract(edge_vertex_1, e1), edge_normal);
    bool prev_inside = prev_num < 0.0f;

    // loop through all vertices
    for (uint32_t j = 0; j < polygon.num_vertices; j++) {
        // check if second point is inside
        const Vec3f e2 = polygon.vertices[j];
        const float num = vec3f::dot(vec3f::subtract(edge_vertex_1, e2), edge_normal);
        const bool cur_inside = num < 0.0f;

        // in -> out or out -> in: add point on clipping plane
        if (cur_inside != prev_inside) {
            // solve: (edge_vertex_1 - X) . edge_normal = 0 and X = e1 + t * (e2 - e1) for X
            const Vec3f e12 = vec3f::subtract(e2, e1);
            const float denom = vec3f::dot(e12, edge_normal);
            const Vec3f clipped = denom != 0.0f ? vec3f::scale_and_add(e1, e12, prev_num / denom) : e1;

            // project point on line segment v1, v2 to see if it falls outside of the edge
            const float projection = vec3f::dot(vec3f::subtract(clipped, v1), v12);
            if (projection < 0.0f) {
                push_vertex(out, v1);
            } else if (projection > v12_len_sq) {
                push_vertex(out, v2);
            } else {
                push_vertex(out, clipped);
            }
        }

        // unlike clip_poly_vs_plane, interior vertices are not added: only the intersection points

        // update previous state
        prev_num = num;
        prev_inside = cur_inside;
        e1 = e2;
    }
}

}  // namespace clip

TUMBLE_NAMESPACE_END
