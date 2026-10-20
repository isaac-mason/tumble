#include "physics/collision/triangle.h"

#include "math/scalar.h"
#include "math/vec3f.h"

namespace triangle {

static void handle_degenerate_triangle(ClosestPointOnTriangleResult &out, Vec3f a, Vec3f b, Vec3f c,
                                       Vec3f ab, Vec3f ac) {
    Vec3f q;

    // start with vertex C being the closest
    uint32_t closest_feature = VERTEX_C;
    out.point = c;
    float best_dist_sq = vec3f::squared_length(c);

    // try vertex A
    const float a_len_sq = vec3f::squared_length(a);
    if (a_len_sq < best_dist_sq) {
        closest_feature = VERTEX_A;
        out.point = a;
        best_dist_sq = a_len_sq;
    }

    // try vertex B
    const float b_len_sq = vec3f::squared_length(b);
    if (b_len_sq < best_dist_sq) {
        closest_feature = VERTEX_B;
        out.point = b;
        best_dist_sq = b_len_sq;
    }

    constexpr float FLT_EPSILON_SQ = scalar::FLOAT_EPSILON * scalar::FLOAT_EPSILON;

    // edge AC
    const float ac_len_sq = vec3f::squared_length(ac);
    if (ac_len_sq > FLT_EPSILON_SQ) {
        const float v = scalar::clamp(-vec3f::dot(a, ac) / ac_len_sq, 0.0f, 1.0f);
        q = vec3f::scale_and_add(a, ac, v);
        const float dist_sq = vec3f::squared_length(q);
        if (dist_sq < best_dist_sq) {
            closest_feature = EDGE_AC;
            out.point = q;
            best_dist_sq = dist_sq;
        }
    }

    // edge BC
    Vec3f bc = vec3f::subtract(c, b);
    const float bc_len_sq = vec3f::squared_length(bc);
    if (bc_len_sq > FLT_EPSILON_SQ) {
        const float v = scalar::clamp(-vec3f::dot(b, bc) / bc_len_sq, 0.0f, 1.0f);
        q = vec3f::scale_and_add(b, bc, v);
        const float dist_sq = vec3f::squared_length(q);
        if (dist_sq < best_dist_sq) {
            closest_feature = EDGE_BC;
            out.point = q;
            best_dist_sq = dist_sq;
        }
    }

    // edge AB
    const float ab_len_sq = vec3f::squared_length(ab);
    if (ab_len_sq > FLT_EPSILON_SQ) {
        const float v = scalar::clamp(-vec3f::dot(a, ab) / ab_len_sq, 0.0f, 1.0f);
        q = vec3f::scale_and_add(a, ab, v);
        const float dist_sq = vec3f::squared_length(q);
        if (dist_sq < best_dist_sq) {
            closest_feature = EDGE_AB;
            out.point = q;
            best_dist_sq = dist_sq;
        }
    }

    out.distance_sq = best_dist_sq;
    out.feature = closest_feature;
}

void get_closest_point_on_triangle(ClosestPointOnTriangleResult &out, Vec3f a, Vec3f b, Vec3f c) {
    // edge vectors
    const Vec3f ab = vec3f::subtract(b, a);
    const Vec3f ac = vec3f::subtract(c, a);

    // check for degenerate triangle by computing normal
    const Vec3f n = vec3f::cross(ab, ac);
    const float n_len_sq = vec3f::squared_length(n);

    if (n_len_sq < 1e-10f) {
        // degenerate triangle - fallback to vertices and edges
        handle_degenerate_triangle(out, a, b, c, ab, ac);
        return;
    }

    // point relative to a (which is -a since p = origin = 0)
    const Vec3f ap = vec3f::negate(a);

    const float d1 = vec3f::dot(ab, ap);
    const float d2 = vec3f::dot(ac, ap);

    // check if P in vertex region outside A
    if (d1 <= 0.0f && d2 <= 0.0f) {
        out.point = a;
        out.distance_sq = vec3f::squared_length(a);
        out.feature = VERTEX_A;
        return;
    }

    // point relative to b (which is -b since p = origin = 0)
    const Vec3f bp = vec3f::negate(b);

    const float d3 = vec3f::dot(ab, bp);
    const float d4 = vec3f::dot(ac, bp);

    // check if P in vertex region outside B
    if (d3 >= 0.0f && d4 <= d3) {
        out.point = b;
        out.distance_sq = vec3f::squared_length(b);
        out.feature = VERTEX_B;
        return;
    }

    // check if P in edge region of AB
    if (d1 * d4 <= d3 * d2 && d1 >= 0.0f && d3 <= 0.0f) {
        const float v = d1 / (d1 - d3);
        out.point = vec3f::scale_and_add(a, ab, v);
        out.distance_sq = vec3f::squared_length(out.point);
        out.feature = EDGE_AB;
        return;
    }

    // point relative to c (which is -c since p = origin = 0)
    const Vec3f cp = vec3f::negate(c);

    const float d5 = vec3f::dot(ab, cp);
    const float d6 = vec3f::dot(ac, cp);

    // check if P in vertex region outside C
    if (d6 >= 0.0f && d5 <= d6) {
        out.point = c;
        out.distance_sq = vec3f::squared_length(c);
        out.feature = VERTEX_C;
        return;
    }

    // check if P in edge region of AC
    if (d5 * d2 <= d1 * d6 && d2 >= 0.0f && d6 <= 0.0f) {
        const float w = d2 / (d2 - d6);
        out.point = vec3f::scale_and_add(a, ac, w);
        out.distance_sq = vec3f::squared_length(out.point);
        out.feature = EDGE_AC;
        return;
    }

    // check if P in edge region of BC
    const float d4_d3 = d4 - d3;
    const float d5_d6 = d5 - d6;
    if (d3 * d6 <= d5 * d4 && d4_d3 >= 0.0f && d5_d6 >= 0.0f) {
        const float w = d4_d3 / (d4_d3 + d5_d6);
        Vec3f bc = vec3f::subtract(c, b);
        out.point = vec3f::scale_and_add(b, bc, w);
        out.distance_sq = vec3f::squared_length(out.point);
        out.feature = EDGE_BC;
        return;
    }

    // P inside face region.
    // compute closest point using the more accurate method from jolt:
    // distance = (centroid - origin) . normal / |normal|
    // closest = distance * normal / |normal|
    Vec3f sum = vec3f::add(a, b);
    sum = vec3f::add(sum, c);
    const float centroid_dot_n = vec3f::dot(sum, n);
    out.point = vec3f::scale(n, centroid_dot_n / (3.0f * n_len_sq));
    out.distance_sq = vec3f::squared_length(out.point);
    out.feature = FACE;
}

}  // namespace triangle
