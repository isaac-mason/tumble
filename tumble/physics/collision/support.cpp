#include "tumble/physics/collision/support.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/mem.h"

TUMBLE_NAMESPACE_BEGIN

namespace support {

// every evaluator rotates the direction into local space, finds the core support, adds the radius
// along the local direction and transforms the point back. these are the shared steps.

/** the direction into local space (inverse rotation = transposed 3x3) */
static inline Vec3f to_local_direction(const Support &support, Vec3f direction) {
    return support.has_transform ? mat4f::multiply3x3_transposed_vec(support.transform, direction) : direction;
}

/** rounds `point` by `radius` along the local direction */
static inline Vec3f add_radius_along(Vec3f point, Vec3f direction, float radius) {
    if (radius > 0.0f) {
        const float length_sq = vec3f::squared_length(direction);
        if (length_sq > 0.0f) return vec3f::scale_and_add(point, direction, radius / scalar::sqrt(length_sq));
    }
    return point;
}

/** the local point back to the caller's space */
static inline Vec3f to_caller_space(const Support &support, Vec3f point) {
    if (!support.has_transform) return point;
    return {vec3f::fix_w(v4f::add(mat4f::combine3(support.transform, point.v), support.transform.c[3]))};
}

Support create() {
    Support support;
    util::zero(&support, sizeof(Support));
    support.get_support = sphere_support;
    support.transform = mat4f::identity();
    support.hull.output_scale = 1.0f;
    support.hull.last_vertex = -1;
    return support;
}

/**
 * a sphere's support is rotation invariant: R·(r·dir_local) where dir_local = Rᵀ·dir is just r·dir,
 * so the direction transform and the rotation half of the transform-back cancel and only the
 * translation survives. add_radius is applied along that same direction, so it folds into the radius.
 */
Vec3f sphere_support(Support &support, Vec3f direction) {
    const Vec3f point = add_radius_along(vec3f::zero(), direction, support.sphere.radius + support.add_radius);
    if (support.has_transform) return {vec3f::fix_w(v4f::add(point.v, support.transform.c[3]))};
    return point;
}

/** the corner picked by the sign of the local direction on each axis */
Vec3f box_support(Support &support, Vec3f direction) {
    const Vec3f local = to_local_direction(support, direction);
    const Vec3f half_extents = support.box.half_extents;
    const Vec3f point = {v4f::select(half_extents.v, v4f::neg(half_extents.v), v4f::ge(local.v, v4f::zero()))};
    return to_caller_space(support, add_radius_along(point, local, support.add_radius));
}

/** the near end of the core segment, rounded by the capsule radius */
Vec3f capsule_support(Support &support, Vec3f direction) {
    const Vec3f local = to_local_direction(support, direction);
    const float half_height = support.capsule.half_height;
    const Vec3f point = vec3f::make(0.0f, vec3f::y(local) > 0.0f ? half_height : -half_height, 0.0f);
    // the rounding that makes the segment a capsule, plus the driver's
    return to_caller_space(support, add_radius_along(point, local, support.capsule.radius + support.add_radius));
}

/** the radial extreme in the local xz plane on the near/far cap */
Vec3f cylinder_support(Support &support, Vec3f direction) {
    const Vec3f local = to_local_direction(support, direction);
    const float lx = vec3f::x(local), lz = vec3f::z(local);
    const float y = vec3f::y(local) >= 0.0f ? support.cylinder.half_height : -support.cylinder.half_height;
    Vec3f point = vec3f::make(0.0f, y, 0.0f);
    const float horizontal_length = scalar::sqrt(lx * lx + lz * lz);
    if (horizontal_length > 0.0f) {
        const float scale = support.cylinder.radius / horizontal_length;
        point = vec3f::make(lx * scale, y, lz * scale);
    }
    return to_caller_space(support, add_radius_along(point, local, support.add_radius));
}

/** three points with a last-maximal tie-break; mesh contact quality depends on it */
Vec3f triangle_support(Support &support, Vec3f direction) {
    const Vec3f local = to_local_direction(support, direction);
    const Vec3f *v = support.triangle.vertices;
    const float dot_a = vec3f::dot(v[0], local);
    const float dot_b = vec3f::dot(v[1], local);
    const float dot_c = vec3f::dot(v[2], local);
    const uint32_t index = dot_a > dot_b ? (dot_a > dot_c ? 0 : 2) : dot_b > dot_c ? 1 : 2;
    return to_caller_space(support, add_radius_along(v[index], local, support.add_radius));
}

/** the point itself, rounded by add_radius along the local direction */
Vec3f point_support(Support &support, Vec3f direction) {
    Vec3f point = support.point;
    if (support.add_radius > 0.0f) point = add_radius_along(point, to_local_direction(support, direction), support.add_radius);
    return to_caller_space(support, point);
}

/**
 * the index of the vertex with the greatest dot against `direction`, the first on a tie; -1 when none
 * beats -inf. four vertices per iteration, each lane keeping its own running max and index, then
 * the lanes reduced and a scalar tail.
 */
static int32_t hull_brute_scan(const float *vertices, uint32_t vertex_count, Vec3f direction) {
    const float dx = vec3f::x(direction), dy = vec3f::y(direction), dz = vec3f::z(direction);
    const float neg_inf = -scalar::INF;
    float best_dot = neg_inf;
    int32_t best_index = -1;
    uint32_t i = 0;
    if (vertex_count >= 4) {
        const v128_t direction_x = v4f::splat(dx);
        const v128_t direction_y = v4f::splat(dy);
        const v128_t direction_z = v4f::splat(dz);
        v128_t lane_best_dot = v4f::splat(neg_inf);
        v128_t lane_best_index = wasm_i32x4_splat(-1);
        v128_t lane_index = wasm_i32x4_make(0, 1, 2, 3);
        const v128_t four = wasm_i32x4_splat(4);
        for (; i + 4 <= vertex_count; i += 4) {
            v128_t x, y, z;
            v4f::load3x4(vertices + i * 3, x, y, z);
            // x·dx + y·dy + z·dz in the scalar order, so lanes and tail agree bit for bit
            const v128_t dot = v4f::add(v4f::add(v4f::mul(x, direction_x), v4f::mul(y, direction_y)), v4f::mul(z, direction_z));
            const v128_t better = v4f::gt(dot, lane_best_dot);
            lane_best_dot = v4f::select(dot, lane_best_dot, better);
            lane_best_index = v4f::select(lane_index, lane_best_index, better);
            lane_index = wasm_i32x4_add(lane_index, four);
        }
        const float lane_dots[4] = {v4f::lane<0>(lane_best_dot), v4f::lane<1>(lane_best_dot), v4f::lane<2>(lane_best_dot),
                                    v4f::lane<3>(lane_best_dot)};
        const int32_t lane_indices[4] = {wasm_i32x4_extract_lane(lane_best_index, 0), wasm_i32x4_extract_lane(lane_best_index, 1),
                                         wasm_i32x4_extract_lane(lane_best_index, 2), wasm_i32x4_extract_lane(lane_best_index, 3)};
        for (uint32_t lane = 0; lane < 4; lane++) {
            if (lane_indices[lane] < 0) continue;
            if (lane_dots[lane] > best_dot || (lane_dots[lane] == best_dot && lane_indices[lane] < best_index)) {
                best_dot = lane_dots[lane];
                best_index = lane_indices[lane];
            }
        }
    }
    for (; i < vertex_count; i++) {
        const float *vertex = vertices + i * 3;
        const float dot = vertex[0] * dx + vertex[1] * dy + vertex[2] * dz;
        if (dot > best_dot) {
            best_dot = dot;
            best_index = (int32_t)i;
        }
    }
    return best_index;
}

/**
 * a hull or a polygon face. brute scan when no adjacency is baked or the pair is cold; otherwise a
 * warm hill-climb over the baked 1-ring.
 */
Vec3f hull_support(Support &support, Vec3f direction) {
    const Vec3f local = to_local_direction(support, direction);
    const float lx = vec3f::x(local), ly = vec3f::y(local), lz = vec3f::z(local);
    auto &hull = support.hull;
    const float *vertices = hull.vertices;
    const uint32_t *neighbors_start = hull.neighbors_start;
    Vec3f point = vec3f::zero();

    if (neighbors_start == nullptr || hull.last_vertex == -1) {
        // brute scan: no adjacency baked, or cold pair-fill. when accelerated, this first call seeds
        // the warm-start hint with the exact argmax.
        const int32_t best = hull_brute_scan(vertices, hull.vertex_count, local);
        if (best >= 0) point = vec3f::load(vertices, (uint32_t)best * 3);
        if (neighbors_start != nullptr) hull.last_vertex = best < 0 ? 0 : best;
    } else {
        // warm hill-climb over the baked 1-ring: take the best neighbour of the current vertex,
        // repeat until none improves. a local max over a convex hull's vertex graph is the global
        // max, so this returns a true support vertex. steps cap guards non-termination.
        const uint32_t *neighbors = hull.neighbors;
        const uint32_t vertex_count = hull.vertex_count;
        uint32_t current = (uint32_t)hull.last_vertex;
        const float *vertex = vertices + current * 3;
        float best_dot = vertex[0] * lx + vertex[1] * ly + vertex[2] * lz;
        uint32_t steps = 0;
        uint32_t previous;
        do {
            previous = current;
            const uint32_t end = neighbors_start[current + 1];
            for (uint32_t k = neighbors_start[current]; k < end; k++) {
                const uint32_t neighbor = neighbors[k];
                const float *neighbor_vertex = vertices + neighbor * 3;
                const float dot = neighbor_vertex[0] * lx + neighbor_vertex[1] * ly + neighbor_vertex[2] * lz;
                if (dot > best_dot) {
                    best_dot = dot;
                    current = neighbor;
                }
            }
        } while (current != previous && ++steps <= vertex_count);
        hull.last_vertex = (int32_t)current;
        point = vec3f::load(vertices, current * 3);
    }

    // uniform-scale fast path: scale the winning local vertex (must precede add_radius + transform-back)
    return to_caller_space(support, add_radius_along(vec3f::scale(point, hull.output_scale), local, support.add_radius));
}

/** triangle operand (mesh): copies the 3 verts */
void set_triangle_support(Support &out, Vec3f a, Vec3f b, Vec3f c, float convex_radius) {
    out.get_support = triangle_support;
    out.has_transform = false;
    out.add_radius = 0.0f;
    out.convex_radius = convex_radius;
    out.triangle.vertices[0] = a;
    out.triangle.vertices[1] = b;
    out.triangle.vertices[2] = c;
}

/** polygon face (kcc): borrows the face's vertex array (read-only, valid for this pair) */
void set_polygon_support(Support &out, const Vec3f *vertices, uint32_t vertex_count, float convex_radius) {
    out.get_support = hull_support;  // a face is just a convex vertex set
    out.has_transform = false;
    out.add_radius = 0.0f;
    out.convex_radius = convex_radius;
    // the hull evaluator scans flat xyz, so the face is copied into the scratch
    CC_ASSERT(vertex_count <= MAX_HULL_POINTS);
    for (uint32_t i = 0; i < vertex_count; i++) vec3f::store(out.hull.scratch, vertices[i], i * 3);
    out.hull.vertices = out.hull.scratch;
    out.hull.vertex_count = vertex_count;
    out.hull.output_scale = 1.0f;
    // a borrowed polygon face carries no adjacency: force the brute scan
    out.hull.neighbors_start = nullptr;
    out.hull.neighbors = nullptr;
    out.hull.last_vertex = -1;
}

/** point operand (collide point): copies the point */
void set_point_support(Support &out, Vec3f point) {
    out.get_support = point_support;
    out.has_transform = false;
    out.add_radius = 0.0f;
    out.convex_radius = 0.0f;
    out.point = point;
}

/** `dst` at least `length` long, as the ts grows its array with zeros */
static void grow_to(collections::Vec<float> &dst, uint32_t length) {
    dst.reserve(length);
    while (dst.length < length) dst.push(0.0f);
}

/**
 * intersects three planes n·p + d = 0 (cramer's rule; the cross products are the adjugate columns).
 * false when near-parallel.
 */
static bool intersect_planes(Vec3f &out, Vec3f n1, float d1, Vec3f n2, float d2, Vec3f n3, float d3) {
    const Vec3f c1 = vec3f::cross(n2, n3);
    const float denom = vec3f::dot(n1, c1);
    if (scalar::abs(denom) < 0.000001f) return false;
    const Vec3f c2 = vec3f::cross(n3, n1);
    const Vec3f c3 = vec3f::cross(n1, n2);
    const float s = -1.0f / denom;
    // d1 c1 + d2 c2 + d3 c3, scaled
    const Vec3f sum = vec3f::add(vec3f::add(vec3f::scale(c1, d1), vec3f::scale(c2, d2)), vec3f::scale(c3, d3));
    out = vec3f::scale(sum, s);
    return true;
}

/**
 * each neighbouring face plane is offset inward by the convex radius (constant += r) and the up-to-3
 * planes are intersected. for a 2-face vertex the third plane is perpendicular to the first two
 * through the vertex; its n1 × n2 normal is left unnormalized (the intersection is invariant to
 * per-plane scale). computed once per shape at create, so `convex_radius` is passed in rather than
 * read off the shape.
 */
void compute_shrunk_hull_points(collections::Vec<float> &dst, uint32_t num_points, const float *point_positions, const uint32_t *point_num_faces,
                                const int32_t *point_faces, const Plane3f *planes, float convex_radius) {
    grow_to(dst, num_points * 3);

    for (uint32_t pi = 0; pi < num_points; pi++) {
        const uint32_t pb = pi * 3;
        const Vec3f position = vec3f::load(point_positions, pb);
        const uint32_t num_faces = point_num_faces[pi];

        // first neighbouring face plane (normal is unit; offset inward: constant + r)
        const Plane3f &plane1 = planes[point_faces[pb]];
        const Vec3f n1 = vec3f::load(plane1.normal);
        Vec3f result;

        if (num_faces == 1) {
            // simple case: shift back along the single plane normal
            result = vec3f::scale_and_add(position, n1, -convex_radius);
        } else {
            const Plane3f &plane2 = planes[point_faces[pb + 1]];
            const Vec3f n2 = vec3f::load(plane2.normal);

            // offset the two face planes inward by the convex radius (use the stored plane constants)
            const float d1 = plane1.constant + convex_radius;
            const float d2 = plane2.constant + convex_radius;

            // third plane: 3rd face plane (offset inward), or a perpendicular plane through the vertex
            Vec3f n3;
            float d3;
            if (num_faces == 3) {
                const Plane3f &plane3 = planes[point_faces[pb + 2]];
                n3 = vec3f::load(plane3.normal);
                d3 = plane3.constant + convex_radius;
            } else {
                n3 = vec3f::cross(n1, n2);
                d3 = -vec3f::dot(n3, position);
            }

            // near-parallel planes: fall back to pushing back along the first plane
            if (!intersect_planes(result, n1, d1, n2, d2, n3, d3)) result = vec3f::scale_and_add(position, n1, -convex_radius);
        }

        vec3f::store(dst.data, result, pb);
    }
}

float scale_convex_radius(float radius, Vec3f scale) {
    // use minimum absolute scale component
    const float min_scale = scalar::min(scalar::min(scalar::abs(vec3f::x(scale)), scalar::abs(vec3f::y(scale))), scalar::abs(vec3f::z(scale)));
    return radius * min_scale;
}

/** a face normal through the inverse scale, renormalized */
static Vec3f scaled_normal(Vec3f normal, Vec3f inverse_scale) {
    const Vec3f n = vec3f::multiply(normal, inverse_scale);
    const float length_sq = vec3f::squared_length(n);
    if (length_sq > 0.0f) return vec3f::scale(n, 1.0f / scalar::sqrt(length_sq));
    return n;
}

/**
 * positions are scaled, face-plane normals transformed by the inverse scale and renormalized, planes
 * rebuilt through the scaled vertex, offset inward by the scaled convex radius, then intersected.
 * the 2-face third plane uses the unnormalized cross of n1, n2.
 */
void compute_scaled_shrunk_hull_points(float *dst, uint32_t num_points, const float *point_positions,
                                       const uint32_t *point_num_faces, const int32_t *point_faces, const Plane3f *planes,
                                       float convex_radius, Vec3f scale) {
    const float scaled_radius = scale_convex_radius(convex_radius, scale);

    const Vec3f inverse_scale = vec3f::inverse(scale);

    for (uint32_t pi = 0; pi < num_points; pi++) {
        const uint32_t pb = pi * 3;
        // scaled vertex position
        const Vec3f position = vec3f::multiply(vec3f::load(point_positions, pb), scale);
        const uint32_t num_faces = point_num_faces[pi];

        const Vec3f n1 = scaled_normal(vec3f::load(planes[point_faces[pb]].normal), inverse_scale);
        Vec3f result;

        if (num_faces == 1) {
            result = vec3f::scale_and_add(position, n1, -scaled_radius);
        } else {
            const Vec3f n2 = scaled_normal(vec3f::load(planes[point_faces[pb + 1]].normal), inverse_scale);

            // planes rebuilt through the scaled vertex, offset inward by the scaled convex radius
            const float d1 = -vec3f::dot(n1, position) + scaled_radius;
            const float d2 = -vec3f::dot(n2, position) + scaled_radius;

            Vec3f n3;
            float d3;
            if (num_faces == 3) {
                n3 = scaled_normal(vec3f::load(planes[point_faces[pb + 2]].normal), inverse_scale);
                d3 = -vec3f::dot(n3, position) + scaled_radius;
            } else {
                // third plane perpendicular to the first two, through the scaled vertex (unnormalized normal)
                n3 = vec3f::cross(n1, n2);
                d3 = -vec3f::dot(n3, position);
            }

            if (!intersect_planes(result, n1, d1, n2, d2, n3, d3)) result = vec3f::scale_and_add(position, n1, -scaled_radius);
        }

        vec3f::store(dst, result, pb);
    }
}

}  // namespace support

TUMBLE_NAMESPACE_END
