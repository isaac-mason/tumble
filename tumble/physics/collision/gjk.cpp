#include "physics/collision/gjk.h"

#include "math/scalar.h"
#include "math/simd.h"
#include "math/vec3f.h"
#include "physics/collision/closest_points.h"
#include "util/assert.h"

namespace gjk {

void compute_closest_point_on_line(ClosestPointResult &out, Vec3f a, Vec3f b, float squared_tolerance) {
    closest_points::BarycentricCoordinatesResult bary;
    closest_points::compute_barycentric_coordinates_2d(bary, a, b, squared_tolerance);
    const float u = bary.u;
    const float v = bary.v;

    if (v <= 0.0f) {
        // a is closest point
        out.point = a;
        out.point_set = 0b0001;
    } else if (u <= 0.0f) {
        // b is closest point
        out.point = b;
        out.point_set = 0b0010;
    } else {
        // closest point lies on line ab: out = a * u + b * v == lerp(a, b, v)
        out.point = vec3f::lerp(a, b, v);
        out.point_set = 0b0011;
    }
}

void compute_closest_point_on_triangle(ClosestPointResult &out, Vec3f in_a, Vec3f in_b, Vec3f in_c, bool must_include_c,
                                       float squared_tolerance) {
    // the most accurate normal is calculated by using the two shortest edges
    const Vec3f in_ac = vec3f::subtract(in_c, in_a);
    const Vec3f in_bc = vec3f::subtract(in_c, in_b);

    const bool swap_ac = vec3f::dot(in_bc, in_bc) < vec3f::dot(in_ac, in_ac);

    const Vec3f a = swap_ac ? in_c : in_a;
    const Vec3f c = swap_ac ? in_a : in_c;

    // calculate normal
    const Vec3f ab = vec3f::subtract(in_b, a);
    const Vec3f ac = vec3f::subtract(c, a);
    const Vec3f n = vec3f::cross(ab, ac);

    const float normal_length_squared = vec3f::dot(n, n);

    // check degenerate
    if (normal_length_squared < 1.0e-10f) {
        // degenerate, fallback to vertices and edges
        uint32_t closest_set = 0b0100;
        Vec3f closest = in_c;
        float best_distance_squared = vec3f::dot(in_c, in_c);

        if (!must_include_c) {
            // try vertex a
            const float a_length_squared = vec3f::dot(in_a, in_a);
            if (a_length_squared < best_distance_squared) {
                closest_set = 0b0001;
                closest = in_a;
                best_distance_squared = a_length_squared;
            }

            // try vertex b
            const float b_length_squared = vec3f::dot(in_b, in_b);
            if (b_length_squared < best_distance_squared) {
                closest_set = 0b0010;
                closest = in_b;
                best_distance_squared = b_length_squared;
            }
        }

        // edge ac
        const float ac_length_squared = vec3f::dot(ac, ac);
        if (ac_length_squared > squared_tolerance) {
            const float v = scalar::clamp(-vec3f::dot(a, ac) / ac_length_squared, 0.0f, 1.0f);
            const Vec3f q = vec3f::scale_and_add(a, ac, v);
            const float distance_squared = vec3f::dot(q, q);
            if (distance_squared < best_distance_squared) {
                closest_set = 0b0101;
                closest = q;
                best_distance_squared = distance_squared;
            }
        }

        // edge bc
        const float bc_length_squared = vec3f::dot(in_bc, in_bc);
        if (bc_length_squared > squared_tolerance) {
            const float v = scalar::clamp(-vec3f::dot(in_b, in_bc) / bc_length_squared, 0.0f, 1.0f);
            const Vec3f q = vec3f::scale_and_add(in_b, in_bc, v);
            const float distance_squared = vec3f::dot(q, q);
            if (distance_squared < best_distance_squared) {
                closest_set = 0b0110;
                closest = q;
                best_distance_squared = distance_squared;
            }
        }

        if (!must_include_c) {
            // edge ab
            const Vec3f in_ab = vec3f::subtract(in_b, in_a);
            const float ab_length_squared = vec3f::dot(in_ab, in_ab);
            if (ab_length_squared > squared_tolerance) {
                const float v = scalar::clamp(-vec3f::dot(in_a, in_ab) / ab_length_squared, 0.0f, 1.0f);
                const Vec3f q = vec3f::scale_and_add(in_a, in_ab, v);
                const float distance_squared = vec3f::dot(q, q);
                if (distance_squared < best_distance_squared) {
                    closest_set = 0b0011;
                    closest = q;
                }
            }
        }

        out.point_set = closest_set;
        out.point = closest;
        return;
    }

    // check if p in vertex region outside a
    const Vec3f ap = vec3f::negate(a);
    const float d1 = vec3f::dot(ab, ap);
    const float d2 = vec3f::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) {
        out.point_set = swap_ac ? 0b0100 : 0b0001;
        out.point = a;
        return;
    }

    // check if p in vertex region outside b
    const Vec3f bp = vec3f::negate(in_b);
    const float d3 = vec3f::dot(ab, bp);
    const float d4 = vec3f::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) {
        out.point_set = 0b0010;
        out.point = in_b;
        return;
    }

    // check if p in edge region of ab
    if (d1 * d4 <= d3 * d2 && d1 >= 0.0f && d3 <= 0.0f) {
        const float v = d1 / (d1 - d3);
        out.point_set = swap_ac ? 0b0110 : 0b0011;
        out.point = vec3f::scale_and_add(a, ab, v);
        return;
    }

    // check if p in vertex region outside c
    const Vec3f cp = vec3f::negate(c);
    const float d5 = vec3f::dot(ab, cp);
    const float d6 = vec3f::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) {
        out.point_set = swap_ac ? 0b0001 : 0b0100;
        out.point = c;
        return;
    }

    // check if p in edge region of ac
    if (d5 * d2 <= d1 * d6 && d2 >= 0.0f && d6 <= 0.0f) {
        const float w = d2 / (d2 - d6);
        out.point_set = 0b0101;
        out.point = vec3f::scale_and_add(a, ac, w);
        return;
    }

    // check if p in edge region of bc
    const float diff_d4_d3 = d4 - d3;
    const float diff_d5_d6 = d5 - d6;
    if (d3 * d6 <= d5 * d4 && diff_d4_d3 >= 0.0f && diff_d5_d6 >= 0.0f) {
        const float w = diff_d4_d3 / (diff_d4_d3 + diff_d5_d6);
        out.point_set = swap_ac ? 0b0011 : 0b0110;
        out.point = vec3f::scale_and_add(in_b, vec3f::subtract(c, in_b), w);
        return;
    }

    // p inside face region
    out.point_set = 0b0111;
    const Vec3f sum = vec3f::add(vec3f::add(a, in_b), c);
    out.point = vec3f::scale(n, vec3f::dot(sum, n) / (3.0f * normal_length_squared));
}

void compute_closest_point_on_tetrahedron(ClosestPointResult &out, Vec3f in_a, Vec3f in_b, Vec3f in_c, Vec3f in_d,
                                          bool must_include_d, float tolerance) {
    const float squared_tolerance = tolerance * tolerance;

    // start out assuming point inside all halfspaces
    out.point_set = 0b1111;
    out.point = vec3f::zero();

    float best_distance_squared = scalar::INF;

    // determine for each of the faces if the origin is outside
    const Vec3f ab = vec3f::subtract(in_b, in_a);
    const Vec3f ac = vec3f::subtract(in_c, in_a);
    const Vec3f ad = vec3f::subtract(in_d, in_a);
    const Vec3f bd = vec3f::subtract(in_d, in_b);
    const Vec3f bc = vec3f::subtract(in_c, in_b);

    // triangle normals
    const Vec3f ab_ac = vec3f::cross(ab, ac);
    const Vec3f ac_ad = vec3f::cross(ac, ad);
    const Vec3f ad_ab = vec3f::cross(ad, ab);
    const Vec3f bd_bc = vec3f::cross(bd, bc);

    // side of the origin for each plane
    const float sign_p_abc = vec3f::dot(in_a, ab_ac);
    const float sign_p_acd = vec3f::dot(in_a, ac_ad);
    const float sign_p_adb = vec3f::dot(in_a, ad_ab);
    const float sign_p_bdc = vec3f::dot(in_b, bd_bc);

    // side that is outside (determined by the 4th point)
    const float sign_d_abc = vec3f::dot(ad, ab_ac);   // d
    const float sign_d_acd = vec3f::dot(ab, ac_ad);   // b
    const float sign_d_adb = vec3f::dot(ac, ad_ab);   // c
    const float sign_d_bdc = -vec3f::dot(ab, bd_bc);  // a

    bool origin_out_abc;
    bool origin_out_acd;
    bool origin_out_adb;
    bool origin_out_bdc;

    if (sign_d_abc > 0.0f && sign_d_acd > 0.0f && sign_d_adb > 0.0f && sign_d_bdc > 0.0f) {
        origin_out_abc = sign_p_abc >= -tolerance;
        origin_out_acd = sign_p_acd >= -tolerance;
        origin_out_adb = sign_p_adb >= -tolerance;
        origin_out_bdc = sign_p_bdc >= -tolerance;
    } else if (sign_d_abc < 0.0f && sign_d_acd < 0.0f && sign_d_adb < 0.0f && sign_d_bdc < 0.0f) {
        origin_out_abc = sign_p_abc <= tolerance;
        origin_out_acd = sign_p_acd <= tolerance;
        origin_out_adb = sign_p_adb <= tolerance;
        origin_out_bdc = sign_p_bdc <= tolerance;
    } else {
        // mixed signs, degenerate tetrahedron: consider every face
        origin_out_abc = true;
        origin_out_acd = true;
        origin_out_adb = true;
        origin_out_bdc = true;
    }

    ClosestPointResult other;

    // if point outside face abc
    if (origin_out_abc) {
        if (must_include_d) {
            out.point_set = 0b0001;
            out.point = in_a;
        } else {
            compute_closest_point_on_triangle(out, in_a, in_b, in_c, false, squared_tolerance);
        }
        best_distance_squared = vec3f::dot(out.point, out.point);
    }

    // face acd
    if (origin_out_acd) {
        compute_closest_point_on_triangle(other, in_a, in_c, in_d, must_include_d, squared_tolerance);
        const float distance_squared = vec3f::dot(other.point, other.point);
        if (distance_squared < best_distance_squared) {
            best_distance_squared = distance_squared;
            out.point = other.point;
            out.point_set = (other.point_set & 0b0001) + ((other.point_set & 0b0110) << 1);
        }
    }

    // face adb
    if (origin_out_adb) {
        compute_closest_point_on_triangle(other, in_a, in_b, in_d, must_include_d, squared_tolerance);
        const float distance_squared = vec3f::dot(other.point, other.point);
        if (distance_squared < best_distance_squared) {
            best_distance_squared = distance_squared;
            out.point = other.point;
            out.point_set = (other.point_set & 0b0011) + ((other.point_set & 0b0100) << 1);
        }
    }

    // face bdc
    if (origin_out_bdc) {
        compute_closest_point_on_triangle(other, in_b, in_c, in_d, must_include_d, squared_tolerance);
        const float distance_squared = vec3f::dot(other.point, other.point);
        if (distance_squared < best_distance_squared) {
            out.point = other.point;
            out.point_set = other.point_set << 1;
        }
    }
}

static constexpr float GJK_TOLERANCE = 1e-5f;
static constexpr uint32_t GJK_MAX_ITERATIONS = 100;

struct ClosestPointToSimplexResult {
    Vec3f point;
    float squared_distance;
    uint32_t point_set;
    bool closest_point_found;
};

/** y = x - p per point (ray cast variant) */
static void recompute_simplex_y_from_p(simplex::Simplex &simplex, Vec3f x) {
    for (uint32_t i = 0; i < simplex.size; i++) simplex.y[i] = vec3f::subtract(x, simplex.p[i]);
}

/** y = x - (q - p) per point (shape cast variant) */
static void recompute_simplex_y_from_pq(simplex::Simplex &simplex, Vec3f x) {
    for (uint32_t i = 0; i < simplex.size; i++) simplex.y[i] = vec3f::add(vec3f::subtract(x, simplex.q[i]), simplex.p[i]);
}

/** compacts p down to the points selected by in_set (bit i selects point i) */
static void update_point_set_p(simplex::Simplex &simplex, uint32_t in_set) {
    uint32_t new_size = 0;
    for (uint32_t i = 0; i < simplex.size; i++) {
        if ((in_set & (1u << i)) != 0) {
            simplex.p[new_size] = simplex.p[i];
            new_size++;
        }
    }
    simplex.size = new_size;
}

/** compacts y, p and q down to the points selected by in_set */
static void update_point_set_ypq(simplex::Simplex &simplex, uint32_t in_set) {
    uint32_t new_size = 0;
    for (uint32_t i = 0; i < simplex.size; i++) {
        if ((in_set & (1u << i)) != 0) {
            simplex.y[new_size] = simplex.y[i];
            simplex.p[new_size] = simplex.p[i];
            simplex.q[new_size] = simplex.q[i];
            new_size++;
        }
    }
    simplex.size = new_size;
}

/** compacts p and q down to the points selected by in_set */
static void update_point_set_pq(simplex::Simplex &simplex, uint32_t in_set) {
    uint32_t new_size = 0;
    for (uint32_t i = 0; i < simplex.size; i++) {
        if ((in_set & (1u << i)) != 0) {
            simplex.p[new_size] = simplex.p[i];
            simplex.q[new_size] = simplex.q[i];
            new_size++;
        }
    }
    simplex.size = new_size;
}

static bool compute_closest_point_to_simplex(ClosestPointToSimplexResult &result, float prev_squared_dist, bool last_point_part_of_closest,
                                             const simplex::Simplex &simplex) {
    const Vec3f *y = simplex.y;
    ClosestPointResult closest;
    switch (simplex.size) {
        case 1: {
            // single point
            closest.point_set = 0b0001;
            closest.point = y[0];
            break;
        }
        case 2: {
            // line segment
            compute_closest_point_on_line(closest, y[0], y[1], 1e-10f);
            break;
        }
        case 3: {
            // triangle
            compute_closest_point_on_triangle(closest, y[0], y[1], y[2], last_point_part_of_closest, 1e-10f);
            break;
        }
        case 4: {
            // tetrahedron
            compute_closest_point_on_tetrahedron(closest, y[0], y[1], y[2], y[3], last_point_part_of_closest, 1e-5f);
            break;
        }
        default: {
            CC_ASSERT(false);
            result.closest_point_found = false;
            return false;
        }
    }

    const float squared_distance = vec3f::dot(closest.point, closest.point);

    // check if we found a closer point
    if (squared_distance < prev_squared_dist) {
        result.point = closest.point;
        result.squared_distance = squared_distance;
        result.point_set = closest.point_set;
        result.closest_point_found = true;
        return true;
    }

    // no better match found
    result.closest_point_found = false;
    return false;
}

/** u * points[0] + v * points[1] */
static inline Vec3f weighted_sum_2(const Vec3f *points, float u, float v) {
    return vec3f::scale_and_add(vec3f::scale(points[0], u), points[1], v);
}

/** u * points[0] + v * points[1] + w * points[2] */
static inline Vec3f weighted_sum_3(const Vec3f *points, float u, float v, float w) {
    return vec3f::scale_and_add(weighted_sum_2(points, u, v), points[2], w);
}

void cast_ray(GjkCastRayResult &out, Vec3f ray_origin, Vec3f ray_direction, float tolerance, support::Support &support,
              float max_lambda) {
    const float squared_tolerance = tolerance * tolerance;

    simplex::Simplex simplex;
    simplex.size = 0;

    ClosestPointToSimplexResult closest_point_to_simplex;

    float lambda = 0.0f;

    Vec3f x = ray_origin;

    // v = x - support(0)
    Vec3f p = support.get_support(support, vec3f::zero());
    Vec3f v = vec3f::subtract(x, p);

    float v_len_sq = scalar::FLOAT_MAX;
    bool allow_restart = false;

    uint32_t iterations = 0;
    while (iterations < GJK_MAX_ITERATIONS) {
        iterations++;

        // get new support point
        p = support.get_support(support, v);

        const Vec3f w = vec3f::subtract(x, p);

        const float v_dot_w = vec3f::dot(v, w);

        if (v_dot_w > 0.0f) {
            // if ray and normal are in the same direction, we've passed a and there's no collision
            const float v_dot_r = vec3f::dot(v, ray_direction);

            // instead of checking >= 0, check with epsilon as we don't want the division below to overflow to infinity as it can cause a float exception
            if (v_dot_r >= -1.0e-18f) {
                out.is_hit_found = false;
                out.lambda = 0.0f;
                return;
            }

            // update the lower bound for lambda
            const float delta = v_dot_w / v_dot_r;
            const float old_lambda = lambda;
            lambda -= delta;

            // if lambda didn't change, we cannot converge any further and we assume a hit
            if (old_lambda == lambda) break;

            // if lambda is bigger or equal than max, we don't have a hit
            if (lambda >= max_lambda) {
                out.is_hit_found = false;
                out.lambda = 0.0f;
                return;
            }

            // update x to new closest point on the ray
            x = vec3f::scale_and_add(ray_origin, ray_direction, lambda);

            // we've shifted x, so reset v_len_sq so that it is not used as early out for GetClosest
            v_len_sq = scalar::FLOAT_MAX;

            // we allow rebuilding the simplex once after x changes because the simplex was built
            // for another x and numerical round off builds up as you keep adding points to an
            // existing simplex
            allow_restart = true;
        }

        // add p to set P: P = P U {p}
        simplex.p[simplex.size] = p;
        simplex.size++;

        // calculate Y = {x} - P
        recompute_simplex_y_from_p(simplex, x);

        // determine the new closest point from Y to origin
        const bool found = compute_closest_point_to_simplex(closest_point_to_simplex, v_len_sq, false, simplex);
        if (found) {
            v_len_sq = closest_point_to_simplex.squared_distance;
            v = closest_point_to_simplex.point;
        }

        if (!found) {
            // only allow 1 restart, if we still can't get a closest point
            // we're so close that we return this as a hit
            if (!allow_restart) break;

            // if we fail to converge, we start again with the last point as simplex
            allow_restart = false;

            simplex.p[0] = p;
            simplex.size = 1;

            v = vec3f::subtract(x, p);
            v_len_sq = scalar::FLOAT_MAX;
            continue;
        } else if (closest_point_to_simplex.point_set == 0xf) {
            // we're inside the tetrahedron, we have a hit (verify that length of v is 0)
            break;
        }

        // update the points P to form the new simplex
        // note: we're not updating Y as Y will shift with x so we have to calculate it every iteration
        update_point_set_p(simplex, closest_point_to_simplex.point_set);

        // check if x is close enough to a
        if (v_len_sq <= squared_tolerance) break;
    }

    // store hit fraction
    out.is_hit_found = true;
    out.lambda = lambda;
    simplex::copy(out.simplex, simplex);
}

void cast_shape(GjkCastShapeResult &out, const Mat4f &transform_a_to_b, support::Support &shape_a_support,
                support::Support &shape_b_support, Vec3f displacement, float tolerance, float convex_radius_a,
                float convex_radius_b, float max_lambda) {
    // calculate how close a and b (without their convex radius) need to be to each other in order for us to consider this a collision
    float squared_tolerance = tolerance * tolerance;
    const float sum_convex_radius = convex_radius_a + convex_radius_b;

    // apply the a->b transform to shape a's support.
    // clear any add_radius left on a from a prior epa fold (a is reused across a cast loop, e.g. mesh
    // triangles), so the gjk pass sees a's bare core: its radius enters via `sum_convex_radius` below.
    // b keeps its add_radius: callers may pass a genuinely inflated b (e.g. the kcc padded polygon).
    shape_a_support.has_transform = true;
    shape_a_support.transform = transform_a_to_b;
    shape_a_support.add_radius = 0.0f;

    simplex::Simplex simplex;
    simplex.size = 0;

    ClosestPointToSimplexResult closest_point_to_simplex;

    float lambda = 0.0f;

    // since a is already transformed we can start the cast from zero
    Vec3f x = vec3f::zero();

    // v = -support_b + support_a (minkowski difference b - a in the space of a)
    Vec3f q = vec3f::negate(shape_b_support.get_support(shape_b_support, vec3f::zero()));
    Vec3f p = shape_a_support.get_support(shape_a_support, vec3f::zero());
    Vec3f v = vec3f::subtract(q, p);

    float v_len_sq = scalar::FLOAT_MAX;
    bool allow_restart = false;

    // keeps track of separating axis of the previous iteration.
    // initialized at zero as we don't know if our first v is actually a separating axis.
    Vec3f prev_v = vec3f::zero();

    uint32_t iterations = 0;
    while (iterations < GJK_MAX_ITERATIONS) {
        iterations++;

        // calculate the minkowski difference b - a
        // a is moving, so we need to add the back side of b to the front side of a
        // keep the support points on a and b separate so that in the end we can calculate a contact point
        p = shape_a_support.get_support(shape_a_support, vec3f::negate(v));
        q = shape_b_support.get_support(shape_b_support, v);

        const Vec3f w = vec3f::subtract(x, vec3f::subtract(q, p));

        // difference from article to this code:
        //
        // we did not include the convex radius in p and q in order to be able to calculate a good separating axis at the end of the algorithm.
        // however when moving forward along displacement we do need to take this into account so that we keep a and b separated by the sum of their convex radii.
        //
        // from p we have to subtract: convex_radius_a * v / |v|
        // to q we have to add: convex_radius_b * v / |v|
        // this means that to w we have to add: -(convex_radius_a + convex_radius_b) * v / |v|
        // so to v . w we have to add: v . (-(convex_radius_a + convex_radius_b) * v / |v|) = -(convex_radius_a + convex_radius_b) * |v|
        const float v_dot_w = vec3f::dot(v, w) - sum_convex_radius * scalar::sqrt(vec3f::squared_length(v));

        if (v_dot_w > 0.0f) {
            // if ray and normal are in the same direction, we've passed a and there's no collision
            const float v_dot_r = vec3f::dot(v, displacement);

            // instead of checking >= 0, check with epsilon as we don't want the division below to overflow to infinity as it can cause a float exception
            if (v_dot_r >= -1.0e-18f) {
                out.hit = false;
                return;
            }

            // update the lower bound for lambda
            const float delta = v_dot_w / v_dot_r;
            const float old_lambda = lambda;
            lambda -= delta;

            // if lambda didn't change, we cannot converge any further and we assume a hit
            if (old_lambda == lambda) break;

            // if lambda is bigger or equal than max, we don't have a hit
            if (lambda >= max_lambda) {
                out.hit = false;
                return;
            }

            // update x to new closest point on the ray
            x = vec3f::scale(displacement, lambda);

            // we've shifted x, so reset v_len_sq so that it is not used as early out when GetClosest returns false
            v_len_sq = scalar::FLOAT_MAX;

            // now that we've moved, we know that a and b are not intersecting at lambda = 0, so we can update our tolerance to stop iterating
            // as soon as a and b are convex_radius_a + convex_radius_b apart
            squared_tolerance = tolerance + sum_convex_radius;
            squared_tolerance = squared_tolerance * squared_tolerance;

            // we allow rebuilding the simplex once after x changes because the simplex was built for another x and numerical round off builds
            // up as you keep adding points to an existing simplex
            allow_restart = true;
        }

        // add p to set P, q to set Q: P = P U {p}, Q = Q U {q}
        simplex.p[simplex.size] = p;
        simplex.q[simplex.size] = q;
        simplex.size++;

        // calculate Y = {x} - (Q - P)
        recompute_simplex_y_from_pq(simplex, x);

        // determine the new closest point from Y to origin
        const bool found = compute_closest_point_to_simplex(closest_point_to_simplex, v_len_sq, false, simplex);
        if (found) {
            v_len_sq = closest_point_to_simplex.squared_distance;
            v = closest_point_to_simplex.point;
        }

        if (!found) {
            // only allow 1 restart, if we still can't get a closest point we're so close that we return this as a hit
            if (!allow_restart) {
                // the last support point did not produce a closer simplex, remove it so that the
                // contact points below are reconstructed from the previous valid simplex
                simplex.size--;
                break;
            }

            // if we fail to converge, we start again with the last point as simplex
            allow_restart = false;

            simplex.p[0] = p;
            simplex.q[0] = q;
            simplex.size = 1;

            v = vec3f::subtract(x, q);
            v_len_sq = scalar::FLOAT_MAX;
            continue;
        } else if (closest_point_to_simplex.point_set == 0xf) {
            // we're inside the tetrahedron, we have a hit (verify that length of v is 0)
            break;
        }

        // update the points P and Q to form the new simplex
        // note: we're not updating Y as Y will shift with x so we have to calculate it every iteration
        update_point_set_pq(simplex, closest_point_to_simplex.point_set);

        // check if a and b are touching according to our tolerance
        if (v_len_sq <= squared_tolerance) break;

        // store our v to return as separating axis
        prev_v = v;
    }

    // calculate Y = {x} - (Q - P) again so we can calculate the contact points
    recompute_simplex_y_from_pq(simplex, x);

    // compute normalized v for separating axis
    Vec3f normalized_v = vec3f::zero();
    const float v_len = scalar::sqrt(vec3f::squared_length(v));
    if (v_len > 0.0f) normalized_v = vec3f::scale(v, 1.0f / v_len);

    // compute contact points from simplex
    out.point_a = vec3f::zero();
    out.point_b = vec3f::zero();

    const Vec3f radius_b_offset = vec3f::scale(normalized_v, convex_radius_b);
    const Vec3f radius_a_offset = vec3f::scale(normalized_v, -convex_radius_a);

    switch (simplex.size) {
        case 1: {
            out.point_b = vec3f::add(simplex.q[0], radius_b_offset);
            out.point_a = lambda > 0.0f ? out.point_b : vec3f::add(simplex.p[0], radius_a_offset);
            break;
        }
        case 2: {
            closest_points::BarycentricCoordinatesResult bary;
            closest_points::compute_barycentric_coordinates_2d(bary, simplex.y[0], simplex.y[1], 1e-10f);

            out.point_b = vec3f::add(weighted_sum_2(simplex.q, bary.u, bary.v), radius_b_offset);
            out.point_a = lambda > 0.0f ? out.point_b : vec3f::add(weighted_sum_2(simplex.p, bary.u, bary.v), radius_a_offset);
            break;
        }
        case 3:
        case 4: {
            closest_points::BarycentricCoordinatesResult bary;
            closest_points::compute_barycentric_coordinates_3d(bary, simplex.y[0], simplex.y[1], simplex.y[2], 1e-10f);

            out.point_b = vec3f::add(weighted_sum_3(simplex.q, bary.u, bary.v, bary.w), radius_b_offset);
            out.point_a = lambda > 0.0f ? out.point_b : vec3f::add(weighted_sum_3(simplex.p, bary.u, bary.v, bary.w), radius_a_offset);
            break;
        }
    }

    // store results; epa continues from the simplex when the cast starts intersecting
    simplex::copy(out.simplex, simplex);
    out.lambda = lambda;
    out.hit = true;

    // use current v if we have convex radius, otherwise use previous v as approximation
    // (when there's no convex radius, the current v might be inaccurate due to numerical rounding)
    out.separating_axis = vec3f::negate(sum_convex_radius > 0.0f ? v : prev_v);
}

void closest_points(GjkClosestPoints &out, support::Support &support_a, support::Support &support_b, float tolerance,
                    Vec3f direction, float max_distance_squared) {
    const float squared_tolerance = tolerance * tolerance;

    simplex::Simplex simplex;
    simplex.size = 0;

    // v and its length^2
    ClosestPointToSimplexResult closest_point_to_simplex;
    closest_point_to_simplex.point = direction;
    closest_point_to_simplex.squared_distance = vec3f::squared_length(direction);
    closest_point_to_simplex.point_set = 0;
    closest_point_to_simplex.closest_point_found = true;
    // previous length^2 of v
    float previous_squared_distance = scalar::FLOAT_MAX;

    uint32_t iterations = 0;
    while (iterations++ < GJK_MAX_ITERATIONS) {
        // get support points for shape a and b in the direction of v
        const Vec3f p = support_a.get_support(support_a, closest_point_to_simplex.point);
        const Vec3f q = support_b.get_support(support_b, vec3f::negate(closest_point_to_simplex.point));

        // get support point of the minkowski sum a - b of v
        const Vec3f w = vec3f::subtract(p, q);

        const float dot = vec3f::dot(closest_point_to_simplex.point, w);

        // test if we have a separation of more than max_distance_squared, in which case we terminate early
        if (dot < 0.0f && dot * dot > closest_point_to_simplex.squared_distance * max_distance_squared) {
            out.squared_distance = scalar::FLOAT_MAX;
            out.penetration_axis = closest_point_to_simplex.point;
            return;
        }

        // store the point for later use
        simplex.y[simplex.size] = w;
        simplex.p[simplex.size] = p;
        simplex.q[simplex.size] = q;
        simplex.size++;

        compute_closest_point_to_simplex(closest_point_to_simplex, previous_squared_distance, true, simplex);

        if (!closest_point_to_simplex.closest_point_found) {
            // remove last added point from simplex
            simplex.size--;
            break;
        }

        // if there are 4 points, the origin is inside the tetrahedron and we're done
        if (closest_point_to_simplex.point_set == 0xf) {
            closest_point_to_simplex.point = vec3f::zero();
            closest_point_to_simplex.squared_distance = 0.0f;
            break;
        }

        // update the points of the simplex
        update_point_set_ypq(simplex, closest_point_to_simplex.point_set);

        // if v is very close to zero, we consider this a collision
        if (closest_point_to_simplex.squared_distance <= squared_tolerance) {
            closest_point_to_simplex.point = vec3f::zero();
            closest_point_to_simplex.squared_distance = 0.0f;
            break;
        }

        // if v is very small compared to the length of y, we also consider this a collision
        float y_max_length_squared = 0.0f;
        for (uint32_t i = 0; i < simplex.size; i++) y_max_length_squared = scalar::max(y_max_length_squared, vec3f::squared_length(simplex.y[i]));

        if (closest_point_to_simplex.squared_distance <= GJK_TOLERANCE * y_max_length_squared) {
            closest_point_to_simplex.point = vec3f::zero();
            closest_point_to_simplex.squared_distance = 0.0f;
            break;
        }

        // the next separation axis to test is the negative of the closest point of the minkowski sum to the origin
        // note: this must be done before terminating as converged since the separating axis is -v
        closest_point_to_simplex.point = vec3f::negate(closest_point_to_simplex.point);

        // if the squared length of v is not changing enough, we've converged and there is no collision
        if (previous_squared_distance - closest_point_to_simplex.squared_distance <= GJK_TOLERANCE * previous_squared_distance) {
            // v is a separating axis
            break;
        }

        previous_squared_distance = closest_point_to_simplex.squared_distance;
    }

    // extract the closest points
    out.point_a = vec3f::zero();
    out.point_b = vec3f::zero();

    simplex::copy(out.simplex, simplex);

    // handle early termination case: if simplex is empty, gjk terminated early (shapes are far apart)
    // return a large distance to indicate separation
    if (simplex.size == 0) {
        out.squared_distance = scalar::FLOAT_MAX;
        out.penetration_axis = vec3f::zero();
        return;
    }

    switch (simplex.size) {
        case 1: {
            // single point in simplex
            out.point_a = simplex.p[0];
            out.point_b = simplex.q[0];
            break;
        }
        case 2: {
            // line segment in simplex
            closest_points::BarycentricCoordinatesResult bary;
            closest_points::compute_barycentric_coordinates_2d(bary, simplex.y[0], simplex.y[1], 1e-10f);

            out.point_a = weighted_sum_2(simplex.p, bary.u, bary.v);
            out.point_b = weighted_sum_2(simplex.q, bary.u, bary.v);
            break;
        }
        case 3: {
            // triangle in simplex
            closest_points::BarycentricCoordinatesResult bary;
            closest_points::compute_barycentric_coordinates_3d(bary, simplex.y[0], simplex.y[1], simplex.y[2], 1e-10f);

            out.point_a = weighted_sum_3(simplex.p, bary.u, bary.v, bary.w);
            out.point_b = weighted_sum_3(simplex.q, bary.u, bary.v, bary.w);
            break;
        }
        default: {
            // for a full simplex (4 points = tetrahedron), the origin is inside the minkowski difference
            // in this case, point_a and point_b remain at zero (as initialized)
            break;
        }
    }

    // store the separating axis / penetration axis
    out.penetration_axis = closest_point_to_simplex.point;
    out.squared_distance = closest_point_to_simplex.squared_distance;
}

}  // namespace gjk
