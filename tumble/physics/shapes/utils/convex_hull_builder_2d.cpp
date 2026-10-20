#include "physics/shapes/utils/convex_hull_builder_2d.h"

#include "math/scalar.h"
#include "math/vec3f.h"
#include "util/assert.h"
#include "util/mem.h"

namespace convex_hull_builder_2d {

ConvexHullBuilder2D create(const Vec3f *positions, uint32_t num_positions) {
    ConvexHullBuilder2D builder;
    util::zero(&builder, sizeof(builder));
    builder.positions = positions;
    builder.num_positions = num_positions;
    builder.first_edge = -1;
    builder.num_edges = 0;
    builder.edge_free_head = -1;
    return builder;
}

void release(ConvexHullBuilder2D &builder) {
    for (uint32_t i = 0; i < builder.edge_pool.length; i++) builder.edge_pool[i].conflict_list.release();
    builder.edge_pool.release();
    builder.first_edge = -1;
    builder.num_edges = 0;
    builder.edge_free_head = -1;
}

/** a fresh unlinked edge starting at start_idx; a recycled slot keeps its conflict list storage. */
static int32_t create_edge(ConvexHullBuilder2D &builder, int32_t start_idx) {
    int32_t index = builder.edge_free_head;
    if (index != -1) {
        builder.edge_free_head = builder.edge_pool[index].next_edge;
    } else {
        Edge fresh;
        util::zero(&fresh, sizeof(fresh));
        builder.edge_pool.push(fresh);
        index = (int32_t)builder.edge_pool.length - 1;
    }
    Edge &edge = builder.edge_pool[index];
    edge.normal = vec3f::zero();
    edge.center = vec3f::zero();
    edge.conflict_list.length = 0;
    edge.prev_edge = -1;
    edge.next_edge = -1;
    edge.start_idx = start_idx;
    edge.furthest_point_distance_sq = 0.0f;
    return index;
}

/** clears the edge and returns it to the pool. */
static void free_edge(ConvexHullBuilder2D &builder, int32_t index) {
    Edge &edge = builder.edge_pool[index];
    edge.conflict_list.length = 0;
    edge.prev_edge = -1;
    edge.next_edge = builder.edge_free_head;
    builder.edge_free_head = index;
}

/** free all edges, walks the circular edge list and removes all edges */
static void free_edges(ConvexHullBuilder2D &builder) {
    if (builder.first_edge == -1) return;

    // walk circular list and clear all edges
    int32_t edge = builder.first_edge;
    do {
        const int32_t next = builder.edge_pool[edge].next_edge;
        free_edge(builder, edge);
        edge = next;
    } while (edge != -1 && edge != builder.first_edge);

    builder.first_edge = -1;
    builder.num_edges = 0;
}

/**
 * calculates the center and outward-pointing normal for a 2d edge. for an edge from p1 to p2 the
 * normal is (edge.y, -edge.x, 0), so that (normal x edge).z > 0 (points outward).
 */
static void calculate_normal_and_center(Edge &edge, const ConvexHullBuilder2D &builder) {
    const Vec3f p1 = builder.positions[edge.start_idx];
    const Vec3f p2 = builder.positions[builder.edge_pool[edge.next_edge].start_idx];

    // center of edge
    edge.center = vec3f::add(p1, p2);
    edge.center = vec3f::scale(edge.center, 0.5f);

    // create outward pointing normal
    const float edge_x = vec3f::x(p2) - vec3f::x(p1);
    const float edge_y = vec3f::y(p2) - vec3f::y(p1);
    edge.normal = vec3f::make(edge_y, -edge_x, 0.0f);
}

/** whether the point is in front of the edge (outside the hull) */
static bool is_facing(const Edge &edge, Vec3f position) {
    Vec3f delta = vec3f::subtract(position, edge.center);
    return vec3f::dot(edge.normal, delta) > 0.0f;
}

/**
 * finds the edge for which the point is furthest in front and adds it to that edge's conflict list,
 * keeping the furthest point last.
 */
static void assign_point_to_edge(ConvexHullBuilder2D &builder, int32_t position_idx, const int32_t *edges, uint32_t num_edges) {
    const Vec3f point = builder.positions[position_idx];

    int32_t best_edge = -1;
    float best_dist_sq = 0.0f;

    // test against all edges
    for (uint32_t i = 0; i < num_edges; i++) {
        const Edge &edge = builder.edge_pool[edges[i]];

        // determine distance to edge
        Vec3f delta = vec3f::subtract(point, edge.center);
        const float dot = vec3f::dot(edge.normal, delta);
        if (dot > 0.0f) {
            const float dist_sq = dot * dot / vec3f::squared_length(edge.normal);
            if (dist_sq > best_dist_sq) {
                best_edge = edges[i];
                best_dist_sq = dist_sq;
            }
        }
    }

    // if this point is in front of the edge, add it to the conflict list
    if (best_edge != -1) {
        Edge &edge = builder.edge_pool[best_edge];
        collections::Vec<int32_t> &list = edge.conflict_list;
        list.push(position_idx);
        if (best_dist_sq > edge.furthest_point_distance_sq) {
            // this point is further away than any others, update the distance and keep the point last
            edge.furthest_point_distance_sq = best_dist_sq;
        } else if (list.length >= 2) {
            // not the furthest point, move it to before the last point
            list[list.length - 1] = list[list.length - 2];
            list[list.length - 2] = position_idx;
        }
    }
}

EResult initialize(ConvexHullBuilder2D &builder, int32_t idx1, int32_t idx2, int32_t idx3, int32_t max_vertices, float tolerance,
                   collections::Vec<int32_t> &out_edges) {
    // clear any leftovers
    free_edges(builder);
    out_edges.length = 0;

    EResult result = SUCCESS;

    // determine a suitable tolerance for detecting that points are colinear
    // formula as per: "Implementing Quickhull - Dirk Gregorius"
    Vec3f vmax = vec3f::zero();
    for (uint32_t i = 0; i < builder.num_positions; i++) vmax = vec3f::fmax(vmax, vec3f::abs(builder.positions[i]));
    const float colinear_tolerance = 2.0f * scalar::FLOAT_EPSILON * (vec3f::x(vmax) + vec3f::y(vmax));
    const float colinear_tolerance_sq = colinear_tolerance * colinear_tolerance;

    // increase desired tolerance if accuracy doesn't allow it
    const float tolerance_sq = scalar::max(colinear_tolerance_sq, tolerance * tolerance);

    // start with the initial indices in counter clockwise order
    const Vec3f p1 = builder.positions[idx1];
    const Vec3f p2 = builder.positions[idx2];
    const Vec3f p3 = builder.positions[idx3];
    const float z = (vec3f::x(p2) - vec3f::x(p1)) * (vec3f::y(p3) - vec3f::y(p1)) - (vec3f::y(p2) - vec3f::y(p1)) * (vec3f::x(p3) - vec3f::x(p1));
    if (z < 0.0f) {
        const int32_t temp = idx1;
        idx1 = idx2;
        idx2 = temp;
    }

    // create and link edges
    const int32_t e1 = create_edge(builder, idx1);
    const int32_t e2 = create_edge(builder, idx2);
    const int32_t e3 = create_edge(builder, idx3);
    collections::Vec<Edge> &pool = builder.edge_pool;
    pool[e1].next_edge = e2;
    pool[e1].prev_edge = e3;
    pool[e2].next_edge = e3;
    pool[e2].prev_edge = e1;
    pool[e3].next_edge = e1;
    pool[e3].prev_edge = e2;
    builder.first_edge = e1;
    builder.num_edges = 3;

    // build the initial conflict lists
    const int32_t edges[3] = {e1, e2, e3};
    for (int32_t i = 0; i < 3; i++) calculate_normal_and_center(pool[edges[i]], builder);
    for (uint32_t idx = 0; idx < builder.num_positions; idx++) {
        const int32_t index = (int32_t)idx;
        if (index != idx1 && index != idx2 && index != idx3) assign_point_to_edge(builder, index, edges, 3);
    }

    // add the remaining points to the hull
    for (;;) {
        // check if we've reached the max amount of vertices that are allowed
        if (builder.num_edges >= max_vertices) {
            result = MAX_VERTICES_REACHED;
            break;
        }

        // find the edge with the furthest point on it
        int32_t edge_with_furthest_point = -1;
        float furthest_dist_sq = 0.0f;
        int32_t edge = builder.first_edge;
        do {
            if (pool[edge].furthest_point_distance_sq > furthest_dist_sq) {
                furthest_dist_sq = pool[edge].furthest_point_distance_sq;
                edge_with_furthest_point = edge;
            }
            edge = pool[edge].next_edge;
        } while (edge != builder.first_edge);

        // if there is none closer than our tolerance value, we're done
        if (edge_with_furthest_point == -1 || furthest_dist_sq < tolerance_sq) break;

        // take the furthest point
        collections::Vec<int32_t> &furthest_list = pool[edge_with_furthest_point].conflict_list;
        CC_ASSERT(furthest_list.length > 0);
        const int32_t furthest_point_idx = furthest_list.data[--furthest_list.length];
        const Vec3f furthest_point = builder.positions[furthest_point_idx];

        // find the horizon of edges that need to be removed
        // walk backwards from edge_with_furthest_point
        int32_t first_edge = edge_with_furthest_point;
        do {
            const int32_t prev = pool[first_edge].prev_edge;
            if (!is_facing(pool[prev], furthest_point)) break;
            first_edge = prev;
        } while (first_edge != edge_with_furthest_point);

        // walk forwards from edge_with_furthest_point
        int32_t last_edge = edge_with_furthest_point;
        do {
            const int32_t next = pool[last_edge].next_edge;
            if (!is_facing(pool[next], furthest_point)) break;
            last_edge = next;
        } while (last_edge != edge_with_furthest_point);

        // create new edges
        const int32_t new_e1 = create_edge(builder, pool[first_edge].start_idx);
        const int32_t new_e2 = create_edge(builder, furthest_point_idx);
        pool[new_e1].prev_edge = pool[first_edge].prev_edge;
        pool[new_e2].next_edge = pool[last_edge].next_edge;
        pool[new_e1].next_edge = new_e2;
        pool[new_e2].prev_edge = new_e1;
        pool[pool[new_e1].prev_edge].next_edge = new_e1;
        pool[pool[new_e2].next_edge].prev_edge = new_e2;
        builder.first_edge = new_e1;  // the old first edge may be deleted below
        builder.num_edges += 2;

        // calculate normals
        const int32_t new_edges[2] = {new_e1, new_e2};
        for (int32_t i = 0; i < 2; i++) calculate_normal_and_center(pool[new_edges[i]], builder);

        // delete the old edges
        for (;;) {
            const int32_t next = pool[first_edge].next_edge;

            // redistribute points in conflict list
            const collections::Vec<int32_t> &list = pool[first_edge].conflict_list;
            for (uint32_t i = 0; i < list.length; i++) assign_point_to_edge(builder, list[i], new_edges, 2);

            // clear the edge
            free_edge(builder, first_edge);
            builder.num_edges--;

            if (first_edge == last_edge) break;
            first_edge = next;
        }
    }

    // convert the edge list to a list of indices
    int32_t output_edge = builder.first_edge;
    do {
        out_edges.push(pool[output_edge].start_idx);
        output_edge = pool[output_edge].next_edge;
    } while (output_edge != builder.first_edge);

    return result;
}

bool validate_edges(const ConvexHullBuilder2D &builder) {
    if (builder.first_edge == -1) return builder.num_edges == 0;

    const collections::Vec<Edge> &pool = builder.edge_pool;
    int32_t count = 0;
    int32_t edge = builder.first_edge;
    do {
        const int32_t next = pool[edge].next_edge;
        const int32_t prev = pool[edge].prev_edge;
        if (next == -1 || prev == -1) return false;
        if (pool[next].prev_edge != edge || pool[prev].next_edge != edge) return false;
        count++;
        edge = next;
    } while (edge != builder.first_edge);

    return count == builder.num_edges;
}

}  // namespace convex_hull_builder_2d
