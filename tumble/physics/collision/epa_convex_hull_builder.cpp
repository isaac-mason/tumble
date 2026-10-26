#include "tumble/physics/collision/epa_convex_hull_builder.h"

#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace epa_convex_hull_builder {

void reset(EpaConvexHullBuilderState &state) {
    // all triangles are freed en masse
    state.triangle_high_watermark = 0;
    state.triangle_free_head = -1;
    state.queue_size = 0;
    state.edges_size = 0;
}

bool triangle_is_facing(const EpaConvexHullBuilderState &state, int32_t tri, Vec3f position) {
    const EpaConvexHullBuilderState::Triangle &triangle = state.triangles[tri];
    // dot(normal, position - centroid) > 0
    return vec3f::dot(triangle.normal, vec3f::subtract(position, triangle.centroid)) > 0.0f;
}

static inline float closest_length_sq_of(const EpaConvexHullBuilderState &state, int32_t tri) { return state.triangles[tri].closest_length_sq; }

static void push_triangle_to_queue(EpaConvexHullBuilderState &state, int32_t tri) {
    // a triangle is queued at most once, so the queue never outgrows the pool
    TUMBLE_ASSERT(state.queue_size < static_cast<uint32_t>(EPA_MAX_TRIANGLES));
    int32_t *queue = state.queue;
    queue[state.queue_size++] = tri;
    state.triangles[tri].flags |= TRI_FLAG_IN_QUEUE;

    // binary heap push: bubble up the new element
    uint32_t current = state.queue_size - 1;
    while (current > 0) {
        const int32_t current_element = queue[current];
        const uint32_t parent = (current - 1) >> 1;
        const int32_t parent_element = queue[parent];

        // min heap: parent should have smaller closest_length_sq than child
        if (closest_length_sq_of(state, parent_element) > closest_length_sq_of(state, current_element)) {
            queue[parent] = current_element;
            queue[current] = parent_element;
            current = parent;
        } else {
            break;
        }
    }
}

void link_triangle(EpaConvexHullBuilderState &state, int32_t t1, int32_t edge1, int32_t t2, int32_t edge2) {
    TUMBLE_ASSERT(edge1 >= 0 && edge1 < 3);
    TUMBLE_ASSERT(edge2 >= 0 && edge2 < 3);
    EpaConvexHullBuilderState::Edge &e1 = state.triangles[t1].edges[edge1];
    EpaConvexHullBuilderState::Edge &e2 = state.triangles[t2].edges[edge2];

    // not connected yet, and the vertices match
    TUMBLE_ASSERT(e1.neighbour_triangle == -1);
    TUMBLE_ASSERT(e2.neighbour_triangle == -1);
    TUMBLE_ASSERT(e1.start_index == state.triangles[t2].edges[(edge2 + 1) % 3].start_index);
    TUMBLE_ASSERT(e2.start_index == state.triangles[t1].edges[(edge1 + 1) % 3].start_index);

    e1.neighbour_triangle = t2;
    e1.neighbour_edge = edge2;
    e2.neighbour_triangle = t1;
    e2.neighbour_edge = edge1;
}

int32_t create_triangle(EpaConvexHullBuilderState &state, int32_t idx1, int32_t idx2, int32_t idx3) {
    int32_t tri;
    const int32_t free_head = state.triangle_free_head;
    if (free_head != -1) {
        // take from free list
        tri = free_head;
        state.triangle_free_head = state.triangles[tri].next_free;
    } else {
        // take from never-used watermark
        const int32_t index = state.triangle_high_watermark;
        if (index >= EPA_MAX_TRIANGLES) return -1;
        state.triangle_high_watermark = index + 1;
        tri = index;
    }

    EpaConvexHullBuilderState::Triangle &triangle = state.triangles[tri];

    // reset defaults (flags word clears removed/in_queue/lambda_relative_to_0/closest_point_interior)
    triangle.closest_length_sq = scalar::INF;
    triangle.lambda[0] = 0.0f;
    triangle.lambda[1] = 0.0f;
    triangle.flags = 0;

    triangle.edges[0].start_index = idx1;
    triangle.edges[1].start_index = idx2;
    triangle.edges[2].start_index = idx3;

    triangle.edges[0].neighbour_triangle = -1;
    triangle.edges[1].neighbour_triangle = -1;
    triangle.edges[2].neighbour_triangle = -1;

    const Vec3f y0 = state.positions[idx1];
    const Vec3f y1 = state.positions[idx2];
    const Vec3f y2 = state.positions[idx3];

    const Vec3f centroid = vec3f::divide(vec3f::add(vec3f::add(y0, y1), y2), vec3f::splat(3.0f));
    triangle.centroid = centroid;

    const Vec3f y10 = vec3f::subtract(y1, y0);
    const Vec3f y20 = vec3f::subtract(y2, y0);
    const Vec3f y21 = vec3f::subtract(y2, y1);

    // the most accurate normal is calculated by using the two shortest edges
    const float y20_dot_y20 = vec3f::dot(y20, y20);
    const float y21_dot_y21 = vec3f::dot(y21, y21);

    // both branches compute: normal, distance to origin, and barycentric coordinates
    // they differ only in which edge pair is selected for numerical stability
    if (y20_dot_y20 < y21_dot_y21) {
        // we select the edges y10 and y20
        const Vec3f normal = vec3f::cross(y10, y20);
        triangle.normal = normal;

        // check if triangle is degenerate (area too small)
        const float normal_len_sq = vec3f::dot(normal, normal);
        if (normal_len_sq > EPA_MIN_TRIANGLE_AREA) {
            // signed squared distance from origin to triangle plane: |c.n| * c.n keeps the sign while squaring
            const float c_dot_n = vec3f::dot(centroid, normal);
            triangle.closest_length_sq = (scalar::abs(c_dot_n) * c_dot_n) / normal_len_sq;

            // calculate barycentric coordinates of closest point on triangle to origin
            const float y10_dot_y10 = vec3f::dot(y10, y10);
            const float y10_dot_y20 = vec3f::dot(y10, y20);
            const float determinant = y10_dot_y10 * y20_dot_y20 - y10_dot_y20 * y10_dot_y20;

            // determinant > 0 means non-degenerate triangle (edges not parallel)
            if (determinant > 0.0f) {
                const float y0_dot_y10 = vec3f::dot(y0, y10);
                const float y0_dot_y20 = vec3f::dot(y0, y20);
                const float l0 = (y10_dot_y20 * y0_dot_y20 - y20_dot_y20 * y0_dot_y10) / determinant;
                const float l1 = (y10_dot_y20 * y0_dot_y10 - y10_dot_y10 * y0_dot_y20) / determinant;
                triangle.lambda[0] = l0;
                triangle.lambda[1] = l1;
                triangle.flags |= TRI_FLAG_LAMBDA_RELATIVE_TO_0;

                // the closest point is interior if l0 >= 0, l1 >= 0 and l0 + l1 <= 1
                if (l0 > -EPA_BARYCENTRIC_EPSILON && l1 > -EPA_BARYCENTRIC_EPSILON && l0 + l1 < 1.0f + EPA_BARYCENTRIC_EPSILON) {
                    triangle.flags |= TRI_FLAG_CLOSEST_POINT_INTERIOR;
                }
            }
        }
    } else {
        // we select the edges y10 and y21
        const Vec3f normal = vec3f::cross(y10, y21);
        triangle.normal = normal;

        // check if triangle is degenerate (area too small)
        const float normal_len_sq = vec3f::dot(normal, normal);
        if (normal_len_sq > EPA_MIN_TRIANGLE_AREA) {
            // signed squared distance from origin to triangle plane: |c.n| * c.n keeps the sign while squaring
            const float c_dot_n = vec3f::dot(centroid, normal);
            triangle.closest_length_sq = (scalar::abs(c_dot_n) * c_dot_n) / normal_len_sq;

            // calculate barycentric coordinates of closest point on triangle to origin (y1 as reference)
            const float y10_dot_y10 = vec3f::dot(y10, y10);
            const float y10_dot_y21 = vec3f::dot(y10, y21);
            const float determinant = y10_dot_y10 * y21_dot_y21 - y10_dot_y21 * y10_dot_y21;

            // determinant > 0 means non-degenerate triangle (edges not parallel)
            if (determinant > 0.0f) {
                const float y1_dot_y10 = vec3f::dot(y1, y10);
                const float y1_dot_y21 = vec3f::dot(y1, y21);
                const float l0 = (y21_dot_y21 * y1_dot_y10 - y10_dot_y21 * y1_dot_y21) / determinant;
                const float l1 = (y10_dot_y21 * y1_dot_y10 - y10_dot_y10 * y1_dot_y21) / determinant;
                triangle.lambda[0] = l0;
                triangle.lambda[1] = l1;
                // lambda_relative_to_0 stays clear

                // the closest point is interior if l0 >= 0, l1 >= 0 and l0 + l1 <= 1
                if (l0 > -EPA_BARYCENTRIC_EPSILON && l1 > -EPA_BARYCENTRIC_EPSILON && l0 + l1 < 1.0f + EPA_BARYCENTRIC_EPSILON) {
                    triangle.flags |= TRI_FLAG_CLOSEST_POINT_INTERIOR;
                }
            }
        }
    }

    return tri;
}

void initialize(EpaConvexHullBuilderState &state, int32_t idx1, int32_t idx2, int32_t idx3) {
    reset(state);

    // create triangles (back to back)
    const int32_t t1 = create_triangle(state, idx1, idx2, idx3);
    const int32_t t2 = create_triangle(state, idx1, idx3, idx2);
    TUMBLE_ASSERT(t1 != -1 && t2 != -1);

    link_triangle(state, t1, 0, t2, 2);
    link_triangle(state, t1, 1, t2, 1);
    link_triangle(state, t1, 2, t2, 0);

    // always add both triangles to the priority queue
    push_triangle_to_queue(state, t1);
    push_triangle_to_queue(state, t2);
}

bool has_next_triangle(const EpaConvexHullBuilderState &state) { return state.queue_size > 0; }

int32_t peek_closest_triangle_in_queue(const EpaConvexHullBuilderState &state) { return state.queue_size == 0 ? -1 : state.queue[0]; }

int32_t pop_closest_triangle_from_queue(EpaConvexHullBuilderState &state) {
    if (state.queue_size == 0) return -1;
    int32_t *heap = state.queue;

    // binary heap pop: move root to end, then bubble down
    const int32_t temp = heap[state.queue_size - 1];
    heap[state.queue_size - 1] = heap[0];
    heap[0] = temp;

    const uint32_t count = state.queue_size - 1;
    uint32_t largest = 0;

    while (true) {
        uint32_t child = (largest << 1) + 1;
        if (child >= count) break;

        const uint32_t prev_largest = largest;

        // find the child with the smallest closest_length_sq
        if (closest_length_sq_of(state, heap[largest]) > closest_length_sq_of(state, heap[child])) largest = child;

        ++child;

        if (child < count && closest_length_sq_of(state, heap[largest]) > closest_length_sq_of(state, heap[child])) largest = child;

        if (prev_largest == largest) break;

        const int32_t temp_element = heap[prev_largest];
        heap[prev_largest] = heap[largest];
        heap[largest] = temp_element;
    }

    return heap[--state.queue_size];
}

int32_t find_facing_triangle(const EpaConvexHullBuilderState &state, Vec3f position, float &out_best_dist_sq) {
    int32_t best = -1;
    float best_dist_sq = 0.0f;
    const int32_t *queue = state.queue;
    const uint32_t queue_length = state.queue_size;

    const V128 px = v4f::splat(vec3f::x(position));
    const V128 py = v4f::splat(vec3f::y(position));
    const V128 pz = v4f::splat(vec3f::z(position));

    // four triangles at a time: the plane distance per lane, then the scalar pick in queue order
    for (uint32_t base = 0; base < queue_length; base += 4) {
        const uint32_t count = queue_length - base < 4 ? queue_length - base : 4;
        const EpaConvexHullBuilderState::Triangle *tris[4];
        int32_t indices[4];
        for (uint32_t lane = 0; lane < 4; lane++) {
            indices[lane] = queue[base + (lane < count ? lane : 0)];
            tris[lane] = &state.triangles[indices[lane]];
        }

        V128 nx, ny, nz, cx, cy, cz;
        v4f::transpose3x4(tris[0]->normal.v, tris[1]->normal.v, tris[2]->normal.v, tris[3]->normal.v, nx, ny, nz);
        v4f::transpose3x4(tris[0]->centroid.v, tris[1]->centroid.v, tris[2]->centroid.v, tris[3]->centroid.v, cx, cy, cz);

        const V128 abx = v4f::sub(px, cx);
        const V128 aby = v4f::sub(py, cy);
        const V128 abz = v4f::sub(pz, cz);
        const V128 dot = v4f::add(v4f::add(v4f::mul(nx, abx), v4f::mul(ny, aby)), v4f::mul(nz, abz));
        const V128 normal_len_sq = v4f::add(v4f::add(v4f::mul(nx, nx), v4f::mul(ny, ny)), v4f::mul(nz, nz));
        const V128 dist_sq = v4f::div(v4f::mul(dot, dot), normal_len_sq);

        float dots[4], dist_sqs[4];
        v4f::store4(dots, dot);
        v4f::store4(dist_sqs, dist_sq);

        for (uint32_t lane = 0; lane < count; lane++) {
            if ((tris[lane]->flags & TRI_FLAG_REMOVED) != 0) continue;
            if (dots[lane] > 0.0f && dist_sqs[lane] > best_dist_sq) {
                best = indices[lane];
                best_dist_sq = dist_sqs[lane];
            }
        }
    }

    out_best_dist_sq = best_dist_sq;
    return best;
}

void free_triangle(EpaConvexHullBuilderState &state, int32_t tri) {
    EpaConvexHullBuilderState::Triangle &triangle = state.triangles[tri];
    // only a removed, unlinked triangle goes back
    TUMBLE_ASSERT((triangle.flags & TRI_FLAG_REMOVED) != 0);
    TUMBLE_ASSERT(triangle.edges[0].neighbour_triangle == -1 && triangle.edges[1].neighbour_triangle == -1 &&
              triangle.edges[2].neighbour_triangle == -1);
    triangle.next_free = state.triangle_free_head;
    state.triangle_free_head = tri;
}

void unlink_triangle(EpaConvexHullBuilderState &state, int32_t tri) {
    EpaConvexHullBuilderState::Triangle &triangle = state.triangles[tri];

    for (uint32_t edge = 0; edge < 3; edge++) {
        EpaConvexHullBuilderState::Edge &e = triangle.edges[edge];
        if (e.neighbour_triangle != -1) {
            state.triangles[e.neighbour_triangle].edges[e.neighbour_edge].neighbour_triangle = -1;
            e.neighbour_triangle = -1;
        }
    }

    // if this triangle is not in the priority queue, we can delete it now
    if ((triangle.flags & TRI_FLAG_IN_QUEUE) == 0) free_triangle(state, tri);
}

bool find_edge(EpaConvexHullBuilderState &state, int32_t facing_triangle, Vec3f position) {
    EpaConvexHullBuilderState::Edge *edges = state.edges;
    EpaConvexHullBuilderState::StackFrame *stack = state.stack;

    state.edges_size = 0;

    state.triangles[facing_triangle].flags |= TRI_FLAG_REMOVED;

    // build our own stack, starting with the triangle/edge provided
    int32_t cur_stack_pos = 0;
    stack[0] = {facing_triangle, 0, -1};

    // next index that we expect to find
    int32_t next_expected_start_idx = -1;

    while (true) {
        EpaConvexHullBuilderState::StackFrame &frame = stack[cur_stack_pos];
        const int32_t cur_triangle = frame.triangle;
        const int32_t cur_edge = frame.edge;

        const int32_t cur_iter = frame.iteration + 1;
        frame.iteration = cur_iter;

        if (cur_iter >= 3) {
            // this triangle needs to be removed, unlink it now
            unlink_triangle(state, cur_triangle);

            if (--cur_stack_pos < 0) break;
        } else {
            // visit neighbour: edge index is (edge + iter) % 3
            const int32_t edge_sum = cur_edge + cur_iter;
            const int32_t edge_idx = edge_sum >= 3 ? edge_sum - 3 : edge_sum;
            const EpaConvexHullBuilderState::Edge &edge = state.triangles[cur_triangle].edges[edge_idx];
            const int32_t n = edge.neighbour_triangle;
            const int32_t n_edge = edge.neighbour_edge;
            const int32_t e_start_index = edge.start_index;

            if (n != -1 && (state.triangles[n].flags & TRI_FLAG_REMOVED) == 0) {
                // check if vertex is on the front side of this triangle
                if (triangle_is_facing(state, n, position)) {
                    // vertex on front, this triangle needs to be removed
                    state.triangles[n].flags |= TRI_FLAG_REMOVED;

                    cur_stack_pos++;
                    TUMBLE_ASSERT(cur_stack_pos < EPA_MAX_EDGE_LENGTH);
                    if (cur_stack_pos >= EPA_MAX_EDGE_LENGTH) return false;
                    stack[cur_stack_pos] = {n, n_edge, 0};
                } else {
                    // detect islands - if edge doesn't connect to previous edge
                    if (e_start_index != next_expected_start_idx && next_expected_start_idx != -1) return false;

                    // next expected index is the start index of our neighbour's edge
                    next_expected_start_idx = state.triangles[n].edges[n_edge].start_index;

                    // vertex behind, keep edge
                    TUMBLE_ASSERT(state.edges_size < static_cast<uint32_t>(EPA_MAX_EDGE_LENGTH));
                    if (state.edges_size >= static_cast<uint32_t>(EPA_MAX_EDGE_LENGTH)) return false;
                    edges[state.edges_size++] = {n, n_edge, e_start_index};
                }
            }
        }
    }

    // assert that we have a fully connected loop
    if (state.edges_size != 0 && edges[0].start_index != next_expected_start_idx) return false;

    // need at least 3 edges to form a valid hull
    return state.edges_size >= 3;
}

bool add_point(EpaConvexHullBuilderState &state, int32_t facing_triangle, int32_t idx, float closest_dist_sq,
               EpaConvexHullBuilderState::NewTriangles &out_triangles) {
    // check if structure is intact
    TUMBLE_ASSERT(validate_triangles(state));

    // find edge of convex hull of triangles that are not facing the new vertex
    if (!find_edge(state, facing_triangle, state.positions[idx])) return false;

    // create new triangles
    const uint32_t first = out_triangles.size;
    const uint32_t num_edges = state.edges_size;
    for (uint32_t i = 0; i < num_edges; i++) {
        const uint32_t i_next = i + 1 < num_edges ? i + 1 : 0;
        const int32_t nt = create_triangle(state, state.edges[i].start_index, state.edges[i_next].start_index, idx);
        if (nt == -1) return false;

        TUMBLE_ASSERT(out_triangles.size < static_cast<uint32_t>(EPA_MAX_EDGE_LENGTH));
        if (out_triangles.size >= static_cast<uint32_t>(EPA_MAX_EDGE_LENGTH)) return false;
        out_triangles.values[out_triangles.size++] = nt;

        // check if we need to put this triangle in the priority queue
        const float closest_length_sq = closest_length_sq_of(state, nt);
        const bool closest_point_interior = (state.triangles[nt].flags & TRI_FLAG_CLOSEST_POINT_INTERIOR) != 0;
        if ((closest_point_interior && closest_length_sq < closest_dist_sq) || closest_length_sq < 0.0f) push_triangle_to_queue(state, nt);
    }

    // link edges in second pass (can't do in first pass because we need all triangles created)
    for (uint32_t i = 0; i < num_edges; i++) {
        const uint32_t i_next = i + 1 < num_edges ? i + 1 : 0;
        const int32_t t1 = out_triangles.values[first + i];
        const int32_t t2 = out_triangles.values[first + i_next];
        const EpaConvexHullBuilderState::Edge &edge = state.edges[i];

        link_triangle(state, t1, 0, edge.neighbour_triangle, edge.neighbour_edge);
        link_triangle(state, t1, 1, t2, 2);
    }

    // check if structure is intact
    TUMBLE_ASSERT(validate_triangles(state));

    return true;
}

bool validate_triangle(const EpaConvexHullBuilderState &state, int32_t tri) {
    const EpaConvexHullBuilderState::Triangle &triangle = state.triangles[tri];

    if ((triangle.flags & TRI_FLAG_REMOVED) != 0) {
        // removed triangles are not connected to anything
        for (uint32_t i = 0; i < 3; i++) {
            if (triangle.edges[i].neighbour_triangle != -1) return false;
        }
        return true;
    }

    for (int32_t i = 0; i < 3; i++) {
        const EpaConvexHullBuilderState::Edge &my_edge = triangle.edges[i];

        // we have a neighbour
        const int32_t neighbour = my_edge.neighbour_triangle;
        if (neighbour == -1) return false;
        const int32_t neighbour_edge_index = my_edge.neighbour_edge;
        const EpaConvexHullBuilderState::Triangle &neighbour_triangle = state.triangles[neighbour];
        const EpaConvexHullBuilderState::Edge &neighbour_edge = neighbour_triangle.edges[neighbour_edge_index];

        // our neighbour's edge points to us
        if (neighbour_edge.neighbour_triangle != tri || neighbour_edge.neighbour_edge != i) return false;

        // the next edge of the neighbour starts at this edge's vertex
        if (neighbour_triangle.edges[(neighbour_edge_index + 1) % 3].start_index != my_edge.start_index) return false;

        // my next edge starts at my neighbour's edge's vertex
        if (triangle.edges[(i + 1) % 3].start_index != neighbour_edge.start_index) return false;
    }
    return true;
}

bool validate_triangles(const EpaConvexHullBuilderState &state) {
    for (int32_t tri = 0; tri < state.triangle_high_watermark; tri++) {
        if (!validate_triangle(state, tri)) return false;
    }
    return true;
}

}  // namespace epa_convex_hull_builder

TUMBLE_NAMESPACE_END
