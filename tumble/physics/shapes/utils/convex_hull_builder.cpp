#include "tumble/physics/shapes/utils/convex_hull_builder.h"

#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/utils/convex_hull_builder_2d.h"
#include "tumble/util/assert.h"
#include "tumble/util/mem.h"

TUMBLE_NAMESPACE_BEGIN

namespace convex_hull_builder {

/** find_edge's own stack depth */
static constexpr int32_t MAX_EDGE_LENGTH = 128;

static inline Face &face_at(ConvexHullBuilder &builder, int32_t face) { return builder.face_pool[face]; }
static inline const Face &face_at(const ConvexHullBuilder &builder, int32_t face) { return builder.face_pool[face]; }
static inline Edge &edge_at(ConvexHullBuilder &builder, int32_t edge) { return builder.edge_pool[edge]; }
static inline const Edge &edge_at(const ConvexHullBuilder &builder, int32_t edge) { return builder.edge_pool[edge]; }

ConvexHullBuilder create(const Vec3f *positions, uint32_t num_positions) {
    ConvexHullBuilder builder;
    util::zero(&builder, sizeof(builder));
    builder.positions = positions;
    builder.num_positions = num_positions;
    builder.face_free_head = -1;
    builder.edge_free_head = -1;
    return builder;
}

void release(ConvexHullBuilder &builder) {
    for (uint32_t i = 0; i < builder.face_pool.length; i++) builder.face_pool[i].conflict_list.release();
    builder.faces.release();
    builder.coplanar_list.release();
    builder.face_pool.release();
    builder.edge_pool.release();
    builder.coplanar_scratch.release();
    builder.horizon.release();
    builder.new_faces.release();
    builder.affected_faces.release();
    builder.positions_2d.release();
    builder.face_free_head = -1;
    builder.edge_free_head = -1;
}

/** a new edge of `face` starting at start_idx, unlinked. */
static int32_t create_edge(ConvexHullBuilder &builder, int32_t face, int32_t start_idx) {
    int32_t index = builder.edge_free_head;
    if (index != -1) {
        builder.edge_free_head = edge_at(builder, index).next_edge;
    } else {
        builder.edge_pool.push({});
        index = (int32_t)builder.edge_pool.length - 1;
    }
    Edge &edge = edge_at(builder, index);
    edge.face = face;
    edge.next_edge = -1;
    edge.neighbour_edge = -1;
    edge.start_idx = start_idx;
    return index;
}

static void free_edge(ConvexHullBuilder &builder, int32_t index) {
    Edge &edge = edge_at(builder, index);
    edge.face = -1;
    edge.neighbour_edge = -1;
    edge.next_edge = builder.edge_free_head;
    builder.edge_free_head = index;
}

/**
 * calculates the normal and centroid for a face using a triangle fan approach.
 * the normal length is 2x the area of the face.
 */
static void calculate_normal_and_centroid(ConvexHullBuilder &builder, int32_t face_index) {
    Face &face = face_at(builder, face_index);

    // get point that we use to construct a triangle fan
    int32_t e = face.first_edge;
    if (e == -1) return;
    const Vec3f y0 = builder.positions[edge_at(builder, e).start_idx];

    // get the 2nd point
    e = edge_at(builder, e).next_edge;
    if (e == -1 || e == face.first_edge) return;
    const Vec3f *y1 = &builder.positions[edge_at(builder, e).start_idx];

    // start accumulating the centroid
    face.centroid = vec3f::add(y0, *y1);
    int32_t n = 2;

    // start accumulating the normal
    face.normal = vec3f::zero();

    // loop over remaining edges accumulating normals in a triangle fan fashion
    for (e = edge_at(builder, e).next_edge; e != -1 && e != face.first_edge; e = edge_at(builder, e).next_edge) {
        // get the 3rd point
        const Vec3f &y2 = builder.positions[edge_at(builder, e).start_idx];

        // calculate edges (counter clockwise)
        const Vec3f edge0 = vec3f::subtract(*y1, y0);
        Vec3f edge1 = vec3f::subtract(y2, *y1);
        Vec3f edge2 = vec3f::subtract(y0, y2);

        // the best normal is calculated by using the two shortest edges
        // see: https://box2d.org/posts/2014/01/troublesome-troublesome-triangle/
        // we check which edge is shorter: e1 or e2
        const bool e1_shorter_than_e2 = vec3f::dot(edge1, edge1) < vec3f::dot(edge2, edge2);

        // select the normal with the shortest edge
        Vec3f normal;
        if (e1_shorter_than_e2) {
            normal = vec3f::cross(edge0, edge1);
        } else {
            normal = vec3f::cross(edge2, edge0);
        }
        face.normal = vec3f::add(face.normal, normal);

        // accumulate centroid
        face.centroid = vec3f::add(face.centroid, y2);
        n++;

        // update y1 for next triangle
        y1 = &y2;
    }

    // finalize centroid
    face.centroid = vec3f::scale(face.centroid, 1.0f / (float)n);
}

/** whether the point is in front of the face plane */
static bool is_facing(const Face &face, Vec3f position) {
    Vec3f delta = vec3f::subtract(position, face.centroid);
    return vec3f::dot(face.normal, delta) > 0.0f;
}

/** walks the circular edge list to find the edge that points to this edge */
static int32_t get_previous_edge(const ConvexHullBuilder &builder, int32_t edge) {
    int32_t prev_edge = edge;
    while (edge_at(builder, prev_edge).next_edge != -1 && edge_at(builder, prev_edge).next_edge != edge) {
        prev_edge = edge_at(builder, prev_edge).next_edge;
    }
    return prev_edge;
}

/** deletes all faces in the hull */
static void free_faces(ConvexHullBuilder &builder) {
    // every slot goes back to the pools, keeping the conflict lists' storage
    builder.face_free_head = -1;
    for (int32_t i = (int32_t)builder.face_pool.length - 1; i >= 0; i--) {
        Face &face = face_at(builder, i);
        face.conflict_list.length = 0;
        face.first_edge = -1;
        face.removed = true;
        face.next_free = builder.face_free_head;
        builder.face_free_head = i;
    }
    builder.edge_free_head = -1;
    for (int32_t i = (int32_t)builder.edge_pool.length - 1; i >= 0; i--) free_edge(builder, i);

    builder.faces.length = 0;
    builder.coplanar_list.length = 0;
}

/**
 * formula as per: implementing quickhull - dirk gregorius.
 * returns a suitable tolerance for detecting that points are coplanar.
 */
static float determine_coplanar_distance(const ConvexHullBuilder &builder) {
    // find component-wise maximum absolute values
    Vec3f vmax = vec3f::zero();
    for (uint32_t i = 0; i < builder.num_positions; i++) vmax = vec3f::fmax(vmax, vec3f::abs(builder.positions[i]));
    return 3.0f * scalar::FLOAT_EPSILON * (vec3f::x(vmax) + vec3f::y(vmax) + vec3f::z(vmax));
}

/** finds the face for which the point is furthest to the front; -1 and 0 when it is behind them all. */
static void get_face_for_point(const ConvexHullBuilder &builder, Vec3f point, const int32_t *faces, uint32_t num_faces,
                               int32_t &out_face, float &out_dist_sq) {
    out_face = -1;
    out_dist_sq = 0.0f;

    for (uint32_t i = 0; i < num_faces; i++) {
        const Face &f = face_at(builder, faces[i]);
        if (f.removed) continue;

        // determine distance to face
        Vec3f delta = vec3f::subtract(point, f.centroid);
        const float dot = vec3f::dot(f.normal, delta);
        if (dot > 0.0f) {
            const float dist_sq = dot * dot / vec3f::squared_length(f.normal);
            if (dist_sq > out_dist_sq) {
                out_face = faces[i];
                out_dist_sq = dist_sq;
            }
        }
    }
}

/** squared distance from the point to the edges of the face: 0 if it is inside all of them, else to the closest one outside. */
static float get_distance_to_edge_sq(const ConvexHullBuilder &builder, Vec3f point, int32_t face_index) {
    const Face &face = face_at(builder, face_index);
    bool all_inside = true;
    float edge_dist_sq = scalar::FLOAT_MAX;

    // test if it is inside the edges of the polygon
    int32_t edge = face.first_edge;
    if (edge == -1) return 0.0f;

    const Vec3f *p1 = &builder.positions[edge_at(builder, get_previous_edge(builder, edge)).start_idx];
    do {
        const Vec3f &p2 = builder.positions[edge_at(builder, edge).start_idx];

        // check if point is outside this edge: (p2 - p1).cross(point - p1).dot(normal) < 0
        const Vec3f p2_minus_p1 = vec3f::subtract(p2, *p1);
        const Vec3f point_minus_p1 = vec3f::subtract(point, *p1);
        Vec3f cross = vec3f::cross(p2_minus_p1, point_minus_p1);
        if (vec3f::dot(cross, face.normal) < 0.0f) {
            // it is outside
            all_inside = false;

            // measure distance to this edge as jolt's GetClosestPointOnLine does
            Vec3f a = vec3f::subtract(*p1, point);
            Vec3f b = vec3f::subtract(p2, point);
            const Vec3f ab = vec3f::subtract(b, a);
            const float ab_len_sq = vec3f::squared_length(ab);

            float dist_sq;
            if (ab_len_sq < scalar::FLOAT_EPSILON * scalar::FLOAT_EPSILON) {
                // degenerate line segment, pick closer endpoint
                dist_sq = scalar::min(vec3f::squared_length(a), vec3f::squared_length(b));
            } else {
                const float v = -vec3f::dot(a, ab) / ab_len_sq;
                const float u = 1.0f - v;
                if (v <= 0.0f) {
                    // a is closest
                    dist_sq = vec3f::squared_length(a);
                } else if (u <= 0.0f) {
                    // b is closest
                    dist_sq = vec3f::squared_length(b);
                } else {
                    // closest point on segment: u * a + v * b
                    const Vec3f closest = vec3f::scale_and_add(vec3f::scale(a, u), b, v);
                    dist_sq = vec3f::squared_length(closest);
                }
            }

            edge_dist_sq = scalar::min(edge_dist_sq, dist_sq);
        }

        p1 = &p2;
        edge = edge_at(builder, edge).next_edge;
    } while (edge != -1 && edge != face.first_edge);

    return all_inside ? 0.0f : edge_dist_sq;
}

/**
 * assigns a position to the supplied face it is furthest in front of. true if it went on that
 * face's conflict list; a point within tolerance is discarded, or put on the coplanar list when it
 * is outside the face's edges by more than the tolerance.
 */
static bool assign_point_to_face(ConvexHullBuilder &builder, int32_t position_idx, const int32_t *faces, uint32_t num_faces,
                                 float tolerance_sq) {
    const Vec3f point = builder.positions[position_idx];

    // find the face for which the point is furthest away
    int32_t best_face;
    float best_dist_sq;
    get_face_for_point(builder, point, faces, num_faces, best_face, best_dist_sq);

    if (best_face != -1) {
        // check if this point is within the tolerance margin to the plane
        if (best_dist_sq <= tolerance_sq) {
            // check distance to edges
            const float dist_to_edge_sq = get_distance_to_edge_sq(builder, point, best_face);
            if (dist_to_edge_sq > tolerance_sq) {
                // point is outside of the face and too far away to discard
                builder.coplanar_list.push({position_idx, dist_to_edge_sq});
            }
        } else {
            // this point is in front of the face, add it to the conflict list
            Face &face = face_at(builder, best_face);
            collections::Vec<int32_t> &list = face.conflict_list;
            list.push(position_idx);
            if (best_dist_sq > face.furthest_point_distance_sq) {
                // this point is further away than any others, update the distance and keep the point last
                face.furthest_point_distance_sq = best_dist_sq;
            } else if (list.length >= 2) {
                // not the furthest point, move it to before the last point
                list[list.length - 1] = list[list.length - 2];
                list[list.length - 2] = position_idx;
            }
            return true;
        }
    }

    return false;
}

/** a new empty face, appended to the hull. */
static int32_t create_face(ConvexHullBuilder &builder) {
    int32_t index = builder.face_free_head;
    if (index != -1) {
        builder.face_free_head = face_at(builder, index).next_free;
    } else {
        Face fresh;
        util::zero(&fresh, sizeof(fresh));
        builder.face_pool.push(fresh);
        index = (int32_t)builder.face_pool.length - 1;
    }
    Face &face = face_at(builder, index);
    face.normal = vec3f::zero();
    face.centroid = vec3f::zero();
    face.conflict_list.length = 0;
    face.first_edge = -1;
    face.furthest_point_distance_sq = 0.0f;
    face.removed = false;
    face.next_free = -1;

    builder.faces.push(index);
    return index;
}

/** a new triangle face over three vertex indices, with its normal and centroid. */
static int32_t create_triangle(ConvexHullBuilder &builder, int32_t idx0, int32_t idx1, int32_t idx2) {
    const int32_t face = create_face(builder);

    // create 3 edges, linked in a circular list
    const int32_t e0 = create_edge(builder, face, idx0);
    const int32_t e1 = create_edge(builder, face, idx1);
    const int32_t e2 = create_edge(builder, face, idx2);
    edge_at(builder, e0).next_edge = e1;
    edge_at(builder, e1).next_edge = e2;
    edge_at(builder, e2).next_edge = e0;
    face_at(builder, face).first_edge = e0;

    calculate_normal_and_centroid(builder, face);
    return face;
}

/** links two edges as neighbours */
static void link_face(ConvexHullBuilder &builder, int32_t edge1, int32_t edge2) {
    // not connected yet, not on the same face, and the vertices match
    CC_ASSERT(edge_at(builder, edge1).neighbour_edge == -1 && edge_at(builder, edge2).neighbour_edge == -1);
    CC_ASSERT(edge_at(builder, edge1).face != edge_at(builder, edge2).face);
    CC_ASSERT(edge_at(builder, edge1).start_idx == edge_at(builder, edge_at(builder, edge2).next_edge).start_idx);
    CC_ASSERT(edge_at(builder, edge2).start_idx == edge_at(builder, edge_at(builder, edge1).next_edge).start_idx);

    edge_at(builder, edge1).neighbour_edge = edge2;
    edge_at(builder, edge2).neighbour_edge = edge1;
}

/** unlinks a face from all of its neighbouring faces */
static void unlink_face(ConvexHullBuilder &builder, int32_t face) {
    const int32_t first_edge = face_at(builder, face).first_edge;
    int32_t e = first_edge;
    if (e == -1) return;

    do {
        Edge &edge = edge_at(builder, e);
        if (edge.neighbour_edge != -1) {
            // unlink bidirectionally
            edge_at(builder, edge.neighbour_edge).neighbour_edge = -1;
            edge.neighbour_edge = -1;
        }
        e = edge.next_edge;
    } while (e != -1 && e != first_edge);
}

/** frees a removed face and its edges back to the pools */
static void free_face(ConvexHullBuilder &builder, int32_t face_index) {
    // unlink from all neighbours (should already be done, but be safe)
    unlink_face(builder, face_index);

    Face &face = face_at(builder, face_index);
    if (face.first_edge != -1) {
        int32_t e = face.first_edge;
        do {
            const int32_t next = edge_at(builder, e).next_edge;
            free_edge(builder, e);
            e = next;
        } while (e != -1 && e != face.first_edge);
        face.first_edge = -1;
    }

    face.conflict_list.length = 0;
    face.removed = true;
    face.next_free = builder.face_free_head;
    builder.face_free_head = face_index;
}

/** find_edge's own stack, instead of recursing */
struct StackEntry {
    int32_t first_edge;
    int32_t current_edge;
    bool visited;
};

/**
 * finds the horizon edges when adding a new vertex to the hull: marks every face the vertex sees
 * as removed, flooding out from facing_face, and returns the edges around them as a loop.
 */
static void find_edge(ConvexHullBuilder &builder, int32_t facing_face, Vec3f vertex, collections::Vec<FullEdge> &out_edges) {
    CC_ASSERT(out_edges.length == 0);

    // should start with a facing face
    CC_ASSERT(is_facing(face_at(builder, facing_face), vertex));

    // flag as removed
    face_at(builder, facing_face).removed = true;

    StackEntry stack[MAX_EDGE_LENGTH];
    int32_t cur_stack_pos = 0;

    // start with the face/edge provided
    const int32_t first_edge = face_at(builder, facing_face).first_edge;
    CC_ASSERT(first_edge != -1);
    stack[0] = {first_edge, first_edge, false};

    for (;;) {
        StackEntry &cur_entry = stack[cur_stack_pos];

        // get current edge and advance to next
        const int32_t e = cur_entry.current_edge;
        CC_ASSERT(edge_at(builder, e).next_edge != -1);
        cur_entry.current_edge = edge_at(builder, e).next_edge;

        // if we're back at the first edge we've completed the face and we're done
        if (e == cur_entry.first_edge && cur_entry.visited) {
            // this face needs to be removed, unlink it now, caller will free
            unlink_face(builder, edge_at(builder, e).face);

            // pop from stack
            cur_stack_pos--;
            if (cur_stack_pos < 0) break;
        } else {
            cur_entry.visited = true;

            // visit neighbour face
            const int32_t ne = edge_at(builder, e).neighbour_edge;
            if (ne != -1) {
                const int32_t n = edge_at(builder, ne).face;
                if (!face_at(builder, n).removed) {
                    // check if vertex is on the front side of this face
                    if (is_facing(face_at(builder, n), vertex)) {
                        // vertex on front, this face needs to be removed
                        face_at(builder, n).removed = true;

                        // add element to the stack of elements to visit
                        cur_stack_pos++;
                        CC_ASSERT(cur_stack_pos < MAX_EDGE_LENGTH);
                        CC_ASSERT(edge_at(builder, ne).next_edge != -1);

                        // we don't need to test this edge again since we came from it
                        stack[cur_stack_pos] = {ne, edge_at(builder, ne).next_edge, false};
                    } else {
                        // vertex behind, keep edge
                        out_edges.push({ne, edge_at(builder, e).start_idx, edge_at(builder, ne).start_idx});
                    }
                }
            }
        }
    }

    // validate that we have a fully connected loop
    for (uint32_t i = 0; i < out_edges.length; i++) {
        CC_ASSERT(out_edges[i].end_idx == out_edges[(i + 1) % out_edges.length].start_idx);
    }
}

/** adds a face to the affected faces list if not already present */
static void mark_affected(collections::Vec<int32_t> &affected_faces, int32_t face) {
    for (uint32_t i = 0; i < affected_faces.length; i++) {
        if (affected_faces[i] == face) return;
    }
    affected_faces.push(face);
}

/** removes a face that has only 2 edges left (degenerate) */
static bool remove_two_edge_face(ConvexHullBuilder &builder, int32_t face, collections::Vec<int32_t> &affected_faces) {
    // check if this face contains only 2 edges
    const int32_t edge = face_at(builder, face).first_edge;
    const int32_t next_edge = edge_at(builder, edge).next_edge;
    CC_ASSERT(edge != next_edge);  // 1 edge faces should not exist
    if (edge_at(builder, next_edge).next_edge == edge) {
        // schedule both neighbours for re-checking
        const int32_t neighbour_edge = edge_at(builder, edge).neighbour_edge;
        const int32_t next_neighbour_edge = edge_at(builder, next_edge).neighbour_edge;
        mark_affected(affected_faces, edge_at(builder, neighbour_edge).face);
        mark_affected(affected_faces, edge_at(builder, next_neighbour_edge).face);

        // link my neighbours to each other
        edge_at(builder, neighbour_edge).neighbour_edge = next_neighbour_edge;
        edge_at(builder, next_neighbour_edge).neighbour_edge = neighbour_edge;

        // unlink my edges
        edge_at(builder, edge).neighbour_edge = -1;
        edge_at(builder, next_edge).neighbour_edge = -1;

        // mark this face as removed
        face_at(builder, face).removed = true;
        return true;
    }

    return false;
}

/** removes edges that are invalid after face merging (self-loops and duplicate edges) */
static void remove_invalid_edges(ConvexHullBuilder &builder, int32_t face, collections::Vec<int32_t> &affected_faces) {
    // this marks that the plane needs to be recalculated (we delay this until the end of the
    // function since we don't use the plane and we want to avoid calculating it multiple times)
    bool recalculate_plane = false;

    // we keep going through this loop until no more edges were removed
    bool removed;
    do {
        removed = false;

        // loop over all edges in this face
        int32_t edge = face_at(builder, face).first_edge;
        int32_t neighbour_face = edge_at(builder, edge_at(builder, edge).neighbour_edge).face;
        do {
            const int32_t next_edge = edge_at(builder, edge).next_edge;
            const int32_t next_neighbour_face = edge_at(builder, edge_at(builder, next_edge).neighbour_edge).face;

            if (neighbour_face == face) {
                // we only remove 1 edge at a time, check if this edge's next edge is our neighbour.
                // if this check fails, we will continue to scan along the edge until we find an edge where this is the case.
                if (edge_at(builder, edge).neighbour_edge == next_edge) {
                    // this edge leads back to the starting point, this means the edge is interior and needs to be removed
                    const int32_t prev_edge = get_previous_edge(builder, edge);
                    edge_at(builder, prev_edge).next_edge = edge_at(builder, next_edge).next_edge;
                    if (face_at(builder, face).first_edge == edge || face_at(builder, face).first_edge == next_edge) {
                        face_at(builder, face).first_edge = prev_edge;
                    }
                    free_edge(builder, edge);
                    free_edge(builder, next_edge);

                    // check if face now has only 2 edges left
                    if (remove_two_edge_face(builder, face, affected_faces)) return;  // bail if face no longer exists

                    // restart the loop
                    recalculate_plane = true;
                    removed = true;
                    break;
                }
            } else if (neighbour_face == next_neighbour_face) {
                // there are two edges that connect to the same face, we will remove the second one

                // first merge the neighbours edges
                const int32_t neighbour_edge = edge_at(builder, next_edge).neighbour_edge;
                const int32_t next_neighbour_edge = edge_at(builder, neighbour_edge).next_edge;
                if (face_at(builder, neighbour_face).first_edge == next_neighbour_edge) {
                    face_at(builder, neighbour_face).first_edge = neighbour_edge;
                }
                edge_at(builder, neighbour_edge).next_edge = edge_at(builder, next_neighbour_edge).next_edge;
                edge_at(builder, neighbour_edge).neighbour_edge = edge;
                free_edge(builder, next_neighbour_edge);

                // then merge my own edges
                if (face_at(builder, face).first_edge == next_edge) face_at(builder, face).first_edge = edge;
                edge_at(builder, edge).next_edge = edge_at(builder, next_edge).next_edge;
                edge_at(builder, edge).neighbour_edge = neighbour_edge;
                free_edge(builder, next_edge);

                // check if neighbour has only 2 edges left
                if (!remove_two_edge_face(builder, neighbour_face, affected_faces)) {
                    // no, we need to recalculate its plane
                    calculate_normal_and_centroid(builder, neighbour_face);

                    // mark neighbour face as affected
                    mark_affected(affected_faces, neighbour_face);
                }

                // check if face now has only 2 edges left
                if (remove_two_edge_face(builder, face, affected_faces)) return;  // bail if face no longer exists

                // restart loop
                recalculate_plane = true;
                removed = true;
                break;
            }

            // this edge is ok, go to the next edge
            edge = next_edge;
            neighbour_face = next_neighbour_face;
        } while (edge != face_at(builder, face).first_edge);
    } while (removed);

    if (recalculate_plane) calculate_normal_and_centroid(builder, face);
}

/** merges two faces by removing a shared edge */
static void merge_faces(ConvexHullBuilder &builder, int32_t in_edge) {
    // get the face
    const int32_t face = edge_at(builder, in_edge).face;

    // find the previous and next edge
    const int32_t next_edge = edge_at(builder, in_edge).next_edge;
    const int32_t prev_edge = get_previous_edge(builder, in_edge);

    // get the other face
    const int32_t other_edge = edge_at(builder, in_edge).neighbour_edge;
    const int32_t other_face = edge_at(builder, other_edge).face;

    // cannot merge a face with itself
    CC_ASSERT(face != other_face);

    // loop over the edges of the other face and make them belong to face
    int32_t edge = edge_at(builder, other_edge).next_edge;
    edge_at(builder, prev_edge).next_edge = edge;
    for (;;) {
        edge_at(builder, edge).face = face;
        if (edge_at(builder, edge).next_edge == other_edge) {
            // terminate when we are back at other_edge
            edge_at(builder, edge).next_edge = next_edge;
            break;
        }
        edge = edge_at(builder, edge).next_edge;
    }

    // if the first edge happens to be in_edge we need to fix it because this edge is no longer part of the face.
    //
    // note that we replace it with the first edge of the merged face so that if the merge_faces function is called
    // from a loop that loops around the face that it will still terminate after visiting all edges once.
    if (face_at(builder, face).first_edge == in_edge) face_at(builder, face).first_edge = edge_at(builder, prev_edge).next_edge;

    free_edge(builder, in_edge);
    free_edge(builder, other_edge);

    // mark the other face as removed
    face_at(builder, other_face).first_edge = -1;
    face_at(builder, other_face).removed = true;

    // recalculate plane
    calculate_normal_and_centroid(builder, face);

    // merge conflict lists
    Face &merged = face_at(builder, face);
    Face &other = face_at(builder, other_face);
    collections::Vec<int32_t> &list = merged.conflict_list;
    if (merged.furthest_point_distance_sq > other.furthest_point_distance_sq) {
        // this face has a point that's further away, make sure it remains the last one as we add the other points to this faces list
        CC_ASSERT(list.length > 0);
        const int32_t last_point = list.data[--list.length];
        for (uint32_t i = 0; i < other.conflict_list.length; i++) list.push(other.conflict_list[i]);
        list.push(last_point);
    } else {
        // the other face has a point that's furthest away, add that list at the end
        for (uint32_t i = 0; i < other.conflict_list.length; i++) list.push(other.conflict_list[i]);
        merged.furthest_point_distance_sq = other.furthest_point_distance_sq;
    }
    other.conflict_list.length = 0;
}

/** merges a degenerate (very small area) face with its longest edge neighbour */
static void merge_degenerate_face(ConvexHullBuilder &builder, int32_t face, collections::Vec<int32_t> &affected_faces) {
    // check area of face
    if (vec3f::squared_length(face_at(builder, face).normal) < MIN_TRIANGLE_AREA_SQ) {
        // find longest edge, since this face is a sliver this should keep the face convex
        float max_length_sq = 0.0f;
        int32_t longest_edge = -1;
        const int32_t first_edge = face_at(builder, face).first_edge;
        int32_t e = first_edge;
        const Vec3f *p1 = &builder.positions[edge_at(builder, e).start_idx];
        do {
            const int32_t next = edge_at(builder, e).next_edge;
            const Vec3f &p2 = builder.positions[edge_at(builder, next).start_idx];
            const float length_sq = vec3f::squared_distance(p2, *p1);
            if (length_sq >= max_length_sq) {
                max_length_sq = length_sq;
                longest_edge = e;
            }
            p1 = &p2;
            e = next;
        } while (e != first_edge);

        // merge with face on longest edge
        merge_faces(builder, longest_edge);

        // remove any invalid edges
        remove_invalid_edges(builder, face, affected_faces);
    }
}

/** merges faces that are coplanar or concave with the current face */
static void merge_coplanar_or_concave_faces(ConvexHullBuilder &builder, int32_t face, float coplanar_tolerance_sq,
                                            collections::Vec<int32_t> &affected_faces) {
    bool merged = false;

    int32_t edge = face_at(builder, face).first_edge;
    do {
        // store next edge since this edge can be removed
        const int32_t next_edge = edge_at(builder, edge).next_edge;

        // test if centroid of one face is above plane of the other face by coplanar_tolerance_sq.
        // if so we need to merge other face into face.
        const Face &f = face_at(builder, face);
        const Face &other_face = face_at(builder, edge_at(builder, edge_at(builder, edge).neighbour_edge).face);
        const Vec3f delta_centroid = vec3f::subtract(other_face.centroid, f.centroid);
        const float dist_other_face_centroid = vec3f::dot(f.normal, delta_centroid);
        const float signed_dist_other_face_centroid_sq = scalar::abs(dist_other_face_centroid) * dist_other_face_centroid;
        const float dist_face_centroid = -vec3f::dot(other_face.normal, delta_centroid);
        const float signed_dist_face_centroid_sq = scalar::abs(dist_face_centroid) * dist_face_centroid;
        const float face_normal_len_sq = vec3f::squared_length(f.normal);
        const float other_face_normal_len_sq = vec3f::squared_length(other_face.normal);

        if ((signed_dist_other_face_centroid_sq > -coplanar_tolerance_sq * face_normal_len_sq ||
             signed_dist_face_centroid_sq > -coplanar_tolerance_sq * other_face_normal_len_sq) &&
            vec3f::dot(f.normal, other_face.normal) > 0.0f) {  // never merge faces that are back to back
            merge_faces(builder, edge);
            merged = true;
        }

        edge = next_edge;
    } while (edge != face_at(builder, face).first_edge);

    if (merged) remove_invalid_edges(builder, face, affected_faces);
}

/**
 * adds a new vertex to the hull: finds the horizon, fans new faces from it to the vertex into
 * out_new_faces, links them, then merges what became degenerate, coplanar or concave.
 */
static void add_point(ConvexHullBuilder &builder, int32_t facing_face, int32_t idx, float coplanar_tolerance_sq,
                      collections::Vec<int32_t> &out_new_faces) {
    // get position
    const Vec3f pos = builder.positions[idx];

    // find edge of convex hull of faces that are not facing the new vertex
    collections::Vec<FullEdge> &edges = builder.horizon;
    edges.length = 0;
    find_edge(builder, facing_face, pos, edges);
    CC_ASSERT(edges.length >= 3);

    // create new faces
    out_new_faces.length = 0;
    for (uint32_t i = 0; i < edges.length; i++) {
        CC_ASSERT(edges[i].start_idx != edges[i].end_idx);
        out_new_faces.push(create_triangle(builder, edges[i].start_idx, edges[i].end_idx, idx));
    }

    // link edges
    for (uint32_t i = 0; i < out_new_faces.length; i++) {
        const int32_t first_edge = face_at(builder, out_new_faces[i]).first_edge;

        // link to the horizon edge's neighbour
        if (edges[i].neighbour_edge != -1) link_face(builder, first_edge, edges[i].neighbour_edge);

        // link to next new face in the ring
        const int32_t next_first_edge = face_at(builder, out_new_faces[(i + 1) % out_new_faces.length]).first_edge;
        link_face(builder, edge_at(builder, first_edge).next_edge, edge_at(builder, edge_at(builder, next_first_edge).next_edge).next_edge);
    }

    // loop on faces that were modified until nothing needs to be checked anymore
    collections::Vec<int32_t> &affected_faces = builder.affected_faces;
    affected_faces.length = 0;
    for (uint32_t i = 0; i < out_new_faces.length; i++) affected_faces.push(out_new_faces[i]);
    while (affected_faces.length > 0) {
        // take the next face
        const int32_t face = affected_faces.data[--affected_faces.length];

        if (!face_at(builder, face).removed) {
            // merge with neighbour if this is a degenerate face
            merge_degenerate_face(builder, face, affected_faces);

            // merge with coplanar neighbours (or when the neighbour forms a concave edge)
            if (!face_at(builder, face).removed) merge_coplanar_or_concave_faces(builder, face, coplanar_tolerance_sq, affected_faces);
        }
    }
}

/** frees every face marked removed and drops it from the hull, keeping the order of the rest. */
static void garbage_collect_faces(ConvexHullBuilder &builder) {
    uint32_t kept = 0;
    for (uint32_t i = 0; i < builder.faces.length; i++) {
        const int32_t f = builder.faces[i];
        if (face_at(builder, f).removed) {
            free_face(builder, f);
        } else {
            builder.faces[kept++] = f;
        }
    }
    builder.faces.length = kept;
}

Result initialize(ConvexHullBuilder &builder, int32_t max_vertices, float tolerance) {
    // free the faces possibly left over from an earlier hull
    free_faces(builder);

    const Vec3f *positions = builder.positions;
    const int32_t num_positions = (int32_t)builder.num_positions;

    // need at least 3 points to make a hull
    if (num_positions < 3) return TOO_FEW_POINTS;

    // determine a suitable tolerance for detecting that points are coplanar
    const float coplanar_distance = determine_coplanar_distance(builder);
    const float coplanar_tolerance_sq = coplanar_distance * coplanar_distance;

    // increase desired tolerance if accuracy doesn't allow it
    const float tolerance_sq = scalar::max(coplanar_tolerance_sq, tolerance * tolerance);

    // find point furthest from the origin
    int32_t idx1 = -1;
    float max_dist_sq = -1.0f;
    for (int32_t i = 0; i < num_positions; i++) {
        const float dist_sq = vec3f::squared_length(positions[i]);
        if (dist_sq > max_dist_sq) {
            max_dist_sq = dist_sq;
            idx1 = i;
        }
    }

    // find point that is furthest away from this point
    int32_t idx2 = -1;
    max_dist_sq = -1.0f;
    for (int32_t i = 0; i < num_positions; i++) {
        if (i != idx1) {
            const float dist_sq = vec3f::squared_distance(positions[i], positions[idx1]);
            if (dist_sq > max_dist_sq) {
                max_dist_sq = dist_sq;
                idx2 = i;
            }
        }
    }

    // find point that forms the biggest triangle
    int32_t idx3 = -1;
    float best_triangle_area_sq = -1.0f;
    for (int32_t i = 0; i < num_positions; i++) {
        if (i != idx1 && i != idx2) {
            Vec3f a = vec3f::subtract(positions[idx1], positions[i]);
            Vec3f b = vec3f::subtract(positions[idx2], positions[i]);
            Vec3f cross = vec3f::cross(a, b);
            const float triangle_area_sq = vec3f::squared_length(cross);
            if (triangle_area_sq > best_triangle_area_sq) {
                best_triangle_area_sq = triangle_area_sq;
                idx3 = i;
            }
        }
    }

    // could not find a suitable initial triangle because its area was too small
    if (best_triangle_area_sq < MIN_TRIANGLE_AREA_SQ) return DEGENERATE;

    // check if we have only 3 vertices
    if (num_positions == 3) {
        // create two triangles (back to back)
        const int32_t t1 = create_triangle(builder, idx1, idx2, idx3);
        const int32_t t2 = create_triangle(builder, idx1, idx3, idx2);

        // link face edges
        const int32_t t1_e0 = face_at(builder, t1).first_edge;
        const int32_t t1_e1 = edge_at(builder, t1_e0).next_edge;
        const int32_t t1_e2 = edge_at(builder, t1_e1).next_edge;
        const int32_t t2_e0 = face_at(builder, t2).first_edge;
        const int32_t t2_e1 = edge_at(builder, t2_e0).next_edge;
        const int32_t t2_e2 = edge_at(builder, t2_e1).next_edge;
        link_face(builder, t1_e0, t2_e2);
        link_face(builder, t1_e1, t2_e1);
        link_face(builder, t1_e2, t2_e0);

        return SUCCESS;
    }

    // find point that forms the biggest tetrahedron
    Vec3f initial_plane_normal, initial_plane_centroid, edge1, edge2;
    edge1 = vec3f::subtract(positions[idx2], positions[idx1]);
    edge2 = vec3f::subtract(positions[idx3], positions[idx1]);
    initial_plane_normal = vec3f::cross(edge1, edge2);
    initial_plane_normal = vec3f::normalize(initial_plane_normal);

    initial_plane_centroid = vec3f::add(positions[idx1], positions[idx2]);
    initial_plane_centroid = vec3f::add(initial_plane_centroid, positions[idx3]);
    initial_plane_centroid = vec3f::scale(initial_plane_centroid, 1.0f / 3.0f);

    int32_t idx4 = -1;
    float max_dist = 0.0f;
    for (int32_t i = 0; i < num_positions; i++) {
        if (i != idx1 && i != idx2 && i != idx3) {
            const Vec3f diff = vec3f::subtract(positions[i], initial_plane_centroid);
            const float dist = vec3f::dot(diff, initial_plane_normal);
            if (scalar::abs(dist) > scalar::abs(max_dist)) {
                max_dist = dist;
                idx4 = i;
            }
        }
    }

    // check if the hull is coplanar
    if (max_dist * max_dist <= COPLANAR_SLOP_FACTOR * COPLANAR_SLOP_FACTOR * coplanar_tolerance_sq) {
        // first project all points in 2d space

        // find perpendicular by setting smallest component to 0 and swapping/negating others
        Vec3f base1, base2;
        const float abs_x = scalar::abs(vec3f::x(initial_plane_normal));
        const float abs_y = scalar::abs(vec3f::y(initial_plane_normal));
        const float abs_z = scalar::abs(vec3f::z(initial_plane_normal));
        if (abs_x <= abs_y && abs_x <= abs_z) {
            // x is smallest
            base1 = vec3f::make(0.0f, -vec3f::z(initial_plane_normal), vec3f::y(initial_plane_normal));
        } else if (abs_y <= abs_x && abs_y <= abs_z) {
            // y is smallest
            base1 = vec3f::make(-vec3f::z(initial_plane_normal), 0.0f, vec3f::x(initial_plane_normal));
        } else {
            // z is smallest
            base1 = vec3f::make(-vec3f::y(initial_plane_normal), vec3f::x(initial_plane_normal), 0.0f);
        }
        base1 = vec3f::normalize(base1);
        base2 = vec3f::cross(initial_plane_normal, base1);

        // project all points to 2d
        collections::Vec<Vec3f> &positions_2d = builder.positions_2d;
        positions_2d.length = 0;
        positions_2d.reserve((uint32_t)num_positions);
        for (int32_t i = 0; i < num_positions; i++) {
            positions_2d.push(vec3f::make(vec3f::dot(base1, positions[i]), vec3f::dot(base2, positions[i]), 0.0f));
        }

        // build hull
        collections::Vec<int32_t> edges_2d = {};
        convex_hull_builder_2d::ConvexHullBuilder2D builder_2d = convex_hull_builder_2d::create(positions_2d.data, positions_2d.length);
        const convex_hull_builder_2d::EResult result_2d =
            convex_hull_builder_2d::initialize(builder_2d, idx1, idx2, idx3, max_vertices, tolerance, edges_2d);
        convex_hull_builder_2d::release(builder_2d);
        const uint32_t n = edges_2d.length;

        // create faces (back to back)
        const int32_t f1 = create_face(builder);
        const int32_t f2 = create_face(builder);

        // create edges for face 1, and for face 2 in reversed order
        collections::Vec<int32_t> edges_f1 = {};
        collections::Vec<int32_t> edges_f2 = {};
        for (uint32_t i = 0; i < n; i++) edges_f1.push(create_edge(builder, f1, edges_2d[i]));
        for (uint32_t i = n; i-- > 0;) edges_f2.push(create_edge(builder, f2, edges_2d[i]));
        for (uint32_t i = 0; i < n; i++) {
            edge_at(builder, edges_f1[i]).next_edge = edges_f1[(i + 1) % n];
            edge_at(builder, edges_f2[i]).next_edge = edges_f2[(i + 1) % n];
        }
        face_at(builder, f1).first_edge = edges_f1[0];
        face_at(builder, f2).first_edge = edges_f2[0];

        // link edges
        for (uint32_t i = 0; i < n; i++) link_face(builder, edges_f1[i], edges_f2[(2 * n - 2 - i) % n]);

        // calculate the plane for both faces
        calculate_normal_and_centroid(builder, f1);
        face_at(builder, f2).normal = vec3f::negate(face_at(builder, f1).normal);
        face_at(builder, f2).centroid = face_at(builder, f1).centroid;

        edges_f1.release();
        edges_f2.release();
        edges_2d.release();
        return result_2d == convex_hull_builder_2d::MAX_VERTICES_REACHED ? MAX_VERTICES_REACHED : SUCCESS;
    }

    // ensure the planes are facing outwards
    if (max_dist < 0.0f) {
        const int32_t temp = idx2;
        idx2 = idx3;
        idx3 = temp;
    }

    // create tetrahedron
    const int32_t t1 = create_triangle(builder, idx1, idx2, idx4);
    const int32_t t2 = create_triangle(builder, idx2, idx3, idx4);
    const int32_t t3 = create_triangle(builder, idx3, idx1, idx4);
    const int32_t t4 = create_triangle(builder, idx1, idx3, idx2);

    // link face edges
    int32_t te[4][3];
    const int32_t tetrahedron[4] = {t1, t2, t3, t4};
    for (int32_t t = 0; t < 4; t++) {
        te[t][0] = face_at(builder, tetrahedron[t]).first_edge;
        te[t][1] = edge_at(builder, te[t][0]).next_edge;
        te[t][2] = edge_at(builder, te[t][1]).next_edge;
    }
    link_face(builder, te[0][0], te[3][2]);
    link_face(builder, te[0][1], te[1][2]);
    link_face(builder, te[0][2], te[2][1]);
    link_face(builder, te[1][0], te[3][1]);
    link_face(builder, te[1][1], te[2][2]);
    link_face(builder, te[2][0], te[3][0]);

    // build the initial conflict lists
    for (int32_t idx = 0; idx < num_positions; idx++) {
        if (idx != idx1 && idx != idx2 && idx != idx3 && idx != idx4) assign_point_to_face(builder, idx, tetrahedron, 4, tolerance_sq);
    }

    // overestimate of the actual amount of vertices we use, for limiting the amount of vertices in the hull
    int32_t num_vertices_used = 4;

    // loop through the remainder of the points and add them
    for (;;) {
        // find the face with the furthest point on it
        int32_t face_with_furthest_point = -1;
        float furthest_dist_sq = 0.0f;
        for (uint32_t i = 0; i < builder.faces.length; i++) {
            const Face &f = face_at(builder, builder.faces[i]);
            if (f.furthest_point_distance_sq > furthest_dist_sq) {
                furthest_dist_sq = f.furthest_point_distance_sq;
                face_with_furthest_point = builder.faces[i];
            }
        }

        int32_t furthest_point_idx;
        if (face_with_furthest_point != -1) {
            // take the furthest point
            collections::Vec<int32_t> &list = face_at(builder, face_with_furthest_point).conflict_list;
            CC_ASSERT(list.length > 0);
            furthest_point_idx = list.data[--list.length];
        } else if (builder.coplanar_list.length > 0) {
            // try to assign points to faces (this also recalculates the distance to the hull for the coplanar vertices)
            const collections::Vec<Coplanar> coplanar = builder.coplanar_list;
            builder.coplanar_list = builder.coplanar_scratch;
            builder.coplanar_list.length = 0;
            builder.coplanar_scratch = coplanar;
            bool added = false;
            for (uint32_t i = 0; i < coplanar.length; i++) {
                added = assign_point_to_face(builder, coplanar[i].position_idx, builder.faces.data, builder.faces.length, tolerance_sq) || added;
            }

            // if we were able to assign a point, loop again to pick it up
            if (added) continue;

            // if the coplanar list is empty, there are no points left and we're done
            if (builder.coplanar_list.length == 0) break;

            collections::Vec<Coplanar> &list = builder.coplanar_list;
            do {
                // find the vertex that is furthest from the hull
                uint32_t best_idx = 0;
                float best_dist_sq = list[0].distance_sq;
                for (uint32_t idx = 1; idx < list.length; idx++) {
                    if (list[idx].distance_sq > best_dist_sq) {
                        best_idx = idx;
                        best_dist_sq = list[idx].distance_sq;
                    }
                }

                // swap it to the end and remove it
                const Coplanar best = list[best_idx];
                list[best_idx] = list[list.length - 1];
                list.length--;
                furthest_point_idx = best.position_idx;

                // find the face for which the point is furthest away
                float dist_sq;
                get_face_for_point(builder, positions[furthest_point_idx], builder.faces.data, builder.faces.length,
                                   face_with_furthest_point, dist_sq);
            } while (list.length > 0 && face_with_furthest_point == -1);

            if (face_with_furthest_point == -1) break;
        } else {
            // if there are no more vertices, we're done
            break;
        }

        // check if we have a limit on the max vertices that we should produce
        if (num_vertices_used >= max_vertices) {
            // count the actual amount of used vertices (we did not take the removal of any vertices into account)
            num_vertices_used = get_num_vertices_used(builder);

            // check if there are too many
            if (num_vertices_used >= max_vertices) return MAX_VERTICES_REACHED;
        }

        // we're about to add another vertex
        ++num_vertices_used;

        // add the point to the hull
        collections::Vec<int32_t> &new_faces = builder.new_faces;
        add_point(builder, face_with_furthest_point, furthest_point_idx, coplanar_tolerance_sq, new_faces);

        // redistribute points on conflict lists belonging to removed faces
        for (uint32_t i = 0; i < builder.faces.length; i++) {
            const Face &face = face_at(builder, builder.faces[i]);
            if (face.removed) {
                for (uint32_t j = 0; j < face.conflict_list.length; j++) {
                    assign_point_to_face(builder, face.conflict_list[j], new_faces.data, new_faces.length, tolerance_sq);
                }
            }
        }

        // permanently delete faces that we removed in add_point()
        garbage_collect_faces(builder);
    }

    // check if we are left with a hull. it is possible that hull building fails if the points are nearly coplanar.
    if (builder.faces.length < 2) return TOO_FEW_FACES;

    return SUCCESS;
}

int32_t get_num_vertices_used(const ConvexHullBuilder &builder) {
    collections::Vec<uint8_t> used = {};
    used.reserve(builder.num_positions);
    used.length = builder.num_positions;
    util::zero(used.data, used.length);

    int32_t count = 0;
    for (uint32_t i = 0; i < builder.faces.length; i++) {
        const int32_t first_edge = face_at(builder, builder.faces[i]).first_edge;
        int32_t edge = first_edge;
        if (edge == -1) continue;
        do {
            const int32_t start_idx = edge_at(builder, edge).start_idx;
            if (!used[start_idx]) {
                used[start_idx] = 1;
                count++;
            }
            edge = edge_at(builder, edge).next_edge;
        } while (edge != first_edge);
    }

    used.release();
    return count;
}

bool contains_face(const ConvexHullBuilder &builder, const int32_t *indices, uint32_t num_indices) {
    // loop through all faces
    for (uint32_t i = 0; i < builder.faces.length; i++) {
        const int32_t first_edge = face_at(builder, builder.faces[i]).first_edge;
        if (first_edge == -1) continue;

        // find if any index from indices appears in this face's first edge
        int32_t first_index = -1;
        for (uint32_t k = 0; k < num_indices; k++) {
            if (indices[k] == edge_at(builder, first_edge).start_idx) {
                first_index = (int32_t)k;
                break;
            }
        }
        if (first_index != -1) {
            uint32_t matches = 0;
            int32_t edge = first_edge;
            uint32_t index_pos = (uint32_t)first_index;
            do {
                // check if index matches
                if (indices[index_pos] != edge_at(builder, edge).start_idx) break;

                // increment number of matches
                matches++;

                // next index in list of indices (circular)
                index_pos++;
                if (index_pos == num_indices) index_pos = 0;

                // next edge
                edge = edge_at(builder, edge).next_edge;
            } while (edge != -1 && edge != first_edge);

            if (matches == num_indices) return true;
        }
    }

    return false;
}

void get_center_of_mass_and_volume(const ConvexHullBuilder &builder, Vec3f &out_center_of_mass, float &out_volume) {
    const Vec3f *positions = builder.positions;

    // fourth point is the average of all face centroids
    Vec3f v4 = vec3f::make(0.0f, 0.0f, 0.0f);
    for (uint32_t i = 0; i < builder.faces.length; i++) v4 = vec3f::add(v4, face_at(builder, builder.faces[i]).centroid);
    v4 = vec3f::scale(v4, 1.0f / (float)builder.faces.length);

    // calculate mass and center of mass of this convex hull by summing all tetrahedrons
    float volume = 0.0f;
    Vec3f center_of_mass = vec3f::make(0.0f, 0.0f, 0.0f);

    for (uint32_t i = 0; i < builder.faces.length; i++) {
        const int32_t first_edge = face_at(builder, builder.faces[i]).first_edge;

        // get the first vertex that we'll use to create a triangle fan
        int32_t e = first_edge;
        if (e == -1) continue;
        const Vec3f v1 = positions[edge_at(builder, e).start_idx];

        // get the second vertex
        e = edge_at(builder, e).next_edge;
        if (e == -1 || e == first_edge) continue;
        const Vec3f *v2 = &positions[edge_at(builder, e).start_idx];

        for (e = edge_at(builder, e).next_edge; e != -1 && e != first_edge; e = edge_at(builder, e).next_edge) {
            // fetch the last point of the triangle
            const Vec3f &v3 = positions[edge_at(builder, e).start_idx];

            // calculate center of mass and mass of this tetrahedron,
            // see: https://en.wikipedia.org/wiki/Tetrahedron#Volume
            const Vec3f v1_minus_v4 = vec3f::subtract(v1, v4);
            const Vec3f v2_minus_v4 = vec3f::subtract(*v2, v4);
            const Vec3f v3_minus_v4 = vec3f::subtract(v3, v4);
            Vec3f cross = vec3f::cross(v2_minus_v4, v3_minus_v4);
            const float volume_tetrahedron = vec3f::dot(v1_minus_v4, cross);  // needs to be divided by 6, postpone this until the end

            // center_of_mass = v1 + v2 + v3 + v4 (needs to be divided by 4, postpone this until the end)
            Vec3f com = vec3f::add(v1, *v2);
            com = vec3f::add(com, v3);
            com = vec3f::add(com, v4);

            // accumulate results
            volume += volume_tetrahedron;
            center_of_mass = vec3f::scale_and_add(center_of_mass, com, volume_tetrahedron);

            // update v2 for next triangle
            v2 = &v3;
        }
    }

    // calculate center of mass, fall back to average point in case there is no volume (everything is on a plane in this case)
    if (volume > scalar::FLOAT_EPSILON) {
        out_center_of_mass = vec3f::scale(center_of_mass, 1.0f / (4.0f * volume));
    } else {
        out_center_of_mass = v4;
    }

    out_volume = volume / 6.0f;
}

MaxError &determine_max_error(MaxError &out, const ConvexHullBuilder &builder) {
    const float coplanar_distance = determine_coplanar_distance(builder);

    // this measures the distance from a polygon to the furthest point outside of the hull
    float max_error = 0.0f;
    int32_t max_error_face = -1;
    int32_t max_error_point = -1;

    for (uint32_t i = 0; i < builder.num_positions; i++) {
        const Vec3f v = builder.positions[i];

        // this measures the closest edge from all faces to point v
        //
        // note that we take the min of all faces since there may be multiple near coplanar faces so if we were to test this per face
        // we may find that a point is outside of a polygon and mark it as an error, while it is actually inside a nearly coplanar polygon.
        float min_edge_dist_sq = scalar::FLOAT_MAX;
        int32_t min_edge_dist_face = -1;

        for (uint32_t k = 0; k < builder.faces.length; k++) {
            const int32_t f = builder.faces[k];
            const Face &face = face_at(builder, f);

            // check if point is on or in front of plane
            const float normal_len = vec3f::length(face.normal);
            CC_ASSERT(normal_len > 0.0f);
            const Vec3f v_minus_centroid = vec3f::subtract(v, face.centroid);
            const float plane_dist = vec3f::dot(face.normal, v_minus_centroid) / normal_len;
            if (plane_dist > -COPLANAR_SLOP_FACTOR * coplanar_distance) {
                // check distance to the edges of this face
                const float edge_dist_sq = get_distance_to_edge_sq(builder, v, f);
                if (edge_dist_sq < min_edge_dist_sq) {
                    min_edge_dist_sq = edge_dist_sq;
                    min_edge_dist_face = f;
                }

                // if the point is inside the polygon and the point is in front of the plane, measure the distance
                if (edge_dist_sq == 0.0f && plane_dist > max_error) {
                    max_error = plane_dist;
                    max_error_face = f;
                    max_error_point = (int32_t)i;
                }
            }
        }

        // if the minimum distance to an edge is further than our current max error, we use that as max error
        const float min_edge_dist = scalar::sqrt(min_edge_dist_sq);
        if (min_edge_dist_face != -1 && min_edge_dist > max_error) {
            max_error = min_edge_dist;
            max_error_face = min_edge_dist_face;
            max_error_point = (int32_t)i;
        }
    }

    out.face_with_max_error = max_error_face;
    out.max_error = max_error;
    out.max_error_position_idx = max_error_point;
    out.coplanar_distance = coplanar_distance;
    return out;
}

}  // namespace convex_hull_builder

TUMBLE_NAMESPACE_END
