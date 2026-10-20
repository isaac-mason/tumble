#pragma once

// crashcat's epa-convex-hull-builder.ts: the convex hull epa grows around the origin, after jolt's
// EPAConvexHullBuilder, with its triangle and edge records. trades accuracy for speed and gives up
// when the hull forms defects.

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace epa_convex_hull_builder {

constexpr int32_t EPA_MAX_TRIANGLES = 256;
/** max silhouette edges find_edge collects, and max depth of its flood-fill stack */
constexpr int32_t EPA_MAX_EDGE_LENGTH = 128;
constexpr float EPA_MIN_TRIANGLE_AREA = 1e-10f;
constexpr float EPA_BARYCENTRIC_EPSILON = 1e-3f;

/** flags bit: triangle has been removed from the hull */
constexpr int32_t TRI_FLAG_REMOVED = 1;
/** flags bit: triangle is currently in the priority queue */
constexpr int32_t TRI_FLAG_IN_QUEUE = 2;
/** flags bit: barycentric coordinates are relative to vertex 0 (else vertex 1) */
constexpr int32_t TRI_FLAG_LAMBDA_RELATIVE_TO_0 = 4;
/** flags bit: closest point to origin lies within the triangle interior */
constexpr int32_t TRI_FLAG_CLOSEST_POINT_INTERIOR = 8;

/** a triangle edge: the triangle across it (-1 for null), that triangle's edge index, and the edge's start vertex */
struct Edge {
    int32_t neighbour_triangle;
    int32_t neighbour_edge;
    int32_t start_index;
};

/** a hull triangle; a triangle "is" its index into the pool, -1 means null */
struct Triangle {
    Edge edges[3];
    Vec3f normal;
    Vec3f centroid;
    /** signed squared distance from the origin to the triangle plane */
    float closest_length_sq;
    /** barycentric coordinates of the closest point, relative to vertex 0 or 1 by the flags */
    float lambda[2];
    int32_t flags;
    /** the free-list link: next free triangle index, -1 if none */
    int32_t next_free;
};

/** one frame of find_edge's flood fill: a triangle, the edge it was entered by, and the edges visited so far */
struct StackFrame {
    int32_t triangle;
    int32_t edge;
    int32_t iteration;
};

/** a batch of triangle indices created by one add_point */
struct NewTriangles {
    int32_t values[EPA_MAX_EDGE_LENGTH];
    uint32_t size;
};

/**
 * the builder's whole state, fixed-capacity inline storage as jolt's StaticArrays: a zeroed (or reset) state is valid
 * and nothing is heap allocated.
 */
struct EpaConvexHullBuilderState {
    Triangle triangles[EPA_MAX_TRIANGLES];
    /** next never-used triangle index */
    int32_t triangle_high_watermark;
    /** head of the free list, -1 if empty */
    int32_t triangle_free_head;
    /** binary min-heap of triangle indices by closest_length_sq */
    int32_t queue[EPA_MAX_TRIANGLES];
    uint32_t queue_size;
    /** the support points the triangles index into, a borrow of the epa's y list */
    const Vec3f *positions;
    StackFrame stack[EPA_MAX_EDGE_LENGTH];
    /** silhouette edge list output by find_edge */
    Edge edges[EPA_MAX_EDGE_LENGTH];
    uint32_t edges_size;
};

/** frees every triangle and empties the queue and the edge list. */
void reset(EpaConvexHullBuilderState &state);
/** whether position is in front of triangle `tri`. */
bool triangle_is_facing(const EpaConvexHullBuilderState &state, int32_t tri, Vec3f position);
/** links edge1 of t1 with edge2 of t2, both ways. */
void link_triangle(EpaConvexHullBuilderState &state, int32_t t1, int32_t edge1, int32_t t2, int32_t edge2);
/** a new unlinked triangle over three position indices with its normal, centroid and closest point; -1 when the pool is full. */
int32_t create_triangle(EpaConvexHullBuilderState &state, int32_t idx1, int32_t idx2, int32_t idx3);
/** resets the hull to the two back-to-back triangles over three position indices, both queued. */
void initialize(EpaConvexHullBuilderState &state, int32_t idx1, int32_t idx2, int32_t idx3);
bool has_next_triangle(const EpaConvexHullBuilderState &state);
/** the queued triangle closest to the origin, -1 when the queue is empty. */
int32_t peek_closest_triangle_in_queue(const EpaConvexHullBuilderState &state);
/** removes and returns the queued triangle closest to the origin, -1 when the queue is empty. */
int32_t pop_closest_triangle_from_queue(EpaConvexHullBuilderState &state);
/** the facing triangle whose plane is furthest from position, -1 if none faces it; out_best_dist_sq its squared distance */
int32_t find_facing_triangle(const EpaConvexHullBuilderState &state, Vec3f position, float &out_best_dist_sq);
/** returns a removed, unlinked triangle to the pool. */
void free_triangle(EpaConvexHullBuilderState &state, int32_t tri);
/** detaches a triangle from its neighbours, freeing it now unless the queue still holds it. */
void unlink_triangle(EpaConvexHullBuilderState &state, int32_t tri);
/**
 * removes every triangle position sees, flooding out from `facing_triangle`, and leaves the silhouette loop in
 * `edges`. false when the seen region is not one connected cap.
 */
bool find_edge(EpaConvexHullBuilderState &state, int32_t facing_triangle, Vec3f position);
/**
 * adds position `idx` to the hull: removes what it sees from `facing_triangle` and fans new
 * triangles from the silhouette to it, appended to `out_triangles`. a new triangle is queued when
 * its closest point is interior and nearer than `closest_dist_sq`, or the origin is outside it.
 */
bool add_point(EpaConvexHullBuilderState &state, int32_t facing_triangle, int32_t idx, float closest_dist_sq,
               NewTriangles &out_triangles);
/** a live triangle is linked both ways on all three edges with matching vertices; a removed one is unlinked. */
bool validate_triangle(const EpaConvexHullBuilderState &state, int32_t tri);
/** validate_triangle over every triangle the pool has handed out since the last reset (freed ones are removed and unlinked). */
bool validate_triangles(const EpaConvexHullBuilderState &state);

}  // namespace epa_convex_hull_builder

TUMBLE_NAMESPACE_END
