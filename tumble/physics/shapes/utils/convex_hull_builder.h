#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// quickhull in 3d, merging coplanar and degenerate faces as it goes, so a face is a polygon, not only a triangle
namespace convex_hull_builder {

/** minimum triangle area squared */
constexpr float MIN_TRIANGLE_AREA_SQ = 1e-12f;

/** extra slop used to determine if the hull is coplanar / a point is on a face */
constexpr float COPLANAR_SLOP_FACTOR = 6.0f;

enum Result : int32_t {
    SUCCESS = 0,
    MAX_VERTICES_REACHED = 1,
    /** fewer than 3 points */
    TOO_FEW_POINTS = 2,
    /** the points were nearly coplanar and the hull collapsed */
    TOO_FEW_FACES = 3,
    /** no initial triangle with enough area: the points coincide or lie on a line */
    DEGENERATE = 4,
};

/**
 * a face of the hull, one slot of the face pool. its polygon is the circular list of edges from
 * first_edge (pool indices, -1 for null), counter clockwise seen from outside.
 */
struct Face {
    /** outward; its length is twice the face's area */
    Vec3f normal;
    Vec3f centroid;
    /** points in front of this face; the last is the furthest */
    collections::Vec<int32_t> conflict_list;
    int32_t first_edge;
    float furthest_point_distance_sq;
    bool removed;
    /** the next free slot while this one is in the pool's free list */
    int32_t next_free;
};

/** a half edge, one slot of the edge pool; a freed slot is chained through next_edge. */
struct Edge {
    /** the face this edge belongs to */
    int32_t face;
    int32_t next_edge;
    /** the same edge, reversed, on the neighbouring face */
    int32_t neighbour_edge;
    /** index into positions of the vertex this edge starts at */
    int32_t start_idx;
};

struct Coplanar {
    int32_t position_idx;
    float distance_sq;
};

/** a horizon edge found by find_edge: the edge from start_idx to end_idx, linked to neighbour_edge. */
struct FullEdge {
    int32_t neighbour_edge;
    int32_t start_idx;
    int32_t end_idx;
};

struct ConvexHullBuilder {
    /** borrowed */
    const Vec3f *positions;
    uint32_t num_positions;
    /** the hull: face pool indices, in creation order */
    collections::Vec<int32_t> faces;
    collections::Vec<Coplanar> coplanar_list;

    collections::Vec<Face> face_pool;
    int32_t face_free_head;
    collections::Vec<Edge> edge_pool;
    int32_t edge_free_head;

    // scratch kept between calls
    collections::Vec<Coplanar> coplanar_scratch;
    collections::Vec<FullEdge> horizon;
    collections::Vec<int32_t> new_faces;
    collections::Vec<int32_t> affected_faces;
    collections::Vec<Vec3f> positions_2d;
};

struct MaxError {
    /** face pool index, -1 if none */
    int32_t face_with_max_error;
    float max_error;
    /** -1 if none */
    int32_t max_error_position_idx;
    float coplanar_distance;
};

/** an empty builder over `positions`; its storage grows on first use. */
ConvexHullBuilder create(const Vec3f *positions, uint32_t num_positions);

void release(ConvexHullBuilder &builder);

/** builds the hull, at most max_vertices of them, discarding points within tolerance of it. */
Result initialize(ConvexHullBuilder &builder, int32_t max_vertices, float tolerance);

/** the number of distinct vertices the hull's faces use. */
int32_t get_num_vertices_used(const ConvexHullBuilder &builder);

/** whether the hull has a face whose vertices are `indices`, in this cyclic order. */
bool contains_face(const ConvexHullBuilder &builder, const int32_t *indices, uint32_t num_indices);

/**
 * center of mass and volume of the hull, summing tetrahedra from each face's triangle fan to the
 * average face centroid. with no volume the center is that average.
 */
void get_center_of_mass_and_volume(const ConvexHullBuilder &builder, Vec3f &out_center_of_mass, float &out_volume);

/** the point furthest outside the hull and the face it is furthest from; for checking a tolerance. */
MaxError &determine_max_error(MaxError &out, const ConvexHullBuilder &builder);

}  // namespace convex_hull_builder

TUMBLE_NAMESPACE_END
