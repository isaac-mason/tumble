#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// quickhull in the xy plane, for the 3d builder when every point lies in one plane

struct ConvexHullBuilder2D {
    enum class Result : int32_t {
        SUCCESS = 0,
        MAX_VERTICES_REACHED = 1,
    };

    /** an edge of the hull, one slot of the edge pool; prev / next are pool indices, -1 for null. */
    struct Edge {
        Vec3f normal;
        Vec3f center;
        /** points in front of this edge; the last is the furthest */
        collections::Array<int32_t> conflict_list;
        int32_t prev_edge;
        int32_t next_edge;
        int32_t start_idx;
        float furthest_point_distance_sq;
    };

    /** borrowed; only x and y are used */
    const Vec3f *positions;
    uint32_t num_positions;
    /** pool index of an edge on the circular list, -1 when there is no hull */
    int32_t first_edge;
    int32_t num_edges;
    /** edge storage; a freed slot is chained through next_edge from edge_free_head */
    collections::Array<Edge> edge_pool;
    int32_t edge_free_head;
};

namespace convex_hull_builder_2d {

/** an empty builder over `positions`; its storage grows on first use. */
ConvexHullBuilder2D create(const Vec3f *positions, uint32_t num_positions);

/**
 * builds the hull starting from the triangle idx1, idx2, idx3 and writes its vertex indices,
 * counter clockwise, to `out_edges`. stops early with MAX_VERTICES_REACHED at max_vertices.
 */
ConvexHullBuilder2D::Result initialize(ConvexHullBuilder2D &builder, int32_t idx1, int32_t idx2, int32_t idx3, int32_t max_vertices, float tolerance,
                   collections::Array<int32_t> &out_edges);

/** every edge is linked both ways and the count matches num_edges. */
bool validate_edges(const ConvexHullBuilder2D &builder);

}  // namespace convex_hull_builder_2d

TUMBLE_NAMESPACE_END
