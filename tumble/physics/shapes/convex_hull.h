#pragma once

// crashcat's convex-hull.ts: a convex hull built from a point cloud by the quickhull builder, its
// faces polygons with outward planes, rounded by a convex radius.

#include <stdint.h>

#include "physics/shapes/shapes.h"
#include "physics/shapes/utils/convex_hull_builder.h"
#include "util/collections.h"

namespace convex_hull {

using shapes::Shape;

constexpr int32_t MAX_POINTS_IN_HULL = 256;
static_assert((uint32_t)MAX_POINTS_IN_HULL == support::MAX_HULL_POINTS, "a support's inline hull scratch holds every hull point");
constexpr uint32_t MAX_FACE_VERTICES = 32;

/** auto-bake the support hill-climb adjacency above this vertex count (below it, brute scan wins) */
constexpr uint32_t SUPPORT_HILL_CLIMB_MIN_POINTS = 32;

enum BakeSupportAdjacency : uint32_t {
    /** bake when the hull has more than SUPPORT_HILL_CLIMB_MIN_POINTS vertices */
    BAKE_SUPPORT_ADJACENCY_AUTO = 0,
    BAKE_SUPPORT_ADJACENCY_NEVER = 1,
    BAKE_SUPPORT_ADJACENCY_ALWAYS = 2,
};

/** why build returned null; the builder's failures keep the builder's values. */
enum BuildResult : int32_t {
    BUILD_SUCCESS = convex_hull_builder::SUCCESS,
    BUILD_TOO_FEW_POINTS = convex_hull_builder::TOO_FEW_POINTS,
    BUILD_TOO_FEW_FACES = convex_hull_builder::TOO_FEW_FACES,
    BUILD_DEGENERATE = convex_hull_builder::DEGENERATE,
    /** a negative convex radius */
    BUILD_INVALID_CONVEX_RADIUS = 5,
    /** a point lies outside the built hull by more than 4 * max(coplanar distance, hull tolerance) */
    BUILD_HULL_ERROR_TOO_LARGE = 6,
    /** more than MAX_POINTS_IN_HULL vertices survived; a larger hull tolerance gives fewer */
    BUILD_TOO_MANY_POINTS = 7,
    /** a hull vertex on fewer than 2 faces */
    BUILD_POINT_WITH_TOO_FEW_FACES = 8,
    BUILD_OUT_OF_MEMORY = 9,
};

struct ConvexHullFace {
    /** index of the face's first entry in vertex_indices */
    uint32_t first_vertex;
    uint32_t num_vertices;
};

/**
 * the built hull: the heavy part of a hull shape, js-owned and borrowed by any number of shapes (box3d's model: the
 * shape def is cloned, the data is not). built by convex_hull::build, freed by convex_hull::release after every
 * shape over it is destroyed.
 */
struct ConvexHullData {
    /** flat xyz, 3 per hull vertex, in the input's space (not relative to the centre of mass) */
    collections::Vec<float> point_positions;
    /** the vertices shrunk by convex_radius, flat xyz in the same order; empty when convex_radius is 0 (the shrink is the identity) */
    collections::Vec<float> shrunk_point_positions;
    /** neighbouring faces per vertex (1..3), used by the convex-radius shrink */
    collections::Vec<uint32_t> point_num_faces;
    /** up to 3 neighbouring face indices per vertex, 3 per vertex, -1 unused */
    collections::Vec<int32_t> point_faces;
    uint32_t num_points;
    collections::Vec<ConvexHullFace> faces;
    /** one per face, the normal unit and outward */
    collections::Vec<Plane3f> planes;
    /** every face's vertex ring, counter clockwise seen from outside, indexing the points */
    collections::Vec<uint32_t> vertex_indices;
    /**
     * csr vertex 1-ring adjacency for support hill climbing, empty when not baked: vertex p's neighbours are
     * point_neighbors[point_neighbors_start[p] .. point_neighbors_start[p + 1]).
     */
    collections::Vec<uint32_t> point_neighbors_start;
    collections::Vec<uint32_t> point_neighbors;
    float convex_radius;
    /** about the centre of mass at unit density, column-major */
    Float4x4 inertia;
    /** local bounds of the points */
    Box3f aabb;
    Float3 center_of_mass;
    float volume;
};

struct ConvexHullBuildSettings {
    /** flat input points xyz, interior points allowed; read during build only */
    const float *positions = nullptr;
    /** the number of points, a third of the floats in positions */
    uint32_t num_positions = 0;
    float convex_radius = support::DEFAULT_CONVEX_RADIUS;
    /**
     * points are allowed this far outside the hull; bigger gives a hull with fewer vertices. the used value can be
     * larger when the points are far apart
     */
    float hull_tolerance = 1e-3f;
    /** how far a vertex may shift at a sharp edge when the hull is shrunk by the convex radius */
    float max_error_convex_radius = 0.05f;
    BakeSupportAdjacency bake_support_adjacency = BAKE_SUPPORT_ADJACENCY_AUTO;
};

struct ConvexHullShapeSettings {
    /** a built hull the shape borrows; outlives the shape */
    ConvexHullData *data = nullptr;
    float density = shapes::DEFAULT_SHAPE_DENSITY;
    int32_t material_id = -1;
};

/**
 * builds the hull, then shrinks the convex radius until it fits the hull's thickness and its sharpest vertex. null when
 * the points make no hull (or the memory cannot grow); `out_result`, when given, says why.
 */
ConvexHullData *build(const ConvexHullBuildSettings &settings, BuildResult *out_result = nullptr);
/** frees a built hull; every shape over it is destroyed first */
void release(ConvexHullData *data);
/** a shape over a built hull; null when the memory cannot grow */
Shape *create(const ConvexHullShapeSettings &settings);
/** frees the shape, not the hull it borrows */
void destroy(Shape *shape);
/** recomputes the bounds from the hull's points */
void update(Shape *shape);

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
/** the normal of the plane nearest to the position */
void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** the face whose outward normal is most opposed to `direction`, its winding reversed under an inside-out scale */
void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** the distance from the centre of mass to the nearest plane, at least 0 */
float get_inner_radius(const Shape *shape);
/** crashcat's setHullSupport */
void set_support(support::Support &out, const Shape *shape, support::SupportFunctionMode mode, Vec3f scale);

shapes::ShapeDef def();

}  // namespace convex_hull
