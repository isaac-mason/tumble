#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/shapes/utils/convex_hull_builder.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// a hull built from a point cloud by quickhull, its faces polygons with outward planes, rounded by a convex radius

namespace convex_hull {

constexpr int32_t MAX_POINTS_IN_HULL = 256;
static_assert((uint32_t)MAX_POINTS_IN_HULL == support::MAX_HULL_POINTS, "a support's inline hull scratch holds every hull point");
constexpr uint32_t MAX_FACE_VERTICES = 32;

/** auto-bake the support hill-climb adjacency above this vertex count (below it, brute scan wins) */
constexpr uint32_t SUPPORT_HILL_CLIMB_MIN_POINTS = 32;

}
struct ConvexHullShape : Shape {
      // namespace convex_hull

    enum class BakeSupportAdjacency : uint32_t {
        /** bake when the hull has more than convex_hull::SUPPORT_HILL_CLIMB_MIN_POINTS vertices */
        AUTO = 0,
        NEVER = 1,
        ALWAYS = 2,
    };

    /** why build returned null; the builder's failures keep the builder's values. */
    enum class BuildResult : int32_t {
        SUCCESS = (uint32_t)ConvexHullBuilder::Result::SUCCESS,
        TOO_FEW_POINTS = (uint32_t)ConvexHullBuilder::Result::TOO_FEW_POINTS,
        TOO_FEW_FACES = (uint32_t)ConvexHullBuilder::Result::TOO_FEW_FACES,
        DEGENERATE = (uint32_t)ConvexHullBuilder::Result::DEGENERATE,
        /** a negative convex radius */
        INVALID_CONVEX_RADIUS = 5,
        /** a point lies outside the built hull by more than 4 * max(coplanar distance, hull tolerance) */
        HULL_ERROR_TOO_LARGE = 6,
        /** more than convex_hull::MAX_POINTS_IN_HULL vertices survived; a larger hull tolerance gives fewer */
        TOO_MANY_POINTS = 7,
        /** a hull vertex on fewer than 2 faces */
        POINT_WITH_TOO_FEW_FACES = 8,
    };

    struct Face {
        /** index of the face's first entry in vertex_indices */
        uint32_t first_vertex;
        uint32_t num_vertices;
    };

    /** the built hull: the shape's own, built by create and freed with the shape */
    struct Data {
        /** flat xyz, 3 per hull vertex, in the input's space (not relative to the centre of mass) */
        collections::Array<float> point_positions;
        /** the vertices shrunk by convex_radius, flat xyz in the same order; empty when convex_radius is 0 (the shrink is the identity) */
        collections::Array<float> shrunk_point_positions;
        /** neighbouring faces per vertex (1..3), used by the convex-radius shrink */
        collections::Array<uint32_t> point_num_faces;
        /** up to 3 neighbouring face indices per vertex, 3 per vertex, -1 unused */
        collections::Array<int32_t> point_faces;
        uint32_t num_points;
        collections::Array<Face> faces;
        /** one per face, the normal unit and outward */
        collections::Array<Plane3f> planes;
        /** every face's vertex ring, counter clockwise seen from outside, indexing the points */
        collections::Array<uint32_t> vertex_indices;
        /**
         * csr vertex 1-ring adjacency for support hill climbing, empty when not baked: vertex p's neighbours are
         * point_neighbors[point_neighbors_start[p] .. point_neighbors_start[p + 1]).
         */
        collections::Array<uint32_t> point_neighbors_start;
        collections::Array<uint32_t> point_neighbors;
        float convex_radius;
        /** about the centre of mass at unit density, column-major */
        Float4x4 inertia;
        /** local bounds of the points */
        Box3f aabb;
        Float3 center_of_mass;
        float volume;
    };

    static constexpr ShapeType TYPE = ShapeType::CONVEX_HULL;
    Data *data;
    float density;
};

struct ConvexHullShapeSettings {
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
    ConvexHullShape::BakeSupportAdjacency bake_support_adjacency = ConvexHullShape::BakeSupportAdjacency::AUTO;
    float density = shapes::DEFAULT_SHAPE_DENSITY;
    int32_t material_id = -1;
};

namespace convex_hull {

/**
 * a shape over the hull of the points, which it builds: the hull, then the convex radius shrunk until it fits the hull's
 * thickness and its sharpest vertex. null when the points make no hull; `out_result`, when
 * given, says why
 */
Shape *create(const ConvexHullShapeSettings &settings, ConvexHullShape::BuildResult *out_result = nullptr);
/** recomputes the bounds from the hull's points */
void update(Shape *shape);

void compute_mass_properties(MassProperties &out, const Shape *shape);
/** the normal of the plane nearest to the position */
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
/** the face whose outward normal is most opposed to `direction`, its winding reversed under an inside-out scale */
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
/** the distance from the centre of mass to the nearest plane, at least 0 */
float get_inner_radius(const Shape *shape);
void set_support(Support &out, const Shape *shape, SupportFunctionMode mode, Vec3f scale);

ShapeDef def();

}  // namespace convex_hull

TUMBLE_NAMESPACE_END
