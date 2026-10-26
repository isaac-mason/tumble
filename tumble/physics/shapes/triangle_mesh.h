#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// a static mesh, its triangles built into a Data it owns; queries come into its unscaled space for the bvh
// and scale the triangles they test

struct TriangleMeshShape : Shape {
    /** bvh split strategies */
    enum class BvhSplitStrategy : uint32_t {
        /**
         * split at midpoint of center bounds.
         * - fast: O(n) with simple min/max computation
         * - good balance for most geometries
         */
        CENTER = 0,

        /**
         * split at arithmetic mean of centers.
         * - fast: O(n) iteration to compute mean
         * - can create unbalanced splits on skewed distributions
         */
        AVERAGE = 1,

        /**
         * surface area heuristic with binning.
         * - slow: O(n) with high constant factor (binning + cost evaluation)
         * - best quality: optimal expected query cost
         * - use for static meshes queried many times
         */
        SAH = 2,
    };

    struct Triangle;
    struct Data;

    static constexpr ShapeType TYPE = ShapeType::TRIANGLE_MESH;
    Data *data;
};

/** settings for building a triangle mesh */
struct TriangleMeshShapeSettings {
    /** flat input positions xyz; read during build only */
    const float *positions = nullptr;
    /** the number of vertices, a third of the floats in positions */
    uint32_t num_positions = 0;
    /** three vertex indices per triangle, counter clockwise seen from the front */
    const uint32_t *indices = nullptr;
    uint32_t num_indices = 0;
    /** optional per-triangle material ids, one per triangle (num_indices / 3); null gives -1 (no material) for all */
    const int32_t *material_ids = nullptr;
    /** a triangle is considered degenerate if |cross product| < this value */
    float degenerate_tolerance = 1e-6f;
    TriangleMeshShape::BvhSplitStrategy bvh_split_strategy = TriangleMeshShape::BvhSplitStrategy::CENTER;
    /** maximum triangles per leaf node in the bvh */
    uint32_t bvh_max_leaf_tris = 8;
    /**
     * cosine threshold for active edge determination: a convex edge whose cos(dihedral angle) is
     * below this is active. concave edges are never active; a negative value keeps every edge active.
     */
    float active_edge_cos_threshold_angle = 0.996195f;  // cos(5 degrees)
};

namespace triangle_mesh {

/** a shape over the mesh of the triangles, which it builds */
Shape *create(const TriangleMeshShapeSettings &settings);
/** the bounds from the bvh root */
void update(Shape *shape);

/** triangle meshes are static collision geometry: zero mass, unit inertia */
void compute_mass_properties(MassProperties &out, const Shape *shape);
/** the normal of the triangle the sub shape id names */
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
/** the triangle the sub shape id names, its winding reversed under an inside-out scale */
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
/** the triangle's */
int32_t get_material(const Shape *shape, SubShapeId sub_shape_id);

void cast_ray_vs_triangle_mesh(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                               const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform);

/** inside when a ray from the point up through the mesh crosses an odd number of triangles */
void collide_point_vs_triangle_mesh(CollidePointCollector &collector, const CollidePointSettings &settings,
                                    Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                    const Transformf &transform_b);

/** sweeps convex a against the mesh's triangles, front to back along the bvh */
void cast_convex_vs_triangle_mesh(CastShapeCollector &collector, const CastShapeSettings &settings,
                                  const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                  Vec3f displacement_a, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                  const Transformf &transform_b);

/** convex a against every triangle the bvh puts near it, gjk then epa, in a's local space */
void collide_convex_vs_triangle_mesh(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                     const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                     const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/** sphere a against the triangles by the closest point on each, no gjk */
void collide_sphere_vs_triangle_mesh(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                     const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                     const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/** sweeps sphere a against the triangles by direct tests: the start overlap, the face, then the edges and vertices */
void cast_sphere_vs_triangle_mesh(CastShapeCollector &collector, const CastShapeSettings &settings,
                                  const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                  Vec3f displacement_a, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                  const Transformf &transform_b);

ShapeDef def();

}  // namespace triangle_mesh

TUMBLE_NAMESPACE_END
