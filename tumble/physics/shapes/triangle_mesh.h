#pragma once

#include <stdint.h>

#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/shapes/utils/triangle_mesh_data.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// a static mesh over a built TriangleMeshData; queries come into its unscaled space for the bvh and scale the triangles
// they test
namespace triangle_mesh {

using shapes::Shape;

struct TriangleMeshShapeSettings {
    /** a built mesh (triangle_mesh_builder::build) the shape borrows, outliving the shape */
    triangle_mesh_data::TriangleMeshData *data = nullptr;
    /** the shape's material; hits report the hit triangle's own */
    int32_t material_id = -1;
    /** the shape takes the mesh instead, and its destroy frees it; for a mesh no other shape uses */
    bool owns_data = false;
};

/** a shape over a built mesh; null when the allocator fails */
Shape *create(const TriangleMeshShapeSettings &settings);
/** frees the shape, and the mesh when it owns it */
void destroy(Shape *shape);
/** the bounds from the bvh root */
void update(Shape *shape);

/** triangle meshes are static collision geometry: zero mass, unit inertia */
void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
/** the normal of the triangle the sub shape id names */
void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** the triangle the sub shape id names, its winding reversed under an inside-out scale */
void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** the triangle's */
int32_t get_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id);

void cast_ray_vs_triangle_mesh(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                               const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform);

/** inside when a ray from the point up through the mesh crosses an odd number of triangles */
void collide_point_vs_triangle_mesh(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                                    Vec3f point, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                    const Transformf &transform_b);

/** sweeps convex a against the mesh's triangles, front to back along the bvh */
void cast_convex_vs_triangle_mesh(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                  const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                  Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                  const Transformf &transform_b);

/** convex a against every triangle the bvh puts near it, gjk then epa, in a's local space */
void collide_convex_vs_triangle_mesh(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                     const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                     const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/** sphere a against the triangles by the closest point on each, no gjk */
void collide_sphere_vs_triangle_mesh(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                     const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                     const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

/** sweeps sphere a against the triangles by direct tests: the start overlap, the face, then the edges and vertices */
void cast_sphere_vs_triangle_mesh(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                  const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                  Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                  const Transformf &transform_b);

shapes::ShapeDef def();

}  // namespace triangle_mesh

TUMBLE_NAMESPACE_END
