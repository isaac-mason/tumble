#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// infinite, with a bounded box for the broadphase; the negative half space (signed distance < 0) is solid. it has no
// mass and belongs on static bodies

struct PlaneShape : Shape {
    static constexpr ShapeType TYPE = ShapeType::PLANE;
    Plane3f plane;
    /** the bounded extent the plane is treated as having */
    float half_extent;
};

namespace plane {

constexpr float DEFAULT_PLANE_HALF_EXTENT = 1000.0f;

}  // namespace plane

struct PlaneShapeSettings {
    /** point . normal + constant = 0; the normal is normalised by create */
    Plane3f plane = {{0.0f, 1.0f, 0.0f}, 0.0f};
    /** half extent of the plane's bounded representation */
    float half_extent = plane::DEFAULT_PLANE_HALF_EXTENT;
    int32_t material_id = -1;
};

namespace plane {

Shape *create(const PlaneShapeSettings &settings);
/** bounds and volume after the fields changed */
void update(Shape *shape);

/** no mass: a plane is static */
void compute_mass_properties(MassProperties &out, const Shape *shape);
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
/** the bounded quad, wound as the scale leaves it */
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
int32_t get_material(const Shape *shape, SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);

void cast_ray_vs_plane(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                       const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
void collide_point_vs_plane(CollidePointCollector &collector, const CollidePointSettings &settings,
                            Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                            const Transformf &transform_b);

/** convex a against plane b: a's support point opposite the plane normal, in a's space */
void collide_convex_vs_plane(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                             const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                             const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** convex a swept by `displacement_a` against plane b, in the plane's space */
void cast_convex_vs_plane(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                          const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                          const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

ShapeDef def();

}  // namespace plane

TUMBLE_NAMESPACE_END
