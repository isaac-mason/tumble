#pragma once

// crashcat's plane.ts: an infinite plane with a bounded representation for the broadphase. the negative
// half space (signed distance < 0) is solid. a plane has no mass and belongs on static bodies.

#include <stdint.h>

#include "tumble/physics/shapes/shapes.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace plane {

using shapes::Shape;

constexpr float DEFAULT_PLANE_HALF_EXTENT = 1000.0f;

struct PlaneShapeSettings {
    /** point . normal + constant = 0; the normal is normalised by create */
    Plane3f plane = {{0.0f, 1.0f, 0.0f}, 0.0f};
    /** half extent of the plane's bounded representation */
    float half_extent = DEFAULT_PLANE_HALF_EXTENT;
    int32_t material_id = -1;
};

/** null when the memory cannot grow */
Shape *create(const PlaneShapeSettings &settings);
void destroy(Shape *shape);
/** bounds and volume after the fields changed */
void update(Shape *shape);

/** no mass: a plane is static */
void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** the bounded quad, wound as the scale leaves it */
void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
int32_t get_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);

void cast_ray_vs_plane(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                       const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
void collide_point_vs_plane(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                            Vec3f point, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                            const Transformf &transform_b);

/** convex a against plane b: a's support point opposite the plane normal, in a's space */
void collide_convex_vs_plane(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                             const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                             const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** convex a swept by `displacement_a` against plane b, in the plane's space */
void cast_convex_vs_plane(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const Shape *shape_a,
                          const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                          const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

shapes::ShapeDef def();

}  // namespace plane

TUMBLE_NAMESPACE_END
