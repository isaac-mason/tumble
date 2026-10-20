#pragma once

// crashcat's sphere.ts: a sphere centred at the origin.

#include <stdint.h>

#include "physics/shapes/shapes.h"

namespace sphere {

using shapes::Shape;

struct SphereShapeSettings {
    float radius;
    float density = shapes::DEFAULT_SHAPE_DENSITY;
    int32_t material_id = -1;
};

Shape *create(const SphereShapeSettings &settings);
void destroy(Shape *shape);
/** recomputes bounds and volume after the fields changed */
void update(Shape *shape);

float compute_sphere_volume(float radius);
void compute_sphere_local_bounds(Box3f &out, float radius);

void set_support(support::Support &out, const Shape *shape, support::SupportFunctionMode mode, Vec3f scale);
void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);

void collide_point_vs_sphere(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                             Vec3f point, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                             const Transformf &transform_b);
/** sphere vs sphere by the centre distance, no gjk */
void collide_sphere_vs_sphere(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                              const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                              const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

shapes::ShapeDef def();

}  // namespace sphere
