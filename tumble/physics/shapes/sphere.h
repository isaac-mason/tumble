#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// centred at the origin

struct SphereShape : Shape {
    static constexpr ShapeType TYPE = ShapeType::SPHERE;
    float radius;
    float density;
};

struct SphereShapeSettings {
    float radius;
    float density = shapes::DEFAULT_SHAPE_DENSITY;
    int32_t material_id = -1;
};

namespace sphere {

Shape *create(const SphereShapeSettings &settings);
/** recomputes bounds and volume after the fields changed */
void update(Shape *shape);

float compute_sphere_volume(float radius);
void compute_sphere_local_bounds(Box3f &out, float radius);

void set_support(Support &out, const Shape *shape, SupportFunctionMode mode, Vec3f scale);
void compute_mass_properties(MassProperties &out, const Shape *shape);
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);

void collide_point_vs_sphere(CollidePointCollector &collector, const CollidePointSettings &settings,
                             Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                             const Transformf &transform_b);
/** sphere vs sphere by the centre distance, no gjk */
void collide_sphere_vs_sphere(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                              const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                              const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

ShapeDef def();

}  // namespace sphere

TUMBLE_NAMESPACE_END
