#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// about the y axis, rounded by a convex radius

struct CylinderShape : Shape {
    static constexpr ShapeType TYPE = ShapeType::CYLINDER;
    float half_height;
    float radius;
    float convex_radius;
    float density;
};

struct CylinderShapeSettings {
    float half_height;
    float radius;
    float convex_radius = support::DEFAULT_CONVEX_RADIUS;
    float density = shapes::DEFAULT_SHAPE_DENSITY;
    int32_t material_id = -1;
};

namespace cylinder {

/** the convex radius is clamped to the smaller of half height and radius */
Shape *create(const CylinderShapeSettings &settings);
/** after the fields changed: the convex radius clamped to fit, the bounds and volume recomputed */
void update(Shape *shape);

void compute_mass_properties(MassProperties &out, const Shape *shape);
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
/** a vertical edge of the side when `direction` is mostly horizontal, else the opposing cap as an octagon */
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);
/** uniform scale only */
void set_support(Support &out, const Shape *shape, SupportFunctionMode mode, Vec3f scale);

ShapeDef def();

}  // namespace cylinder

TUMBLE_NAMESPACE_END
