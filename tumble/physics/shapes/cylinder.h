#pragma once

// crashcat's cylinder.ts: a cylinder about the y axis, rounded by a convex radius.

#include <stdint.h>

#include "physics/shapes/shapes.h"

namespace cylinder {

using shapes::Shape;

struct CylinderShapeSettings {
    float half_height;
    float radius;
    float convex_radius = support::DEFAULT_CONVEX_RADIUS;
    float density = shapes::DEFAULT_SHAPE_DENSITY;
    int32_t material_id = -1;
};

/** the convex radius is clamped to the smaller of half height and radius */
Shape *create(const CylinderShapeSettings &settings);
void destroy(Shape *shape);
/** recomputes bounds and volume after the fields changed */
void update(Shape *shape);

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** a vertical edge of the side when `direction` is mostly horizontal, else the opposing cap as an octagon */
void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);
/** crashcat's setCylinderSupport; uniform scale only */
void set_support(support::Support &out, const Shape *shape, support::SupportFunctionMode mode, Vec3f scale);

shapes::ShapeDef def();

}  // namespace cylinder
