#pragma once

// crashcat's capsule.ts: a segment along local y from -half_height_of_cylinder to
// +half_height_of_cylinder, rounded by radius.

#include <stdint.h>

#include "tumble/physics/shapes/shapes.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace capsule {

using shapes::Shape;

struct CapsuleShapeSettings {
    /** half height of the central cylinder, the hemispheres excluded */
    float half_height_of_cylinder;
    float radius;
    float density = shapes::DEFAULT_SHAPE_DENSITY;
    int32_t material_id = -1;
};

/** radius and half height must be > 0 */
Shape *create(const CapsuleShapeSettings &settings);
void destroy(Shape *shape);
/** recomputes bounds and volume after the fields changed */
void update(Shape *shape);

float compute_capsule_volume(float half_height_of_cylinder, float radius);
void compute_capsule_local_bounds(Box3f &out, float half_height_of_cylinder, float radius);
Vec3f compute_capsule_center_of_mass();

void set_support(support::Support &out, const Shape *shape, support::SupportFunctionMode mode, Vec3f scale);
void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);

shapes::ShapeDef def();

}  // namespace capsule

TUMBLE_NAMESPACE_END
