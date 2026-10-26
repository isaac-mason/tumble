#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// a segment along local y from -half_height_of_cylinder to +half_height_of_cylinder, rounded by radius

struct CapsuleShape : Shape {
    static constexpr ShapeType TYPE = ShapeType::CAPSULE;
    /** half height of the central cylinder, the hemispheres excluded */
    float half_height_of_cylinder;
    /** the radius, which is the convex radius */
    float radius;
    float density;
};

struct CapsuleShapeSettings {
    /** half height of the central cylinder, the hemispheres excluded */
    float half_height_of_cylinder;
    float radius;
    float density = shapes::DEFAULT_SHAPE_DENSITY;
    int32_t material_id = -1;
};

namespace capsule {

/** radius and half height must be > 0 */
Shape *create(const CapsuleShapeSettings &settings);
/** recomputes bounds and volume after the fields changed */
void update(Shape *shape);

float compute_capsule_volume(float half_height_of_cylinder, float radius);
void compute_capsule_local_bounds(Box3f &out, float half_height_of_cylinder, float radius);
Vec3f compute_capsule_center_of_mass();

void set_support(Support &out, const Shape *shape, SupportFunctionMode mode, Vec3f scale);
void compute_mass_properties(MassProperties &out, const Shape *shape);
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
float get_inner_radius(const Shape *shape);

ShapeDef def();

}  // namespace capsule

TUMBLE_NAMESPACE_END
