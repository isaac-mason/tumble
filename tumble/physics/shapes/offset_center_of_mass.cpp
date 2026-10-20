#include "tumble/physics/shapes/offset_center_of_mass.h"

#include "tumble/math/aabox.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace offset_center_of_mass {

Shape *create(const OffsetCenterOfMassShapeSettings &settings) {
    TUMBLE_ASSERT(settings.shape != nullptr);
    Shape *shape = shapes::allocate(shapes::SHAPE_OFFSET_CENTER_OF_MASS);
    if (shape == nullptr) return nullptr;
    shape->offset_center_of_mass.shape = settings.shape;
    shape->offset_center_of_mass.offset = settings.offset;
    shape->material_id = -1;

    update(shape);

    return shape;
}

void destroy(Shape *shape) { shapes::release(shape); }

static void compute_offset_center_of_mass_local_bounds(Box3f &out, const Shape *shape) {
    // the geometry stays at the local origin: only the centre of mass moves, so the bounds are the inner shape's
    out = shape->offset_center_of_mass.shape->aabb;
}

static Vec3f compute_offset_center_of_mass_center_of_mass(const Shape *shape) {
    // add offset to inner shape's center of mass
    return vec3f::add(vec3f::load(shape->offset_center_of_mass.shape->center_of_mass), vec3f::load(shape->offset_center_of_mass.offset));
}

void update(Shape *shape) {
    compute_offset_center_of_mass_local_bounds(shape->aabb, shape);
    vec3f::store(shape->center_of_mass, compute_offset_center_of_mass_center_of_mass(shape));
    shape->volume = shape->offset_center_of_mass.shape->volume;
}

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape) {
    mass_properties::MassProperties child_mass_properties;
    shapes::compute_mass_properties(child_mass_properties, shape->offset_center_of_mass.shape);

    // translate inertia by offset using the parallel axis theorem: from the inner shape's centre of mass to the new one
    mass_properties::translate(out, child_mass_properties, vec3f::load(shape->offset_center_of_mass.offset));
}

void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    // the geometry is at the origin, not centred on the centre of mass, so we just pass through
    const Shape *inner = shape->offset_center_of_mass.shape;
    shapes::def_of(inner).get_surface_normal(io, inner, sub_shape_id);
}

void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const Shape *inner = shape->offset_center_of_mass.shape;
    shapes::def_of(inner).get_supporting_face(io, direction, inner, sub_shape_id);
}

float get_inner_radius(const Shape *shape) {
    // inner radius doesn't change with the centre of mass offset
    return shapes::get_shape_inner_radius(shape->offset_center_of_mass.shape);
}

int32_t get_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    return shapes::get_shape_material(shape->offset_center_of_mass.shape, sub_shape_id);
}

void get_leaf_shape(shapes::GetLeafShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    // pass through to inner shape
    shapes::get_leaf_shape(out, shape->offset_center_of_mass.shape, sub_shape_id);
}

void get_sub_shape_transformed_shape(shapes::GetSubShapeTransformedShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    // the inner shape sits where this shape does
    shapes::get_sub_shape_transformed_shape(out, shape->offset_center_of_mass.shape, sub_shape_id);
}

/* cast ray */

void cast_ray_vs_offset_center_of_mass(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings,
                                       const cast_ray::Rayf &ray, const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id,
                                       const Transformf &transform) {
    // geometry doesn't move when the centre of mass changes: pass through
    const Shape *inner = shape->offset_center_of_mass.shape;
    shapes::def_of(inner).cast_ray(collector, settings, ray, inner, sub_shape_id, transform);
}

/* collide point */

void collide_point_vs_offset_center_of_mass(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                                            Vec3f point, const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id,
                                            const Transformf &transform) {
    // geometry doesn't move when the centre of mass changes: pass through
    const Shape *inner = shape->offset_center_of_mass.shape;
    shapes::def_of(inner).collide_point(collector, settings, point, inner, sub_shape_id, transform);
}

/* collide shape vs shape */

// a transform places a shape's origin (where its geometry is), not its centre of mass, so the dispatch is a pass through

void collide_offset_center_of_mass_vs_shape(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                            const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a,
                                            const Transformf &transform_a, const Shape *shape_b,
                                            const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    // dispatch to inner shape vs shape b
    const Shape *inner = shape_a->offset_center_of_mass.shape;
    const shapes::CollideShapeVsShapeFn fn = shapes::collide_fn(inner->type, shape_b->type);
    if (fn == nullptr) return;

    fn(collector, settings, inner, sub_shape_id_a, transform_a, shape_b, sub_shape_id_b, transform_b);
}

void collide_shape_vs_offset_center_of_mass(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                            const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a,
                                            const Transformf &transform_a, const Shape *shape_b,
                                            const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    // dispatch to shape a vs inner shape
    const Shape *inner = shape_b->offset_center_of_mass.shape;
    const shapes::CollideShapeVsShapeFn fn = shapes::collide_fn(shape_a->type, inner->type);
    if (fn == nullptr) return;

    fn(collector, settings, shape_a, sub_shape_id_a, transform_a, inner, sub_shape_id_b, transform_b);
}

/* cast shape vs shape */

void cast_offset_center_of_mass_vs_shape(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                         const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                         Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                         const Transformf &transform_b) {
    // cast inner shape vs shape b
    const Shape *inner = shape_a->offset_center_of_mass.shape;
    const shapes::CastShapeVsShapeFn fn = shapes::cast_fn(inner->type, shape_b->type);
    if (fn == nullptr) return;

    fn(collector, settings, inner, sub_shape_id_a, transform_a, displacement_a, shape_b, sub_shape_id_b, transform_b);
}

void cast_shape_vs_offset_center_of_mass(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                         const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                         Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                         const Transformf &transform_b) {
    // cast shape a vs inner shape
    const Shape *inner = shape_b->offset_center_of_mass.shape;
    const shapes::CastShapeVsShapeFn fn = shapes::cast_fn(shape_a->type, inner->type);
    if (fn == nullptr) return;

    fn(collector, settings, shape_a, sub_shape_id_a, transform_a, displacement_a, inner, sub_shape_id_b, transform_b);
}

shapes::ShapeDef def() {
    shapes::ShapeDef shape_def = {};
    shape_def.type = shapes::SHAPE_OFFSET_CENTER_OF_MASS;
    shape_def.compute_mass_properties = compute_mass_properties;
    shape_def.get_surface_normal = get_surface_normal;
    shape_def.get_supporting_face = get_supporting_face;
    shape_def.get_material = get_material;
    shape_def.get_inner_radius = get_inner_radius;
    shape_def.get_leaf_shape = get_leaf_shape;
    shape_def.get_sub_shape_transformed_shape = get_sub_shape_transformed_shape;
    shape_def.cast_ray = cast_ray_vs_offset_center_of_mass;
    shape_def.collide_point = collide_point_vs_offset_center_of_mass;
    return shapes::define_shape(shape_def);
}

}  // namespace offset_center_of_mass

TUMBLE_NAMESPACE_END
