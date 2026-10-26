#include "tumble/physics/shapes/scaled.h"

#include "tumble/math/aabox.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace scaled {

Shape *create(const ScaledShapeSettings &settings) {
    TUMBLE_ASSERT(settings.shape != nullptr);
    ScaledShape *shape = shapes::allocate<ScaledShape>();
    shape->shape = settings.shape;
    shape->scale = settings.scale;
    shape->material_id = -1;
    update(shape);
    return shape;
}

static void destroy(Shape *shape) {
    shapes::release<ScaledShape>(shape);
}

static float compute_scaled_volume(const Shape *shape) {
    const Vec3f scale = vec3f::load(shapes::as<ScaledShape>(shape).scale);
    return shapes::as<ScaledShape>(shape).shape->volume * scalar::abs(vec3f::x(scale)) * scalar::abs(vec3f::y(scale)) * scalar::abs(vec3f::z(scale));
}

static void compute_scaled_local_bounds(Box3f &out, const Shape *shape) {
    // a negative scale mirrors the bounds; scale sorts each axis back into min <= max
    aabox::store(out, aabox::scale(aabox::load(shapes::as<ScaledShape>(shape).shape->aabb), vec3f::load(shapes::as<ScaledShape>(shape).scale)));
}

static Vec3f compute_scaled_center_of_mass(const Shape *shape) {
    return vec3f::multiply(vec3f::load(shapes::as<ScaledShape>(shape).shape->center_of_mass), vec3f::load(shapes::as<ScaledShape>(shape).scale));
}

void update(Shape *shape) {
    compute_scaled_local_bounds(shape->aabb, shape);
    vec3f::store(shape->center_of_mass, compute_scaled_center_of_mass(shape));
    shape->volume = compute_scaled_volume(shape);
}

void compute_mass_properties(MassProperties &out, const Shape *shape) {
    MassProperties child_mass_properties;
    shapes::compute_mass_properties(child_mass_properties, shapes::as<ScaledShape>(shape).shape);
    mass_properties::scale(out, child_mass_properties, vec3f::load(shapes::as<ScaledShape>(shape).scale));
}

void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id) {
    const Vec3f scale = vec3f::load(shapes::as<ScaledShape>(shape).scale);

    // accumulate scale
    io.scale = vec3f::multiply(io.scale, scale);

    // transform surface position to local space: divide by scale (inverse transform)
    io.position = vec3f::divide(io.position, scale);

    // get normal from inner shape
    shapes::def_of(shapes::as<ScaledShape>(shape).shape).get_surface_normal(io, shapes::as<ScaledShape>(shape).shape, sub_shape_id);

    // transform normal: divide by scale and renormalise, the (M^-1)^T transformation for plane normals
    io.normal = vec3f::divide(io.normal, scale);
    io.normal = vec3f::normalize(io.normal);
}

void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id) {
    // accumulate scale: scale = scale * shape scale
    io.scale = vec3f::multiply(io.scale, vec3f::load(shapes::as<ScaledShape>(shape).scale));

    // compute face in inner shape space; the sub shape id passes unchanged (decorator shapes don't consume bits)
    shapes::def_of(shapes::as<ScaledShape>(shape).shape).get_supporting_face(io, direction, shapes::as<ScaledShape>(shape).shape, sub_shape_id);
}

float get_inner_radius(const Shape *shape) {
    const Vec3f scale = vec3f::load(shapes::as<ScaledShape>(shape).scale);
    const float min_scale = scalar::min(scalar::abs(vec3f::x(scale)), scalar::min(scalar::abs(vec3f::y(scale)), scalar::abs(vec3f::z(scale))));
    return min_scale * shapes::get_shape_inner_radius(shapes::as<ScaledShape>(shape).shape);
}

int32_t get_material(const Shape *shape, SubShapeId sub_shape_id) {
    return shapes::get_shape_material(shapes::as<ScaledShape>(shape).shape, sub_shape_id);
}

void get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, SubShapeId sub_shape_id) {
    // pass through to inner shape
    shapes::get_leaf_shape(out, shapes::as<ScaledShape>(shape).shape, sub_shape_id);
}

void get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, SubShapeId sub_shape_id) {
    // apply scale
    out.scale = vec3f::multiply(out.scale, vec3f::load(shapes::as<ScaledShape>(shape).scale));

    // pass through to inner shape
    shapes::get_sub_shape_transformed_shape(out, shapes::as<ScaledShape>(shape).shape, sub_shape_id);
}

/** the transform with this shape's scale accumulated */
static Transformf scaled_transform(const Transformf &transform, const Shape *shape) {
    Transformf out = transform;
    vec3f::store(out.scale, vec3f::multiply(vec3f::load(transform.scale), vec3f::load(shapes::as<ScaledShape>(shape).scale)));
    return out;
}

/* cast ray */

void cast_ray_vs_scaled(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                        const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    // cast against the inner shape with the accumulated scale
    const Transformf inner_transform = scaled_transform(transform, shape);
    shapes::def_of(shapes::as<ScaledShape>(shape).shape).cast_ray(collector, settings, ray, shapes::as<ScaledShape>(shape).shape, sub_shape_id, inner_transform);
}

/* collide point */

void collide_point_vs_scaled(CollidePointCollector &collector, const CollidePointSettings &settings,
                             Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                             const Transformf &transform_b) {
    const Transformf inner_transform = scaled_transform(transform_b, shape_b);
    shapes::def_of(shapes::as<ScaledShape>(shape_b).shape).collide_point(collector, settings, point, shapes::as<ScaledShape>(shape_b).shape, sub_shape_id_b, inner_transform);
}

/* collide shape */

void collide_scaled_vs_shape(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                             const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                             const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const Shape *inner = shapes::as<ScaledShape>(shape_a).shape;
    const CollideShapeVsShapeFn fn = shapes::collide_fn(inner->type, shape_b->type);
    if (fn == nullptr) return;

    const Transformf inner_transform = scaled_transform(transform_a, shape_a);
    fn(collector, settings, inner, sub_shape_id_a, inner_transform, shape_b, sub_shape_id_b, transform_b);
}

void collide_shape_vs_scaled(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                             const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                             const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const Shape *inner = shapes::as<ScaledShape>(shape_b).shape;
    const CollideShapeVsShapeFn fn = shapes::collide_fn(shape_a->type, inner->type);
    if (fn == nullptr) return;

    const Transformf inner_transform = scaled_transform(transform_b, shape_b);
    fn(collector, settings, shape_a, sub_shape_id_a, transform_a, inner, sub_shape_id_b, inner_transform);
}

/* cast shape */

void cast_scaled_vs_shape(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                          const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                          const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const Shape *inner = shapes::as<ScaledShape>(shape_a).shape;
    const CastShapeVsShapeFn fn = shapes::cast_fn(inner->type, shape_b->type);
    if (fn == nullptr) return;

    const Transformf inner_transform = scaled_transform(transform_a, shape_a);
    fn(collector, settings, inner, sub_shape_id_a, inner_transform, displacement_a, shape_b, sub_shape_id_b, transform_b);
}

void cast_shape_vs_scaled(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                          const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                          const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const Shape *inner = shapes::as<ScaledShape>(shape_b).shape;
    const CastShapeVsShapeFn fn = shapes::cast_fn(shape_a->type, inner->type);
    if (fn == nullptr) return;

    const Transformf inner_transform = scaled_transform(transform_b, shape_b);
    fn(collector, settings, shape_a, sub_shape_id_a, transform_a, displacement_a, inner, sub_shape_id_b, inner_transform);
}

ShapeDef def() {
    ShapeDef shape_def = {};
    shape_def.type = ShapeType::SCALED;
    shape_def.destroy = destroy;
    shape_def.compute_mass_properties = compute_mass_properties;
    shape_def.get_surface_normal = get_surface_normal;
    shape_def.get_supporting_face = get_supporting_face;
    shape_def.get_material = get_material;
    shape_def.get_inner_radius = get_inner_radius;
    shape_def.get_leaf_shape = get_leaf_shape;
    shape_def.get_sub_shape_transformed_shape = get_sub_shape_transformed_shape;
    shape_def.cast_ray = cast_ray_vs_scaled;
    shape_def.collide_point = collide_point_vs_scaled;
    return shapes::define_shape(shape_def);
}

}  // namespace scaled

TUMBLE_NAMESPACE_END
