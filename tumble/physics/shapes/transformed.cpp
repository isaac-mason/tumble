#include "tumble/physics/shapes/transformed.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/transform.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace transformed {

Shape *create(const TransformedShapeSettings &settings) {
    TUMBLE_ASSERT(settings.shape != nullptr);
    TransformedShape *shape = shapes::allocate<TransformedShape>();
    shape->shape = settings.shape;
    shape->position = settings.position;
    shape->quaternion = settings.quaternion;
    shape->material_id = -1;

    update(shape);

    return shape;
}

static void destroy(Shape *shape) {
    shapes::release<TransformedShape>(shape);
}

static float compute_transformed_volume(const Shape *shape) {
    // translation and rotation don't affect volume
    return shapes::as<TransformedShape>(shape).shape->volume;
}

static AABox compute_transformed_local_bounds(const Shape *shape) {
    const AABox child_aabb = shapes::as<TransformedShape>(shape).shape->aabb;
    const Quatf rotation = shapes::as<TransformedShape>(shape).quaternion;
    const Vec3f position = shapes::as<TransformedShape>(shape).position;

    // transform all 8 corners of the child aabb
    AABox bounds = aabox::empty();
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            for (int z = 0; z < 2; z++) {
                const Vec3f corner = vec3f::make(vec3f::x(x == 0 ? child_aabb.min : child_aabb.max), vec3f::y(y == 0 ? child_aabb.min : child_aabb.max),
                                                 vec3f::z(z == 0 ? child_aabb.min : child_aabb.max));

                // rotate and translate corner, then expand the bounds
                bounds = aabox::expand_by_point(bounds, vec3f::add(vec3f::transform_quat(corner, rotation), position));
            }
        }
    }
    return bounds;
}

static Vec3f compute_transformed_center_of_mass(const Shape *shape) {
    // the inner shape's centre of mass, rotated, then translated
    const Vec3f rotated = vec3f::transform_quat(shapes::as<TransformedShape>(shape).shape->center_of_mass, shapes::as<TransformedShape>(shape).quaternion);
    return vec3f::add(rotated, shapes::as<TransformedShape>(shape).position);
}

void update(Shape *shape) {
    shape->aabb = compute_transformed_local_bounds(shape);
    shape->center_of_mass = compute_transformed_center_of_mass(shape);
    shape->volume = compute_transformed_volume(shape);
}

/** the inner shape's world transform: the parent's scale applies to this shape's position, and reaches the inner shape rotated into its frame */
static void inner_transform(Transform &out, const Transform &parent, const Shape *shape) {
    Vec3f translation = vec3f::multiply(shapes::as<TransformedShape>(shape).position, parent.scale);
    translation = vec3f::transform_quat(translation, parent.quaternion);
    out.position = vec3f::add(parent.position, translation);
    out.quaternion = quatf::multiply(parent.quaternion, shapes::as<TransformedShape>(shape).quaternion);
    out.scale = shapes::transform_scale(shapes::as<TransformedShape>(shape).quaternion, parent.scale);
}

void compute_mass_properties(MassProperties &out, const Shape *shape) {
    // only rotates the inertia (translation doesn't affect inertia about the centre of mass)
    MassProperties child_mass_properties;
    shapes::compute_mass_properties(child_mass_properties, shapes::as<TransformedShape>(shape).shape);

    Mat4f rotation = mat4f::from_quat(shapes::as<TransformedShape>(shape).quaternion);

    mass_properties::rotate(out, child_mass_properties, rotation);
}

void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id) {
    // transform surface position to the inner shape's local space
    io.position = vec3f::subtract(io.position, shapes::as<TransformedShape>(shape).position);
    const Quatf inv_rotation = quatf::conjugate(shapes::as<TransformedShape>(shape).quaternion);
    io.position = vec3f::transform_quat(io.position, inv_rotation);

    // get normal from inner shape in its local space
    shapes::def_of(shapes::as<TransformedShape>(shape).shape).get_surface_normal(io, shapes::as<TransformedShape>(shape).shape, sub_shape_id);

    // the normal comes back in the inner shape's local space; this shape's rotation alone takes it out
    io.normal = vec3f::transform_quat(io.normal, shapes::as<TransformedShape>(shape).quaternion);
}

void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id) {
    // build the shape's local transform matrix. the transform carries no scale, so the position is scaled here
    const Vec3f scaled_position = vec3f::multiply(shapes::as<TransformedShape>(shape).position, io.scale);
    const Mat4f shape_matrix = mat4f::from_rotation_translation(shapes::as<TransformedShape>(shape).quaternion, scaled_position);

    // accumulate transform: transform = transform * shape matrix
    io.transform = mat4f::multiply(io.transform, shape_matrix);

    // transform direction to local space using the transposed rotation (inverse)
    const Vec3f local_direction = mat4f::multiply3x3_transposed_vec(shape_matrix, direction);

    // the scale as the inner shape sees it
    io.scale = shapes::transform_scale(shapes::as<TransformedShape>(shape).quaternion, io.scale);

    // compute face in local space; the sub shape id passes unchanged (decorator shapes don't consume bits)
    shapes::def_of(shapes::as<TransformedShape>(shape).shape).get_supporting_face(io, local_direction, shapes::as<TransformedShape>(shape).shape, sub_shape_id);
}

float get_inner_radius(const Shape *shape) { return shapes::get_shape_inner_radius(shapes::as<TransformedShape>(shape).shape); }

int32_t get_material(const Shape *shape, SubShapeId sub_shape_id) {
    return shapes::get_shape_material(shapes::as<TransformedShape>(shape).shape, sub_shape_id);
}

void get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, SubShapeId sub_shape_id) {
    // pass through to inner shape
    shapes::get_leaf_shape(out, shapes::as<TransformedShape>(shape).shape, sub_shape_id);
}

void get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, SubShapeId sub_shape_id) {
    // apply decorated transform: position += rotate(rotation, scale * shape position), rotation = rotation * shape rotation
    Vec3f shape_position = vec3f::multiply(shapes::as<TransformedShape>(shape).position, out.scale);
    shape_position = vec3f::transform_quat(shape_position, out.rotation);
    out.position = vec3f::add(out.position, shape_position);
    out.rotation = quatf::multiply(out.rotation, shapes::as<TransformedShape>(shape).quaternion);
    out.scale = shapes::transform_scale(shapes::as<TransformedShape>(shape).quaternion, out.scale);

    // pass through to inner shape
    shapes::get_sub_shape_transformed_shape(out, shapes::as<TransformedShape>(shape).shape, sub_shape_id);
}

/* cast ray */

void cast_ray_vs_transformed(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                             const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transform &transform) {
    // accumulate transform: world = parent * child
    Transform world;
    inner_transform(world, transform, shape);

    // transform the ray from world space to the inner shape's local space
    const Quatf inv_quat = quatf::conjugate(world.quaternion);

    Rayf local_ray;
    local_ray.origin = vec3f::subtract(ray.origin, world.position);
    local_ray.origin = vec3f::transform_quat(local_ray.origin, inv_quat);
    local_ray.direction = vec3f::transform_quat(ray.direction, inv_quat);
    local_ray.length = ray.length;

    // cast against the inner shape at the identity pose (already accumulated), with the scale it sees
    const Transform local = transform::make(vec3f::zero(), quatf::identity(), world.scale);
    shapes::def_of(shapes::as<TransformedShape>(shape).shape).cast_ray(collector, settings, local_ray, shapes::as<TransformedShape>(shape).shape, sub_shape_id, local);
}

/* collide point */

void collide_point_vs_transformed(CollidePointCollector &collector, const CollidePointSettings &settings,
                                  Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                  const Transform &transform_b) {
    // accumulate transform
    Transform world;
    inner_transform(world, transform_b, shape_b);

    shapes::def_of(shapes::as<TransformedShape>(shape_b).shape).collide_point(collector, settings, point, shapes::as<TransformedShape>(shape_b).shape, sub_shape_id_b, world);
}

/* collide shape */

void collide_transformed_vs_shape(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                  const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transform &transform_a,
                                  const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transform &transform_b) {
    // accumulate transform
    Transform world_a;
    inner_transform(world_a, transform_a, shape_a);

    const Shape *inner = shapes::as<TransformedShape>(shape_a).shape;
    const CollideShapeVsShapeFn fn = shapes::collide_fn(inner->type, shape_b->type);
    if (fn == nullptr) return;

    fn(collector, settings, inner, sub_shape_id_a, world_a, shape_b, sub_shape_id_b, transform_b);
}

void collide_shape_vs_transformed(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                  const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transform &transform_a,
                                  const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transform &transform_b) {
    // accumulate transform
    Transform world_b;
    inner_transform(world_b, transform_b, shape_b);

    const Shape *inner = shapes::as<TransformedShape>(shape_b).shape;
    const CollideShapeVsShapeFn fn = shapes::collide_fn(shape_a->type, inner->type);
    if (fn == nullptr) return;

    fn(collector, settings, shape_a, sub_shape_id_a, transform_a, inner, sub_shape_id_b, world_b);
}

/* cast shape */

void cast_transformed_vs_shape(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                               const SubShapeIdBuilder &sub_shape_id_a, const Transform &transform_a, Vec3f displacement_a,
                               const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transform &transform_b) {
    // accumulate transform
    Transform world_a;
    inner_transform(world_a, transform_a, shape_a);

    const Shape *inner = shapes::as<TransformedShape>(shape_a).shape;
    const CastShapeVsShapeFn fn = shapes::cast_fn(inner->type, shape_b->type);
    if (fn == nullptr) return;

    // the displacement is world space, the same for the inner shape
    fn(collector, settings, inner, sub_shape_id_a, world_a, displacement_a, shape_b, sub_shape_id_b, transform_b);
}

void cast_shape_vs_transformed(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                               const SubShapeIdBuilder &sub_shape_id_a, const Transform &transform_a, Vec3f displacement_a,
                               const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transform &transform_b) {
    // accumulate transform
    Transform world_b;
    inner_transform(world_b, transform_b, shape_b);

    const Shape *inner = shapes::as<TransformedShape>(shape_b).shape;
    const CastShapeVsShapeFn fn = shapes::cast_fn(shape_a->type, inner->type);
    if (fn == nullptr) return;

    fn(collector, settings, shape_a, sub_shape_id_a, transform_a, displacement_a, inner, sub_shape_id_b, world_b);
}

ShapeDef def() {
    ShapeDef shape_def = {};
    shape_def.type = ShapeType::TRANSFORMED;
    shape_def.destroy = destroy;
    shape_def.compute_mass_properties = compute_mass_properties;
    shape_def.get_surface_normal = get_surface_normal;
    shape_def.get_supporting_face = get_supporting_face;
    shape_def.get_material = get_material;
    shape_def.get_inner_radius = get_inner_radius;
    shape_def.get_leaf_shape = get_leaf_shape;
    shape_def.get_sub_shape_transformed_shape = get_sub_shape_transformed_shape;
    shape_def.cast_ray = cast_ray_vs_transformed;
    shape_def.collide_point = collide_point_vs_transformed;
    return shapes::define_shape(shape_def);
}

}  // namespace transformed

TUMBLE_NAMESPACE_END
