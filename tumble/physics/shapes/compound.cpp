#include "tumble/physics/shapes/compound.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/cast_utils.h"
#include "tumble/physics/shapes/static_compound.h"
#include "tumble/util/alloc.h"
#include "tumble/util/assert.h"
#include "tumble/util/mem.h"

TUMBLE_NAMESPACE_BEGIN

namespace compound {

Shape *create(const CompoundShapeSettings &settings) {
    TUMBLE_ASSERT(settings.num_children > 0);

    CompoundData *data = util::alloc<CompoundData>();
    if (data == nullptr) return nullptr;
    util::zero(data, sizeof(CompoundData));
    if (!data->children.reserve(settings.num_children) || !data->child_bounds.reserve(settings.num_children)) {
        data->children.release();
        data->child_bounds.release();
        util::free(data);
        return nullptr;
    }
    for (uint32_t i = 0; i < settings.num_children; i++) data->children.push(settings.children[i]);

    Shape *shape = shapes::allocate(shapes::SHAPE_COMPOUND);
    if (shape == nullptr) {
        data->children.release();
        data->child_bounds.release();
        util::free(data);
        return nullptr;
    }
    shape->compound.data = data;
    shape->material_id = -1;

    update(shape);

    return shape;
}

void destroy(Shape *shape) {
    CompoundData *data = shape->compound.data;
    data->children.release();
    data->child_bounds.release();
    util::free(data);
    shapes::release(shape);
}

const collections::Vec<CompoundShapeChild> &children_of(const Shape *shape) {
    TUMBLE_ASSERT(shape->type == shapes::SHAPE_COMPOUND || shape->type == shapes::SHAPE_STATIC_COMPOUND);
    return shape->type == shapes::SHAPE_COMPOUND ? shape->compound.data->children : shape->static_compound.data->children;
}

void child_transform(Transformf &out, const Transformf &parent, const CompoundShapeChild &child) {
    Vec3f translation = vec3f::multiply(vec3f::load(child.position), vec3f::load(parent.scale));
    translation = vec3f::transform_quat(translation, quatf::load(parent.quaternion));
    vec3f::store(out.position, vec3f::add(vec3f::load(parent.position), translation));
    quatf::store(out.quaternion, quatf::multiply(quatf::load(parent.quaternion), quatf::load(child.quaternion)));
    // the child sees the scale in its own space
    vec3f::store(out.scale, shapes::transform_scale(quatf::load(child.quaternion), vec3f::load(parent.scale)));
}

static float compute_compound_volume(const collections::Vec<CompoundShapeChild> &children) {
    // sum volumes of all children
    float total_volume = 0.0f;
    for (uint32_t i = 0; i < children.length; i++) total_volume += children[i].shape->volume;
    return total_volume;
}

static Box3f &compute_compound_local_bounds(Box3f &out, const collections::Vec<CompoundShapeChild> &children) {
    AABox bounds = aabox::empty();

    for (uint32_t i = 0; i < children.length; i++) {
        const CompoundShapeChild &child = children[i];
        const AABox child_aabb = aabox::load(child.shape->aabb);
        const Quatf rotation = quatf::load(child.quaternion);
        const Vec3f position = vec3f::load(child.position);

        // transform all 8 corners of the child aabb by the child's rotation and position
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                for (int z = 0; z < 2; z++) {
                    const Vec3f corner = vec3f::make(vec3f::x(x == 0 ? child_aabb.min : child_aabb.max), vec3f::y(y == 0 ? child_aabb.min : child_aabb.max),
                                                     vec3f::z(z == 0 ? child_aabb.min : child_aabb.max));
                    bounds = aabox::expand_by_point(bounds, vec3f::add(vec3f::transform_quat(corner, rotation), position));
                }
            }
        }
    }
    return aabox::store(out, bounds);
}

/** the child's centre of mass in the compound's space: position + rotate(quaternion, child centre of mass) */
static Vec3f child_center_of_mass(const CompoundShapeChild &child) {
    const Vec3f rotated = vec3f::transform_quat(vec3f::load(child.shape->center_of_mass), quatf::load(child.quaternion));
    return vec3f::add(rotated, vec3f::load(child.position));
}

static Vec3f compute_compound_center_of_mass(const collections::Vec<CompoundShapeChild> &children) {
    // weighted by child mass
    Vec3f out = vec3f::zero();
    float total_mass = 0.0f;

    mass_properties::MassProperties child_mass;
    for (uint32_t i = 0; i < children.length; i++) {
        const CompoundShapeChild &child = children[i];
        shapes::compute_mass_properties(child_mass, child.shape);

        out = vec3f::scale_and_add(out, child_center_of_mass(child), child_mass.mass);

        total_mass += child_mass.mass;
    }

    if (total_mass > 0.0f) out = vec3f::divide(out, vec3f::splat(total_mass));
    return out;
}

static void compute_compound_child_bounds(collections::Vec<Box3f> &out, const collections::Vec<CompoundShapeChild> &children) {
    TUMBLE_ASSERT(out.capacity >= children.length);
    out.length = children.length;
    for (uint32_t child_index = 0; child_index < children.length; child_index++) {
        const CompoundShapeChild &child = children[child_index];
        const Mat4f transform = mat4f::from_rotation_translation(quatf::load(child.quaternion), vec3f::load(child.position));
        aabox::store(out[child_index], aabox::transform_mat4(aabox::load(child.shape->aabb), transform));
    }
}

/**
 * bring a box from the compound's scaled local space into its unscaled space, where the child bounds are.
 * a negative scale component swaps that axis' min and max, so they are re-ordered.
 */
static AABox divide_box_by_scale(const AABox &box, Vec3f scale) {
    if (vec3f::x(scale) == 1.0f && vec3f::y(scale) == 1.0f && vec3f::z(scale) == 1.0f) return box;

    const V128 lo = v4f::div(box.min.v, scale.v);
    const V128 hi = v4f::div(box.max.v, scale.v);
    // lo < hi ? lo : hi, and lo < hi ? hi : lo, per lane
    const V128 lo_first = v4f::lt(lo, hi);
    return {{v4f::select(lo, hi, lo_first)}, {v4f::select(hi, lo, lo_first)}};
}

/** the bounds of a posed shape in the unscaled space of a posed compound, grown by `expand` on every side */
static AABox bounds_in_compound_space(const Shape *shape, const Transformf &transform, const Transformf &compound_transform, float expand) {
    Quatf inv_quat = quatf::conjugate(quatf::load(compound_transform.quaternion));
    Vec3f local_position = vec3f::subtract(vec3f::load(transform.position), vec3f::load(compound_transform.position));
    local_position = vec3f::transform_quat(local_position, inv_quat);
    const Quatf local_quaternion = quatf::multiply(inv_quat, quatf::load(transform.quaternion));

    const Mat4f aabb_transform = mat4f::from_rotation_translation_scale(local_quaternion, local_position, vec3f::load(transform.scale));
    const AABox bounds = aabox::expand_by_margin(aabox::transform_mat4(aabox::load(shape->aabb), aabb_transform), expand);
    return divide_box_by_scale(bounds, vec3f::load(compound_transform.scale));
}

void update(Shape *shape) {
    const collections::Vec<CompoundShapeChild> &children = children_of(shape);
    if (shape->type == shapes::SHAPE_COMPOUND) compute_compound_child_bounds(shape->compound.data->child_bounds, children);
    compute_compound_local_bounds(shape->aabb, children);
    vec3f::store(shape->center_of_mass, compute_compound_center_of_mass(children));
    shape->volume = compute_compound_volume(children);
}

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape) {
    const collections::Vec<CompoundShapeChild> &children = children_of(shape);
    out.mass = 0.0f;
    mat4f::store(out.inertia, mat4f::zero());

    if (children.length == 0) {
        out.inertia.m[15] = 1.0f;
        return;
    }

    // accumulate mass properties from all children with their transforms
    mass_properties::MassProperties child_mass;
    mass_properties::MassProperties rotated_inertia;
    Mat4f child_rotation;
    for (uint32_t i = 0; i < children.length; i++) {
        const CompoundShapeChild &child = children[i];
        shapes::compute_mass_properties(child_mass, child.shape);

        // child centre of mass relative to the compound's
        const Vec3f child_com_relative = vec3f::subtract(child_center_of_mass(child), vec3f::load(shape->center_of_mass));

        out.mass += child_mass.mass;

        // rotate the child's inertia into the compound's space, then the parallel axis theorem
        child_rotation = mat4f::from_quat(quatf::load(child.quaternion));
        mass_properties::rotate(rotated_inertia, child_mass, child_rotation);
        mass_properties::translate(rotated_inertia, rotated_inertia, child_com_relative);

        for (uint32_t j = 0; j < 15; j++) out.inertia.m[j] += rotated_inertia.inertia.m[j];
    }

    out.inertia.m[15] = 1.0f;
}

void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const collections::Vec<CompoundShapeChild> &children = children_of(shape);
    sub_shape::PopResult popped;
    sub_shape::pop_index(popped, sub_shape_id, children.length);

    if (popped.value >= children.length) {
        TUMBLE_ASSERT(false);
        return;
    }

    const CompoundShapeChild &child = children[popped.value];

    // transform surface position to the child's local space
    io.position = vec3f::subtract(io.position, vec3f::load(child.position));
    const Quatf inv_rotation = quatf::conjugate(quatf::load(child.quaternion));
    io.position = vec3f::transform_quat(io.position, inv_rotation);

    // get normal from child shape in its local space
    shapes::def_of(child.shape).get_surface_normal(io, child.shape, popped.remainder);

    // the normal comes back in the child's local space; this child's rotation alone takes it to the compound's
    io.normal = vec3f::transform_quat(io.normal, quatf::load(child.quaternion));
}

void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const collections::Vec<CompoundShapeChild> &children = children_of(shape);
    sub_shape::PopResult popped;
    sub_shape::pop_index(popped, sub_shape_id, children.length);

    if (popped.value >= children.length) {
        TUMBLE_ASSERT(false);
        return;
    }

    const CompoundShapeChild &child = children[popped.value];

    // accumulate the child's transform: transform = transform * child transform. the transform carries
    // no scale, so the child's position is scaled here
    const Vec3f scaled_position = vec3f::multiply(vec3f::load(child.position), io.scale);
    const Mat4f child_transform_matrix = mat4f::from_rotation_translation(quatf::load(child.quaternion), scaled_position);
    io.transform = mat4f::multiply(io.transform, child_transform_matrix);

    // direction into the child's space by the inverse (transposed) rotation
    Vec3f local_direction = mat4f::multiply3x3_transposed_vec(child_transform_matrix, direction);

    // the child sees the scale in its own space
    io.scale = shapes::transform_scale(quatf::load(child.quaternion), io.scale);

    shapes::def_of(child.shape).get_supporting_face(io, local_direction, child.shape, popped.remainder);
}

float get_inner_radius(const Shape *shape) {
    const collections::Vec<CompoundShapeChild> &children = children_of(shape);
    float inner_radius = scalar::FLOAT_MAX;
    for (uint32_t i = 0; i < children.length; i++) {
        inner_radius = scalar::min(inner_radius, shapes::get_shape_inner_radius(children[i].shape));
    }
    return inner_radius;
}

int32_t get_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const collections::Vec<CompoundShapeChild> &children = children_of(shape);
    sub_shape::PopResult popped;
    sub_shape::pop_index(popped, sub_shape_id, children.length);
    TUMBLE_ASSERT(popped.value < children.length);
    return shapes::get_shape_material(children[popped.value].shape, popped.remainder);
}

void get_leaf_shape(shapes::GetLeafShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const collections::Vec<CompoundShapeChild> &children = children_of(shape);
    sub_shape::PopResult popped;
    sub_shape::pop_index(popped, sub_shape_id, children.length);
    if (popped.value >= children.length) {
        // no longer a valid index
        out.shape = nullptr;
        out.remainder = sub_shape::EMPTY_SUB_SHAPE_ID;
        return;
    }

    const Shape *child_shape = children[popped.value].shape;

    shapes::get_leaf_shape(out, child_shape, popped.remainder);
}

void get_sub_shape_transformed_shape(shapes::GetSubShapeTransformedShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const collections::Vec<CompoundShapeChild> &children = children_of(shape);
    sub_shape::PopResult popped;
    sub_shape::pop_index(popped, sub_shape_id, children.length);
    TUMBLE_ASSERT(popped.value < children.length);
    const CompoundShapeChild &child = children[popped.value];

    // apply the child's transform: position += rotate(rotation, scale * child position), rotation = rotation * child rotation
    Vec3f child_position = vec3f::multiply(vec3f::load(child.position), out.scale);
    child_position = vec3f::transform_quat(child_position, out.rotation);
    out.position = vec3f::add(out.position, child_position);
    out.rotation = quatf::multiply(out.rotation, quatf::load(child.quaternion));
    out.scale = shapes::transform_scale(quatf::load(child.quaternion), out.scale);

    shapes::get_sub_shape_transformed_shape(out, child.shape, popped.remainder);
}

/* cast ray */

void cast_ray_vs_compound(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                          const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    const collections::Vec<CompoundShapeChild> &children = shape->compound.data->children;
    const collections::Vec<Box3f> &child_bounds = shape->compound.data->child_bounds;

    // the ray in the compound's unscaled space, where the child bounds are
    Quatf inv_quat = quatf::conjugate(quatf::load(transform.quaternion));
    Vec3f local_origin = vec3f::subtract(ray.origin, vec3f::load(transform.position));
    local_origin = vec3f::transform_quat(local_origin, inv_quat);
    local_origin = vec3f::divide(local_origin, vec3f::load(transform.scale));
    Vec3f local_direction = vec3f::transform_quat(ray.direction, inv_quat);
    local_direction = vec3f::divide(local_direction, vec3f::load(transform.scale));
    const Vec3f inv_displacement = vec3f::make(cast_utils::safe_reciprocal(vec3f::x(local_direction) * ray.length),
                                               cast_utils::safe_reciprocal(vec3f::y(local_direction) * ray.length),
                                               cast_utils::safe_reciprocal(vec3f::z(local_direction) * ray.length));

    for (uint32_t child_index = 0; child_index < children.length; child_index++) {
        if (collector.early_out_fraction <= 0.0f) break;

        // skip children the ray cannot reach before the closest hit so far
        if (cast_utils::ray_fraction_to_box3(local_origin, inv_displacement, aabox::load(child_bounds[child_index])) >= collector.early_out_fraction) continue;

        const CompoundShapeChild &child = children[child_index];

        sub_shape::SubShapeIdBuilder child_sub_shape_id;
        sub_shape::push_index(child_sub_shape_id, sub_shape_id, child_index, children.length);

        Transformf child_world;
        child_transform(child_world, transform, child);

        shapes::def_of(child.shape).cast_ray(collector, settings, ray, child.shape, child_sub_shape_id, child_world);
    }
}

/* collide point */

void collide_point_vs_compound(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                               Vec3f point, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                               const Transformf &transform_b) {
    const collections::Vec<CompoundShapeChild> &children = shape_b->compound.data->children;
    const collections::Vec<Box3f> &child_bounds = shape_b->compound.data->child_bounds;

    // the point in the compound's unscaled space, where the child bounds are
    Quatf inv_quat = quatf::conjugate(quatf::load(transform_b.quaternion));
    Vec3f local_point = vec3f::subtract(point, vec3f::load(transform_b.position));
    local_point = vec3f::transform_quat(local_point, inv_quat);
    local_point = vec3f::divide(local_point, vec3f::load(transform_b.scale));

    for (uint32_t child_index = 0; child_index < children.length; child_index++) {
        if (collide_point::should_early_out(collector)) break;

        // skip children whose bounds do not hold the point
        if (!aabox::contains_point(aabox::load(child_bounds[child_index]), local_point)) continue;

        const CompoundShapeChild &child = children[child_index];

        sub_shape::SubShapeIdBuilder child_sub_shape_id;
        sub_shape::push_index(child_sub_shape_id, sub_shape_id_b, child_index, children.length);

        Transformf child_world;
        child_transform(child_world, transform_b, child);

        shapes::def_of(child.shape).collide_point(collector, settings, point, child.shape, child_sub_shape_id, child_world);
    }
}

/* collide shape */

void collide_compound_vs_shape(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                               const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                               const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const collections::Vec<CompoundShapeChild> &children = shape_a->compound.data->children;
    const collections::Vec<Box3f> &child_bounds = shape_a->compound.data->child_bounds;

    // b's bounds in a's unscaled space
    const AABox query_bounds = bounds_in_compound_space(shape_b, transform_b, transform_a, settings.max_separation_distance);

    for (uint32_t child_index = 0; child_index < children.length; child_index++) {
        if (collide_shape::should_early_out(collector)) break;

        // skip children whose bounds the other shape's bounds miss
        if (!aabox::intersects(aabox::load(child_bounds[child_index]), query_bounds)) continue;

        const CompoundShapeChild &child = children[child_index];

        sub_shape::SubShapeIdBuilder child_sub_shape_id;
        sub_shape::push_index(child_sub_shape_id, sub_shape_id_a, child_index, children.length);

        Transformf child_world;
        child_transform(child_world, transform_a, child);

        const shapes::CollideShapeVsShapeFn fn = shapes::collide_fn(child.shape->type, shape_b->type);
        if (fn != nullptr) fn(collector, settings, child.shape, child_sub_shape_id, child_world, shape_b, sub_shape_id_b, transform_b);
    }
}

void collide_shape_vs_compound(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                               const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                               const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const collections::Vec<CompoundShapeChild> &children = shape_b->compound.data->children;
    const collections::Vec<Box3f> &child_bounds = shape_b->compound.data->child_bounds;

    // a's bounds in b's unscaled space
    const AABox query_bounds = bounds_in_compound_space(shape_a, transform_a, transform_b, settings.max_separation_distance);

    for (uint32_t child_index = 0; child_index < children.length; child_index++) {
        if (collide_shape::should_early_out(collector)) break;

        // skip children whose bounds the other shape's bounds miss
        if (!aabox::intersects(aabox::load(child_bounds[child_index]), query_bounds)) continue;

        const CompoundShapeChild &child = children[child_index];

        sub_shape::SubShapeIdBuilder child_sub_shape_id;
        sub_shape::push_index(child_sub_shape_id, sub_shape_id_b, child_index, children.length);

        Transformf child_world;
        child_transform(child_world, transform_b, child);

        const shapes::CollideShapeVsShapeFn fn = shapes::collide_fn(shape_a->type, child.shape->type);
        if (fn != nullptr) fn(collector, settings, shape_a, sub_shape_id_a, transform_a, child.shape, child_sub_shape_id, child_world);
    }
}

/* cast shape */

void cast_compound_vs_shape(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const Shape *shape_a,
                            const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                            const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const collections::Vec<CompoundShapeChild> &children = shape_a->compound.data->children;

    for (uint32_t child_index = 0; child_index < children.length; child_index++) {
        if (collector.early_out_fraction <= -collector::UNBOUNDED) break;

        const CompoundShapeChild &child = children[child_index];

        sub_shape::SubShapeIdBuilder child_sub_shape_id;
        sub_shape::push_index(child_sub_shape_id, sub_shape_id_a, child_index, children.length);

        Transformf child_world;
        child_transform(child_world, transform_a, child);

        // the displacement is world space, the same for every child
        const shapes::CastShapeVsShapeFn fn = shapes::cast_fn(child.shape->type, shape_b->type);
        if (fn != nullptr) {
            fn(collector, settings, child.shape, child_sub_shape_id, child_world, displacement_a, shape_b, sub_shape_id_b, transform_b);
        }
    }
}

void cast_shape_vs_compound(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const Shape *shape_a,
                            const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                            const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const collections::Vec<CompoundShapeChild> &children = shape_b->compound.data->children;
    const collections::Vec<Box3f> &child_bounds = shape_b->compound.data->child_bounds;

    // a's bounds at the start of the cast and its displacement in b's unscaled space: the children are
    // culled by sweeping the centre of those bounds against child bounds grown by their extent
    const AABox query_bounds = bounds_in_compound_space(shape_a, transform_a, transform_b, 0.0f);
    const Vec3f center = aabox::center(query_bounds);
    const Vec3f extent = aabox::extents(query_bounds);
    const Quatf inv_quat = quatf::conjugate(quatf::load(transform_b.quaternion));
    const Vec3f local_displacement = vec3f::divide(vec3f::transform_quat(displacement_a, inv_quat), vec3f::load(transform_b.scale));
    const Vec3f inv_displacement = vec3f::make(cast_utils::safe_reciprocal(vec3f::x(local_displacement)), cast_utils::safe_reciprocal(vec3f::y(local_displacement)),
                                               cast_utils::safe_reciprocal(vec3f::z(local_displacement)));

    for (uint32_t child_index = 0; child_index < children.length; child_index++) {
        if (collector.early_out_fraction <= -collector::UNBOUNDED) break;

        // skip children the swept bounds cannot reach before the closest hit so far
        const AABox grown = aabox::expand_by_extents(aabox::load(child_bounds[child_index]), extent);
        const float positive_early_out = scalar::max(scalar::FLOAT_MIN_NORMAL, collector.early_out_fraction);
        if (cast_utils::ray_fraction_to_box3(center, inv_displacement, grown) >= positive_early_out) continue;

        const CompoundShapeChild &child = children[child_index];

        sub_shape::SubShapeIdBuilder child_sub_shape_id;
        sub_shape::push_index(child_sub_shape_id, sub_shape_id_b, child_index, children.length);

        Transformf child_world;
        child_transform(child_world, transform_b, child);

        const shapes::CastShapeVsShapeFn fn = shapes::cast_fn(shape_a->type, child.shape->type);
        if (fn != nullptr) {
            fn(collector, settings, shape_a, sub_shape_id_a, transform_a, displacement_a, child.shape, child_sub_shape_id, child_world);
        }
    }
}

shapes::ShapeDef def() {
    shapes::ShapeDef shape_def = {};
    shape_def.type = shapes::SHAPE_COMPOUND;
    shape_def.compute_mass_properties = compute_mass_properties;
    shape_def.get_surface_normal = get_surface_normal;
    shape_def.get_supporting_face = get_supporting_face;
    shape_def.get_material = get_material;
    shape_def.get_inner_radius = get_inner_radius;
    shape_def.get_leaf_shape = get_leaf_shape;
    shape_def.get_sub_shape_transformed_shape = get_sub_shape_transformed_shape;
    shape_def.cast_ray = cast_ray_vs_compound;
    shape_def.collide_point = collide_point_vs_compound;
    return shapes::define_shape(shape_def);
}

}  // namespace compound

TUMBLE_NAMESPACE_END
