#include "physics/shapes/static_compound.h"

#include "math/aabox.h"
#include "math/mat4f.h"
#include "math/quatf.h"
#include "math/scalar.h"
#include "math/vec3f.h"
#include "physics/collision/cast_utils.h"
#include "util/alloc.h"
#include "util/assert.h"
#include "util/mem.h"

namespace static_compound {

using static_compound_bvh::STACK_SIZE;

static void release_data(StaticCompoundData *data) {
    data->children.release();
    data->bvh.release();
    util::free(data);
}

Shape *create(const StaticCompoundShapeSettings &settings) {
    StaticCompoundData *data = util::alloc<StaticCompoundData>();
    if (data == nullptr) return nullptr;
    util::zero(data, sizeof(StaticCompoundData));
    data->bvh_settings.max_leaf_children = settings.bvh_max_leaf_children;
    if (!data->children.reserve(settings.num_children)) {
        release_data(data);
        return nullptr;
    }
    for (uint32_t i = 0; i < settings.num_children; i++) data->children.push(settings.children[i]);

    Shape *shape = shapes::allocate(shapes::SHAPE_STATIC_COMPOUND);
    if (shape == nullptr) {
        release_data(data);
        return nullptr;
    }
    shape->static_compound.data = data;
    shape->material_id = -1;

    if (!update(shape)) {
        destroy(shape);
        return nullptr;
    }

    return shape;
}

void destroy(Shape *shape) {
    release_data(shape->static_compound.data);
    shapes::release(shape);
}

bool update(Shape *shape) {
    StaticCompoundData &data = *shape->static_compound.data;

    // rebuild the bvh (reorders the children)
    if (!static_compound_bvh::build(data.bvh, data.children, data.bvh_settings)) return false;

    // recompute derived properties
    compound::update(shape);
    return true;
}

/* cast ray */

void cast_ray(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray, const Shape *shape,
              const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    const StaticCompoundData &data = *shape->static_compound.data;
    const bvh::Node *nodes = data.bvh.data;
    if (data.bvh.length == 0) return;

    // ray from world space into the compound's local space
    Quatf inv_quat = quatf::conjugate(quatf::load(transform.quaternion));
    Vec3f local_ray_origin = vec3f::subtract(ray.origin, vec3f::load(transform.position));
    local_ray_origin = vec3f::transform_quat(local_ray_origin, inv_quat);
    const Vec3f local_ray_direction = vec3f::transform_quat(ray.direction, inv_quat);

    // walk the bvh in the compound's unscaled local space; the entry fractions are unchanged by it
    const Vec3f local_origin = vec3f::divide(local_ray_origin, vec3f::load(transform.scale));
    const Vec3f local_direction = vec3f::divide(local_ray_direction, vec3f::load(transform.scale));

    // reciprocals of the local displacement, once per query, for the fraction-space slab tests
    const Vec3f inv_displacement = vec3f::make(cast_utils::safe_reciprocal(vec3f::x(local_direction) * ray.length),
                                    cast_utils::safe_reciprocal(vec3f::y(local_direction) * ray.length),
                                    cast_utils::safe_reciprocal(vec3f::z(local_direction) * ray.length));

    // the collector's early-out fraction, held in a local and refreshed after each child cast
    float early_out = collector.early_out_fraction;

    int32_t stack_nodes[STACK_SIZE];
    float stack_dist[STACK_SIZE];
    uint32_t stack_size = 0;
    stack_nodes[stack_size] = 0;
    stack_dist[stack_size] = -scalar::INF;  // root always visited
    stack_size++;

    while (stack_size > 0) {
        stack_size--;
        const int32_t node = stack_nodes[stack_size];
        const float node_distance = stack_dist[stack_size];

        // a node is only ever pushed with a finite entry fraction, so it needs no re-test of its bounds here
        if (node_distance >= early_out) continue;

        if (bvh::node_is_leaf(nodes, node)) {
            const uint32_t child_start = static_compound_bvh::node_child_start(nodes, node);
            const uint32_t child_count = static_compound_bvh::node_child_count(nodes, node);

            for (uint32_t i = 0; i < child_count; i++) {
                // an any-hit collector drops its early-out to zero: nothing further can qualify
                if (early_out <= 0.0f) break;

                const uint32_t child_index = child_start + i;
                const CompoundShapeChild &child = data.children[child_index];

                sub_shape::SubShapeIdBuilder child_sub_shape_id;
                sub_shape::push_index(child_sub_shape_id, sub_shape_id, child_index, data.children.length);

                // the children are placed in world space, so they get the world ray
                Transformf child_world;
                compound::child_transform(child_world, transform, child);

                shapes::def_of(child.shape).cast_ray(collector, settings, ray, child.shape, child_sub_shape_id, child_world);

                early_out = collector.early_out_fraction;
            }
        } else {
            // internal node: push the farther child first so the closer is popped first
            const int32_t left = bvh::node_left(node);
            const int32_t right = bvh::node_right(nodes, node);

            const float left_dist = cast_utils::ray_fraction_to_box3(local_origin, inv_displacement, aabox::load(nodes[left].bounds));
            const float right_dist = cast_utils::ray_fraction_to_box3(local_origin, inv_displacement, aabox::load(nodes[right].bounds));

            CC_ASSERT(stack_size + 2 <= STACK_SIZE);
            const int32_t near = left_dist <= right_dist ? left : right;
            const int32_t far = left_dist <= right_dist ? right : left;
            const float near_dist = left_dist <= right_dist ? left_dist : right_dist;
            const float far_dist = left_dist <= right_dist ? right_dist : left_dist;
            if (far_dist < early_out) {
                stack_nodes[stack_size] = far;
                stack_dist[stack_size] = far_dist;
                stack_size++;
            }
            if (near_dist < early_out) {
                stack_nodes[stack_size] = near;
                stack_dist[stack_size] = near_dist;
                stack_size++;
            }
        }
    }
}

/* collide point */

void collide_point(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings, Vec3f point,
                   const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const StaticCompoundData &data = *shape_b->static_compound.data;
    const bvh::Node *nodes = data.bvh.data;
    if (data.bvh.length == 0) return;

    // point into the compound's local space for the bvh walk: inverse(quaternion) * (point - position)
    const Quatf inv_quat_b = quatf::conjugate(quatf::load(transform_b.quaternion));
    Vec3f local_point = vec3f::subtract(point, vec3f::load(transform_b.position));
    local_point = vec3f::transform_quat(local_point, inv_quat_b);
    // the bvh is in the compound's unscaled local space
    local_point = vec3f::divide(local_point, vec3f::load(transform_b.scale));
    const AABox point_box = {local_point, local_point};

    int32_t stack_nodes[STACK_SIZE];
    uint32_t stack_size = 0;
    stack_nodes[stack_size++] = 0;

    while (stack_size > 0) {
        if (collide_point::should_early_out(collector)) break;

        const int32_t node = stack_nodes[--stack_size];

        if (!bvh::node_intersects_box(nodes, node, point_box)) continue;

        if (bvh::node_is_leaf(nodes, node)) {
            const uint32_t child_start = static_compound_bvh::node_child_start(nodes, node);
            const uint32_t child_count = static_compound_bvh::node_child_count(nodes, node);

            for (uint32_t i = 0; i < child_count; i++) {
                if (collide_point::should_early_out(collector)) break;

                const uint32_t child_index = child_start + i;
                const CompoundShapeChild &child = data.children[child_index];

                sub_shape::SubShapeIdBuilder child_sub_shape_id;
                sub_shape::push_index(child_sub_shape_id, sub_shape_id_b, child_index, data.children.length);

                Transformf child_world;
                compound::child_transform(child_world, transform_b, child);

                shapes::def_of(child.shape).collide_point(collector, settings, point, child.shape, child_sub_shape_id, child_world);
            }
        } else {
            // internal node: push both children
            CC_ASSERT(stack_size + 2 <= STACK_SIZE);
            stack_nodes[stack_size++] = bvh::node_left(node);
            stack_nodes[stack_size++] = bvh::node_right(nodes, node);
        }
    }
}

/* collide shape */

/**
 * bring a box from the compound's scaled local space into the unscaled space its bvh is built in.
 * a negative scale component swaps that axis' min and max, so they are re-ordered.
 */
static AABox divide_box_by_scale(const AABox &box, Vec3f scale) {
    // compound vs compound nests these walks per child, so the unscaled case is worth skipping
    if (vec3f::x(scale) == 1.0f && vec3f::y(scale) == 1.0f && vec3f::z(scale) == 1.0f) return box;

    const v128_t lo = v4f::div(box.min.v, scale.v);
    const v128_t hi = v4f::div(box.max.v, scale.v);
    // lo < hi ? lo : hi, and lo < hi ? hi : lo, per lane
    const v128_t lo_first = v4f::lt(lo, hi);
    return {{v4f::select(lo, hi, lo_first)}, {v4f::select(hi, lo, lo_first)}};
}

/** `other`'s pose in the compound's local space (unscaled by the compound's scale) */
static void relative_pose(Vec3f &out_position, Quatf &out_quaternion, const Transformf &compound, const Transformf &other) {
    Quatf inv_quat = quatf::conjugate(quatf::load(compound.quaternion));
    out_position = vec3f::subtract(vec3f::load(other.position), vec3f::load(compound.position));
    out_position = vec3f::transform_quat(out_position, inv_quat);
    out_quaternion = quatf::multiply(inv_quat, quatf::load(other.quaternion));
}

/** walks the static compound's bvh, calling `visit` for each child whose leaf overlaps `query_bounds` until `stop` says so */
template <typename Collector, typename Stop, typename Visit>
static inline void walk_overlapping(const StaticCompoundData &data, const AABox &query_bounds, const Collector &collector, Stop stop,
                                    Visit visit) {
    const bvh::Node *nodes = data.bvh.data;

    int32_t stack_nodes[STACK_SIZE];
    uint32_t stack_size = 0;
    stack_nodes[stack_size++] = 0;

    while (stack_size > 0) {
        if (stop(collector)) break;

        const int32_t node = stack_nodes[--stack_size];

        if (!bvh::node_intersects_box(nodes, node, query_bounds)) continue;

        if (bvh::node_is_leaf(nodes, node)) {
            const uint32_t child_start = static_compound_bvh::node_child_start(nodes, node);
            const uint32_t child_count = static_compound_bvh::node_child_count(nodes, node);

            for (uint32_t i = 0; i < child_count; i++) {
                if (stop(collector)) break;
                visit(child_start + i);
            }
        } else {
            CC_ASSERT(stack_size + 2 <= STACK_SIZE);
            stack_nodes[stack_size++] = bvh::node_left(node);
            stack_nodes[stack_size++] = bvh::node_right(nodes, node);
        }
    }
}

static bool collide_should_early_out(const collide_shape::CollideShapeCollector &collector) {
    return collide_shape::should_early_out(collector);
}

static bool cast_should_early_out(const cast_shape::CastShapeCollector &collector) {
    return collector.early_out_fraction <= -collector::UNBOUNDED;
}

void collide_static_compound_vs_shape(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                      const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a,
                                      const Transformf &transform_a, const Shape *shape_b,
                                      const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const StaticCompoundData &data = *shape_a->static_compound.data;
    if (data.bvh.length == 0) return;

    // shape b's bounds (scaled by b's scale) in compound a's local space, for the bvh culling
    Vec3f local_position_b;
    Quatf local_quaternion_b;
    relative_pose(local_position_b, local_quaternion_b, transform_a, transform_b);
    Mat4f aabb_transform = mat4f::from_rotation_translation_scale(local_quaternion_b, local_position_b, vec3f::load(transform_b.scale));
    AABox query_bounds = aabox::transform_mat4(aabox::load(shape_b->aabb), aabb_transform);

    // expand by max separation distance
    query_bounds = aabox::expand_by_margin(query_bounds, settings.max_separation_distance);
    query_bounds = divide_box_by_scale(query_bounds, vec3f::load(transform_a.scale));

    walk_overlapping(data, query_bounds, collector, collide_should_early_out, [&](uint32_t child_index) {
        const CompoundShapeChild &child = data.children[child_index];

        sub_shape::SubShapeIdBuilder child_sub_shape_id;
        sub_shape::push_index(child_sub_shape_id, sub_shape_id_a, child_index, data.children.length);

        Transformf child_world;
        compound::child_transform(child_world, transform_a, child);

        const shapes::CollideShapeVsShapeFn fn = shapes::collide_fn(child.shape->type, shape_b->type);
        if (fn != nullptr) fn(collector, settings, child.shape, child_sub_shape_id, child_world, shape_b, sub_shape_id_b, transform_b);
    });
}

void collide_shape_vs_static_compound(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                      const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a,
                                      const Transformf &transform_a, const Shape *shape_b,
                                      const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const StaticCompoundData &data = *shape_b->static_compound.data;
    if (data.bvh.length == 0) return;

    // shape a's bounds (scaled by a's scale) in compound b's local space, for the bvh culling
    Vec3f local_position_a;
    Quatf local_quaternion_a;
    relative_pose(local_position_a, local_quaternion_a, transform_b, transform_a);
    Mat4f aabb_transform = mat4f::from_rotation_translation_scale(local_quaternion_a, local_position_a, vec3f::load(transform_a.scale));
    AABox query_bounds = aabox::transform_mat4(aabox::load(shape_a->aabb), aabb_transform);

    // expand by max separation distance
    query_bounds = aabox::expand_by_margin(query_bounds, settings.max_separation_distance);
    query_bounds = divide_box_by_scale(query_bounds, vec3f::load(transform_b.scale));

    walk_overlapping(data, query_bounds, collector, collide_should_early_out, [&](uint32_t child_index) {
        const CompoundShapeChild &child = data.children[child_index];

        sub_shape::SubShapeIdBuilder child_sub_shape_id;
        sub_shape::push_index(child_sub_shape_id, sub_shape_id_b, child_index, data.children.length);

        Transformf child_world;
        compound::child_transform(child_world, transform_b, child);

        const shapes::CollideShapeVsShapeFn fn = shapes::collide_fn(shape_a->type, child.shape->type);
        if (fn != nullptr) fn(collector, settings, shape_a, sub_shape_id_a, transform_a, child.shape, child_sub_shape_id, child_world);
    });
}

/* cast shape */

void cast_static_compound_vs_shape(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                   const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                   Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                   const Transformf &transform_b) {
    const StaticCompoundData &data = *shape_a->static_compound.data;
    if (data.bvh.length == 0) return;

    // shape b's swept bounds in compound a's local space for the bvh culling: the compound moves by the
    // displacement, so from its frame b moves by -displacement
    Vec3f local_position_b;
    Quatf local_quaternion_b;
    relative_pose(local_position_b, local_quaternion_b, transform_a, transform_b);

    const Quatf inv_quat_a = quatf::conjugate(quatf::load(transform_a.quaternion));
    Vec3f local_displacement = vec3f::negate(displacement_a);
    local_displacement = vec3f::transform_quat(local_displacement, inv_quat_a);

    // start bounds (scaled by b's scale), end bounds translated by the displacement, and their union
    Mat4f aabb_transform = mat4f::from_rotation_translation_scale(local_quaternion_b, local_position_b, vec3f::load(transform_b.scale));
    AABox query_bounds = aabox::transform_mat4(aabox::load(shape_b->aabb), aabb_transform);
    const AABox end_bounds = {vec3f::add(query_bounds.min, local_displacement), vec3f::add(query_bounds.max, local_displacement)};
    query_bounds = aabox::union_(query_bounds, end_bounds);
    query_bounds = divide_box_by_scale(query_bounds, vec3f::load(transform_a.scale));

    walk_overlapping(data, query_bounds, collector, cast_should_early_out, [&](uint32_t child_index) {
        const CompoundShapeChild &child = data.children[child_index];

        sub_shape::SubShapeIdBuilder child_sub_shape_id;
        sub_shape::push_index(child_sub_shape_id, sub_shape_id_a, child_index, data.children.length);

        Transformf child_world;
        compound::child_transform(child_world, transform_a, child);

        const shapes::CastShapeVsShapeFn fn = shapes::cast_fn(child.shape->type, shape_b->type);
        if (fn != nullptr) {
            fn(collector, settings, child.shape, child_sub_shape_id, child_world, displacement_a, shape_b, sub_shape_id_b, transform_b);
        }
    });
}

/* cast shape vs static compound */

/** the child's aabb in the compound's local space, through its rotation and position */
static AABox calculate_child_aabb(const CompoundShapeChild &child) {
    const Mat4f matrix = mat4f::from_rotation_translation(quatf::load(child.quaternion), vec3f::load(child.position));
    return aabox::transform_mat4(aabox::load(child.shape->aabb), matrix);
}

void cast_shape_vs_static_compound(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                   const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                   Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                   const Transformf &transform_b) {
    const StaticCompoundData &data = *shape_b->static_compound.data;
    const bvh::Node *nodes = data.bvh.data;

    // early out if there are no children
    if (data.bvh.length == 0) return;

    // a into b's local space
    Vec3f position_a_in_b;
    Quatf quaternion_a_in_b;
    relative_pose(position_a_in_b, quaternion_a_in_b, transform_b, transform_a);

    const Quatf inverse_quaternion_b = quatf::conjugate(quatf::load(transform_b.quaternion));
    Vec3f displacement_in_b = vec3f::transform_quat(displacement_a, inverse_quaternion_b);
    // the bvh is in the compound's unscaled local space
    if (transform_b.scale.x != 1.0f || transform_b.scale.y != 1.0f || transform_b.scale.z != 1.0f) {
        displacement_in_b = vec3f::divide(displacement_in_b, vec3f::load(transform_b.scale));
    }

    // base aabb of shape a at t = 0 in the compound's local space
    const Mat4f aabb_matrix = mat4f::from_rotation_translation_scale(quaternion_a_in_b, position_a_in_b, vec3f::load(transform_a.scale));
    const AABox swept_aabb = divide_box_by_scale(aabox::transform_mat4(aabox::load(shape_a->aabb), aabb_matrix), vec3f::load(transform_b.scale));

    // a ray from the base aabb's centre along the normalised displacement
    cast_ray::Rayf ray;
    ray.origin = aabox::center(swept_aabb);
    ray.length = vec3f::length(displacement_in_b);
    const float inv_ray_length = ray.length > 1e-10f ? 1.0f / ray.length : 0.0f;
    ray.direction = vec3f::scale(displacement_in_b, inv_ray_length);

    // half extents of the base aabb, which every bound the ray is tested against grows by
    const Vec3f half_extents = aabox::extents(swept_aabb);

    int32_t stack_nodes[STACK_SIZE];
    float stack_dist[STACK_SIZE];
    uint32_t stack_size = 0;
    stack_nodes[stack_size] = 0;
    stack_dist[stack_size] = 0.0f;  // root always visited
    stack_size++;

    while (stack_size > 0) {
        stack_size--;
        const int32_t node = stack_nodes[stack_size];
        const float node_distance = stack_dist[stack_size];

        // early out: if fraction to this node >= closest hit, skip it
        if (node_distance >= scalar::max(scalar::FLOAT_MIN_NORMAL, collector.early_out_fraction)) continue;

        if (bvh::node_is_leaf(nodes, node)) {
            const uint32_t child_start = static_compound_bvh::node_child_start(nodes, node);
            const uint32_t child_count = static_compound_bvh::node_child_count(nodes, node);

            for (uint32_t i = 0; i < child_count; i++) {
                const uint32_t child_index = child_start + i;
                const CompoundShapeChild &child = data.children[child_index];

                // early out: ray x child bounds expanded by the half extents
                if (!cast_utils::ray_hits_box3(ray, aabox::expand_by_extents(calculate_child_aabb(child), half_extents))) continue;

                // stop once the collector wants no more hits
                if (collector.early_out_fraction <= -collector::UNBOUNDED) return;

                sub_shape::SubShapeIdBuilder child_sub_shape_id;
                sub_shape::push_index(child_sub_shape_id, sub_shape_id_b, child_index, data.children.length);

                Transformf child_world;
                compound::child_transform(child_world, transform_b, child);

                const shapes::CastShapeVsShapeFn fn = shapes::cast_fn(shape_a->type, child.shape->type);
                if (fn != nullptr) {
                    fn(collector, settings, shape_a, sub_shape_id_a, transform_a, displacement_a, child.shape, child_sub_shape_id, child_world);
                }
            }
        } else {
            // internal node: the distances along the ray to both children's expanded bounds, nearest popped first
            const int32_t left = bvh::node_left(node);
            const int32_t right = bvh::node_right(nodes, node);

            const float left_dist = cast_utils::ray_distance_to_box3(ray, aabox::expand_by_extents(aabox::load(nodes[left].bounds), half_extents));
            const float right_dist = cast_utils::ray_distance_to_box3(ray, aabox::expand_by_extents(aabox::load(nodes[right].bounds), half_extents));

            // floored at FLT_MIN so nodes the cast starts inside survive a hit at fraction 0
            const float positive_early_out = scalar::max(scalar::FLOAT_MIN_NORMAL, collector.early_out_fraction);

            CC_ASSERT(stack_size + 2 <= STACK_SIZE);
            const int32_t near = left_dist <= right_dist ? left : right;
            const int32_t far = left_dist <= right_dist ? right : left;
            const float near_dist = left_dist <= right_dist ? left_dist : right_dist;
            const float far_dist = left_dist <= right_dist ? right_dist : left_dist;
            if (far_dist < positive_early_out) {
                stack_nodes[stack_size] = far;
                stack_dist[stack_size] = far_dist;
                stack_size++;
            }
            if (near_dist < positive_early_out) {
                stack_nodes[stack_size] = near;
                stack_dist[stack_size] = near_dist;
                stack_size++;
            }
        }
    }
}

shapes::ShapeDef def() {
    shapes::ShapeDef shape_def = {};
    shape_def.type = shapes::SHAPE_STATIC_COMPOUND;
    shape_def.compute_mass_properties = compound::compute_mass_properties;
    shape_def.get_surface_normal = compound::get_surface_normal;
    shape_def.get_supporting_face = compound::get_supporting_face;
    shape_def.get_material = compound::get_material;
    shape_def.get_inner_radius = compound::get_inner_radius;
    shape_def.get_leaf_shape = compound::get_leaf_shape;
    shape_def.get_sub_shape_transformed_shape = compound::get_sub_shape_transformed_shape;
    shape_def.cast_ray = cast_ray;
    shape_def.collide_point = collide_point;
    return shapes::define_shape(shape_def);
}

}  // namespace static_compound
