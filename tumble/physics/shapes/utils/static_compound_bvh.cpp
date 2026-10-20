#include "tumble/physics/shapes/utils/static_compound_bvh.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/alloc.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace static_compound_bvh {

using compound::CompoundShapeChild;

/**
 * stride for precomputed child build data.
 * layout per child: [min_x, min_y, min_z, max_x, max_y, max_z], the child's exact aabb, so node
 * bounds built as unions of these contain their children exactly.
 */
static constexpr uint32_t BUILD_DATA_STRIDE = 6;
static constexpr uint32_t BUILD_DATA_MIN_X = 0;
static constexpr uint32_t BUILD_DATA_MAX_X = 3;

/** prevents degenerate splits where all centers are at the same point */
static constexpr float MIN_SIZE = 1e-5f;

/** a split axis (-1 for no valid split) and position */
struct Split {
    int32_t axis;
    float pos;
};

struct BuildContext {
    const BvhBuildSettings &settings;
    collections::Vec<bvh::Node> &out;
    /** the child each build slot holds, permuted alongside build_data */
    uint32_t *indices;
    /** precomputed child bounds as exact min and max, BUILD_DATA_STRIDE per slot */
    float *build_data;
};

static inline float axis_of(Vec3f v, int32_t axis) { return axis == 0 ? vec3f::x(v) : axis == 1 ? vec3f::y(v) : vec3f::z(v); }

/** the child's aabb center along axis (0=x, 1=y, 2=z), from its build data entry */
static inline float build_data_center(const float *entry, uint32_t axis) {
    return (entry[BUILD_DATA_MIN_X + axis] + entry[BUILD_DATA_MAX_X + axis]) * 0.5f;
}

/** compute child aabb in compound local space (transformed by position and rotation) */
static AABox compute_child_bounds(const CompoundShapeChild &child) {
    const Mat4f transform = mat4f::from_rotation_translation(quatf::load(child.quaternion), vec3f::load(child.position));
    return aabox::transform_mat4(aabox::load(child.shape->aabb), transform);
}

/** precompute child bounds (exact min and max) */
static void precompute_child_build_data(float *build_data, const collections::Vec<CompoundShapeChild> &children) {
    for (uint32_t i = 0; i < children.length; i++) {
        float *entry = build_data + i * BUILD_DATA_STRIDE;
        const AABox child_bounds = compute_child_bounds(children[i]);
        vec3f::store(entry, child_bounds.min, BUILD_DATA_MIN_X);
        vec3f::store(entry, child_bounds.max, BUILD_DATA_MAX_X);
    }
}

/** swap two indices in the index array and their build data */
static void swap_indices(BuildContext &ctx, uint32_t index_a, uint32_t index_b) {
    if (index_a == index_b) return;

    const uint32_t tmp = ctx.indices[index_a];
    ctx.indices[index_a] = ctx.indices[index_b];
    ctx.indices[index_b] = tmp;

    float *a = ctx.build_data + index_a * BUILD_DATA_STRIDE;
    float *b = ctx.build_data + index_b * BUILD_DATA_STRIDE;
    for (uint32_t i = 0; i < BUILD_DATA_STRIDE; i++) {
        const float tmp_data = a[i];
        a[i] = b[i];
        b[i] = tmp_data;
    }
}

/**
 * compute the split axis and position by spatial median: the midpoint of the centroid bounds along
 * the longest axis.
 */
static Split get_optimal_split(Vec3f center_min, Vec3f center_max) {
    const Vec3f extent = vec3f::subtract(center_max, center_min);
    const float ex = vec3f::x(extent), ey = vec3f::y(extent), ez = vec3f::z(extent);

    if (ex < MIN_SIZE && ey < MIN_SIZE && ez < MIN_SIZE) return {-1, 0.0f};

    // choose longest axis
    const int32_t axis = ex >= ey && ex >= ez ? 0 : ey >= ez ? 1 : 2;

    // split at midpoint
    return {axis, (axis_of(center_min, axis) + axis_of(center_max, axis)) / 2.0f};
}

/** partition children in place, hoare's scheme; returns the first slot of the right side */
static uint32_t partition_children(BuildContext &ctx, uint32_t start_index, uint32_t end_index, int32_t axis, float split_pos) {
    int32_t left = (int32_t)start_index;
    int32_t right = (int32_t)end_index - 1;

    while (left <= right) {
        const float left_center = build_data_center(ctx.build_data + (uint32_t)left * BUILD_DATA_STRIDE, (uint32_t)axis);
        if (left_center < split_pos) {
            left++;
        } else {
            const float right_center = build_data_center(ctx.build_data + (uint32_t)right * BUILD_DATA_STRIDE, (uint32_t)axis);
            if (right_center >= split_pos) {
                right--;
            } else {
                swap_indices(ctx, (uint32_t)left, (uint32_t)right);
                left++;
                right--;
            }
        }
    }

    return (uint32_t)left;
}

/**
 * builds the subtree over slots [start_index, end_index) in pre-order: the node itself, its left
 * subtree right after it, then its right subtree. returns the node's index.
 */
static int32_t build_recursive(BuildContext &ctx, uint32_t start_index, uint32_t end_index, uint32_t depth) {
    const uint32_t count = end_index - start_index;
    const float *build_data = ctx.build_data;

    // the node goes first; its fields are written once its children are known
    const int32_t node = (int32_t)ctx.out.length;
    ctx.out.push({});

    // compute bounds and center bounds; a lane moves only when strictly below the min (above the max), as min and max do
    AABox bounds = {vec3f::zero(), vec3f::zero()};
    Vec3f center_min = vec3f::splat(scalar::INF);
    Vec3f center_max = vec3f::splat(-scalar::INF);

    for (uint32_t i = start_index; i < end_index; i++) {
        const float *entry = build_data + i * BUILD_DATA_STRIDE;
        const AABox entry_bounds = {vec3f::load(entry, BUILD_DATA_MIN_X), vec3f::load(entry, BUILD_DATA_MAX_X)};
        const Vec3f center = vec3f::scale(vec3f::add(entry_bounds.min, entry_bounds.max), 0.5f);

        if (i == start_index) {
            bounds = entry_bounds;
            center_min = center;
            center_max = center_min;
            continue;
        }

        bounds = aabox::union_(bounds, entry_bounds);
        center_min = vec3f::min(center_min, center);
        center_max = vec3f::max(center_max, center);
    }

    aabox::store(ctx.out[node].bounds, bounds);

    // a leaf when small enough, at the depth limit, when no valid split is found, or when the split
    // puts every center on the same side
    bool leaf = count <= ctx.settings.max_leaf_children || depth >= MAX_DEPTH;
    Split split = {-1, 0.0f};
    uint32_t mid = start_index;
    if (!leaf) {
        split = get_optimal_split(center_min, center_max);
        leaf = split.axis == -1;
    }
    if (!leaf) {
        mid = partition_children(ctx, start_index, end_index, split.axis, split.pos);
        leaf = mid == start_index || mid == end_index;
    }

    if (leaf) {
        ctx.out[node].right_or_start = (int32_t)start_index;
        ctx.out[node].axis_or_count = -((int32_t)count + 1);  // negative encodes leaf + count
        return node;
    }

    // recursively build children; the left child immediately follows this node
    build_recursive(ctx, start_index, mid, depth + 1);
    const int32_t right = build_recursive(ctx, mid, end_index, depth + 1);
    ctx.out[node].right_or_start = right;
    ctx.out[node].axis_or_count = split.axis;
    return node;
}

bool build(collections::Vec<bvh::Node> &out, collections::Vec<CompoundShapeChild> &children, const BvhBuildSettings &settings) {
    out.length = 0;
    const uint32_t count = children.length;
    if (count == 0) return true;

    // every leaf holds at least one child, so a tree has at most 2n - 1 nodes
    if (!out.reserve(2 * count - 1)) return false;

    uint32_t *indices = util::alloc<uint32_t>(count);
    float *build_data = util::alloc<float>(count * BUILD_DATA_STRIDE);
    CompoundShapeChild *reordered = util::alloc<CompoundShapeChild>(count);
    if (indices == nullptr || build_data == nullptr || reordered == nullptr) {
        util::free(indices, count);
        util::free(build_data, count * BUILD_DATA_STRIDE);
        util::free(reordered, count);
        return false;
    }

    for (uint32_t i = 0; i < count; i++) indices[i] = i;
    precompute_child_build_data(build_data, children);

    BuildContext ctx = {settings, out, indices, build_data};
    build_recursive(ctx, 0, count, 0);

    // reorder children based on the final indices order
    for (uint32_t i = 0; i < count; i++) reordered[i] = children[indices[i]];
    for (uint32_t i = 0; i < count; i++) children[i] = reordered[i];

    util::free(indices, count);
    util::free(build_data, count * BUILD_DATA_STRIDE);
    util::free(reordered, count);
    return true;
}

StaticCompoundBvhStats stats(const bvh::Node *nodes, uint32_t node_count) {
    StaticCompoundBvhStats out = {};
    if (node_count == 0) return out;

    uint32_t depth_sum = 0;
    out.min_depth = 0xffffffffu;

    int32_t stack_node[STACK_SIZE + 1];
    uint32_t stack_depth[STACK_SIZE + 1];
    uint32_t stack_size = 0;
    stack_node[stack_size] = 0;
    stack_depth[stack_size] = 0;
    stack_size++;

    while (stack_size > 0) {
        stack_size--;
        const int32_t node = stack_node[stack_size];
        const uint32_t depth = stack_depth[stack_size];
        out.node_count++;

        if (bvh::node_is_leaf(nodes, node)) {
            out.leaf_count++;
            if (depth < out.min_depth) out.min_depth = depth;
            if (depth > out.max_depth) out.max_depth = depth;
            depth_sum += depth;
            out.total_children += node_child_count(nodes, node);
        } else {
            TUMBLE_ASSERT(stack_size + 2 <= STACK_SIZE + 1);
            stack_node[stack_size] = bvh::node_right(nodes, node);
            stack_depth[stack_size] = depth + 1;
            stack_size++;
            stack_node[stack_size] = bvh::node_left(node);
            stack_depth[stack_size] = depth + 1;
            stack_size++;
        }
    }

    out.avg_depth = out.leaf_count > 0 ? (float)depth_sum / (float)out.leaf_count : 0.0f;
    return out;
}

}  // namespace static_compound_bvh

TUMBLE_NAMESPACE_END
