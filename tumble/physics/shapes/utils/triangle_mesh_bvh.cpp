#include "tumble/physics/shapes/utils/triangle_mesh_bvh.h"

#include "tumble/math/aabox.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/alloc.h"
#include "tumble/util/assert.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

namespace triangle_mesh_bvh {

using triangle_mesh_data::Triangle;

/**
 * stride for precomputed triangle build data.
 * layout per triangle: [min_x, min_y, min_z, max_x, max_y, max_z], the triangle's exact aabb, so node
 * bounds built as unions of these contain their triangles exactly.
 */
static constexpr uint32_t BUILD_DATA_STRIDE = 6;
static constexpr uint32_t BUILD_DATA_MIN_X = 0;
static constexpr uint32_t BUILD_DATA_MIN_Y = 1;
static constexpr uint32_t BUILD_DATA_MIN_Z = 2;
static constexpr uint32_t BUILD_DATA_MAX_X = 3;
static constexpr uint32_t BUILD_DATA_MAX_Y = 4;
static constexpr uint32_t BUILD_DATA_MAX_Z = 5;

/** prevents degenerate splits where all centers are at the same point */
static constexpr float MIN_SIZE = 1e-5f;

struct SahBin {
    /** aabb bounds of triangles in this bin */
    AABox bounds;
    /** accumulated bounds from left */
    AABox left_cache_bounds;
    /** accumulated bounds from right (inclusive of this bin) */
    AABox right_cache_bounds;
    /** number of triangles in this bin */
    uint32_t count;
    /** split candidate position (center value) */
    float candidate;
};

/** a split axis (-1 for no valid split) and position */
struct Split {
    int32_t axis;
    float pos;
};

struct BuildContext {
    TriangleMeshData &data;
    const BvhBuildSettings &settings;
    collections::Vec<bvh::Node> &out;
    /** precomputed triangle bounds (exact min and max), BUILD_DATA_STRIDE per triangle */
    float *build_data;
    SahBin sah_bins[BIN_COUNT];
};

static void reset_sah_bin(SahBin &bin) {
    bin.bounds = aabox::empty();
    bin.left_cache_bounds = aabox::empty();
    bin.right_cache_bounds = aabox::empty();
    bin.count = 0;
    bin.candidate = 0.0f;
}

/** the triangle's aabb from its build data */
static inline AABox build_data_bounds(const float *entry) {
    return {vec3f::load(entry, BUILD_DATA_MIN_X), vec3f::load(entry, BUILD_DATA_MAX_X)};
}

/** the triangle's aabb center from its build data */
static inline Vec3f build_data_center(const float *entry) {
    return vec3f::scale(vec3f::add(vec3f::load(entry, BUILD_DATA_MIN_X), vec3f::load(entry, BUILD_DATA_MAX_X)), 0.5f);
}

/** the triangle's aabb center along axis (0 = x, 1 = y, 2 = z), from its build data */
static inline float build_data_center(const float *entry, int32_t axis) {
    return (entry[BUILD_DATA_MIN_X + axis] + entry[BUILD_DATA_MAX_X + axis]) * 0.5f;
}

/** io grown by box: a lane takes box's min only when below io's (`<`), its max only when above (`>`), as union_ */
static inline void expand_bounds(AABox &io, const AABox &box) { io = aabox::union_(io, box); }

static inline float axis_of(Vec3f v, int32_t axis) { return axis == 0 ? vec3f::x(v) : axis == 1 ? vec3f::y(v) : vec3f::z(v); }

static void precompute_triangle_build_data(float *out, const TriangleMeshData &data) {
    const uint32_t count = data.triangle_count;
    const float *positions = data.positions;

    for (uint32_t tri_idx = 0; tri_idx < count; tri_idx++) {
        const Triangle &triangle = data.triangles[tri_idx];
        float *entry = out + tri_idx * BUILD_DATA_STRIDE;

        // get vertex positions
        const float *a = positions + triangle.index_a * 3;
        const float *b = positions + triangle.index_b * 3;
        const float *c = positions + triangle.index_c * 3;

        // compute min bounds
        float min_x = a[0], min_y = a[1], min_z = a[2];
        if (b[0] < min_x) min_x = b[0];
        if (b[1] < min_y) min_y = b[1];
        if (b[2] < min_z) min_z = b[2];
        if (c[0] < min_x) min_x = c[0];
        if (c[1] < min_y) min_y = c[1];
        if (c[2] < min_z) min_z = c[2];

        // compute max bounds
        float max_x = a[0], max_y = a[1], max_z = a[2];
        if (b[0] > max_x) max_x = b[0];
        if (b[1] > max_y) max_y = b[1];
        if (b[2] > max_z) max_z = b[2];
        if (c[0] > max_x) max_x = c[0];
        if (c[1] > max_y) max_y = c[1];
        if (c[2] > max_z) max_z = c[2];

        entry[BUILD_DATA_MIN_X] = min_x;
        entry[BUILD_DATA_MIN_Y] = min_y;
        entry[BUILD_DATA_MIN_Z] = min_z;
        entry[BUILD_DATA_MAX_X] = max_x;
        entry[BUILD_DATA_MAX_Y] = max_y;
        entry[BUILD_DATA_MAX_Z] = max_z;
    }
}

/** swap precomputed build data entries when swapping triangles, to keep build data in sync. */
static void swap_triangle_build_data(float *build_data, uint32_t index_a, uint32_t index_b) {
    if (index_a == index_b) return;

    float *a = build_data + index_a * BUILD_DATA_STRIDE;
    float *b = build_data + index_b * BUILD_DATA_STRIDE;
    for (uint32_t i = 0; i < BUILD_DATA_STRIDE; i++) {
        const float tmp = a[i];
        a[i] = b[i];
        b[i] = tmp;
    }
}

static Split get_optimal_split(BuildContext &ctx, uint32_t start_index, uint32_t end_index, const AABox &node_bounds,
                               Vec3f center_min, Vec3f center_max);
static float get_average_center(const float *build_data, uint32_t start_index, uint32_t end_index, int32_t axis);
static uint32_t partition_triangles(BuildContext &ctx, uint32_t start_index, uint32_t end_index, int32_t axis, float split_pos);

/**
 * builds the subtree over triangles [start_index, end_index) in pre-order: the node itself, its
 * left subtree right after it, then its right subtree. returns the node's index.
 */
static int32_t build_recursive(BuildContext &ctx, uint32_t start_index, uint32_t end_index) {
    const uint32_t count = end_index - start_index;
    const float *build_data = ctx.build_data;

    // the node goes first; its fields are written once its children are known
    const int32_t node = (int32_t)ctx.out.length;
    ctx.out.push({});

    // single-pass computation of both node bounds and center bounds
    AABox bounds = {vec3f::zero(), vec3f::zero()};
    Vec3f center_min = vec3f::splat(scalar::INF);
    Vec3f center_max = vec3f::splat(-scalar::INF);

    if (count > 0) {
        // initialize with first triangle from precomputed data
        const float *entry = build_data + start_index * BUILD_DATA_STRIDE;
        bounds = build_data_bounds(entry);
        center_min = build_data_center(entry);
        center_max = center_min;

        // expand by remaining triangles (single pass for both bounds and centers); a center lane moves
        // only when strictly below the min (above the max), as min and max do
        for (uint32_t i = start_index + 1; i < end_index; i++) {
            entry = build_data + i * BUILD_DATA_STRIDE;
            expand_bounds(bounds, build_data_bounds(entry));

            const Vec3f center = build_data_center(entry);
            center_min = vec3f::min(center_min, center);
            center_max = vec3f::max(center_max, center);
        }
    }

    aabox::store(ctx.out[node].bounds, bounds);

    // a leaf when small enough (configurable threshold), when no valid split is found, or when the
    // split puts every center on the same side
    bool leaf = count <= ctx.settings.max_leaf_tris;
    Split split = {-1, 0.0f};
    uint32_t mid = start_index;
    if (!leaf) {
        // get optimal split (strategy picks its own axis and position)
        split = get_optimal_split(ctx, start_index, end_index, bounds, center_min, center_max);
        leaf = split.axis == -1;
    }
    if (!leaf) {
        // partition triangles using the chosen axis and position
        mid = partition_triangles(ctx, start_index, end_index, split.axis, split.pos);
        leaf = mid == start_index || mid == end_index;
    }

    if (leaf) {
        ctx.out[node].right_or_start = (int32_t)start_index;
        ctx.out[node].axis_or_count = -((int32_t)count + 1);  // negative encodes leaf + count
        return node;
    }

    // recursively build children; the left child immediately follows this node
    build_recursive(ctx, start_index, mid);
    const int32_t right = build_recursive(ctx, mid, end_index);
    ctx.out[node].right_or_start = right;
    ctx.out[node].axis_or_count = split.axis;
    return node;
}

bool build(collections::Vec<bvh::Node> &out, TriangleMeshData &data, const BvhBuildSettings &settings) {
    out.length = 0;
    const uint32_t count = data.triangle_count;
    if (count == 0) return true;

    // every leaf holds at least one triangle, so a tree has at most 2n - 1 nodes
    if (!out.reserve(2 * count - 1)) return false;

    float *build_data = util::alloc<float>(count * BUILD_DATA_STRIDE);
    if (build_data == nullptr) return false;

    // precompute triangle bounds (exact min and max)
    precompute_triangle_build_data(build_data, data);

    BuildContext ctx = {data, settings, out, build_data, {}};
    build_recursive(ctx, 0, count);

    util::free(build_data);
    return true;
}

/**
 * compute optimal split axis and position for the strategy. each strategy computes its own axis:
 * - CENTER: longest center extent
 * - AVERAGE: longest node bounds extent
 * - SAH: evaluates all axes for cost
 */
static Split get_optimal_split(BuildContext &ctx, uint32_t start_index, uint32_t end_index, const AABox &node_bounds,
                               Vec3f center_min, Vec3f center_max) {
    const float *build_data = ctx.build_data;
    int32_t axis = -1;
    float pos = 0.0f;

    const BvhSplitStrategy strategy = ctx.settings.strategy;
    if (strategy == BVH_SPLIT_STRATEGY_CENTER) {
        // use longest center extent
        const Vec3f extent = vec3f::subtract(center_max, center_min);
        const float ex = vec3f::x(extent), ey = vec3f::y(extent), ez = vec3f::z(extent);

        if (ex < MIN_SIZE && ey < MIN_SIZE && ez < MIN_SIZE) {
            // all centers at same point, no valid split
            axis = -1;
        } else {
            axis = ex >= ey && ex >= ez ? 0 : ey >= ez ? 1 : 2;
            pos = (axis_of(center_min, axis) + axis_of(center_max, axis)) / 2.0f;
        }
    } else if (strategy == BVH_SPLIT_STRATEGY_AVERAGE) {
        // use longest node bounds extent (not center extent), as three-mesh-bvh does
        const Vec3f extent = aabox::size(node_bounds);
        const float ex = vec3f::x(extent), ey = vec3f::y(extent), ez = vec3f::z(extent);
        axis = ex >= ey && ex >= ez ? 0 : ey >= ez ? 1 : 2;
        pos = get_average_center(build_data, start_index, end_index, axis);
    } else if (strategy == BVH_SPLIT_STRATEGY_SAH) {
        // surface area heuristic, binned as three-mesh-bvh does
        const uint32_t count = end_index - start_index;
        SahBin *bins = ctx.sah_bins;

        const Vec3f center_size = vec3f::subtract(center_max, center_min);

        if (vec3f::x(center_size) < MIN_SIZE && vec3f::y(center_size) < MIN_SIZE && vec3f::z(center_size) < MIN_SIZE) {
            // all centers at same point, axis remains -1
        } else {
            // root surface area for probability calculation
            const float root_surface_area = aabox::surface_area(node_bounds);
            float best_cost = TRIANGLE_INTERSECT_COST * (float)count;  // cost of making this a leaf
            int32_t best_axis = -1;
            float best_pos = 0.0f;

            for (int32_t a = 0; a < 3; a++) {
                const float axis_length = axis_of(center_size, a);
                if (axis_length < MIN_SIZE) continue;  // skip degenerate axis

                const float axis_left = axis_of(center_min, a);
                const float bin_width = axis_length / (float)BIN_COUNT;

                if (count < BIN_COUNT / 4) {
                    // small mesh mode: use exact triangle positions as candidates, one bin each
                    for (uint32_t i = 0; i < count; i++) {
                        const float *entry = build_data + (start_index + i) * BUILD_DATA_STRIDE;
                        SahBin &bin = bins[i];
                        reset_sah_bin(bin);
                        bin.candidate = build_data_center(entry, a);
                        // initialize bounds with this triangle's aabb
                        bin.bounds = build_data_bounds(entry);
                    }

                    // sort the first `count` bins by candidate position (stable, in place).
                    // bins past `count` hold stale candidates from earlier axes and nodes and must not take part
                    sort::insertion_sort(bins, bins + count, [](const SahBin &a, const SahBin &b) { return a.candidate < b.candidate; });

                    // remove duplicate candidates by compacting distinct bins to the front
                    uint32_t split_count = count;
                    uint32_t write_idx = 1;
                    for (uint32_t read_idx = 1; read_idx < count; read_idx++) {
                        if (bins[read_idx].candidate != bins[write_idx - 1].candidate) {
                            if (write_idx != read_idx) {
                                const SahBin temp = bins[write_idx];
                                bins[write_idx] = bins[read_idx];
                                bins[read_idx] = temp;
                            }
                            write_idx++;
                        } else {
                            split_count--;
                        }
                    }

                    // for each triangle, determine left vs right for each split candidate
                    for (uint32_t i = start_index; i < end_index; i++) {
                        const float *entry = build_data + i * BUILD_DATA_STRIDE;
                        const AABox triangle_bounds = build_data_bounds(entry);
                        const float center = build_data_center(entry, a);

                        for (uint32_t bi = 0; bi < split_count; bi++) {
                            SahBin &bin = bins[bi];
                            if (center >= bin.candidate) {
                                // right side: expand right cache bounds
                                expand_bounds(bin.right_cache_bounds, triangle_bounds);
                            } else {
                                // left side: expand left cache bounds and increment count
                                expand_bounds(bin.left_cache_bounds, triangle_bounds);
                                bin.count++;
                            }
                        }
                    }

                    // evaluate sah cost for each split candidate
                    for (uint32_t bi = 0; bi < split_count; bi++) {
                        const SahBin &bin = bins[bi];
                        const uint32_t left_count = bin.count;
                        const uint32_t right_count = count - left_count;

                        // probabilities, only for non-empty sides
                        float left_prob = 0.0f;
                        if (left_count != 0) left_prob = aabox::surface_area(bin.left_cache_bounds) / root_surface_area;

                        float right_prob = 0.0f;
                        if (right_count != 0) right_prob = aabox::surface_area(bin.right_cache_bounds) / root_surface_area;

                        const float cost =
                            TRAVERSAL_COST + TRIANGLE_INTERSECT_COST * (left_prob * (float)left_count + right_prob * (float)right_count);

                        if (cost < best_cost) {
                            best_cost = cost;
                            best_axis = a;
                            best_pos = bin.candidate;
                        }
                    }
                } else {
                    // large mesh mode: fixed BIN_COUNT bins over the center extent
                    for (uint32_t b = 0; b < BIN_COUNT; b++) {
                        SahBin &bin = bins[b];
                        reset_sah_bin(bin);
                        bin.candidate = axis_left + bin_width * (float)(b + 1);
                    }

                    // bin all triangles
                    for (uint32_t i = start_index; i < end_index; i++) {
                        const float *entry = build_data + i * BUILD_DATA_STRIDE;
                        const AABox triangle_bounds = build_data_bounds(entry);
                        const float center = build_data_center(entry, a);

                        // bin index (relative_pos >= 0 always since center >= axis_left)
                        const float relative_pos = center - axis_left;
                        uint32_t bin_index = (uint32_t)scalar::floor(relative_pos / bin_width);
                        if (bin_index >= BIN_COUNT) bin_index = BIN_COUNT - 1;

                        SahBin &bin = bins[bin_index];
                        bin.count++;

                        // expand bin bounds to include this triangle's aabb
                        expand_bounds(bin.bounds, triangle_bounds);
                    }

                    // cache right bounds from right to left
                    bins[BIN_COUNT - 1].right_cache_bounds = bins[BIN_COUNT - 1].bounds;
                    for (int32_t b = (int32_t)BIN_COUNT - 2; b >= 0; b--) {
                        bins[b].right_cache_bounds = aabox::union_(bins[b].bounds, bins[b + 1].right_cache_bounds);
                    }

                    // accumulate left bounds and evaluate cost
                    AABox left_bounds = aabox::empty();

                    uint32_t left_count = 0;
                    for (uint32_t b = 0; b < BIN_COUNT - 1; b++) {
                        const SahBin &bin = bins[b];
                        const uint32_t bin_count = bin.count;

                        // union current bin into left bounds (if not empty)
                        if (bin_count != 0) {
                            left_bounds = left_count == 0 ? bin.bounds : aabox::union_(left_bounds, bin.bounds);
                        }

                        left_count += bin_count;

                        // evaluate cost at this split position
                        const SahBin &next_bin = bins[b + 1];
                        const uint32_t right_count = count - left_count;

                        float left_prob = 0.0f;
                        if (left_count != 0) left_prob = aabox::surface_area(left_bounds) / root_surface_area;

                        float right_prob = 0.0f;
                        if (right_count != 0) right_prob = aabox::surface_area(next_bin.right_cache_bounds) / root_surface_area;

                        const float cost =
                            TRAVERSAL_COST + TRIANGLE_INTERSECT_COST * (left_prob * (float)left_count + right_prob * (float)right_count);

                        if (cost < best_cost) {
                            best_cost = cost;
                            best_axis = a;
                            best_pos = bin.candidate;
                        }
                    }
                }
            }

            if (best_axis != -1) {
                axis = best_axis;
                pos = best_pos;
            }
        }
    } else {
        TUMBLE_ASSERT(false && "invalid bvh split strategy");
    }

    return {axis, pos};
}

/** mean (not median) aabb center position along axis, for the AVERAGE strategy */
static float get_average_center(const float *build_data, uint32_t start_index, uint32_t end_index, int32_t axis) {
    float sum = 0.0f;
    const uint32_t count = end_index - start_index;

    for (uint32_t i = start_index; i < end_index; i++) {
        sum += build_data_center(build_data + i * BUILD_DATA_STRIDE, axis);
    }

    return sum / (float)count;
}

/**
 * partition triangles in place (hoare): centers < split_pos go left, >= split_pos go right.
 * reorders the triangles, so after the build every leaf holds a contiguous range of them.
 * returns the index of the first triangle in the right partition.
 */
static uint32_t partition_triangles(BuildContext &ctx, uint32_t start_index, uint32_t end_index, int32_t axis, float split_pos) {
    // signed: right walks to start_index - 1 when every center goes left
    int32_t left = (int32_t)start_index;
    int32_t right = (int32_t)end_index - 1;
    const float *build_data = ctx.build_data;

    while (left <= right) {
        const float left_center = build_data_center(build_data + (uint32_t)left * BUILD_DATA_STRIDE, axis);
        if (left_center < split_pos) {
            left++;
        } else {
            const float right_center = build_data_center(build_data + (uint32_t)right * BUILD_DATA_STRIDE, axis);
            if (right_center >= split_pos) {
                right--;
            } else {
                // swap triangles in the triangle buffer and build data
                triangle_mesh_data::swap_triangles(ctx.data, (uint32_t)left, (uint32_t)right);
                swap_triangle_build_data(ctx.build_data, (uint32_t)left, (uint32_t)right);
                left++;
                right--;
            }
        }
    }

    return (uint32_t)left;
}

TriangleMeshBvhStats stats(const bvh::Node *nodes, uint32_t node_count) {
    TriangleMeshBvhStats result = {0, 0, 0, 0, 0.0f};
    if (node_count == 0) return result;

    struct StackEntry {
        int32_t node;
        uint32_t depth;
    };

    uint32_t depth_sum = 0;
    uint32_t min_depth = 0xffffffffu;
    collections::Vec<StackEntry> stack = {};
    stack.push({0, 0});

    while (stack.length > 0) {
        const StackEntry entry = stack[stack.length - 1];
        stack.length--;
        result.node_count++;

        if (bvh::node_is_leaf(nodes, entry.node)) {
            result.leaf_count++;
            if (entry.depth < min_depth) min_depth = entry.depth;
            if (entry.depth > result.max_depth) result.max_depth = entry.depth;
            depth_sum += entry.depth;
        } else {
            stack.push({bvh::node_right(nodes, entry.node), entry.depth + 1});
            stack.push({bvh::node_left(entry.node), entry.depth + 1});
        }
    }

    stack.release();
    result.min_depth = min_depth;
    result.avg_depth = result.leaf_count > 0 ? (float)depth_sum / (float)result.leaf_count : 0.0f;
    return result;
}

}  // namespace triangle_mesh_bvh

TUMBLE_NAMESPACE_END
