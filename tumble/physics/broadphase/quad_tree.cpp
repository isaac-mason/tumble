#include "physics/broadphase/quad_tree.h"

#include "math/aabox.h"
#include "math/scalar.h"
#include "math/vec3f.h"
#include "physics/body/rigid_body.h"
#include "util/assert.h"

namespace quad_tree {

using rigid_body::RigidBody;

// ---- nodes ----

static void init_node(Node &node, bool is_changed) {
    for (uint32_t i = 0; i < 4; i++) {
        node.bounds_min_x[i] = LARGE_FLOAT;
        node.bounds_min_y[i] = LARGE_FLOAT;
        node.bounds_min_z[i] = LARGE_FLOAT;
        node.bounds_max_x[i] = -LARGE_FLOAT;
        node.bounds_max_y[i] = -LARGE_FLOAT;
        node.bounds_max_z[i] = -LARGE_FLOAT;
        node.child_node_id[i] = INVALID_NODE_ID;
    }
    node.parent_node_index = INVALID_NODE_INDEX;
    node.is_changed = is_changed ? 1 : 0;
}

static AABox get_child_bounds(const Node &node, uint32_t child_index) {
    return {vec3f::make(node.bounds_min_x[child_index], node.bounds_min_y[child_index], node.bounds_min_z[child_index]),
            vec3f::make(node.bounds_max_x[child_index], node.bounds_max_y[child_index], node.bounds_max_z[child_index])};
}

static void set_child_bounds(Node &node, uint32_t child_index, const AABox &bounds) {
    node.bounds_min_x[child_index] = vec3f::x(bounds.min);
    node.bounds_min_y[child_index] = vec3f::y(bounds.min);
    node.bounds_min_z[child_index] = vec3f::z(bounds.min);
    node.bounds_max_x[child_index] = vec3f::x(bounds.max);
    node.bounds_max_y[child_index] = vec3f::y(bounds.max);
    node.bounds_max_z[child_index] = vec3f::z(bounds.max);
}

static void invalidate_child_bounds(Node &node, uint32_t child_index) {
    node.bounds_min_x[child_index] = LARGE_FLOAT;
    node.bounds_min_y[child_index] = LARGE_FLOAT;
    node.bounds_min_z[child_index] = LARGE_FLOAT;
    node.bounds_max_x[child_index] = -LARGE_FLOAT;
    node.bounds_max_y[child_index] = -LARGE_FLOAT;
    node.bounds_max_z[child_index] = -LARGE_FLOAT;
}

static AABox get_node_bounds(const Node &node) {
    const v128_t min_x = v4f::load4(node.bounds_min_x);
    const v128_t min_y = v4f::load4(node.bounds_min_y);
    const v128_t min_z = v4f::load4(node.bounds_min_z);
    const v128_t max_x = v4f::load4(node.bounds_max_x);
    const v128_t max_y = v4f::load4(node.bounds_max_y);
    const v128_t max_z = v4f::load4(node.bounds_max_z);
    // across the four lanes in two shuffles; bounds hold no nan, so the lane min and max are exact
    const auto hmin = [](v128_t v) {
        const v128_t pairs = v4f::min(v, v4f::shuffle<2, 3, 0, 1>(v));
        return v4f::lane<0>(v4f::min(pairs, v4f::shuffle<1, 0, 3, 2>(pairs)));
    };
    const auto hmax = [](v128_t v) {
        const v128_t pairs = v4f::max(v, v4f::shuffle<2, 3, 0, 1>(v));
        return v4f::lane<0>(v4f::max(pairs, v4f::shuffle<1, 0, 3, 2>(pairs)));
    };
    return {vec3f::make(hmin(min_x), hmin(min_y), hmin(min_z)), vec3f::make(hmax(max_x), hmax(max_y), hmax(max_z))};
}

/** grows the child's bounds to hold the box; true when they changed */
static bool encapsulate_child_bounds(Node &node, uint32_t child_index, const AABox &bounds) {
    bool changed = false;
    const auto grow_min = [&](float &v, float b) {
        if (b < v) {
            v = b;
            changed = true;
        }
    };
    const auto grow_max = [&](float &v, float b) {
        if (b > v) {
            v = b;
            changed = true;
        }
    };
    grow_min(node.bounds_min_x[child_index], vec3f::x(bounds.min));
    grow_min(node.bounds_min_y[child_index], vec3f::y(bounds.min));
    grow_min(node.bounds_min_z[child_index], vec3f::z(bounds.min));
    grow_max(node.bounds_max_x[child_index], vec3f::x(bounds.max));
    grow_max(node.bounds_max_y[child_index], vec3f::y(bounds.max));
    grow_max(node.bounds_max_z[child_index], vec3f::z(bounds.max));
    return changed;
}

static uint32_t allocate_node(QuadTree &tree, bool is_changed) {
    uint32_t index;
    if (tree.free_nodes.length > 0) {
        index = tree.free_nodes.data[--tree.free_nodes.length];
    } else {
        index = tree.nodes.length;
        if (!tree.nodes.push({})) return INVALID_NODE_INDEX;
    }
    init_node(tree.nodes[index], is_changed);
    return index;
}

// ---- body locations ----

static void get_body_location(const RigidBody &body, uint32_t &node_index, uint32_t &child_index) {
    const uint32_t location = (uint32_t)body.broadphase_node;
    node_index = location & 0x3fffffffu;
    child_index = location >> 30;
}

static void set_body_location(RigidBody &body, uint32_t node_index, uint32_t child_index) {
    CC_ASSERT(node_index <= 0x3fffffffu && child_index < 4);
    body.broadphase_node = (int32_t)(node_index | (child_index << 30));
}

// ---- tree ----

QuadTree create() {
    QuadTree tree = {};
    tree.expansion_margin = 0.05f;
    tree.root_node = allocate_node(tree, false);
    return tree;
}

void release(QuadTree &tree) {
    tree.nodes.release();
    tree.free_nodes.release();
    tree.free_node_batch.release();
    tree.node_stack.release();
    tree.fraction_stack.release();
    tree.build_ids.release();
    tree.build_centers.release();
    tree.build_stack.release();
    tree.root_node = INVALID_NODE_INDEX;
    tree.num_bodies = 0;
    tree.dirty = false;
}

Box3f &bounds(Box3f &out, const QuadTree &tree) {
    if (tree.num_bodies == 0) return aabox::store(out, aabox::empty());
    return aabox::store(out, get_node_bounds(tree.nodes[tree.root_node]));
}

static AABox body_bounds(const QuadTree &tree, const RigidBody &body) {
    uint32_t node_index, child_index;
    get_body_location(body, node_index, child_index);
    return get_child_bounds(tree.nodes[node_index], child_index);
}

Box3f &read_body_bounds(Box3f &out, const QuadTree &tree, const RigidBody &body) { return aabox::store(out, body_bounds(tree, body)); }

static void mark_node_and_parents_changed(QuadTree &tree, uint32_t node_index) {
    uint32_t index = node_index;
    do {
        // if a node has changed, its parent has too
        Node &node = tree.nodes[index];
        if (node.is_changed) break;
        node.is_changed = 1;
        index = node.parent_node_index;
    } while (index != INVALID_NODE_INDEX);
}

static void widen_and_mark_node_and_parents_changed(QuadTree &tree, uint32_t node_index, const AABox &new_bounds) {
    uint32_t index = node_index;
    for (;;) {
        Node &node = tree.nodes[index];
        node.is_changed = 1;

        const uint32_t parent_index = node.parent_node_index;
        if (parent_index == INVALID_NODE_INDEX) break;

        // which child of the parent this node is
        Node &parent = tree.nodes[parent_index];
        const NodeId id = from_node_index(index);
        uint32_t child_index = 4;
        for (uint32_t i = 0; i < 4; i++) {
            if (parent.child_node_id[i] == id) {
                child_index = i;
                break;
            }
        }
        CC_ASSERT(child_index != 4);

        // the bounds only ever grow until the rebuild
        if (!encapsulate_child_bounds(parent, child_index, new_bounds)) {
            // no change to the bounds: only the marking remains
            if (!parent.is_changed) mark_node_and_parents_changed(tree, parent_index);
            break;
        }
        index = parent_index;
    }
}

static bool try_insert_leaf(QuadTree &tree, uint32_t node_index, RigidBody *body, const AABox &leaf_bounds) {
    Node &node = tree.nodes[node_index];
    for (uint32_t child_index = 0; child_index < 4; child_index++) {
        if (node.child_node_id[child_index] != INVALID_NODE_ID) continue;
        node.child_node_id[child_index] = body->index;
        set_body_location(*body, node_index, child_index);
        set_child_bounds(node, child_index, leaf_bounds);
        widen_and_mark_node_and_parents_changed(tree, node_index, leaf_bounds);
        tree.num_bodies++;
        return true;
    }
    return false;
}

static void create_new_root(QuadTree &tree, RigidBody *body, const AABox &leaf_bounds) {
    const uint32_t root_index = tree.root_node;

    // marked changed: this is no efficient tree yet
    const uint32_t new_root_index = allocate_node(tree, true);
    if (new_root_index == INVALID_NODE_INDEX) return;
    Node &root = tree.nodes[root_index];
    Node &new_root = tree.nodes[new_root_index];

    // the old root as the first child. jolt gives it an infinite box, as another thread may be widening the old root
    // meanwhile; nothing runs alongside here, so it gets the old root's own bounds
    new_root.child_node_id[0] = from_node_index(root_index);
    set_child_bounds(new_root, 0, get_node_bounds(root));

    new_root.child_node_id[1] = body->index;
    set_child_bounds(new_root, 1, leaf_bounds);

    tree.root_node = new_root_index;
    set_body_location(*body, new_root_index, 1);
    root.parent_node_index = new_root_index;
    tree.num_bodies++;
}

void add(QuadTree &tree, RigidBody *body) {
    const AABox leaf_bounds = aabox::expand_by_margin(aabox::load(body->aabb), tree.expansion_margin);
    tree.dirty = true;
    if (!try_insert_leaf(tree, tree.root_node, body, leaf_bounds)) create_new_root(tree, body, leaf_bounds);
}

void remove(QuadTree &tree, RigidBody *body) {
    if (body->broadphase_node == -1) return;
    tree.dirty = true;

    uint32_t node_index, child_index;
    get_body_location(*body, node_index, child_index);
    body->broadphase_node = -1;

    // no query finds the slot from here; it takes adds again
    Node &node = tree.nodes[node_index];
    invalidate_child_bounds(node, child_index);
    node.child_node_id[child_index] = INVALID_NODE_ID;

    // volumes never shrink before the rebuild, so only the marking bubbles up
    mark_node_and_parents_changed(tree, node_index);
    tree.num_bodies--;
}

bool update(QuadTree &tree, const RigidBody *body) {
    if (body->broadphase_node == -1) return false;
    uint32_t node_index, child_index;
    get_body_location(*body, node_index, child_index);
    Node &node = tree.nodes[node_index];

    // still inside its fat box
    const AABox aabb = aabox::load(body->aabb);
    if (aabox::contains(get_child_bounds(node, child_index), aabb)) return false;

    // the fat box is replaced, so the persistent pairs see where the body is now, then the ancestors widen to fit
    const AABox new_bounds = aabox::expand_by_margin(aabb, tree.expansion_margin);
    set_child_bounds(node, child_index, new_bounds);
    widen_and_mark_node_and_parents_changed(tree, node_index, new_bounds);
    tree.dirty = true;
    return true;
}

// ---- rebuild ----

/**
 * a node's bounds, or a body's fat box. a body's is read from its slot in the tree being replaced: the rebuild frees
 * the old nodes only after the new tree is in, so they hold it throughout.
 */
static AABox get_node_or_body_bounds(const QuadTree &tree, const bodies::Bodies &bodies, NodeId id) {
    if (is_node(id)) return get_node_bounds(tree.nodes[node_index(id)]);
    return body_bounds(tree, *bodies.pool[id]);
}

static float axis(Vec3f v, uint32_t dimension) { return dimension == 0 ? vec3f::x(v) : dimension == 1 ? vec3f::y(v) : vec3f::z(v); }

/** sorts ids spatially into two groups; the second starts at mid_point. ids and centers are shuffled together */
static void partition(NodeId *ids, Vec3f *centers, int32_t number, int32_t &mid_point) {
    if (number <= 4) {
        mid_point = number / 2;
        return;
    }

    // the bounding box of the box centres
    Vec3f center_min = vec3f::splat(LARGE_FLOAT);
    Vec3f center_max = vec3f::splat(-LARGE_FLOAT);
    for (int32_t i = 0; i < number; i++) {
        center_min = vec3f::min(center_min, centers[i]);
        center_max = vec3f::max(center_max, centers[i]);
    }

    // the split plane, on the axis of the largest extent
    const Vec3f extent = vec3f::subtract(center_max, center_min);
    const float ex = vec3f::x(extent);
    const float ey = vec3f::y(extent);
    const float ez = vec3f::z(extent);
    const uint32_t dimension = ex > ey ? (ez > ex ? 2 : 0) : (ez > ey ? 2 : 1);
    const float split = 0.5f * axis(vec3f::add(center_min, center_max), dimension);

    int32_t start = 0;
    int32_t end = number;
    while (start < end) {
        // the first element on the right of the plane
        while (start < end && axis(centers[start], dimension) < split) start++;
        // the last element on the left of the plane
        while (start < end && axis(centers[end - 1], dimension) >= split) end--;
        if (start < end) {
            const NodeId id = ids[start];
            ids[start] = ids[end - 1];
            ids[end - 1] = id;
            const Vec3f center = centers[start];
            centers[start] = centers[end - 1];
            centers[end - 1] = center;
            start++;
            end--;
        }
    }
    CC_ASSERT(start == end);

    // a failed split halves by count
    mid_point = start > 0 && start < number ? start : number / 2;
}

/** sorts ids[begin, end) spatially into four groups, each from split[i] to split[i + 1] */
static void partition4(NodeId *ids, Vec3f *centers, int32_t begin, int32_t end, int32_t *split) {
    NodeId *node_ids = ids + begin;
    Vec3f *node_centers = centers + begin;
    const int32_t number = end - begin;

    partition(node_ids, node_centers, number, split[2]);
    partition(node_ids, node_centers, split[2], split[1]);
    partition(node_ids + split[2], node_centers + split[2], number - split[2], split[3]);

    split[0] = begin;
    split[1] += begin;
    split[2] += begin;
    split[3] += split[2];
    split[4] = end;
}

/** a tree over ids[0, number): the root's id (a body's own when number is 1), its bounds in out_bounds */
static NodeId build_tree(QuadTree &tree, const bodies::Bodies &bodies, int32_t number, uint32_t max_depth_mark_changed, AABox &out_bounds) {
    NodeId *ids = tree.build_ids.data;
    if (number == 0) {
        out_bounds = {vec3f::splat(LARGE_FLOAT), vec3f::splat(-LARGE_FLOAT)};
        return INVALID_NODE_ID;
    }
    if (number == 1) {
        // a node returned as the root has no parent
        if (is_node(ids[0])) tree.nodes[node_index(ids[0])].parent_node_index = INVALID_NODE_INDEX;
        out_bounds = get_node_or_body_bounds(tree, bodies, ids[0]);
        return ids[0];
    }

    // the centres of everything inserted
    collections::Vec<Vec3f> &centers_vec = tree.build_centers;
    centers_vec.length = 0;
    if (!centers_vec.reserve((uint32_t)number)) return INVALID_NODE_ID;
    centers_vec.length = (uint32_t)number;
    Vec3f *centers = centers_vec.data;
    for (int32_t i = 0; i < number; i++) centers[i] = aabox::center(get_node_or_body_bounds(tree, bodies, ids[i]));

    // the recursive build, on a stack of its own
    collections::Vec<BuildEntry> &stack = tree.build_stack;
    stack.length = 0;
    const Vec3f empty_min = vec3f::splat(LARGE_FLOAT);
    const Vec3f empty_max = vec3f::splat(-LARGE_FLOAT);
    BuildEntry root = {};
    root.node_index = allocate_node(tree, max_depth_mark_changed > 0);
    root.child_index = -1;
    root.depth = 0;
    root.node_bounds_min = empty_min;
    root.node_bounds_max = empty_max;
    partition4(ids, centers, 0, number, root.split);
    stack.push(root);

    for (;;) {
        const uint32_t top = stack.length - 1;
        BuildEntry &cur = stack[top];
        cur.child_index++;

        if (cur.child_index >= 4) {
            if (top == 0) break;

            // our bounds into the parent's, and this node into the parent's slot
            BuildEntry &prev = stack[top - 1];
            prev.node_bounds_min = vec3f::min(prev.node_bounds_min, cur.node_bounds_min);
            prev.node_bounds_max = vec3f::max(prev.node_bounds_max, cur.node_bounds_max);
            tree.nodes[cur.node_index].parent_node_index = prev.node_index;
            Node &parent = tree.nodes[prev.node_index];
            parent.child_node_id[prev.child_index] = from_node_index(cur.node_index);
            set_child_bounds(parent, (uint32_t)prev.child_index, {cur.node_bounds_min, cur.node_bounds_max});
            stack.length--;
            continue;
        }

        const int32_t low = cur.split[cur.child_index];
        const int32_t high = cur.split[cur.child_index + 1];
        const int32_t num_bodies = high - low;

        if (num_bodies == 1) {
            const NodeId child = ids[low];
            const AABox b = get_node_or_body_bounds(tree, bodies, child);

            Node &node = tree.nodes[cur.node_index];
            node.child_node_id[cur.child_index] = child;
            set_child_bounds(node, (uint32_t)cur.child_index, b);
            if (is_node(child)) {
                tree.nodes[node_index(child)].parent_node_index = cur.node_index;
            } else {
                set_body_location(*bodies.pool[child], cur.node_index, (uint32_t)cur.child_index);
            }

            cur.node_bounds_min = vec3f::min(cur.node_bounds_min, b.min);
            cur.node_bounds_max = vec3f::max(cur.node_bounds_max, b.max);
        } else if (num_bodies > 1) {
            const uint32_t next_depth = cur.depth + 1;
            BuildEntry next = {};
            next.node_index = allocate_node(tree, max_depth_mark_changed > next_depth);
            next.child_index = -1;
            next.depth = next_depth;
            next.node_bounds_min = empty_min;
            next.node_bounds_max = empty_max;
            partition4(ids, centers, low, high, next.split);
            // cur is not used past here: the push may move the stack
            stack.push(next);
        }
    }

    out_bounds = {stack[0].node_bounds_min, stack[0].node_bounds_max};
    return from_node_index(stack[0].node_index);
}

/** jolt's UpdatePrepare: the new root, built from the bodies and the unchanged subtrees */
static NodeId update_prepare(QuadTree &tree, const bodies::Bodies &bodies, bool full_rebuild) {
    CC_ASSERT(tree.free_node_batch.length == 0);
    tree.dirty = false;

    // collect the bodies and the unchanged subtrees, freeing the changed nodes into the batch
    collections::Vec<NodeId> &ids = tree.build_ids;
    collections::Vec<NodeId> &stack = tree.node_stack;
    ids.length = 0;
    stack.length = 0;
    stack.push(from_node_index(tree.root_node));
    do {
        const NodeId id = stack.data[--stack.length];
        if (is_body(id)) {
            ids.push(id);
            continue;
        }
        const uint32_t index = node_index(id);
        const Node &node = tree.nodes[index];
        if (!node.is_changed && !full_rebuild) {
            // unchanged: one unit
            ids.push(id);
        } else {
            for (uint32_t i = 0; i < 4; i++) {
                if (is_valid(node.child_node_id[i])) stack.push(node.child_node_id[i]);
            }
            tree.free_node_batch.push(index);
        }
    } while (stack.length > 0);

    CC_ASSERT(full_rebuild ? ids.length == tree.num_bodies : ids.length <= tree.num_bodies);

    if (ids.length == 0) return from_node_index(allocate_node(tree, false));

    AABox root_bounds;
    NodeId root_id = build_tree(tree, bodies, (int32_t)ids.length, MAX_DEPTH_MARK_CHANGED, root_bounds);
    if (is_body(root_id)) {
        // a single body needs a root node over it
        const uint32_t root_index = allocate_node(tree, false);
        Node &root = tree.nodes[root_index];
        set_child_bounds(root, 0, root_bounds);
        root.child_node_id[0] = root_id;
        set_body_location(*bodies.pool[root_id], root_index, 0);
        root_id = from_node_index(root_index);
    }
    return root_id;
}

/** jolt's UpdateFinalize: the new tree becomes the current one, and the nodes it replaced are free */
static void update_finalize(QuadTree &tree, NodeId root_id) {
    tree.root_node = node_index(root_id);
    for (uint32_t i = 0; i < tree.free_node_batch.length; i++) tree.free_nodes.push(tree.free_node_batch[i]);
    tree.free_node_batch.length = 0;
}

void rebuild(QuadTree &tree, const bodies::Bodies &bodies) { update_finalize(tree, update_prepare(tree, bodies, false)); }

// ---- walks ----

/** grows a walk's stack keeping what it holds: walks index past length */
template <typename T>
static bool grow_keeping(collections::Vec<T> &stack, uint32_t wanted) {
    if (wanted <= stack.capacity) return true;
    return stack.grow_preserving(wanted < 128 ? 128 : (wanted > stack.capacity * 2 ? wanted : stack.capacity * 2));
}

/** room for four more entries above top on the walk's stack */
static NodeId *reserve_stack(QuadTree &tree, uint32_t top) {
    if (!grow_keeping(tree.node_stack, top + 5)) return nullptr;
    return tree.node_stack.data;
}

/** the closed overlap of the query box with the four children: a lane mask in the low four bits */
static inline uint32_t overlap4(const Node &node, v128_t query_min_x, v128_t query_min_y, v128_t query_min_z, v128_t query_max_x, v128_t query_max_y,
                                v128_t query_max_z) {
    const v128_t x = wasm_v128_and(v4f::le(v4f::load4(node.bounds_min_x), query_max_x), v4f::ge(v4f::load4(node.bounds_max_x), query_min_x));
    const v128_t y = wasm_v128_and(v4f::le(v4f::load4(node.bounds_min_y), query_max_y), v4f::ge(v4f::load4(node.bounds_max_y), query_min_y));
    const v128_t z = wasm_v128_and(v4f::le(v4f::load4(node.bounds_min_z), query_max_z), v4f::ge(v4f::load4(node.bounds_max_z), query_min_z));
    return v4f::bitmask(wasm_v128_and(wasm_v128_and(x, y), z));
}

/** whether the point lies in each of the four children, faces included */
static inline uint32_t contains4(const Node &node, v128_t px, v128_t py, v128_t pz) {
    const v128_t x = wasm_v128_and(v4f::ge(px, v4f::load4(node.bounds_min_x)), v4f::le(px, v4f::load4(node.bounds_max_x)));
    const v128_t y = wasm_v128_and(v4f::ge(py, v4f::load4(node.bounds_min_y)), v4f::le(py, v4f::load4(node.bounds_max_y)));
    const v128_t z = wasm_v128_and(v4f::ge(pz, v4f::load4(node.bounds_min_z)), v4f::le(pz, v4f::load4(node.bounds_max_z)));
    return v4f::bitmask(wasm_v128_and(wasm_v128_and(x, y), z));
}

/**
 * the overlap walks: from the root, a node's children whose lanes the test sets are pushed in lane order, and the walk
 * pops from the top. leaf(body) returns false to stop the walk.
 */
template <typename Test, typename Leaf>
static void walk_tree(QuadTree &tree, const bodies::Bodies &bodies, Test test, Leaf leaf) {
    NodeId *stack = reserve_stack(tree, 0);
    if (stack == nullptr) return;
    int32_t top = 0;
    stack[0] = from_node_index(tree.root_node);
    do {
        const NodeId id = stack[top];
        if (is_body(id)) {
            if (!leaf(bodies.pool[id])) return;
        } else if (is_valid(id)) {
            stack = reserve_stack(tree, (uint32_t)top);
            if (stack == nullptr) return;
            const Node &node = tree.nodes[node_index(id)];
            uint32_t mask = test(node) & 0xfu;
            while (mask != 0) {
                const uint32_t lane = (uint32_t)__builtin_ctz(mask);
                mask &= mask - 1;
                stack[top++] = node.child_node_id[lane];
            }
        }
        --top;
    } while (top >= 0);
}

/**
 * a ray segment for the slab tests: origin, per-axis reciprocal of the direction (0 on an axis the
 * ray runs parallel to) and the segment's end in t.
 */
struct SlabRay {
    v128_t origin;
    v128_t inv;
    v128_t parallel;
    float t_max;
};

static inline SlabRay slab_ray(v128_t origin, v128_t inv, float t_max) { return {origin, inv, v4f::eq(inv, v4f::zero()), t_max}; }

/**
 * entry t of the segment into a box, in [0, t_max], or infinity when the segment misses the box or
 * the box lies entirely behind the origin. the three slabs side by side in the lanes.
 */
static inline float slab_entry(const SlabRay &ray, v128_t min, v128_t max) {
    // parallel to a slab: the segment stays at the origin there, which must lie inside it
    const uint32_t outside = v4f::bitmask(v4f::lt(ray.origin, min)) | v4f::bitmask(v4f::gt(ray.origin, max));
    if (outside & v4f::bitmask(ray.parallel) & 7u) return scalar::INF;
    const v128_t t0 = v4f::mul(v4f::sub(min, ray.origin), ray.inv);
    const v128_t t1 = v4f::mul(v4f::sub(max, ray.origin), ray.inv);
    const v128_t ordered = v4f::lt(t0, t1);
    const v128_t near = v4f::select(v4f::zero(), v4f::select(t0, t1, ordered), ray.parallel);
    const v128_t far = v4f::select(v4f::splat(ray.t_max), v4f::select(t1, t0, ordered), ray.parallel);
    const float t_min = scalar::max(scalar::max(0.0f, v4f::lane<0>(near)), scalar::max(v4f::lane<1>(near), v4f::lane<2>(near)));
    const float t_max = scalar::min(scalar::min(ray.t_max, v4f::lane<0>(far)), scalar::min(v4f::lane<1>(far), v4f::lane<2>(far)));
    return t_max < t_min ? scalar::INF : t_min;
}

/** reciprocal of a ray displacement component, computed once per query; (nearly) zero marks the axis parallel. */
static inline float safe_reciprocal(float d) { return d > 1e-30f || d < -1e-30f ? 1.0f / d : 0.0f; }

/** the query filter's group/mask, object layer and body filter tests on a body */
static bool filter_body(const filter::Filter &query_filter, const RigidBody *body) {
    if (!filter::should_pair_collide(query_filter.collision_groups, query_filter.collision_mask, body->collision_groups, body->collision_mask)) {
        return false;
    }
    if (!filter::filter_object_layer(query_filter, body->object_layer)) return false;
    if (query_filter.body_filter && !query_filter.body_filter(body, query_filter.body_filter_context)) return false;
    return true;
}

void intersect_aabb_fat_leaves(const bodies::Bodies &bodies, QuadTree &tree, const Box3f &aabb, BodyVisitor &visitor) {
    if (tree.num_bodies == 0) return;
    const v128_t min_x = v4f::splat(aabb.min.x), min_y = v4f::splat(aabb.min.y), min_z = v4f::splat(aabb.min.z);
    const v128_t max_x = v4f::splat(aabb.max.x), max_y = v4f::splat(aabb.max.y), max_z = v4f::splat(aabb.max.z);
    walk_tree(
        tree, bodies, [&](const Node &node) { return overlap4(node, min_x, min_y, min_z, max_x, max_y, max_z); },
        [&](RigidBody *body) {
            visitor.visit(&visitor, body);
            return !visitor.should_exit;
        });
}

void intersect_aabb(const bodies::Bodies &bodies, QuadTree &tree, const Box3f &aabb, const filter::Filter &query_filter, BodyVisitor &visitor) {
    if (tree.num_bodies == 0) return;
    const v128_t min_x = v4f::splat(aabb.min.x), min_y = v4f::splat(aabb.min.y), min_z = v4f::splat(aabb.min.z);
    const v128_t max_x = v4f::splat(aabb.max.x), max_y = v4f::splat(aabb.max.y), max_z = v4f::splat(aabb.max.z);
    const AABox query = aabox::load(aabb);
    walk_tree(
        tree, bodies, [&](const Node &node) { return overlap4(node, min_x, min_y, min_z, max_x, max_y, max_z); },
        [&](RigidBody *body) {
            if (!filter_body(query_filter, body)) return true;
            // the fat box overlapped: the body's own box must too
            if (!aabox::intersects(aabox::load(body->aabb), query)) return true;
            visitor.visit(&visitor, body);
            return !visitor.should_exit;
        });
}

void intersect_point(const bodies::Bodies &bodies, QuadTree &tree, Vec3f point, const filter::Filter &query_filter, BodyVisitor &visitor) {
    if (tree.num_bodies == 0) return;
    const v128_t px = v4f::splat(vec3f::x(point)), py = v4f::splat(vec3f::y(point)), pz = v4f::splat(vec3f::z(point));
    walk_tree(
        tree, bodies, [&](const Node &node) { return contains4(node, px, py, pz); },
        [&](RigidBody *body) {
            if (!filter_body(query_filter, body)) return true;
            if (!aabox::contains_point(aabox::load(body->aabb), point)) return true;
            visitor.visit(&visitor, body);
            return !visitor.should_exit;
        });
}

void walk(const bodies::Bodies &bodies, QuadTree &tree, BodyVisitor &visitor) {
    if (tree.num_bodies == 0) return;
    walk_tree(
        tree, bodies,
        [](const Node &node) {
            uint32_t mask = 0;
            for (uint32_t i = 0; i < 4; i++) mask |= is_valid(node.child_node_id[i]) ? 1u << i : 0u;
            return mask;
        },
        [&](RigidBody *body) {
            visitor.visit(&visitor, body);
            return !visitor.should_exit;
        });
}

/**
 * slab_entry for the four children at once: each lane's entry t of the segment into the child grown by
 * expand, in [0, t_max], or infinity when it misses, lies behind the origin, or the child is empty
 */
static inline v128_t slab_entry4(const SlabRay &ray, v128_t expand, const Node &node) {
    const v128_t ex = v4f::shuffle<0, 0, 0, 0>(expand), ey = v4f::shuffle<1, 1, 1, 1>(expand), ez = v4f::shuffle<2, 2, 2, 2>(expand);
    const v128_t min_x = v4f::sub(v4f::load4(node.bounds_min_x), ex), max_x = v4f::add(v4f::load4(node.bounds_max_x), ex);
    const v128_t min_y = v4f::sub(v4f::load4(node.bounds_min_y), ey), max_y = v4f::add(v4f::load4(node.bounds_max_y), ey);
    const v128_t min_z = v4f::sub(v4f::load4(node.bounds_min_z), ez), max_z = v4f::add(v4f::load4(node.bounds_max_z), ez);
    const v128_t seg_max = v4f::splat(ray.t_max);
    const v128_t zero = v4f::zero();

    v128_t miss = wasm_v128_or(wasm_v128_or(v4f::gt(min_x, max_x), v4f::gt(min_y, max_y)), v4f::gt(min_z, max_z));
    v128_t near = zero;
    v128_t far = seg_max;
    const auto slab = [&](v128_t origin, v128_t inv, v128_t parallel, v128_t lo, v128_t hi) {
        // parallel to the slab: the segment stays at the origin there, which must lie inside it
        miss = wasm_v128_or(miss, wasm_v128_and(parallel, wasm_v128_or(v4f::lt(origin, lo), v4f::gt(origin, hi))));
        const v128_t t0 = v4f::mul(v4f::sub(lo, origin), inv);
        const v128_t t1 = v4f::mul(v4f::sub(hi, origin), inv);
        const v128_t ordered = v4f::lt(t0, t1);
        near = v4f::max(near, v4f::select(zero, v4f::select(t0, t1, ordered), parallel));
        far = v4f::min(far, v4f::select(seg_max, v4f::select(t1, t0, ordered), parallel));
    };
    slab(v4f::shuffle<0, 0, 0, 0>(ray.origin), v4f::shuffle<0, 0, 0, 0>(ray.inv), v4f::shuffle<0, 0, 0, 0>(ray.parallel), min_x, max_x);
    slab(v4f::shuffle<1, 1, 1, 1>(ray.origin), v4f::shuffle<1, 1, 1, 1>(ray.inv), v4f::shuffle<1, 1, 1, 1>(ray.parallel), min_y, max_y);
    slab(v4f::shuffle<2, 2, 2, 2>(ray.origin), v4f::shuffle<2, 2, 2, 2>(ray.inv), v4f::shuffle<2, 2, 2, 2>(ray.parallel), min_z, max_z);
    miss = wasm_v128_or(miss, v4f::lt(far, near));
    return v4f::select(v4f::splat(scalar::INF), near, miss);
}

/**
 * the closest-first walk the two casts share (jolt's ray visitor over WalkTree): a node's children sorted furthest
 * first onto the stack with their entry fractions, so the closest pops first, and any entry at or past the closest hit
 * so far is culled. children are grown by expand (a cast shape's half extents, zero for a ray), and the body's own box
 * is tested at the leaf.
 */
static void cast_walk(const bodies::Bodies &bodies, QuadTree &tree, const SlabRay &ray, v128_t expand, const filter::Filter &query_filter,
                      BodyVisitor &visitor) {
    NodeId *stack = reserve_stack(tree, 0);
    if (stack == nullptr || !grow_keeping(tree.fraction_stack, tree.node_stack.capacity)) return;
    float *fractions = tree.fraction_stack.data;

    float best_fraction = visitor.early_out_fraction;
    int32_t top = 0;
    stack[0] = from_node_index(tree.root_node);
    fractions[0] = -1.0f;
    do {
        const NodeId id = stack[top];
        if (is_body(id)) {
            RigidBody *body = bodies.pool[id];
            if (filter_body(query_filter, body) &&
                slab_entry(ray, v4f::sub(v4f::load3(&body->aabb.min.x), expand), v4f::add(v4f::load3(&body->aabb.max.x), expand)) != scalar::INF) {
                visitor.visit(&visitor, body);
                if (visitor.should_exit) return;
                // a hit may have shrunk the collector's early-out fraction
                best_fraction = visitor.early_out_fraction;
            }
        } else if (is_valid(id)) {
            stack = reserve_stack(tree, (uint32_t)top);
            if (stack == nullptr || !grow_keeping(tree.fraction_stack, tree.node_stack.capacity)) return;
            fractions = tree.fraction_stack.data;
            const Node &node = tree.nodes[node_index(id)];
            const v128_t entry = slab_entry4(ray, expand, node);
            float lane_fraction[4];
            v4f::store4(lane_fraction, entry);

            // furthest first, keeping only those closer than the best hit
            uint32_t count = 0;
            for (uint32_t lane = 0; lane < 4; lane++) {
                const float f = lane_fraction[lane];
                if (!(f < best_fraction)) continue;
                uint32_t at = count++;
                while (at > 0 && fractions[top + at - 1] < f) {
                    fractions[top + at] = fractions[top + at - 1];
                    stack[top + at] = stack[top + at - 1];
                    at--;
                }
                fractions[top + at] = f;
                stack[top + at] = node.child_node_id[lane];
            }
            top += (int32_t)count;
        }

        // the next entry that can still beat the best hit
        do {
            --top;
        } while (top >= 0 && !(fractions[top] < best_fraction));
    } while (top >= 0);
}

void cast_ray(const bodies::Bodies &bodies, QuadTree &tree, const cast_ray::Rayf &ray, const filter::Filter &query_filter, BodyVisitor &visitor) {
    if (tree.num_bodies == 0) return;

    // reciprocals of the displacement, once per query, for the fraction-space slab tests
    const SlabRay slab = slab_ray(
        ray.origin.v,
        v4f::make(safe_reciprocal(vec3f::x(ray.direction) * ray.length), safe_reciprocal(vec3f::y(ray.direction) * ray.length),
                  safe_reciprocal(vec3f::z(ray.direction) * ray.length), 0.0f),
        1.0f);
    cast_walk(bodies, tree, slab, v4f::zero(), query_filter, visitor);
}

void cast_aabb(const bodies::Bodies &bodies, QuadTree &tree, const Box3f &cast_bounds, Vec3f displacement, const filter::Filter &query_filter,
               BodyVisitor &visitor) {
    if (tree.num_bodies == 0) return;

    // the box shrunk to its centre and every child grown by its half extents, then cast as a point
    const v128_t cast_min = v4f::load3(&cast_bounds.min.x);
    const v128_t cast_max = v4f::load3(&cast_bounds.max.x);
    const v128_t origin = v4f::mul(v4f::add(cast_min, cast_max), v4f::splat(0.5f));
    const v128_t half = v4f::mul(v4f::sub(cast_max, cast_min), v4f::splat(0.5f));

    // reciprocals of the displacement, once per query, for the fraction-space slab tests; a still cast is an overlap test,
    // its entries 0 or infinity
    const SlabRay ray = slab_ray(origin,
                                 v4f::make(safe_reciprocal(vec3f::x(displacement)), safe_reciprocal(vec3f::y(displacement)),
                                           safe_reciprocal(vec3f::z(displacement)), 0.0f),
                                 1.0f);
    cast_walk(bodies, tree, ray, half, query_filter, visitor);
}

}  // namespace quad_tree
