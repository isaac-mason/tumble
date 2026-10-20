#include "tumble/physics/shapes/utils/triangle_mesh_data.h"

#include "tumble/instance.h"
#include "test.h"
#include "test_math.h"
#include "tumble/util/alloc.h"
#include "tumble/util/mem.h"

using triangle_mesh_data::TriangleMeshData;
using triangle_mesh_data::Triangle;

static uint32_t heap_bytes_in_use() {
    util::HeapStats stats;
    util::heap_stats(&stats);
    return stats.bytes_in_use;
}

/** two triangles over four vertices: a unit square in z = 0 and a tilted one above it */
static TriangleMeshData *make_two_triangles() {
    TriangleMeshData *data = triangle_mesh_data::create(4, 2, 0);
    const float positions[12] = {0, 0, 0, 2, 0, 0, 0, 3, 0, 1, -1, 4};
    util::copy(data->positions, positions, sizeof(positions));
    data->triangles[0] = {0, 1, 2, {0, 0, 1}, 0b111, 5};
    data->triangles[1] = {1, 0, 3, {0, -1, 0}, 0b010, -1};
    return data;
}

void test_triangle_mesh_data_layout(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    const uint32_t before = heap_bytes_in_use();

    TriangleMeshData *data = triangle_mesh_data::create(5, 3, 7);
    CHECK(ctx, data != nullptr);
    CHECK(ctx, data->vertex_count == 5 && data->triangle_count == 3 && data->node_count == 7);

    // one block: the header, then the arrays in order, each 16-aligned, none overlapping the next
    const uint32_t base = util::address_of(data);
    const uint32_t positions = util::address_of(data->positions);
    const uint32_t triangles = util::address_of(data->triangles);
    const uint32_t aabbs = util::address_of(data->triangle_aabbs);
    const uint32_t nodes = util::address_of(data->nodes);
    CHECK(ctx, (positions & 15) == 0 && (triangles & 15) == 0 && (aabbs & 15) == 0 && (nodes & 15) == 0);
    CHECK(ctx, positions >= base + sizeof(TriangleMeshData));
    CHECK(ctx, triangles >= positions + 5 * 3 * 4);
    CHECK(ctx, aabbs >= triangles + 3 * triangle_mesh_data::TRIANGLE_STRIDE * 4);
    CHECK(ctx, nodes >= aabbs + 3 * 6 * 4);
    CHECK(ctx, nodes + 7 * bvh::NODE_STRIDE * 4 == base + triangle_mesh_data::byte_size(5, 3, 7));
    CHECK(ctx, heap_bytes_in_use() - before == triangle_mesh_data::byte_size(5, 3, 7));

    // zeroed
    bool zeroed = true;
    for (uint32_t i = 0; i < 15; i++) zeroed = zeroed && data->positions[i] == 0.0f;
    for (uint32_t i = 0; i < 3; i++) zeroed = zeroed && data->triangles[i].index_a == 0 && data->triangles[i].active_edges == 0;
    for (uint32_t i = 0; i < 7; i++) zeroed = zeroed && data->nodes[i].axis_or_count == 0;
    CHECK(ctx, zeroed);

    // the interleaved words sit at the ts's offsets
    data->triangles[1] = {7, 8, 9, {0.5f, 0.25f, 0.125f}, 0b101, -3};
    const uint32_t *words = reinterpret_cast<const uint32_t *>(data->triangles) + triangle_mesh_data::TRIANGLE_STRIDE;
    const float *floats = reinterpret_cast<const float *>(words);
    CHECK(ctx, words[triangle_mesh_data::OFFSET_INDEX_A] == 7);
    CHECK(ctx, words[triangle_mesh_data::OFFSET_INDEX_C] == 9);
    CHECK(ctx, floats[triangle_mesh_data::OFFSET_NORMAL_Y] == 0.25f);
    CHECK(ctx, words[triangle_mesh_data::OFFSET_ACTIVE_EDGES] == 0b101);
    CHECK(ctx, (int32_t)words[triangle_mesh_data::OFFSET_MATERIAL_ID] == -3);

    triangle_mesh_data::destroy(data);
    CHECK(ctx, heap_bytes_in_use() == before);

    // an empty mesh is just the header
    TriangleMeshData *empty = triangle_mesh_data::create(0, 0, 0);
    CHECK(ctx, empty != nullptr && empty->triangle_count == 0 && empty->node_count == 0);
    triangle_mesh_data::destroy(empty);
    CHECK(ctx, heap_bytes_in_use() == before);
}

void test_triangle_mesh_data_accessors(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    TriangleMeshData *data = make_two_triangles();

    uint32_t indices[3];
    triangle_mesh_data::get_triangle_indices(indices, *data, 1);
    CHECK(ctx, indices[0] == 1 && indices[1] == 0 && indices[2] == 3);

    Vec3f a, b, c;
    triangle_mesh_data::get_triangle_vertices(a, b, c, *data, 1);
    CHECK(ctx, vec3f::exact_equals(a, vec3f::make(2, 0, 0)));
    CHECK(ctx, vec3f::exact_equals(b, vec3f::make(0, 0, 0)));
    CHECK(ctx, vec3f::exact_equals(c, vec3f::make(1, -1, 4)));

    Vec3f normal = triangle_mesh_data::get_triangle_normal(*data, 0);
    CHECK(ctx, vec3f::x(normal) == 0 && vec3f::y(normal) == 0 && vec3f::z(normal) == 1);
    triangle_mesh_data::set_triangle_normal(*data, 0, vec3f::make(1, 0, 0));
    normal = triangle_mesh_data::get_triangle_normal(*data, 0);
    CHECK(ctx, vec3f::x(normal) == 1 && vec3f::y(normal) == 0 && vec3f::z(normal) == 0);

    CHECK(ctx, triangle_mesh_data::get_active_edges(*data, 0) == 0b111);
    CHECK(ctx, triangle_mesh_data::get_active_edges(*data, 1) == 0b010);
    triangle_mesh_data::set_active_edges(*data, 1, 0b100);
    CHECK(ctx, triangle_mesh_data::get_active_edges(*data, 1) == 0b100);

    CHECK(ctx, triangle_mesh_data::get_material_id(*data, 0) == 5);
    CHECK(ctx, triangle_mesh_data::get_material_id(*data, 1) == -1);

    triangle_mesh_data::destroy(data);
}

void test_triangle_mesh_data_aabb_and_centroid(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    TriangleMeshData *data = make_two_triangles();

    Box3f aabb;
    triangle_mesh_data::calculate_triangle_aabb(aabb, *data, 0);
    CHECK(ctx, aabb.min.x == 0 && aabb.min.y == 0 && aabb.min.z == 0);
    CHECK(ctx, aabb.max.x == 2 && aabb.max.y == 3 && aabb.max.z == 0);
    triangle_mesh_data::calculate_triangle_aabb(aabb, *data, 1);
    CHECK(ctx, aabb.min.x == 0 && aabb.min.y == -1 && aabb.min.z == 0);
    CHECK(ctx, aabb.max.x == 2 && aabb.max.y == 0 && aabb.max.z == 4);

    Vec3f centroid = triangle_mesh_data::calculate_triangle_centroid(*data, 0);
    CHECK_NEAR(ctx, vec3f::x(centroid), 2.0f / 3.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(centroid), 1.0f, 1e-6f);
    CHECK(ctx, vec3f::z(centroid) == 0);
    centroid = triangle_mesh_data::calculate_triangle_centroid(*data, 1);
    CHECK_NEAR(ctx, vec3f::x(centroid), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(centroid), -1.0f / 3.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(centroid), 4.0f / 3.0f, 1e-6f);

    triangle_mesh_data::precompute_triangle_aabbs(*data);
    const float *aabbs = data->triangle_aabbs;
    CHECK(ctx, aabbs[0] == 0 && aabbs[1] == 0 && aabbs[2] == 0 && aabbs[3] == 2 && aabbs[4] == 3 && aabbs[5] == 0);
    CHECK(ctx, aabbs[6] == 0 && aabbs[7] == -1 && aabbs[8] == 0 && aabbs[9] == 2 && aabbs[10] == 0 && aabbs[11] == 4);

    triangle_mesh_data::destroy(data);
}

void test_triangle_mesh_data_swap(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    TriangleMeshData *data = make_two_triangles();

    // the whole stride moves; swapping with itself changes nothing
    triangle_mesh_data::swap_triangles(*data, 0, 0);
    CHECK(ctx, data->triangles[0].material_id == 5);

    triangle_mesh_data::swap_triangles(*data, 0, 1);
    const Triangle &first = data->triangles[0];
    const Triangle &second = data->triangles[1];
    CHECK(ctx, first.index_a == 1 && first.index_b == 0 && first.index_c == 3);
    CHECK(ctx, first.normal.y == -1 && first.active_edges == 0b010 && first.material_id == -1);
    CHECK(ctx, second.index_a == 0 && second.index_b == 1 && second.index_c == 2);
    CHECK(ctx, second.normal.z == 1 && second.active_edges == 0b111 && second.material_id == 5);

    triangle_mesh_data::destroy(data);
}
