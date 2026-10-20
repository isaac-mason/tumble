#include "tumble/physics/manifold/clip.h"

#include "tumble/math/scalar.h"
#include "test.h"
#include "test_math.h"

namespace {

face::Face make_face(const float *vertices, uint32_t num_vertices) {
    face::Face face;
    face.num_vertices = num_vertices;
    for (uint32_t i = 0; i < num_vertices; i++) face.vertices[i] = vec3f::load(vertices, i * 3);
    return face;
}

bool vertices_close(const face::Face &actual, const float *expected, uint32_t num_vertices) {
    if (actual.num_vertices != num_vertices) return false;
    float actual_floats[3 * face::FACE_MAX_VERTICES];
    flatten3(actual_floats, actual.vertices, num_vertices);
    for (uint32_t i = 0; i < num_vertices * 3; i++) {
        if (scalar::abs(actual_floats[i] - expected[i]) > 1e-6f) return false;
    }
    return true;
}

}  // namespace

void test_clip_poly_vs_plane_triangle_through_center(TestCtx &ctx) {
    const float triangle_vertices[] = {0, 0, 0, 2, 0, 0, 1, 2, 0};
    const face::Face triangle = make_face(triangle_vertices, 3);

    // plane at y = 1, normal pointing down: keeps y < 1
    face::Face out;
    clip::clip_poly_vs_plane(out, triangle, vec3f::make(0, 1, 0), vec3f::make(0, -1, 0));

    // a trapezoid: the clipped edges' intersections and the bottom edge
    const float expected[] = {0.5f, 1, 0, 0, 0, 0, 2, 0, 0, 1.5f, 1, 0};
    CHECK(ctx, vertices_close(out, expected, 4));

    // an unnormalised normal clips the same
    clip::clip_poly_vs_plane(out, triangle, vec3f::make(0, 1, 0), vec3f::make(0, -5, 0));
    CHECK(ctx, vertices_close(out, expected, 4));
}

void test_clip_poly_vs_plane_inside_and_outside(TestCtx &ctx) {
    const float triangle_vertices[] = {0, 0, 0, 1, 0, 0, 0.5f, 1, 0};
    const face::Face triangle = make_face(triangle_vertices, 3);
    face::Face out;

    // wholly inside y < 2: unchanged
    clip::clip_poly_vs_plane(out, triangle, vec3f::make(0, 2, 0), vec3f::make(0, -1, 0));
    CHECK(ctx, vertices_close(out, triangle_vertices, 3));

    // wholly outside y < -1: nothing
    clip::clip_poly_vs_plane(out, triangle, vec3f::make(0, -1, 0), vec3f::make(0, -1, 0));
    CHECK(ctx, out.num_vertices == 0);

    // fewer than two vertices: nothing
    const face::Face point = make_face(triangle_vertices, 1);
    clip::clip_poly_vs_plane(out, point, vec3f::make(0, 2, 0), vec3f::make(0, -1, 0));
    CHECK(ctx, out.num_vertices == 0);
}

void test_clip_poly_vs_plane_edge_parallel_to_plane(TestCtx &ctx) {
    // square whose bottom edge sits below y = 0 and runs parallel to the plane
    const float square_vertices[] = {0, -0.1f, 0, 1, -0.1f, 0, 1, 1, 0, 0, 1, 0};
    const face::Face square = make_face(square_vertices, 4);
    face::Face out;
    clip::clip_poly_vs_plane(out, square, vec3f::make(0, 0, 0), vec3f::make(0, -1, 0));

    // the bottom edge and the two crossings at y = 0
    const float expected[] = {0, 0, 0, 0, -0.1f, 0, 1, -0.1f, 0, 1, 0, 0};
    CHECK(ctx, vertices_close(out, expected, 4));
}

void test_clip_poly_vs_poly(TestCtx &ctx) {
    face::Face out;
    const Float3 normal = {0, 0, 1};

    // unit square against the 45 degree diamond of the same circumradius: an octagon, every vertex on both boundaries
    const float square_vertices[] = {-0.5f, -0.5f, 0, 0.5f, -0.5f, 0, 0.5f, 0.5f, 0, -0.5f, 0.5f, 0};
    const float s = scalar::sqrt(2.0f) / 2.0f;
    const float diamond_vertices[] = {-s, 0, 0, 0, -s, 0, s, 0, 0, 0, s, 0};
    const face::Face square = make_face(square_vertices, 4);
    const face::Face diamond = make_face(diamond_vertices, 4);
    clip::clip_poly_vs_poly(out, square, diamond, vec3f::load(normal));
    CHECK(ctx, out.num_vertices == 8);
    bool on_both = true;
    for (uint32_t i = 0; i < out.num_vertices; i++) {
        const float x = vec3f::x(out.vertices[i]), y = vec3f::y(out.vertices[i]);
        const bool on_square = scalar::abs(scalar::abs(x) - 0.5f) < 1e-5f || scalar::abs(scalar::abs(y) - 0.5f) < 1e-5f;
        const bool on_diamond = scalar::abs(scalar::abs(x) + scalar::abs(y) - s) < 1e-5f;
        on_both = on_both && on_square && on_diamond;
    }
    CHECK(ctx, on_both);

    // a small square inside a large one is unchanged
    const float small_vertices[] = {0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0};
    const float large_vertices[] = {-1, -1, 0, 2, -1, 0, 2, 2, 0, -1, 2, 0};
    const face::Face small = make_face(small_vertices, 4);
    const face::Face large = make_face(large_vertices, 4);
    clip::clip_poly_vs_poly(out, small, large, vec3f::load(normal));
    CHECK(ctx, out.num_vertices == 4);

    // disjoint squares leave nothing
    const float far_vertices[] = {5, 5, 0, 6, 5, 0, 6, 6, 0, 5, 6, 0};
    clip::clip_poly_vs_poly(out, small, make_face(far_vertices, 4), vec3f::load(normal));
    CHECK(ctx, out.num_vertices == 0);

    // [0,2]^2 against [1,3]^2 overlaps in [1,2]^2
    const float a_vertices[] = {0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0};
    const float b_vertices[] = {1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0};
    clip::clip_poly_vs_poly(out, make_face(a_vertices, 4), make_face(b_vertices, 4), vec3f::load(normal));
    CHECK(ctx, out.num_vertices == 4);
    bool in_overlap = true;
    for (uint32_t i = 0; i < out.num_vertices; i++) {
        const float x = vec3f::x(out.vertices[i]), y = vec3f::y(out.vertices[i]);
        in_overlap = in_overlap && x > 1 - 1e-6f && x < 2 + 1e-6f && y > 1 - 1e-6f && y < 2 + 1e-6f;
    }
    CHECK(ctx, in_overlap);

    // a clipping polygon of fewer than three vertices clips nothing
    const float line_vertices[] = {0.5f, 0, 0, 0.5f, 1, 0};
    clip::clip_poly_vs_poly(out, small, make_face(line_vertices, 2), vec3f::load(normal));
    CHECK(ctx, out.num_vertices == 0);
}

void test_clip_poly_vs_edge(TestCtx &ctx) {
    face::Face out;
    const float square_vertices[] = {0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0};
    const face::Face square = make_face(square_vertices, 4);

    // an edge above the square at x = 1 projects onto z = 0 and crosses the bottom and top edges
    clip::clip_poly_vs_edge(out, square, vec3f::make(1, 0, 5), vec3f::make(1, 2, 5), vec3f::make(0, 0, 1));
    const float expected[] = {1, 0, 0, 1, 2, 0};
    CHECK(ctx, vertices_close(out, expected, 2));

    // at x = 0.5 likewise
    clip::clip_poly_vs_edge(out, square, vec3f::make(0.5f, 0, 5), vec3f::make(0.5f, 2, 5), vec3f::make(0, 0, 1));
    const float expected_offset[] = {0.5f, 0, 0, 0.5f, 2, 0};
    CHECK(ctx, vertices_close(out, expected_offset, 2));

    // a short edge in the middle of a large square: the crossings clamp to the projected edge's ends
    const float large_vertices[] = {0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0};
    clip::clip_poly_vs_edge(out, make_face(large_vertices, 4), vec3f::make(2, 1.5f, 5), vec3f::make(2, 2.5f, 5), vec3f::make(0, 0, 1));
    const float expected_clamped[] = {2, 1.5f, 0, 2, 2.5f, 0};
    CHECK(ctx, vertices_close(out, expected_clamped, 2));

    // a polygon of fewer than three vertices clips nothing
    const float line_vertices[] = {0, 0, 0, 1, 0, 0};
    clip::clip_poly_vs_edge(out, make_face(line_vertices, 2), vec3f::make(0.5f, -1, 0), vec3f::make(0.5f, 1, 0), vec3f::make(-1, 0, 0));
    CHECK(ctx, out.num_vertices == 0);

    // a triangle: the outputs lie in the triangle's plane at x = 1
    const float triangle_vertices[] = {0, 0, 0, 2, 0, 0, 1, 2, 0};
    clip::clip_poly_vs_edge(out, make_face(triangle_vertices, 3), vec3f::make(1, 0, 5), vec3f::make(1, 2, 5), vec3f::make(0, 0, 1));
    CHECK(ctx, out.num_vertices >= 1);
    bool in_plane = true;
    for (uint32_t i = 0; i < out.num_vertices; i++) in_plane = in_plane && scalar::abs(vec3f::z(out.vertices[i])) < 1e-6f;
    CHECK(ctx, in_plane);
}
