#include "tumble/physics/manifold/manifold.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "test.h"
#include "test_math.h"

namespace {

face::Face make_face(const float *vertices, uint32_t num_vertices) {
    face::Face face;
    face.num_vertices = num_vertices;
    for (uint32_t i = 0; i < num_vertices; i++) face.vertices[i] = vec3f::load(vertices, i * 3);
    return face;
}

const float UNIT_SQUARE_Z0[] = {-0.5f, -0.5f, 0, 0.5f, -0.5f, 0, 0.5f, 0.5f, 0, -0.5f, 0.5f, 0};
const float UNIT_SQUARE_Z01[] = {-0.5f, -0.5f, 0.1f, 0.5f, -0.5f, 0.1f, 0.5f, 0.5f, 0.1f, -0.5f, 0.5f, 0.1f};

/** whether some point of the manifold is (a, b) in world space, a on shape a and b on shape b */
bool has_point(const manifold::ContactManifold &m, const Float3 &a, const Float3 &b) {
    for (uint32_t i = 0; i < m.num_contact_points; i++) {
        const Vec3f on_a = manifold::get_world_space_contact_point_on_a(m, i);
        const Vec3f on_b = manifold::get_world_space_contact_point_on_b(m, i);
        if (near3(on_a, a.x, a.y, a.z, 1e-5f) && near3(on_b, b.x, b.y, b.z, 1e-5f)) return true;
    }
    return false;
}

/** sets point i on both sides, b displaced along +z by depth */
void set_point(manifold::ContactManifold &m, uint32_t i, float x, float y, float depth) {
    m.relative_contact_points_on_a[i] = vec3f::make(x, y, 0);
    m.relative_contact_points_on_b[i] = vec3f::make(x, y, depth);
}

}  // namespace

void test_manifold_between_two_faces_aligned_squares(TestCtx &ctx) {
    manifold::ContactManifold m = manifold::create();
    const face::Face face_1 = make_face(UNIT_SQUARE_Z0, 4);
    const face::Face face_2 = make_face(UNIT_SQUARE_Z01, 4);

    manifold::manifold_between_two_faces(m, vec3f::make(0, 0, 0), vec3f::make(0, 0, 0.1f), vec3f::make(0, 0, 0.1f), 0.2f, face_1, face_2);

    // the four corners, each projected from face 2 back onto face 1
    CHECK(ctx, m.num_contact_points == 4);
    for (uint32_t i = 0; i < 4; i++) {
        const float x = UNIT_SQUARE_Z0[i * 3], y = UNIT_SQUARE_Z0[i * 3 + 1];
        CHECK(ctx, has_point(m, {x, y, 0}, {x, y, 0.1f}));
    }

    // relative to the base offset
    m = manifold::create();
    m.base_offset = vec3f::make(10, 20, 30);
    manifold::manifold_between_two_faces(m, vec3f::make(0, 0, 0), vec3f::make(0, 0, 0.1f), vec3f::make(0, 0, 0.1f), 0.2f, face_1, face_2);
    CHECK(ctx, m.num_contact_points == 4);
    for (uint32_t i = 0; i < m.num_contact_points; i++) {
        const Vec3f relative = m.relative_contact_points_on_a[i];
        CHECK_NEAR(ctx, scalar::abs(vec3f::x(relative) + 10.0f), 0.5f, 1e-4f);
        CHECK_NEAR(ctx, scalar::abs(vec3f::y(relative) + 20.0f), 0.5f, 1e-4f);
        CHECK_NEAR(ctx, vec3f::z(relative), -30.0f, 1e-4f);
    }
    CHECK(ctx, near3(m.base_offset, 10, 20, 30, 0.0f));
}

void test_manifold_between_two_faces_rotated_square_octagon(TestCtx &ctx) {
    manifold::ContactManifold m = manifold::create();
    const face::Face face_1 = make_face(UNIT_SQUARE_Z0, 4);
    const float s = scalar::sqrt(2.0f) / 2.0f;
    const float diamond[] = {0, -s, 0.1f, s, 0, 0.1f, 0, s, 0.1f, -s, 0, 0.1f};
    const face::Face face_2 = make_face(diamond, 4);

    manifold::manifold_between_two_faces(m, vec3f::make(0, 0, 0), vec3f::make(0, 0, 0.1f), vec3f::make(0, 0, 0.1f), 0.2f, face_1, face_2);

    // the octagon of the square clipped by the diamond, every point 0.1 apart along the axis
    CHECK(ctx, m.num_contact_points == 8);
    for (uint32_t i = 0; i < m.num_contact_points; i++) {
        CHECK_NEAR(ctx, vec3f::z(m.relative_contact_points_on_a[i]), 0.0f, 1e-6f);
        CHECK_NEAR(ctx, vec3f::z(m.relative_contact_points_on_b[i]), 0.1f, 1e-6f);
    }

    // the ts's case: a diamond inscribed in the square, its vertices on the square's edges, each kept on one plane
    // and re-entered at the next, so every vertex comes out twice
    const float inscribed[] = {0, -0.5f, 0.1f, 0.5f, 0, 0.1f, 0, 0.5f, 0.1f, -0.5f, 0, 0.1f};
    manifold::manifold_between_two_faces(m, vec3f::make(0, 0, 0), vec3f::make(0, 0, 0.1f), vec3f::make(0, 0, 0.1f), 0.2f, face_1, make_face(inscribed, 4));
    CHECK(ctx, m.num_contact_points == 8);
    for (uint32_t i = 0; i < m.num_contact_points; i++) {
        const Vec3f p = m.relative_contact_points_on_b[i];
        CHECK_NEAR(ctx, scalar::abs(vec3f::x(p)) + scalar::abs(vec3f::y(p)), 0.5f, 1e-6f);
    }
}

void test_manifold_between_two_faces_edge_vs_square(TestCtx &ctx) {
    manifold::ContactManifold m = manifold::create();
    const float edge[] = {0, -0.5f, 0, 0, 0.5f, 0};
    const face::Face face_1 = make_face(edge, 2);
    const face::Face face_2 = make_face(UNIT_SQUARE_Z01, 4);

    manifold::manifold_between_two_faces(m, vec3f::make(0, 0, 0), vec3f::make(0, 0, 0.1f), vec3f::make(0, 0, 0.1f), 0.2f, face_1, face_2);

    // the edge's ends, where it crosses the square's boundary, projected back onto the edge
    CHECK(ctx, m.num_contact_points == 2);
    CHECK(ctx, has_point(m, {0, -0.5f, 0}, {0, -0.5f, 0.1f}));
    CHECK(ctx, has_point(m, {0, 0.5f, 0}, {0, 0.5f, 0.1f}));
}

void test_manifold_between_two_faces_fallbacks(TestCtx &ctx) {
    manifold::ContactManifold m = manifold::create();
    const face::Face square_1 = make_face(UNIT_SQUARE_Z0, 4);
    const face::Face square_2 = make_face(UNIT_SQUARE_Z01, 4);
    const Float3 point_1 = {0, 0, 0}, point_2 = {0, 0, 0.1f};

    // disjoint faces: the contact points
    const float far_square[] = {10, 10, 0.1f, 11, 10, 0.1f, 11, 11, 0.1f, 10, 11, 0.1f};
    manifold::manifold_between_two_faces(m, vec3f::load(point_1), vec3f::load(point_2), vec3f::make(0, 0, 0.1f), 0.2f, square_1, make_face(far_square, 4));
    CHECK(ctx, m.num_contact_points == 1 && has_point(m, point_1, point_2));

    // every point deeper than the max contact distance: the contact points
    manifold::manifold_between_two_faces(m, vec3f::load(point_1), vec3f::load(point_2), vec3f::make(0, 0, 0.1f), 0.05f, square_1, square_2);
    CHECK(ctx, m.num_contact_points == 1 && has_point(m, point_1, point_2));

    // face 1 a single vertex
    manifold::manifold_between_two_faces(m, vec3f::load(point_1), vec3f::load(point_2), vec3f::make(0, 0, 0.1f), 0.2f, make_face(UNIT_SQUARE_Z0, 1), square_2);
    CHECK(ctx, m.num_contact_points == 1 && has_point(m, point_1, point_2));

    // a penetration axis in face 1's plane cannot project back onto it
    manifold::manifold_between_two_faces(m, vec3f::load(point_1), vec3f::load(point_2), vec3f::make(1, 0, 0), 0.2f, square_1, square_2);
    CHECK(ctx, m.num_contact_points == 1 && has_point(m, point_1, point_2));
}

void test_manifold_between_two_faces_square_vs_edge_swaps(TestCtx &ctx) {
    // face 1 a square, face 2 an edge from the square's centre to beyond its +x side: the faces swap so the square is
    // clipped against the edge, and the points come back on their own sides
    manifold::ContactManifold m = manifold::create();
    const float edge_2[] = {0, 0, 0.1f, 1, 0, 0.1f};
    manifold::manifold_between_two_faces(m, vec3f::make(0, 0, 0), vec3f::make(0, 0, 0.1f), vec3f::make(0, 0, 0.1f), 0.2f, make_face(UNIT_SQUARE_Z0, 4),
                                         make_face(edge_2, 2));

    // the part of the edge over the square, x in [0, 0.5]: its ends on the edge (b) and below them on the square (a)
    CHECK(ctx, m.num_contact_points == 2);
    CHECK(ctx, has_point(m, {0, 0, 0}, {0, 0, 0.1f}));
    CHECK(ctx, has_point(m, {0.5f, 0, 0}, {0.5f, 0, 0.1f}));
}

void test_manifold_capsule_along_box_top(TestCtx &ctx) {

    // a 2 m box at the origin, and a capsule (radius 0.5, cylinder half height 0.5) turned to lie along x, its bottom
    // line sunk 0.1 into the box's top face
    shapes::Shape *box_shape = box::create({.half_extents = {1, 1, 1}});
    shapes::Shape *capsule_shape = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.5f});
    Float4 capsule_rotation = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI / 2.0f);

    // penetration axis from the box (shape 1) into the capsule (shape 2): the box's face against -axis is its top,
    // the capsule's face against +axis (in its local space, +x) is its bottom line
    shapes::SupportingFaceResult io;
    io.transform = mat4f::identity();
    io.scale = vec3f::make(1, 1, 1);
    box::get_supporting_face(io, vec3f::make(0, -1, 0), box_shape, sub_shape::EMPTY_SUB_SHAPE_ID);
    const face::Face box_face = io.face;
    io.transform = mat4f::from_rotation_translation(quatf::load(capsule_rotation), vec3f::make(0, 1.4f, 0));
    capsule::get_supporting_face(io, vec3f::make(1, 0, 0), capsule_shape, sub_shape::EMPTY_SUB_SHAPE_ID);
    const face::Face capsule_face = io.face;
    CHECK(ctx, box_face.num_vertices == 4 && capsule_face.num_vertices == 2);

    manifold::ContactManifold m = manifold::create();
    m.base_offset = vec3f::make(0, 1, 0);
    manifold::manifold_between_two_faces(m, vec3f::make(0, 1, 0), vec3f::make(0, 0.9f, 0), vec3f::make(0, 1, 0), 0.02f, box_face, capsule_face);

    // the resting line's ends x = +-0.5 on the capsule (y = 0.9), each with its point 0.1 above on the box's top
    CHECK(ctx, m.num_contact_points == 2);
    CHECK(ctx, has_point(m, {-0.5f, 1, 0}, {-0.5f, 0.9f, 0}));
    CHECK(ctx, has_point(m, {0.5f, 1, 0}, {0.5f, 0.9f, 0}));

    box::destroy(box_shape);
    capsule::destroy(capsule_shape);
}

void test_manifold_two_boxes_face_to_face(TestCtx &ctx) {

    // a 2 m box at the origin, a 1 m box on it, 0.25 off centre in x and sunk 0.1 into its top face
    shapes::Shape *box_a = box::create({.half_extents = {1, 1, 1}});
    shapes::Shape *box_b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    const Float3 position_b = {0.25f, 1.4f, 0};
    const Float3 penetration_axis = {0, 1, 0};

    // a's face towards b (the face most against -axis is its top) and b's face towards a
    shapes::SupportingFaceResult io;
    io.transform = mat4f::identity();
    io.scale = vec3f::make(1, 1, 1);
    box::get_supporting_face(io, vec3f::make(0, -1, 0), box_a, sub_shape::EMPTY_SUB_SHAPE_ID);
    const face::Face face_a = io.face;
    io.transform = translation(vec3f::load(position_b));
    box::get_supporting_face(io, vec3f::make(0, 1, 0), box_b, sub_shape::EMPTY_SUB_SHAPE_ID);
    const face::Face face_b = io.face;
    CHECK(ctx, face_a.num_vertices == 4 && face_b.num_vertices == 4);

    manifold::ContactManifold m = manifold::create();
    m.base_offset = vec3f::make(0, 1, 0);
    manifold::manifold_between_two_faces(m, vec3f::make(0.25f, 1, 0), vec3f::make(0.25f, 0.9f, 0), vec3f::load(penetration_axis), 0.02f, face_a, face_b);

    // b's bottom face lies wholly inside a's top face: its four corners on b, each lifted 0.1 onto a's top
    CHECK(ctx, m.num_contact_points == 4);
    const float xs[] = {-0.25f, 0.75f};
    const float zs[] = {-0.5f, 0.5f};
    for (uint32_t i = 0; i < 2; i++) {
        for (uint32_t j = 0; j < 2; j++) CHECK(ctx, has_point(m, {xs[i], 1, zs[j]}, {xs[i], 0.9f, zs[j]}));
    }

    box::destroy(box_a);
    box::destroy(box_b);
}

void test_manifold_edge_vs_edge_single_point(TestCtx &ctx) {
    // two crossing edges (a capsule across a capsule): neither face can clip, so the manifold is the contact pair
    const float edge_1[] = {-1, 0, 0, 1, 0, 0};
    const float edge_2[] = {0, -0.05f, -1, 0, -0.05f, 1};
    manifold::ContactManifold m = manifold::create();
    m.base_offset = vec3f::make(0, -0.025f, 0);
    manifold::manifold_between_two_faces(m, vec3f::make(0, 0, 0), vec3f::make(0, -0.05f, 0), vec3f::make(0, -1, 0), 0.1f, make_face(edge_1, 2), make_face(edge_2, 2));

    CHECK(ctx, m.num_contact_points == 1);
    CHECK(ctx, near3(m.relative_contact_points_on_a[0], 0, 0.025f, 0, 1e-5f));
    CHECK(ctx, near3(m.relative_contact_points_on_b[0], 0, -0.025f, 0, 1e-5f));
}

void test_manifold_swap_shapes(TestCtx &ctx) {
    manifold::ContactManifold m = manifold::create();
    m.num_contact_points = 2;
    m.relative_contact_points_on_a[0] = vec3f::make(1, 2, 3);
    m.relative_contact_points_on_a[1] = vec3f::make(4, 5, 6);
    m.relative_contact_points_on_b[0] = vec3f::make(-1, -2, -3);
    m.relative_contact_points_on_b[1] = vec3f::make(-4, -5, -6);
    m.world_space_normal = vec3f::make(0, 1, 0);
    m.sub_shape_id_a = 7;
    m.sub_shape_id_b = 9;
    m.material_id_a = 2;
    m.material_id_b = 5;

    manifold::swap_shapes(m);

    CHECK(ctx, near3(m.world_space_normal, 0, -1, 0, 0));
    CHECK(ctx, m.sub_shape_id_a == 9 && m.sub_shape_id_b == 7);
    CHECK(ctx, m.material_id_a == 5 && m.material_id_b == 2);
    CHECK(ctx, near3(m.relative_contact_points_on_a[0], -1, -2, -3, 0) && near3(m.relative_contact_points_on_a[1], -4, -5, -6, 0));
    CHECK(ctx, near3(m.relative_contact_points_on_b[0], 1, 2, 3, 0) && near3(m.relative_contact_points_on_b[1], 4, 5, 6, 0));
}

void test_manifold_prune_keeps_deepest_and_widest(TestCtx &ctx) {
    manifold::ContactManifold m = manifold::create();
    m.base_offset = vec3f::make(10, 20, 30);

    // the edge midpoints of a 2 m square first, then its corners, the first corner the deepest
    set_point(m, 0, 0, -1, 0.02f);
    set_point(m, 1, 1, 0, 0.02f);
    set_point(m, 2, 0, 1, 0.02f);
    set_point(m, 3, -1, 0, 0.02f);
    set_point(m, 4, -1, -1, 0.05f);
    set_point(m, 5, -1, 1, 0.02f);
    set_point(m, 6, 1, 1, 0.02f);
    set_point(m, 7, 1, -1, 0.02f);
    m.num_contact_points = 8;

    manifold::prune_contact_points(m, vec3f::make(0, 0, 1));

    // the deepest corner, then the opposite corner's sides in winding order: [p1, p3, p2, p4]
    CHECK(ctx, m.num_contact_points == 4);
    CHECK(ctx, near3(m.relative_contact_points_on_a[0], -1, -1, 0, 0) && near3(m.relative_contact_points_on_b[0], -1, -1, 0.05f, 0));
    CHECK(ctx, near3(m.relative_contact_points_on_a[1], -1, 1, 0, 0));
    CHECK(ctx, near3(m.relative_contact_points_on_a[2], 1, 1, 0, 0));
    CHECK(ctx, near3(m.relative_contact_points_on_a[3], 1, -1, 0, 0) && near3(m.relative_contact_points_on_b[3], 1, -1, 0.02f, 0));
    CHECK(ctx, near3(m.base_offset, 10, 20, 30, 0));
}

void test_manifold_prune_square_keeps_its_four_corners(TestCtx &ctx) {
    // the ts's 8-point square, corners first: p1 = 0 (the first deepest-widest), p2 = 2 opposite, p3 = 3 and p4 = 1
    // either side of the diagonal. kept as [p1, p3, p2, p4], copying point 3 into slot 1 must not lose point 1
    manifold::ContactManifold m = manifold::create();
    const float square[][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}, {0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    for (uint32_t i = 0; i < 8; i++) set_point(m, i, square[i][0], square[i][1], 0.02f);
    m.num_contact_points = 8;

    manifold::prune_contact_points(m, vec3f::make(0, 0, 1));

    CHECK(ctx, m.num_contact_points == 4);
    const float corners[][2] = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}};
    for (uint32_t i = 0; i < 4; i++) {
        CHECK(ctx, near3(m.relative_contact_points_on_a[i], corners[i][0], corners[i][1], 0, 0));
        CHECK(ctx, near3(m.relative_contact_points_on_b[i], corners[i][0], corners[i][1], 0.02f, 0));
    }
}

void test_manifold_prune_counts(TestCtx &ctx) {
    // the ts's cases: eight points of a square and six of a hexagon prune to four
    manifold::ContactManifold m = manifold::create();
    const float square[][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}, {0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    for (uint32_t i = 0; i < 8; i++) set_point(m, i, square[i][0], square[i][1], 0.02f);
    m.num_contact_points = 8;
    manifold::prune_contact_points(m, vec3f::make(0, 0, 1));
    CHECK(ctx, m.num_contact_points == 4);

    m = manifold::create();
    for (uint32_t i = 0; i < 6; i++) {
        const float angle = (float)i / 6.0f * 2.0f * scalar::PI;
        set_point(m, i, scalar::cos(angle) * 0.5f, scalar::sin(angle) * 0.5f, 0.01f);
    }
    m.num_contact_points = 6;
    manifold::prune_contact_points(m, vec3f::make(0, 0, 1));
    CHECK(ctx, m.num_contact_points == 4);

    // collinear points have nothing either side of the p1-p2 line: the two ends only
    m = manifold::create();
    for (uint32_t i = 0; i < 5; i++) set_point(m, i, -1.0f + 0.5f * (float)i, 0, 0.01f);
    m.num_contact_points = 5;
    manifold::prune_contact_points(m, vec3f::make(0, 0, 1));
    CHECK(ctx, m.num_contact_points == 2);
    const bool ends = (near3(m.relative_contact_points_on_a[0], -1, 0, 0, 0) && near3(m.relative_contact_points_on_a[1], 1, 0, 0, 0)) ||
                      (near3(m.relative_contact_points_on_a[0], 1, 0, 0, 0) && near3(m.relative_contact_points_on_a[1], -1, 0, 0, 0));
    CHECK(ctx, ends);
}
