#include "tumble/physics/shapes/compound.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/static_compound.h"
#include "test.h"
#include "test_math.h"
#include "tumble/util/alloc.h"

using compound::CompoundShapeChild;
using shapes::Shape;

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};
static const Float4 IDENTITY = {0, 0, 0, 1};

/** the child index the first level of `id` names, in a list of `count` */
static uint32_t child_index_of(sub_shape::SubShapeId id, uint32_t count) {
    sub_shape::PopResult popped;
    sub_shape::pop_index(popped, id, count);
    return popped.value;
}

static Shape *make_compound(const CompoundShapeChild *children, uint32_t count) {
    return compound::create({.children = children, .num_children = count});
}

/* creation, bounds, centre of mass */

void test_compound_aabb(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    Shape *b = box::create({.half_extents = {2, 2, 2}});
    const CompoundShapeChild children[] = {{{0, 0, 0}, IDENTITY, s}, {{0, 0, 0}, IDENTITY, b}};
    Shape *c = make_compound(children, 2);

    // the union of the sphere's [-1, 1] and the box's [-2, 2]
    CHECK(ctx, c->aabb.min.x == -2 && c->aabb.min.y == -2 && c->aabb.min.z == -2);
    CHECK(ctx, c->aabb.max.x == 2 && c->aabb.max.y == 2 && c->aabb.max.z == 2);

    // a child turned a quarter about z and moved: its extents swap x and y
    Shape *tall = box::create({.half_extents = {1, 3, 0.5f}});
    const CompoundShapeChild turned[] = {{{10, 0, 0}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f), tall}};
    Shape *t = make_compound(turned, 1);
    CHECK(ctx, near3(t->aabb.min, 7, -1, -0.5f, 1e-5f));
    CHECK(ctx, near3(t->aabb.max, 13, 1, 0.5f, 1e-5f));

    compound::destroy(c);
    compound::destroy(t);
    sphere::destroy(s);
    box::destroy(b);
    box::destroy(tall);
}

void test_compound_create(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    Shape *b = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeChild children[] = {{{0, 0, 0}, IDENTITY, s}, {{0, 0, 0}, IDENTITY, b}};
    Shape *c = make_compound(children, 2);

    CHECK(ctx, c->type == shapes::SHAPE_COMPOUND);
    CHECK(ctx, c->compound.data->children.length == 2);
    CHECK(ctx, c->compound.data->children[0].shape == s);
    CHECK(ctx, c->compound.data->children[1].shape == b);
    CHECK(ctx, c->material_id == -1);
    // the volume is the children's summed
    CHECK_NEAR(ctx, c->volume, 4.0f / 3.0f * scalar::PI + 8.0f, 1e-4f);

    // the list is the compound's own: the settings' array may go away
    CompoundShapeChild scratch[] = {{{0, 0, 0}, IDENTITY, s}};
    Shape *copy = make_compound(scratch, 1);
    scratch[0].shape = b;
    CHECK(ctx, copy->compound.data->children[0].shape == s);

    // destroying the compound leaves the children: freeing them afterwards gives back exactly their two blocks
    compound::destroy(c);
    compound::destroy(copy);
    const size_t with_children = util::allocated_blocks();
    sphere::destroy(s);
    box::destroy(b);
    CHECK(ctx, with_children - util::allocated_blocks() == 2);
}

void test_compound_center_of_mass(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    Shape *b = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeChild centred[] = {{{0, 0, 0}, IDENTITY, s}, {{0, 0, 0}, IDENTITY, b}};
    Shape *c = make_compound(centred, 2);
    CHECK(ctx, c->center_of_mass.x == 0 && c->center_of_mass.y == 0 && c->center_of_mass.z == 0);

    // equal masses at 0 and 4: the centre of mass halfway
    Shape *s2 = sphere::create({.radius = 1.0f});
    const CompoundShapeChild offset[] = {{{0, 0, 0}, IDENTITY, s}, {{4, 0, 0}, IDENTITY, s2}};
    Shape *o = make_compound(offset, 2);
    CHECK(ctx, near3(o->center_of_mass, 2, 0, 0, 1e-5f));

    // weighted by mass: a 2 m box (8000 kg) at 0 and a 1 m box (1000 kg) at 3
    Shape *big = box::create({.half_extents = {1, 1, 1}});
    Shape *small = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    const CompoundShapeChild weighted[] = {{{0, 0, 0}, IDENTITY, big}, {{3, 0, 0}, IDENTITY, small}};
    Shape *w = make_compound(weighted, 2);
    CHECK(ctx, near3(w->center_of_mass, 3.0f / 9.0f, 0, 0, 1e-5f));

    compound::destroy(c);
    compound::destroy(o);
    compound::destroy(w);
    sphere::destroy(s);
    sphere::destroy(s2);
    box::destroy(b);
    box::destroy(big);
    box::destroy(small);
}

/* mass properties */

void test_compound_mass_properties_parallel_axis(TestCtx &ctx) {
    dispatch::init();
    // two unit spheres 4 apart: each is 2 from the centre of mass along x
    Shape *s = sphere::create({.radius = 1.0f});
    const CompoundShapeChild children[] = {{{0, 0, 0}, IDENTITY, s}, {{4, 0, 0}, IDENTITY, s}};
    Shape *c = make_compound(children, 2);

    mass_properties::MassProperties mp;
    shapes::compute_mass_properties(mp, c);
    const float m = 4.0f / 3.0f * scalar::PI * 1000.0f;
    const float own = 0.4f * m;  // 2/5 m r^2
    CHECK_NEAR(ctx, mp.mass, 2.0f * m, 1e-1f);
    // about x the offsets are on the axis: only the spheres' own inertia
    CHECK_NEAR(ctx, mp.inertia.m[0], 2.0f * own, 1.0f);
    // about y and z each sphere adds m d^2 with d = 2
    CHECK_NEAR(ctx, mp.inertia.m[5], 2.0f * (own + m * 4.0f), 2.0f);
    CHECK_NEAR(ctx, mp.inertia.m[10], 2.0f * (own + m * 4.0f), 2.0f);
    // no products of inertia
    CHECK_NEAR(ctx, mp.inertia.m[1], 0.0f, 1e-2f);
    CHECK_NEAR(ctx, mp.inertia.m[2], 0.0f, 1e-2f);
    CHECK_NEAR(ctx, mp.inertia.m[6], 0.0f, 1e-2f);
    CHECK(ctx, mp.inertia.m[15] == 1.0f);

    // off-axis: at (0, 0, 0) and (3, 4, 0) the offsets from the centre are (+-1.5, +-2, 0), giving
    // Ixx += m dy^2, Iyy += m dx^2, Izz += m (dx^2 + dy^2), Ixy -= m dx dy per sphere
    const CompoundShapeChild diagonal[] = {{{0, 0, 0}, IDENTITY, s}, {{3, 4, 0}, IDENTITY, s}};
    Shape *d = make_compound(diagonal, 2);
    shapes::compute_mass_properties(mp, d);
    CHECK_NEAR(ctx, mp.inertia.m[0], 2.0f * (own + m * 4.0f), 2.0f);
    CHECK_NEAR(ctx, mp.inertia.m[5], 2.0f * (own + m * 2.25f), 2.0f);
    CHECK_NEAR(ctx, mp.inertia.m[10], 2.0f * (own + m * 6.25f), 2.0f);
    CHECK_NEAR(ctx, mp.inertia.m[1], -2.0f * m * 3.0f, 2.0f);
    CHECK_NEAR(ctx, mp.inertia.m[4], -2.0f * m * 3.0f, 2.0f);

    compound::destroy(c);
    compound::destroy(d);
    sphere::destroy(s);
}

void test_compound_mass_properties_rotated_child(TestCtx &ctx) {
    dispatch::init();
    // a box turned a quarter about z: its x and y inertia swap, and a single child sits on the centre of mass
    Shape *b = box::create({.half_extents = {1, 2, 3}, .convex_radius = 0.0f});
    mass_properties::MassProperties own;
    shapes::compute_mass_properties(own, b);

    const CompoundShapeChild children[] = {{{0, 0, 5}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f), b}};
    Shape *c = make_compound(children, 1);
    CHECK(ctx, near3(c->center_of_mass, 0, 0, 5, 1e-5f));

    mass_properties::MassProperties mp;
    shapes::compute_mass_properties(mp, c);
    CHECK_NEAR(ctx, mp.mass, own.mass, 1e-2f);
    CHECK_NEAR(ctx, mp.inertia.m[0], own.inertia.m[5], 1.0f);
    CHECK_NEAR(ctx, mp.inertia.m[5], own.inertia.m[0], 1.0f);
    CHECK_NEAR(ctx, mp.inertia.m[10], own.inertia.m[10], 1.0f);
    CHECK_NEAR(ctx, mp.inertia.m[1], 0.0f, 1e-1f);

    compound::destroy(c);
    box::destroy(b);
}

/* per-shape functions through the leaf */

void test_compound_inner_radius(TestCtx &ctx) {
    dispatch::init();
    Shape *s5 = sphere::create({.radius = 5.0f});
    Shape *s3 = sphere::create({.radius = 3.0f});
    const CompoundShapeChild children[] = {{{0, 0, 0}, IDENTITY, s5}, {{0, 0, 0}, IDENTITY, s3}};
    Shape *c = make_compound(children, 2);
    CHECK(ctx, shapes::get_shape_inner_radius(c) == 3.0f);

    // nested: the smallest sphere anywhere
    Shape *s10 = sphere::create({.radius = 10.0f});
    const CompoundShapeChild inner_children[] = {{{0, 0, 0}, IDENTITY, s10}, {{0, 0, 0}, IDENTITY, s5}};
    Shape *inner = make_compound(inner_children, 2);
    const CompoundShapeChild outer_children[] = {{{0, 0, 0}, IDENTITY, inner}, {{0, 0, 0}, IDENTITY, s3}};
    Shape *outer = make_compound(outer_children, 2);
    CHECK(ctx, shapes::get_shape_inner_radius(outer) == 3.0f);

    compound::destroy(c);
    compound::destroy(outer);
    compound::destroy(inner);
    sphere::destroy(s3);
    sphere::destroy(s5);
    sphere::destroy(s10);
}

void test_compound_surface_normal(TestCtx &ctx) {
    dispatch::init();
    Shape *box1 = box::create({.half_extents = {1, 1, 1}});
    Shape *box2 = box::create({.half_extents = {2, 2, 2}});
    const CompoundShapeChild children[] = {{{0, 0, 0}, IDENTITY, box1}, {{0, 0, 0}, IDENTITY, box2}};
    Shape *c = make_compound(children, 2);

    // child 1, the larger box, from outside its +x face
    sub_shape::SubShapeIdBuilder id;
    sub_shape::push_index(id, sub_shape::builder(), 1, 2);
    Vec3f normal = shapes::get_shape_surface_normal(c, vec3f::make(2.1f, 0, 0), id.value);
    CHECK(ctx, near3(normal, 1, 0, 0, 1e-5f));

    // a child moved by (5, 3, 0) and turned a quarter about z: its +x face is now the +y face, at y = 4
    Shape *inner_box = box::create({.half_extents = {1, 2, 1}});
    Shape *s = sphere::create({.radius = 1.0f});
    const CompoundShapeChild placed[] = {{{0, 0, 0}, IDENTITY, s}, {{5, 3, 0}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f), inner_box}};
    Shape *p = make_compound(placed, 2);
    normal = shapes::get_shape_surface_normal(p, vec3f::make(5, 4.1f, 0), id.value);
    CHECK(ctx, near3(normal, 0, 1, 0, 1e-5f));

    // one child takes no bits: the empty id names it
    const CompoundShapeChild single[] = {{{0, 0, 0}, IDENTITY, box1}};
    Shape *one = make_compound(single, 1);
    normal = shapes::get_shape_surface_normal(one, vec3f::make(1, 1, 1), sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK_NEAR(ctx, vec3f::length(normal), 1.0f, 1e-5f);

    compound::destroy(c);
    compound::destroy(p);
    compound::destroy(one);
    box::destroy(box1);
    box::destroy(box2);
    box::destroy(inner_box);
    sphere::destroy(s);
}

void test_compound_supporting_face(TestCtx &ctx) {
    dispatch::init();
    Float4x4 identity;
    mat4f::store(identity, mat4f::identity());
    face::Face face;

    Shape *box1 = box::create({.half_extents = {1, 1, 1}});
    Shape *box2 = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeChild boxes[] = {{{0, 0, 0}, IDENTITY, box1}, {{0, 0, 0}, IDENTITY, box2}};
    Shape *c = make_compound(boxes, 2);
    // id 0 is the first child
    shapes::get_shape_supporting_face(face, c, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);

    Shape *s = sphere::create({.radius = 1.0f});
    const CompoundShapeChild mixed[] = {{{0, 0, 0}, IDENTITY, s}, {{5, 0, 0}, IDENTITY, box1}};
    Shape *m = make_compound(mixed, 2);
    shapes::get_shape_supporting_face(face, m, 0, vec3f::make(1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 0);  // a sphere has no face
    // the box child at x = 5: towards -x its +x face, at x = 6 in the compound's space
    shapes::get_shape_supporting_face(face, m, 1, vec3f::make(-1, 0, 0), mat4f::load(identity), vec3f::make(1, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < 4; i++) CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), 6.0f, 1e-5f);

    // the compound itself placed at y = 10: the face follows
    const Mat4f lifted = mat4f::from_rotation_translation(quatf::load(IDENTITY), vec3f::make(0, 10, 0));
    shapes::get_shape_supporting_face(face, m, 1, vec3f::make(-1, 0, 0), lifted, vec3f::make(1, 1, 1));
    for (uint32_t i = 0; i < 4; i++) {
        CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), 6.0f, 1e-5f);
        CHECK(ctx, scalar::abs(vec3f::y(face.vertices[i]) - 10.0f) <= 1.0f + 1e-5f);
    }

    compound::destroy(c);
    compound::destroy(m);
    box::destroy(box1);
    box::destroy(box2);
    sphere::destroy(s);
}

void test_compound_leaf_shape(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    Shape *b = box::create({.half_extents = {1, 1, 1}});
    Shape *cap = capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.5f});
    const CompoundShapeChild children[] = {{{0, 0, 0}, IDENTITY, s}, {{3, 0, 0}, IDENTITY, b}, {{0, 3, 0}, IDENTITY, cap}};
    Shape *c = make_compound(children, 3);

    // three children take two bits
    shapes::GetLeafShapeResult leaf;
    for (uint32_t i = 0; i < 3; i++) {
        sub_shape::SubShapeIdBuilder id;
        sub_shape::push_index(id, sub_shape::builder(), i, 3);
        CHECK(ctx, id.current_bit == 2);
        shapes::get_leaf_shape(leaf, c, id.value);
        CHECK(ctx, leaf.shape == children[i].shape);
        CHECK(ctx, leaf.remainder == sub_shape::EMPTY_SUB_SHAPE_ID);
    }
    // the empty id resolves to nothing
    shapes::get_leaf_shape(leaf, c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, leaf.shape == nullptr);

    // the transformed sub shape: the capsule at (0, 3, 0) under a compound turned a quarter about z sits at (-3, 0, 0)
    sub_shape::SubShapeIdBuilder id;
    sub_shape::push_index(id, sub_shape::builder(), 2, 3);
    shapes::GetSubShapeTransformedShapeResult transformed;
    transformed.position = vec3f::make(10, 0, 0);
    transformed.rotation = axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f);
    transformed.scale = vec3f::make(1, 1, 1);
    shapes::get_sub_shape_transformed_shape(transformed, c, id.value);
    CHECK(ctx, transformed.shape == cap);
    CHECK(ctx, near3(transformed.position, 7, 0, 0, 1e-5f));

    compound::destroy(c);
    sphere::destroy(s);
    box::destroy(b);
    capsule::destroy(cap);
}

/* through the dispatch tables */

void test_compound_dispatch_wiring(TestCtx &ctx) {
    dispatch::init();
    const shapes::ShapeType others[] = {shapes::SHAPE_SPHERE, shapes::SHAPE_BOX, shapes::SHAPE_CAPSULE, shapes::SHAPE_CONVEX_HULL,
                                        shapes::SHAPE_CYLINDER};
    for (shapes::ShapeType t : others) {
        CHECK(ctx, shapes::collide_fn(shapes::SHAPE_COMPOUND, t) == compound::collide_compound_vs_shape);
        CHECK(ctx, shapes::collide_fn(t, shapes::SHAPE_COMPOUND) == compound::collide_shape_vs_compound);
        CHECK(ctx, shapes::cast_fn(shapes::SHAPE_COMPOUND, t) == compound::cast_compound_vs_shape);
        CHECK(ctx, shapes::cast_fn(t, shapes::SHAPE_COMPOUND) == compound::cast_shape_vs_compound);
    }
    // compound vs compound walks b first
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_COMPOUND, shapes::SHAPE_COMPOUND) == compound::collide_shape_vs_compound);
    CHECK(ctx, shapes::cast_fn(shapes::SHAPE_COMPOUND, shapes::SHAPE_COMPOUND) == compound::cast_shape_vs_compound);
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_COMPOUND, shapes::SHAPE_EMPTY) == nullptr);

    const shapes::ShapeDef &def = shapes::shape_defs[shapes::shape_type_index(shapes::SHAPE_COMPOUND)];
    CHECK(ctx, def.type == shapes::SHAPE_COMPOUND && def.set_support == nullptr);
    CHECK(ctx, def.get_leaf_shape == compound::get_leaf_shape && def.cast_ray == compound::cast_ray_vs_compound);
}

static void collide(collide_shape::CollideShapeCollector &collector, const Shape *a, const Transformf &transform_a, const Shape *b,
                    const Transformf &transform_b) {
    shapes::collide_fn(a->type, b->type)(collector, collide_shape::default_settings(), a, NO_SUB_SHAPE, transform_a, b, NO_SUB_SHAPE,
                                         transform_b);
}

void test_compound_collide_two_boxes_vs_sphere(TestCtx &ctx) {
    dispatch::init();
    Shape *b0 = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .material_id = 3});
    Shape *b1 = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .material_id = 4});
    const CompoundShapeChild children[] = {{{-2, 0, 0}, IDENTITY, b0}, {{2, 0, 0}, IDENTITY, b1}};
    Shape *c = make_compound(children, 2);
    Shape *s = sphere::create({.radius = 0.5f, .material_id = 9});
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);

    // the sphere 0.1 into the top of the right box
    collide(collector, c, pose({0, 0, 0}), s, pose({2, 0.9f, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const collide_shape::CollideShapeHit &hit = collector.hits[0];
    CHECK(ctx, child_index_of(hit.sub_shape_id_a, 2) == 1);
    CHECK(ctx, hit.sub_shape_id_b == sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK_NEAR(ctx, hit.penetration, 0.1f, 1e-4f);
    CHECK(ctx, vec3f::dot(unit(hit.penetration_axis), vec3f::make(0, 1, 0)) > 0.9999f);
    CHECK(ctx, near3(hit.point_a, 2, 0.5f, 0, 1e-4f));
    CHECK(ctx, shapes::get_shape_material(c, hit.sub_shape_id_a) == 4 && shapes::get_shape_material(s, hit.sub_shape_id_b) == 9);

    // the other way round: the compound is b, the axis points down into it
    collide_shape::reset(collector);
    collide(collector, s, pose({-2, -0.9f, 0}), c, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, child_index_of(collector.hits[0].sub_shape_id_b, 2) == 0);
    CHECK(ctx, collector.hits[0].sub_shape_id_a == sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.1f, 1e-4f);
    CHECK(ctx, vec3f::dot(unit(collector.hits[0].penetration_axis), vec3f::make(0, 1, 0)) > 0.9999f);
    // the sub shape id leads to the left box
    shapes::GetLeafShapeResult leaf;
    shapes::get_leaf_shape(leaf, c, collector.hits[0].sub_shape_id_b);
    CHECK(ctx, leaf.shape == b0);

    // the compound turned a quarter about z: the right box is now above the origin at (0, 2, 0)
    collide_shape::reset(collector);
    collide(collector, c, pose({0, 0, 0}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f)), s, pose({0, 2.9f, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, child_index_of(collector.hits[0].sub_shape_id_a, 2) == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.1f, 1e-4f);
    CHECK(ctx, vec3f::dot(unit(collector.hits[0].penetration_axis), vec3f::make(0, 1, 0)) > 0.9999f);

    // scaled by 2: the right box spans x 3..5 and its top is at y = 1
    collide_shape::reset(collector);
    collide(collector, c, pose({0, 0, 0}, IDENTITY, {2, 2, 2}), s, pose({4, 1.4f, 0}));
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, child_index_of(collector.hits[0].sub_shape_id_a, 2) == 1);
    CHECK_NEAR(ctx, collector.hits[0].penetration, 0.1f, 1e-4f);

    // between the boxes: nothing
    collide_shape::reset(collector);
    collide(collector, c, pose({0, 0, 0}), s, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 0);

    // an any collector stops after the first child that hits
    Shape *wide = sphere::create({.radius = 3.0f});
    collide_shape::CollideShapeCollector any = collide_shape::create_collector(collector::COLLECT_ANY);
    collide(any, c, pose({0, 0, 0}), wide, pose({0, 0, 0}));
    CHECK(ctx, any.has_hit && child_index_of(any.hits[0].sub_shape_id_a, 2) == 0);
    collide_shape::reset(collector);
    collide(collector, c, pose({0, 0, 0}), wide, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 2);

    collide_shape::release(collector);
    collide_shape::release(any);
    compound::destroy(c);
    box::destroy(b0);
    box::destroy(b1);
    sphere::destroy(s);
    sphere::destroy(wide);
}

void test_compound_cast_ray_and_point(TestCtx &ctx) {
    dispatch::init();
    Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    const CompoundShapeChild children[] = {{{-2, 0, 0}, IDENTITY, b}, {{2, 0, 0}, IDENTITY, b}};
    Shape *c = make_compound(children, 2);

    // straight down onto the right box's top at y = 0.5
    cast_ray::CastRayCollector rays = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    const cast_ray::Rayf ray = {vec3f::make(2, 5, 0), vec3f::make(0, -1, 0), 10.0f};
    shapes::def_of(c).cast_ray(rays, cast_ray::default_settings(), ray, c, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, rays.has_hit);
    CHECK_NEAR(ctx, rays.hits[0].fraction, 0.45f, 1e-5f);
    CHECK(ctx, child_index_of(rays.hits[0].sub_shape_id, 2) == 1);

    // along x through both: the nearer, the left box's -x face at x = -2.5
    cast_ray::reset(rays);
    const cast_ray::Rayf across = {vec3f::make(-10, 0, 0), vec3f::make(1, 0, 0), 20.0f};
    shapes::def_of(c).cast_ray(rays, cast_ray::default_settings(), across, c, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, rays.has_hit);
    CHECK_NEAR(ctx, rays.hits[0].fraction, 7.5f / 20.0f, 1e-5f);
    CHECK(ctx, child_index_of(rays.hits[0].sub_shape_id, 2) == 0);

    collide_point::CollidePointCollector points = collide_point::create_collector(collector::COLLECT_ALL);
    shapes::def_of(c).collide_point(points, collide_point::default_settings(), vec3f::make(-2, 0.2f, 0.1f), c, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, points.hits.length == 1);
    CHECK(ctx, child_index_of(points.hits[0].sub_shape_id_b, 2) == 0);
    collide_point::reset(points);
    shapes::def_of(c).collide_point(points, collide_point::default_settings(), vec3f::make(0, 0, 0), c, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, points.hits.length == 0);
    // the compound moved up by 10: the point follows
    collide_point::reset(points);
    shapes::def_of(c).collide_point(points, collide_point::default_settings(), vec3f::make(2, 10.2f, 0), c, NO_SUB_SHAPE, pose({0, 10, 0}));
    CHECK(ctx, points.hits.length == 1);
    CHECK(ctx, child_index_of(points.hits[0].sub_shape_id_b, 2) == 1);

    cast_ray::release(rays);
    collide_point::release(points);
    compound::destroy(c);
    box::destroy(b);
}

void test_compound_cast_shape(TestCtx &ctx) {
    dispatch::init();
    Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.0f});
    const CompoundShapeChild children[] = {{{-2, 0, 0}, IDENTITY, b}, {{2, 0, 0}, IDENTITY, b}};
    Shape *c = make_compound(children, 2);
    Shape *s = sphere::create({.radius = 0.5f});
    const cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_CLOSEST);

    // a sphere dropped onto the right box: its bottom at 4.5 meets the top at 0.5 after 4 of 10
    shapes::cast_fn(s->type, c->type)(collector, settings, s, NO_SUB_SHAPE, pose({2, 5, 0}), vec3f::make(0, -10, 0), c, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, collector.has_hit);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.4f, 1e-3f);
    CHECK(ctx, child_index_of(collector.hits[0].sub_shape_id_b, 2) == 1);
    CHECK(ctx, collector.hits[0].sub_shape_id_a == sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::dot(collector.hits[0].normal, vec3f::make(0, 1, 0)) > 0.999f);

    // the compound swept up onto a resting sphere: the same contact, the ids on the other side
    cast_shape::reset(collector);
    shapes::cast_fn(c->type, s->type)(collector, settings, c, NO_SUB_SHAPE, pose({0, -5, 0}), vec3f::make(0, 10, 0), s, NO_SUB_SHAPE, pose({-2, 0, 0}));
    CHECK(ctx, collector.has_hit);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.4f, 1e-3f);
    CHECK(ctx, child_index_of(collector.hits[0].sub_shape_id_a, 2) == 0);

    // a sweep that passes between the boxes: nothing
    cast_shape::reset(collector);
    shapes::cast_fn(s->type, c->type)(collector, settings, s, NO_SUB_SHAPE, pose({0, 5, 0}), vec3f::make(0, -10, 0), c, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, !collector.has_hit);

    cast_shape::release(collector);
    compound::destroy(c);
    box::destroy(b);
    sphere::destroy(s);
}

void test_compound_nested(TestCtx &ctx) {
    dispatch::init();
    Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    Shape *other = box::create({.half_extents = {0.25f, 0.25f, 0.25f}});
    // inner: two boxes stacked 3 apart, the upper first; outer: the inner at (10, 0, 0), then a small box at the origin
    const CompoundShapeChild inner_children[] = {{{0, 3, 0}, IDENTITY, b}, {{0, 0, 0}, IDENTITY, b}};
    Shape *inner = make_compound(inner_children, 2);
    const CompoundShapeChild outer_children[] = {{{10, 0, 0}, IDENTITY, inner}, {{0, 0, 0}, IDENTITY, other}};
    Shape *outer = make_compound(outer_children, 2);

    CHECK(ctx, near3(outer->aabb.min, -0.25f, -0.5f, -0.5f, 1e-5f));
    CHECK(ctx, near3(outer->aabb.max, 10.5f, 3.5f, 0.5f, 1e-5f));
    CHECK_NEAR(ctx, outer->volume, 2.0f + 0.125f, 1e-5f);

    // the mass matches the same three boxes in one flat compound
    const CompoundShapeChild flat_children[] = {{{0, 0, 0}, IDENTITY, other}, {{10, 0, 0}, IDENTITY, b}, {{10, 3, 0}, IDENTITY, b}};
    Shape *flat = make_compound(flat_children, 3);
    CHECK(ctx, near3(outer->center_of_mass, flat->center_of_mass, 1e-4f));
    mass_properties::MassProperties nested_mass, flat_mass;
    shapes::compute_mass_properties(nested_mass, outer);
    shapes::compute_mass_properties(flat_mass, flat);
    CHECK_NEAR(ctx, nested_mass.mass, flat_mass.mass, 1e-2f);
    for (uint32_t i = 0; i < 11; i++) CHECK_NEAR(ctx, nested_mass.inertia.m[i], flat_mass.inertia.m[i], 0.5f);

    // a sphere 0.1 into the upper inner box: the path is outer child 0, then inner child 0
    Shape *s = sphere::create({.radius = 0.5f});
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    collide(collector, outer, pose({0, 0, 0}), s, pose({10, 3.9f, 0}));
    CHECK(ctx, collector.hits.length == 1);
    const sub_shape::SubShapeId id = collector.hits.length == 1 ? collector.hits[0].sub_shape_id_a : sub_shape::EMPTY_SUB_SHAPE_ID;
    sub_shape::PopResult outer_level;
    sub_shape::pop_index(outer_level, id, 2);
    CHECK(ctx, outer_level.value == 0);
    sub_shape::PopResult inner_level;
    sub_shape::pop_index(inner_level, outer_level.remainder, 2);
    CHECK(ctx, inner_level.value == 0);
    CHECK(ctx, inner_level.remainder == sub_shape::EMPTY_SUB_SHAPE_ID);
    if (collector.hits.length == 1) CHECK_NEAR(ctx, collector.hits[0].penetration, 0.1f, 1e-4f);

    // the leaf lookup consumes both levels
    shapes::GetLeafShapeResult leaf;
    shapes::get_leaf_shape(leaf, outer, id);
    CHECK(ctx, leaf.shape == b && leaf.remainder == sub_shape::EMPTY_SUB_SHAPE_ID);
    // and the transformed sub shape lands on the upper inner box
    shapes::GetSubShapeTransformedShapeResult transformed;
    transformed.position = vec3f::make(0, 0, 0);
    transformed.rotation = quatf::load(IDENTITY);
    transformed.scale = vec3f::make(1, 1, 1);
    shapes::get_sub_shape_transformed_shape(transformed, outer, id);
    CHECK(ctx, transformed.shape == b && near3(transformed.position, 10, 3, 0, 1e-5f));
    // the surface normal through both levels (no rotations)
    const Vec3f normal = shapes::get_shape_surface_normal(outer, vec3f::make(10, 3.6f, 0), id);
    CHECK(ctx, near3(normal, 0, 1, 0, 1e-5f));

    // the ray finds the same box
    cast_ray::CastRayCollector rays = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    const cast_ray::Rayf ray = {vec3f::make(10, 10, 0), vec3f::make(0, -1, 0), 20.0f};
    shapes::def_of(outer).cast_ray(rays, cast_ray::default_settings(), ray, outer, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, rays.has_hit && rays.hits[0].sub_shape_id == id);
    CHECK_NEAR(ctx, rays.hits[0].fraction, 6.5f / 20.0f, 1e-5f);

    // compound vs compound: the flat compound 0.4 above the nested one overlaps each box with its twin. b's
    // children are walked first: the inner's upper and lower box (0.6 deep), then the small box (0.1 deep)
    collide_shape::reset(collector);
    collide(collector, flat, pose({0, 0.4f, 0}), outer, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 3);
    if (collector.hits.length == 3) {
        const uint32_t flat_index[] = {2, 1, 0};
        const uint32_t outer_index[] = {0, 0, 1};
        const uint32_t inner_index[] = {0, 1, 0};
        const float depths[] = {0.6f, 0.6f, 0.1f};
        for (uint32_t i = 0; i < 3; i++) {
            const collide_shape::CollideShapeHit &h = collector.hits[i];
            CHECK(ctx, child_index_of(h.sub_shape_id_a, 3) == flat_index[i]);
            sub_shape::PopResult b_level;
            sub_shape::pop_index(b_level, h.sub_shape_id_b, 2);
            CHECK(ctx, b_level.value == outer_index[i]);
            if (outer_index[i] == 0) CHECK(ctx, child_index_of(b_level.remainder, 2) == inner_index[i]);
            CHECK_NEAR(ctx, h.penetration, depths[i], 1e-3f);
        }
    }

    collide_shape::release(collector);
    cast_ray::release(rays);
    compound::destroy(outer);
    compound::destroy(inner);
    compound::destroy(flat);
    box::destroy(b);
    box::destroy(other);
    sphere::destroy(s);
}

/* sub shape ids: the last child of a power of two count pushed onto an empty id is all ones, the empty id's bits */

static sub_shape::SubShapeId ray_down_sub_shape_id(TestCtx &ctx, const Shape *shape, float x) {
    cast_ray::CastRayCollector rays = cast_ray::create_collector(collector::COLLECT_ALL);
    const cast_ray::Rayf ray = {vec3f::make(x, 10, 0), vec3f::make(0, -1, 0), 20.0f};
    shapes::def_of(shape).cast_ray(rays, cast_ray::default_settings(), ray, shape, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, rays.hits.length == 1);
    const sub_shape::SubShapeId id = rays.hits.length == 1 ? rays.hits[0].sub_shape_id : 0;
    cast_ray::release(rays);
    return id;
}

static shapes::GetSubShapeTransformedShapeResult transformed_shape_of(const Shape *shape, sub_shape::SubShapeId id, const Float3 &scale) {
    shapes::GetSubShapeTransformedShapeResult result;
    result.position = vec3f::make(0, 0, 0);
    result.rotation = quatf::load(IDENTITY);
    result.scale = vec3f::load(scale);
    shapes::get_sub_shape_transformed_shape(result, shape, id);
    return result;
}

void test_compound_sub_shape_id_last_of_two(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    Shape *leaf = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeChild children[] = {{{-2, 0, 0}, IDENTITY, s}, {{2, 0, 0}, IDENTITY, leaf}};
    Shape *c = make_compound(children, 2);

    // the ray hit's id, the leaf and transformed shape lookups, and the normal all resolve the second child
    const sub_shape::SubShapeId id = ray_down_sub_shape_id(ctx, c, 2);
    CHECK(ctx, id == 0xffffffffu);
    shapes::GetLeafShapeResult leaf_result;
    shapes::get_leaf_shape(leaf_result, c, id);
    CHECK(ctx, leaf_result.shape == leaf);
    const shapes::GetSubShapeTransformedShapeResult transformed = transformed_shape_of(c, id, {1, 1, 1});
    CHECK(ctx, transformed.shape == leaf);
    CHECK(ctx, vec3f::x(transformed.position) == 2.0f);
    const Vec3f normal = shapes::get_shape_surface_normal(c, vec3f::make(2, 1, 0), id);
    CHECK(ctx, near3(normal, 0, 1, 0, 1e-6f));

    // under scale (3, 1, 1) the transformed shape sits at the scaled position
    const shapes::GetSubShapeTransformedShapeResult scaled = transformed_shape_of(c, id, {3, 1, 1});
    CHECK(ctx, scaled.shape == leaf);
    CHECK(ctx, vec3f::x(scaled.position) == 6.0f);

    compound::destroy(c);
    sphere::destroy(s);
    box::destroy(leaf);
}

void test_compound_sub_shape_id_path_one_one(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    Shape *leaf = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    const CompoundShapeChild inner_children[] = {{{-2, 0, 0}, IDENTITY, s}, {{2, 0, 0}, IDENTITY, leaf}};
    Shape *inner = make_compound(inner_children, 2);
    const CompoundShapeChild outer_children[] = {{{-2, 0, 0}, IDENTITY, s}, {{2, 0, 0}, IDENTITY, inner}};
    Shape *outer = make_compound(outer_children, 2);

    // inner child 1 sits at 2 + 2 along x
    const sub_shape::SubShapeId id = ray_down_sub_shape_id(ctx, outer, 4);
    CHECK(ctx, id == 0xffffffffu);
    shapes::GetLeafShapeResult leaf_result;
    shapes::get_leaf_shape(leaf_result, outer, id);
    CHECK(ctx, leaf_result.shape == leaf);
    const shapes::GetSubShapeTransformedShapeResult transformed = transformed_shape_of(outer, id, {1, 1, 1});
    CHECK(ctx, transformed.shape == leaf);
    CHECK(ctx, vec3f::x(transformed.position) == 4.0f);

    // a one child compound takes no bits and keeps the rest of the path
    Shape *first = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeChild pair_children[] = {{{-2, 0, 0}, IDENTITY, first}, {{2, 0, 0}, IDENTITY, s}};
    Shape *pair = make_compound(pair_children, 2);
    const CompoundShapeChild wrapper_children[] = {{{0, 0, 0}, IDENTITY, pair}};
    Shape *wrapper = make_compound(wrapper_children, 1);
    const sub_shape::SubShapeId first_id = ray_down_sub_shape_id(ctx, wrapper, -2);
    CHECK(ctx, first_id == 0xfffffffeu);
    shapes::get_leaf_shape(leaf_result, wrapper, first_id);
    CHECK(ctx, leaf_result.shape == first);

    compound::destroy(outer);
    compound::destroy(inner);
    compound::destroy(wrapper);
    compound::destroy(pair);
    sphere::destroy(s);
    box::destroy(leaf);
    box::destroy(first);
}

void test_compound_nested_rotated_surface_normal(TestCtx &ctx) {
    dispatch::init();
    // outer turns its child a quarter about z, inner turns its box a quarter about x
    Shape *leaf = box::create({.half_extents = {1, 2, 3}});
    const CompoundShapeChild inner_children[] = {{{0, 0, 0}, stored_axis_angle(vec3f::make(1, 0, 0), scalar::PI * 0.5f), leaf}};
    Shape *inner = make_compound(inner_children, 1);
    const CompoundShapeChild outer_children[] = {{{0, 0, 0}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f), inner}};
    Shape *outer = make_compound(outer_children, 1);

    // the box's +y face centre (0, 2, 0) goes to (0, 0, 2) under x, which z leaves alone
    const Vec3f normal = shapes::get_shape_surface_normal(outer, vec3f::make(0, 0, 2), sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, near3(normal, 0, 0, 1, 1e-6f));

    compound::destroy(outer);
    compound::destroy(inner);
    box::destroy(leaf);
}

void test_compound_scaled_supporting_face(TestCtx &ctx) {
    dispatch::init();
    Shape *b = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeChild children[] = {{{-2, 0, 0}, IDENTITY, b}, {{2, 0, 0}, IDENTITY, b}};
    Shape *c = make_compound(children, 2);
    Float4x4 identity;
    mat4f::store(identity, mat4f::identity());

    // the second box under scale (2, 1, 1): centre x = 4, half extent 2, so its +x face is at x = 6
    face::Face face;
    shapes::get_shape_supporting_face(face, c, 0xffffffffu, vec3f::make(-1, 0, 0), mat4f::load(identity), vec3f::make(2, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < 4; i++) CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), 6.0f, 1e-6f);

    compound::destroy(c);
    box::destroy(b);
}

/* a rotated child under a non-uniform scale */

static bool point_hits(const Shape *shape, const Float3 &point, const Float3 &scale) {
    collide_point::CollidePointCollector points = collide_point::create_collector(collector::COLLECT_ALL);
    shapes::def_of(shape).collide_point(points, collide_point::default_settings(), vec3f::load(point), shape, NO_SUB_SHAPE, pose({0, 0, 0}, IDENTITY, scale));
    const bool hit = points.has_hit;
    collide_point::release(points);
    return hit;
}

void test_compound_rotated_child_under_non_uniform_scale(TestCtx &ctx) {
    dispatch::init();
    // a bar long along its own x, turned to lie along y, under a scale that stretches x by 2. the scale applies in the
    // compound's space, so the bar is 1 wide in x (0.25 * 2 each side) and 2 tall in y; passed to the child unrotated it
    // would stretch the bar's own long axis instead (0.25 in x, 2 each side in y)
    Shape *bar = box::create({.half_extents = {1, 0.25f, 0.25f}, .convex_radius = 0.0f});
    const CompoundShapeChild children[] = {{{0, 0, 0}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f), bar}};
    const Float3 scale = {2, 1, 1};
    Shape *shapes_under_test[2] = {make_compound(children, 1), static_compound::create({.children = children, .num_children = 1})};
    Float4x4 identity;
    mat4f::store(identity, mat4f::identity());

    for (Shape *c : shapes_under_test) {
        CHECK(ctx, point_hits(c, {0.4f, 0, 0}, scale));
        CHECK(ctx, point_hits(c, {0, 0.9f, 0}, scale));
        CHECK(ctx, !point_hits(c, {0, 1.5f, 0}, scale));
        CHECK(ctx, !point_hits(c, {0.6f, 0, 0}, scale));

        // the face against -x is the bar's side at x = 0.5, 2 tall and 0.5 deep
        face::Face face;
        shapes::get_shape_supporting_face(face, c, 0xfffffffeu, vec3f::make(-1, 0, 0), mat4f::load(identity), vec3f::load(scale));
        CHECK(ctx, face.num_vertices == 4);
        for (uint32_t i = 0; i < 4; i++) {
            CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), 0.5f, 1e-5f);
            CHECK_NEAR(ctx, scalar::abs(vec3f::y(face.vertices[i])), 1.0f, 1e-5f);
            CHECK_NEAR(ctx, scalar::abs(vec3f::z(face.vertices[i])), 0.25f, 1e-5f);
        }

        // the leaf's transformed shape carries the scale in the bar's own frame
        shapes::GetSubShapeTransformedShapeResult leaf;
        leaf.position = vec3f::make(0, 0, 0);
        leaf.rotation = quatf::load(IDENTITY);
        leaf.scale = vec3f::load(scale);
        shapes::get_sub_shape_transformed_shape(leaf, c, 0xfffffffeu);
        CHECK(ctx, near3(leaf.scale, 1, 2, 1, 1e-5f));
    }

    compound::destroy(shapes_under_test[0]);
    static_compound::destroy(shapes_under_test[1]);
    box::destroy(bar);
}

/* material by sub shape id */

void test_compound_material(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f, .material_id = 2});
    Shape *b = box::create({.half_extents = {1, 1, 1}, .material_id = 5});
    Shape *inner_sphere = sphere::create({.radius = 0.5f, .material_id = 9});
    const CompoundShapeChild inner_children[] = {{{0, 0, 0}, IDENTITY, b}, {{0, 2, 0}, IDENTITY, inner_sphere}};
    Shape *inner = make_compound(inner_children, 2);
    const CompoundShapeChild children[] = {{{-4, 0, 0}, IDENTITY, s}, {{0, 0, 0}, IDENTITY, b}, {{4, 0, 0}, IDENTITY, inner}};
    Shape *c = make_compound(children, 3);

    // the first level names the child, the rest goes to the child
    sub_shape::SubShapeIdBuilder id;
    sub_shape::push_index(id, sub_shape::builder(), 0, 3);
    CHECK(ctx, shapes::get_shape_material(c, id.value) == 2);
    sub_shape::push_index(id, sub_shape::builder(), 1, 3);
    CHECK(ctx, shapes::get_shape_material(c, id.value) == 5);
    sub_shape::SubShapeIdBuilder nested;
    sub_shape::push_index(id, sub_shape::builder(), 2, 3);
    sub_shape::push_index(nested, id, 1, 2);
    CHECK(ctx, shapes::get_shape_material(c, nested.value) == 9);
    sub_shape::push_index(nested, id, 0, 2);
    CHECK(ctx, shapes::get_shape_material(c, nested.value) == 5);

    // a ray hit's id looks up the material the hit reported
    cast_ray::CastRayCollector rays = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    const cast_ray::Rayf ray = {vec3f::make(4, 10, 0), vec3f::make(0, -1, 0), 20.0f};
    shapes::def_of(c).cast_ray(rays, cast_ray::default_settings(), ray, c, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, rays.has_hit && shapes::get_shape_material(c, rays.hits[0].sub_shape_id) == 9);
    CHECK(ctx, rays.has_hit && shapes::get_shape_material(c, rays.hits[0].sub_shape_id) == 9);
    cast_ray::release(rays);

    compound::destroy(c);
    compound::destroy(inner);
    sphere::destroy(inner_sphere);
    box::destroy(b);
    sphere::destroy(s);
}

/* a child rotated by -identity is unrotated: under a non-uniform scale it sees the scale as +identity does */

void test_compound_negated_identity_child_under_non_uniform_scale(TestCtx &ctx) {
    dispatch::init();
    Shape *b = box::create({.half_extents = {1, 2, 3}});
    const Float4 negated_identity = {0, 0, 0, -1};
    const CompoundShapeChild positive_children[] = {{{1, 0, 0}, IDENTITY, b}};
    const CompoundShapeChild negative_children[] = {{{1, 0, 0}, negated_identity, b}};
    Shape *positive = make_compound(positive_children, 1);
    Shape *negative = make_compound(negative_children, 1);
    const Float3 scale = {2, 3, 4};

    const shapes::GetSubShapeTransformedShapeResult positive_result = transformed_shape_of(positive, sub_shape::EMPTY_SUB_SHAPE_ID, scale);
    const shapes::GetSubShapeTransformedShapeResult negative_result = transformed_shape_of(negative, sub_shape::EMPTY_SUB_SHAPE_ID, scale);
    CHECK(ctx, vec3f::x(negative_result.scale) == 2 && vec3f::y(negative_result.scale) == 3 && vec3f::z(negative_result.scale) == 4);
    CHECK(ctx, near3(negative_result.position, positive_result.position, 1e-6f));

    // rays down and along x meet the scaled box at the same place: top at y = 6, side at x = 2 + 2
    const cast_ray::Rayf rays[2] = {{vec3f::make(2.5f, 10, 0), vec3f::make(0, -1, 0), 20.0f}, {vec3f::make(10, 0, 0), vec3f::make(-1, 0, 0), 20.0f}};
    for (const cast_ray::Rayf &ray : rays) {
        cast_ray::CastRayCollector positive_hits = cast_ray::create_collector(collector::COLLECT_CLOSEST);
        cast_ray::CastRayCollector negative_hits = cast_ray::create_collector(collector::COLLECT_CLOSEST);
        shapes::def_of(positive).cast_ray(positive_hits, cast_ray::default_settings(), ray, positive, NO_SUB_SHAPE, pose({0, 0, 0}, IDENTITY, scale));
        shapes::def_of(negative).cast_ray(negative_hits, cast_ray::default_settings(), ray, negative, NO_SUB_SHAPE, pose({0, 0, 0}, IDENTITY, scale));
        CHECK(ctx, positive_hits.has_hit && negative_hits.has_hit);
        CHECK(ctx, positive_hits.has_hit && negative_hits.has_hit && scalar::abs(positive_hits.hits[0].fraction - negative_hits.hits[0].fraction) <= 1e-6f);
        cast_ray::release(positive_hits);
        cast_ray::release(negative_hits);
    }
    CHECK(ctx, point_hits(negative, {3.9f, 5.9f, 11.9f}, scale) && !point_hits(negative, {4.1f, 0, 0}, scale));
    CHECK(ctx, point_hits(positive, {3.9f, 5.9f, 11.9f}, scale) && !point_hits(positive, {4.1f, 0, 0}, scale));

    compound::destroy(positive);
    compound::destroy(negative);
    box::destroy(b);
}
