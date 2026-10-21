#include "tumble/physics/shapes/static_compound.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/utils/static_compound_bvh.h"
#include "test.h"
#include "test_math.h"
#include "test_rng.h"

using compound::CompoundShapeChild;
using shapes::Shape;

static const sub_shape::SubShapeIdBuilder NO_SUB_SHAPE = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};
static const Float4 IDENTITY = {0, 0, 0, 1};

static Shape *make_static(const CompoundShapeChild *children, uint32_t count, uint32_t max_leaf_children = 4) {
    return static_compound::create({.children = children, .num_children = count, .bvh_max_leaf_children = max_leaf_children});
}

static const collections::Vec<CompoundShapeChild> &children_of(const Shape *shape) { return shape->static_compound.data->children; }

/** the child the first level of `id` names */
static const CompoundShapeChild &child_of(const Shape *shape, sub_shape::SubShapeId id) {
    sub_shape::PopResult popped;
    sub_shape::pop_index(popped, id, children_of(shape).length);
    return children_of(shape)[popped.value];
}

static void cast_ray_at(cast_ray::CastRayCollector &collector, const Shape *shape, const Transformf &transform, const Float3 &origin,
                        const Float3 &direction, float length) {
    const Vec3f unit_direction = vec3f::normalize(vec3f::load(direction));
    const cast_ray::Rayf ray = {vec3f::load(origin), unit_direction, length};
    shapes::def_of(shape).cast_ray(collector, cast_ray::default_settings(), ray, shape, NO_SUB_SHAPE, transform);
}

static void collide_point_at(collide_point::CollidePointCollector &collector, const Shape *shape, const Transformf &transform,
                             const Float3 &point) {
    shapes::def_of(shape).collide_point(collector, collide_point::default_settings(), vec3f::load(point), shape, NO_SUB_SHAPE, transform);
}


void test_static_compound_creation(TestCtx &ctx) {
    dispatch::init();

    // empty
    Shape *empty = make_static(nullptr, 0);
    CHECK(ctx, empty->type == shapes::SHAPE_STATIC_COMPOUND);
    CHECK(ctx, children_of(empty).length == 0);
    CHECK(ctx, empty->static_compound.data->bvh.length == 0);
    CHECK(ctx, empty->volume == 0.0f);

    // single child
    Shape *s = sphere::create({.radius = 1.0f});
    const CompoundShapeChild one[] = {{{0, 0, 0}, IDENTITY, s}};
    Shape *single = make_static(one, 1);
    CHECK(ctx, children_of(single).length == 1);
    CHECK(ctx, single->static_compound.data->bvh.length > 0);
    CHECK_NEAR(ctx, single->volume, 4.0f / 3.0f * scalar::PI, 1e-4f);

    // multiple children
    Shape *b = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeChild two[] = {{{0, 0, 0}, IDENTITY, s}, {{5, 0, 0}, IDENTITY, b}};
    Shape *multiple = make_static(two, 2);
    CHECK(ctx, children_of(multiple).length == 2);
    CHECK(ctx, multiple->static_compound.data->bvh.length > 0);

    static_compound::destroy(empty);
    static_compound::destroy(single);
    static_compound::destroy(multiple);
    sphere::destroy(s);
    box::destroy(b);
}

void test_static_compound_aabb(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    const CompoundShapeChild one[] = {{{0, 0, 0}, IDENTITY, s}};
    Shape *single = make_static(one, 1);
    CHECK(ctx, near3(single->aabb.min, -1, -1, -1, 1e-5f) && near3(single->aabb.max, 1, 1, 1, 1e-5f));

    // sphere at the origin and a box at (10, 0, 0): [-1, -1, -1] to [11, 1, 1]
    Shape *b = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeChild offset[] = {{{0, 0, 0}, IDENTITY, s}, {{10, 0, 0}, IDENTITY, b}};
    Shape *o = make_static(offset, 2);
    CHECK(ctx, near3(o->aabb.min, -1, -1, -1, 1e-5f) && near3(o->aabb.max, 11, 1, 1, 1e-5f));

    // the same as a compound over the same children: [-1, -2, -2] to [7, 2, 2]
    Shape *b2 = box::create({.half_extents = {2, 2, 2}});
    const CompoundShapeChild parity[] = {{{0, 0, 0}, IDENTITY, s}, {{5, 0, 0}, IDENTITY, b2}};
    Shape *st = make_static(parity, 2);
    Shape *dyn = compound::create({.children = parity, .num_children = 2});
    CHECK(ctx, near3(st->aabb.min, -1, -2, -2, 1e-5f) && near3(st->aabb.max, 7, 2, 2, 1e-5f));
    CHECK(ctx, near3(st->aabb.min, dyn->aabb.min, 0.0f) && near3(st->aabb.max, dyn->aabb.max, 0.0f));

    static_compound::destroy(single);
    static_compound::destroy(o);
    static_compound::destroy(st);
    compound::destroy(dyn);
    sphere::destroy(s);
    box::destroy(b);
    box::destroy(b2);
}

void test_static_compound_center_of_mass(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    const CompoundShapeChild one[] = {{{0, 0, 0}, IDENTITY, s}};
    Shape *single = make_static(one, 1);
    CHECK(ctx, near3(single->center_of_mass, 0, 0, 0, 1e-5f));

    const CompoundShapeChild two[] = {{{0, 0, 0}, IDENTITY, s}, {{4, 0, 0}, IDENTITY, s}};
    Shape *pair = make_static(two, 2);
    CHECK(ctx, near3(pair->center_of_mass, 2, 0, 0, 1e-5f));

    // the mass properties are the compound's, whatever order the bvh left the children in
    Shape *dyn = compound::create({.children = two, .num_children = 2});
    mass_properties::MassProperties a, b;
    shapes::compute_mass_properties(a, pair);
    shapes::compute_mass_properties(b, dyn);
    CHECK_NEAR(ctx, a.mass, b.mass, 1e-2f);
    for (uint32_t i = 0; i < 16; i++) CHECK_NEAR(ctx, a.inertia.m[i], b.inertia.m[i], 1e-1f);

    static_compound::destroy(single);
    static_compound::destroy(pair);
    compound::destroy(dyn);
    sphere::destroy(s);
}

void test_static_compound_bvh_structure(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 0.5f});
    CompoundShapeChild row[10];
    for (uint32_t i = 0; i < 10; i++) row[i] = {{(float)i * 2.0f, 0, 0}, IDENTITY, s};
    Shape *shape = make_static(row, 10);
    const collections::Vec<bvh::Node> &tree = shape->static_compound.data->bvh;

    const static_compound_bvh::StaticCompoundBvhStats stats = static_compound_bvh::stats(tree.data, tree.length);
    CHECK(ctx, stats.node_count == tree.length);
    CHECK(ctx, stats.leaf_count > 0);
    CHECK(ctx, stats.max_depth > 0);
    CHECK(ctx, stats.total_children == 10);

    // every leaf's children lie inside the leaf's bounds, and every node inside its parent's
    for (uint32_t node = 0; node < tree.length; node++) {
        const bvh::Node &n = tree[node];
        if (bvh::node_is_leaf(tree.data, (int32_t)node)) {
            const uint32_t start = static_compound_bvh::node_child_start(tree.data, (int32_t)node);
            const uint32_t count = static_compound_bvh::node_child_count(tree.data, (int32_t)node);
            CHECK(ctx, count <= 4);
            for (uint32_t i = start; i < start + count; i++) {
                const Float3 &p = children_of(shape)[i].position;
                CHECK(ctx, p.x - 0.5f >= n.bounds.min.x - 1e-5f && p.x + 0.5f <= n.bounds.max.x + 1e-5f);
            }
        } else {
            const bvh::Node &left = tree[(uint32_t)bvh::node_left((int32_t)node)];
            const bvh::Node &right = tree[(uint32_t)bvh::node_right(tree.data, (int32_t)node)];
            CHECK(ctx, left.bounds.min.x >= n.bounds.min.x && left.bounds.max.x <= n.bounds.max.x);
            CHECK(ctx, right.bounds.min.x >= n.bounds.min.x && right.bounds.max.x <= n.bounds.max.x);
        }
    }

    // a 5 x 5 x 5 lattice: every child counted once and a sensible depth for 125
    CompoundShapeChild lattice[125];
    for (uint32_t x = 0; x < 5; x++)
        for (uint32_t y = 0; y < 5; y++)
            for (uint32_t z = 0; z < 5; z++) lattice[x * 25 + y * 5 + z] = {{(float)x * 3, (float)y * 3, (float)z * 3}, IDENTITY, s};
    Shape *cube = make_static(lattice, 125);
    const collections::Vec<bvh::Node> &cube_tree = cube->static_compound.data->bvh;
    const static_compound_bvh::StaticCompoundBvhStats cube_stats = static_compound_bvh::stats(cube_tree.data, cube_tree.length);
    CHECK(ctx, cube_stats.total_children == 125);
    CHECK(ctx, cube_stats.max_depth >= 3 && cube_stats.max_depth <= 20);

    // an empty tree
    const static_compound_bvh::StaticCompoundBvhStats none = static_compound_bvh::stats(nullptr, 0);
    CHECK(ctx, none.node_count == 0 && none.total_children == 0);

    static_compound::destroy(shape);
    static_compound::destroy(cube);
    sphere::destroy(s);
}

void test_static_compound_update(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    const CompoundShapeChild one[] = {{{0, 0, 0}, IDENTITY, s}};
    Shape *shape = make_static(one, 1);
    const uint32_t original_nodes = shape->static_compound.data->bvh.length;

    // add a child by hand, then rebuild
    Shape *s2 = sphere::create({.radius = 1.0f});
    shape->static_compound.data->children.push({{10, 0, 0}, IDENTITY, s2});
    CHECK(ctx, static_compound::update(shape));
    CHECK(ctx, shape->static_compound.data->bvh.length >= original_nodes);
    CHECK_NEAR(ctx, shape->aabb.max.x, 11.0f, 1e-5f);
    CHECK(ctx, static_compound_bvh::stats(shape->static_compound.data->bvh.data, shape->static_compound.data->bvh.length).total_children == 2);

    static_compound::destroy(shape);
    sphere::destroy(s);
    sphere::destroy(s2);
}

void test_static_compound_cast_ray(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ALL);

    // one sphere at x = 5: hit at 4 of 1000
    const CompoundShapeChild one[] = {{{5, 0, 0}, IDENTITY, s}};
    Shape *single = make_static(one, 1);
    cast_ray_at(collector, single, pose({0, 0, 0}), {0, 0, 0}, {1, 0, 0}, 1000.0f);
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits[0].status == cast_ray::COLLIDING);
    CHECK_NEAR(ctx, collector.hits[0].fraction, 0.004f, 1e-6f);

    // pointing away: nothing
    cast_ray::reset(collector);
    cast_ray_at(collector, single, pose({0, 0, 0}), {0, 0, 0}, {-1, 0, 0}, 1000.0f);
    CHECK(ctx, collector.hits.length == 0);

    // two spheres along the ray: both
    const CompoundShapeChild two[] = {{{5, 0, 0}, IDENTITY, s}, {{10, 0, 0}, IDENTITY, s}};
    Shape *pair = make_static(two, 2);
    cast_ray::reset(collector);
    cast_ray_at(collector, pair, pose({0, 0, 0}), {0, 0, 0}, {1, 0, 0}, 1000.0f);
    CHECK(ctx, collector.hits.length == 2);

    // fifty spheres along x: every one, through the bvh
    CompoundShapeChild row[50];
    for (uint32_t i = 0; i < 50; i++) row[i] = {{(float)i * 3.0f, 0, 0}, IDENTITY, s};
    Shape *many = make_static(row, 50);
    cast_ray::reset(collector);
    cast_ray_at(collector, many, pose({0, 0, 0}), {0, 0, 0}, {1, 0, 0}, 1000.0f);
    CHECK(ctx, collector.hits.length == 50);

    // the closest from x = -10: the sphere at the origin, entered at x = -1 after 9 of 1000
    cast_ray::CastRayCollector closest = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    cast_ray_at(closest, many, pose({0, 0, 0}), {-10, 0, 0}, {1, 0, 0}, 1000.0f);
    CHECK(ctx, closest.has_hit);
    CHECK_NEAR(ctx, closest.hits[0].fraction, 9.0f / 1000.0f, 1e-6f);
    CHECK(ctx, near3(child_of(many, closest.hits[0].sub_shape_id).position, 0, 0, 0, 0.0f));

    // the whole shape moved and turned a quarter about z: the row now runs along y from (0, 100, 0)
    Float4 quarter_z = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f);
    cast_ray::reset(closest);
    cast_ray_at(closest, many, pose({0, 100, 0}, quarter_z), {0, 128.5f, 0}, {0, 1, 0}, 100.0f);
    CHECK(ctx, closest.has_hit);
    CHECK_NEAR(ctx, closest.hits[0].fraction, 0.5f / 100.0f, 1e-5f);
    CHECK(ctx, near3(child_of(many, closest.hits[0].sub_shape_id).position, 30, 0, 0, 0.0f));

    // scaled by 2 along x: the spheres at 6 i, the bvh walked in unscaled space
    cast_ray::reset(collector);
    cast_ray_at(collector, many, pose({0, 0, 0}, IDENTITY, {2, 2, 2}), {0, 0, -10}, {0, 0, 1}, 20.0f);
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits.length == 1 && near3(child_of(many, collector.hits[0].sub_shape_id).position, 0, 0, 0, 0.0f));
    cast_ray::reset(collector);
    cast_ray_at(collector, many, pose({0, 0, 0}, IDENTITY, {2, 2, 2}), {60, 0, -10}, {0, 0, 1}, 20.0f);
    CHECK(ctx, collector.hits.length == 1);
    CHECK(ctx, collector.hits.length == 1 && near3(child_of(many, collector.hits[0].sub_shape_id).position, 30, 0, 0, 0.0f));

    cast_ray::release(collector);
    cast_ray::release(closest);
    static_compound::destroy(single);
    static_compound::destroy(pair);
    static_compound::destroy(many);
    sphere::destroy(s);
}

void test_static_compound_collide_point(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    collide_point::CollidePointCollector collector = collide_point::create_collector(collector::COLLECT_ALL);

    const CompoundShapeChild one[] = {{{5, 0, 0}, IDENTITY, s}};
    Shape *single = make_static(one, 1);
    collide_point_at(collector, single, pose({0, 0, 0}), {5, 0, 0});
    CHECK(ctx, collector.hits.length == 1);
    collide_point::reset(collector);
    collide_point_at(collector, single, pose({0, 0, 0}), {0, 0, 0});
    CHECK(ctx, collector.hits.length == 0);

    // inside two overlapping spheres: both
    Shape *s2 = sphere::create({.radius = 2.0f});
    const CompoundShapeChild overlapping[] = {{{0, 0, 0}, IDENTITY, s2}, {{1, 0, 0}, IDENTITY, s2}};
    Shape *pair = make_static(overlapping, 2);
    collide_point::reset(collector);
    collide_point_at(collector, pair, pose({0, 0, 0}), {0.5f, 0, 0});
    CHECK(ctx, collector.hits.length == 2);

    collide_point::release(collector);
    static_compound::destroy(single);
    static_compound::destroy(pair);
    sphere::destroy(s);
    sphere::destroy(s2);
}

/* a 10 x 10 grid of boxes */

static constexpr uint32_t GRID = 10;
static constexpr float SPACING = 3.0f;

static Shape *make_grid(Shape *box_shape) {
    CompoundShapeChild cells[GRID * GRID];
    for (uint32_t i = 0; i < GRID; i++)
        for (uint32_t j = 0; j < GRID; j++) cells[i * GRID + j] = {{(float)i * SPACING, 0, (float)j * SPACING}, IDENTITY, box_shape};
    return make_static(cells, GRID * GRID);
}

void test_static_compound_dispatch_wiring(TestCtx &ctx) {
    dispatch::init();
    const shapes::ShapeType others[] = {shapes::SHAPE_SPHERE, shapes::SHAPE_BOX, shapes::SHAPE_CAPSULE, shapes::SHAPE_CONVEX_HULL,
                                        shapes::SHAPE_CYLINDER, shapes::SHAPE_STATIC_COMPOUND};
    for (shapes::ShapeType t : others) {
        if (t != shapes::SHAPE_STATIC_COMPOUND) {
            CHECK(ctx, shapes::collide_fn(shapes::SHAPE_STATIC_COMPOUND, t) == static_compound::collide_static_compound_vs_shape);
            CHECK(ctx, shapes::cast_fn(shapes::SHAPE_STATIC_COMPOUND, t) == static_compound::cast_static_compound_vs_shape);
        }
        CHECK(ctx, shapes::collide_fn(t, shapes::SHAPE_STATIC_COMPOUND) == static_compound::collide_shape_vs_static_compound);
        CHECK(ctx, shapes::cast_fn(t, shapes::SHAPE_STATIC_COMPOUND) == static_compound::cast_shape_vs_static_compound);
    }
    // where the two compounds meet the static compound's rows lie over the compound's
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_STATIC_COMPOUND, shapes::SHAPE_COMPOUND) == static_compound::collide_static_compound_vs_shape);
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_COMPOUND, shapes::SHAPE_STATIC_COMPOUND) == static_compound::collide_shape_vs_static_compound);
    CHECK(ctx, shapes::collide_fn(shapes::SHAPE_STATIC_COMPOUND, shapes::SHAPE_EMPTY) == nullptr);

    const shapes::ShapeDef &def = shapes::shape_defs[shapes::shape_type_index(shapes::SHAPE_STATIC_COMPOUND)];
    CHECK(ctx, def.type == shapes::SHAPE_STATIC_COMPOUND && def.set_support == nullptr);
    CHECK(ctx, def.cast_ray == static_compound::cast_ray && def.collide_point == static_compound::collide_point);
    CHECK(ctx, def.get_leaf_shape == compound::get_leaf_shape && def.compute_mass_properties == compound::compute_mass_properties);
}

void test_static_compound_grid_cast_sphere(TestCtx &ctx) {
    dispatch::init();
    Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.0f});
    Shape *grid = make_grid(b);
    Shape *s = sphere::create({.radius = 0.25f});
    const cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    cast_shape::CastShapeCollector closest = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    cast_shape::CastShapeCollector all = cast_shape::create_collector(collector::COLLECT_ALL);

    // a sphere dropped from y = 5 onto every cell: its bottom at 4.75 meets the top at 0.5 after 4.25 of 10
    uint32_t right_cell = 0;
    uint32_t single_hit = 0;
    float worst_fraction_error = 0.0f;
    for (uint32_t i = 0; i < GRID; i++) {
        for (uint32_t j = 0; j < GRID; j++) {
            const Float3 cell = {(float)i * SPACING, 0, (float)j * SPACING};
            const Float3 start = {cell.x + 0.1f, 5, cell.z - 0.2f};
            cast_shape::reset(closest);
            shapes::cast_fn(s->type, grid->type)(closest, settings, s, NO_SUB_SHAPE, pose(start), vec3f::make(0, -10, 0), grid, NO_SUB_SHAPE,
                                                 pose({0, 0, 0}));
            if (!closest.has_hit) continue;
            worst_fraction_error = scalar::max(worst_fraction_error, scalar::abs(closest.hits[0].fraction - 0.425f));
            if (near3(child_of(grid, closest.hits[0].sub_shape_id_b).position, cell, 0.0f)) right_cell++;

            cast_shape::reset(all);
            shapes::cast_fn(s->type, grid->type)(all, settings, s, NO_SUB_SHAPE, pose(start), vec3f::make(0, -10, 0), grid, NO_SUB_SHAPE, pose({0, 0, 0}));
            if (all.hits.length == 1) single_hit++;
        }
    }
    CHECK(ctx, right_cell == GRID * GRID);
    CHECK(ctx, single_hit == GRID * GRID);
    CHECK(ctx, worst_fraction_error < 1e-3f);

    // between the cells: nothing
    cast_shape::reset(all);
    shapes::cast_fn(s->type, grid->type)(all, settings, s, NO_SUB_SHAPE, pose({1.5f, 5, 1.5f}), vec3f::make(0, -10, 0), grid, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, all.hits.length == 0);

    // sideways along a row at z = 6: the nearest cell first, at x = -0.5 after 9.25 of 20
    cast_shape::reset(closest);
    shapes::cast_fn(s->type, grid->type)(closest, settings, s, NO_SUB_SHAPE, pose({-10, 0, 6}), vec3f::make(20, 0, 0), grid, NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, closest.has_hit);
    CHECK_NEAR(ctx, closest.hits[0].fraction, 9.25f / 20.0f, 1e-3f);
    CHECK(ctx, near3(child_of(grid, closest.hits[0].sub_shape_id_b).position, 0, 0, 6, 0.0f));

    // the grid swept up into a resting sphere above cell (4, 7): the same fraction, the ids on a
    const Float3 cell = {4 * SPACING, 0, 7 * SPACING};
    cast_shape::reset(closest);
    shapes::cast_fn(grid->type, s->type)(closest, settings, grid, NO_SUB_SHAPE, pose({0, -5, 0}), vec3f::make(0, 10, 0), s, NO_SUB_SHAPE, pose(cell));
    CHECK(ctx, closest.has_hit);
    CHECK_NEAR(ctx, closest.hits[0].fraction, 0.425f, 1e-3f);
    CHECK(ctx, near3(child_of(grid, closest.hits[0].sub_shape_id_a).position, cell, 0.0f));

    // the grid moved and turned a quarter about y: cell (i, j) now at (3 j, 0, -3 i) + (100, 0, 0)
    Float4 quarter_y = stored_axis_angle(vec3f::make(0, 1, 0), scalar::PI * 0.5f);
    cast_shape::reset(closest);
    shapes::cast_fn(s->type, grid->type)(closest, settings, s, NO_SUB_SHAPE, pose({100 + 7 * SPACING, 5, -4 * SPACING}), vec3f::make(0, -10, 0), grid,
                                         NO_SUB_SHAPE, pose({100, 0, 0}, quarter_y));
    CHECK(ctx, closest.has_hit);
    CHECK_NEAR(ctx, closest.hits[0].fraction, 0.425f, 1e-3f);
    CHECK(ctx, near3(child_of(grid, closest.hits[0].sub_shape_id_b).position, cell, 0.0f));

    cast_shape::release(closest);
    cast_shape::release(all);
    static_compound::destroy(grid);
    box::destroy(b);
    sphere::destroy(s);
}

void test_static_compound_grid_collide(TestCtx &ctx) {
    dispatch::init();
    Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    Shape *grid = make_grid(b);
    collide_point::CollidePointCollector points = collide_point::create_collector(collector::COLLECT_ALL);

    // a point inside each cell finds that cell alone
    uint32_t right_cell = 0;
    for (uint32_t i = 0; i < GRID; i++) {
        for (uint32_t j = 0; j < GRID; j++) {
            const Float3 cell = {(float)i * SPACING, 0, (float)j * SPACING};
            collide_point::reset(points);
            collide_point_at(points, grid, pose({0, 0, 0}), {cell.x + 0.2f, 0.1f, cell.z - 0.3f});
            if (points.hits.length == 1 && near3(child_of(grid, points.hits[0].sub_shape_id_b).position, cell, 0.0f)) right_cell++;
        }
    }
    CHECK(ctx, right_cell == GRID * GRID);
    // between cells and outside the grid: nothing
    collide_point::reset(points);
    collide_point_at(points, grid, pose({0, 0, 0}), {1.5f, 0, 1.5f});
    CHECK(ctx, points.hits.length == 0);
    collide_point::reset(points);
    collide_point_at(points, grid, pose({0, 0, 0}), {-5, 0, 0});
    CHECK(ctx, points.hits.length == 0);

    // a sphere at the corner of four cells touches exactly those four
    Shape *big = box::create({.half_extents = {1.2f, 0.5f, 1.2f}});
    Shape *coarse = make_grid(big);
    Shape *s = sphere::create({.radius = 1.0f});
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    shapes::collide_fn(s->type, coarse->type)(collector, collide_shape::default_settings(), s, NO_SUB_SHAPE, pose({4.5f, 0, 7.5f}), coarse,
                                              NO_SUB_SHAPE, pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 4);
    bool corners[4] = {false, false, false, false};
    for (uint32_t h = 0; h < collector.hits.length; h++) {
        const Float3 &p = child_of(coarse, collector.hits[h].sub_shape_id_b).position;
        const int32_t ci = (int32_t)(p.x / SPACING) - 1;
        const int32_t cj = (int32_t)(p.z / SPACING) - 2;
        if (ci >= 0 && ci < 2 && cj >= 0 && cj < 2) corners[ci * 2 + cj] = true;
    }
    CHECK(ctx, corners[0] && corners[1] && corners[2] && corners[3]);

    // the other way round, and with the grid scaled by 2: the sphere at the scaled corner
    collide_shape::reset(collector);
    shapes::collide_fn(coarse->type, s->type)(collector, collide_shape::default_settings(), coarse, NO_SUB_SHAPE,
                                              pose({0, 0, 0}, IDENTITY, {2, 2, 2}), s, NO_SUB_SHAPE, pose({9, 0, 15}));
    CHECK(ctx, collector.hits.length == 4);

    collide_shape::release(collector);
    collide_point::release(points);
    static_compound::destroy(grid);
    static_compound::destroy(coarse);
    box::destroy(b);
    box::destroy(big);
    sphere::destroy(s);
}

static uint32_t dyn_child_index(sub_shape::SubShapeId id) {
    sub_shape::PopResult popped;
    sub_shape::pop_index(popped, id, 64);
    return popped.value;
}

void test_static_compound_agrees_with_compound(TestCtx &ctx) {
    dispatch::init();
    // the bvh walk finds what walking every child finds, for rays and points in and around a scattered set
    Shape *b = box::create({.half_extents = {0.4f, 0.7f, 0.3f}});
    Shape *s = sphere::create({.radius = 0.6f});
    uint32_t state = 12345;
    CompoundShapeChild scattered[64];
    for (uint32_t i = 0; i < 64; i++) {
        const Float3 position = {mulberry32_unit(state) * 20 - 10, mulberry32_unit(state) * 20 - 10, mulberry32_unit(state) * 20 - 10};
        Float4 rotation = stored_axis_angle(vec3f::make(0, 1, 0), mulberry32_unit(state) * 6.0f);
        scattered[i] = {position, rotation, (i & 1) ? b : s};
    }
    Shape *st = make_static(scattered, 64, 2);
    Shape *dyn = compound::create({.children = scattered, .num_children = 64});
    const Transformf placed = pose({1, 2, 3}, IDENTITY, {1, 1, 1});

    cast_ray::CastRayCollector st_ray = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    cast_ray::CastRayCollector dyn_ray = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    collide_point::CollidePointCollector st_point = collide_point::create_collector(collector::COLLECT_ALL);
    collide_point::CollidePointCollector dyn_point = collide_point::create_collector(collector::COLLECT_ALL);
    uint32_t ray_agree = 0, ray_hits = 0, point_agree = 0, point_hits = 0;
    const uint32_t trials = 400;
    for (uint32_t t = 0; t < trials; t++) {
        // rays from anywhere around the set aimed near a random child, points near another
        const Float3 origin = {mulberry32_unit(state) * 30 - 14, mulberry32_unit(state) * 30 - 13, mulberry32_unit(state) * 30 - 12};
        const Float3 &aim = scattered[(uint32_t)(mulberry32_unit(state) * 64) & 63].position;
        const Float3 target = {aim.x + 1 + mulberry32_unit(state) * 3 - 1.5f, aim.y + 2 + mulberry32_unit(state) * 3 - 1.5f,
                              aim.z + 3 + mulberry32_unit(state) * 3 - 1.5f};
        Float3 direction;
        vec3f::store(direction, vec3f::subtract(vec3f::load(target), vec3f::load(origin)));
        const Float3 &near_child = scattered[(uint32_t)(mulberry32_unit(state) * 64) & 63].position;
        const Float3 point = {near_child.x + 1 + mulberry32_unit(state) - 0.5f, near_child.y + 2 + mulberry32_unit(state) - 0.5f,
                             near_child.z + 3 + mulberry32_unit(state) - 0.5f};
        cast_ray::reset(st_ray);
        cast_ray::reset(dyn_ray);
        cast_ray_at(st_ray, st, placed, origin, direction, 25.0f);
        cast_ray_at(dyn_ray, dyn, placed, origin, direction, 25.0f);
        if (st_ray.has_hit == dyn_ray.has_hit &&
            (!st_ray.has_hit || (scalar::abs(st_ray.hits[0].fraction - dyn_ray.hits[0].fraction) <= 1e-6f &&
                                 near3(child_of(st, st_ray.hits[0].sub_shape_id).position,
                                       dyn->compound.data->children[dyn_child_index(dyn_ray.hits[0].sub_shape_id)].position, 0.0f)))) {
            ray_agree++;
        }
        if (st_ray.has_hit) ray_hits++;

        collide_point::reset(st_point);
        collide_point::reset(dyn_point);
        collide_point_at(st_point, st, placed, point);
        collide_point_at(dyn_point, dyn, placed, point);
        if (st_point.hits.length == dyn_point.hits.length) point_agree++;
        if (st_point.hits.length > 0) point_hits++;
    }
    CHECK(ctx, ray_agree == trials);
    CHECK(ctx, point_agree == trials);
    // the loop reaches both outcomes
    CHECK(ctx, ray_hits > trials / 4 && ray_hits < trials);
    CHECK(ctx, point_hits > trials / 10 && point_hits < trials);

    cast_ray::release(st_ray);
    cast_ray::release(dyn_ray);
    collide_point::release(st_point);
    collide_point::release(dyn_point);
    static_compound::destroy(st);
    compound::destroy(dyn);
    box::destroy(b);
    sphere::destroy(s);
}

void test_static_compound_nested(TestCtx &ctx) {
    dispatch::init();
    // static compounds inside a static compound: each walk keeps its own stack and bounds
    Shape *b = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    CompoundShapeChild row[8];
    for (uint32_t i = 0; i < 8; i++) row[i] = {{(float)i * 2.0f, 0, 0}, IDENTITY, b};
    Shape *inner = make_static(row, 8, 1);
    CompoundShapeChild rows[8];
    for (uint32_t i = 0; i < 8; i++) rows[i] = {{0, 0, (float)i * 2.0f}, IDENTITY, inner};
    Shape *outer = make_static(rows, 8, 1);

    // a sphere spanning four cells of the 8 x 8 grid of boxes
    Shape *s = sphere::create({.radius = 0.9f});
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    shapes::collide_fn(s->type, outer->type)(collector, collide_shape::default_settings(), s, NO_SUB_SHAPE, pose({7, 0, 9}), outer, NO_SUB_SHAPE,
                                             pose({0, 0, 0}));
    CHECK(ctx, collector.hits.length == 4);
    collide_shape::reset(collector);
    shapes::collide_fn(outer->type, s->type)(collector, collide_shape::default_settings(), outer, NO_SUB_SHAPE, pose({0, 0, 0}), s, NO_SUB_SHAPE,
                                             pose({7, 0, 9}));
    CHECK(ctx, collector.hits.length == 4);

    // every box is reached by a point, through both levels of ids
    collide_point::CollidePointCollector points = collide_point::create_collector(collector::COLLECT_ALL);
    uint32_t found = 0;
    for (uint32_t x = 0; x < 8; x++) {
        for (uint32_t z = 0; z < 8; z++) {
            collide_point::reset(points);
            collide_point_at(points, outer, pose({0, 0, 0}), {(float)x * 2.0f + 0.1f, 0.2f, (float)z * 2.0f - 0.1f});
            if (points.hits.length != 1) continue;
            sub_shape::PopResult outer_level;
            sub_shape::pop_index(outer_level, points.hits[0].sub_shape_id_b, 8);
            sub_shape::PopResult inner_level;
            sub_shape::pop_index(inner_level, outer_level.remainder, 8);
            const Float3 &row_position = children_of(outer)[outer_level.value].position;
            const Float3 &box_position = children_of(inner)[inner_level.value].position;
            if (row_position.z == (float)z * 2.0f && box_position.x == (float)x * 2.0f) found++;
        }
    }
    CHECK(ctx, found == 64);

    collide_shape::release(collector);
    collide_point::release(points);
    static_compound::destroy(outer);
    static_compound::destroy(inner);
    box::destroy(b);
    sphere::destroy(s);
}

/* sub shape ids: the last child of a power of two count pushed onto an empty id is all ones, the empty id's bits */

static sub_shape::SubShapeId ray_down_sub_shape_id(TestCtx &ctx, const Shape *shape, float x) {
    cast_ray::CastRayCollector rays = cast_ray::create_collector(collector::COLLECT_ALL);
    cast_ray_at(rays, shape, pose({0, 0, 0}), {x, 10, 0}, {0, -1, 0}, 20.0f);
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

void test_static_compound_sub_shape_id_last_of_two(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    Shape *leaf = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeChild children[] = {{{-2, 0, 0}, IDENTITY, s}, {{2, 0, 0}, IDENTITY, leaf}};
    Shape *c = make_static(children, 2);
    CHECK(ctx, children_of(c)[1].shape == leaf);

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

    static_compound::destroy(c);
    sphere::destroy(s);
    box::destroy(leaf);
}

void test_static_compound_sub_shape_id_path_one_one(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f});
    Shape *leaf = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    const CompoundShapeChild inner_children[] = {{{-2, 0, 0}, IDENTITY, s}, {{2, 0, 0}, IDENTITY, leaf}};
    Shape *inner = make_static(inner_children, 2);
    const CompoundShapeChild outer_children[] = {{{-2, 0, 0}, IDENTITY, s}, {{2, 0, 0}, IDENTITY, inner}};
    Shape *outer = make_static(outer_children, 2);

    // inner child 1 sits at 2 + 2 along x
    const sub_shape::SubShapeId id = ray_down_sub_shape_id(ctx, outer, 4);
    CHECK(ctx, id == 0xffffffffu);
    shapes::GetLeafShapeResult leaf_result;
    shapes::get_leaf_shape(leaf_result, outer, id);
    CHECK(ctx, leaf_result.shape == leaf);
    const shapes::GetSubShapeTransformedShapeResult transformed = transformed_shape_of(outer, id, {1, 1, 1});
    CHECK(ctx, transformed.shape == leaf);
    CHECK(ctx, vec3f::x(transformed.position) == 4.0f);

    static_compound::destroy(outer);
    static_compound::destroy(inner);
    sphere::destroy(s);
    box::destroy(leaf);
}

void test_static_compound_nested_rotated_surface_normal(TestCtx &ctx) {
    dispatch::init();
    // outer turns its child a quarter about z, inner turns its box a quarter about x
    Shape *leaf = box::create({.half_extents = {1, 2, 3}});
    const CompoundShapeChild inner_children[] = {{{0, 0, 0}, stored_axis_angle(vec3f::make(1, 0, 0), scalar::PI * 0.5f), leaf}};
    Shape *inner = make_static(inner_children, 1);
    const CompoundShapeChild outer_children[] = {{{0, 0, 0}, stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f), inner}};
    Shape *outer = make_static(outer_children, 1);

    // the box's +y face centre (0, 2, 0) goes to (0, 0, 2) under x, which z leaves alone
    const Vec3f normal = shapes::get_shape_surface_normal(outer, vec3f::make(0, 0, 2), sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, near3(normal, 0, 0, 1, 1e-6f));

    static_compound::destroy(outer);
    static_compound::destroy(inner);
    box::destroy(leaf);
}

void test_static_compound_scaled_supporting_face(TestCtx &ctx) {
    dispatch::init();
    Shape *b = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeChild children[] = {{{-2, 0, 0}, IDENTITY, b}, {{2, 0, 0}, IDENTITY, b}};
    Shape *c = make_static(children, 2);
    CHECK(ctx, children_of(c)[1].position.x == 2.0f);
    Float4x4 identity;
    mat4f::store(identity, mat4f::identity());

    // the second box under scale (2, 1, 1): centre x = 4, half extent 2, so its +x face is at x = 6
    face::Face face;
    shapes::get_shape_supporting_face(face, c, 0xffffffffu, vec3f::make(-1, 0, 0), mat4f::load(identity), vec3f::make(2, 1, 1));
    CHECK(ctx, face.num_vertices == 4);
    for (uint32_t i = 0; i < 4; i++) CHECK_NEAR(ctx, vec3f::x(face.vertices[i]), 6.0f, 1e-6f);

    static_compound::destroy(c);
    box::destroy(b);
}

void test_static_compound_as_a_culls_with_b_scale(TestCtx &ctx) {
    dispatch::init();
    Shape *b = box::create({.half_extents = {1, 1, 1}});
    const CompoundShapeChild children[] = {{{0, 0, 0}, IDENTITY, b}};
    Shape *c = make_static(children, 1);
    Shape *s = sphere::create({.radius = 1.0f});

    // collide: a unit sphere scaled by 3 at x = 3.5 reaches x = 0.5, inside the box. unscaled, its
    // bounds start at x = 2.5 and miss the box's [-1, 1]
    collide_shape::CollideShapeCollector collides = collide_shape::create_collector(collector::COLLECT_ALL);
    shapes::collide_fn(c->type, s->type)(collides, collide_shape::default_settings(), c, NO_SUB_SHAPE, pose({0, 0, 0}), s, NO_SUB_SHAPE,
                                         pose({3.5f, 0, 0}, IDENTITY, {3, 3, 3}));
    CHECK(ctx, collides.hits.length == 1);
    if (collides.hits.length == 1) CHECK_NEAR(ctx, collides.hits[0].penetration, 0.5f, 1e-4f);

    // cast: the compound moving 4 along x; its +x face travels 1 -> 5 and meets the sphere's near side
    // at x = 4 three quarters of the way. unscaled, the sphere's swept bounds [2, 8] miss the box
    cast_shape::CastShapeCollector casts = cast_shape::create_collector(collector::COLLECT_ALL);
    shapes::cast_fn(c->type, s->type)(casts, cast_shape::default_settings(), c, NO_SUB_SHAPE, pose({0, 0, 0}), vec3f::make(4, 0, 0), s, NO_SUB_SHAPE,
                                      pose({7, 0, 0}, IDENTITY, {3, 3, 3}));
    CHECK(ctx, casts.hits.length == 1);
    if (casts.hits.length == 1) CHECK_NEAR(ctx, casts.hits[0].fraction, 0.75f, 1e-3f);

    collide_shape::release(collides);
    cast_shape::release(casts);
    static_compound::destroy(c);
    box::destroy(b);
    sphere::destroy(s);
}

static bool box_contains(const Box3f &outer, const Box3f &inner) {
    return outer.min.x <= inner.min.x && outer.min.y <= inner.min.y && outer.min.z <= inner.min.z && outer.max.x >= inner.max.x &&
           outer.max.y >= inner.max.y && outer.max.z >= inner.max.z;
}

void test_static_compound_bvh_bounds_exact(TestCtx &ctx) {
    dispatch::init();
    Shape *b = box::create({.half_extents = {0.21f, 0.07f, 0.13f}, .convex_radius = 0.0f});
    CompoundShapeChild children[64];
    for (uint32_t i = 0; i < 64; i++) {
        children[i] = {{(float)i * 0.1f + 0.37f, (float)(i % 7) * 0.13f - 0.3f, (float)(i % 5) * 0.29f + 0.01f}, IDENTITY, b};
    }
    Shape *shape = make_static(children, 64, 2);
    const collections::Vec<bvh::Node> &tree = shape->static_compound.data->bvh;

    // every node contains its children with zero tolerance
    uint32_t checked = 0;
    for (uint32_t node = 0; node < tree.length; node++) {
        const bvh::Node &n = tree[node];
        if (bvh::node_is_leaf(tree.data, (int32_t)node)) {
            const uint32_t start = static_compound_bvh::node_child_start(tree.data, (int32_t)node);
            const uint32_t count = static_compound_bvh::node_child_count(tree.data, (int32_t)node);
            for (uint32_t i = start; i < start + count; i++) {
                const CompoundShapeChild &child = children_of(shape)[i];
                const Mat4f transform = mat4f::from_rotation_translation(quatf::load(child.quaternion), vec3f::load(child.position));
                Box3f child_bounds;
                aabox::store(child_bounds, aabox::transform_mat4(aabox::load(child.shape->aabb), transform));
                CHECK(ctx, box_contains(n.bounds, child_bounds));
                checked++;
            }
        } else {
            CHECK(ctx, box_contains(n.bounds, tree[(uint32_t)bvh::node_left((int32_t)node)].bounds));
            CHECK(ctx, box_contains(n.bounds, tree[(uint32_t)bvh::node_right(tree.data, (int32_t)node)].bounds));
        }
    }
    CHECK(ctx, checked == 64);

    static_compound::destroy(shape);
    box::destroy(b);
}

/* material by sub shape id */

void test_static_compound_material(TestCtx &ctx) {
    dispatch::init();
    Shape *s = sphere::create({.radius = 1.0f, .material_id = 2});
    Shape *b = box::create({.half_extents = {1, 1, 1}, .material_id = 5});
    const CompoundShapeChild children[] = {{{-4, 0, 0}, IDENTITY, s}, {{0, 0, 0}, IDENTITY, b}, {{4, 0, 0}, IDENTITY, s}};
    Shape *c = make_static(children, 3);

    // the child the first level names, whatever order the bvh keeps the children in
    uint32_t matched = 0;
    for (uint32_t i = 0; i < 3; i++) {
        sub_shape::SubShapeIdBuilder id;
        sub_shape::push_index(id, sub_shape::builder(), i, 3);
        const int32_t expected = children_of(c)[i].shape == b ? 5 : 2;
        if (shapes::get_shape_material(c, id.value) == expected) matched++;
    }
    CHECK(ctx, matched == 3);

    // a ray hit's id looks up the material the hit reported
    cast_ray::CastRayCollector rays = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    cast_ray_at(rays, c, pose({0, 0, 0}), {0, 10, 0}, {0, -1, 0}, 20.0f);
    CHECK(ctx, rays.has_hit && shapes::get_shape_material(c, rays.hits[0].sub_shape_id) == 5);
    CHECK(ctx, rays.has_hit && shapes::get_shape_material(c, rays.hits[0].sub_shape_id) == 5);
    cast_ray::release(rays);

    static_compound::destroy(c);
    box::destroy(b);
    sphere::destroy(s);
}
