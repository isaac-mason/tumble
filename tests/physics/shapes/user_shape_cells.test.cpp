#include "physics/world_test_scene.h"
#include "test.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/mass_properties.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex.h"

// a grid of 1 m cubes in user slot 2, its sub shape id naming a cell, as a voxel shape is: not convex, so its own collide
// and cast rows hand each solid cell's cube to the narrowphase, and every call that takes a sub shape id reads the
// cell back. a sphere of its own in user slot 1 stands for any other user type

namespace {

struct CellsShape : shapes::Shape {
    static constexpr shapes::ShapeType TYPE = shapes::SHAPE_USER_2;
    /** cells a side, in local space [0, size) on each axis */
    uint32_t size;
    /** size^3, x fastest then z then y */
    const uint8_t *solid;
    /** a material id per cell */
    const int32_t *materials;
    /** the cube every cell is, its centre at the cell's */
    shapes::Shape *cube;
};

const CellsShape &cells_of(const shapes::Shape *shape) { return shapes::as<CellsShape>(shape); }

uint32_t cell_count(const CellsShape &cells) { return cells.size * cells.size * cells.size; }

void cell_at(uint32_t &x, uint32_t &y, uint32_t &z, const CellsShape &cells, uint32_t index) {
    x = index % cells.size;
    z = (index / cells.size) % cells.size;
    y = index / (cells.size * cells.size);
}

compound::CompoundShapeChild cell_child(const CellsShape &cells, uint32_t index) {
    uint32_t x, y, z;
    cell_at(x, y, z, cells, index);
    return {{(float)x + 0.5f, (float)y + 0.5f, (float)z + 0.5f}, {0.0f, 0.0f, 0.0f, 1.0f}, cells.cube};
}

/** the cell an id names and what is left of the id below it; false for an id naming no solid cell */
bool pop_cell(uint32_t &out_index, sub_shape::SubShapeId &out_remainder, const CellsShape &cells, sub_shape::SubShapeId sub_shape_id) {
    sub_shape::PopResult popped;
    sub_shape::pop_index(popped, sub_shape_id, cell_count(cells));
    if (popped.value >= cell_count(cells) || cells.solid[popped.value] == 0) return false;
    out_index = popped.value;
    out_remainder = popped.remainder;
    return true;
}

void cells_compute_mass_properties(mass_properties::MassProperties &out, const shapes::Shape *shape) {
    mass_properties::set_mass_and_inertia_of_solid_box(out, aabox::size(aabox::load(shape->aabb)), 1000.0f);
}

void cells_get_surface_normal(shapes::SurfaceNormalResult &io, const shapes::Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const CellsShape &cells = cells_of(shape);
    uint32_t index;
    sub_shape::SubShapeId remainder;
    if (!pop_cell(index, remainder, cells, sub_shape_id)) return;
    const compound::CompoundShapeChild child = cell_child(cells, index);
    io.position = vec3f::subtract(io.position, vec3f::load(child.position));
    shapes::def_of(cells.cube).get_surface_normal(io, cells.cube, remainder);
}

void cells_get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const shapes::Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const CellsShape &cells = cells_of(shape);
    uint32_t index;
    sub_shape::SubShapeId remainder;
    if (!pop_cell(index, remainder, cells, sub_shape_id)) return;
    const compound::CompoundShapeChild child = cell_child(cells, index);
    io.transform = mat4f::multiply(io.transform, mat4f::from_rotation_translation(quatf::identity(), vec3f::multiply(vec3f::load(child.position), io.scale)));
    shapes::def_of(cells.cube).get_supporting_face(io, direction, cells.cube, remainder);
}

int32_t cells_get_material(const shapes::Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const CellsShape &cells = cells_of(shape);
    uint32_t index;
    sub_shape::SubShapeId remainder;
    return pop_cell(index, remainder, cells, sub_shape_id) ? cells.materials[index] : -1;
}

void cells_get_leaf_shape(shapes::GetLeafShapeResult &out, const shapes::Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const CellsShape &cells = cells_of(shape);
    uint32_t index;
    sub_shape::SubShapeId remainder;
    if (!pop_cell(index, remainder, cells, sub_shape_id)) {
        out.shape = nullptr;
        out.remainder = sub_shape::EMPTY_SUB_SHAPE_ID;
        return;
    }
    shapes::get_leaf_shape(out, cells.cube, remainder);
}

float cells_get_inner_radius(const shapes::Shape *) { return 0.5f; }

void cells_cast_ray(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                    const shapes::Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    const CellsShape &cells = cells_of(shape);
    for (uint32_t i = 0; i < cell_count(cells); i++) {
        if (cells.solid[i] == 0) continue;
        sub_shape::SubShapeIdBuilder cell_id;
        sub_shape::push_index(cell_id, sub_shape_id, i, cell_count(cells));
        Transformf at;
        compound::child_transform(at, transform, cell_child(cells, i));
        shapes::def_of(cells.cube).cast_ray(collector, settings, ray, cells.cube, cell_id, at);
    }
}

void cells_collide_point(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings, Vec3f point,
                         const shapes::Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    const CellsShape &cells = cells_of(shape);
    for (uint32_t i = 0; i < cell_count(cells); i++) {
        if (cells.solid[i] == 0) continue;
        sub_shape::SubShapeIdBuilder cell_id;
        sub_shape::push_index(cell_id, sub_shape_id, i, cell_count(cells));
        Transformf at;
        compound::child_transform(at, transform, cell_child(cells, i));
        shapes::def_of(cells.cube).collide_point(collector, settings, point, cells.cube, cell_id, at);
    }
}

/** the cells as shape a: each solid cell's cube against b, through the row its type and b's take */
void collide_cells_vs_shape(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                            const shapes::Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                            const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const CellsShape &cells = cells_of(shape_a);
    for (uint32_t i = 0; i < cell_count(cells); i++) {
        if (cells.solid[i] == 0) continue;
        sub_shape::SubShapeIdBuilder cell_id;
        sub_shape::push_index(cell_id, sub_shape_id_a, i, cell_count(cells));
        Transformf at;
        compound::child_transform(at, transform_a, cell_child(cells, i));
        shapes::collide_fn(cells.cube->type, shape_b->type)(collector, settings, cells.cube, cell_id, at, shape_b, sub_shape_id_b, transform_b);
    }
}

void collide_shape_vs_cells(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                            const shapes::Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                            const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const CellsShape &cells = cells_of(shape_b);
    for (uint32_t i = 0; i < cell_count(cells); i++) {
        if (cells.solid[i] == 0) continue;
        sub_shape::SubShapeIdBuilder cell_id;
        sub_shape::push_index(cell_id, sub_shape_id_b, i, cell_count(cells));
        Transformf at;
        compound::child_transform(at, transform_b, cell_child(cells, i));
        shapes::collide_fn(shape_a->type, cells.cube->type)(collector, settings, shape_a, sub_shape_id_a, transform_a, cells.cube, cell_id, at);
    }
}

void cast_shape_vs_cells(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const shapes::Shape *shape_a,
                         const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                         const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const CellsShape &cells = cells_of(shape_b);
    for (uint32_t i = 0; i < cell_count(cells); i++) {
        if (cells.solid[i] == 0) continue;
        sub_shape::SubShapeIdBuilder cell_id;
        sub_shape::push_index(cell_id, sub_shape_id_b, i, cell_count(cells));
        Transformf at;
        compound::child_transform(at, transform_b, cell_child(cells, i));
        shapes::cast_fn(shape_a->type, cells.cube->type)(collector, settings, shape_a, sub_shape_id_a, transform_a, displacement_a, cells.cube,
                                                         cell_id, at);
    }
}

void cast_cells_vs_shape(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const shapes::Shape *shape_a,
                         const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                         const shapes::Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const CellsShape &cells = cells_of(shape_a);
    for (uint32_t i = 0; i < cell_count(cells); i++) {
        if (cells.solid[i] == 0) continue;
        sub_shape::SubShapeIdBuilder cell_id;
        sub_shape::push_index(cell_id, sub_shape_id_a, i, cell_count(cells));
        Transformf at;
        compound::child_transform(at, transform_a, cell_child(cells, i));
        shapes::cast_fn(cells.cube->type, shape_b->type)(collector, settings, cells.cube, cell_id, at, displacement_a, shape_b, sub_shape_id_b,
                                                         transform_b);
    }
}

/** the user sphere: the built-in sphere's functions over an inner shape */
struct BallShape : shapes::Shape {
    static constexpr shapes::ShapeType TYPE = shapes::SHAPE_USER_1;
    shapes::Shape *inner;
};
const shapes::Shape *inner_of(const shapes::Shape *shape) { return shapes::as<BallShape>(shape).inner; }
void ball_set_support(support::Support &out, const shapes::Shape *shape, support::SupportFunctionMode mode, Vec3f scale) {
    sphere::set_support(out, inner_of(shape), mode, scale);
}
void ball_compute_mass_properties(mass_properties::MassProperties &out, const shapes::Shape *shape) {
    sphere::compute_mass_properties(out, inner_of(shape));
}
void ball_get_surface_normal(shapes::SurfaceNormalResult &io, const shapes::Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    sphere::get_surface_normal(io, inner_of(shape), sub_shape_id);
}
void ball_get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const shapes::Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    sphere::get_supporting_face(io, direction, inner_of(shape), sub_shape_id);
}
int32_t ball_get_material(const shapes::Shape *shape, sub_shape::SubShapeId) { return shape->material_id; }
float ball_get_inner_radius(const shapes::Shape *shape) { return sphere::get_inner_radius(inner_of(shape)); }

/** the two user types beside the built-ins, the cells' rows against every convex type, user ones too, and themselves */
void init_cells() {
    shapes::ShapeDef defs[2] = {};
    defs[0].type = shapes::SHAPE_USER_1;
    defs[0].set_support = ball_set_support;
    defs[0].compute_mass_properties = ball_compute_mass_properties;
    defs[0].get_surface_normal = ball_get_surface_normal;
    defs[0].get_supporting_face = ball_get_supporting_face;
    defs[0].get_material = ball_get_material;
    defs[0].get_inner_radius = ball_get_inner_radius;
    defs[0].cast_ray = convex::cast_ray_vs_convex;
    defs[0].collide_point = convex::collide_point_vs_convex;
    defs[0] = shapes::define_shape(defs[0]);

    defs[1].type = shapes::SHAPE_USER_2;
    defs[1].compute_mass_properties = cells_compute_mass_properties;
    defs[1].get_surface_normal = cells_get_surface_normal;
    defs[1].get_supporting_face = cells_get_supporting_face;
    defs[1].get_material = cells_get_material;
    defs[1].get_inner_radius = cells_get_inner_radius;
    defs[1].get_leaf_shape = cells_get_leaf_shape;
    defs[1].cast_ray = cells_cast_ray;
    defs[1].collide_point = cells_collide_point;
    defs[1].skips_pair_cache = true;
    defs[1] = shapes::define_shape(defs[1]);
    dispatch::init(defs, 2);

    for (uint32_t type = 0; type < shapes::SHAPE_TYPE_COUNT; type++) {
        const shapes::ShapeType other = shapes::shape_defs[type].type;
        if (shapes::shape_defs[type].set_support == nullptr) continue;
        shapes::set_collide_shape_fn(shapes::SHAPE_USER_2, other, collide_cells_vs_shape);
        shapes::set_collide_shape_fn(other, shapes::SHAPE_USER_2, collide_shape_vs_cells);
        shapes::set_cast_shape_fn(other, shapes::SHAPE_USER_2, cast_shape_vs_cells);
        shapes::set_cast_shape_fn(shapes::SHAPE_USER_2, other, cast_cells_vs_shape);
    }
    shapes::set_collide_shape_fn(shapes::SHAPE_USER_2, shapes::SHAPE_USER_2, collide_cells_vs_shape);
    shapes::set_cast_shape_fn(shapes::SHAPE_USER_2, shapes::SHAPE_USER_2, cast_cells_vs_shape);
}

shapes::Shape *create_cells(uint32_t size, const uint8_t *solid, const int32_t *materials, shapes::Shape *cube) {
    CellsShape *shape = shapes::allocate<CellsShape>();
    shape->size = size;
    shape->solid = solid;
    shape->materials = materials;
    shape->cube = cube;
    AABox bounds = aabox::empty();
    float volume = 0.0f;
    for (uint32_t i = 0; i < cell_count(*shape); i++) {
        if (solid[i] == 0) continue;
        uint32_t x, y, z;
        cell_at(x, y, z, *shape, i);
        bounds = aabox::union_(bounds, aabox::make(vec3f::make((float)x, (float)y, (float)z), vec3f::make((float)x + 1.0f, (float)y + 1.0f, (float)z + 1.0f)));
        volume += 1.0f;
    }
    aabox::store(shape->aabb, bounds);
    vec3f::store(shape->center_of_mass, aabox::center(bounds));
    shape->volume = volume;
    shape->material_id = -1;
    return shape;
}

/** a 6 x 6 floor of cells, one deep, every cell its own material: 100 + its index */
struct Floor {
    uint8_t solid[6 * 6 * 6];
    int32_t materials[6 * 6 * 6];
    shapes::Shape *shape;
};

void make_floor(Floor &floor, shapes::Shape *cube) {
    for (uint32_t i = 0; i < 6 * 6 * 6; i++) {
        floor.solid[i] = i < 6 * 6 ? 1 : 0;
        floor.materials[i] = 100 + (int32_t)i;
    }
    floor.shape = create_cells(6, floor.solid, floor.materials, cube);
}

/** the floor's top at y 0, its cells at x and z in [-3, 3) */
rigid_body::RigidBody *add_floor_body(WorldScene &scene, shapes::Shape *floor) {
    return add_body(scene, floor, motion_type::MOTION_STATIC, {-3.0f, -1.0f, -3.0f});
}

rigid_body::RigidBody *add_awake_body(WorldScene &scene, shapes::Shape *shape, const Float3 &position) {
    rigid_body::RigidBodySettings settings = body_settings(shape, scene.moving, motion_type::MOTION_DYNAMIC, position);
    settings.allow_sleeping = false;
    return rigid_body::create(*scene.world, settings);
}

float tilt(const rigid_body::RigidBody *body) { return 2.0f * scalar::acos(scalar::min(1.0f, scalar::abs(body->quaternion.w))); }

}  // namespace

void test_user_shape_cells_hold_a_box_flat_and_their_contacts_name_cells(TestCtx &ctx) {
    shapes::Shape *cube = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    Floor floor;
    make_floor(floor, cube);
    // after the scene, whose making starts the tables over with the built-ins alone
    WorldScene scene = make_world_scene(ctx);
    init_cells();
    rigid_body::RigidBody *ground = add_floor_body(scene, floor.shape);
    rigid_body::RigidBody *crate = add_awake_body(scene, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}), {0.2f, 2.0f, 0.3f});
    for (uint32_t i = 0; i < 120; i++) step_world(*scene.world);

    // resting flat on the cells' faces: the supporting faces the cells' ids name gave a manifold, not a point
    CHECK_NEAR(ctx, crate->position.y, 0.5f, 0.02f);
    CHECK(ctx, tilt(crate) < 0.01f);

    // every contact names a top cell under the crate, and reports that cell's material; skipping the pair cache, the
    // pair collides anew each step and persists
    const events::PhysicsEvents &events = scene.world->events;
    CHECK(ctx, events.contacts_persisted.length > 0);
    for (uint32_t i = 0; i < events.contacts_persisted.length; i++) {
        const events::ContactEvent &event = events.contacts_persisted[i];
        const bool ground_is_a = event.body_id_a == ground->id;
        const sub_shape::SubShapeId id = ground_is_a ? event.sub_shape_id_a : event.sub_shape_id_b;
        uint32_t index;
        sub_shape::SubShapeId remainder;
        CHECK(ctx, pop_cell(index, remainder, cells_of(floor.shape), id));
        uint32_t x, y, z;
        cell_at(x, y, z, cells_of(floor.shape), index);
        CHECK(ctx, y == 0 && (x == 2 || x == 3) && (z == 2 || z == 3));
        CHECK(ctx, (ground_is_a ? event.material_id_a : event.material_id_b) == 100 + (int32_t)index);
    }
    release_world_scene(scene);
    shapes::release<CellsShape>(floor.shape);
    box::destroy(cube);
}

void test_user_shape_cells_body_rests_on_cells_and_beside_a_user_sphere(TestCtx &ctx) {
    shapes::Shape *cube = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    Floor floor;
    make_floor(floor, cube);

    // a plank of two cells, dynamic: its mass from its def, its pairs with the floor through the cells' own rows both ways
    uint8_t plank_solid[8] = {1, 1, 0, 0, 0, 0, 0, 0};
    int32_t plank_materials[8] = {};
    shapes::Shape *plank = create_cells(2, plank_solid, plank_materials, cube);

    BallShape *ball = shapes::allocate<BallShape>();
    ball->inner = sphere::create({.radius = 0.5f});
    ball->aabb = ball->inner->aabb;
    ball->center_of_mass = ball->inner->center_of_mass;
    ball->volume = ball->inner->volume;
    ball->material_id = -1;

    WorldScene scene = make_world_scene(ctx);
    init_cells();
    add_floor_body(scene, floor.shape);
    rigid_body::RigidBody *plank_body = add_awake_body(scene, plank, {-1.7f, 1.5f, 0.2f});
    rigid_body::RigidBody *ball_body = add_awake_body(scene, ball, {1.5f, 2.0f, -1.5f});
    CHECK_NEAR(ctx, plank_body->motion_properties.inv_mass, 1.0f / 2000.0f, 1e-9f);
    for (uint32_t i = 0; i < 180; i++) step_world(*scene.world);

    // the plank's origin is its low corner, so it rests at y 0, flat
    CHECK_NEAR(ctx, plank_body->position.y, 0.0f, 0.02f);
    CHECK(ctx, tilt(plank_body) < 0.01f);
    CHECK_NEAR(ctx, ball_body->position.y, 0.5f, 0.02f);
    release_world_scene(scene);

    sphere::destroy(ball->inner);
    shapes::release<BallShape>(ball);
    shapes::release<CellsShape>(plank);
    shapes::release<CellsShape>(floor.shape);
    box::destroy(cube);
}

void test_user_shape_cells_queries_name_the_cell_they_meet(TestCtx &ctx) {
    shapes::Shape *cube = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    Floor floor;
    make_floor(floor, cube);
    WorldScene scene = make_world_scene(ctx);
    init_cells();
    rigid_body::RigidBody *ground = add_floor_body(scene, floor.shape);
    filter::Filter filter = filter::create(scene.world->settings.layers);

    // straight down over the cell at x 4, z 1 (world x 1.5, z -1.5): the top at y 0, its normal up, read from the cell
    const cast_ray::Rayf ray = {vec3f::make(1.5f, 5.0f, -1.5f), vec3f::make(0.0f, -1.0f, 0.0f), 10.0f};
    cast_ray::CastRayCollector down = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    query::cast_ray(*scene.world, down, cast_ray::default_settings(), ray, filter);
    const cast_ray::CastRayHit *hit = cast_ray::hit(down);
    CHECK(ctx, hit != nullptr);
    if (hit != nullptr) {
        CHECK_NEAR(ctx, hit->fraction, 0.5f, 1e-5f);
        uint32_t index;
        sub_shape::SubShapeId remainder;
        CHECK(ctx, pop_cell(index, remainder, cells_of(floor.shape), hit->sub_shape_id));
        CHECK(ctx, index == 1 * 6 + 4);
        const Vec3f point = vec3f::scale_and_add(ray.origin, ray.direction, ray.length * hit->fraction);
        const Vec3f normal = rigid_body::get_world_space_surface_normal(ground, hit->sub_shape_id, point);
        CHECK(ctx, vec3f::y(normal) > 0.999f);
    }
    cast_ray::release(down);

    // a point inside a cell, and one in the empty layer above
    collide_point::CollidePointCollector inside = collide_point::create_collector(collector::COLLECT_ALL);
    query::collide_point(*scene.world, inside, collide_point::default_settings(), vec3f::make(-2.5f, -0.5f, 2.5f), filter);
    CHECK(ctx, inside.hits.length == 1 && inside.hits[0].sub_shape_id_b != sub_shape::EMPTY_SUB_SHAPE_ID);
    collide_point::reset(inside);
    query::collide_point(*scene.world, inside, collide_point::default_settings(), vec3f::make(-2.5f, 0.5f, 2.5f), filter);
    CHECK(ctx, inside.hits.length == 0);
    collide_point::release(inside);

    // a sphere swept down onto the same cell: it touches at y 0 and names the cell
    shapes::Shape *probe = sphere::create({.radius = 0.25f});
    cast_shape::CastShapeCollector sweep = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    const Transformf start = {{1.5f, 2.25f, -1.5f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    query::cast_shape(*scene.world, sweep, cast_shape::default_settings(), probe, start, vec3f::make(0.0f, -4.0f, 0.0f), filter);
    const cast_shape::CastShapeHit *touch = cast_shape::hit(sweep);
    CHECK(ctx, touch != nullptr);
    if (touch != nullptr) {
        CHECK_NEAR(ctx, touch->fraction, 0.5f, 1e-3f);
        uint32_t index;
        sub_shape::SubShapeId remainder;
        CHECK(ctx, pop_cell(index, remainder, cells_of(floor.shape), touch->sub_shape_id_b) && index == 1 * 6 + 4);
    }
    cast_shape::release(sweep);
    sphere::destroy(probe);

    // a plank of cells swept down onto the floor of cells: its low face, at its origin, meets the top at y 0
    uint8_t plank_solid[8] = {1, 1, 0, 0, 0, 0, 0, 0};
    int32_t plank_materials[8] = {};
    shapes::Shape *plank = create_cells(2, plank_solid, plank_materials, cube);
    cast_shape::CastShapeCollector drop = cast_shape::create_collector(collector::COLLECT_CLOSEST);
    const Transformf above = {{-1.7f, 3.0f, 0.2f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    query::cast_shape(*scene.world, drop, cast_shape::default_settings(), plank, above, vec3f::make(0.0f, -4.0f, 0.0f), filter);
    const cast_shape::CastShapeHit *landing = cast_shape::hit(drop);
    CHECK(ctx, landing != nullptr);
    if (landing != nullptr) {
        CHECK_NEAR(ctx, landing->fraction, 0.75f, 1e-3f);
        uint32_t index;
        sub_shape::SubShapeId remainder;
        CHECK(ctx, pop_cell(index, remainder, cells_of(plank), landing->sub_shape_id_a) && pop_cell(index, remainder, cells_of(floor.shape), landing->sub_shape_id_b));
    }
    cast_shape::release(drop);
    shapes::release<CellsShape>(plank);

    filter::release(filter);
    release_world_scene(scene);
    shapes::release<CellsShape>(floor.shape);
    box::destroy(cube);
}
