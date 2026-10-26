#include "tumble/util/error.h"
#include "tumble/math/mat4f.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/collision/cast_utils.h"
#include "tumble/physics/collision/estimate_collision_response.h"
#include "tumble/physics/collision/internal_edge_removing_collector.h"
#include "tumble/physics/ccd.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/physics/constraints/contact_constraints.h"
#include "tumble/physics/constraints/constraint_part/axis_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/motor_settings.h"
#include "tumble/physics/contacts.h"
#include "tumble/physics/events.h"
#include "tumble/physics/islands.h"
#include "tumble/physics/pairs.h"
#include "tumble/physics/query.h"
#include "tumble/physics/update.h"
#include "tumble/physics/world.h"
#include "tumble/physics/world_settings.h"
#include "tumble/physics/manifold/clip.h"
#include "tumble/physics/manifold/manifold.h"
#include "tumble/physics/collision/cast_shape_vs_shape.h"
#include "tumble/physics/collision/collide_point_vs_shape.h"
#include "tumble/physics/collision/collide_shape_vs_shape.h"
#include "tumble/physics/constraints/combine_material.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/utils/face.h"
#include "test.h"
#include "test_math.h"

static CastRayHit ray_hit(float fraction) {
    CastRayHit h = cast_ray::create_hit();
    h.status = CastRayStatus::COLLIDING;
    h.fraction = fraction;
    return h;
}

/** a custom collector that counts and chains to the one in its context, as a wrapper does */
static void counting_add_hit(CastRayCollector &c, const CastRayHit &h) {
    CastRayCollector &inner = *static_cast<CastRayCollector *>(c.context);
    cast_ray::add_hit(inner, h);
    c.early_out_fraction = inner.early_out_fraction;
}

void test_collectors(TestCtx &ctx) {

    // all: every hit in order, never early out
    CastRayCollector all = cast_ray::create_collector(CollectorKind::ALL);
    CHECK(ctx, all.early_out_fraction == collector::INITIAL_EARLY_OUT_FRACTION && !all.has_hit);
    cast_ray::add_hit(all, ray_hit(0.5f));
    cast_ray::add_hit(all, ray_hit(0.2f));
    CHECK(ctx, all.hits.length == 2 && all.hits[0].fraction == 0.5f && all.hits[1].fraction == 0.2f);
    CHECK(ctx, !cast_ray::should_early_out(all));
    CHECK(ctx, all.early_out_fraction == collector::INITIAL_EARLY_OUT_FRACTION);

    // any: the first hit and out
    CastRayCollector any = cast_ray::create_collector(CollectorKind::ANY);
    CHECK(ctx, !cast_ray::should_early_out(any) && cast_ray::hit(any) == nullptr);
    cast_ray::add_hit(any, ray_hit(0.7f));
    CHECK(ctx, cast_ray::should_early_out(any));
    CHECK(ctx, any.early_out_fraction == collector::SHOULD_EARLY_OUT_FRACTION);
    CHECK(ctx, cast_ray::hit(any)->fraction == 0.7f && any.hits.length == 1);

    // closest: the lowest fraction wins and tightens the early-out bound
    CastRayCollector closest = cast_ray::create_collector(CollectorKind::CLOSEST);
    cast_ray::add_hit(closest, ray_hit(0.6f));
    cast_ray::add_hit(closest, ray_hit(0.9f));
    cast_ray::add_hit(closest, ray_hit(0.3f));
    CHECK(ctx, closest.hits.length == 1 && cast_ray::hit(closest)->fraction == 0.3f);
    CHECK(ctx, closest.early_out_fraction == 0.3f);
    CHECK(ctx, !cast_ray::should_early_out(closest));
    cast_ray::reset(closest);
    CHECK(ctx, !closest.has_hit && closest.early_out_fraction == collector::INITIAL_EARLY_OUT_FRACTION && closest.hits.length == 0);

    // custom: chains to the closest one through its context
    CastRayCollector custom = cast_ray::create_collector(CollectorKind::CUSTOM);
    custom.custom_add_hit = counting_add_hit;
    custom.context = &closest;
    cast_ray::add_hit(custom, ray_hit(0.4f));
    cast_ray::add_hit(custom, ray_hit(0.8f));
    CHECK(ctx, cast_ray::hit(closest)->fraction == 0.4f && custom.early_out_fraction == 0.4f);
    cast_ray::add_miss(custom);

    // a collide collector's closest is the deepest penetration, and its bound starts unbounded
    CollideShapeCollector deepest = collide_shape::create_collector(CollectorKind::CLOSEST);
    CHECK(ctx, deepest.early_out_fraction == __FLT_MAX__);
    CollideShapeHit h = collide_shape::create_hit();
    h.penetration = 0.1f;
    h.face_a.num_vertices = 3;
    h.face_a.vertices[2] = vec3f::make(0.0f, 0.0f, 9.0f);
    collide_shape::add_hit(deepest, h);
    h.penetration = 0.05f;
    collide_shape::add_hit(deepest, h);
    h.penetration = 0.2f;
    collide_shape::add_hit(deepest, h);
    CHECK(ctx, deepest.hits.length == 1 && collide_shape::hit(deepest)->penetration == 0.2f);
    CHECK(ctx, collide_shape::hit(deepest)->face_a.num_vertices == 3 && vec3f::z(collide_shape::hit(deepest)->face_a.vertices[2]) == 9.0f);
    // the bound is minus the best penetration, so deeper hits still get through
    CHECK(ctx, deepest.early_out_fraction == -0.2f);

    CHECK(ctx, util::error() == Error::NONE);
}

void test_collectors_any_early_out(TestCtx &ctx) {
    dispatch::init();

    // the ray's any collector drops to 0; the others to -FLT_MAX, below any -penetration bound
    CollideShapeCollector any_collide = collide_shape::create_collector(CollectorKind::ANY);
    CollideShapeHit shallow = collide_shape::create_hit();
    shallow.penetration = 0.01f;
    collide_shape::add_hit(any_collide, shallow);
    CHECK(ctx, any_collide.early_out_fraction == -__FLT_MAX__ && collide_shape::should_early_out(any_collide));

    CollidePointCollector any_point = collide_point::create_collector(CollectorKind::ANY);
    collide_point::add_hit(any_point, collide_point::create_hit());
    CHECK(ctx, any_point.early_out_fraction == -__FLT_MAX__ && collide_point::should_early_out(any_point));

    CastShapeCollector any_cast = cast_shape::create_collector(CollectorKind::ANY);
    CHECK(ctx, !cast_shape::should_early_out(any_cast));
    CastShapeHit cast_hit = cast_shape::create_hit();
    cast_hit.fraction = 0.0f;
    cast_shape::add_hit(any_cast, cast_hit);
    CHECK(ctx, any_cast.early_out_fraction == -__FLT_MAX__ && cast_shape::should_early_out(any_cast));
    // a closest cast collector bounds a fraction-0 hit by its depth and keeps looking for deeper ones
    CastShapeCollector closest_cast = cast_shape::create_collector(CollectorKind::CLOSEST);
    cast_hit.penetration_depth = 0.2f;
    cast_shape::add_hit(closest_cast, cast_hit);
    CHECK(ctx, closest_cast.early_out_fraction == -0.2f && !cast_shape::should_early_out(closest_cast));
    cast_hit.penetration_depth = 0.1f;
    cast_shape::add_hit(closest_cast, cast_hit);
    CHECK(ctx, closest_cast.hits[0].penetration_depth == 0.2f);
    cast_hit.penetration_depth = 0.5f;
    cast_shape::add_hit(closest_cast, cast_hit);
    CHECK(ctx, closest_cast.hits[0].penetration_depth == 0.5f && closest_cast.early_out_fraction == -0.5f);
    cast_hit.fraction = 0.3f;
    cast_shape::add_hit(closest_cast, cast_hit);
    CHECK(ctx, closest_cast.hits[0].fraction == 0.0f);

    // two spheres 0.5 deep: under a 0 bound (-0.5 < 0) the deeper hit would replace the shallow one; -FLT_MAX prunes it
    Ref<Shape> a = sphere::create({.radius = 1.0f});
    Ref<Shape> b = sphere::create({.radius = 1.0f});
    const SubShapeIdBuilder no_sub_shape = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};
    const Transformf transform_a = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    const Transformf transform_b = {{1.5f, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    convex::collide_convex_vs_convex(any_collide, collide_shape::default_settings(), a, no_sub_shape, transform_a, b, no_sub_shape,
                                     transform_b);
    CHECK(ctx, any_collide.hits.length == 1 && collide_shape::hit(any_collide)->penetration == 0.01f);

    // the same pair into a fresh any collector does hit
    collide_shape::reset(any_collide);
    convex::collide_convex_vs_convex(any_collide, collide_shape::default_settings(), a, no_sub_shape, transform_a, b, no_sub_shape,
                                     transform_b);
    CHECK(ctx, any_collide.has_hit);
    CHECK_NEAR(ctx, collide_shape::hit(any_collide)->penetration, 0.5f, 1e-4f);

    a = nullptr;
    b = nullptr;
    CHECK(ctx, util::error() == Error::NONE);
}

void test_combine_material(TestCtx &ctx) {
    using namespace combine_material;
    CHECK_NEAR(ctx, combine(0.2f, 0.6f, MaterialCombineMode::AVERAGE, MaterialCombineMode::AVERAGE), 0.4f, 1e-6f);
    CHECK_NEAR(ctx, combine(0.5f, 0.5f, MaterialCombineMode::MULTIPLY, MaterialCombineMode::MULTIPLY), 0.25f, 1e-6f);
    CHECK_NEAR(ctx, combine(0.2f, 0.6f, MaterialCombineMode::MIN, MaterialCombineMode::MIN), 0.2f, 1e-6f);
    CHECK_NEAR(ctx, combine(0.2f, 0.6f, MaterialCombineMode::MAX, MaterialCombineMode::MAX), 0.6f, 1e-6f);
    CHECK_NEAR(ctx, combine(4.0f, 9.0f, MaterialCombineMode::GEOMETRIC_MEAN, MaterialCombineMode::GEOMETRIC_MEAN), 6.0f, 1e-6f);
    // the more restrictive mode wins when they differ: max > min > geometric mean > multiply > average
    CHECK_NEAR(ctx, combine(0.2f, 0.6f, MaterialCombineMode::AVERAGE, MaterialCombineMode::MAX), 0.6f, 1e-6f);
    CHECK_NEAR(ctx, combine(0.2f, 0.6f, MaterialCombineMode::MIN, MaterialCombineMode::MULTIPLY), 0.2f, 1e-6f);
    CHECK_NEAR(ctx, combine(4.0f, 9.0f, MaterialCombineMode::MULTIPLY, MaterialCombineMode::GEOMETRIC_MEAN), 6.0f, 1e-6f);
}

void test_face(TestCtx &ctx) {
    SupportingFace f;
    face::clear(f);
    f.num_vertices = 2;
    f.vertices[0] = vec3f::make(1.0f, 0.0f, 0.0f);
    f.vertices[1] = vec3f::make(0.0f, 1.0f, 0.0f);
    // a quarter turn about z then a move: (1,0,0) -> (0,1,0) + t, (0,1,0) -> (-1,0,0) + t
    Float4x4 m;
    mat4f::store(m, mat4f::identity());
    m.m[0] = 0.0f;
    m.m[1] = 1.0f;
    m.m[4] = -1.0f;
    m.m[5] = 0.0f;
    m.m[12] = 10.0f;
    m.m[13] = 20.0f;
    m.m[14] = 30.0f;
    face::transform_with_mat4_rotation_translation(f, mat4f::load(m));
    CHECK_NEAR(ctx, vec3f::x(f.vertices[0]), 10.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(f.vertices[0]), 21.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(f.vertices[0]), 30.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::x(f.vertices[1]), 9.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(f.vertices[1]), 20.0f, 1e-6f);
    // with a scale applied before the rotation
    face::clear(f);
    f.num_vertices = 1;
    f.vertices[0] = vec3f::make(1.0f, 1.0f, 1.0f);
    face::transform_with_mat4_scale(f, mat4f::load(m), vec3f::make(2.0f, 3.0f, 4.0f));
    CHECK_NEAR(ctx, vec3f::x(f.vertices[0]), 10.0f - 3.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(f.vertices[0]), 20.0f + 2.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(f.vertices[0]), 30.0f + 4.0f, 1e-6f);
    CHECK(ctx, face::is_scale_inside_out(vec3f::make(1.0f, -1.0f, 1.0f)) && !face::is_scale_inside_out(vec3f::make(-1.0f, -1.0f, 1.0f)));
    SupportingFace g;
    face::copy(g, f);
    CHECK(ctx, g.num_vertices == 1 && vec3f::z(g.vertices[0]) == vec3f::z(f.vertices[0]));
}

/** a stand-in shape type for the table tests: a leaf with the defaults */
static float inner_radius_two(const Shape *) { return 2.0f; }
static void no_surface_normal(SurfaceNormalResult &, const Shape *, SubShapeId) {}
static int32_t no_material(const Shape *, SubShapeId) { return -1; }
static void no_supporting_face(SupportingFaceResult &io, Vec3f, const Shape *, SubShapeId) { io.face.num_vertices = 0; }
static void no_cast_ray(CastRayCollector &, const CastRaySettings &, const Rayf &, const Shape *,
                        const SubShapeIdBuilder &, const Transformf &) {}
static void no_collide_point(CollidePointCollector &, const CollidePointSettings &, Vec3f,
                             const Shape *, const SubShapeIdBuilder &, const Transformf &) {}
static void collide_marker(CollideShapeCollector &c, const CollideShapeSettings &, const Shape *,
                           const SubShapeIdBuilder &, const Transformf &, const Shape *, const SubShapeIdBuilder &,
                           const Transformf &) {
    c.early_out_fraction = 42.0f;
}

void test_shape_tables(TestCtx &ctx) {
    using namespace shapes;
    CHECK(ctx, shape_type_index(ShapeType::SPHERE) == 0 && shape_type_index(ShapeType::STATIC_COMPOUND) == 12);
    // define_shape fills what a def leaves null
    ShapeDef def = {};
    def.type = ShapeType::EMPTY;
    def.get_inner_radius = inner_radius_two;
    def.get_surface_normal = no_surface_normal;
    def.get_supporting_face = no_supporting_face;
    def.get_material = no_material;
    def.cast_ray = no_cast_ray;
    def.collide_point = no_collide_point;
    set_shape_def(define_shape(def));
    Shape shape = {};
    shape.type = ShapeType::EMPTY;
    CHECK(ctx, !is_convex(&shape));
    CHECK(ctx, get_shape_inner_radius(&shape) == 2.0f);
    MassProperties mp = mass_properties::create();
    compute_mass_properties(mp, &shape);
    CHECK(ctx, mp.mass == 1.0f && mp.inertia.m[0] == 1.0f);
    GetLeafShapeResult leaf;
    get_leaf_shape(leaf, &shape, 1234);
    CHECK(ctx, leaf.shape == &shape && leaf.remainder == 1234);
    // a pair's entry replaces an earlier one, and the reverse pair is its own entry
    set_collide_shape_fn(ShapeType::EMPTY, ShapeType::PLANE, collide_marker);
    CHECK(ctx, collide_fn(ShapeType::EMPTY, ShapeType::PLANE) == collide_marker);
    CHECK(ctx, collide_fn(ShapeType::PLANE, ShapeType::EMPTY) == nullptr);
    CHECK(ctx, cast_fn(ShapeType::EMPTY, ShapeType::PLANE) == nullptr);
}

void test_world_tier_types(TestCtx &ctx) {
    // the w1 headers: a manifold starts empty and relative to nothing, its world points add the base offset
    ContactManifold m = manifold::create();
    CHECK(ctx, m.num_contact_points == 0 && m.penetration_depth == 0.0f);
    CHECK(ctx, m.sub_shape_id_a == sub_shape::EMPTY_SUB_SHAPE_ID && m.sub_shape_id_b == sub_shape::EMPTY_SUB_SHAPE_ID);
    m.base_offset = vec3f::make(10.0f, 0.0f, 0.0f);
    m.relative_contact_points_on_b[3] = vec3f::make(1.0f, 2.0f, 3.0f);
    const Vec3f world = manifold::get_world_space_contact_point_on_b(m, 3);
    CHECK(ctx, vec3f::exact_equals(world, vec3f::make(11.0f, 2.0f, 3.0f)));
    CHECK(ctx, manifold::MAX_CONTACT_POINTS == 4 && manifold::MAX_CLIPPING_CONTACT_POINTS == face::FACE_MAX_VERTICES);
    manifold::reset(m);
    CHECK(ctx, vec3f::x(m.base_offset) == 0.0f && m.num_contact_points == 0);

    const RayIntersectsTriangleResult tri = cast_utils::create_result();
    CHECK(ctx, !tri.hit && tri.fraction == 0.0f && !tri.front_facing);

    const CollisionEstimationResult estimate = estimate_collision_response::create_result();
    CHECK(ctx, estimate.num_impulses == 0 && estimate.contact_impulse[63] == 0.0f);

    // the edge-removing wrapper is a custom collide collector pointing back at itself once set up (cpp); the record is plain
    InternalEdgeRemovingCollector wrapper = {};
    CHECK(ctx, wrapper.chained == nullptr && wrapper.delayed_results.length == 0 && wrapper.voided_features.length == 0);

    // pair edge keys pack a record and a side
    CHECK(ctx, pairs::pair_edge_key(5, 1) == 11 && pairs::get_pair_edge_record(11) == 5 && pairs::get_pair_edge_side(11) == 1);
    CHECK(ctx, pairs::get_pair_edge_side(pairs::pair_edge_key(5, 0)) == 0);

    // the manifold cache double buffers: the write side is the other one, and a flip swaps them
    Contacts cs = {};
    Contacts::Contact c = {};
    c.manifolds[1].num_contact_points = 3;
    CHECK(ctx, &contacts::get_read_manifold(c, cs) == &c.manifolds[0] && contacts::get_write_manifold(c, cs).num_contact_points == 3);
    contacts::flip_manifold_cache(cs);
    CHECK(ctx, &contacts::get_read_manifold(c, cs) == &c.manifolds[1]);

    // change_contact marks the event changed and hands back its own settings, the combined material it was built from
    ContactEvent contact = {};
    contact.settings.combined_friction = 0.7f;
    ContactSettings &changed = events::change_contact(contact);
    CHECK(ctx, contact.settings_changed && !contact.rejected && &changed == &contact.settings && changed.combined_friction == 0.7f);
    const ContactSettings defaults = ContactSettings();
    CHECK(ctx, defaults.inv_mass_scale_2 == 1.0f && defaults.combined_friction == 0.0f && !defaults.is_sensor);

    // world settings carry the defaults; the layers are the only part that needs releasing
    WorldSettings settings = world_settings::create();
    CHECK(ctx, settings.gravity.y == -9.81f && settings.gravity_enabled && settings.solver.velocity_iterations == 10);
    CHECK(ctx, settings.narrowphase.speculative_contact_distance == 0.02f && !settings.narrowphase.collide_with_backfaces);
    CHECK_NEAR(ctx, settings.narrowphase.normal_cos_max_delta_rotation, scalar::cos(5.0f * scalar::DEGREES_TO_RADIANS), 1e-6f);
    CHECK(ctx, settings.sleeping.time_before_sleep == 0.5f && settings.ccd.linear_cast_threshold == 0.05f);
    const uint32_t bp = world_settings::add_broadphase_layer(settings);
    const uint32_t moving = world_settings::add_object_layer(settings, bp);
    world_settings::enable_collision(settings, moving, moving);
    CHECK(ctx, layers::object_layer_collides_with_object_layer(settings.layers, moving, moving));

    // a constraint pool addresses its records by stride; every record starts with the base
    ConstraintBase base = constraints::make_constraint_base();
    CHECK(ctx, base.pooled && base.enabled && base.id == constraint_id::INVALID_CONSTRAINT_ID && base.num_velocity_steps_override == 0);
    const MotorSettings motor = motor_settings::create();
    CHECK(ctx, motor.spring.frequency_or_stiffness == 2.0f && motor.spring.damping == 1.0f && motor.max_torque_limit == __FLT_MAX__);
    CHECK(ctx, spring_settings::create().frequency_or_stiffness == 0.0f);
    AxisConstraintPart axis = axis_constraint_part::create();
    CHECK(ctx, !axis_constraint_part::is_active(axis));
    axis.effective_mass = 2.0f;
    CHECK(ctx, axis_constraint_part::is_active(axis));
    axis_constraint_part::deactivate(axis);
    CHECK(ctx, !axis_constraint_part::is_active(axis));
    CHECK(ctx, sizeof(ContactConstraints::Constraint) < 1024);
    CCD sweeps = ccd::create();
    CHECK(ctx, sweeps.ccd_bodies.length == 0);
}
