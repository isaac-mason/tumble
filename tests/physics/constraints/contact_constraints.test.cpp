#include "tumble/physics/constraints/contact_constraints.h"

#include "../contact_test_scene.h"
#include "tumble/math/scalar.h"
#include "tumble/physics/collision/narrowphase.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"
#include "test_math.h"

using rigid_body::RigidBody;

namespace {

constexpr float SPHERE_RADIUS = 0.5f;

/** a 10 x 1 x 10 static slab with its top face at y = 0 */
RigidBody *add_ground(ContactScene &scene, const ContactBodySettings &settings = {}) {
    return add_contact_body(scene, box::create({.half_extents = {5.0f, 0.5f, 5.0f}}), motion_type::MOTION_STATIC, {0.0f, -0.5f, 0.0f}, settings);
}

/** a sphere resting on y = 0, touching at the origin */
RigidBody *add_ball(ContactScene &scene, const ContactBodySettings &settings = {}, motion_type::MotionType motion_type = motion_type::MOTION_DYNAMIC) {
    return add_contact_body(scene, sphere::create({.radius = SPHERE_RADIUS}), motion_type, {0.0f, SPHERE_RADIUS, 0.0f}, settings);
}

manifold::ContactManifold ball_manifold() {
    const Float3 point = {0.0f, 0.0f, 0.0f};
    return touching_manifold(&point, 1, {0.0f, 1.0f, 0.0f});
}

float ball_mass(const RigidBody *ball) { return 1.0f / ball->motion_properties.inv_mass; }

/** the velocity of the ball's surface at its contact point (centre of mass - r up) */
Float3 ball_contact_velocity(const RigidBody *ball) {
    Float3 out;
    return vec3f::store(out, rigid_body::get_velocity_at_point_com(ball, vec3f::make(0.0f, -SPHERE_RADIUS, 0.0f)));
}

float sum_normal_lambdas(const contact_constraints::ContactConstraint &constraint) {
    float sum = 0.0f;
    for (uint32_t i = 0; i < constraint.num_contact_points; i++) sum += constraint.contact_points[i].normal_constraint.total_lambda;
    return sum;
}

/** a unit box on the ground, its bottom face's corners touching y = 0 */
struct BoxOnGround {
    RigidBody *ground;
    RigidBody *box;
    manifold::ContactManifold manifold;
};

BoxOnGround make_box_on_ground(ContactScene &scene) {
    BoxOnGround out;
    out.ground = add_ground(scene);
    out.box = add_contact_body(scene, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}), motion_type::MOTION_DYNAMIC, {0.0f, 0.5f, 0.0f});
    const Float3 corners[4] = {{-0.5f, 0.0f, -0.5f}, {0.5f, 0.0f, -0.5f}, {0.5f, 0.0f, 0.5f}, {-0.5f, 0.0f, 0.5f}};
    out.manifold = touching_manifold(corners, 4, {0.0f, 1.0f, 0.0f});
    return out;
}

contact_constraints::ContactSettings default_override() {
    contact_constraints::ContactSettings settings = contact_constraints::ContactSettings();
    return settings;
}

/** answers sized to the events, the first contact event answered with `settings` */
void answer_first(ContactScene &scene, const contact_constraints::ContactSettings &settings, bool reject = false) {
    events::reset_contact_answers(scene.answers, scene.events, 0);
    events::ContactOverride &answer = scene.answers.contact_override[0];
    answer.written = true;
    answer.reject = reject;
    answer.settings = settings;
}

/** 64 bit fnv-1a over the little endian bytes of six u32 words, a byte at a time */
uint64_t fnv1a_64(const uint32_t (&words)[4]) {
    uint64_t hash = 0xcbf29ce484222325ull;
    for (uint32_t word : words) {
        for (uint32_t byte = 0; byte < 4; byte++) {
            hash ^= (word >> (byte * 8)) & 0xffu;
            hash *= 0x100000001b3ull;
        }
    }
    return hash;
}

void apply_answers(ContactScene &scene) {
    contact_constraints::apply_contact_overrides(scene.contact_constraints, scene.contacts, scene.bodies, scene.events, scene.answers,
                                                 scene.settings, CONTACT_SCENE_DELTA_TIME);
}

}  // namespace

void test_contact_constraints_box_resting_removes_approach_and_warm_starts(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    BoxOnGround scenario = make_box_on_ground(scene);
    RigidBody *box = scenario.box;
    const float mass = 1.0f / box->motion_properties.inv_mass;
    CHECK_NEAR(ctx, mass, 1000.0f, 1e-2f);

    // step 1: the box comes down at 1 m/s onto the ground
    begin_contact_step(scene);
    box->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    CHECK(ctx, add_contact(scene, scenario.ground, box, scenario.manifold));
    CHECK(ctx, scene.contact_constraints.count == 1);
    CHECK(ctx, scene.events.contacts_added.length == 1 && scene.events.contacts_persisted.length == 0);

    // the ground was made first, so it is a and the normal points from it to the box
    const contact_constraints::ContactConstraint &constraint = scene.contact_constraints.pool[0];
    CHECK(ctx, constraint.body_index_a == scenario.ground->index && constraint.body_index_b == box->index);
    CHECK(ctx, constraint.num_contact_points == 4);
    CHECK_NEAR(ctx, vec3f::dot(vec3f::load(constraint.tangent_1), vec3f::load(constraint.normal)), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::dot(vec3f::load(constraint.tangent_2), vec3f::load(constraint.normal)), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::dot(vec3f::load(constraint.tangent_1), vec3f::load(constraint.tangent_2)), 0.0f, 1e-6f);

    solve_contact_velocities(scene, 10);

    // the approach is gone, the box neither bounces nor turns, and the impulse is the momentum it had
    CHECK_NEAR(ctx, box->motion_properties.linear_velocity.y, 0.0f, 1e-3f);
    CHECK_NEAR(ctx, vec3f::length(vec3f::load(box->motion_properties.angular_velocity)), 0.0f, 1e-3f);
    CHECK_NEAR(ctx, sum_normal_lambdas(constraint), mass * 1.0f, 1.0f);
    // four coplanar points over-determine the three dofs, so the split between them is the solver's, but it carries
    // no moment about the box's centre
    float moment_x = 0.0f;
    float moment_z = 0.0f;
    for (uint32_t i = 0; i < 4; i++) {
        const contact_constraints::WorldContactPoint &point = constraint.contact_points[i];
        CHECK(ctx, point.normal_constraint.total_lambda >= 0.0f);
        moment_x += point.normal_constraint.total_lambda * point.position_b.z;
        moment_z += point.normal_constraint.total_lambda * point.position_b.x;
    }
    CHECK_NEAR(ctx, moment_x, 0.0f, 1.0f);
    CHECK_NEAR(ctx, moment_z, 0.0f, 1.0f);
    const float lambda_step_1 = sum_normal_lambdas(constraint);
    end_contact_step(scene);

    // step 2: the same approach again; the stored impulses carry over and warm start alone stops the box
    begin_contact_step(scene);
    box->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    box->motion_properties.angular_velocity = {0.0f, 0.0f, 0.0f};
    CHECK(ctx, add_contact(scene, scenario.ground, box, scenario.manifold));
    CHECK(ctx, scene.events.contacts_added.length == 0 && scene.events.contacts_persisted.length == 1);
    CHECK_NEAR(ctx, sum_normal_lambdas(scene.contact_constraints.pool[0]), lambda_step_1, 1e-3f);

    contact_constraints::warm_start_velocity_constraints(scene.contact_constraints, scene.bodies, all_contact_indices(scene), 1.0f);
    CHECK_NEAR(ctx, box->motion_properties.linear_velocity.y, 0.0f, 1e-3f);
    contact_constraints::solve_velocity_constraints_for_island(scene.contact_constraints, scene.bodies, all_contact_indices(scene));
    CHECK_NEAR(ctx, box->motion_properties.linear_velocity.y, 0.0f, 1e-3f);
    CHECK_NEAR(ctx, vec3f::length(vec3f::load(box->motion_properties.angular_velocity)), 0.0f, 1e-3f);
    CHECK_NEAR(ctx, sum_normal_lambdas(scene.contact_constraints.pool[0]), lambda_step_1, 1.0f);

    release_contact_scene(scene);
}

void test_contact_constraints_event_carries_the_manifold(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    BoxOnGround scenario = make_box_on_ground(scene);
    scenario.manifold.penetration_depth = 0.01f;
    scenario.manifold.material_id_a = 3;
    scenario.manifold.material_id_b = 7;

    // handed over in b, a order: the event still puts the lower id first, with the manifold swapped to match
    begin_contact_step(scene);
    manifold::ContactManifold reversed = scenario.manifold;
    manifold::swap_shapes(reversed);
    CHECK(ctx, add_contact(scene, scenario.box, scenario.ground, reversed));

    CHECK(ctx, scene.events.contacts_added.length == 1);
    const events::ContactEvent &event = scene.events.contacts_added[0];
    CHECK(ctx, event.body_id_a == scenario.ground->id && event.body_id_b == scenario.box->id);
    CHECK(ctx, event.body_index_a == scenario.ground->index && event.body_index_b == scenario.box->index);
    CHECK(ctx, event.material_id_a == 3 && event.material_id_b == 7);
    CHECK_NEAR(ctx, event.normal.y, 1.0f, 1e-6f);
    CHECK_NEAR(ctx, event.penetration, 0.01f, 1e-7f);
    CHECK(ctx, event.num_points == 4);
    for (uint32_t i = 0; i < 4; i++) {
        CHECK(ctx, vec3f::exact_equals(vec3f::load(event.points_a[i]), scenario.manifold.relative_contact_points_on_a[i]));
        CHECK(ctx, vec3f::exact_equals(vec3f::load(event.points_b[i]), scenario.manifold.relative_contact_points_on_b[i]));
    }
    CHECK(ctx, event.contact_index == scene.contact_constraints.pool[0].contact_index);

    release_contact_scene(scene);
}

void test_contact_constraints_restitution_bounces_less_gravity(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *ground = add_ground(scene);
    RigidBody *ball = add_ball(scene, {.restitution = 0.5f});

    // restitution combines by max: 0.5. the bounce takes back the velocity gravity added this step
    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {0.0f, -5.0f, 0.0f};
    CHECK(ctx, add_contact(scene, ground, ball, ball_manifold()));
    CHECK_NEAR(ctx, scene.contact_constraints.pool[0].restitution, 0.5f, 1e-7f);
    solve_contact_velocities(scene, 10);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.y, 0.5f * (5.0f - 9.81f * CONTACT_SCENE_DELTA_TIME), 1e-4f);
    CHECK_NEAR(ctx, scene.contact_constraints.pool[0].contact_points[0].normal_constraint.total_lambda,
               ball_mass(ball) * (5.0f + 0.5f * (5.0f - 9.81f * CONTACT_SCENE_DELTA_TIME)), 0.1f);

    // below min_velocity_for_restitution (1 m/s) there is no bounce
    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {0.0f, -0.5f, 0.0f};
    CHECK(ctx, add_contact(scene, ground, ball, ball_manifold()));
    solve_contact_velocities(scene, 10);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.y, 0.0f, 1e-5f);

    release_contact_scene(scene);
}

void test_contact_constraints_speculative_contact_allows_closing_the_gap(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *ground = add_ground(scene);
    RigidBody *ball = add_ball(scene);

    // 1 cm apart: the ball may close the gap this step (0.01 / dt = 0.6 m/s) but no faster
    manifold::ContactManifold gap = manifold::create();
    gap.num_contact_points = 1;
    gap.world_space_normal = vec3f::make(0.0f, 1.0f, 0.0f);
    gap.relative_contact_points_on_a[0] = vec3f::make(0.0f, 0.0f, 0.0f);
    gap.relative_contact_points_on_b[0] = vec3f::make(0.0f, 0.01f, 0.0f);

    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {0.0f, -0.4f, 0.0f};
    CHECK(ctx, add_contact(scene, ground, ball, gap));
    solve_contact_velocities(scene, 10);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.y, -0.4f, 1e-6f);

    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {0.0f, -2.0f, 0.0f};
    CHECK(ctx, add_contact(scene, ground, ball, gap));
    solve_contact_velocities(scene, 10);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.y, -0.01f / CONTACT_SCENE_DELTA_TIME, 1e-4f);

    release_contact_scene(scene);
}

void test_contact_constraints_friction_slides_at_the_cone(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *ground = add_ground(scene, {.friction = 0.5f});
    RigidBody *ball = add_ball(scene, {.friction = 0.5f});

    // mu = sqrt(0.5 * 0.5). stopping the slip would take 2 m/s / (1/m + r^2/I) = 4/7 m, more than mu * (normal impulse
    // m * 1 m/s), so the ball slides and friction takes exactly mu * 1 m/s off its centre
    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {2.0f, -1.0f, 0.0f};
    CHECK(ctx, add_contact(scene, ground, ball, ball_manifold()));
    CHECK_NEAR(ctx, scene.contact_constraints.pool[0].friction, 0.5f, 1e-6f);
    solve_contact_velocities(scene, 10);

    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.y, 0.0f, 1e-5f);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.x, 1.5f, 1e-4f);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.z, 0.0f, 1e-6f);
    // the friction torque spins it up: dw = (r x J) / I, r = (0, -0.5, 0), J = (-0.5 m, 0, 0), I = 2/5 m r^2
    CHECK_NEAR(ctx, ball->motion_properties.angular_velocity.z, -(0.5f * 0.5f) / (0.4f * 0.25f), 1e-3f);

    release_contact_scene(scene);
}

void test_contact_constraints_friction_sticks_inside_the_cone(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *ground = add_ground(scene, {.friction = 1.0f});
    RigidBody *ball = add_ball(scene, {.friction = 1.0f});

    // mu = 1 covers the 4/7 m it takes, so the contact point stops: v = 2 - 4/7 and the ball rolls
    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {2.0f, -1.0f, 0.0f};
    CHECK(ctx, add_contact(scene, ground, ball, ball_manifold()));
    solve_contact_velocities(scene, 10);

    CHECK_NEAR(ctx, vec3f::length(vec3f::load(ball_contact_velocity(ball))), 0.0f, 1e-4f);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.x, 10.0f / 7.0f, 1e-4f);

    release_contact_scene(scene);
}

void test_contact_constraints_sensor_reports_without_impulse(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *ground = add_ground(scene);
    RigidBody *ball = add_ball(scene, {.sensor = true});

    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    CHECK(ctx, !add_contact(scene, ground, ball, ball_manifold()));
    CHECK(ctx, scene.contact_constraints.count == 0);
    CHECK(ctx, scene.events.contacts_added.length == 1);
    CHECK(ctx, scene.events.contacts_added[0].body_id_b == ball->id);
    solve_contact_velocities(scene, 10);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.y, -1.0f, 0.0f);

    // the contact is cached with no impulse, so the next step reports it persisted
    contacts::Contact &contact = scene.contacts.contacts[(uint32_t)scene.events.contacts_added[0].contact_index];
    const contacts::CachedManifold &write = contacts::get_write_manifold(contact, scene.contacts);
    CHECK(ctx, write.num_contact_points == 1 && write.contact_points[0].normal_lambda == 0.0f);
    end_contact_step(scene);
    begin_contact_step(scene);
    CHECK(ctx, !add_contact(scene, ground, ball, ball_manifold()));
    CHECK(ctx, scene.events.contacts_added.length == 0 && scene.events.contacts_persisted.length == 1);

    release_contact_scene(scene);
}

void test_contact_constraints_kinematic_pushes_dynamic(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *platform =
        add_contact_body(scene, box::create({.half_extents = {5.0f, 0.5f, 5.0f}}), motion_type::MOTION_KINEMATIC, {0.0f, -0.5f, 0.0f});
    RigidBody *ball = add_ball(scene);

    // a kinematic and a static pair makes no constraint; a kinematic rising under a ball carries it up
    begin_contact_step(scene);
    platform->motion_properties.linear_velocity = {0.0f, 1.0f, 0.0f};
    CHECK(ctx, add_contact(scene, platform, ball, ball_manifold()));
    solve_contact_velocities(scene, 10);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.y, 1.0f, 1e-5f);
    CHECK_NEAR(ctx, platform->motion_properties.linear_velocity.y, 1.0f, 0.0f);

    RigidBody *wall = add_contact_body(scene, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}), motion_type::MOTION_STATIC, {0.0f, 0.0f, 4.0f});
    begin_contact_step(scene);
    CHECK(ctx, !add_contact(scene, platform, wall, ball_manifold()));
    CHECK(ctx, scene.contact_constraints.count == 0 && scene.events.contacts_added.length == 1);

    release_contact_scene(scene);
}

void test_contact_constraints_position_solve_pushes_out_by_baumgarte(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *ground = add_ground(scene);
    RigidBody *ball = add_ball(scene);

    // the ball 10 cm into the ground: one position iteration corrects baumgarte * (depth - slop)
    manifold::ContactManifold sunk = manifold::create();
    sunk.num_contact_points = 1;
    sunk.world_space_normal = vec3f::make(0.0f, 1.0f, 0.0f);
    sunk.penetration_depth = 0.1f;
    sunk.relative_contact_points_on_a[0] = vec3f::make(0.0f, 0.0f, 0.0f);
    sunk.relative_contact_points_on_b[0] = vec3f::make(0.0f, -0.1f, 0.0f);

    begin_contact_step(scene);
    CHECK(ctx, add_contact(scene, ground, ball, sunk));
    const float before = ball->center_of_mass_position.y;
    const world_settings::SolverSettings &solver = scene.settings.solver;
    CHECK(ctx, contact_constraints::solve_position_constraints_for_island(scene.contact_constraints, scene.bodies, all_contact_indices(scene),
                                                                          solver.penetration_slop, solver.baumgarte_factor,
                                                                          solver.max_penetration_distance));
    CHECK_NEAR(ctx, ball->center_of_mass_position.y - before, solver.baumgarte_factor * (0.1f - solver.penetration_slop), 1e-5f);

    // within the slop nothing moves
    begin_contact_step(scene);
    sunk.relative_contact_points_on_b[0] = vec3f::make(0.0f, -0.01f, 0.0f);
    ball->center_of_mass_position.y = SPHERE_RADIUS;
    CHECK(ctx, add_contact(scene, ground, ball, sunk));
    CHECK(ctx, !contact_constraints::solve_position_constraints_for_island(scene.contact_constraints, scene.bodies,
                                                                           all_contact_indices(scene), solver.penetration_slop,
                                                                           solver.baumgarte_factor, solver.max_penetration_distance));

    release_contact_scene(scene);
}

void test_contact_constraints_from_cache_carries_the_manifold(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    BoxOnGround scenario = make_box_on_ground(scene);
    RigidBody *box = scenario.box;

    begin_contact_step(scene);
    box->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    CHECK(ctx, add_contact(scene, scenario.ground, box, scenario.manifold));
    solve_contact_velocities(scene, 10);
    const float lambda_step_1 = sum_normal_lambdas(scene.contact_constraints.pool[0]);
    const int32_t contact_index = scene.contact_constraints.pool[0].contact_index;
    end_contact_step(scene);

    // the pair barely moved: the cached manifold builds the constraint, warm started, with no narrowphase
    begin_contact_step(scene);
    box->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    box->motion_properties.angular_velocity = {0.0f, 0.0f, 0.0f};
    contacts::Contact &contact = scene.contacts.contacts[(uint32_t)contact_index];
    CHECK(ctx, contact_constraints::add_contact_constraint_from_cache(scene.contact_constraints, scene.contacts, *scenario.ground, *box,
                                                                      contact, scene.settings, scene.events, CONTACT_SCENE_DELTA_TIME,
                                                                      scene.bodies.step_stamp));
    CHECK(ctx, contact.last_processed_frame == scene.contacts.frame_stamp);
    CHECK(ctx, scene.events.contacts_persisted.length == 1 && scene.events.contacts_added.length == 0);
    const events::ContactEvent &event = scene.events.contacts_persisted[0];
    CHECK(ctx, event.num_points == 4 && event.material_id_a == -1 && event.material_id_b == -1);
    CHECK_NEAR(ctx, event.normal.y, 1.0f, 1e-6f);
    CHECK_NEAR(ctx, event.penetration, 0.0f, 1e-6f);
    for (uint32_t i = 0; i < 4; i++) {
        CHECK_NEAR(ctx, vec3f::distance(vec3f::load(event.points_a[i]), scenario.manifold.relative_contact_points_on_a[i]), 0.0f, 1e-6f);
    }
    CHECK_NEAR(ctx, sum_normal_lambdas(scene.contact_constraints.pool[0]), lambda_step_1, 1e-3f);

    contact_constraints::warm_start_velocity_constraints(scene.contact_constraints, scene.bodies, all_contact_indices(scene), 1.0f);
    CHECK_NEAR(ctx, box->motion_properties.linear_velocity.y, 0.0f, 1e-3f);

    release_contact_scene(scene);
}

void test_contact_constraints_from_cache_keeps_the_materials(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);

    // a quad of two triangles with their own materials; the first (0, 2, 1) covers x > z
    const float positions[12] = {-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10};
    const uint32_t indices[6] = {0, 2, 1, 0, 3, 2};
    const int32_t materials[2] = {5, 9};
    triangle_mesh_data::TriangleMeshData *data = triangle_mesh_builder::build(
        {.positions = positions, .num_positions = 4, .indices = indices, .num_indices = 6, .material_ids = materials});
    RigidBody *floor = add_contact_body(scene, triangle_mesh::create({.data = data}), motion_type::MOTION_STATIC, {0, 0, 0}, {.friction = 0.8f});

    // resting in the middle of the first triangle, away from the shared diagonal
    RigidBody *ball = add_contact_body(scene, sphere::create({.radius = SPHERE_RADIUS, .material_id = 3}), motion_type::MOTION_DYNAMIC,
                                       {5.0f, SPHERE_RADIUS - 0.01f, -3.0f}, {.friction = 0.2f});

    // the narrowphase's manifold: the ball as a against the floor, from the collision's one hit
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_ALL);
    const sub_shape::SubShapeIdBuilder no_sub_shape = {sub_shape::EMPTY_SUB_SHAPE_ID, 0};
    const Transformf ball_pose = {ball->position, ball->quaternion, {1, 1, 1}};
    const Transformf floor_pose = {floor->position, floor->quaternion, {1, 1, 1}};
    narrowphase::collide_shape_vs_shape(collector, collide_shape::default_settings(), ball->shape, no_sub_shape, ball_pose, floor->shape,
                                        no_sub_shape, floor_pose);
    CHECK(ctx, collector.hits.length == 1);
    if (collector.hits.length != 1) {
        collide_shape::release(collector);
        release_contact_scene(scene);
        return;
    }
    const collide_shape::CollideShapeHit &hit = collector.hits[0];
    CHECK(ctx, hit.material_id_a == 3 && hit.material_id_b == 5);
    Float3 hit_point_a;
    vec3f::store(hit_point_a, hit.point_a);
    manifold::ContactManifold contact_manifold = touching_manifold(&hit_point_a, 1, {0.0f, -1.0f, 0.0f}, hit.penetration);
    contact_manifold.relative_contact_points_on_b[0] = hit.point_b;
    contact_manifold.world_space_normal = vec3f::normalize(hit.penetration_axis);
    contact_manifold.sub_shape_id_a = hit.sub_shape_id_a;
    contact_manifold.sub_shape_id_b = hit.sub_shape_id_b;
    contact_manifold.material_id_a = hit.material_id_a;
    contact_manifold.material_id_b = hit.material_id_b;
    collide_shape::release(collector);

    begin_contact_step(scene);
    CHECK(ctx, add_contact(scene, ball, floor, contact_manifold));
    CHECK(ctx, scene.events.contacts_added.length == 1 && scene.contact_constraints.count == 1);
    const events::ContactEvent added = scene.events.contacts_added[0];
    const float added_friction = scene.contact_constraints.pool[0].friction;
    // body a is the floor (the lower id)
    CHECK(ctx, added.body_id_a == floor->id && added.material_id_a == 5 && added.material_id_b == 3);
    CHECK_NEAR(ctx, added.settings.combined_friction, scalar::sqrt(0.8f * 0.2f), 1e-6f);
    const int32_t contact_index = scene.contact_constraints.pool[0].contact_index;
    end_contact_step(scene);

    // persisted through the cache, which keeps no materials: the same materials and friction as when first built
    for (uint32_t step = 0; step < 3; step++) {
        begin_contact_step(scene);
        contacts::Contact &contact = scene.contacts.contacts[(uint32_t)contact_index];
        CHECK(ctx, contact_constraints::add_contact_constraint_from_cache(scene.contact_constraints, scene.contacts, *floor, *ball, contact,
                                                                          scene.settings, scene.events, CONTACT_SCENE_DELTA_TIME,
                                                                          scene.bodies.step_stamp));
        CHECK(ctx, scene.events.contacts_persisted.length == 1 && scene.contact_constraints.count == 1);
        const events::ContactEvent &persisted = scene.events.contacts_persisted[0];
        CHECK(ctx, persisted.sub_shape_id_a == added.sub_shape_id_a && persisted.sub_shape_id_b == added.sub_shape_id_b);
        CHECK(ctx, persisted.material_id_a == added.material_id_a && persisted.material_id_b == added.material_id_b);
        CHECK(ctx, persisted.settings.combined_friction == added.settings.combined_friction);
        CHECK(ctx, scene.contact_constraints.pool[0].friction == added_friction);
        end_contact_step(scene);
    }

    release_contact_scene(scene);
}

void test_contact_constraints_override_conveyor(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *belt = add_ground(scene);
    RigidBody *ball = add_ball(scene);

    // frictionless as built, so the parts are off; js turns friction on and moves the belt's surface at 2 m/s along +x.
    // the relative surface velocity is body 2's surface relative to body 1's, so the belt (a) moving gives -2
    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    CHECK(ctx, add_contact(scene, belt, ball, ball_manifold()));
    CHECK(ctx, !contact_constraint_part::is_active(scene.contact_constraints.pool[0].friction_constraint_1));

    contact_constraints::ContactSettings settings = default_override();
    settings.combined_friction = 1.0f;
    settings.relative_linear_surface_velocity = {-2.0f, 0.0f, 0.0f};
    answer_first(scene, settings);
    apply_answers(scene);
    CHECK(ctx, scene.contact_constraints.count == 1);
    CHECK_NEAR(ctx, scene.contact_constraints.pool[0].friction, 1.0f, 0.0f);
    CHECK(ctx, contact_constraint_part::is_active(scene.contact_constraints.pool[0].friction_constraint_1));

    // the ball's contact point is carried along with the belt (it sticks: 4/7 m of the m friction allows)
    solve_contact_velocities(scene, 10);
    const Float3 contact_velocity = ball_contact_velocity(ball);
    CHECK_NEAR(ctx, contact_velocity.x, 2.0f, 1e-4f);
    CHECK_NEAR(ctx, contact_velocity.z, 0.0f, 1e-5f);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.y, 0.0f, 1e-5f);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.x, 2.0f * 2.0f / 7.0f, 1e-4f);

    release_contact_scene(scene);
}

void test_contact_constraints_override_restitution_and_mass_scale(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *ground = add_ground(scene);
    RigidBody *ball = add_ball(scene);

    // built with no restitution; js asks for 0.5 and the normal bias is rebuilt to bounce
    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {0.0f, -5.0f, 0.0f};
    CHECK(ctx, add_contact(scene, ground, ball, ball_manifold()));
    contact_constraints::ContactSettings settings = default_override();
    settings.combined_restitution = 0.5f;
    answer_first(scene, settings);
    apply_answers(scene);
    solve_contact_velocities(scene, 10);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.y, 0.5f * (5.0f - 9.81f * CONTACT_SCENE_DELTA_TIME), 1e-4f);

    // half the inverse mass: the ball counts double, so stopping it takes twice the impulse
    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    CHECK(ctx, add_contact(scene, ground, ball, ball_manifold()));
    settings = default_override();
    settings.inv_mass_scale_2 = 0.5f;
    answer_first(scene, settings);
    apply_answers(scene);
    CHECK_NEAR(ctx, scene.contact_constraints.pool[0].inv_mass_b, 0.5f * ball->motion_properties.inv_mass, 1e-12f);
    solve_contact_velocities(scene, 10);
    CHECK_NEAR(ctx, scene.contact_constraints.pool[0].contact_points[0].normal_constraint.total_lambda, 2.0f * ball_mass(ball), 0.05f);

    // an inverse mass scale of 0 on the only dynamic body leaves no constraint (jolt's test)
    begin_contact_step(scene);
    CHECK(ctx, add_contact(scene, ground, ball, ball_manifold()));
    settings = default_override();
    settings.inv_mass_scale_2 = 0.0f;
    answer_first(scene, settings);
    apply_answers(scene);
    CHECK(ctx, scene.contact_constraints.count == 0);

    release_contact_scene(scene);
}

void test_contact_constraints_override_reject_drops_and_zeroes_the_cache(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *ground = add_ground(scene);
    RigidBody *ball = add_ball(scene);
    RigidBody *other = add_contact_body(scene, sphere::create({.radius = SPHERE_RADIUS}), motion_type::MOTION_DYNAMIC, {3.0f, SPHERE_RADIUS, 0.0f});
    const Float3 other_point = {3.0f, 0.0f, 0.0f};
    const manifold::ContactManifold other_manifold = touching_manifold(&other_point, 1, {0.0f, 1.0f, 0.0f});

    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    CHECK(ctx, add_contact(scene, ground, ball, ball_manifold()));
    solve_contact_velocities(scene, 10);
    end_contact_step(scene);

    // persisted with last step's impulse, plus a second contact; js rejects the first
    begin_contact_step(scene);
    ball->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    other->motion_properties.linear_velocity = {0.0f, -1.0f, 0.0f};
    CHECK(ctx, add_contact(scene, ground, ball, ball_manifold()));
    CHECK(ctx, add_contact(scene, ground, other, other_manifold));
    CHECK(ctx, scene.contact_constraints.count == 2);
    CHECK(ctx, scene.events.contacts_persisted.length == 1 && scene.events.contacts_added.length == 1);
    const int32_t rejected_contact = scene.contact_constraints.pool[0].contact_index;
    const int32_t kept_contact = scene.contact_constraints.pool[1].contact_index;
    CHECK(ctx, scene.contact_constraints.pool[0].contact_points[0].normal_constraint.total_lambda > 0.0f);

    // answers: contacts_added first (the other ball, untouched), then contacts_persisted (the ball, rejected)
    events::reset_contact_answers(scene.answers, scene.events, 0);
    events::ContactOverride &answer = scene.answers.contact_override[1];
    answer.written = true;
    answer.reject = true;
    answer.settings = default_override();
    apply_answers(scene);

    CHECK(ctx, scene.contact_constraints.count == 1);
    CHECK(ctx, scene.contact_constraints.pool[0].contact_index == kept_contact);
    const contacts::CachedManifold &write =
        contacts::get_write_manifold(scene.contacts.contacts[(uint32_t)rejected_contact], scene.contacts);
    CHECK(ctx, write.contact_points[0].normal_lambda == 0.0f && write.friction_lambda_1 == 0.0f);

    // nothing holds the rejected ball up
    solve_contact_velocities(scene, 10);
    CHECK_NEAR(ctx, ball->motion_properties.linear_velocity.y, -1.0f, 0.0f);
    CHECK_NEAR(ctx, other->motion_properties.linear_velocity.y, 0.0f, 1e-5f);

    // a sensor answer drops the constraint likewise
    begin_contact_step(scene);
    CHECK(ctx, add_contact(scene, ground, other, other_manifold));
    contact_constraints::ContactSettings sensor = default_override();
    sensor.is_sensor = true;
    answer_first(scene, sensor);
    apply_answers(scene);
    CHECK(ctx, scene.contact_constraints.count == 0);

    release_contact_scene(scene);
}

void test_contact_constraints_sort_is_by_key_then_ids(TestCtx &ctx) {
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *ground = add_ground(scene);
    RigidBody *balls[5];
    begin_contact_step(scene);
    for (uint32_t i = 0; i < 5; i++) {
        balls[i] = add_contact_body(scene, sphere::create({.radius = SPHERE_RADIUS}), motion_type::MOTION_DYNAMIC, {(float)i * 2.0f, SPHERE_RADIUS, 0.0f});
        const Float3 point = {(float)i * 2.0f, 0.0f, 0.0f};
        CHECK(ctx, add_contact(scene, ground, balls[i], touching_manifold(&point, 1, {0.0f, 1.0f, 0.0f})));
    }

    collections::Vec<uint32_t> indices = {};
    for (uint32_t i = 5; i-- > 0;) indices.push(i);
    contact_constraints::sort_contact_indices(scene.contact_constraints, scene.bodies, indices);
    CHECK(ctx, indices.length == 5);
    for (uint32_t i = 1; i < indices.length; i++) {
        CHECK(ctx, scene.contact_constraints.pool[indices[i - 1]].sort_key <= scene.contact_constraints.pool[indices[i]].sort_key);
    }
    // the key is the 64 bit fnv-1a hash of body id a, sub shape a, body id b, sub shape b, each as its four little
    // endian bytes
    const contact_constraints::ContactConstraint &first = scene.contact_constraints.pool[0];
    const body_id::BodyId id_a = scene.bodies.pool[first.body_index_a]->id;
    const body_id::BodyId id_b = scene.bodies.pool[first.body_index_b]->id;
    CHECK(ctx, first.sort_key == fnv1a_64({id_a, sub_shape::EMPTY_SUB_SHAPE_ID, id_b, sub_shape::EMPTY_SUB_SHAPE_ID}));

    indices.release();
    release_contact_scene(scene);
}

void test_contact_constraints_sort_key_known_pair(TestCtx &ctx) {
    // bodies 1 and 2 (sequence 0) with no sub shapes: each id is one word of the hash
    CHECK(ctx, contact_constraints::compute_contact_sort_key(body_id::ser(1, 0), sub_shape::EMPTY_SUB_SHAPE_ID, body_id::ser(2, 0),
                                                             sub_shape::EMPTY_SUB_SHAPE_ID) ==
                   fnv1a_64({1, sub_shape::EMPTY_SUB_SHAPE_ID, 2, sub_shape::EMPTY_SUB_SHAPE_ID}));
    // the sequence and the sub shapes take part
    CHECK(ctx, contact_constraints::compute_contact_sort_key(body_id::ser(3, 1), 0x12345678u, body_id::ser(7, 2), 0) ==
                   fnv1a_64({body_id::ser(3, 1), 0x12345678u, body_id::ser(7, 2), 0}));
    CHECK(ctx, contact_constraints::compute_contact_sort_key(body_id::ser(3, 1), 0, body_id::ser(7, 2), 0) !=
                   contact_constraints::compute_contact_sort_key(body_id::ser(3, 2), 0, body_id::ser(7, 2), 0));
}

void test_contact_constraints_dynamic_body_without_mass_makes_a_constraint(TestCtx &ctx) {
    // whether to make a constraint goes by the motion type and the inverse mass scale, not the inverse mass: a dynamic
    // body whose translation is locked (inverse mass 0) still turns against the ground
    ContactScene scene = make_contact_scene(ctx);
    RigidBody *ground = add_ground(scene);
    RigidBody *ball = add_ball(scene);
    ball->motion_properties.inv_mass = 0.0f;

    begin_contact_step(scene);
    CHECK(ctx, add_contact(scene, ground, ball, ball_manifold()));
    CHECK(ctx, scene.contact_constraints.count == 1);

    // and an answer that scales the dynamic body's inverse mass to 0 drops it
    answer_first(scene, [] {
        contact_constraints::ContactSettings settings = default_override();
        settings.inv_mass_scale_2 = 0.0f;
        return settings;
    }());
    apply_answers(scene);
    CHECK(ctx, scene.contact_constraints.count == 0);

    release_contact_scene(scene);
}

void test_contact_combine_materials(TestCtx &ctx) {
    // a side whose material id the table holds takes the table's values; -1 or an id past the end takes its body's
    rigid_body::RigidBody body_a = {};
    rigid_body::RigidBody body_b = {};
    body_a.friction = 0.2f;
    body_a.restitution = 0.0f;
    body_a.friction_combine_mode = combine_material::GEOMETRIC_MEAN;
    body_a.restitution_combine_mode = combine_material::MAX;
    body_b = body_a;
    body_b.friction = 0.8f;

    contact_constraints::ContactConstraints table = contact_constraints::create();
    contact_constraints::ContactSettings settings = contact_constraints::ContactSettings();
    contact_constraints::combine_materials(settings, table, body_a, 3, body_b, -1);
    CHECK_NEAR(ctx, settings.combined_friction, scalar::sqrt(0.2f * 0.8f), 1e-6f);
    CHECK(ctx, settings.combined_restitution == 0.0f);

    CHECK(ctx, util::init(test_heap_begin()));
    table.materials.push({0.0f, 0.0f});
    table.materials.push({0.0f, 0.0f});
    table.materials.push({0.0f, 0.0f});
    table.materials.push({0.5f, 0.9f});
    contact_constraints::combine_materials(settings, table, body_a, 3, body_b, -1);
    CHECK_NEAR(ctx, settings.combined_friction, scalar::sqrt(0.5f * 0.8f), 1e-6f);
    CHECK_NEAR(ctx, settings.combined_restitution, 0.9f, 0.0f);
    // the other side's id, and one past the end
    contact_constraints::combine_materials(settings, table, body_a, 4, body_b, 3);
    CHECK_NEAR(ctx, settings.combined_friction, scalar::sqrt(0.2f * 0.5f), 1e-6f);
    // both sides from the table: ice under anything is ice
    contact_constraints::combine_materials(settings, table, body_a, 0, body_b, 3);
    CHECK(ctx, settings.combined_friction == 0.0f);
    contact_constraints::release(table);
}
