#include "physics/constraints/contact_constraints.h"

#include "math/mat4f.h"
#include "math/quatf.h"
#include "math/scalar.h"
#include "math/vec3f.h"
#include "physics/body/motion_properties.h"
#include "physics/body/motion_type.h"
#include "physics/constraints/combine_material.h"
#include "physics/shapes/shapes.h"
#include "util/assert.h"
#include "util/sort.h"

namespace contact_constraints {

using motion_properties::MotionProperties;
using rigid_body::RigidBody;

ContactConstraints create() {
    ContactConstraints contact_constraints = {};
    return contact_constraints;
}

void release(ContactConstraints &contact_constraints) {
    contact_constraints.pool.release();
    contact_constraints.count = 0;
    contact_constraints.materials.release();
}

/** the side's material, null when the table holds none for the id */
static const Material *material_of(const ContactConstraints &contact_constraints, int32_t material_id) {
    if (material_id < 0 || (uint32_t)material_id >= contact_constraints.materials.length) return nullptr;
    return &contact_constraints.materials.data[material_id];
}

void combine_materials(ContactSettings &io_settings, const ContactConstraints &contact_constraints, const RigidBody &body_a,
                       int32_t material_id_a, const RigidBody &body_b, int32_t material_id_b) {
    const Material *material_a = material_of(contact_constraints, material_id_a);
    const Material *material_b = material_of(contact_constraints, material_id_b);
    io_settings.combined_friction = combine_material::combine(material_a != nullptr ? material_a->friction : body_a.friction,
                                                              material_b != nullptr ? material_b->friction : body_b.friction,
                                                              body_a.friction_combine_mode, body_b.friction_combine_mode);
    io_settings.combined_restitution = combine_material::combine(material_a != nullptr ? material_a->restitution : body_a.restitution,
                                                                 material_b != nullptr ? material_b->restitution : body_b.restitution,
                                                                 body_a.restitution_combine_mode, body_b.restitution_combine_mode);
}

static ContactSettings make_contact_settings(const ContactConstraints &contact_constraints, const RigidBody &body_a, int32_t material_id_a,
                                             const RigidBody &body_b, int32_t material_id_b) {
    ContactSettings settings = {};
    combine_materials(settings, contact_constraints, body_a, material_id_a, body_b, material_id_b);
    settings.is_sensor = body_a.sensor || body_b.sensor;
    return settings;
}

/** one of the bodies must be dynamic and keep its mass under the settings to be able to create a contact constraint */
static bool has_dynamic_mass(const RigidBody &body_a, const RigidBody &body_b, const ContactSettings &settings) {
    return (body_a.motion_type == motion_type::MOTION_DYNAMIC && settings.inv_mass_scale_1 != 0.0f) ||
           (body_b.motion_type == motion_type::MOTION_DYNAMIC && settings.inv_mass_scale_2 != 0.0f);
}

/**
 * calculate friction bias with surface velocity support, for conveyor belts, rotating platforms etc: the surface
 * velocity at r_a (relative linear + relative angular x r_a) projected onto the tangent
 */
static float calculate_friction_bias(const ContactSettings &settings, Vec3f r_a, Vec3f tangent) {
    const Vec3f surface_velocity = vec3f::add(vec3f::load(settings.relative_linear_surface_velocity),
                                              vec3f::cross(vec3f::load(settings.relative_angular_surface_velocity), r_a));
    return vec3f::dot(tangent, surface_velocity);
}

/**
 * the normal velocity bias for a contact point: speculative contacts, restitution, and the gravity and force correction.
 * penetration > 0 when the points overlap along the normal, < 0 when apart; normal from a to b, r_a / r_b the moment arms
 * from each centre of mass to that body's contact point.
 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
static float calculate_normal_velocity_bias(const RigidBody &body_a, const RigidBody &body_b, float penetration, Vec3f normal, Vec3f r_a,
                                            Vec3f r_b, float combined_restitution, float delta_time, Vec3f gravity,
                                            float min_velocity_for_restitution) {
    static_assert(TYPE_A != motion_type::MOTION_STATIC || TYPE_B != motion_type::MOTION_STATIC, "static against static");

    // step 1: relative velocity at the contact point; kinematic bodies contribute velocity too
    constexpr bool a_is_moving = TYPE_A != motion_type::MOTION_STATIC;
    constexpr bool b_is_moving = TYPE_B != motion_type::MOTION_STATIC;
    constexpr bool a_is_dynamic = TYPE_A == motion_type::MOTION_DYNAMIC;
    constexpr bool b_is_dynamic = TYPE_B == motion_type::MOTION_DYNAMIC;

    Vec3f relative_velocity;
    if constexpr (a_is_moving && b_is_moving) {
        relative_velocity =
            vec3f::subtract(rigid_body::get_velocity_at_point_com(&body_b, r_b), rigid_body::get_velocity_at_point_com(&body_a, r_a));
    } else if constexpr (a_is_moving) {
        relative_velocity = vec3f::negate(rigid_body::get_velocity_at_point_com(&body_a, r_a));
    } else {
        relative_velocity = rigid_body::get_velocity_at_point_com(&body_b, r_b);
    }
    const float normal_velocity = vec3f::dot(relative_velocity, normal);

    // step 2: if there is no penetration, this is a speculative contact and we apply a bias so that the constraint
    // becomes relative_velocity . contact normal > -penetration / delta_time instead of relative_velocity . contact
    // normal > 0. see: gdc 2013, "physics for game programmers; continuous collision", erin catto
    const float speculative_contact_velocity_bias = scalar::max(0.0f, -penetration / delta_time);

    // step 3: the bias, with restitution when the velocity is big enough
    if (combined_restitution > 0.0f && normal_velocity < -min_velocity_for_restitution) {
        // this is where speculative contacts don't work great: we decide now whether to apply restitution. if the
        // relative velocity is big enough for a hit, we apply it (other constraints may still keep the objects apart)
        if (normal_velocity < -speculative_contact_velocity_bias) {
            // gravity and constant forces are applied at the beginning of the time step, so if we get here there was a
            // collision at the beginning of the step and we've applied too much force; restitution would add energy,
            // so we cancel the velocity those forces added
            Vec3f relative_acceleration = vec3f::zero();

            // the effect of gravity, on every non-static body
            if constexpr (a_is_moving && b_is_moving) {
                relative_acceleration = vec3f::scale(gravity, body_b.motion_properties.gravity_factor - body_a.motion_properties.gravity_factor);
            } else if constexpr (a_is_moving) {
                relative_acceleration = vec3f::scale(gravity, -body_a.motion_properties.gravity_factor);
            } else {
                relative_acceleration = vec3f::scale(gravity, body_b.motion_properties.gravity_factor);
            }

            // the effect of accumulated forces, on dynamic bodies only
            if constexpr (a_is_dynamic) {
                relative_acceleration = vec3f::scale_and_add(relative_acceleration, vec3f::load(body_a.motion_properties.force),
                                                             -body_a.motion_properties.inv_mass);
            }
            if constexpr (b_is_dynamic) {
                relative_acceleration = vec3f::scale_and_add(relative_acceleration, vec3f::load(body_b.motion_properties.force),
                                                             body_b.motion_properties.inv_mass);
            }

            // we only compensate forces towards the contact normal
            const float force_delta_velocity = scalar::min(0.0f, vec3f::dot(relative_acceleration, normal) * delta_time);

            return combined_restitution * (normal_velocity - force_delta_velocity);
        }

        // we predicted no hit; if other constraints make one happen, the speculative contact prevents penetration but
        // applies no restitution
        return speculative_contact_velocity_bias;
    }

    // no restitution, we can safely apply our contact velocity bias
    return speculative_contact_velocity_bias;
}

/** the world inverse inertias a constraint's parts are set up with, scaled by the contact's settings; zero for non-dynamic bodies */
struct ContactInertia {
    Mat4f inv_inertia_a;
    Mat4f inv_inertia_b;
};

/** the scaled world inverse inertia of a dynamic body, memoised per body per step (step_stamp) before the scale */
static Mat4f scaled_world_inverse_inertia(RigidBody &body, float scale, uint32_t step_stamp) {
    if (body.motion_type != motion_type::MOTION_DYNAMIC) return mat4f::zero();
    const Mat4f world_inverse_inertia = motion_properties::get_world_inverse_inertia(body.motion_properties, quatf::load(body.quaternion), step_stamp);
    return scale == 1.0f ? world_inverse_inertia : mat4f::multiply_scalar(world_inverse_inertia, scale);
}

/** combined materials and the bodies' inverse masses and inertias after the contact's scales */
static void set_constraint_settings(ContactConstraint &constraint, RigidBody &body_a, RigidBody &body_b, const ContactSettings &settings,
                                    uint32_t step_stamp, ContactInertia &out_inertia) {
    constraint.friction = settings.combined_friction;
    constraint.restitution = settings.combined_restitution;

    constraint.inv_mass_a = body_a.motion_type == motion_type::MOTION_DYNAMIC ? body_a.motion_properties.inv_mass * settings.inv_mass_scale_1 : 0.0f;
    constraint.inv_mass_b = body_b.motion_type == motion_type::MOTION_DYNAMIC ? body_b.motion_properties.inv_mass * settings.inv_mass_scale_2 : 0.0f;
    constraint.inv_inertia_scale_a = settings.inv_inertia_scale_1;
    constraint.inv_inertia_scale_b = settings.inv_inertia_scale_2;

    out_inertia.inv_inertia_a = scaled_world_inverse_inertia(body_a, settings.inv_inertia_scale_1, step_stamp);
    out_inertia.inv_inertia_b = scaled_world_inverse_inertia(body_b, settings.inv_inertia_scale_2, step_stamp);
}

/**
 * the manifold-level friction: each body's friction point (the mean of that body's own contact points, so a speculative
 * contact's separation along the normal does not lengthen the lever arms), each point's moment arm to a's friction point in
 * the contact plane, and the two linear parts and the angular part. r_as / r_bs are each point relative to its body's centre
 * of mass, in world space.
 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
static void calculate_friction_constraint_properties(ContactConstraint &constraint, const ContactInertia &inertia, const Vec3f *r_as,
                                                     const Vec3f *r_bs, const ContactSettings &settings) {
    const uint32_t num_points = constraint.num_contact_points;

    // friction points = the mean of each body's own contact points
    Vec3f friction_point_a = r_as[0];
    Vec3f friction_point_b = r_bs[0];
    for (uint32_t i = 1; i < num_points; i++) {
        friction_point_a = vec3f::add(friction_point_a, r_as[i]);
        friction_point_b = vec3f::add(friction_point_b, r_bs[i]);
    }
    const Vec3f count = vec3f::splat((float)num_points);
    friction_point_a = vec3f::divide(friction_point_a, count);
    friction_point_b = vec3f::divide(friction_point_b, count);

    // per-point moment arm to a's friction point, projected onto the contact plane
    const Vec3f normal = vec3f::load(constraint.normal);
    for (uint32_t i = 0; i < num_points; i++) {
        const Vec3f delta = vec3f::subtract(r_as[i], friction_point_a);
        constraint.contact_points[i].distance_to_friction_center = vec3f::length(vec3f::scale_and_add(delta, normal, -vec3f::dot(delta, normal)));
    }

    const Vec3f tangent_1 = vec3f::load(constraint.tangent_1);
    const Vec3f tangent_2 = vec3f::load(constraint.tangent_2);
    const float friction_bias_1 = calculate_friction_bias(settings, friction_point_a, tangent_1);
    const float friction_bias_2 = calculate_friction_bias(settings, friction_point_a, tangent_2);

    contact_constraint_part::calculate_constraint_properties<TYPE_A, TYPE_B>(constraint.friction_constraint_1, constraint.inv_mass_a,
                                                                             inertia.inv_inertia_a, friction_point_a, constraint.inv_mass_b,
                                                                             inertia.inv_inertia_b, friction_point_b, tangent_1, friction_bias_1);
    contact_constraint_part::calculate_constraint_properties<TYPE_A, TYPE_B>(constraint.friction_constraint_2, constraint.inv_mass_a,
                                                                             inertia.inv_inertia_a, friction_point_a, constraint.inv_mass_b,
                                                                             inertia.inv_inertia_b, friction_point_b, tangent_2, friction_bias_2);

    // only apply angular friction with more than 1 contact point; its bias is the relative angular surface velocity
    // along the normal
    if (num_points > 1) {
        angular_friction_constraint_part::calculate_constraint_properties<TYPE_A, TYPE_B>(
            constraint.angular_friction_constraint, inertia.inv_inertia_a, inertia.inv_inertia_b, normal,
            vec3f::dot(vec3f::load(settings.relative_angular_surface_velocity), normal));
    } else {
        angular_friction_constraint_part::deactivate(constraint.angular_friction_constraint);
    }
}

/** fold the four little endian bytes of a u32 into a 64 bit fnv-1a hash */
static inline uint64_t fnv1a_64_word(uint64_t hash, uint32_t word) {
    for (uint32_t byte = 0; byte < 4; byte++) {
        hash ^= (word >> (byte * 8)) & 0xffu;
        hash *= 0x100000001b3ull;
    }
    return hash;
}

uint64_t compute_contact_sort_key(body_id::BodyId body_id_a, sub_shape::SubShapeId sub_shape_id_a, body_id::BodyId body_id_b,
                                  sub_shape::SubShapeId sub_shape_id_b) {
    uint64_t hash = 0xcbf29ce484222325ull;
    hash = fnv1a_64_word(hash, body_id_a);
    hash = fnv1a_64_word(hash, sub_shape_id_a);
    hash = fnv1a_64_word(hash, body_id_b);
    hash = fnv1a_64_word(hash, sub_shape_id_b);
    return hash;
}

/**
 * the next constraint slot with the manifold-level fields both the narrowphase and the cache paths share: body and
 * sub-shape ids, sort key, normal and tangent basis, combined materials and scaled masses (jolt: CreateConstraint).
 * null (error set) when the pool cannot grow.
 */
static ContactConstraint *begin_contact_constraint(ContactConstraints &contact_constraints, RigidBody &body_a, RigidBody &body_b,
                                                   sub_shape::SubShapeId sub_shape_id_a, sub_shape::SubShapeId sub_shape_id_b,
                                                   int32_t contact_index, Vec3f world_space_normal, const ContactSettings &settings,
                                                   uint32_t step_stamp, ContactInertia &out_inertia) {
    if (contact_constraints.count >= contact_constraints.pool.length && !contact_constraints.pool.push(ContactConstraint())) {
        return nullptr;
    }
    ContactConstraint &constraint = contact_constraints.pool[contact_constraints.count];
    contact_constraints.count++;

    constraint.body_index_a = body_a.index;
    constraint.body_index_b = body_b.index;
    constraint.sub_shape_id_a = sub_shape_id_a;
    constraint.sub_shape_id_b = sub_shape_id_b;
    constraint.contact_index = contact_index;
    constraint.sort_key = compute_contact_sort_key(body_a.id, sub_shape_id_a, body_b.id, sub_shape_id_b);

    // normal and the orthonormal tangent basis from it
    vec3f::store(constraint.normal, world_space_normal);
    const Vec3f tangent_1 = vec3f::normalized_perpendicular(world_space_normal);
    vec3f::store(constraint.tangent_1, tangent_1);
    vec3f::store(constraint.tangent_2, vec3f::cross(world_space_normal, tangent_1));

    set_constraint_settings(constraint, body_a, body_b, settings, step_stamp, out_inertia);

    constraint.num_contact_points = 0;
    return &constraint;
}

/**
 * finish contact point i of a constraint whose local positions and warm start lambda are set: its restitution /
 * speculative bias and non-penetration part, r_a / r_b from each body's centre of mass to that body's own contact point
 * (jolt: WorldContactPoint::CalculateNonPenetrationConstraintProperties)
 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
static void setup_contact_point(ContactConstraint &constraint, uint32_t i, const RigidBody &body_a, const RigidBody &body_b,
                                const ContactInertia &inertia, Vec3f r_a, Vec3f r_b, float penetration,
                                const ContactSettings &settings, const world_settings::WorldSettings &world_settings, float delta_time) {
    WorldContactPoint &point = constraint.contact_points[i];

    const Vec3f normal = vec3f::load(constraint.normal);
    const float normal_velocity_bias = calculate_normal_velocity_bias<TYPE_A, TYPE_B>(
        body_a, body_b, penetration, normal, r_a, r_b, settings.combined_restitution, delta_time, vec3f::load(world_settings.gravity),
        world_settings.solver.min_velocity_for_restitution);

    contact_constraint_part::calculate_constraint_properties<TYPE_A, TYPE_B>(point.normal_constraint, constraint.inv_mass_a,
                                                                             inertia.inv_inertia_a, r_a, constraint.inv_mass_b,
                                                                             inertia.inv_inertia_b, r_b, normal, normal_velocity_bias);
}

/** a point relative to a manifold's base offset into body space: rot^T point - rot^T (centre of mass - base offset) */
static Vec3f to_body_space(const Mat4f &rotation, Vec3f point, Vec3f com_minus_base) {
    return vec3f::subtract(mat4f::multiply3x3_transposed_vec(rotation, point), mat4f::multiply3x3_transposed_vec(rotation, com_minus_base));
}

/**
 * the cached body-local points of a constraint relative to each body's centre of mass under the current rotations, and
 * their penetration along the normal (jolt: GetContactsFromCache)
 */
static void cached_moment_arms(Vec3f &out_r_a, Vec3f &out_r_b, float &out_penetration, const Mat4f &rotation_a, const Mat4f &rotation_b,
                               Vec3f delta_com, Vec3f local_a, Vec3f local_b, Vec3f normal) {
    out_r_a = mat4f::multiply3x3_vec(rotation_a, local_a);
    out_r_b = mat4f::multiply3x3_vec(rotation_b, local_b);
    out_penetration = vec3f::dot(vec3f::add(vec3f::subtract(out_r_a, out_r_b), delta_com), normal);
}

/** the manifold-level friction parts with the previous step's lambdas transferred wholesale (zero for a new contact) */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
static void setup_friction_constraints(ContactConstraint &constraint, const ContactInertia &inertia, const ContactSettings &settings,
                                       const Vec3f *r_as, const Vec3f *r_bs, float previous_friction_lambda_1, float previous_friction_lambda_2,
                                       float previous_angular_friction_lambda) {
    if (settings.combined_friction > 0.0f && constraint.num_contact_points > 0) {
        constraint.friction_constraint_1.total_lambda = previous_friction_lambda_1;
        constraint.friction_constraint_2.total_lambda = previous_friction_lambda_2;
        constraint.angular_friction_constraint.total_lambda = previous_angular_friction_lambda;
        calculate_friction_constraint_properties<TYPE_A, TYPE_B>(constraint, inertia, r_as, r_bs, settings);
    } else {
        contact_constraint_part::deactivate(constraint.friction_constraint_1);
        contact_constraint_part::deactivate(constraint.friction_constraint_2);
        angular_friction_constraint_part::deactivate(constraint.angular_friction_constraint);
    }
}

/**
 * every part of a constraint whose points, local positions and warm start lambdas are set: each point's non-penetration
 * part from its moment arms and penetration, then the manifold's friction parts starting from the given lambdas
 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
static void calculate_contact_constraint_properties(ContactConstraint &constraint, const RigidBody &body_a, const RigidBody &body_b,
                                                    const ContactInertia &inertia, const Vec3f *r_as, const Vec3f *r_bs, const float *penetrations,
                                                    const ContactSettings &settings, const world_settings::WorldSettings &world_settings,
                                                    float delta_time, float previous_friction_lambda_1, float previous_friction_lambda_2,
                                                    float previous_angular_friction_lambda) {
    for (uint32_t i = 0; i < constraint.num_contact_points; i++) {
        setup_contact_point<TYPE_A, TYPE_B>(constraint, i, body_a, body_b, inertia, r_as[i], r_bs[i], penetrations[i], settings, world_settings,
                                            delta_time);
    }
    setup_friction_constraints<TYPE_A, TYPE_B>(constraint, inertia, settings, r_as, r_bs, previous_friction_lambda_1, previous_friction_lambda_2,
                                               previous_angular_friction_lambda);
}

typedef void (*CalculatePropertiesFn)(ContactConstraint &constraint, const RigidBody &body_a, const RigidBody &body_b,
                                      const ContactInertia &inertia, const Vec3f *r_as, const Vec3f *r_bs, const float *penetrations,
                                      const ContactSettings &settings, const world_settings::WorldSettings &world_settings, float delta_time,
                                      float previous_friction_lambda_1, float previous_friction_lambda_2, float previous_angular_friction_lambda);

/** by [motion type a][motion type b]; a constraint always has a dynamic body */
static constexpr CalculatePropertiesFn CALCULATE_PROPERTIES_TABLE[3][3] = {
    {nullptr, nullptr, calculate_contact_constraint_properties<motion_type::MOTION_STATIC, motion_type::MOTION_DYNAMIC>},
    {nullptr, nullptr, calculate_contact_constraint_properties<motion_type::MOTION_KINEMATIC, motion_type::MOTION_DYNAMIC>},
    {calculate_contact_constraint_properties<motion_type::MOTION_DYNAMIC, motion_type::MOTION_STATIC>,
     calculate_contact_constraint_properties<motion_type::MOTION_DYNAMIC, motion_type::MOTION_KINEMATIC>,
     calculate_contact_constraint_properties<motion_type::MOTION_DYNAMIC, motion_type::MOTION_DYNAMIC>},
};

static void calculate_contact_constraint_properties(ContactConstraint &constraint, const RigidBody &body_a, const RigidBody &body_b,
                                                    const ContactInertia &inertia, const Vec3f *r_as, const Vec3f *r_bs, const float *penetrations,
                                                    const ContactSettings &settings, const world_settings::WorldSettings &world_settings,
                                                    float delta_time, float previous_friction_lambda_1, float previous_friction_lambda_2,
                                                    float previous_angular_friction_lambda) {
    const CalculatePropertiesFn fn = CALCULATE_PROPERTIES_TABLE[body_a.motion_type][body_b.motion_type];
    CC_ASSERT(fn != nullptr);
    fn(constraint, body_a, body_b, inertia, r_as, r_bs, penetrations, settings, world_settings, delta_time, previous_friction_lambda_1,
       previous_friction_lambda_2, previous_angular_friction_lambda);
}

/**
 * the transferred friction lambdas mirrored into the write-side cache, so they hold if the constraint never runs;
 * store_applied_impulses overwrites them once solved
 */
static void mirror_friction_lambdas(contacts::CachedManifold &write, const ContactConstraint &constraint) {
    write.friction_lambda_1 = constraint.friction_constraint_1.total_lambda;
    write.friction_lambda_2 = constraint.friction_constraint_2.total_lambda;
    write.angular_friction_lambda = constraint.angular_friction_constraint.total_lambda;
}

static void zero_friction_lambdas(contacts::CachedManifold &write) {
    write.friction_lambda_1 = 0.0f;
    write.friction_lambda_2 = 0.0f;
    write.angular_friction_lambda = 0.0f;
}

/** the listener's onContactAdded / onContactPersisted, as an event with the manifold's world points */
static void append_contact_event(events::PhysicsEvents &events, bool persisted, int32_t contact_index, const RigidBody &body_a,
                                 const RigidBody &body_b, sub_shape::SubShapeId sub_shape_id_a, sub_shape::SubShapeId sub_shape_id_b,
                                 int32_t material_id_a, int32_t material_id_b, Vec3f normal, float penetration,
                                 uint32_t num_points, const Float3 *points_a, const Float3 *points_b, const ContactSettings &settings) {
    events::ContactEvent event;
    event.settings = settings;
    event.contact_index = contact_index;
    event.body_index_a = body_a.index;
    event.body_index_b = body_b.index;
    event.body_id_a = body_a.id;
    event.body_id_b = body_b.id;
    event.user_data_a = body_a.user_data;
    event.user_data_b = body_b.user_data;
    event.sub_shape_id_a = sub_shape_id_a;
    event.sub_shape_id_b = sub_shape_id_b;
    event.material_id_a = material_id_a;
    event.material_id_b = material_id_b;
    vec3f::store(event.normal, normal);
    event.penetration = penetration;
    event.num_points = num_points;
    for (uint32_t i = 0; i < manifold::MAX_CONTACT_POINTS; i++) {
        if (i < num_points) {
            event.points_a[i] = points_a[i];
            event.points_b[i] = points_b[i];
        } else {
            event.points_a[i] = {0.0f, 0.0f, 0.0f};
            event.points_b[i] = {0.0f, 0.0f, 0.0f};
        }
    }
    (persisted ? events.contacts_persisted : events.contacts_added).push(event);
}

bool add_contact_constraint(ContactConstraints &contact_constraints, contacts::Contacts &contacts, pairs::Pairs &pairs, int32_t pair_record,
                            RigidBody &body_a_in, RigidBody &body_b_in, const manifold::ContactManifold &manifold_in,
                            const world_settings::WorldSettings &settings, events::PhysicsEvents &events, float delta_time,
                            uint32_t step_stamp) {
    CC_ASSERT(manifold_in.num_contact_points <= manifold::MAX_CONTACT_POINTS);

    // swap bodies so that body a id < body b id
    RigidBody *body_a_ptr = &body_a_in;
    RigidBody *body_b_ptr = &body_b_in;
    const manifold::ContactManifold *manifold_ptr = &manifold_in;
    manifold::ContactManifold swapped;
    if (body_a_in.id > body_b_in.id) {
        body_a_ptr = &body_b_in;
        body_b_ptr = &body_a_in;
        swapped = manifold_in;
        manifold::swap_shapes(swapped);
        manifold_ptr = &swapped;
    }
    RigidBody &body_a = *body_a_ptr;
    RigidBody &body_b = *body_b_ptr;
    const manifold::ContactManifold &contact_manifold = *manifold_ptr;
    const uint32_t num_points = contact_manifold.num_contact_points;

    // find or create the contact nested under this pair record (every contact of the chain shares the id-sorted body
    // pair, so matching sub-shape ids is enough after the swap above)
    contacts::Contact *existing_contact =
        contacts::find_contact_in_pair(contacts, pairs, pair_record, contact_manifold.sub_shape_id_a, contact_manifold.sub_shape_id_b);
    contacts::Contact *contact = existing_contact;
    if (contact == nullptr) {
        contact = contacts::create_contact(contacts, pairs, pair_record, body_a, body_b, contact_manifold.sub_shape_id_a,
                                           contact_manifold.sub_shape_id_b);
        if (contact == nullptr) return false;
    }
    const bool persisted = existing_contact != nullptr;

    // stamp contact as processed this frame (for stale contact cleanup)
    contact->last_processed_frame = contacts.frame_stamp;

    // double-buffered cache: read the step before's data, write this step's (jolt: mReadCache->Find, mWriteCache->Create)
    const contacts::CachedManifold &read = contacts::get_read_manifold(*contact, contacts);
    contacts::CachedManifold &write = contacts::get_write_manifold(*contact, contacts);

    // rotation matrices for both bodies, transposed for the inverse rotation
    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    // the world space normal into body b's local space
    const Vec3f world_space_normal = contact_manifold.world_space_normal;
    vec3f::store(write.contact_normal, vec3f::normalize(mat4f::multiply3x3_transposed_vec(rotation_b, world_space_normal)));
    write.num_contact_points = num_points;

    // each body's centre of mass relative to the base offset: the manifold's points go to body space and to moment arms
    // without passing through world space
    const Vec3f base_offset = contact_manifold.base_offset;
    const Vec3f com_minus_base_a = vec3f::subtract(vec3f::load(body_a.center_of_mass_position), base_offset);
    const Vec3f com_minus_base_b = vec3f::subtract(vec3f::load(body_b.center_of_mass_position), base_offset);

    // world points (for the event) and body-local points: local = rot^T point - rot^T (centre of mass - base offset)
    Float3 positions_a[manifold::MAX_CONTACT_POINTS];
    Float3 positions_b[manifold::MAX_CONTACT_POINTS];
    Float3 local_positions_a[manifold::MAX_CONTACT_POINTS];
    Float3 local_positions_b[manifold::MAX_CONTACT_POINTS];
    for (uint32_t i = 0; i < num_points; i++) {
        vec3f::store(positions_a[i], manifold::get_world_space_contact_point_on_a(contact_manifold, i));
        vec3f::store(positions_b[i], manifold::get_world_space_contact_point_on_b(contact_manifold, i));
        vec3f::store(local_positions_a[i], to_body_space(rotation_a, contact_manifold.relative_contact_points_on_a[i], com_minus_base_a));
        vec3f::store(local_positions_b[i], to_body_space(rotation_b, contact_manifold.relative_contact_points_on_b[i], com_minus_base_b));
    }

    const ContactSettings contact_settings =
        make_contact_settings(contact_constraints, body_a, contact_manifold.material_id_a, body_b, contact_manifold.material_id_b);

    // the listener's call, which js answers through contact_override before the solve
    if (persisted) write.flags |= contacts::MANIFOLD_FLAG_CONTACT_PERSISTED;
    append_contact_event(events, persisted, contact->contact_index, body_a, body_b, contact_manifold.sub_shape_id_a,
                         contact_manifold.sub_shape_id_b, contact_manifold.material_id_a, contact_manifold.material_id_b,
                         contact_manifold.world_space_normal, contact_manifold.penetration_depth, num_points, positions_a, positions_b,
                         contact_settings);

    // a sensor makes no constraint, and one of the bodies must be dynamic and have mass
    ContactConstraint *constraint = nullptr;
    ContactInertia inertia;
    if (!contact_settings.is_sensor && has_dynamic_mass(body_a, body_b, contact_settings)) {
        constraint = begin_contact_constraint(contact_constraints, body_a, body_b, contact_manifold.sub_shape_id_a,
                                              contact_manifold.sub_shape_id_b, contact->contact_index, world_space_normal,
                                              contact_settings, step_stamp, inertia);
    }

    if (constraint != nullptr) {
        Vec3f r_as[manifold::MAX_CONTACT_POINTS];
        Vec3f r_bs[manifold::MAX_CONTACT_POINTS];
        float penetrations[manifold::MAX_CONTACT_POINTS];
        const float threshold = settings.contacts.contact_point_preserve_lambda_max_dist_sq;

        for (uint32_t i = 0; i < num_points; i++) {
            WorldContactPoint &point = constraint->contact_points[i];
            constraint->num_contact_points++;
            point.position_a = positions_a[i];
            point.position_b = positions_b[i];
            point.local_position_a = local_positions_a[i];
            point.local_position_b = local_positions_b[i];

            // a close contact point from last update carries its lambda over (first match wins, both points within the
            // threshold); friction lambdas carry over at manifold level below
            point.normal_constraint.total_lambda = 0.0f;
            if (persisted) {
                for (uint32_t j = 0; j < read.num_contact_points; j++) {
                    const contacts::CachedContactPoint &cached = read.contact_points[j];
                    if (vec3f::squared_distance(vec3f::load(point.local_position_a), vec3f::load(cached.position_1)) <= threshold &&
                        vec3f::squared_distance(vec3f::load(point.local_position_b), vec3f::load(cached.position_2)) <= threshold) {
                        point.normal_constraint.total_lambda = cached.normal_lambda;
                        break;
                    }
                }
            }

            // penetration along the normal, then the points relative to each body's centre of mass
            const Vec3f point_a = contact_manifold.relative_contact_points_on_a[i];
            const Vec3f point_b = contact_manifold.relative_contact_points_on_b[i];
            penetrations[i] = vec3f::dot(vec3f::subtract(point_a, point_b), world_space_normal);
            r_as[i] = vec3f::subtract(point_a, com_minus_base_a);
            r_bs[i] = vec3f::subtract(point_b, com_minus_base_b);
        }

        if (persisted) {
            calculate_contact_constraint_properties(*constraint, body_a, body_b, inertia, r_as, r_bs, penetrations, contact_settings, settings,
                                                    delta_time, read.friction_lambda_1, read.friction_lambda_2, read.angular_friction_lambda);
        } else {
            calculate_contact_constraint_properties(*constraint, body_a, body_b, inertia, r_as, r_bs, penetrations, contact_settings, settings,
                                                    delta_time, 0.0f, 0.0f, 0.0f);
        }

        // the write-side cache for next frame's warm starting; the impulses are updated after solving
        for (uint32_t i = 0; i < num_points; i++) {
            const WorldContactPoint &point = constraint->contact_points[i];
            contacts::CachedContactPoint &cached = write.contact_points[i];
            cached.position_1 = point.local_position_a;
            cached.position_2 = point.local_position_b;
            cached.normal_lambda = point.normal_constraint.total_lambda;
        }
        mirror_friction_lambdas(write, *constraint);
        return true;
    }

    // no constraint (a sensor, no dynamic mass): the points still go to the write-side cache, so the next step reports
    // the contact persisted, with no impulses
    for (uint32_t i = 0; i < num_points; i++) {
        contacts::CachedContactPoint &cached = write.contact_points[i];
        cached.position_1 = local_positions_a[i];
        cached.position_2 = local_positions_b[i];
        cached.normal_lambda = 0.0f;
    }
    zero_friction_lambdas(write);
    return false;
}

bool add_contact_constraint_from_cache(ContactConstraints &contact_constraints, contacts::Contacts &contacts, RigidBody &body_a,
                                       RigidBody &body_b, contacts::Contact &contact, const world_settings::WorldSettings &settings,
                                       events::PhysicsEvents &events, float delta_time, uint32_t step_stamp) {
    // stamp contact as processed this frame (for stale contact cleanup)
    contact.last_processed_frame = contacts.frame_stamp;

    const contacts::CachedManifold &read = contacts::get_read_manifold(contact, contacts);
    contacts::CachedManifold &write = contacts::get_write_manifold(contact, contacts);
    const uint32_t num_points = read.num_contact_points;

    // carry the cached manifold forward; store_applied_impulses overwrites the lambdas once solved
    write.contact_normal = read.contact_normal;
    write.num_contact_points = num_points;
    for (uint32_t i = 0; i < num_points; i++) write.contact_points[i] = read.contact_points[i];
    write.friction_lambda_1 = read.friction_lambda_1;
    write.friction_lambda_2 = read.friction_lambda_2;
    write.angular_friction_lambda = read.angular_friction_lambda;
    write.flags |= contacts::MANIFOLD_FLAG_CONTACT_PERSISTED;

    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    // the normal is cached in b's local space
    const Vec3f world_space_normal = vec3f::normalize(mat4f::multiply3x3_vec(rotation_b, vec3f::load(read.contact_normal)));

    // world positions of the cached body-local points under the current transforms
    const Vec3f center_of_mass_a = vec3f::load(body_a.center_of_mass_position);
    const Vec3f center_of_mass_b = vec3f::load(body_b.center_of_mass_position);
    Float3 positions_a[manifold::MAX_CONTACT_POINTS];
    Float3 positions_b[manifold::MAX_CONTACT_POINTS];
    for (uint32_t i = 0; i < num_points; i++) {
        vec3f::store(positions_a[i], vec3f::add(mat4f::multiply3x3_vec(rotation_a, vec3f::load(read.contact_points[i].position_1)), center_of_mass_a));
        vec3f::store(positions_b[i], vec3f::add(mat4f::multiply3x3_vec(rotation_b, vec3f::load(read.contact_points[i].position_2)), center_of_mass_b));
    }

    // the cache keeps no materials: look them up through the shapes by sub shape id
    const int32_t material_id_a = shapes::get_shape_material(body_a.shape, contact.sub_shape_id_a);
    const int32_t material_id_b = shapes::get_shape_material(body_b.shape, contact.sub_shape_id_b);
    const ContactSettings contact_settings = make_contact_settings(contact_constraints, body_a, material_id_a, body_b, material_id_b);

    // we don't have the penetration depth anymore: estimate it as the deepest (a - b) . normal of the cached points
    float max_penetration = -scalar::FLOAT_MAX;
    for (uint32_t i = 0; i < num_points; i++) {
        const Vec3f difference = vec3f::subtract(vec3f::load(positions_a[i]), vec3f::load(positions_b[i]));
        max_penetration = scalar::max(max_penetration, vec3f::dot(difference, world_space_normal));
    }
    append_contact_event(events, true, contact.contact_index, body_a, body_b, contact.sub_shape_id_a, contact.sub_shape_id_b, material_id_a,
                         material_id_b, world_space_normal, num_points == 0 ? 0.0f : max_penetration, num_points, positions_a, positions_b,
                         contact_settings);

    // a sensor makes no constraint, and one of the bodies must be dynamic and have mass
    ContactConstraint *constraint = nullptr;
    ContactInertia inertia;
    if (!contact_settings.is_sensor && has_dynamic_mass(body_a, body_b, contact_settings)) {
        constraint = begin_contact_constraint(contact_constraints, body_a, body_b, contact.sub_shape_id_a, contact.sub_shape_id_b,
                                              contact.contact_index, world_space_normal, contact_settings, step_stamp, inertia);
    }

    if (constraint != nullptr) {
        // a's centre of mass relative to b's
        const Vec3f delta_com = vec3f::subtract(center_of_mass_a, center_of_mass_b);

        Vec3f r_as[manifold::MAX_CONTACT_POINTS];
        Vec3f r_bs[manifold::MAX_CONTACT_POINTS];
        float penetrations[manifold::MAX_CONTACT_POINTS];
        for (uint32_t i = 0; i < num_points; i++) {
            WorldContactPoint &point = constraint->contact_points[i];
            constraint->num_contact_points++;
            point.position_a = positions_a[i];
            point.position_b = positions_b[i];
            point.local_position_a = read.contact_points[i].position_1;
            point.local_position_b = read.contact_points[i].position_2;
            point.normal_constraint.total_lambda = read.contact_points[i].normal_lambda;
            cached_moment_arms(r_as[i], r_bs[i], penetrations[i], rotation_a, rotation_b, delta_com, vec3f::load(point.local_position_a),
                               vec3f::load(point.local_position_b), world_space_normal);
        }

        calculate_contact_constraint_properties(*constraint, body_a, body_b, inertia, r_as, r_bs, penetrations, contact_settings, settings,
                                                delta_time, read.friction_lambda_1, read.friction_lambda_2, read.angular_friction_lambda);
        mirror_friction_lambdas(write, *constraint);
        return true;
    }

    // no constraint: the points carried over are kept for persisted semantics, but nothing solves, so the write-side
    // lambdas read zero
    for (uint32_t i = 0; i < num_points; i++) write.contact_points[i].normal_lambda = 0.0f;
    zero_friction_lambdas(write);
    return false;
}

void apply_contact_override(ContactConstraints &contact_constraints, uint32_t constraint_index, contacts::Contacts &contacts,
                            bodies::Bodies &bodies, const ContactSettings &settings, const world_settings::WorldSettings &world_settings,
                            float delta_time) {
    CC_ASSERT(constraint_index < contact_constraints.count);
    CC_ASSERT(!settings.is_sensor);
    ContactConstraint &constraint = contact_constraints.pool[constraint_index];
    RigidBody &body_a = *bodies.pool[constraint.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.body_index_b];

    ContactInertia inertia;
    set_constraint_settings(constraint, body_a, body_b, settings, bodies.step_stamp, inertia);

    // the parts keep their total lambdas, the warm start the build gave them; friction the build left off (no friction at
    // build time) starts from the cached impulses instead, as it would have had the settings been there when it was built
    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));
    const Vec3f delta_com = vec3f::subtract(vec3f::load(body_a.center_of_mass_position), vec3f::load(body_b.center_of_mass_position));
    const Vec3f normal = vec3f::load(constraint.normal);
    Vec3f r_as[manifold::MAX_CONTACT_POINTS];
    Vec3f r_bs[manifold::MAX_CONTACT_POINTS];
    float penetrations[manifold::MAX_CONTACT_POINTS];
    for (uint32_t i = 0; i < constraint.num_contact_points; i++) {
        const WorldContactPoint &point = constraint.contact_points[i];
        cached_moment_arms(r_as[i], r_bs[i], penetrations[i], rotation_a, rotation_b, delta_com, vec3f::load(point.local_position_a),
                           vec3f::load(point.local_position_b), normal);
    }
    float friction_1 = constraint.friction_constraint_1.total_lambda;
    float friction_2 = constraint.friction_constraint_2.total_lambda;
    float angular_friction = constraint.angular_friction_constraint.total_lambda;
    if (friction_1 == 0.0f && friction_2 == 0.0f && angular_friction == 0.0f && settings.combined_friction > 0.0f) {
        const contacts::CachedManifold &read = contacts::get_read_manifold(contacts.contacts[(uint32_t)constraint.contact_index], contacts);
        friction_1 = read.friction_lambda_1;
        friction_2 = read.friction_lambda_2;
        angular_friction = read.angular_friction_lambda;
    }
    calculate_contact_constraint_properties(constraint, body_a, body_b, inertia, r_as, r_bs, penetrations, settings, world_settings, delta_time,
                                            friction_1, friction_2, angular_friction);
}

void apply_contact_overrides(ContactConstraints &contact_constraints, contacts::Contacts &contacts, bodies::Bodies &bodies,
                             const events::PhysicsEvents &events, const events::PhysicsAnswers &answers,
                             const world_settings::WorldSettings &world_settings, float delta_time) {
    const uint32_t num_added = events.contacts_added.length;
    uint32_t num_answers = answers.contact_override.length;
    if (num_answers > num_added + events.contacts_persisted.length) num_answers = num_added + events.contacts_persisted.length;

    bool any_written = false;
    for (uint32_t i = 0; i < num_answers && !any_written; i++) any_written = answers.contact_override[i].written;
    if (!any_written || contact_constraints.count == 0) return;

    // contact index to the constraint built from it, -1 for none
    collections::Vec<int32_t> constraint_of_contact = {};
    collections::Vec<uint8_t> dropped = {};
    if (!constraint_of_contact.reserve(contacts.contacts.length) || !dropped.reserve(contact_constraints.count)) {
        constraint_of_contact.release();
        dropped.release();
        return;
    }
    constraint_of_contact.length = contacts.contacts.length;
    for (uint32_t i = 0; i < constraint_of_contact.length; i++) constraint_of_contact[i] = -1;
    dropped.length = contact_constraints.count;
    for (uint32_t k = 0; k < contact_constraints.count; k++) {
        constraint_of_contact[(uint32_t)contact_constraints.pool[k].contact_index] = (int32_t)k;
        dropped[k] = 0;
    }

    bool any_dropped = false;
    for (uint32_t i = 0; i < num_answers; i++) {
        const events::ContactOverride &answer = answers.contact_override[i];
        if (!answer.written) continue;
        const events::ContactEvent &event = i < num_added ? events.contacts_added[i] : events.contacts_persisted[i - num_added];
        const int32_t constraint_index = constraint_of_contact[(uint32_t)event.contact_index];
        if (constraint_index < 0) continue;

        const ContactConstraint &constraint = contact_constraints.pool[(uint32_t)constraint_index];
        const RigidBody &body_a = *bodies.pool[constraint.body_index_a];
        const RigidBody &body_b = *bodies.pool[constraint.body_index_b];
        const ContactSettings &settings = answer.settings;
        const bool keeps = !answer.reject && !settings.is_sensor && has_dynamic_mass(body_a, body_b, settings);
        if (!keeps) {
            // nothing solves it, so nothing it caches may warm start the next step
            contacts::CachedManifold &write = contacts::get_write_manifold(contacts.contacts[(uint32_t)event.contact_index], contacts);
            for (uint32_t j = 0; j < write.num_contact_points; j++) write.contact_points[j].normal_lambda = 0.0f;
            zero_friction_lambdas(write);
            dropped[(uint32_t)constraint_index] = 1;
            any_dropped = true;
            continue;
        }
        apply_contact_override(contact_constraints, (uint32_t)constraint_index, contacts, bodies, settings, world_settings, delta_time);
    }

    if (any_dropped) {
        uint32_t kept = 0;
        for (uint32_t k = 0; k < contact_constraints.count; k++) {
            if (dropped[k]) continue;
            if (kept != k) contact_constraints.pool[kept] = contact_constraints.pool[k];
            kept++;
        }
        contact_constraints.count = kept;
    }

    constraint_of_contact.release();
    dropped.release();
}

/** the two bodies' velocities a constraint's parts read and accumulate into, loaded and written back once */
struct Velocities {
    Vec3f linear_a;
    Vec3f angular_a;
    Vec3f linear_b;
    Vec3f angular_b;
};

/** jolt's sGetVelocities: a static body's velocities are never read */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline void get_velocities(Velocities &out, const MotionProperties &motion_properties_a, const MotionProperties &motion_properties_b) {
    if constexpr (TYPE_A != motion_type::MOTION_STATIC) {
        out.linear_a = vec3f::load(motion_properties_a.linear_velocity);
        out.angular_a = vec3f::load(motion_properties_a.angular_velocity);
    } else {
        out.linear_a = vec3f::zero();
        out.angular_a = vec3f::zero();
    }
    if constexpr (TYPE_B != motion_type::MOTION_STATIC) {
        out.linear_b = vec3f::load(motion_properties_b.linear_velocity);
        out.angular_b = vec3f::load(motion_properties_b.angular_velocity);
    } else {
        out.linear_b = vec3f::zero();
        out.angular_b = vec3f::zero();
    }
}

/** jolt's sSetVelocities: dynamic bodies only, with the locked translation axes zeroed */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline void set_velocities(MotionProperties &motion_properties_a, MotionProperties &motion_properties_b, const Velocities &velocities) {
    if constexpr (TYPE_A == motion_type::MOTION_DYNAMIC) {
        vec3f::store(motion_properties_a.linear_velocity,
                     motion_properties::apply_translation_dof_constraint(velocities.linear_a, motion_properties_a.allowed_degrees_of_freedom));
        vec3f::store(motion_properties_a.angular_velocity, velocities.angular_a);
    }
    if constexpr (TYPE_B == motion_type::MOTION_DYNAMIC) {
        vec3f::store(motion_properties_b.linear_velocity,
                     motion_properties::apply_translation_dof_constraint(velocities.linear_b, motion_properties_b.allowed_degrees_of_freedom));
        vec3f::store(motion_properties_b.angular_velocity, velocities.angular_b);
    }
}

/** the previous step's impulses, scaled for the new time step, applied once before the velocity iterations */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
static void warm_start_constraint(ContactConstraint &constraint, MotionProperties &motion_properties_a,
                                  MotionProperties &motion_properties_b, float warm_start_ratio) {
    Velocities velocities;
    get_velocities<TYPE_A, TYPE_B>(velocities, motion_properties_a, motion_properties_b);

    const float inv_mass_a = constraint.inv_mass_a;
    const float inv_mass_b = constraint.inv_mass_b;
    const Vec3f normal = vec3f::load(constraint.normal);
    bool any_impulse_applied = false;

    if (contact_constraint_part::is_active(constraint.friction_constraint_1) &&
        contact_constraint_part::warm_start<TYPE_A, TYPE_B>(constraint.friction_constraint_1, velocities.linear_a, velocities.angular_a,
                                                            velocities.linear_b, velocities.angular_b, inv_mass_a, inv_mass_b,
                                                            vec3f::load(constraint.tangent_1), warm_start_ratio)) {
        any_impulse_applied = true;
    }
    if (contact_constraint_part::is_active(constraint.friction_constraint_2) &&
        contact_constraint_part::warm_start<TYPE_A, TYPE_B>(constraint.friction_constraint_2, velocities.linear_a, velocities.angular_a,
                                                            velocities.linear_b, velocities.angular_b, inv_mass_a, inv_mass_b,
                                                            vec3f::load(constraint.tangent_2), warm_start_ratio)) {
        any_impulse_applied = true;
    }
    if (angular_friction_constraint_part::is_active(constraint.angular_friction_constraint) &&
        angular_friction_constraint_part::warm_start<TYPE_A, TYPE_B>(constraint.angular_friction_constraint, velocities.angular_a,
                                                                     velocities.angular_b, warm_start_ratio)) {
        any_impulse_applied = true;
    }

    for (uint32_t i = 0; i < constraint.num_contact_points; i++) {
        if (contact_constraint_part::warm_start<TYPE_A, TYPE_B>(constraint.contact_points[i].normal_constraint, velocities.linear_a,
                                                                velocities.angular_a, velocities.linear_b, velocities.angular_b, inv_mass_a,
                                                                inv_mass_b, normal, warm_start_ratio)) {
            any_impulse_applied = true;
        }
    }

    // nothing stored from last frame: the loaded velocities equal the bodies', nothing to write back
    if (any_impulse_applied) set_velocities<TYPE_A, TYPE_B>(motion_properties_a, motion_properties_b, velocities);
}

/**
 * one projected gauss-seidel pass over a constraint: friction first, then non-penetration, which is more important so
 * is solved last. true if any impulse was applied
 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
static bool solve_velocity_constraint(ContactConstraint &constraint, MotionProperties &motion_properties_a,
                                      MotionProperties &motion_properties_b) {
    Velocities velocities;
    get_velocities<TYPE_A, TYPE_B>(velocities, motion_properties_a, motion_properties_b);

    const float inv_mass_a = constraint.inv_mass_a;
    const float inv_mass_b = constraint.inv_mass_b;
    const Vec3f normal = vec3f::load(constraint.normal);
    bool any_impulse_applied = false;

    const bool linear_friction_active =
        contact_constraint_part::is_active(constraint.friction_constraint_1) || contact_constraint_part::is_active(constraint.friction_constraint_2);
    const bool angular_friction_active = angular_friction_constraint_part::is_active(constraint.angular_friction_constraint);

    // the friction caps from the previous iteration's non-penetration impulses: friction * sum(lambda_n) for the
    // linear parts, friction * sum(d_i lambda_n_i) for the angular part
    float max_linear_lambda = 0.0f;
    float max_angular_lambda = 0.0f;
    if (linear_friction_active || angular_friction_active) {
        for (uint32_t i = 0; i < constraint.num_contact_points; i++) {
            const WorldContactPoint &point = constraint.contact_points[i];
            const float lambda = point.normal_constraint.total_lambda;
            max_linear_lambda += lambda;
            max_angular_lambda += point.distance_to_friction_center * lambda;
        }
        max_linear_lambda *= constraint.friction;
        max_angular_lambda *= constraint.friction;
    }

    // the two linear friction parts with the joint friction-cone clamp
    if (linear_friction_active) {
        const Vec3f tangent_1 = vec3f::load(constraint.tangent_1);
        const Vec3f tangent_2 = vec3f::load(constraint.tangent_2);
        float lambda_1 = contact_constraint_part::solve_velocity_constraint_get_total_lambda<TYPE_A, TYPE_B>(
            constraint.friction_constraint_1, velocities.linear_a, velocities.angular_a, velocities.linear_b, velocities.angular_b, tangent_1);
        float lambda_2 = contact_constraint_part::solve_velocity_constraint_get_total_lambda<TYPE_A, TYPE_B>(
            constraint.friction_constraint_2, velocities.linear_a, velocities.angular_a, velocities.linear_b, velocities.angular_b, tangent_2);

        // the + FLOAT_MIN_NORMAL keeps a total_lambda_sq too small to divide by from being scaled
        const float total_lambda_sq = lambda_1 * lambda_1 + lambda_2 * lambda_2;
        if (total_lambda_sq > max_linear_lambda * max_linear_lambda + scalar::FLOAT_MIN_NORMAL) {
            const float scale = max_linear_lambda / scalar::sqrt(total_lambda_sq);
            lambda_1 *= scale;
            lambda_2 *= scale;
        }

        if (contact_constraint_part::solve_velocity_constraint_apply_lambda<TYPE_A, TYPE_B>(
                constraint.friction_constraint_1, velocities.linear_a, velocities.angular_a, velocities.linear_b, velocities.angular_b,
                inv_mass_a, inv_mass_b, tangent_1, lambda_1)) {
            any_impulse_applied = true;
        }
        if (contact_constraint_part::solve_velocity_constraint_apply_lambda<TYPE_A, TYPE_B>(
                constraint.friction_constraint_2, velocities.linear_a, velocities.angular_a, velocities.linear_b, velocities.angular_b,
                inv_mass_a, inv_mass_b, tangent_2, lambda_2)) {
            any_impulse_applied = true;
        }
    }

    // the angular friction part, clamped symmetrically about the contact normal
    if (angular_friction_active &&
        angular_friction_constraint_part::solve_velocity_constraint<TYPE_A, TYPE_B>(constraint.angular_friction_constraint, velocities.angular_a,
                                                                                    velocities.angular_b, normal, -max_angular_lambda,
                                                                                    max_angular_lambda)) {
        any_impulse_applied = true;
    }

    // the non-penetration parts: contacts can only push, never pull
    for (uint32_t i = 0; i < constraint.num_contact_points; i++) {
        contact_constraint_part::ContactConstraintPart &normal_constraint = constraint.contact_points[i].normal_constraint;
        const float total_lambda = scalar::max(
            0.0f, contact_constraint_part::solve_velocity_constraint_get_total_lambda<TYPE_A, TYPE_B>(
                      normal_constraint, velocities.linear_a, velocities.angular_a, velocities.linear_b, velocities.angular_b, normal));
        if (contact_constraint_part::solve_velocity_constraint_apply_lambda<TYPE_A, TYPE_B>(normal_constraint, velocities.linear_a,
                                                                                         velocities.angular_a, velocities.linear_b,
                                                                                         velocities.angular_b, inv_mass_a, inv_mass_b, normal,
                                                                                         total_lambda)) {
            any_impulse_applied = true;
        }
    }

    if (!any_impulse_applied) return false;

    set_velocities<TYPE_A, TYPE_B>(motion_properties_a, motion_properties_b, velocities);
    return true;
}

typedef void (*WarmStartFn)(ContactConstraint &constraint, MotionProperties &motion_properties_a, MotionProperties &motion_properties_b,
                            float warm_start_ratio);
typedef bool (*SolveVelocityFn)(ContactConstraint &constraint, MotionProperties &motion_properties_a,
                                MotionProperties &motion_properties_b);

/** by [motion type a][motion type b]; a constraint always has a dynamic body */
static constexpr WarmStartFn WARM_START_TABLE[3][3] = {
    {nullptr, nullptr, warm_start_constraint<motion_type::MOTION_STATIC, motion_type::MOTION_DYNAMIC>},
    {nullptr, nullptr, warm_start_constraint<motion_type::MOTION_KINEMATIC, motion_type::MOTION_DYNAMIC>},
    {warm_start_constraint<motion_type::MOTION_DYNAMIC, motion_type::MOTION_STATIC>, warm_start_constraint<motion_type::MOTION_DYNAMIC, motion_type::MOTION_KINEMATIC>,
     warm_start_constraint<motion_type::MOTION_DYNAMIC, motion_type::MOTION_DYNAMIC>},
};

static constexpr SolveVelocityFn SOLVE_VELOCITY_TABLE[3][3] = {
    {nullptr, nullptr, solve_velocity_constraint<motion_type::MOTION_STATIC, motion_type::MOTION_DYNAMIC>},
    {nullptr, nullptr, solve_velocity_constraint<motion_type::MOTION_KINEMATIC, motion_type::MOTION_DYNAMIC>},
    {solve_velocity_constraint<motion_type::MOTION_DYNAMIC, motion_type::MOTION_STATIC>, solve_velocity_constraint<motion_type::MOTION_DYNAMIC, motion_type::MOTION_KINEMATIC>,
     solve_velocity_constraint<motion_type::MOTION_DYNAMIC, motion_type::MOTION_DYNAMIC>},
};

void warm_start_velocity_constraints(ContactConstraints &contact_constraints, bodies::Bodies &bodies, const collections::Vec<uint32_t> &constraint_indices,
                                     float warm_start_ratio) {
    for (uint32_t i = 0; i < constraint_indices.length; i++) {
        ContactConstraint &constraint = contact_constraints.pool[constraint_indices[i]];
        RigidBody &body_a = *bodies.pool[constraint.body_index_a];
        RigidBody &body_b = *bodies.pool[constraint.body_index_b];
        const WarmStartFn fn = WARM_START_TABLE[body_a.motion_type][body_b.motion_type];
        if (fn != nullptr) fn(constraint, body_a.motion_properties, body_b.motion_properties, warm_start_ratio);
    }
}

bool solve_velocity_constraints_for_island(ContactConstraints &contact_constraints, bodies::Bodies &bodies,
                                           const collections::Vec<uint32_t> &constraint_indices) {
    bool any_impulse_applied = false;
    for (uint32_t i = 0; i < constraint_indices.length; i++) {
        ContactConstraint &constraint = contact_constraints.pool[constraint_indices[i]];
        RigidBody &body_a = *bodies.pool[constraint.body_index_a];
        RigidBody &body_b = *bodies.pool[constraint.body_index_b];

        // skip if both bodies are static/kinematic
        const SolveVelocityFn fn = SOLVE_VELOCITY_TABLE[body_a.motion_type][body_b.motion_type];
        if (fn != nullptr && fn(constraint, body_a.motion_properties, body_b.motion_properties)) any_impulse_applied = true;
    }
    return any_impulse_applied;
}

void store_applied_impulses(ContactConstraints &contact_constraints, contacts::Contacts &contacts) {
    for (uint32_t i = 0; i < contact_constraints.count; i++) {
        const ContactConstraint &constraint = contact_constraints.pool[i];
        contacts::CachedManifold &write = contacts::get_write_manifold(contacts.contacts[(uint32_t)constraint.contact_index], contacts);

        // per-point non-penetration impulse
        for (uint32_t j = 0; j < constraint.num_contact_points; j++) {
            write.contact_points[j].normal_lambda = constraint.contact_points[j].normal_constraint.total_lambda;
        }

        // manifold-level friction impulses
        mirror_friction_lambdas(write, constraint);
    }
}

/** the world inverse inertia of a dynamic body under a rotation, scaled by the contact's inverse inertia scale */
static Mat4f scaled_inverse_inertia_for_rotation(const RigidBody &body, float scale, const Mat4f &rotation) {
    const Mat4f inverse_inertia = motion_properties::get_inverse_inertia_for_rotation(body.motion_properties, rotation);
    return scale == 1.0f ? inverse_inertia : mat4f::multiply_scalar(inverse_inertia, scale);
}

template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
static bool solve_position_constraint(ContactConstraint &constraint, RigidBody &body_a, RigidBody &body_b, float penetration_slop,
                                      float baumgarte_factor, float max_penetration_distance) {
    const Vec3f normal = vec3f::load(constraint.normal);

    // the bodies may have moved since the last iteration
    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    // a's centre of mass relative to b's
    const Vec3f delta_com = vec3f::subtract(vec3f::load(body_a.center_of_mass_position), vec3f::load(body_b.center_of_mass_position));

    // world inverse inertia, built on the first penetrating point: a constraint whose points are all separated
    // (speculative contacts) skips it entirely
    bool inertia_computed = false;
    Mat4f inv_inertia_a = mat4f::zero();
    Mat4f inv_inertia_b = mat4f::zero();

    bool any_impulse_applied = false;
    for (uint32_t i = 0; i < constraint.num_contact_points; i++) {
        WorldContactPoint &point = constraint.contact_points[i];

        // the contact points relative to each body's centre of mass in world space (the bodies may have moved)
        const Vec3f r_a = mat4f::multiply3x3_vec(rotation_a, vec3f::load(point.local_position_a));
        const Vec3f r_b = mat4f::multiply3x3_vec(rotation_b, vec3f::load(point.local_position_b));

        // separation along the normal (negative if interpenetrating), allowing a little penetration (the slop) so
        // contacts don't jitter between touching and not, and clamped so a deep overlap doesn't take a huge impulse
        const Vec3f penetration_vector = vec3f::subtract(vec3f::subtract(r_b, r_a), delta_com);
        const float separation = scalar::max(vec3f::dot(penetration_vector, normal) + penetration_slop, -max_penetration_distance);

        // only enforce the constraint when separation < 0 (otherwise we're apart)
        if (separation >= 0.0f) continue;

        if (!inertia_computed) {
            inertia_computed = true;
            if constexpr (TYPE_A == motion_type::MOTION_DYNAMIC) {
                inv_inertia_a = scaled_inverse_inertia_for_rotation(body_a, constraint.inv_inertia_scale_a, rotation_a);
            }
            if constexpr (TYPE_B == motion_type::MOTION_DYNAMIC) {
                inv_inertia_b = scaled_inverse_inertia_for_rotation(body_b, constraint.inv_inertia_scale_b, rotation_b);
            }
        }

        // the constraint properties with the current inertia and moment arms (the bodies may have moved)
        contact_constraint_part::calculate_constraint_properties<TYPE_A, TYPE_B>(point.normal_constraint, constraint.inv_mass_a, inv_inertia_a,
                                                                                 r_a, constraint.inv_mass_b, inv_inertia_b, r_b, normal, 0.0f);

        if (contact_constraint_part::solve_position_constraint<TYPE_A, TYPE_B>(point.normal_constraint, body_a, constraint.inv_mass_a, body_b,
                                                                               constraint.inv_mass_b, normal, separation, baumgarte_factor)) {
            any_impulse_applied = true;
        }
    }
    return any_impulse_applied;
}

typedef bool (*SolvePositionFn)(ContactConstraint &constraint, RigidBody &body_a, RigidBody &body_b, float penetration_slop,
                                float baumgarte_factor, float max_penetration_distance);

static constexpr SolvePositionFn SOLVE_POSITION_TABLE[3][3] = {
    {nullptr, nullptr, solve_position_constraint<motion_type::MOTION_STATIC, motion_type::MOTION_DYNAMIC>},
    {nullptr, nullptr, solve_position_constraint<motion_type::MOTION_KINEMATIC, motion_type::MOTION_DYNAMIC>},
    {solve_position_constraint<motion_type::MOTION_DYNAMIC, motion_type::MOTION_STATIC>,
     solve_position_constraint<motion_type::MOTION_DYNAMIC, motion_type::MOTION_KINEMATIC>,
     solve_position_constraint<motion_type::MOTION_DYNAMIC, motion_type::MOTION_DYNAMIC>},
};

bool solve_position_constraints_for_island(ContactConstraints &contact_constraints, bodies::Bodies &bodies,
                                           const collections::Vec<uint32_t> &constraint_indices, float penetration_slop, float baumgarte_factor,
                                           float max_penetration_distance) {
    bool any_impulse_applied = false;
    for (uint32_t c = 0; c < constraint_indices.length; c++) {
        ContactConstraint &constraint = contact_constraints.pool[constraint_indices[c]];
        RigidBody &body_a = *bodies.pool[constraint.body_index_a];
        RigidBody &body_b = *bodies.pool[constraint.body_index_b];

        // skip if both bodies are static/kinematic
        const SolvePositionFn fn = SOLVE_POSITION_TABLE[body_a.motion_type][body_b.motion_type];
        if (fn != nullptr && fn(constraint, body_a, body_b, penetration_slop, baumgarte_factor, max_penetration_distance)) {
            any_impulse_applied = true;
        }
    }
    return any_impulse_applied;
}

/** by sort key, then body a's id, then body b's id */
static bool contact_sorts_before(const ContactConstraints &contact_constraints, const bodies::Bodies &bodies, uint32_t a, uint32_t b) {
    const ContactConstraint &constraint_a = contact_constraints.pool[a];
    const ContactConstraint &constraint_b = contact_constraints.pool[b];
    if (constraint_a.sort_key != constraint_b.sort_key) return constraint_a.sort_key < constraint_b.sort_key;
    const body_id::BodyId body_a_id_a = bodies.pool[constraint_a.body_index_a]->id;
    const body_id::BodyId body_a_id_b = bodies.pool[constraint_b.body_index_a]->id;
    if (body_a_id_a != body_a_id_b) return body_a_id_a < body_a_id_b;
    return bodies.pool[constraint_a.body_index_b]->id < bodies.pool[constraint_b.body_index_b]->id;
}

void sort_contact_indices(const ContactConstraints &contact_constraints, const bodies::Bodies &bodies, collections::Vec<uint32_t> &contact_indices) {
    sort::merge_sort(contact_indices.data, contact_indices.data + contact_indices.length,
                     [&](uint32_t a, uint32_t b) { return contact_sorts_before(contact_constraints, bodies, a, b); });
}

}  // namespace contact_constraints
