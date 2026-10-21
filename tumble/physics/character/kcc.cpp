#include "tumble/physics/character/kcc.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/vec4f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/sleep.h"
#include "tumble/physics/broadphase/broadphase.h"
#include "tumble/physics/character/character_constraints.h"
#include "tumble/physics/collision/cast_shape_vs_shape.h"
#include "tumble/physics/collision/collide_shape_vs_shape.h"
#include "tumble/physics/collision/gjk.h"
#include "tumble/physics/collision/support.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/physics/pairs.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/util/object.h"
#include "tumble/util/sort.h"

TUMBLE_NAMESPACE_BEGIN

namespace kcc {

using rigid_body::RigidBody;
using world::World;

/**
 * when cos_max_slope_angle is greater than this value, slope checking is disabled.
 * all contacts are considered supported regardless of angle.
 */
static constexpr float NO_MAX_SLOPE_ANGLE = 0.9999f;

/**
 * backface flip threshold for surface normals: ~-sin(1 degree). allow a 1 degree slop:
 * a perpendicular (grazing) contact is not backfacing and must not be flipped.
 */
static constexpr float BACK_FACE_FLIP_THRESHOLD = -0.0174524f;

static constexpr float IMPULSE_DAMPING_FACTOR = 0.9f;
static constexpr float IMPULSE_PENETRATION_RESOLUTION_FACTOR = 0.4f;

/** the most steep slope normals walk_stairs keeps */
static constexpr uint32_t WALK_STAIRS_MAX_STEEP_SLOPE_NORMALS = 16;

struct IgnoreSingleBodyChainedBodyFilter {
    body_id::BodyId body_id;
    filter::BodyFilter inner_body_filter;
    void *inner_body_filter_context;
};

static bool ignore_single_body_chained_body_filter(const RigidBody *body, void *context) {
    const IgnoreSingleBodyChainedBodyFilter &state = *static_cast<const IgnoreSingleBodyChainedBodyFilter *>(context);
    if (body->id == state.body_id) return false;
    if (state.inner_body_filter != nullptr) return state.inner_body_filter(body, state.inner_body_filter_context);
    return true;
}

/** the caller's filter with the inner body skipped: a view sharing its layer lists */
static filter::Filter chain_ignore_single_body(const filter::Filter &filter, IgnoreSingleBodyChainedBodyFilter &state, body_id::BodyId body_id) {
    state.body_id = body_id;
    state.inner_body_filter = filter.body_filter;
    state.inner_body_filter_context = filter.body_filter_context;
    filter::Filter chained = filter;
    chained.body_filter = ignore_single_body_chained_body_filter;
    chained.body_filter_context = &state;
    return chained;
}


static void reset_contact(CharacterContact &contact) {
    contact.position = vec3f::make(0.0f, 0.0f, 0.0f);
    contact.linear_velocity = vec3f::make(0.0f, 0.0f, 0.0f);
    contact.contact_normal = vec3f::make(0.0f, 0.0f, 0.0f);
    contact.surface_normal = vec3f::make(0.0f, 0.0f, 0.0f);
    contact.distance = 0.0f;
    contact.fraction = 0.0f;
    contact.body_id = body_id::INVALID_BODY_ID;
    contact.sub_shape_id = 0;
    contact.material_id = -1;
    contact.motion_type = motion_type::MOTION_STATIC;
    contact.is_sensor = false;
    contact.had_collision = false;
    contact.was_discarded = false;
    contact.can_push_character = false;
    contact.can_receive_impulses = true;
}

static CharacterContact *acquire_contact(collections::Vec<CharacterContact> &contacts) {
    CharacterContact contact;
    reset_contact(contact);
    if (!contacts.push(contact)) return nullptr;
    return &contacts[contacts.length - 1];
}


static TrackedContact *acquire_tracked_contact(KCC &character, body_id::BodyId id, sub_shape::SubShapeId sub_shape_id) {
    const TrackedContact value = {id, sub_shape_id, 0, {true, true}};
    if (!character.tracked_contacts.push(value)) return nullptr;
    return &character.tracked_contacts[character.tracked_contacts.length - 1];
}

/** a live entry by full body id + sub-shape id, null if not found */
static TrackedContact *find_tracked_contact(KCC &character, body_id::BodyId id, sub_shape::SubShapeId sub_shape_id) {
    collections::Vec<TrackedContact> &live = character.tracked_contacts;
    for (uint32_t i = 0; i < live.length; i++) {
        if (live[i].body_id == id && live[i].sub_shape_id == sub_shape_id) return &live[i];
    }
    return nullptr;
}


KCC *create(const KCCSettings &settings, Vec3f position, Quatf quaternion) {
    KCC *character = util::create<KCC>();
    if (character == nullptr) return nullptr;

    character->up = vec3f::normalize(vec3f::load(settings.up));
    const Vec3f up = character->up;

    character->position = position;
    character->quaternion = quaternion;
    character->linear_velocity = vec3f::make(0.0f, 0.0f, 0.0f);

    character->shape = settings.shape;
    character->shape_offset = vec3f::load(settings.shape_offset);

    character->mass = settings.mass;
    character->max_strength = settings.max_strength;
    character->predictive_contact_distance = settings.predictive_contact_distance;
    character->max_collision_iterations = settings.max_collision_iterations;
    character->max_constraint_iterations = settings.max_constraint_iterations;
    character->min_time_remaining = settings.min_time_remaining;
    character->collision_tolerance = settings.collision_tolerance;
    character->character_padding = settings.character_padding;
    character->max_num_hits = settings.max_num_hits;
    character->hit_reduction_cos_max_angle = settings.hit_reduction_cos_max_angle;
    character->penetration_recovery_speed = settings.penetration_recovery_speed;
    character->max_slope_angle = settings.max_slope_angle;
    character->cos_max_slope_angle = scalar::cos(settings.max_slope_angle);
    character->back_face_mode = settings.back_face_mode;
    character->enhanced_internal_edge_removal = settings.enhanced_internal_edge_removal;

    if (settings.has_supporting_volume_plane) {
        const Float3 &normal = settings.supporting_volume_plane.normal;
        character->supporting_volume_plane = vec4f::make(normal.x, normal.y, normal.z, settings.supporting_volume_plane.constant);
    } else {
        // default: infinite plane below character, which accepts any contact as potentially supporting
        character->supporting_volume_plane = vec4f::make(vec3f::x(up), vec3f::y(up), vec3f::z(up), -1.0e10f);
    }

    character->ground.state = IN_AIR;
    character->ground.normal = vec3f::make(0.0f, 1.0f, 0.0f);
    character->ground.velocity = vec3f::make(0.0f, 0.0f, 0.0f);
    character->ground.body_id = body_id::INVALID_BODY_ID;
    character->ground.sub_shape_id = 0;
    character->ground.position = vec3f::make(0.0f, 0.0f, 0.0f);

    character->user_data = settings.user_data;
    character->observes_contacts = settings.observes_contacts;
    character->inner_rigid_body_id = body_id::INVALID_BODY_ID;
    character->inner_rigid_body_shape = settings.inner_rigid_body_shape;
    character->inner_rigid_body_object_layer = settings.inner_rigid_body_object_layer;

    character->last_delta_time = 0.0f;
    return character;
}

void destroy(KCC *character) {
    character->contacts.release();
    character->tracked_contacts.release();
    character->contact_events.release();
    character->constraints.release();
    character->sorted_constraints.release();
    character->previous_constraints.release();
    character->ignored_contacts.release();
    character->supporting_ignored_contacts.release();
    character->supporting_probe_contacts.release();
    character->walk_stairs_probe_contacts.release();
    character->set_shape_contacts.release();
    util::destroy(character);
}

/** the shape's world position at a character position: position + rotated shape offset + character padding * up */
static Vec3f get_shape_position(const KCC &character, Vec3f position) {
    const Vec3f offset = vec3f::transform_quat(character.shape_offset, character.quaternion);
    return vec3f::scale_and_add(vec3f::add(position, offset), character.up, character.character_padding);
}

static Vec3f get_inner_body_position(const KCC &character) { return get_shape_position(character, character.position); }

/** the character's shape pose at a character position, unscaled */
static Transformf shape_transform_at(const KCC &character, Vec3f position) {
    Transformf transform = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    vec3f::store(transform.position, get_shape_position(character, position));
    quatf::store(transform.quaternion, character.quaternion);
    return transform;
}

void add(World &world, KCC &character) {
    if (character.inner_rigid_body_id != body_id::INVALID_BODY_ID) return;

    if (character.inner_rigid_body_shape == nullptr) return;

    rigid_body::RigidBodySettings settings;
    settings.shape = character.inner_rigid_body_shape;
    settings.object_layer = character.inner_rigid_body_object_layer;
    settings.motion_type = motion_type::MOTION_KINEMATIC;
    settings.user_data = character.user_data;
    settings.observes_contacts = character.observes_contacts;
    vec3f::store(settings.position, get_inner_body_position(character));
    quatf::store(settings.quaternion, character.quaternion);
    // disable sleeping so sensor contacts keep coming
    settings.allow_sleeping = false;

    RigidBody *inner_rigid_body = rigid_body::create(world, settings);
    if (inner_rigid_body == nullptr) return;
    character.inner_rigid_body_id = inner_rigid_body->id;
}

void remove(World &world, KCC &character) {
    if (character.inner_rigid_body_id == body_id::INVALID_BODY_ID) return;

    RigidBody *inner_rigid_body = rigid_body::get(world, character.inner_rigid_body_id);
    if (inner_rigid_body != nullptr) rigid_body::remove(world, inner_rigid_body);

    character.inner_rigid_body_id = body_id::INVALID_BODY_ID;
}


bool is_slope_too_steep(const KCC &character, Vec3f normal) {
    // if slope checking is disabled, no slope is too steep
    if (character.cos_max_slope_angle > NO_MAX_SLOPE_ANGLE) return false;
    // a flat surface has dot(normal, up) = 1, a vertical wall has dot = 0
    return vec3f::dot(normal, character.up) < character.cos_max_slope_angle;
}

void get_center_of_mass_transform(const KCC &character, Vec3f &out_position, Quatf &out_quaternion) {
    out_position = vec3f::transform_quat(character.shape_offset, character.quaternion);
    out_position = vec3f::add(out_position, character.position);
    out_quaternion = character.quaternion;
}

bool is_supported(const KCC &character) { return character.ground.state == ON_GROUND || character.ground.state == ON_STEEP_GROUND; }

/**
 * the ground velocity at the character's position for a body moving with the given velocities: the linear velocity
 * plus the motion of the character's position rotating about the body's centre of mass over delta_time
 */
static Vec3f calculate_character_ground_velocity(const KCC &character, Vec3f body_center_of_mass, Vec3f linear_velocity,
                                                  Vec3f angular_velocity, float delta_time) {
    const float angular_velocity_len_sq = vec3f::squared_length(angular_velocity);
    if (angular_velocity_len_sq < 1e-12f) {
        return linear_velocity;
    }

    const float angular_velocity_len = scalar::sqrt(angular_velocity_len_sq);

    const Vec3f axis = vec3f::scale(angular_velocity, 1.0f / angular_velocity_len);

    const Quatf rotation = quatf::set_axis_angle(axis, angular_velocity_len * delta_time);

    // rotate the character position around the body centre of mass
    Vec3f new_position = vec3f::subtract(character.position, body_center_of_mass);
    new_position = vec3f::transform_quat(new_position, rotation);
    new_position = vec3f::add(new_position, body_center_of_mass);

    // linear_velocity + (new_position - character_position) / delta_time
    if (delta_time > 1e-8f) return vec3f::add(vec3f::scale(vec3f::subtract(new_position, character.position), 1.0f / delta_time), linear_velocity);
    return linear_velocity;
}

static bool is_zero(const Float3 &v) { return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f; }

/**
 * the body's velocities (zero for static) with the policy's surface velocity for it added (a conveyor belt on a static
 * body); a zero surface velocity adds nothing, so a body's -0 stays -0
 */
static void get_adjusted_body_velocity(const RigidBody &body, const kcc_policy::Policy *policy, Vec3f &out_linear_velocity,
                                       Vec3f &out_angular_velocity) {
    if (body.motion_type == motion_type::MOTION_DYNAMIC || body.motion_type == motion_type::MOTION_KINEMATIC) {
        out_linear_velocity = vec3f::load(body.motion_properties.linear_velocity);
        out_angular_velocity = vec3f::load(body.motion_properties.angular_velocity);
    } else {
        out_linear_velocity = vec3f::make(0.0f, 0.0f, 0.0f);
        out_angular_velocity = vec3f::make(0.0f, 0.0f, 0.0f);
    }

    const kcc_policy::BodyPolicy &own = kcc_policy::get(policy, body.id);
    if (!is_zero(own.surface_linear_velocity)) out_linear_velocity = vec3f::add(out_linear_velocity, vec3f::load(own.surface_linear_velocity));
    if (!is_zero(own.surface_angular_velocity)) out_angular_velocity = vec3f::add(out_angular_velocity, vec3f::load(own.surface_angular_velocity));
}

/** a contact's properties from a hit on `body`: position on the body, normals, distance, ids, motion type and point velocity */
static void fill_contact_properties(CharacterContact &contact, const KCC &character, const RigidBody &body, const kcc_policy::Policy *policy,
                                    Vec3f point_b, Vec3f contact_normal, float distance, sub_shape::SubShapeId sub_shape_id,
                                    int32_t material_id) {
    // position is the contact point on shape b (the world body)
    contact.position = point_b;
    contact.contact_normal = contact_normal;

    // the geometric surface normal, more stable than the contact normal from gjk / epa
    contact.surface_normal = rigid_body::get_world_space_surface_normal(&body, sub_shape_id, contact.position);

    // flip surface normal if hitting back face
    if (vec3f::dot(contact.contact_normal, contact.surface_normal) < BACK_FACE_FLIP_THRESHOLD) {
        contact.surface_normal = vec3f::negate(contact.surface_normal);
    }

    // replace surface normal with contact normal if contact normal points more upward, this handles edges/corners better
    if (vec3f::dot(contact.contact_normal, character.up) > vec3f::dot(contact.surface_normal, character.up)) {
        contact.surface_normal = contact.contact_normal;
    }

    contact.distance = distance;
    contact.body_id = body.id;
    contact.sub_shape_id = sub_shape_id;
    contact.material_id = material_id;
    contact.motion_type = body.motion_type;
    contact.is_sensor = body.sensor;

    // point velocity: v_point = v_linear + w x (position - centre of mass)
    Vec3f linear_velocity, angular_velocity;
    get_adjusted_body_velocity(body, policy, linear_velocity, angular_velocity);
    Vec3f r = vec3f::subtract(contact.position, vec3f::load(body.center_of_mass_position));
    const Vec3f angular_component = vec3f::cross(angular_velocity, r);
    contact.linear_velocity = vec3f::add(linear_velocity, angular_component);
}


/** false when the policy ignores the contact's body; a removed body's contact is kept */
static bool validate_contact(World &world, const CharacterContact &contact, const kcc_policy::Policy *policy) {
    if (rigid_body::get(world, contact.body_id) == nullptr) return true;
    return !kcc_policy::get(policy, contact.body_id).ignore;
}

static void push_event(KCC &character, ContactEventKind kind, const CharacterContact &contact) {
    character.contact_events.push({kind, contact.body_id, contact.sub_shape_id, contact.position, vec3f::negate(contact.contact_normal)});
}

/**
 * the contact met: added, or persisted the first time this frame, with its settings from the policy for its body; met
 * again this frame, the settings it was first given
 */
static void contact_added(World &world, KCC &character, const CharacterContact &contact, const kcc_policy::Policy *policy,
                          CharacterContactSettings &io_settings) {
    const RigidBody *body = rigid_body::get(world, contact.body_id);

    TrackedContact *tracked = find_tracked_contact(character, contact.body_id, contact.sub_shape_id);
    if (tracked != nullptr) {
        if (++tracked->count == 1) {
            if (body != nullptr) {
                const kcc_policy::BodyPolicy &own = kcc_policy::get(policy, contact.body_id);
                io_settings.can_push_character = own.can_push_character;
                io_settings.can_receive_impulses = own.can_receive_impulses;
                push_event(character, CONTACT_PERSISTED, contact);
            }
            tracked->settings = io_settings;
        } else {
            io_settings = tracked->settings;
        }
    } else {
        if (body != nullptr) {
            const kcc_policy::BodyPolicy &own = kcc_policy::get(policy, contact.body_id);
            io_settings.can_push_character = own.can_push_character;
            io_settings.can_receive_impulses = own.can_receive_impulses;
            push_event(character, CONTACT_ADDED, contact);
        }
        TrackedContact *value = acquire_tracked_contact(character, contact.body_id, contact.sub_shape_id);
        if (value != nullptr) {
            value->count = 1;
            value->settings = io_settings;
        }
    }
}

// the collide collector: every hit at a position into a contact list

struct CollideCollectorContext {
    World *world;
    const KCC *character;
    const kcc_policy::Policy *policy;
    collections::Vec<CharacterContact> *contacts;
    uint32_t max_hits;
    bool max_hits_exceeded;
    float hit_reduction_cos_max_angle;
};

static void character_collide_add_hit(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeHit &hit) {
    CollideCollectorContext &context = *static_cast<CollideCollectorContext *>(collector.context);
    collections::Vec<CharacterContact> &contacts = *context.contacts;

    // if we've reached max hits, try to merge similar contacts before giving up
    if (contacts.length >= context.max_hits) {
        context.max_hits_exceeded = true;

        if (context.hit_reduction_cos_max_angle > -1.0f) {
            // one full pass over every pair: contacts on the same body with very similar normals keep only the deepest
            for (int32_t i = (int32_t)contacts.length - 1; i >= 0; i--) {
                for (int32_t j = i - 1; j >= 0; j--) {
                    const CharacterContact &contact_i = contacts[(uint32_t)i];
                    const CharacterContact &contact_j = contacts[(uint32_t)j];
                    if (contact_i.body_id == contact_j.body_id &&
                        vec3f::dot(contact_i.contact_normal, contact_j.contact_normal) > context.hit_reduction_cos_max_angle) {
                        const bool i_is_last = (uint32_t)i == contacts.length - 1;
                        if (contact_i.distance > contact_j.distance) {
                            // i moves out, the last contact (already visited) takes its place
                            contacts.swap_remove((uint32_t)i);
                            break;
                        }

                        // the last contact moves into j; when that was i, i is seen again at j later
                        contacts.swap_remove((uint32_t)j);
                        if (i_is_last) break;
                    }
                }
            }
        }

        // still too many hits, stop the query
        if (contacts.length >= context.max_hits) {
            collector.early_out_fraction = -scalar::FLOAT_MAX;
            return;
        }
    }

    const RigidBody *body = rigid_body::get(*context.world, hit.body_id_b);
    if (body == nullptr) return;

    CharacterContact *contact = acquire_contact(contacts);
    if (contact == nullptr) return;

    // penetration_axis points from a to b, the normal from b to a; a zero-length axis yields a zero normal, not NaN
    Vec3f contact_normal = vec3f::make(0.0f, 0.0f, 0.0f);
    if (vec3f::squared_length(hit.penetration_axis) > 0.0f) {
        contact_normal = vec3f::normalize(hit.penetration_axis);
        contact_normal = vec3f::negate(contact_normal);
    }

    // distance is negative penetration (negative = penetrating); a static collision has no sweep fraction
    fill_contact_properties(*contact, *context.character, *body, context.policy, hit.point_b, contact_normal, -hit.penetration,
                            hit.sub_shape_id_b, hit.material_id_b);
    contact->fraction = 0.0f;
    contact->had_collision = false;
    contact->was_discarded = false;
    contact->can_push_character = true;
}

static bool character_collide_should_early_out(const collide_shape::CollideShapeCollector &collector) {
    return collector.early_out_fraction <= -scalar::FLOAT_MAX;
}

/** orders contacts by body id then sub shape id, stable, so the order is the same across frames and runs */
static void sort_contacts(collections::Vec<CharacterContact> &contacts) {
    sort::insertion_sort(contacts.data, contacts.data + contacts.length, [](const CharacterContact &a, const CharacterContact &b) {
        return a.body_id != b.body_id ? a.body_id < b.body_id : a.sub_shape_id < b.sub_shape_id;
    });
}

/**
 * the contacts at a position through the broadphase and narrowphase, sorted for determinism, with the padding taken
 * off their distances. `contacts` is cleared first
 */
static void get_contacts_at_position(KCC &character, World &world, Vec3f position, Vec3f movement_direction,
                                     const filter::Filter &filter, const kcc_policy::Policy *policy, collections::Vec<CharacterContact> &contacts) {
    contacts.length = 0;

    const Transformf transform = shape_transform_at(character, position);

    collide_shape::CollideShapeSettings settings = collide_shape::default_settings();
    settings.collide_with_backfaces = character.back_face_mode == BACK_FACE_COLLIDE;
    // max separation distance for predictive contacts
    settings.max_separation_distance = character.predictive_contact_distance + character.character_padding;
    // active edge movement direction for triangle mesh edge handling
    settings.active_edge_movement_direction = movement_direction;
    settings.collide_only_with_active_edges = true;

    CollideCollectorContext context = {&world, &character, policy, &contacts, character.max_num_hits, false, character.hit_reduction_cos_max_angle};
    collide_shape::CollideShapeCollector collector = collide_shape::create_collector(collector::COLLECT_CUSTOM);
    collector.custom_add_hit = character_collide_add_hit;
    collector.custom_should_early_out = character_collide_should_early_out;
    collector.context = &context;

    IgnoreSingleBodyChainedBodyFilter ignore_inner_body;
    const filter::Filter chained_filter = chain_ignore_single_body(filter, ignore_inner_body, character.inner_rigid_body_id);

    if (character.enhanced_internal_edge_removal) {
        query::collide_shape_with_internal_edge_removal(world, collector, settings, character.shape, transform, chained_filter);
    } else {
        query::collide_shape(world, collector, settings, character.shape, transform, chained_filter);
    }

    sort_contacts(contacts);

    // reduce distance to contact by padding to ensure we stay away from objects
    for (uint32_t i = 0; i < contacts.length; i++) contacts[i].distance -= character.character_padding;
}

// the cast collector: the earliest accepted hit along a sweep

struct CastCollectorContext {
    World *world;
    const KCC *character;
    const kcc_policy::Policy *policy;
    const collections::Vec<IgnoredContact> *ignored_contacts;
    Vec3f displacement;
    CharacterContact *out_contact;
    bool has_hit;
};

static void character_cast_add_hit(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeHit &hit) {
    CastCollectorContext &context = *static_cast<CastCollectorContext *>(collector.context);

    if (hit.status != cast_shape::COLLIDING) return;

    // only accept hits earlier than current best
    if (hit.fraction >= collector.early_out_fraction) return;

    // ignore collisions at fraction = 0, these are already-penetrating contacts the collide collector handles
    if (hit.fraction <= 0.0f) return;

    // ignore penetrations we're moving away from
    if (vec3f::dot(hit.normal, context.displacement) >= 0.0f) return;

    // test if this contact should be ignored
    if (context.ignored_contacts != nullptr) {
        const collections::Vec<IgnoredContact> &ignored = *context.ignored_contacts;
        for (uint32_t i = 0; i < ignored.length; i++) {
            if (ignored[i].body_id == hit.body_id_b && ignored[i].sub_shape_id == hit.sub_shape_id_b) return;
        }
    }

    const RigidBody *body = rigid_body::get(*context.world, hit.body_id_b);
    if (body == nullptr) return;

    // sensors don't affect character movement, only detection
    if (body->sensor) return;

    CharacterContact contact;
    reset_contact(contact);
    fill_contact_properties(contact, *context.character, *body, context.policy, hit.point_b, hit.normal, -hit.penetration_depth,
                            hit.sub_shape_id_b, hit.material_id_b);
    contact.fraction = hit.fraction;
    contact.had_collision = true;
    contact.was_discarded = false;
    contact.can_push_character = true;

    // only accept contacts that will cause significant penetration and that the policy validates: current distance
    // plus how far we move into the surface; a grazing contact is ignored
    const float predicted_penetration = contact.distance + vec3f::dot(contact.contact_normal, context.displacement);
    if (predicted_penetration < -context.character->collision_tolerance &&
        validate_contact(*context.world, contact, context.policy)) {
        *context.out_contact = contact;
        context.has_hit = true;
        collector.early_out_fraction = hit.fraction;
    }
}

static bool character_cast_should_early_out(const cast_shape::CastShapeCollector &collector) {
    // fraction 0 hits are rejected and the bound only ever drops to a positive fraction, so the cast runs to the end
    return collector.early_out_fraction <= -scalar::FLOAT_MAX;
}

/** what the padding correction casts with */
constexpr float PADDING_CORRECTION_TOLERANCE = 1e-4f;

/**
 * corrects the fraction for the character padding by casting the character's convex leaves (through transformed,
 * scaled and compound shapes) against the inflated polygon. true if a cast hit, with the fraction lowered to it
 */
static bool correct_fraction_for_character_padding(support::Support &character_support, const shapes::Shape *shape, const Transformf &transform,
                                                   Vec3f displacement, support::Support &polygon, float &io_fraction) {
    switch (shape->type) {
        case shapes::SHAPE_TRANSFORMED: {
            Transformf inner;
            Vec3f translation = vec3f::multiply(vec3f::load(shape->transformed.position), vec3f::load(transform.scale));
            translation = vec3f::transform_quat(translation, quatf::load(transform.quaternion));
            vec3f::store(inner.position, vec3f::add(vec3f::load(transform.position), translation));
            quatf::store(inner.quaternion, quatf::multiply(quatf::load(transform.quaternion), quatf::load(shape->transformed.quaternion)));
            vec3f::store(inner.scale, shapes::transform_scale(quatf::load(shape->transformed.quaternion), vec3f::load(transform.scale)));
            return correct_fraction_for_character_padding(character_support, shape->transformed.shape, inner, displacement, polygon, io_fraction);
        }
        case shapes::SHAPE_SCALED: {
            Transformf inner = transform;
            vec3f::store(inner.scale, vec3f::multiply(vec3f::load(transform.scale), vec3f::load(shape->scaled.scale)));
            return correct_fraction_for_character_padding(character_support, shape->scaled.shape, inner, displacement, polygon, io_fraction);
        }
        case shapes::SHAPE_COMPOUND:
        case shapes::SHAPE_STATIC_COMPOUND: {
            const collections::Vec<compound::CompoundShapeChild> &children = compound::children_of(shape);
            bool corrected = false;
            for (uint32_t i = 0; i < children.length; i++) {
                Transformf child;
                compound::child_transform(child, transform, children[i]);
                corrected |= correct_fraction_for_character_padding(character_support, children[i].shape, child, displacement, polygon,
                                                                    io_fraction);
            }
            return corrected;
        }
        default:
            break;
    }

    if (!shapes::is_convex(shape)) return false;

    // at the convex level, cast the shape (its convex radius included) against the polygon (its padding added by its
    // support), so neither has a radius left for the cast to keep apart
    shapes::set_shape_support(character_support, shape, support::INCLUDE_CONVEX_RADIUS, vec3f::load(transform.scale));

    const Mat4f matrix = mat4f::from_rotation_translation(quatf::load(transform.quaternion), vec3f::load(transform.position));

    gjk::GjkCastShapeResult result = gjk::create_gjk_cast_shape_result();
    gjk::cast_shape(result, matrix, character_support, polygon, displacement, PADDING_CORRECTION_TOLERANCE, 0.0f, 0.0f, io_fraction);

    if (result.hit) {
        io_fraction = result.lambda;
        return true;
    }
    return false;
}

/** sweep along the displacement; true with the earliest accepted contact in `out`, false if the path is clear */
static bool get_first_contact_for_sweep(CharacterContact &out, KCC &character, World &world, Vec3f position, Vec3f displacement,
                                        const filter::Filter &filter, const kcc_policy::Policy *policy,
                                        const collections::Vec<IgnoredContact> *ignored_contacts) {
    const float displacement_len_sq = vec3f::squared_length(displacement);
    if (displacement_len_sq < 1.0e-8f) return false;

    reset_contact(out);

    const Transformf transform = shape_transform_at(character, position);

    // how much extra fraction we need to add to the cast to account for the character padding
    const float character_padding_fraction = character.character_padding / scalar::sqrt(displacement_len_sq);

    cast_shape::CastShapeSettings settings = cast_shape::default_settings();
    settings.collision_tolerance = character.collision_tolerance;
    settings.collide_with_backfaces_triangles = character.back_face_mode == BACK_FACE_COLLIDE;
    settings.collide_with_backfaces_convex = false;
    settings.collide_only_with_active_edges = true;
    settings.use_shrunken_shape_and_convex_radius = true;
    settings.return_deepest_point = false;

    CastCollectorContext context = {&world, &character, policy, ignored_contacts, displacement, &out, false};
    cast_shape::CastShapeCollector collector = cast_shape::create_collector(collector::COLLECT_CUSTOM);
    collector.custom_add_hit = character_cast_add_hit;
    collector.custom_should_early_out = character_cast_should_early_out;
    collector.context = &context;
    // allow the cast beyond 1 for the padding correction
    collector.early_out_fraction = 1.0f + character_padding_fraction;

    IgnoreSingleBodyChainedBodyFilter ignore_inner_body;
    const filter::Filter chained_filter = chain_ignore_single_body(filter, ignore_inner_body, character.inner_rigid_body_id);

    query::cast_shape(world, collector, settings, character.shape, transform, displacement, chained_filter);

    if (!context.has_hit) return false;

    // correct the fraction so the padding, not the shape, touches the face hit
    bool corrected = false;
    const RigidBody *hit_body = rigid_body::get(world, out.body_id);
    if (hit_body != nullptr) {
        // the supporting face against the contact normal, the direction in the body's local space
        Vec3f direction = vec3f::negate(out.contact_normal);
        Quatf inverse_rotation = quatf::conjugate(quatf::load(hit_body->quaternion));
        direction = vec3f::transform_quat(direction, inverse_rotation);

        const Mat4f hit_body_transform = mat4f::from_rotation_translation(quatf::load(hit_body->quaternion), vec3f::load(hit_body->position));

        face::Face hit_face;
        const Vec3f unit_scale = vec3f::make(1.0f, 1.0f, 1.0f);
        shapes::get_shape_supporting_face(hit_face, hit_body->shape, out.sub_shape_id, direction, hit_body_transform, unit_scale);

        // only with a face of 2+ vertices
        if (hit_face.num_vertices >= 2) {
            // the face inflated by the character padding
            support::Support polygon = support::create();
            support::set_polygon_support(polygon, hit_face.vertices, hit_face.num_vertices, 0.0f);
            polygon.add_radius = character.character_padding;

            support::Support character_support = support::create();
            float fraction = out.fraction + character_padding_fraction;
            corrected = correct_fraction_for_character_padding(character_support, character.shape, transform, displacement, polygon, fraction);
            if (corrected) out.fraction = fraction;
        }
    }

    // fallback: move the fraction back by the padding
    if (!corrected) out.fraction = scalar::max(0.0f, out.fraction - character_padding_fraction);

    out.fraction = scalar::min(out.fraction, 1.0f);

    return true;
}

/**
 * discards one of each pair of contacts on the same body whose penetration normals oppose, the shallower one, so the
 * character is not stuck between them. only past 1.25 x padding, as a little penetration is always numerical
 */
static void remove_conflicting_contacts(collections::Vec<CharacterContact> &contacts, float character_padding, collections::Vec<IgnoredContact> &out_ignored) {
    out_ignored.length = 0;

    const float min_required_penetration = 1.25f * character_padding;

    const uint32_t n = contacts.length;
    for (uint32_t i = 0; i < n; i++) {
        CharacterContact &contact_a = contacts[i];
        if (contact_a.was_discarded) continue;
        if (contact_a.distance > -min_required_penetration) continue;

        for (uint32_t j = i + 1; j < n; j++) {
            CharacterContact &contact_b = contacts[j];
            if (contact_b.was_discarded) continue;
            if (contact_b.distance > -min_required_penetration) continue;

            // only the same body: this targets one body's internal-edge double hits. opposing contacts from two
            // different bodies (a real wedge) are both legitimate
            if (contact_a.body_id != contact_b.body_id) continue;

            if (vec3f::dot(contact_a.contact_normal, contact_b.contact_normal) < 0.0f) {
                if (contact_a.distance > contact_b.distance) {
                    // a has smaller penetration, discard it
                    contact_a.was_discarded = true;
                    out_ignored.push({contact_a.body_id, contact_a.sub_shape_id});
                    break;
                }
                contact_b.was_discarded = true;
                out_ignored.push({contact_b.body_id, contact_b.sub_shape_id});
            }
        }
    }
}

/**
 * contacts to plane constraints: a penetrating contact's velocity pushes the character out, and a steep slope adds a
 * vertical wall that holds the character back
 */
static void determine_constraints(const KCC &character, collections::Vec<CharacterContact> &contacts, float delta_time,
                                  collections::Vec<CharacterConstraint> &constraints) {
    constraints.length = 0;

    const float inv_delta_time = delta_time > 0.0f ? 1.0f / delta_time : 0.0f;

    for (uint32_t i = 0; i < contacts.length; i++) {
        CharacterContact &contact = contacts[i];
        if (contact.was_discarded) continue;

        // penetrating contact: add a contact velocity that pushes the character out at the desired speed
        Vec3f contact_velocity = contact.linear_velocity;
        if (contact.distance < 0.0f) {
            contact_velocity = vec3f::scale_and_add(contact_velocity, contact.contact_normal, -contact.distance * character.penetration_recovery_speed * inv_delta_time);
        }

        CharacterConstraint constraint;
        constraint.contact = &contact;
        constraint.toi = 0.0f;
        constraint.projected_velocity = 0.0f;
        constraint.linear_velocity = contact_velocity;
        constraint.plane_normal = contact.contact_normal;
        constraint.plane_distance = contact.distance;
        constraint.is_steep_slope = false;

        // steep slope: an additional vertical wall constraint
        CharacterConstraint vertical_constraint;
        bool has_vertical_constraint = false;
        if (is_slope_too_steep(character, contact.surface_normal)) {
            // only planes that point up; the contact normal allows better sliding than the surface normal
            const float dot_up = vec3f::dot(contact.contact_normal, character.up);
            if (dot_up > 1e-3f) {
                constraint.is_steep_slope = true;

                // horizontal component of normal (remove up component)
                Vec3f horizontal_normal = vec3f::scale_and_add(contact.contact_normal, character.up, -dot_up);
                if (vec3f::squared_length(horizontal_normal) > 1e-6f) {
                    horizontal_normal = vec3f::normalize(horizontal_normal);

                    vertical_constraint.contact = &contact;
                    vertical_constraint.toi = 0.0f;
                    vertical_constraint.projected_velocity = 0.0f;

                    // the original contact velocity (not the push out) projected onto the horizontal normal, so both
                    // planes push at an equal rate; recovery velocity can trap characters surrounded by steep slopes
                    vertical_constraint.linear_velocity = vec3f::scale(horizontal_normal, vec3f::dot(contact.linear_velocity, horizontal_normal));

                    vertical_constraint.plane_normal = horizontal_normal;

                    // distance to travel horizontally to hit the contact plane
                    const float normal_dot_contact = vec3f::dot(horizontal_normal, contact.contact_normal);
                    vertical_constraint.plane_distance =
                        scalar::abs(normal_dot_contact) > 1e-6f ? contact.distance / normal_dot_contact : contact.distance;
                    vertical_constraint.is_steep_slope = true;
                    has_vertical_constraint = true;
                }
            }
        }

        constraints.push(constraint);
        if (has_vertical_constraint) constraints.push(vertical_constraint);
    }
}

/**
 * pushes a dynamic body from a character contact with the effective mass at the contact point, clamped by the
 * character's strength, without the downward part (gravity is applied later)
 */
static void apply_impulse_to_body(World &world, const KCC &character, const CharacterContact &contact, Vec3f character_velocity,
                                  float delta_time) {
    if (contact.motion_type != motion_type::MOTION_DYNAMIC) return;
    if (contact.is_sensor) return;
    if (!contact.can_receive_impulses) return;

    RigidBody *body = rigid_body::get(world, contact.body_id);
    if (body == nullptr) return;

    const motion_properties::MotionProperties &mp = body->motion_properties;
    const float inv_mass = mp.inv_mass;
    if (inv_mass <= 0.0f) return;

    // relative velocity projected onto contact normal
    Vec3f relative_velocity = vec3f::subtract(character_velocity, contact.linear_velocity);
    const float projected_velocity = vec3f::dot(relative_velocity, contact.contact_normal);

    // delta velocity needed: -projected_velocity * damping - penetration correction
    float delta_velocity = -projected_velocity * IMPULSE_DAMPING_FACTOR;
    if (contact.distance < 0.0f && delta_time > 0.0f) {
        delta_velocity -= scalar::min(contact.distance, 0.0f) * IMPULSE_PENETRATION_RESOLUTION_FACTOR / delta_time;
    }

    // don't apply impulses if we're separating
    if (delta_velocity < 0.0f) return;

    // 1/m_eff = 1/m + (r x n) . I^-1 (r x n)
    Vec3f r = vec3f::subtract(contact.position, vec3f::load(body->center_of_mass_position));
    const Vec3f jacobian = vec3f::cross(r, contact.contact_normal);
    const Vec3f inv_inertia_jacobian = motion_properties::multiply_world_space_inverse_inertia_by_vector(mp, quatf::load(body->quaternion), jacobian);
    const float inv_effective_mass = inv_mass + vec3f::dot(jacobian, inv_inertia_jacobian);

    // clamp impulse by max strength (p = f dt)
    const float impulse = scalar::min(delta_velocity / inv_effective_mass, character.max_strength * delta_time);

    Vec3f world_impulse = vec3f::scale(contact.contact_normal, -impulse);

    // cancel impulse in down direction (we apply gravity later)
    const float impulse_dot_up = vec3f::dot(world_impulse, character.up);
    if (impulse_dot_up < 0.0f) world_impulse = vec3f::scale_and_add(world_impulse, character.up, -impulse_dot_up);

    rigid_body::add_impulse_at_position(world, body, world_impulse, contact.position);
}

/**
 * a contact the solver reached: validated, flagged as colliding, reported added or persisted with its settings taken,
 * and a dynamic body pushed by the character's input velocity. false to discard it (rejected, or a sensor)
 */
static bool handle_contact(World &world, KCC &character, CharacterContact &contact, const kcc_policy::Policy *policy, bool track,
                           Vec3f character_velocity, float delta_time) {
    if (!validate_contact(world, contact, policy)) return false;

    contact.had_collision = true;

    CharacterContactSettings settings = {true, true};
    if (track) contact_added(world, character, contact, policy, settings);

    contact.can_push_character = settings.can_push_character;

    // a sensor is reported and pushes nothing
    if (contact.is_sensor) return false;

    contact.can_receive_impulses = settings.can_receive_impulses;

    // before velocity solving: the impulse uses the character's input velocity, not the solved velocity
    if (settings.can_receive_impulses && contact.motion_type == motion_type::MOTION_DYNAMIC) {
        apply_impulse_to_body(world, character, contact, character_velocity, delta_time);
    }

    return true;
}

static void sort_constraints(collections::Vec<CharacterConstraint *> &constraints) {
    sort::insertion_sort(constraints.data, constraints.data + constraints.length, [](const CharacterConstraint *a, const CharacterConstraint *b) {
        return character_constraints::constraint_before(*a, *b);
    });
}

/**
 * moves within the constraints: finds the first constraint hit, moves to it and slides along it (or two of them),
 * until the time is used. velocity is in/out; returns the time simulated
 */
static float solve_constraints(KCC &character, World &world, Vec3f &velocity, float delta_time, float time_remaining,
                               const kcc_policy::Policy *policy, bool track, Vec3f &out_displacement, collections::Vec<IgnoredContact> &io_ignored_contacts,
                               collections::Vec<CharacterConstraint> &constraints) {
    out_displacement = vec3f::make(0.0f, 0.0f, 0.0f);

    if (constraints.length == 0) {
        out_displacement = vec3f::scale_and_add(out_displacement, velocity, time_remaining);
        return time_remaining;
    }

    collections::Vec<CharacterConstraint *> &sorted_constraints = character.sorted_constraints;
    sorted_constraints.length = 0;
    for (uint32_t i = 0; i < constraints.length; i++) sorted_constraints.push(&constraints[i]);

    // last velocity for reversal detection: if velocity reverses we stop to prevent corner sticking
    Vec3f last_velocity = velocity;

    // previous constraints for two-plane intersection
    collections::Vec<CharacterConstraint *> &previous_constraints = character.previous_constraints;
    previous_constraints.length = 0;

    float time_simulated = 0.0f;

    for (uint32_t iteration = 0; iteration < character.max_constraint_iterations; iteration++) {
        for (uint32_t i = 0; i < sorted_constraints.length; i++) {
            CharacterConstraint &constraint = *sorted_constraints[i];
            constraint.toi = character_constraints::calculate_constraint_toi(constraint, velocity, out_displacement, time_remaining);
        }

        sort_constraints(sorted_constraints);

        // the first valid constraint: one we're moving toward within the time remaining
        CharacterConstraint *active_constraint = nullptr;
        bool reached_goal = false;
        for (uint32_t i = 0; i < sorted_constraints.length; i++) {
            CharacterConstraint &constraint = *sorted_constraints[i];

            // we can reach our goal without hitting this constraint
            if (constraint.toi >= time_remaining) {
                out_displacement = vec3f::scale_and_add(out_displacement, velocity, time_remaining);
                time_simulated += time_remaining;
                reached_goal = true;
                break;
            }

            // skip contacts that were previously discarded
            if (constraint.contact->was_discarded) continue;

            // skip if we're not moving toward this constraint
            if (constraint.projected_velocity <= 1e-10f) continue;

            // the policy's validation, had_collision, the added / persisted tracking and settings, impulses
            if (!constraint.contact->had_collision && !handle_contact(world, character, *constraint.contact, policy, track, velocity, delta_time)) {
                constraint.contact->was_discarded = true;
                io_ignored_contacts.push({constraint.contact->body_id, constraint.contact->sub_shape_id});
                continue;
            }

            // cancel velocity of constraint if it cannot push the character
            if (!constraint.contact->can_push_character) constraint.linear_velocity = vec3f::make(0.0f, 0.0f, 0.0f);

            active_constraint = &constraint;
            break;
        }

        if (reached_goal) break;

        if (active_constraint == nullptr) {
            // all constraints were discarded, we can reach our goal
            out_displacement = vec3f::scale_and_add(out_displacement, velocity, time_remaining);
            time_simulated += time_remaining;
            break;
        }

        const float move_time = scalar::max(0.0f, active_constraint->toi);
        out_displacement = vec3f::scale_and_add(out_displacement, velocity, move_time);
        time_remaining -= move_time;
        time_simulated += move_time;

        // if there's not enough time left to be simulated, bail
        if (time_remaining < character.min_time_remaining) break;

        // if we've moved significantly, clear all previous contacts
        if (move_time > 1e-4f) previous_constraints.length = 0;

        // steep slope: cancel the velocity towards the slope before sliding, so hitting the slope before its vertical
        // wall constraint does not slide us up it
        if (active_constraint->is_steep_slope) {
            // a vertical plane (not normalized) that blocks any further movement up the slope
            const Vec3f vertical_plane_normal = vec3f::scale_and_add(active_constraint->plane_normal, character.up, -vec3f::dot(active_constraint->plane_normal, character.up));

            Vec3f relative_velocity = vec3f::subtract(velocity, active_constraint->linear_velocity);

            // remove velocity towards the slope (only if moving toward it)
            const float rel_vel_dot_vertical = vec3f::dot(relative_velocity, vertical_plane_normal);
            if (rel_vel_dot_vertical < 0.0f) {
                const float vertical_len_sq = vec3f::squared_length(vertical_plane_normal);
                if (vertical_len_sq > 1e-8f) {
                    velocity = vec3f::scale_and_add(velocity, vertical_plane_normal, -rel_vel_dot_vertical / vertical_len_sq);
                }
            }
        }

        Vec3f new_velocity;
        new_velocity = character_constraints::slide_velocity_along_plane(velocity, *active_constraint);

        // the previous constraint we will violate the most, excluding parallel normals
        float highest_penetration = 0.0f;
        CharacterConstraint *other_constraint = nullptr;
        for (uint32_t i = 0; i < previous_constraints.length; i++) {
            CharacterConstraint &previous = *previous_constraints[i];
            if (&previous == active_constraint) continue;

            // how much we will penetrate if we move in this direction
            const float penetration =
                vec3f::dot(previous.linear_velocity, previous.plane_normal) - vec3f::dot(new_velocity, previous.plane_normal);
            if (penetration > highest_penetration) {
                // no parallel or anti-parallel normals, their cross product is zero; slack is about 10 degrees
                const float dot = vec3f::dot(previous.plane_normal, active_constraint->plane_normal);
                if (dot < 0.984f && dot > -0.984f) {
                    highest_penetration = penetration;
                    other_constraint = &previous;
                }
            }

            // cancel each constraint's velocity in the other's plane direction, so we don't ping pong between planes
            previous.linear_velocity = vec3f::scale_and_add(previous.linear_velocity, active_constraint->plane_normal, -scalar::min(0.0f, vec3f::dot(previous.linear_velocity, active_constraint->plane_normal)));
            active_constraint->linear_velocity = vec3f::scale_and_add(active_constraint->linear_velocity, previous.plane_normal, -scalar::min(0.0f, vec3f::dot(active_constraint->linear_velocity, previous.plane_normal)));
        }

        // with a 2nd constraint, slide along the intersection of both planes
        if (other_constraint != nullptr) new_velocity = character_constraints::slide_along_edge(velocity, *active_constraint, *other_constraint);

        // a body the policy keeps from sliding: on a still, walkable surface of it the character stops
        const CharacterContact &active_contact = *active_constraint->contact;
        if (rigid_body::get(world, active_contact.body_id) != nullptr && !kcc_policy::get(policy, active_contact.body_id).slide_on_walkable &&
            vec3f::squared_length(active_contact.linear_velocity) <= 1e-12f && !is_slope_too_steep(character, active_contact.contact_normal)) {
            new_velocity = vec3f::make(0.0f, 0.0f, 0.0f);
        }

        velocity = new_velocity;
        previous_constraints.push(active_constraint);

        // the constraint has no pushing velocity and the character is near still
        if (active_constraint->projected_velocity < 1e-8f && vec3f::squared_length(velocity) < 1e-8f) break;

        // a constraint with velocity becomes the reference; otherwise stop on a velocity reversal
        if (vec3f::squared_length(active_constraint->linear_velocity) >= 1e-8f) {
            last_velocity = active_constraint->linear_velocity;
        } else if (vec3f::dot(velocity, last_velocity) < 0.0f) {
            break;
        }
    }

    return time_simulated;
}

/** signed distance from a character space point to the supporting volume plane; positive is a side hit, not supporting */
static float supporting_volume_signed_distance(Vec3f point, Vec4f plane) {
    return vec3f::x(point) * vec4f::x(plane) + vec3f::y(point) * vec4f::y(plane) + vec3f::z(point) * vec4f::z(plane) + vec4f::w(plane);
}

/**
 * the ground state from the contacts: colliding contacts flagged, then supporting or sliding by the supporting volume
 * and the slope, the averaged ground normal and velocity, and for steep slopes only whether they hold the character
 */
static void update_supporting_contact(KCC &character, World &world, bool skip_contact_velocity_check, float last_delta_time,
                                      const kcc_policy::Policy *policy) {
    collections::Vec<CharacterContact> &contacts = character.contacts;

    // flag contacts as colliding if close enough, unless we move away from them (contacts move_shape marked stay)
    for (uint32_t i = 0; i < contacts.length; i++) {
        CharacterContact &contact = contacts[i];
        if (contact.was_discarded) continue;
        if (contact.had_collision) continue;
        if (contact.distance >= character.collision_tolerance) continue;

        if (!skip_contact_velocity_check) {
            Vec3f relative_velocity = vec3f::subtract(character.linear_velocity, contact.linear_velocity);
            if (vec3f::dot(contact.surface_normal, relative_velocity) > 1e-4f) continue;
        }

        // the settings are discarded, can_push_character keeps its value
        if (validate_contact(world, contact, policy)) {
            CharacterContactSettings dummy_settings = {true, true};
            contact_added(world, character, contact, policy, dummy_settings);
            contact.had_collision = true;
        } else {
            contact.was_discarded = true;
        }
    }

    // contact positions into character space, to test them against the supporting volume
    Quatf inverse_rotation = quatf::conjugate(character.quaternion);

    uint32_t num_supported = 0;
    uint32_t num_sliding = 0;
    uint32_t num_avg_normal = 0;
    Vec3f avg_normal = vec3f::make(0.0f, 0.0f, 0.0f);
    Vec3f avg_velocity = vec3f::make(0.0f, 0.0f, 0.0f);

    const CharacterContact *supporting_contact = nullptr;
    float max_cos_angle = -scalar::FLOAT_MAX;
    const CharacterContact *deepest_contact = nullptr;
    float smallest_distance = scalar::FLOAT_MAX;

    for (uint32_t i = 0; i < contacts.length; i++) {
        const CharacterContact &contact = contacts[i];
        if (!contact.had_collision || contact.was_discarded) continue;

        const float cos_angle = vec3f::dot(contact.surface_normal, character.up);

        if (contact.distance < smallest_distance) {
            deepest_contact = &contact;
            smallest_distance = contact.distance;
        }

        // a contact in front of the supporting plane cannot support us (the shape offset is not subtracted)
        Vec3f local_position = vec3f::subtract(contact.position, character.position);
        local_position = vec3f::transform_quat(local_position, inverse_rotation);
        if (supporting_volume_signed_distance(local_position, character.supporting_volume_plane) > 0.0f) continue;

        // the contact with the normal pointing most upward
        if (cos_angle > max_cos_angle) {
            supporting_contact = &contact;
            max_cos_angle = cos_angle;
        }

        const bool supported = character.cos_max_slope_angle > NO_MAX_SLOPE_ANGLE || cos_angle >= character.cos_max_slope_angle;
        if (supported) num_supported++;
        else num_sliding++;

        // contacts within 85 degrees of up average the normal and velocity
        if (cos_angle >= 0.08f) {
            avg_normal = vec3f::add(avg_normal, contact.surface_normal);
            num_avg_normal++;

            // a kinematic supporting contact gives the velocity at our position, so a rotating platform is followed;
            // anything else its contact velocity
            if (contact.motion_type != motion_type::MOTION_KINEMATIC || !supported) {
                avg_velocity = vec3f::add(avg_velocity, contact.linear_velocity);
            } else {
                const RigidBody *body = rigid_body::get(world, contact.body_id);
                if (body != nullptr) {
                    Vec3f linear_velocity, angular_velocity;
                    get_adjusted_body_velocity(*body, policy, linear_velocity, angular_velocity);
                    avg_velocity = vec3f::add(avg_velocity, calculate_character_ground_velocity(character, vec3f::load(body->center_of_mass_position),
                                                                                                linear_velocity, angular_velocity, last_delta_time));
                } else {
                    avg_velocity = vec3f::add(avg_velocity, contact.linear_velocity);
                }
            }
        }
    }

    // prefer the supporting contact, fall back to the deepest
    const CharacterContact *best_contact = supporting_contact != nullptr ? supporting_contact : deepest_contact;

    GroundInfo &ground = character.ground;

    if (num_avg_normal >= 1) {
        ground.normal = vec3f::normalize(avg_normal);
        ground.velocity = vec3f::scale(avg_velocity, 1.0f / (float)num_avg_normal);
    } else if (best_contact != nullptr) {
        ground.normal = best_contact->surface_normal;
        ground.velocity = best_contact->linear_velocity;
    } else {
        ground.normal = vec3f::make(0.0f, 0.0f, 0.0f);
        ground.velocity = vec3f::make(0.0f, 0.0f, 0.0f);
    }

    if (best_contact != nullptr) {
        ground.body_id = best_contact->body_id;
        ground.sub_shape_id = best_contact->sub_shape_id;
        ground.position = best_contact->position;
    } else {
        ground.body_id = body_id::INVALID_BODY_ID;
        ground.sub_shape_id = 0;
        ground.position = vec3f::make(0.0f, 0.0f, 0.0f);
    }

    if (num_supported > 0) {
        ground.state = ON_GROUND;
    } else if (num_sliding > 0) {
        // only steep slopes: we cannot be on ground moving upward relative to it
        Vec3f relative_velocity = vec3f::subtract(character.linear_velocity, deepest_contact->linear_velocity);
        if (vec3f::dot(relative_velocity, character.up) > 1e-4f) {
            ground.state = ON_STEEP_GROUND;
        } else {
            // sliding down, we may stand on several sliding contacts in a way we can't slide off, which also supports
            // us. the probe runs on a copy of the contacts: the solver flips their flags, and the live flags feed
            // finalize_contact_tracking and has_collided_with after this
            collections::Vec<CharacterContact> &probe_contacts = character.supporting_probe_contacts;
            probe_contacts.length = 0;
            for (uint32_t i = 0; i < contacts.length; i++) probe_contacts.push(contacts[i]);

            determine_constraints(character, probe_contacts, last_delta_time, character.constraints);

            // solve with -up velocity over unit time: do we move at all when falling?
            Vec3f down_velocity = vec3f::negate(character.up);
            Vec3f displacement;
            character.supporting_ignored_contacts.length = 0;
            // a probe: no policy and no tracking, so nothing it meets is reported
            const float time_simulated = solve_constraints(character, world, down_velocity, 1.0f, 1.0f, nullptr, false, displacement,
                                                           character.supporting_ignored_contacts, character.constraints);

            character.constraints.length = 0;
            probe_contacts.length = 0;

            // blocked is supported, otherwise sliding
            const float min_required_displacement = 0.6f * last_delta_time;
            if (time_simulated < 0.001f || vec3f::squared_length(displacement) < min_required_displacement * min_required_displacement) {
                ground.state = ON_GROUND;
            } else {
                ground.state = ON_STEEP_GROUND;
            }
        }
    } else {
        // not supported by anything
        ground.state = best_contact != nullptr ? NOT_SUPPORTED : IN_AIR;
    }
}

/** the desired velocity without its horizontal part into steep slopes we collide with, unless on ground or in air */
static Vec3f cancel_velocity_towards_steep_slopes(const KCC &character, Vec3f desired_velocity) {
    Vec3f velocity = desired_velocity;

    // on ground or in air nothing is cancelled (walk_stairs may override the ground state)
    if (character.ground.state == ON_GROUND || character.ground.state == IN_AIR) return velocity;

    const collections::Vec<CharacterContact> &contacts = character.contacts;
    for (uint32_t i = 0; i < contacts.length; i++) {
        const CharacterContact &contact = contacts[i];
        if (!contact.had_collision || contact.was_discarded) continue;
        if (!is_slope_too_steep(character, contact.surface_normal)) continue;

        // the contact normal slides better (the surface normal may point away from the movement), made horizontal
        Vec3f normal = vec3f::scale_and_add(contact.contact_normal, character.up, -vec3f::dot(contact.contact_normal, character.up));

        // cancel horizontal movement toward the slope
        const float dot = vec3f::dot(normal, velocity);
        if (dot < 0.0f) {
            const float len_sq = vec3f::squared_length(normal);
            if (len_sq > 1e-12f) velocity = vec3f::scale_and_add(velocity, normal, -dot / len_sq);
        }
    }
    return velocity;
}

/** supported and pushing horizontally into a steep slope we collide with */
static bool has_steep_slopes_to_walk(const KCC &character, Vec3f linear_velocity) {
    if (!is_supported(character)) return false;

    Vec3f horizontal_velocity = vec3f::scale_and_add(linear_velocity, character.up, -vec3f::dot(linear_velocity, character.up));
    if (vec3f::squared_length(horizontal_velocity) < 1e-12f) return false;

    const collections::Vec<CharacterContact> &contacts = character.contacts;
    for (uint32_t i = 0; i < contacts.length; i++) {
        const CharacterContact &contact = contacts[i];
        if (!contact.had_collision || contact.was_discarded) continue;
        if (!is_slope_too_steep(character, contact.surface_normal)) continue;

        Vec3f relative_velocity = vec3f::subtract(horizontal_velocity, contact.linear_velocity);
        if (vec3f::dot(contact.surface_normal, relative_velocity) < 0.0f) return true;
    }
    return false;
}

/**
 * slides the shape from `position` (in/out) by the velocity: per collision iteration, the contacts, the conflicting
 * ones removed, constraints, the solve, and a sweep that cuts the displacement short at a hit. `out_contacts` is the
 * character's contacts for real movement, scratch for probes
 */
static void move_shape(KCC &character, World &world, Vec3f &position, Vec3f velocity, float delta_time, const filter::Filter &filter,
                       const kcc_policy::Policy *policy, collections::Vec<CharacterContact> &out_contacts) {
    // movement direction for active edge handling
    Vec3f movement_direction = vec3f::make(0.0f, 0.0f, 0.0f);
    const float velocity_len_sq = vec3f::squared_length(velocity);
    if (velocity_len_sq > 1e-12f) movement_direction = vec3f::scale(velocity, 1.0f / scalar::sqrt(velocity_len_sq));

    // the solver changes it
    Vec3f solve_velocity = velocity;

    float time_remaining = delta_time;

    for (uint32_t iteration = 0; iteration < character.max_collision_iterations; iteration++) {
        if (time_remaining < character.min_time_remaining) break;

        get_contacts_at_position(character, world, position, movement_direction, filter, policy, out_contacts);

        remove_conflicting_contacts(out_contacts, character.character_padding, character.ignored_contacts);

        determine_constraints(character, out_contacts, delta_time, character.constraints);

        Vec3f displacement;
        float time_simulated = solve_constraints(character, world, solve_velocity, delta_time, time_remaining, policy, true, displacement,
                                                 character.ignored_contacts, character.constraints);

        // a sweep to test if the path is really unobstructed; a hit limits the displacement and the time, and is
        // found by the next iteration's contacts
        CharacterContact sweep_contact;
        if (get_first_contact_for_sweep(sweep_contact, character, world, position, displacement, filter, policy, &character.ignored_contacts)) {
            displacement = vec3f::scale(displacement, sweep_contact.fraction);
            time_simulated *= sweep_contact.fraction;
        }

        position = vec3f::add(position, displacement);
        time_remaining -= time_simulated;

        // displacement too small, we're stuck
        if (vec3f::squared_length(displacement) < 1e-8f) break;
    }

    character.constraints.length = 0;
}

/** starts a frame's contact tracking: every colliding contact becomes an entry not yet seen */
static void reset_contact_tracking(KCC &character) {
    character.contact_events.length = 0;
    const collections::Vec<CharacterContact> &contacts = character.contacts;
    for (uint32_t i = 0; i < contacts.length; i++) {
        if (contacts[i].had_collision) acquire_tracked_contact(character, contacts[i].body_id, contacts[i].sub_shape_id);
    }
}

/** ends a frame's contact tracking: entries not seen this frame are reported removed, with the body id */
static void finalize_contact_tracking(KCC &character) {
    // re-mark all contacts - reset all counts to 0, then mark active ones as 1
    collections::Vec<TrackedContact> &live = character.tracked_contacts;
    for (uint32_t i = 0; i < live.length; i++) live[i].count = 0;

    const collections::Vec<CharacterContact> &contacts = character.contacts;
    for (uint32_t i = 0; i < contacts.length; i++) {
        if (!contacts[i].had_collision) continue;
        TrackedContact *tracked = find_tracked_contact(character, contacts[i].body_id, contacts[i].sub_shape_id);
        if (tracked != nullptr) tracked->count = 1;
    }

    // unconditionally, by id: the body may already be removed, and the removal must still be reported
    for (uint32_t i = 0; i < live.length; i++) {
        if (live[i].count == 0) {
            character.contact_events.push({CONTACT_REMOVED, live[i].body_id, live[i].sub_shape_id, vec3f::make(0.0f, 0.0f, 0.0f),
                                           vec3f::make(0.0f, 0.0f, 0.0f)});
        }
    }

    live.length = 0;
}

/** the inner body follows the character */
static void update_inner_body_transform(KCC &character, World &world) {
    if (character.inner_rigid_body_id == body_id::INVALID_BODY_ID) return;

    RigidBody *inner_rigid_body = rigid_body::get(world, character.inner_rigid_body_id);
    if (inner_rigid_body == nullptr) return;

    rigid_body::set_transform(world, inner_rigid_body, get_inner_body_position(character), character.quaternion, false);
}

void set_position(World &world, KCC &character, Vec3f position) {
    character.position = position;
    update_inner_body_transform(character, world);
}

void set_quaternion(World &world, KCC &character, Quatf quaternion) {
    character.quaternion = quaternion;
    update_inner_body_transform(character, world);
}

void update_ground_velocity(World &world, KCC &character, const kcc_policy::Policy *policy) {
    if (character.ground.body_id == body_id::INVALID_BODY_ID) return;

    const RigidBody *ground_body = rigid_body::get(world, character.ground.body_id);
    if (ground_body == nullptr) {
        // body was removed - clear ground info
        character.ground.body_id = body_id::INVALID_BODY_ID;
        character.ground.velocity = vec3f::make(0.0f, 0.0f, 0.0f);
        return;
    }

    Vec3f linear_velocity, angular_velocity;
    get_adjusted_body_velocity(*ground_body, policy, linear_velocity, angular_velocity);
    character.ground.velocity = calculate_character_ground_velocity(character, vec3f::load(ground_body->center_of_mass_position), linear_velocity,
                                                                    angular_velocity, character.last_delta_time);
}

/** the unit velocity direction, or zero when still */
static Vec3f movement_direction_of(Vec3f velocity) {
    Vec3f direction = vec3f::make(0.0f, 0.0f, 0.0f);
    const float velocity_len_sq = vec3f::squared_length(velocity);
    if (velocity_len_sq > 1e-12f) direction = vec3f::scale(velocity, 1.0f / scalar::sqrt(velocity_len_sq));
    return direction;
}

void refresh_contacts(World &world, KCC &character, const filter::Filter &filter, const kcc_policy::Policy *policy) {
    reset_contact_tracking(character);

    get_contacts_at_position(character, world, character.position, movement_direction_of(character.linear_velocity), filter, policy,
                             character.contacts);

    update_supporting_contact(character, world, true, character.last_delta_time, policy);

    finalize_contact_tracking(character);
}

bool has_collided_with(const KCC &character, body_id::BodyId id) {
    const collections::Vec<CharacterContact> &contacts = character.contacts;
    for (uint32_t i = 0; i < contacts.length; i++) {
        if (contacts[i].had_collision && contacts[i].body_id == id) return true;
    }
    return false;
}

bool can_walk_stairs(const KCC &character, Vec3f linear_velocity) { return has_steep_slopes_to_walk(character, linear_velocity); }

bool set_shape(World &world, KCC &character, shapes::Shape *new_shape, const filter::Filter &filter, const kcc_policy::Policy *policy,
               float max_penetration_depth) {
    if (character.shape == new_shape) return true;

    if (max_penetration_depth < scalar::FLOAT_MAX) {
        // the new shape's contacts, into scratch
        collections::Vec<CharacterContact> &new_contacts = character.set_shape_contacts;
        shapes::Shape *old_shape = character.shape;
        character.shape = new_shape;
        get_contacts_at_position(character, world, character.position, movement_direction_of(character.linear_velocity), filter, policy,
                                 new_contacts);
        character.shape = old_shape;

        // reject if penetrating deeper than the threshold and not a sensor
        for (uint32_t i = 0; i < new_contacts.length; i++) {
            if (new_contacts[i].distance < -max_penetration_depth && !new_contacts[i].is_sensor) {
                new_contacts.length = 0;
                return false;
            }
        }

        // the change is valid: track the contact changes over the new contacts
        reset_contact_tracking(character);
        character.shape = new_shape;
        const collections::Vec<CharacterContact> old_contacts = character.contacts;
        character.contacts = new_contacts;
        character.set_shape_contacts = old_contacts;
        character.set_shape_contacts.length = 0;
        update_supporting_contact(character, world, true, character.last_delta_time, policy);
        finalize_contact_tracking(character);
    } else {
        character.shape = new_shape;
    }

    if (character.inner_rigid_body_id != body_id::INVALID_BODY_ID) {
        RigidBody *inner_body = rigid_body::get(world, character.inner_rigid_body_id);
        if (inner_body != nullptr) {
            rigid_body::set_shape(world, inner_body, new_shape);
        }
    }

    return true;
}

void move(World &world, KCC &character, float delta_time, Vec3f gravity, const kcc_policy::Policy *policy, const filter::Filter &filter) {
    if (delta_time <= 0.0f) return;

    // for the supporting contact velocity check
    character.last_delta_time = delta_time;

    move_shape(character, world, character.position, character.linear_velocity, delta_time, filter, policy, character.contacts);

    update_supporting_contact(character, world, false, delta_time, policy);

    update_inner_body_transform(character, world);

    // the character's weight on a dynamic ground body: p = f dt = m g dt, along the normal
    if (character.ground.body_id != body_id::INVALID_BODY_ID && character.mass > 0.0f) {
        RigidBody *ground_body = rigid_body::get(world, character.ground.body_id);
        if (ground_body != nullptr && ground_body->motion_type == motion_type::MOTION_DYNAMIC) {
            const float normal_dot_gravity = vec3f::dot(character.ground.normal, gravity);
            if (normal_dot_gravity < 0.0f) {
                Vec3f impulse = vec3f::scale(gravity, -(character.mass * normal_dot_gravity / vec3f::length(gravity) * delta_time));
                rigid_body::add_impulse_at_position(world, ground_body, impulse, character.ground.position);
            }
        }
    }
}

/** moves to the position of a sweep contact: the contact reported, the contacts there with it marked colliding, the ground state */
static void move_to_contact(KCC &character, World &world, Vec3f position, const CharacterContact &contact, const filter::Filter &filter,
                            const kcc_policy::Policy *policy) {
    set_position(world, character, position);

    CharacterContactSettings settings = {true, true};
    contact_added(world, character, contact, policy, settings);

    get_contacts_at_position(character, world, character.position, movement_direction_of(character.linear_velocity), filter, policy,
                             character.contacts);

    // ensure the contact we moved to is marked as colliding
    collections::Vec<CharacterContact> &contacts = character.contacts;
    bool found_contact = false;
    for (uint32_t i = 0; i < contacts.length; i++) {
        if (contacts[i].body_id == contact.body_id && contacts[i].sub_shape_id == contact.sub_shape_id) {
            contacts[i].had_collision = true;
            found_contact = true;
        }
    }

    // not found at the new position, add it
    if (!found_contact && contacts.push(contact)) {
        CharacterContact &added = contacts[contacts.length - 1];
        added.had_collision = true;
        added.was_discarded = false;
    }

    update_supporting_contact(character, world, true, character.last_delta_time, policy);

    // ensure that the rigid body ends up at the new position
    update_inner_body_transform(character, world);
}

bool stick_to_floor(World &world, KCC &character, Vec3f step_down, const filter::Filter &filter, const kcc_policy::Policy *policy) {
    CharacterContact contact;
    if (!get_first_contact_for_sweep(contact, character, world, character.position, step_down, filter, policy, nullptr)) return false;

    Vec3f new_position = vec3f::scale_and_add(character.position, step_down, contact.fraction);

    move_to_contact(character, world, new_position, contact, filter, policy);
    return true;
}

bool walk_stairs(World &world, KCC &character, float delta_time, Vec3f step_up, Vec3f step_forward, Vec3f step_forward_test,
                 Vec3f step_down_extra, const filter::Filter &filter, const kcc_policy::Policy *policy) {
    Vec3f up = step_up;
    CharacterContact contact;
    if (get_first_contact_for_sweep(contact, character, world, character.position, up, filter, policy, nullptr)) {
        if (contact.fraction < 1e-6f) return false;
        // limit up movement to first contact
        up = vec3f::scale(up, contact.fraction);
    }
    const Vec3f up_position = vec3f::add(character.position, up);

    // the steep slopes we're pushing against, collected before the probe changes the contacts
    Vec3f character_velocity = vec3f::scale(step_forward, 1.0f / delta_time);

    Vec3f horizontal_velocity = vec3f::scale_and_add(character_velocity, character.up, -vec3f::dot(character_velocity, character.up));

    Vec3f steep_slope_normals[WALK_STAIRS_MAX_STEEP_SLOPE_NORMALS];
    uint32_t steep_slope_normals_count = 0;
    const collections::Vec<CharacterContact> &contacts = character.contacts;
    for (uint32_t i = 0; i < contacts.length; i++) {
        const CharacterContact &c = contacts[i];
        if (!c.had_collision || c.was_discarded) continue;
        if (!is_slope_too_steep(character, c.surface_normal)) continue;

        Vec3f relative_velocity = vec3f::subtract(horizontal_velocity, c.linear_velocity);
        if (vec3f::dot(c.surface_normal, relative_velocity) < 0.0f && steep_slope_normals_count < WALK_STAIRS_MAX_STEEP_SLOPE_NORMALS) {
            steep_slope_normals[steep_slope_normals_count++] = c.surface_normal;
        }
    }

    if (steep_slope_normals_count == 0) return false;

    // horizontal movement at the elevated position. a probe: its contacts go to scratch, so on any early return the
    // character's contacts remain the real post-update set
    Vec3f new_position = up_position;
    move_shape(character, world, new_position, character_velocity, delta_time, filter, policy, character.walk_stairs_probe_contacts);
    character.walk_stairs_probe_contacts.length = 0;

    Vec3f horizontal_movement = vec3f::subtract(new_position, up_position);
    const float horizontal_movement_sq = vec3f::squared_length(horizontal_movement);
    if (horizontal_movement_sq < 1e-8f) return false;

    // we need to have moved at least 5% of the step forward against a steep slope, else we're just sliding along it
    const float max_dot = -0.05f * vec3f::length(step_forward);
    bool made_progress = false;
    for (uint32_t i = 0; i < steep_slope_normals_count; i++) {
        if (vec3f::dot(steep_slope_normals[i], horizontal_movement) < max_dot) {
            made_progress = true;
            break;
        }
    }
    if (!made_progress) return false;

    // move down toward the floor: the same distance down as we went up, plus extra
    Vec3f down = vec3f::negate(up);
    down = vec3f::add(down, step_down_extra);

    if (!get_first_contact_for_sweep(contact, character, world, new_position, down, filter, policy, nullptr)) return false;

    // the floor must be walkable
    if (is_slope_too_steep(character, contact.surface_normal)) {
        // too steep - try the forward test, if there is one
        if (vec3f::squared_length(step_forward_test) < 1e-12f) return false;

        // move further forward to test the floor, a probe as above
        Vec3f test_position = up_position;
        character_velocity = vec3f::scale(step_forward_test, 1.0f / delta_time);
        move_shape(character, world, test_position, character_velocity, delta_time, filter, policy, character.walk_stairs_probe_contacts);
        character.walk_stairs_probe_contacts.length = 0;

        horizontal_movement = vec3f::subtract(test_position, up_position);
        if (vec3f::squared_length(horizontal_movement) <= horizontal_movement_sq + 1e-8f) return false;

        CharacterContact test_contact;
        if (!get_first_contact_for_sweep(test_contact, character, world, test_position, down, filter, policy, nullptr)) return false;

        if (is_slope_too_steep(character, test_contact.surface_normal)) return false;
    }

    new_position = vec3f::scale_and_add(new_position, down, contact.fraction);
    move_to_contact(character, world, new_position, contact, filter, policy);

    // the contact normal may have been too steep where the forward test found a walkable floor
    character.ground.state = ON_GROUND;

    return true;
}

void update(World &world, KCC &character, float delta_time, Vec3f gravity, const UpdateSettings &settings, const kcc_policy::Policy *policy,
            const filter::Filter &filter) {
    reset_contact_tracking(character);

    // save desired velocity and cancel velocity toward steep slopes
    const Vec3f desired_velocity = character.linear_velocity;
    character.linear_velocity = cancel_velocity_towards_steep_slopes(character, desired_velocity);

    const Vec3f old_position = character.position;

    // on ground before the update
    bool ground_to_air = is_supported(character);

    move(world, character, delta_time, gravity, policy, filter);

    // ... and in the air after
    if (is_supported(character)) ground_to_air = false;

    // stick to floor going from ground to air, unless moving up
    if (ground_to_air && vec3f::squared_length(vec3f::load(settings.stick_to_floor_step_down)) > 1e-12f) {
        const Vec3f step = vec3f::subtract(character.position, old_position);
        const float velocity_up = vec3f::dot(step, character.up) / delta_time;
        if (velocity_up <= 1e-6f) stick_to_floor(world, character, vec3f::load(settings.stick_to_floor_step_down), filter, policy);
    }

    if (vec3f::squared_length(vec3f::load(settings.walk_stairs_step_up)) > 1e-12f) {
        // how much we wanted to move horizontally
        Vec3f desired_horizontal_step = vec3f::scale(desired_velocity, delta_time);
        desired_horizontal_step = vec3f::scale_and_add(desired_horizontal_step, character.up, -vec3f::dot(desired_horizontal_step, character.up));
        const float desired_horizontal_step_len = vec3f::length(desired_horizontal_step);

        if (desired_horizontal_step_len > 0.0f) {
            // how much we moved horizontally
            Vec3f achieved_horizontal_step = vec3f::subtract(character.position, old_position);
            achieved_horizontal_step = vec3f::scale_and_add(achieved_horizontal_step, character.up, -vec3f::dot(achieved_horizontal_step, character.up));

            // only movement in the desired direction counts (sliding downhill while climbing uphill does not)
            const Vec3f step_forward_normalized = vec3f::scale(desired_horizontal_step, 1.0f / desired_horizontal_step_len);
            const float achieved_in_desired_direction = scalar::max(0.0f, vec3f::dot(achieved_horizontal_step, step_forward_normalized));
            achieved_horizontal_step = vec3f::scale(step_forward_normalized, achieved_in_desired_direction);
            const float achieved_horizontal_step_len = vec3f::length(achieved_horizontal_step);

            // we didn't move as far as we wanted and we're against a slope that's too steep
            if (achieved_horizontal_step_len + 1e-4f < desired_horizontal_step_len && has_steep_slopes_to_walk(character, desired_velocity)) {
                // clamped to a minimum: at high frame rates the step may be too small to reach the top of the step
                const Vec3f step_forward = vec3f::scale(step_forward_normalized, scalar::max(settings.walk_stairs_min_step_forward, desired_horizontal_step_len - achieved_horizontal_step_len));

                // how far ahead to scan for a floor when the floor at step_forward is too steep: the ground normal in
                // the horizontal plane, or the movement direction when that differs too much
                Vec3f step_forward_test = vec3f::negate(character.ground.normal);
                step_forward_test = vec3f::scale_and_add(step_forward_test, character.up, -vec3f::dot(step_forward_test, character.up));
                const float step_forward_test_len_sq = vec3f::squared_length(step_forward_test);
                if (step_forward_test_len_sq > 1e-12f) {
                    step_forward_test = vec3f::scale(step_forward_test, 1.0f / scalar::sqrt(step_forward_test_len_sq));
                } else {
                    step_forward_test = step_forward_normalized;
                }
                if (vec3f::dot(step_forward_test, step_forward_normalized) < settings.walk_stairs_cos_angle_forward_contact) {
                    step_forward_test = step_forward_normalized;
                }
                step_forward_test = vec3f::scale(step_forward_test, settings.walk_stairs_step_forward_test);

                walk_stairs(world, character, delta_time, vec3f::load(settings.walk_stairs_step_up), step_forward, step_forward_test,
                            vec3f::load(settings.walk_stairs_step_down_extra), filter, policy);
            }
        }
    }

    finalize_contact_tracking(character);
}

}  // namespace kcc

TUMBLE_NAMESPACE_END
