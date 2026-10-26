#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/vec4f.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/world.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct RigidBody;

struct KCC;

// a kinematic character controller: a shape slid through the world by collide and cast queries, with an optional
// kinematic inner body so the world sees it

/** the settings' fields mean what they do on KCCSettings */
struct KCC {
    enum class GroundState : uint32_t {
        /** walkable */
        ON_GROUND = 0,
        /** too steep to climb */
        ON_STEEP_GROUND = 1,
        /** touching something, but not supported by it */
        NOT_SUPPORTED = 2,
        IN_AIR = 3,
    };

    /** triangle mesh back faces */
    enum class BackFaceMode : uint32_t {
        IGNORE = 0,
        COLLIDE = 1,
    };

    struct Contact {
        Vec3f position;
        /** the contacted body's velocity at the point */
        Vec3f linear_velocity;
        /** toward the character */
        Vec3f contact_normal;
        /** may differ from contact_normal at edges */
        Vec3f surface_normal;
        /** negative is penetration */
        float distance;
        /** along the sweep, 0 to 1 */
        float fraction;
        BodyId body_id;
        SubShapeId sub_shape_id;
        int32_t material_id;
        MotionType motion_type;
        /** a sensor never pushes the character */
        bool is_sensor;
        /** a real collision, not only a predictive contact */
        bool had_collision;
        /** passed over: rejected by the listener, a sensor once met, or the shallower of two opposing contacts on one body */
        bool was_discarded;
        bool can_push_character;
        /** the character may apply impulses to the body */
        bool can_receive_impulses;
    };

    struct Constraint {
        Contact *contact;
        /** time of impact; negative when penetrating */
        float toi;
        /** the velocity along the contact normal */
        float projected_velocity;
        /** the contact's, with the penetration correction */
        Vec3f linear_velocity;
        Vec3f plane_normal;
        /** from the character's centre */
        float plane_distance;
        bool is_steep_slope;
    };

    /** a contact the sweep skips: one the solver discarded */
    struct IgnoredContact {
        BodyId body_id;
        SubShapeId sub_shape_id;
    };

    struct GroundInfo {
        GroundState state;
        /** the average of the supporting normals */
        Vec3f normal;
        Vec3f velocity;
        /** INVALID_BODY_ID in the air */
        BodyId body_id;
        SubShapeId sub_shape_id;
        Vec3f position;
    };

    /** a contact's push settings, as the listener leaves them */
    struct ContactSettings {
        bool can_push_character;
        /** the character may apply impulses to the body */
        bool can_receive_impulses;
    };

    /** a contact the tracking has seen this frame or the last */
    struct TrackedContact {
        BodyId body_id;
        SubShapeId sub_shape_id;
        /** 0 when not seen this frame */
        uint32_t count;
        /** as the listener left them when the contact was first met this frame */
        ContactSettings settings;
    };

    enum class ContactEventKind : uint32_t {
        /** first met this frame, not touching last frame */
        ADDED = 0,
        /** first met this frame, touching last frame too */
        PERSISTED = 1,
        /** touching last frame, not at the end of this one; the body may have been removed */
        REMOVED = 2,
    };

    /** what the tracking saw: a contact added or persisted as the character first met it, or removed at the frame's end */
    struct ContactEvent {
        ContactEventKind kind;
        BodyId body_id;
        SubShapeId sub_shape_id;
        /** on the body; zero for a removal */
        Vec3f position;
        /** away from the character, into the body; zero for a removal */
        Vec3f normal;
    };

    /**
     * the character's callbacks, each optional, as Jolt's CharacterContactListener: how it treats the bodies it meets.
     * kcc_policy::listener makes one from per body data. a body given to a callback is live; normals point away from the
     * character, into the body
     */
    struct Listener {
        /** the body's velocities as the character sees them, before collision detection: a conveyor belt's surface */
        void (*on_adjust_body_velocity)(const KCC &character, const RigidBody &body, Vec3f &io_linear_velocity,
                                        Vec3f &io_angular_velocity, void *context);
        /** false: the character passes through the contact */
        bool (*on_contact_validate)(const KCC &character, const RigidBody &body, SubShapeId sub_shape_id,
                                    Vec3f contact_position, Vec3f contact_normal, void *context);
        /** first met this frame and not touching last frame; the settings start true and are the callback's to change */
        void (*on_contact_added)(const KCC &character, const RigidBody &body, SubShapeId sub_shape_id,
                                 Vec3f contact_position, Vec3f contact_normal, ContactSettings &io_settings, void *context);
        /** first met this frame and touching last frame too */
        void (*on_contact_persisted)(const KCC &character, const RigidBody &body, SubShapeId sub_shape_id,
                                     Vec3f contact_position, Vec3f contact_normal, ContactSettings &io_settings,
                                     void *context);
        /** the character's velocity as the solve leaves it against the contact, the callback's to change */
        void (*on_contact_solve)(const KCC &character, const RigidBody &body, SubShapeId sub_shape_id,
                                 Vec3f contact_position, Vec3f contact_normal, Vec3f contact_velocity, Vec3f character_velocity,
                                 Vec3f &io_new_character_velocity, void *context);
        /** touching last frame, not at the end of this one; by id, as the body may have been removed */
        void (*on_contact_removed)(const KCC &character, BodyId body_id, SubShapeId sub_shape_id, void *context);
        void *context;
    };

    struct BodyPolicy;
    struct Policy;

    /** the Refs to it; the last to let go frees it */
    mutable uint32_t ref_count;

    Vec3f position;
    Quatf quaternion;
    Vec3f linear_velocity;
    Vec3f up;

    /** held */
    Ref<Shape> shape;
    Vec3f shape_offset;

    float mass;
    float max_strength;
    float predictive_contact_distance;
    uint32_t max_collision_iterations;
    uint32_t max_constraint_iterations;
    float min_time_remaining;
    float collision_tolerance;
    float character_padding;
    uint32_t max_num_hits;
    float hit_reduction_cos_max_angle;
    float penetration_recovery_speed;
    float max_slope_angle;
    float cos_max_slope_angle;
    BackFaceMode back_face_mode;
    bool enhanced_internal_edge_removal;

    /**
     * in character space: a contact behind it can support the character, one in front only collides. signed
     * distance(point) = dot(point, normal) + constant
     */
    Vec4f supporting_volume_plane;

    GroundInfo ground;

    uint64_t user_data;
    bool observes_contacts;

    /** INVALID_BODY_ID until add */
    BodyId inner_rigid_body_id;
    /** the world add put the inner body in, which it is removed from when the last Ref lets go; null until add */
    World *world;
    /** held */
    Ref<Shape> inner_rigid_body_shape;
    uint32_t inner_rigid_body_object_layer;

    /** this frame's */
    collections::Array<Contact> contacts;

    collections::Array<TrackedContact> tracked_contacts;
    /** since the frame began: update, refresh_contacts and set_shape begin one, and a move adds to it */
    collections::Array<ContactEvent> contact_events;
    /** update's, for the supporting contact velocity check */
    float last_delta_time;

    // scratch the queries and the solver reuse across calls
    collections::Array<Constraint> constraints;
    collections::Array<Constraint *> sorted_constraints;
    collections::Array<Constraint *> previous_constraints;
    collections::Array<IgnoredContact> ignored_contacts;
    collections::Array<IgnoredContact> supporting_ignored_contacts;
    collections::Array<Contact> supporting_probe_contacts;
    collections::Array<Contact> walk_stairs_probe_contacts;
    collections::Array<Contact> set_shape_contacts;
    /** a query's filter: the caller's, the inner body skipped */
    Filter chained_filter;
};

struct KCCSettings {
    /** held by the character */
    Shape *shape = nullptr;
    /** local, from the character's position */
    Float3 shape_offset = {0.0f, 0.0f, 0.0f};

    /** kg */
    float mass = 70.0f;
    /** the most push force, N */
    float max_strength = 100.0f;

    /** normalised by create */
    Float3 up = {0.0f, 1.0f, 0.0f};
    /** radians */
    float max_slope_angle = 50.0f * 3.14159265358979323846f / 180.0f;
    /** the look-ahead for predictive contacts */
    float predictive_contact_distance = 0.1f;
    /** kept between the shape and surfaces */
    float character_padding = 0.02f;

    /** per update */
    uint32_t max_collision_iterations = 5;
    uint32_t max_constraint_iterations = 15;

    /** the penetration accepted */
    float collision_tolerance = 1e-3f;
    /** 0 to 1 */
    float penetration_recovery_speed = 1.0f;
    /** a move stops with less time than this left */
    float min_time_remaining = 1e-4f;

    /** per query */
    uint32_t max_num_hits = 256;
    /** the cosine of the angle within which hits merge, cos(2.5 degrees) */
    float hit_reduction_cos_max_angle = 0.99904822158f;
    KCC::BackFaceMode back_face_mode = KCC::BackFaceMode::COLLIDE;
    bool enhanced_internal_edge_removal = false;

    /**
     * with has_supporting_volume_plane, what can support the character, in its own space: a capsule might use
     * {{0, 1, 0}, -radius} for stricter filtering. without, the plane is up at -1e10, which accepts any contact
     */
    bool has_supporting_volume_plane = false;
    Plane3f supporting_volume_plane = {{0.0f, 1.0f, 0.0f}, 0.0f};

    /** carried by the inner body */
    uint64_t user_data = 0;
    /** carried by the inner body: the character's contacts become contact pairs (contact_pairs::record_character_contacts) */
    bool observes_contacts = false;

    /** the character's presence in the world, null for none; held by the character */
    Shape *inner_rigid_body_shape = nullptr;
    uint32_t inner_rigid_body_object_layer = 0;
};

struct KCCUpdateSettings {
    /** floor sticking's step down */
    Float3 stick_to_floor_step_down = {0.0f, -0.5f, 0.0f};
    Float3 walk_stairs_step_up = {0.0f, 0.4f, 0.0f};
    /** the least forward movement that counts as a stair */
    float walk_stairs_min_step_forward = 0.02f;
    /** how far ahead the floor is tested */
    float walk_stairs_step_forward_test = 0.15f;
    /** a forward contact is within this cosine, cos(75 degrees) */
    float walk_stairs_cos_angle_forward_contact = 0.25881904510f;
    /** stepped down after a stair walk */
    Float3 walk_stairs_step_down_extra = {0.0f, 0.0f, 0.0f};
};

inline void add_ref(const KCC *character) { __atomic_fetch_add(&character->ref_count, 1, __ATOMIC_RELAXED); }
void release_ref(const KCC *character);

namespace kcc {

/**
 * a character, held by the Refs to it as jolt's CharacterVirtual is: the last to let go frees it, removing its inner
 * body from the world it was added to, which must outlive it while it is there. with an inner body, the world sees it
 * only once added
 */
KCC *create(const KCCSettings &settings, Vec3f position, Quatf quaternion);

/** another Ref holds the character; Ref calls it */
/** a Ref lets go of the character, the last one freeing it; Ref calls it */

/** the kinematic inner body into the world: visible to queries, contacts, ccd and sensors; nothing without one */
void add(World &world, KCC &character);
void remove(World &world, KCC &character);

bool is_slope_too_steep(const KCC &character, Vec3f normal);
/** position plus the rotated shape offset */
void get_center_of_mass_transform(const KCC &character, Vec3f &out_position, Quatf &out_quaternion);
/** on ground or on steep ground */
bool is_supported(const KCC &character);

/** the inner body follows */
void set_position(World &world, KCC &character, Vec3f position);
/** the inner body follows */
void set_quaternion(World &world, KCC &character, Quatf quaternion);

/** from the ground body, its rotation and the listener's adjustment included */
void update_ground_velocity(World &world, KCC &character, const KCC::Listener *listener);

/** the contacts and the ground state where the character is now; for after a teleport */
void refresh_contacts(World &world, KCC &character, const Filter &filter, const KCC::Listener *listener);

/** this frame */
bool has_collided_with(const KCC &character, BodyId body_id);

/** supported and pushing into steep slopes it might walk up */
bool can_walk_stairs(const KCC &character, Vec3f linear_velocity);

/** refused, false, when the new shape would penetrate deeper than max_penetration_depth; FLT_MAX skips the test */
bool set_shape(World &world, KCC &character, Shape *new_shape, const Filter &filter, const KCC::Listener *listener,
               float max_penetration_depth);

/**
 * by the velocity, with collision resolution; the ground state and the inner body follow, and a dynamic ground body is
 * pushed by the character's weight. without update's contact tracking, stairs and floor sticking
 */
void move(World &world, KCC &character, float delta_time, Vec3f gravity, const KCC::Listener *listener,
          const Filter &filter);

/** sweeps down by step_down onto a floor; true if one was found */
bool stick_to_floor(World &world, KCC &character, Vec3f step_down, const Filter &filter,
                    const KCC::Listener *listener);

/** up, forward and down onto a walkable floor; true if the step succeeded */
bool walk_stairs(World &world, KCC &character, float delta_time, Vec3f step_up, Vec3f step_forward,
                 Vec3f step_forward_test, Vec3f step_down_extra, const Filter &filter,
                 const KCC::Listener *listener);

/** move with contact tracking, velocity cancelled toward steep slopes, floor sticking and stair walking */
void update(World &world, KCC &character, float delta_time, Vec3f gravity, const KCCUpdateSettings &settings,
            const KCC::Listener *listener, const Filter &filter);

// a move's phases, as move runs them each collision iteration: collide, solve, cast, then the position advanced by
// the cast's share of the displacement; after the iterations the position committed (set_position), the ground
// updated and the character's weight applied. a caller's own loop runs them in turn, and between collide and solve may
// edit a contact's was_discarded (the solve and the cast then pass it over) and linear_velocity. update adds contact
// tracking around them, velocity cancelled toward steep slopes before, and stick_to_floor and walk_stairs after

/**
 * the character's contacts at a position: the shape collided with the world (movement_direction, unit or zero, for
 * active edges), sorted by body and sub shape, the padding taken off their distances, and of each pair of opposing
 * contacts on one body the shallower discarded
 */
void collide(World &world, KCC &character, Vec3f position, Vec3f movement_direction, const Filter &filter,
             const KCC::Listener *listener);

/**
 * the slide over the contacts for up to time_remaining: each contact the character moves into validated, met and
 * pushed as the listener's settings allow, and slid along (or two at once). velocity in and out; the displacement into
 * out_displacement, and the time simulated returned
 */
float solve(World &world, KCC &character, Vec3f &io_velocity, float delta_time, float time_remaining, const KCC::Listener *listener,
            Vec3f &out_displacement);

/** the first contact sweeping by the displacement from a position, past the discarded contacts: true, with it in out */
bool cast(KCC::Contact &out, World &world, KCC &character, Vec3f position, Vec3f displacement, const Filter &filter,
          const KCC::Listener *listener);

/** the ground from the contacts, flagging those near enough as touching unless moving away (with the velocity check) */
void update_ground(World &world, KCC &character, float delta_time, bool skip_contact_velocity_check, const KCC::Listener *listener);

/** the character's weight on a dynamic ground body, along the ground normal, over delta_time */
void apply_weight(World &world, KCC &character, Vec3f gravity, float delta_time);

/** the frame of contact tracking begun: contact_events cleared, the contacts touching now remembered */
void begin_contact_tracking(KCC &character);
/** the frame ended: the remembered contacts not touching now reported removed */
void end_contact_tracking(KCC &character, const KCC::Listener *listener);

/** the velocity without its horizontal part into steep slopes the character touches, unless on ground or in the air */
Vec3f cancel_velocity_towards_steep_slopes(const KCC &character, Vec3f velocity);

}  // namespace kcc

TUMBLE_NAMESPACE_END
