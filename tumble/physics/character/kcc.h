#pragma once

// crashcat's kcc.ts: a kinematic character controller (jolt's CharacterVirtual) that slides a shape through
// a world by collide and cast queries, with an optional kinematic inner body so the world sees it. kcc.cpp.

#include <stdint.h>

#include "math/types.h"
#include "math/vec3f.h"
#include "math/vec4f.h"
#include "physics/body/body_id.h"
#include "physics/body/motion_type.h"
#include "physics/body/sub_shape.h"
#include "physics/filter.h"
#include "physics/shapes/shapes.h"
#include "physics/world.h"
#include "util/collections.h"

namespace rigid_body {
struct RigidBody;
}

namespace kcc {

/** ground state of the character */
enum GroundState : uint32_t {
    /** on walkable surface, can move freely */
    ON_GROUND = 0,
    /** on slope too steep to climb */
    ON_STEEP_GROUND = 1,
    /** touching something but not supported */
    NOT_SUPPORTED = 2,
    /** not touching anything */
    IN_AIR = 3,
};

/** how to handle back faces of triangle meshes during collision detection */
enum BackFaceMode : uint32_t {
    /** ignore back faces of triangle meshes */
    BACK_FACE_IGNORE = 0,
    /** collide with back faces of triangle meshes */
    BACK_FACE_COLLIDE = 1,
};

/** contact information for a character collision */
struct CharacterContact {
    /** world position of contact */
    Vec3f position;
    /** velocity at contact point (from contacted body) */
    Vec3f linear_velocity;
    /** normal pointing toward character */
    Vec3f contact_normal;
    /** true surface normal (may differ from contact_normal for edges) */
    Vec3f surface_normal;
    /** distance to surface (negative = penetration) */
    float distance;
    /** fraction along sweep (0-1) */
    float fraction;
    /** body id of contacted body */
    body_id::BodyId body_id;
    /** sub-shape id on contacted body */
    sub_shape::SubShapeId sub_shape_id;
    /** material id of contacted sub-shape */
    int32_t material_id;
    /** motion type of contacted body (for constraint prioritization) */
    motion_type::MotionType motion_type;
    /** is the contacted body a sensor? (sensors don't push character) */
    bool is_sensor;
    /** did actual collision occur? (vs just predictive) */
    bool had_collision;
    /** was contact discarded by validation? */
    bool was_discarded;
    /** can this contact push the character? */
    bool can_push_character;
    /** can character push body? (apply impulses) */
    bool can_receive_impulses;
};

/** movement constraint derived from a contact, used during constraint solving */
struct CharacterConstraint {
    /** contact that generated this constraint */
    CharacterContact *contact;
    /** time of impact (can be negative if penetrating) */
    float toi;
    /** velocity projected on contact normal */
    float projected_velocity;
    /** velocity of contact (with penetration correction) */
    Vec3f linear_velocity;
    /** plane constraint normal */
    Vec3f plane_normal;
    /** plane constraint distance from character center */
    float plane_distance;
    /** is this a steep slope constraint? */
    bool is_steep_slope;
};

/** a contact the sweep skips: one the solver discarded */
struct IgnoredContact {
    body_id::BodyId body_id;
    sub_shape::SubShapeId sub_shape_id;
};

/** information about the character's current ground contact */
struct GroundInfo {
    /** current ground state */
    GroundState state;
    /** ground contact normal (average of supporting normals) */
    Vec3f normal;
    /** ground velocity */
    Vec3f velocity;
    /** body id of ground (or INVALID_BODY_ID if in air) */
    body_id::BodyId body_id;
    /** sub-shape id of ground contact */
    sub_shape::SubShapeId sub_shape_id;
    /** position of ground contact */
    Vec3f position;
};

/** per-contact behavior settings, can be modified in listener callbacks to customize contact behavior */
struct CharacterContactSettings {
    /** can body push character? */
    bool can_push_character;
    /** can character push body? (apply impulses) */
    bool can_receive_impulses;
};

/** tracking value for contact change detection: which contacts are new, persisted, or removed */
struct ListenerContactValue {
    body_id::BodyId body_id;
    sub_shape::SubShapeId sub_shape_id;
    /** 0 = not seen this frame, 1+ = seen */
    uint32_t count;
    /** cached settings from callback */
    CharacterContactSettings settings;
};

struct KCC;

/**
 * listener for character contact events. the kcc calls these in the middle of its solve, so they are core side
 * function pointers with a context, as a collector's are; any may be null. normals point away from the character.
 */
struct CharacterListener {
    /** called to adjust a body's velocity before collision detection (e.g., for conveyor belts) */
    void (*on_adjust_body_velocity)(const KCC &character, const rigid_body::RigidBody &body, Vec3f &io_linear_velocity,
                                    Vec3f &io_angular_velocity, void *context);
    /** called to validate contact, returns true to accept the contact, false to reject it */
    bool (*on_contact_validate)(const KCC &character, const rigid_body::RigidBody &body, sub_shape::SubShapeId sub_shape_id,
                                Vec3f contact_position, Vec3f contact_normal, void *context);
    /** called when a new contact is added */
    void (*on_contact_added)(const KCC &character, const rigid_body::RigidBody &body, sub_shape::SubShapeId sub_shape_id,
                             Vec3f contact_position, Vec3f contact_normal, CharacterContactSettings &io_settings, void *context);
    /** called when an existing contact persists (still active this frame) */
    void (*on_contact_persisted)(const KCC &character, const rigid_body::RigidBody &body, sub_shape::SubShapeId sub_shape_id,
                                 Vec3f contact_position, Vec3f contact_normal, CharacterContactSettings &io_settings,
                                 void *context);
    /** called during constraint solving to allow velocity modification */
    void (*on_contact_solve)(const KCC &character, const rigid_body::RigidBody &body, sub_shape::SubShapeId sub_shape_id,
                             Vec3f contact_position, Vec3f contact_normal, Vec3f contact_velocity,
                             Vec3f character_velocity, Vec3f &io_character_velocity, void *context);
    /** called when a contact is removed, with the body id: the body may have been removed */
    void (*on_contact_removed)(const KCC &character, body_id::BodyId body_id, sub_shape::SubShapeId sub_shape_id, void *context);
    void *context;
};

/** settings for creating a kinematic character */
struct KCCSettings {
    /** collision shape for the character; borrowed */
    shapes::Shape *shape = nullptr;
    /** local space offset of shape relative to character position */
    Vec3f shape_offset = vec3f::make(0.0f, 0.0f, 0.0f);

    /** character mass in kg */
    float mass = 70.0f;
    /** maximum push force in N */
    float max_strength = 100.0f;

    /** up direction, normalised by create */
    Vec3f up = vec3f::make(0.0f, 1.0f, 0.0f);
    /** max slope angle in radians (50 degrees) */
    float max_slope_angle = 50.0f * 3.14159265358979323846f / 180.0f;
    /** look-ahead distance for predictive contacts */
    float predictive_contact_distance = 0.1f;
    /** spacing from surfaces */
    float character_padding = 0.02f;

    /** max collision detection iterations per update */
    uint32_t max_collision_iterations = 5;
    /** max constraint solving iterations */
    uint32_t max_constraint_iterations = 15;

    /** penetration tolerance */
    float collision_tolerance = 1e-3f;
    /** penetration recovery speed 0-1 */
    float penetration_recovery_speed = 1.0f;
    /** early-out threshold for time remaining */
    float min_time_remaining = 1e-4f;

    /** max contacts per query */
    uint32_t max_num_hits = 256;
    /** hit merging angle as cosine, cos(2.5 degrees) */
    float hit_reduction_cos_max_angle = 0.99904822158f;
    /** how to handle triangle mesh back faces */
    BackFaceMode back_face_mode = BACK_FACE_COLLIDE;
    /** enable enhanced internal edge removal */
    bool enhanced_internal_edge_removal = false;

    /**
     * plane defining what can support the character (normal.xyz, constant.w), null for [up, -1e10], which accepts any
     * contact. for a capsule you might use [0, 1, 0, -radius] for stricter filtering. copied, not referenced
     */
    const Vec4f *supporting_volume_plane = nullptr;

    /** the user's own value for the character, which the inner body carries */
    uint64_t user_data = 0;

    /** shape of the optional inner rigid body (the character's presence in the world), null for none; borrowed */
    shapes::Shape *inner_rigid_body_shape = nullptr;
    /** object layer of the inner rigid body */
    uint32_t inner_rigid_body_object_layer = 0;
};

/** kinematic character controller state */
struct KCC {
    /** world-space position */
    Vec3f position;
    /** orientation */
    Quatf quaternion;

    /** linear velocity */
    Vec3f linear_velocity;
    /** up direction (normalized) */
    Vec3f up;

    /** collision shape; borrowed */
    shapes::Shape *shape;
    /** local space offset of shape relative to character position */
    Vec3f shape_offset;

    /** character mass (kg) - for pushing objects */
    float mass;
    /** max push force (N) */
    float max_strength;
    /** look-ahead distance for predictive contacts */
    float predictive_contact_distance;
    /** max collision detection iterations per update */
    uint32_t max_collision_iterations;
    /** max constraint solving iterations */
    uint32_t max_constraint_iterations;
    /** early-out threshold for time remaining */
    float min_time_remaining;
    /** penetration tolerance */
    float collision_tolerance;
    /** spacing from surfaces */
    float character_padding;
    /** max contacts per query */
    uint32_t max_num_hits;
    /** hit merging angle (cosine) */
    float hit_reduction_cos_max_angle;
    /** penetration recovery speed (0-1) */
    float penetration_recovery_speed;
    /** max slope angle (radians) - slopes steeper are not walkable */
    float max_slope_angle;
    /** cosine of max slope angle (precomputed) */
    float cos_max_slope_angle;
    /** whether to collide with triangle mesh back faces */
    BackFaceMode back_face_mode;
    /** enable enhanced internal edge removal (expensive, off by default) */
    bool enhanced_internal_edge_removal;

    /**
     * plane defining what can support the character (normal.xyz, constant.w), in character space. every contact
     * behind this plane can support the character; every contact in front of it only collides without providing
     * support. signed distance(point) = dot(point, normal) + constant
     */
    Vec4f supporting_volume_plane;

    /** current ground info */
    GroundInfo ground;

    /** the user's own value for the character */
    uint64_t user_data;

    /** inner rigid body id for physics visibility (or INVALID_BODY_ID) */
    body_id::BodyId inner_rigid_body_id;
    /** shape of the inner rigid body, null for none */
    shapes::Shape *inner_rigid_body_shape;
    /** object layer of the inner rigid body */
    uint32_t inner_rigid_body_object_layer;

    /** active contacts for current frame */
    collections::Vec<CharacterContact> contacts;

    /** live listener contact tracking values */
    collections::Vec<ListenerContactValue> listener_contacts;
    /** last delta time from update() - used for supporting contact velocity check */
    float last_delta_time;

    // scratch the queries and the solver reuse across calls
    collections::Vec<CharacterConstraint> constraints;
    collections::Vec<CharacterConstraint *> sorted_constraints;
    collections::Vec<CharacterConstraint *> previous_constraints;
    collections::Vec<IgnoredContact> ignored_contacts;
    collections::Vec<IgnoredContact> supporting_ignored_contacts;
    collections::Vec<CharacterContact> supporting_probe_contacts;
    collections::Vec<CharacterContact> walk_stairs_probe_contacts;
    collections::Vec<CharacterContact> set_shape_contacts;
};

/** settings for update() with stair walking and floor sticking */
struct UpdateSettings {
    /** step down distance for floor sticking */
    Vec3f stick_to_floor_step_down = vec3f::make(0.0f, -0.5f, 0.0f);
    /** step up distance for stair walking */
    Vec3f walk_stairs_step_up = vec3f::make(0.0f, 0.4f, 0.0f);
    /** minimum forward movement for stair detection */
    float walk_stairs_min_step_forward = 0.02f;
    /** test distance for floor validation */
    float walk_stairs_step_forward_test = 0.15f;
    /** cos angle for forward contact detection, cos(75 degrees) */
    float walk_stairs_cos_angle_forward_contact = 0.25881904510f;
    /** extra step down after stair walk */
    Vec3f walk_stairs_step_down_extra = vec3f::make(0.0f, 0.0f, 0.0f);
};

/**
 * a new character, null when the memory cannot grow. a character with an inner body is visible to the world only once
 * add() made it; a purely virtual one skips add and remove.
 */
KCC *create(const KCCSettings &settings, Vec3f position, Quatf quaternion);
/** frees the character and its lists; remove() it from the world first if it was added */
void destroy(KCC *character);

/** creates the kinematic inner body in the world: visible to queries, contacts, ccd and sensors. no-op without one configured */
void add(KCC &character, world::World &world);
/** removes the inner body from the world */
void remove(KCC &character, world::World &world);

/** true if the slope is steeper than max_slope_angle */
bool is_slope_too_steep(const KCC &character, Vec3f normal);
/** the world-space shape transform: position plus the rotated shape offset */
void get_center_of_mass_transform(const KCC &character, Vec3f &out_position, Quatf &out_quaternion);
/** on ground or on steep ground */
bool is_supported(const KCC &character);

/** sets the position and synchronizes the inner body */
void set_position(KCC &character, world::World &world, Vec3f position);
/** sets the rotation and synchronizes the inner body */
void set_quaternion(KCC &character, world::World &world, Quatf quaternion);

/** recalculates the ground velocity from the ground body (its rotation and the listener's adjustment included) */
void update_ground_velocity(KCC &character, world::World &world, const CharacterListener *listener);

/** collides at the current position and updates the contacts and the ground state; call after teleporting */
void refresh_contacts(KCC &character, world::World &world, const filter::Filter &filter, const CharacterListener *listener);

/** whether the character collided with the body this frame */
bool has_collided_with(const KCC &character, body_id::BodyId body_id);

/** whether the character is supported and pushing into steep slopes it might walk up */
bool can_walk_stairs(const KCC &character, Vec3f linear_velocity);

/**
 * changes the shape, refusing when the new one would penetrate deeper than max_penetration_depth (FLT_MAX skips the
 * test). true if changed
 */
bool set_shape(KCC &character, world::World &world, shapes::Shape *new_shape, const filter::Filter &filter, const CharacterListener *listener,
               float max_penetration_depth);

/**
 * moves the character by its velocity with collision resolution, updates the ground state and the inner body, and
 * pushes a dynamic ground body by the character's weight. no contact tracking: update() adds that, stairs and floor
 * sticking
 */
void move(KCC &character, world::World &world, float delta_time, Vec3f gravity, const CharacterListener *listener,
          const filter::Filter &filter);

/** sweeps down by step_down and moves onto the floor found; true if one was */
bool stick_to_floor(KCC &character, world::World &world, Vec3f step_down, const filter::Filter &filter,
                    const CharacterListener *listener);

/** steps up, forward and down onto a walkable floor; true if the stair walk succeeded */
bool walk_stairs(KCC &character, world::World &world, float delta_time, Vec3f step_up, Vec3f step_forward,
                 Vec3f step_forward_test, Vec3f step_down_extra, const filter::Filter &filter,
                 const CharacterListener *listener);

/** move() with contact tracking, velocity cancelled toward steep slopes, floor sticking and stair walking */
void update(KCC &character, world::World &world, float delta_time, Vec3f gravity, const UpdateSettings &settings,
            const CharacterListener *listener, const filter::Filter &filter);

}  // namespace kcc
