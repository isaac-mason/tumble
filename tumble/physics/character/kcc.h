#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/vec4f.h"
#include "tumble/physics/body/body_id.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/world.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

// a kinematic character controller: a shape slid through the world by collide and cast queries, with an optional
// kinematic inner body so the world sees it
namespace kcc {

enum GroundState : uint32_t {
    /** walkable */
    ON_GROUND = 0,
    /** too steep to climb */
    ON_STEEP_GROUND = 1,
    /** touching something, but not supported by it */
    NOT_SUPPORTED = 2,
    IN_AIR = 3,
};

/** triangle mesh back faces */
enum BackFaceMode : uint32_t {
    BACK_FACE_IGNORE = 0,
    BACK_FACE_COLLIDE = 1,
};

struct CharacterContact {
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
    body_id::BodyId body_id;
    sub_shape::SubShapeId sub_shape_id;
    int32_t material_id;
    motion_type::MotionType motion_type;
    /** a sensor never pushes the character */
    bool is_sensor;
    /** a real collision, not only a predictive contact */
    bool had_collision;
    /** rejected by the listener's validation */
    bool was_discarded;
    bool can_push_character;
    /** the character may apply impulses to the body */
    bool can_receive_impulses;
};

struct CharacterConstraint {
    CharacterContact *contact;
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
    body_id::BodyId body_id;
    sub_shape::SubShapeId sub_shape_id;
};

struct GroundInfo {
    GroundState state;
    /** the average of the supporting normals */
    Vec3f normal;
    Vec3f velocity;
    /** INVALID_BODY_ID in the air */
    body_id::BodyId body_id;
    sub_shape::SubShapeId sub_shape_id;
    Vec3f position;
};

/** per contact; the listener's added and persisted callbacks may change them */
struct CharacterContactSettings {
    bool can_push_character;
    /** the character may apply impulses to the body */
    bool can_receive_impulses;
};

/** tracks which contacts are new, persisted or removed */
struct ListenerContactValue {
    body_id::BodyId body_id;
    sub_shape::SubShapeId sub_shape_id;
    /** 0 when not seen this frame */
    uint32_t count;
    /** as the listener left them */
    CharacterContactSettings settings;
};

struct KCC;

/** called in the middle of the solve; any may be null. normals point away from the character */
struct CharacterListener {
    /** before collision detection, e.g. for conveyor belts */
    void (*on_adjust_body_velocity)(const KCC &character, const rigid_body::RigidBody &body, Vec3f &io_linear_velocity,
                                    Vec3f &io_angular_velocity, void *context);
    /** false rejects the contact */
    bool (*on_contact_validate)(const KCC &character, const rigid_body::RigidBody &body, sub_shape::SubShapeId sub_shape_id,
                                Vec3f contact_position, Vec3f contact_normal, void *context);
    void (*on_contact_added)(const KCC &character, const rigid_body::RigidBody &body, sub_shape::SubShapeId sub_shape_id,
                             Vec3f contact_position, Vec3f contact_normal, CharacterContactSettings &io_settings, void *context);
    void (*on_contact_persisted)(const KCC &character, const rigid_body::RigidBody &body, sub_shape::SubShapeId sub_shape_id,
                                 Vec3f contact_position, Vec3f contact_normal, CharacterContactSettings &io_settings,
                                 void *context);
    /** may change the character's velocity */
    void (*on_contact_solve)(const KCC &character, const rigid_body::RigidBody &body, sub_shape::SubShapeId sub_shape_id,
                             Vec3f contact_position, Vec3f contact_normal, Vec3f contact_velocity,
                             Vec3f character_velocity, Vec3f &io_character_velocity, void *context);
    /** with the body id: the body may have been removed */
    void (*on_contact_removed)(const KCC &character, body_id::BodyId body_id, sub_shape::SubShapeId sub_shape_id, void *context);
    void *context;
};

struct KCCSettings {
    /** borrowed */
    shapes::Shape *shape = nullptr;
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
    BackFaceMode back_face_mode = BACK_FACE_COLLIDE;
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

    /** the character's presence in the world, null for none; borrowed */
    shapes::Shape *inner_rigid_body_shape = nullptr;
    uint32_t inner_rigid_body_object_layer = 0;
};

/** the settings' fields mean what they do on KCCSettings */
struct KCC {
    Vec3f position;
    Quatf quaternion;
    Vec3f linear_velocity;
    Vec3f up;

    shapes::Shape *shape;
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
    body_id::BodyId inner_rigid_body_id;
    shapes::Shape *inner_rigid_body_shape;
    uint32_t inner_rigid_body_object_layer;

    /** this frame's */
    collections::Vec<CharacterContact> contacts;

    collections::Vec<ListenerContactValue> listener_contacts;
    /** update's, for the supporting contact velocity check */
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

struct UpdateSettings {
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

/** null when the allocator fails; with an inner body, the world sees it only once added */
KCC *create(const KCCSettings &settings, Vec3f position, Quatf quaternion);
/** remove it from the world first if it was added */
void destroy(KCC *character);

/** the kinematic inner body into the world: visible to queries, contacts, ccd and sensors; nothing without one */
void add(world::World &world, KCC &character);
void remove(world::World &world, KCC &character);

bool is_slope_too_steep(const KCC &character, Vec3f normal);
/** position plus the rotated shape offset */
void get_center_of_mass_transform(const KCC &character, Vec3f &out_position, Quatf &out_quaternion);
/** on ground or on steep ground */
bool is_supported(const KCC &character);

/** the inner body follows */
void set_position(world::World &world, KCC &character, Vec3f position);
/** the inner body follows */
void set_quaternion(world::World &world, KCC &character, Quatf quaternion);

/** from the ground body, its rotation and the listener's adjustment included */
void update_ground_velocity(world::World &world, KCC &character, const CharacterListener *listener);

/** the contacts and the ground state where the character is now; for after a teleport */
void refresh_contacts(world::World &world, KCC &character, const filter::Filter &filter, const CharacterListener *listener);

/** this frame */
bool has_collided_with(const KCC &character, body_id::BodyId body_id);

/** supported and pushing into steep slopes it might walk up */
bool can_walk_stairs(const KCC &character, Vec3f linear_velocity);

/** refused, false, when the new shape would penetrate deeper than max_penetration_depth; FLT_MAX skips the test */
bool set_shape(world::World &world, KCC &character, shapes::Shape *new_shape, const filter::Filter &filter, const CharacterListener *listener,
               float max_penetration_depth);

/**
 * by the velocity, with collision resolution; the ground state and the inner body follow, and a dynamic ground body is
 * pushed by the character's weight. without update's contact tracking, stairs and floor sticking
 */
void move(world::World &world, KCC &character, float delta_time, Vec3f gravity, const CharacterListener *listener,
          const filter::Filter &filter);

/** sweeps down by step_down onto a floor; true if one was found */
bool stick_to_floor(world::World &world, KCC &character, Vec3f step_down, const filter::Filter &filter,
                    const CharacterListener *listener);

/** up, forward and down onto a walkable floor; true if the step succeeded */
bool walk_stairs(world::World &world, KCC &character, float delta_time, Vec3f step_up, Vec3f step_forward,
                 Vec3f step_forward_test, Vec3f step_down_extra, const filter::Filter &filter,
                 const CharacterListener *listener);

/** move with contact tracking, velocity cancelled toward steep slopes, floor sticking and stair walking */
void update(world::World &world, KCC &character, float delta_time, Vec3f gravity, const UpdateSettings &settings,
            const CharacterListener *listener, const filter::Filter &filter);

}  // namespace kcc

TUMBLE_NAMESPACE_END
