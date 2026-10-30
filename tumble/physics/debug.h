#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/color.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/world.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct KCC;

/**
 * wireframes of bodies, contacts and joints. `lines` holds two vertices a line, `triangles` three a triangle; a draw may
 * grow them, moving their data. clear keeps the memory, so drawing the same scene again does not grow them
 */
struct DebugDraw {
    /** 16 bytes: the position's three floats, then the colour's four bytes */
    struct Vertex {
        Float3 position;
        Color color;
    };

    enum class BodyColorMode : uint32_t {
        MOTION_TYPE = 0,
        INSTANCE = 1,
        SLEEPING = 2,
        ISLAND = 3,
    };

    struct BodiesOptions {
        BodyColorMode color_mode = BodyColorMode::MOTION_TYPE;
        bool show_linear_velocity = false;
        bool show_angular_velocity = false;
        /** a bit per object layer whose bodies are not drawn */
        uint32_t excluded_object_layers = 0;
    };

    struct BodyOptions {
        /** use `color` instead of the colour mode's */
        bool has_color = false;
        Color color = color::WHITE;
        BodyColorMode color_mode = BodyColorMode::MOTION_TYPE;
        bool show_linear_velocity = false;
        bool show_angular_velocity = false;
    };

    struct ShapeOptions {
        Color color = color::WHITE;
    };

    struct JointsOptions {
        /** axis arm length */
        float size = 0.5f;
        /** draw angular / translation limit indicators */
        bool draw_limits = true;
    };

    collections::Array<Vertex> lines;
    uint32_t line_count = 0;
    collections::Array<Vertex> triangles;
    uint32_t triangle_count = 0;
};

namespace debug {

/** empties both buffers, keeping their memory */
void clear(DebugDraw &buffers);

// every draw appends

/** every live body's shape outside the excluded layers, coloured by the mode, with its velocities when asked */
void bodies(DebugDraw &out, const World &world, const DebugDraw::BodiesOptions &options);
/** one body's shape and velocities */
void body(DebugDraw &out, const RigidBody &body, const DebugDraw::BodyOptions &options);
/** one line, as a user shape's draw makes its wireframe */
void line(DebugDraw &out, Vec3f a, Vec3f b, Color color);
/** a shape at a world-space pose */
void shape(DebugDraw &out, const Shape *shape, Vec3f position, Quatf quaternion, const DebugDraw::ShapeOptions &options);
/** each contact constraint's points on body a with the normal */
void contacts(DebugDraw &out, const World &world);
/** the character's contacts from its last move or refresh: each point, and its normal toward the character */
void character_contacts(DebugDraw &out, const KCC &character);
/** each contact constraint's manifold: both points, the outline, the normal and tangents */
void contact_constraints(DebugDraw &out, const World &world);
/** every enabled constraint's anchors and axes, and its limits when asked */
void joints(DebugDraw &out, const World &world, const DebugDraw::JointsOptions &options);

}  // namespace debug

TUMBLE_NAMESPACE_END
