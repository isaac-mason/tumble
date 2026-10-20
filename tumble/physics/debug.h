#pragma once

// crashcat's debug.ts: wireframes of a world's bodies, contacts and joints, appended to line and
// triangle buffers js views and draws. js clears the buffers, draws what it wants, then reads them.

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/world.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

namespace debug {

/** floats per vertex: x, y, z, r, g, b */
constexpr uint32_t VERTEX_STRIDE = 6;
/** floats per line: two vertices */
constexpr uint32_t LINE_STRIDE = 2 * VERTEX_STRIDE;
/** floats per triangle: three vertices */
constexpr uint32_t TRIANGLE_STRIDE = 3 * VERTEX_STRIDE;

/**
 * what a draw appends to. `lines` holds line_count * LINE_STRIDE floats, `triangles` triangle_count *
 * TRIANGLE_STRIDE; both grow through the heap, so js re-reads their data after a draw. clear keeps
 * the memory, so a second draw of the same scene does not grow them.
 */
struct DebugBuffers {
    collections::Vec<float> lines;
    uint32_t line_count;
    collections::Vec<float> triangles;
    uint32_t triangle_count;
};

enum BodyColorMode : uint32_t {
    COLOR_MOTION_TYPE = 0,
    COLOR_INSTANCE = 1,
    COLOR_SLEEPING = 2,
    COLOR_ISLAND = 3,
};

struct BodiesOptions {
    BodyColorMode color_mode = COLOR_MOTION_TYPE;
    bool show_linear_velocity = false;
    bool show_angular_velocity = false;
    /** a bit per object layer whose bodies are not drawn */
    uint32_t excluded_object_layers = 0;
};

struct BodyOptions {
    /** use `color` instead of the colour mode's */
    bool has_color = false;
    Float3 color = {1.0f, 1.0f, 1.0f};
    BodyColorMode color_mode = COLOR_MOTION_TYPE;
    bool show_linear_velocity = false;
    bool show_angular_velocity = false;
};

struct ShapeOptions {
    Float3 color = {1.0f, 1.0f, 1.0f};
};

struct JointsOptions {
    /** axis arm length */
    float size = 0.5f;
    /** draw angular / translation limit indicators */
    bool draw_limits = true;
};

/** empty buffers, or null (error set) when the memory cannot grow */
DebugBuffers *create();
void release(DebugBuffers *buffers);
/** empties both buffers, keeping their memory */
void clear(DebugBuffers &buffers);

// every draw appends; a line that cannot fit because the memory cannot grow is dropped (error set)

/** every live body's shape outside the excluded layers, coloured by the mode, with its velocities when asked */
void bodies(DebugBuffers &out, const world::World &world, const BodiesOptions &options);
/** one body's shape and velocities */
void body(DebugBuffers &out, const rigid_body::RigidBody &body, const BodyOptions &options);
/** a shape at a world-space pose */
void shape(DebugBuffers &out, const shapes::Shape *shape, Vec3f position, Quatf quaternion, const ShapeOptions &options);
/** each contact constraint's points on body a with the normal */
void contacts(DebugBuffers &out, const world::World &world);
/** each contact constraint's manifold: both points, the outline, the normal and tangents */
void contact_constraints(DebugBuffers &out, const world::World &world);
/** every enabled constraint's anchors and axes, and its limits when asked */
void joints(DebugBuffers &out, const world::World &world, const JointsOptions &options);

}  // namespace debug

TUMBLE_NAMESPACE_END
