#pragma once

#include "app/camera.h"
#include "app/renderer.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/debug.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/utils/triangle_mesh_data.h"
#include "tumble/physics/world.h"
#include "tumble/util/collections.h"

struct Click {
    bool clicked;
    bool secondary;
    bool shift;
    Ray ray;
};

struct Pointer {
    float x;
    float y;
    Ray ray;
};

struct Drawing {
    /** a grid on y = 0, where most examples' ground is */
    bool grid;
    bool meshes;
    bool wireframes;
    bool velocities;
    bool contacts;
    bool contact_manifolds;
    bool joints;
    tumble::DebugDraw::BodyColorMode color_mode;
    /** the meshes, a camera following something and what examples draw themselves, between the last two steps; the
     *  debug overlays always show the step */
    bool interpolate;
};

/** a body's place and turn: its shape's origin */
struct BodyPose {
    tumble::Float3 position;
    tumble::Float4 quaternion;
};

struct App {
    Renderer renderer;
    Camera camera;
    /** the example's world, which the app steps, draws and frees when the example ends */
    tumble::World *world;
    /** what the example's bodies are made of, freed after its world */
    tumble::collections::Array<tumble::Ref<tumble::Shape>> shapes;
    bool paused;
    bool step_once;
    /** world steps a second */
    float step_rate;
    float accumulator;
    /**
     * of a step, how far the frame is past the last one: what is drawn blends the state as that step began into the
     * state it left by this, so things move evenly whatever the frame rate. 1 when paused, not interpolating, or
     * taking a screenshot
     */
    float alpha;
    /** each body slot's pose as the last step began, and the slot's sequence then: a slot reused since is not blended */
    tumble::collections::Array<BodyPose> step_poses;
    tumble::collections::Array<uint32_t> step_sequences;
    double step_milliseconds;
    Drawing drawing;
    tumble::DebugDraw debug;
    /** a colour per body index, for bodies an example coloured; alpha 0 where it did not */
    tumble::collections::Array<Color> body_colors;
    Click click;
    Pointer pointer;
    bool keys[512];
};

struct Example {
    const char *name;
    void (*init)(App &app);
    /** before each world step, with the step's delta time */
    void (*step)(App &app, float delta_time);
    /** each frame, after the steps it is owed */
    void (*update)(App &app, float delta_time);
    void (*ui)(App &app);
    /** what the example made beyond its world and the shapes it kept; may be null */
    void (*shutdown)(App &app);
    /** the world step itself, run by its phases (update.h) for an example that answers them; null: world::step */
    void (*step_world)(App &app, float delta_time);
};

namespace app {

/** the world settings most examples use: things that move, and things that do not */
struct Layers {
    uint32_t moving;
    uint32_t not_moving;
};
tumble::World *create_world(Layers &out, tumble::Float3 gravity = {0.0f, -9.81f, 0.0f});

/**
 * frees the world and everything kept, for an example that swaps its world for another (a menu of scenes). from init,
 * step or update: by ui, the frame's meshes are queued
 */
void clear_world(App &app);

/** the shape, of a built-in type, freed with the example's world: a shape and the shapes it wraps are kept separately */
tumble::Shape *keep(App &app, tumble::Shape *shape);

struct LabelStyle {
    /** the label's size in the world, as crashcat's text sprites; a height of 0 draws plain text at screen size */
    float width = 0.0f;
    float height = 0.0f;
    /** of the height, the text's */
    float text_fraction = 0.5f;
    /** none when its alpha is 0 */
    Color background = {0.0f, 0.0f, 0.0f, 0.0f};
    Color color = {1.0f, 1.0f, 1.0f, 1.0f};
};

/** text centred on a world point, over the scene; call it each frame from update or ui */
void label(const App &app, tumble::Float3 at, const char *text, const LabelStyle &style = {});

/** the body as drawn: from its pose as the last step began to its pose now, by alpha */
BodyPose pose(const App &app, const tumble::RigidBody &body);
/** the body drawn where it is rather than swept there: after moving it other than by stepping (a respawn, a reset) */
void snap(App &app, const tumble::RigidBody &body);
/** a value an example moves in its steps, from what it was as the last step began to what it is now, by alpha */
tumble::Float3 blend(const App &app, tumble::Float3 before, tumble::Float3 now);
tumble::Float4 blend(const App &app, tumble::Float4 before, tumble::Float4 now);

void set_color(App &app, const tumble::RigidBody *body, Color color);
DrawList &frame(App &app);
double now_milliseconds();

}
