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
    tumble::debug::BodyColorMode color_mode;
};

struct App {
    Renderer renderer;
    Camera camera;
    /** the example's world, which the app steps, draws and frees when the example ends */
    tumble::world::World *world;
    /** what the example's bodies are made of, freed after its world */
    tumble::collections::Vec<tumble::shapes::Shape *> shapes;
    tumble::collections::Vec<tumble::convex_hull::ConvexHullData *> hulls;
    tumble::collections::Vec<tumble::triangle_mesh_data::TriangleMeshData *> mesh_data;
    bool paused;
    bool step_once;
    /** world steps a second */
    float step_rate;
    float accumulator;
    double step_milliseconds;
    Drawing drawing;
    tumble::debug::DebugBuffers *debug;
    /** a colour per body index, for bodies an example coloured; alpha 0 where it did not */
    tumble::collections::Vec<Color> body_colors;
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
};

namespace app {

/** the world settings most examples use: things that move, and things that do not */
struct Layers {
    uint32_t moving;
    uint32_t not_moving;
};
tumble::world::World *create_world(Layers &out, tumble::Float3 gravity = {0.0f, -9.81f, 0.0f});

/** the shape, freed with the example's world: a shape and the shapes it wraps are kept separately */
tumble::shapes::Shape *keep(App &app, tumble::shapes::Shape *shape);
tumble::convex_hull::ConvexHullData *keep(App &app, tumble::convex_hull::ConvexHullData *hull);
tumble::triangle_mesh_data::TriangleMeshData *keep(App &app, tumble::triangle_mesh_data::TriangleMeshData *data);

void set_color(App &app, const tumble::rigid_body::RigidBody *body, Color color);
DrawList &frame(App &app);
double now_milliseconds();

}
