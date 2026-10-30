#include "app/app.h"
#include "app/examples.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/compound.h"

using namespace tumble;

static constexpr float SPACING = 3.0f;

static struct {
    app::Layers layers;
} state;

static void add_dynamic(App &app, Shape *shape, Float3 position) {
    RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = state.layers.moving;
    settings.motion_type = MotionType::DYNAMIC;
    settings.position = vec3f::load(position);
    settings.restitution = 0.0f;
    settings.friction = 0.5f;
    rigid_body::create(*app.world, settings);
}

static void init(App &app) {
    app.world = app::create_world(state.layers);

    RigidBodySettings ground;
    ground.shape = app::keep(app, box::create({.half_extents = vec3f::make(30.0f, 0.5f, 30.0f)}));
    ground.object_layer = state.layers.not_moving;
    ground.motion_type = MotionType::STATIC;
    ground.position = vec3f::make(0.0f, -0.5f, 0.0f);
    ground.restitution = 0.0f;
    ground.friction = 0.5f;
    rigid_body::create(*app.world, ground);

    // a plain upright box stands
    add_dynamic(app, app::keep(app, box::create({.half_extents = vec3f::make(0.5f, 1.5f, 0.5f), .convex_radius = 0.05f})), {-SPACING, 1.5f, 0.0f});

    // an upside down L, its mass high and to one side, falls over
    Shape *stem = app::keep(app, box::create({.half_extents = vec3f::make(0.25f, 0.75f, 0.25f), .convex_radius = 0.05f}));
    Shape *bar = app::keep(app, box::create({.half_extents = vec3f::make(0.75f, 0.25f, 0.25f), .convex_radius = 0.05f}));
    const CompoundShapeSettings::Child l_shape[] = {
        {.position = vec3f::make(0.0f, 0.75f, 0.0f), .quaternion = quatf::identity(), .shape = stem},
        {.position = vec3f::make(0.75f, 1.25f, 0.0f), .quaternion = quatf::identity(), .shape = bar},
    };
    add_dynamic(app, app::keep(app, compound::create({.children = l_shape, .num_children = 2})), {0.0f, 1.5f, 0.0f});

    // the same L with a mirrored one on top, balanced, stands
    const CompoundShapeSettings::Child double_l_shape[] = {
        {.position = vec3f::make(0.0f, 0.75f, 0.0f), .quaternion = quatf::identity(), .shape = stem},
        {.position = vec3f::make(0.75f, 1.25f, 0.0f), .quaternion = quatf::identity(), .shape = bar},
        {.position = vec3f::make(0.0f, 2.25f, 0.0f), .quaternion = quatf::identity(), .shape = stem},
        {.position = vec3f::make(-0.75f, 1.75f, 0.0f), .quaternion = quatf::identity(), .shape = bar},
    };
    add_dynamic(app, app::keep(app, compound::create({.children = double_l_shape, .num_children = 4})), {SPACING, 1.5f, 0.0f});

    camera::look_at(app.camera, {0.0f, 5.0f, 15.0f}, {0.0f, 2.0f, 0.0f});
}

static const Example EXAMPLE_MASS_PROPERTIES = {"Mass Properties", init, nullptr, nullptr, nullptr, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_MASS_PROPERTIES);
