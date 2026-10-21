#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/physics/shapes/box.h"

using namespace tumble;

static constexpr float AMPLITUDE = 6.0f;
static constexpr float FREQUENCY = 1.0f;

static struct {
    app::Layers layers;
    rigid_body::RigidBody *sensor;
    bool intersecting;
    int steps;
} state;

static Color hex(uint32_t rgb, float alpha = 1.0f) {
    return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, alpha};
}

static ImU32 hex_u32(uint32_t rgb, float alpha = 1.0f) {
    const Color color = hex(rgb, alpha);
    return ImGui::ColorConvertFloat4ToU32({color.r, color.g, color.b, color.a});
}

/** a world-space label as crashcat's sprites are: a box `width` by `height` facing the camera, its text a fraction of the height */
static void label(const App &app, Float3 at, const char *text, float width, float height, float text_fraction, ImU32 background, ImU32 color) {
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float *m = app.camera.view_projection.m;
    const auto project = [&](Float3 p, ImVec2 &out) {
        const float w = m[3] * p.x + m[7] * p.y + m[11] * p.z + m[15];
        if (w <= 1e-4f) return false;
        const float x = (m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12]) / w;
        const float y = (m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13]) / w;
        out = {(x * 0.5f + 0.5f) * display.x, (0.5f - y * 0.5f) * display.y};
        return true;
    };
    ImVec2 centre, top;
    if (!project(at, centre) || !project(vec::scale_and_add(at, app.camera.up, height * 0.5f), top)) return;
    const float half_height = sqrtf((top.x - centre.x) * (top.x - centre.x) + (top.y - centre.y) * (top.y - centre.y));
    const float half_width = half_height * width / height;
    ImDrawList *draw = ImGui::GetBackgroundDrawList();
    if (background != 0) draw->AddRectFilled({centre.x - half_width, centre.y - half_height}, {centre.x + half_width, centre.y + half_height}, background);
    const float size = 2.0f * half_height * text_fraction;
    const ImVec2 extent = ImGui::GetFont()->CalcTextSizeA(size, 1e9f, 0.0f, text);
    const ImVec2 corner = {centre.x - extent.x * 0.5f, centre.y - extent.y * 0.5f};
    // twice, a pixel apart, for crashcat's bold
    draw->AddText(ImGui::GetFont(), size, corner, color, text);
    draw->AddText(ImGui::GetFont(), size, {corner.x + 1.0f, corner.y}, color, text);
}

static void init(App &app) {
    state.steps = 0;
    state.intersecting = false;
    app.world = app::create_world(state.layers, {0.0f, 0.0f, 0.0f});

    // a kinematic sensor swept side to side
    rigid_body::RigidBodySettings sensor;
    sensor.shape = app::keep(app, box::create({.half_extents = {1.5f, 1.0f, 1.5f}}));
    sensor.object_layer = state.layers.moving;
    sensor.motion_type = motion_type::MOTION_KINEMATIC;
    sensor.position = {-8.0f, 1.0f, 0.0f};
    sensor.sensor = true;
    state.sensor = rigid_body::create(*app.world, sensor);

    shapes::Shape *cube = app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}));
    rigid_body::RigidBodySettings dynamic_box;
    dynamic_box.shape = cube;
    dynamic_box.object_layer = state.layers.moving;
    dynamic_box.motion_type = motion_type::MOTION_DYNAMIC;
    dynamic_box.position = {-2.0f, 1.0f, 0.0f};
    dynamic_box.restitution = 0.3f;
    dynamic_box.friction = 0.5f;
    rigid_body::create(*app.world, dynamic_box);

    rigid_body::RigidBodySettings static_box = dynamic_box;
    static_box.object_layer = state.layers.not_moving;
    static_box.motion_type = motion_type::MOTION_STATIC;
    static_box.position = {2.0f, 1.0f, 0.0f};
    rigid_body::create(*app.world, static_box);

    // no ground here
    app.drawing.grid = false;
    camera::look_at(app.camera, {0.0f, 5.0f, 15.0f}, {0.0f, 2.0f, 0.0f});
}

static void step(App &app, float) {
    // crashcat's clock, as steps
    state.steps++;
    const float time = (float)state.steps / 60.0f;
    rigid_body::set_position(*app.world, state.sensor, vec3f::make(sinf(time * FREQUENCY) * AMPLITUDE, 1.0f, 0.0f), false);
}

static bool touches_sensor(const events::ContactEvent &contact) { return contact.body_id_a == state.sensor->id || contact.body_id_b == state.sensor->id; }

static void update(App &app, float) {
    // the last step's contacts, added or kept
    const events::PhysicsEvents &events = app.world->events;
    state.intersecting = false;
    for (uint32_t i = 0; i < events.contacts_added.length; i++) state.intersecting = state.intersecting || touches_sensor(events.contacts_added[i]);
    for (uint32_t i = 0; i < events.contacts_persisted.length; i++) state.intersecting = state.intersecting || touches_sensor(events.contacts_persisted[i]);

    label(app, {state.sensor->position.x, 3.5f, 0.0f}, state.intersecting ? "hit" : "no hit", 4.0f, 1.0f, 0.5f, hex_u32(0x5f5f5f),
          state.intersecting ? hex_u32(0x4caf50) : hex_u32(0xf44336));
}

static void ui(App &) { ImGui::Checkbox("Collide with Non-Dynamic", &state.sensor->collide_kinematic_vs_non_dynamic); }

static const Example EXAMPLE_SENSOR = {"Sensor", init, step, update, ui, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_SENSOR);
