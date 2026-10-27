#include "app/shell.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#if defined(__EMSCRIPTEN__)
#include <emscripten.h>
#endif

#include "app/examples.h"
#include "app/vec.h"
#include "imgui.h"
#include "sokol_app.h"
#include "sokol_imgui.h"
#include "sokol_log.h"
#include "sokol_time.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/shapes/dispatch.h"

using namespace tumble;

static App state;
static int current_example = -1;

/** at most this many steps a frame, so a slow step cannot snowball */
static constexpr int MAX_STEPS_PER_FRAME = 4;

namespace app {

World *create_world(Layers &out, Float3 gravity) {
    WorldSettings settings = world_settings::create();
    out.moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    out.not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, out.moving, out.moving);
    world_settings::enable_collision(settings, out.moving, out.not_moving);
    settings.gravity = gravity;
    World *world = world::create(settings);
    return world;
}

void clear_world(App &app) {
    if (app.world != nullptr) world::destroy(app.world);
    app.world = nullptr;
    // the world, then the example's hold on its shapes
    app.shapes.clear();
    app.body_colors.length = 0;
    app.step_poses.length = 0;
    app.step_sequences.length = 0;
    // a freed shape's memory may be reused by the next world's
    renderer::forget_shapes(app.renderer);
}

Shape *keep(App &app, Shape *shape) {
    app.shapes.push(shape);
    return shape;
}

BodyPose pose(const App &app, const RigidBody &body) {
    const BodyPose now = {body.position, body.quaternion};
    const uint32_t index = body.index;
    if (app.alpha >= 1.0f || index >= app.step_poses.length || app.step_sequences[index] != app.world->bodies.sequences[index]) return now;
    const BodyPose &before = app.step_poses[index];
    return {vec::lerp(before.position, now.position, app.alpha), vec::quat_slerp(before.quaternion, now.quaternion, app.alpha)};
}

void snap(App &app, const RigidBody &body) {
    if (body.index >= app.step_poses.length) return;
    app.step_poses[body.index] = {body.position, body.quaternion};
    app.step_sequences[body.index] = app.world->bodies.sequences[body.index];
}

Float3 blend(const App &app, Float3 before, Float3 now) { return app.alpha >= 1.0f ? now : vec::lerp(before, now, app.alpha); }

Float4 blend(const App &app, Float4 before, Float4 now) { return app.alpha >= 1.0f ? now : vec::quat_slerp(before, now, app.alpha); }

void set_color(App &app, const RigidBody *body, Color color) {
    const uint32_t index = body_id::index(body->id);
    while (app.body_colors.length <= index) app.body_colors.push({0, 0, 0, 0});
    app.body_colors[index] = color;
}

static ImU32 packed(Color color) {
    return ImGui::ColorConvertFloat4ToU32({color.r, color.g, color.b, color.a});
}

/** a world point's place on the screen, false behind the camera */
static bool project(const Camera &camera, Float3 at, ImVec2 &out) {
    const float *m = camera.view_projection.m;
    const float w = m[3] * at.x + m[7] * at.y + m[11] * at.z + m[15];
    if (w <= 1e-4f) return false;
    const float x = (m[0] * at.x + m[4] * at.y + m[8] * at.z + m[12]) / w;
    const float y = (m[1] * at.x + m[5] * at.y + m[9] * at.z + m[13]) / w;
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    out = {(x * 0.5f + 0.5f) * display.x, (0.5f - y * 0.5f) * display.y};
    return true;
}

void label(const App &app, Float3 at, const char *text, const LabelStyle &style) {
    ImDrawList *draw = ImGui::GetBackgroundDrawList();
    ImVec2 centre;
    if (!project(app.camera, at, centre)) return;
    if (style.height <= 0.0f) {
        const ImVec2 size = ImGui::CalcTextSize(text);
        draw->AddText({centre.x - 0.5f * size.x, centre.y - 0.5f * size.y}, packed(style.color), text);
        return;
    }
    // the label's half height on screen, from its top's projection
    ImVec2 top;
    if (!project(app.camera, vec::scale_and_add(at, app.camera.up, style.height * 0.5f), top)) return;
    const float half_height = sqrtf((top.x - centre.x) * (top.x - centre.x) + (top.y - centre.y) * (top.y - centre.y));
    const float half_width = half_height * style.width / style.height;
    if (style.background.a > 0.0f) {
        draw->AddRectFilled({centre.x - half_width, centre.y - half_height}, {centre.x + half_width, centre.y + half_height}, packed(style.background));
    }
    const float size = 2.0f * half_height * style.text_fraction;
    const ImVec2 extent = ImGui::GetFont()->CalcTextSizeA(size, 1e9f, 0.0f, text);
    const ImVec2 corner = {centre.x - extent.x * 0.5f, centre.y - extent.y * 0.5f};
    // twice, a pixel apart, bold as crashcat's
    draw->AddText(ImGui::GetFont(), size, corner, packed(style.color), text);
    draw->AddText(ImGui::GetFont(), size, {corner.x + 1.0f, corner.y}, packed(style.color), text);
}

DrawList &frame(App &app) { return app.renderer.frame; }

double now_milliseconds() { return stm_ms(stm_now()); }

}

namespace shell {

static void show_example_in_url(int index) {
#if defined(__EMSCRIPTEN__)
    char id[128];
    example_id(id, sizeof(id), index);
    char script[256];
    snprintf(script, sizeof(script),
             "var url = new URL(window.location.href); url.hash = ''; url.searchParams.set('example', '%s');"
             " window.history.replaceState(null, '', url.toString());",
             id);
    emscripten_run_script(script);
#else
    (void)index;
#endif
}

static void end_example() {
    if (current_example >= 0 && examples::at(current_example).shutdown) examples::at(current_example).shutdown(state);
    app::clear_world(state);
}

App &app() { return state; }

void select_example(int index) {
    if (index == current_example) return;
    end_example();
    current_example = index;
    state.paused = false;
    state.accumulator = 0;
    state.click = {};
    state.body_colors.length = 0;
    state.camera = camera::create();
    // each example starts from these, changing what it shows in its init
    state.drawing = {true, true, false, false, false, false, true, DebugDraw::BodyColorMode::MOTION_TYPE, true};
    state.alpha = 1.0f;
    examples::at(current_example).init(state);
    show_example_in_url(current_example);
}

void example_id(char *out, size_t capacity, int index) {
    const int written = snprintf(out, capacity, "example_");
    size_t at = written > 0 ? (size_t)written : 0;
    for (const char *c = examples::at(index).name; *c != '\0' && at + 1 < capacity; c++) {
        out[at++] = isalnum((unsigned char)*c) ? (char)tolower((unsigned char)*c) : '_';
    }
    out[at] = '\0';
}

int find_example(const char *name) {
    char id[128];
    for (int i = 0; i < examples::count(); i++) {
        if (strcmp(examples::at(i).name, name) == 0) return i;
        example_id(id, sizeof(id), i);
        if (strcmp(id, name) == 0) return i;
    }
    return -1;
}

void setup(const sg_environment &environment, int initial_example) {
    sg_desc desc = {};
    desc.environment = environment;
    desc.logger.func = slog_func;
    sg_setup(&desc);

    simgui_desc_t imgui = {};
    imgui.logger.func = slog_func;
    simgui_setup(&imgui);

    dispatch::init();

    state.renderer = renderer::create();
    state.step_rate = 60;

    select_example(initial_example);
}

void shutdown() {
    end_example();
    current_example = -1;
    state.shapes.release();
    state.body_colors.release();
    state.step_poses.release();
    state.step_sequences.release();
    renderer::destroy(state.renderer);
    simgui_shutdown();
    sg_shutdown();
    state = {};
}

/** sleeping bodies darker, statics grey: the debug colour modes, for solid meshes */
static Color body_color(const RigidBody &body) {
    const uint32_t index = body_id::index(body.id);
    Color color;
    if (index < state.body_colors.length && state.body_colors[index].a > 0) {
        color = state.body_colors[index];
    } else if (body.motion_type == MotionType::STATIC) {
        color = {0.45f, 0.45f, 0.47f, 1.0f};
    } else if (body.motion_type == MotionType::KINEMATIC) {
        color = {0.55f, 0.75f, 0.95f, 1.0f};
    } else {
        const uint32_t hash = (index + 1) * 2654435761u;
        color = {0.45f + 0.5f * (float)((hash >> 8) & 255) / 255.0f, 0.45f + 0.5f * (float)((hash >> 16) & 255) / 255.0f,
                 0.45f + 0.5f * (float)((hash >> 24) & 255) / 255.0f, 1.0f};
    }
    if (body.motion_type == MotionType::DYNAMIC && rigid_body::is_sleeping(body)) color = {color.r * 0.6f, color.g * 0.6f, color.b * 0.6f, color.a};
    if (body.sensor) color.a = fminf(color.a, 0.35f);
    return color;
}

static void draw_grid() {
    const int half = 25;
    for (int i = -half; i <= half; i++) {
        const Color color = i == 0 ? Color{0.35f, 0.35f, 0.38f, 1.0f} : (i % 5 == 0 ? Color{0.5f, 0.5f, 0.53f, 1.0f} : Color{0.6f, 0.6f, 0.63f, 1.0f});
        const float at = (float)i;
        // a hair above y = 0, clear of a ground's surface
        renderer::line(state.renderer.frame, {at, 0.002f, (float)-half}, {at, 0.002f, (float)half}, color);
        renderer::line(state.renderer.frame, {(float)-half, 0.002f, at}, {(float)half, 0.002f, at}, color);
    }
}

static void draw_world(const World &world) {
    const Drawing &drawing = state.drawing;
    if (drawing.grid) draw_grid();
    if (drawing.meshes) {
        for (uint32_t i = 0; i < world.bodies.pool.length; i++) {
            if (world.bodies.sequences[i] == bodies::SLOT_FREE) continue;
            const RigidBody &body = *world.bodies.pool[i];
            const GpuMesh mesh = renderer::mesh_of(state.renderer, body.shape);
            const BodyPose drawn = app::pose(state, body);
            renderer::mesh(state.renderer, mesh, matrix::compose(drawn.position, drawn.quaternion), body_color(body));
        }
    }
    debug::clear(state.debug);
    if (drawing.wireframes) {
        DebugDraw::BodiesOptions options = {};
        options.color_mode = drawing.color_mode;
        options.show_linear_velocity = drawing.velocities;
        options.show_angular_velocity = drawing.velocities;
        debug::bodies(state.debug, world, options);
    }
    if (drawing.contacts) debug::contacts(state.debug, world);
    if (drawing.contact_manifolds) debug::contact_constraints(state.debug, world);
    if (drawing.joints) debug::joints(state.debug, world, {});
    renderer::debug(state.renderer.frame, state.debug);
}

/** every body's pose as a step begins, what app::pose blends from */
static void take_step_poses(App &app) {
    const Bodies &bodies = app.world->bodies;
    while (app.step_poses.length < bodies.pool.length) {
        app.step_poses.push({});
        app.step_sequences.push(bodies::SLOT_FREE);
    }
    for (uint32_t i = 0; i < bodies.pool.length; i++) {
        app.step_sequences[i] = bodies.sequences[i];
        if (bodies.sequences[i] == bodies::SLOT_FREE) continue;
        app.step_poses[i] = {bodies.pool[i]->position, bodies.pool[i]->quaternion};
    }
}

static void step_world(const Example &example, float delta_time) {
    if (state.world == nullptr) return;
    const bool stepping = !state.paused || state.step_once;
    if (!stepping) return;
    const float step = 1.0f / state.step_rate;
    int steps = 0;
    if (state.step_once) {
        steps = 1;
        state.step_once = false;
        state.accumulator = 0;
    } else {
        state.accumulator += delta_time;
        while (state.accumulator >= step && steps < MAX_STEPS_PER_FRAME) {
            state.accumulator -= step;
            steps++;
        }
        if (steps == MAX_STEPS_PER_FRAME) state.accumulator = 0;
    }
    for (int i = 0; i < steps; i++) {
        take_step_poses(state);
        if (example.step) example.step(state, step);
        const double start = app::now_milliseconds();
        if (example.step_world) {
            example.step_world(state, step);
        } else {
            world::step(*state.world, step);
        }
        state.step_milliseconds = app::now_milliseconds() - start;
    }
}

static void ui(const Example &example, float delta_time) {
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("tumble");
    if (ImGui::BeginCombo("example", example.name)) {
        for (int i = 0; i < examples::count(); i++) {
            if (ImGui::Selectable(examples::at(i).name, i == current_example)) select_example(i);
        }
        ImGui::EndCombo();
    }
    if (state.world != nullptr) {
        ImGui::Checkbox("paused", &state.paused);
        ImGui::SameLine();
        if (ImGui::Button("step")) {
            state.paused = true;
            state.step_once = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("restart")) {
            const int index = current_example;
            end_example();
            current_example = -1;
            select_example(index);
        }
        ImGui::Text("%u bodies, step %.2f ms, frame %.1f ms", state.world->bodies.pool.length - state.world->bodies.free_indices.length,
                    state.step_milliseconds, delta_time * 1000.0f);
        if (ImGui::CollapsingHeader("drawing")) {
            Drawing &drawing = state.drawing;
            ImGui::Checkbox("grid", &drawing.grid);
            ImGui::Checkbox("meshes", &drawing.meshes);
            ImGui::Checkbox("interpolate", &drawing.interpolate);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("draw between the last two steps; the debug overlays show the step");
            ImGui::Checkbox("wireframes", &drawing.wireframes);
            ImGui::Checkbox("with velocities", &drawing.velocities);
            ImGui::Checkbox("contacts", &drawing.contacts);
            ImGui::Checkbox("contact manifolds", &drawing.contact_manifolds);
            ImGui::Checkbox("joints", &drawing.joints);
            const char *modes[] = {"motion type", "instance", "sleeping", "island"};
            int mode = (int)drawing.color_mode;
            if (ImGui::Combo("wireframe colour", &mode, modes, 4)) drawing.color_mode = (DebugDraw::BodyColorMode)mode;
        }
    }
    ImGui::Separator();
    if (example.ui) example.ui(state);
    ImGui::End();
}

void frame(const FrameInput &input) {
    simgui_frame_desc_t imgui_frame = {};
    imgui_frame.width = (int)input.width;
    imgui_frame.height = (int)input.height;
    imgui_frame.delta_time = input.delta_time;
    imgui_frame.dpi_scale = input.dpi_scale;
    simgui_new_frame(&imgui_frame);

    camera::update(state.camera, input.width, input.height);
    state.pointer.x = input.pointer_x;
    state.pointer.y = input.pointer_y;
    state.pointer.ray = camera::screen_ray(state.camera, state.pointer.x, state.pointer.y);
    renderer::clear(state.renderer);

    const Example &example = examples::at(current_example);
    step_world(example, input.fixed_step ? 1.0f / state.step_rate : input.delta_time);
    // paused, or stepping a fixed step a frame, the step itself
    const bool blending = state.drawing.interpolate && !state.paused && !input.fixed_step;
    state.alpha = blending ? state.accumulator * state.step_rate : 1.0f;
    if (example.update) example.update(state, input.delta_time);
    state.click = {};
    if (state.world != nullptr) draw_world(*state.world);

    if (!input.hide_ui) ui(examples::at(current_example), input.delta_time);

    sg_pass pass = {};
    pass.action.colors[0].load_action = SG_LOADACTION_CLEAR;
    pass.action.colors[0].clear_value = {0.1f, 0.1f, 0.11f, 1.0f};
    pass.swapchain = input.swapchain;
    sg_begin_pass(&pass);
    renderer::render(state.renderer, state.camera);
    simgui_render();
    sg_end_pass();
    sg_commit();
}

}
