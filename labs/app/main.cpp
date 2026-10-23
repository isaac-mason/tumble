#include <math.h>
#include <stdio.h>
#if defined(__EMSCRIPTEN__)
#include <emscripten.h>
#endif
#include <stdlib.h>
#include <string.h>

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#elif defined(_WIN32)
#include <windows.h>
#include <GL/gl.h>
#elif !defined(__EMSCRIPTEN__)
#include <GL/gl.h>
#endif

#include "app/app.h"
#include "app/examples.h"
#include "app/vec.h"
#include "imgui.h"
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "sokol_imgui.h"
#include "sokol_log.h"
#include "sokol_time.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/physics/shapes/offset_center_of_mass.h"
#include "tumble/physics/shapes/plane.h"
#include "tumble/physics/shapes/scaled.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/static_compound.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"

using namespace tumble;

struct Input {
    bool down;
    bool dragging;
    sapp_mousebutton button;
    float down_x;
    float down_y;
    float x;
    float y;
};

static App state;
static Input input;
static int current_example = -1;
static int initial_example = 0;
static const char *screenshot_path = nullptr;
static int screenshot_frame = 30;
static int frame_count = 0;
/** seconds a frame, in place of the display's when above 0: a display rate the machine does not have, to try out */
static float frame_time = 0.0f;

/** at most this many steps a frame, so a slow step cannot snowball */
static constexpr int MAX_STEPS_PER_FRAME = 4;

static void destroy_shape(shapes::Shape *shape) {
    switch (shape->type) {
        case shapes::SHAPE_SPHERE: sphere::destroy(shape); break;
        case shapes::SHAPE_BOX: box::destroy(shape); break;
        case shapes::SHAPE_CAPSULE: capsule::destroy(shape); break;
        case shapes::SHAPE_CYLINDER: cylinder::destroy(shape); break;
        case shapes::SHAPE_CONVEX_HULL: convex_hull::destroy(shape); break;
        case shapes::SHAPE_TRIANGLE_MESH: triangle_mesh::destroy(shape); break;
        case shapes::SHAPE_COMPOUND: compound::destroy(shape); break;
        case shapes::SHAPE_STATIC_COMPOUND: static_compound::destroy(shape); break;
        case shapes::SHAPE_TRANSFORMED: transformed::destroy(shape); break;
        case shapes::SHAPE_SCALED: scaled::destroy(shape); break;
        case shapes::SHAPE_OFFSET_CENTER_OF_MASS: offset_center_of_mass::destroy(shape); break;
        case shapes::SHAPE_PLANE: plane::destroy(shape); break;
        case shapes::SHAPE_EMPTY: empty::destroy(shape); break;
        // a user type's shape is its example's to free, at its own size
        default: TUMBLE_ASSERT(false); break;
    }
}

namespace app {

world::World *create_world(Layers &out, Float3 gravity) {
    world_settings::WorldSettings settings = world_settings::create();
    out.moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    out.not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, out.moving, out.moving);
    world_settings::enable_collision(settings, out.moving, out.not_moving);
    settings.gravity = gravity;
    world::World *world = world::create(settings);
    world_settings::release(settings);
    return world;
}

void clear_world(App &app) {
    if (app.world != nullptr) world::destroy(app.world);
    app.world = nullptr;
    // the world, then its shapes (wrappers before what they wrap), then what they were built from
    for (uint32_t i = app.shapes.length; i-- > 0;) destroy_shape(app.shapes[i]);
    for (uint32_t i = 0; i < app.hulls.length; i++) convex_hull::release(app.hulls[i]);
    for (uint32_t i = 0; i < app.mesh_data.length; i++) triangle_mesh_data::destroy(app.mesh_data[i]);
    app.shapes.length = 0;
    app.hulls.length = 0;
    app.mesh_data.length = 0;
    app.body_colors.length = 0;
    app.step_poses.length = 0;
    app.step_sequences.length = 0;
    // a freed shape's memory may be reused by the next world's
    renderer::forget_shapes(app.renderer);
}

shapes::Shape *keep(App &app, shapes::Shape *shape) {
    app.shapes.push(shape);
    return shape;
}

convex_hull::ConvexHullData *keep(App &app, convex_hull::ConvexHullData *hull) {
    app.hulls.push(hull);
    return hull;
}

triangle_mesh_data::TriangleMeshData *keep(App &app, triangle_mesh_data::TriangleMeshData *data) {
    app.mesh_data.push(data);
    return data;
}

BodyPose pose(const App &app, const rigid_body::RigidBody &body) {
    const BodyPose now = {body.position, body.quaternion};
    const uint32_t index = body.index;
    if (app.alpha >= 1.0f || index >= app.step_poses.length || app.step_sequences[index] != app.world->bodies.sequences[index]) return now;
    const BodyPose &before = app.step_poses[index];
    return {vec::lerp(before.position, now.position, app.alpha), vec::quat_slerp(before.quaternion, now.quaternion, app.alpha)};
}

void snap(App &app, const rigid_body::RigidBody &body) {
    if (body.index >= app.step_poses.length) return;
    app.step_poses[body.index] = {body.position, body.quaternion};
    app.step_sequences[body.index] = app.world->bodies.sequences[body.index];
}

Float3 blend(const App &app, Float3 before, Float3 now) { return app.alpha >= 1.0f ? now : vec::lerp(before, now, app.alpha); }

Float4 blend(const App &app, Float4 before, Float4 now) { return app.alpha >= 1.0f ? now : vec::quat_slerp(before, now, app.alpha); }

void set_color(App &app, const rigid_body::RigidBody *body, Color color) {
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


static void end_example() {
    if (current_example >= 0 && examples::at(current_example).shutdown) examples::at(current_example).shutdown(state);
    app::clear_world(state);
}

static void select_example(int index) {
    if (index == current_example) return;
    end_example();
    current_example = index;
    state.paused = false;
    state.accumulator = 0;
    state.click = {};
    state.body_colors.length = 0;
    state.camera = camera::create();
    // each example starts from these, changing what it shows in its init
    state.drawing = {true, true, false, false, false, false, true, debug::COLOR_MOTION_TYPE, true};
    state.alpha = 1.0f;
    examples::at(current_example).init(state);
}

static void init() {
    sg_desc desc = {};
    desc.environment = sglue_environment();
    desc.logger.func = slog_func;
    sg_setup(&desc);

    simgui_desc_t imgui = {};
    imgui.logger.func = slog_func;
    simgui_setup(&imgui);

    stm_setup();
    dispatch::init();

    state.renderer = renderer::create();
    state.debug = debug::create();
    state.step_rate = 60;

    select_example(initial_example);
}

/** sleeping bodies darker, statics grey: the debug colour modes, for solid meshes */
static Color body_color(const rigid_body::RigidBody &body) {
    const uint32_t index = body_id::index(body.id);
    Color color;
    if (index < state.body_colors.length && state.body_colors[index].a > 0) {
        color = state.body_colors[index];
    } else if (body.motion_type == motion_type::MOTION_STATIC) {
        color = {0.45f, 0.45f, 0.47f, 1.0f};
    } else if (body.motion_type == motion_type::MOTION_KINEMATIC) {
        color = {0.55f, 0.75f, 0.95f, 1.0f};
    } else {
        const uint32_t hash = (index + 1) * 2654435761u;
        color = {0.45f + 0.5f * (float)((hash >> 8) & 255) / 255.0f, 0.45f + 0.5f * (float)((hash >> 16) & 255) / 255.0f,
                 0.45f + 0.5f * (float)((hash >> 24) & 255) / 255.0f, 1.0f};
    }
    if (body.sleeping && body.motion_type == motion_type::MOTION_DYNAMIC) color = {color.r * 0.6f, color.g * 0.6f, color.b * 0.6f, color.a};
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

static void draw_world(const world::World &world) {
    const Drawing &drawing = state.drawing;
    if (drawing.grid) draw_grid();
    if (drawing.meshes) {
        for (uint32_t i = 0; i < world.bodies.pool.length; i++) {
            if (world.bodies.sequences[i] == bodies::SLOT_FREE) continue;
            const rigid_body::RigidBody &body = *world.bodies.pool[i];
            const GpuMesh mesh = renderer::mesh_of(state.renderer, body.shape);
            const BodyPose drawn = app::pose(state, body);
            renderer::mesh(state.renderer, mesh, matrix::compose(drawn.position, drawn.quaternion), body_color(body));
        }
    }
    debug::clear(*state.debug);
    if (drawing.wireframes) {
        debug::BodiesOptions options = {};
        options.color_mode = drawing.color_mode;
        options.show_linear_velocity = drawing.velocities;
        options.show_angular_velocity = drawing.velocities;
        debug::bodies(*state.debug, world, options);
    }
    if (drawing.contacts) debug::contacts(*state.debug, world);
    if (drawing.contact_manifolds) debug::contact_constraints(*state.debug, world);
    if (drawing.joints) debug::joints(*state.debug, world, {});
    renderer::debug(state.renderer.frame, *state.debug);
}

/** every body's pose as a step begins, what app::pose blends from */
static void take_step_poses(App &app) {
    const bodies::Bodies &bodies = app.world->bodies;
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

static void save_screenshot() {
#if defined(__EMSCRIPTEN__)
    // the page holds this frame, and the headless browser takes the picture
    state.paused = true;
    EM_ASM({ window.labsScreenshotReady = true; });
    return;
#else
    const int width = sapp_width();
    const int height = sapp_height();
    uint8_t *pixels = (uint8_t *)malloc((size_t)width * height * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    FILE *file = fopen(screenshot_path, "wb");
    if (file) {
        fprintf(file, "P6\n%d %d\n255\n", width, height);
        for (int y = height - 1; y >= 0; y--) fwrite(pixels + (size_t)y * width * 3, 1, (size_t)width * 3, file);
        fclose(file);
    }
    free(pixels);
#endif
    sapp_request_quit();
}

static void ui(const Example &example) {
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
                    state.step_milliseconds, sapp_frame_duration() * 1000.0);
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
            if (ImGui::Combo("wireframe colour", &mode, modes, 4)) drawing.color_mode = (debug::BodyColorMode)mode;
        }
    }
    ImGui::Separator();
    if (example.ui) example.ui(state);
    ImGui::End();
}

static void frame() {
    const float delta_time = frame_time > 0.0f ? frame_time : (float)sapp_frame_duration();

    simgui_frame_desc_t imgui_frame = {};
    imgui_frame.width = sapp_width();
    imgui_frame.height = sapp_height();
    imgui_frame.delta_time = sapp_frame_duration();
    imgui_frame.dpi_scale = sapp_dpi_scale();
    simgui_new_frame(&imgui_frame);

    camera::update(state.camera, sapp_widthf(), sapp_heightf());
    state.pointer.ray = camera::screen_ray(state.camera, state.pointer.x, state.pointer.y);
    renderer::clear(state.renderer);

    const Example &example = examples::at(current_example);
    // screenshots step a fixed time a frame, the same picture on every machine
    step_world(example, screenshot_path != nullptr ? 1.0f / state.step_rate : delta_time);
    // paused, or for a screenshot, the step itself
    const bool blending = state.drawing.interpolate && !state.paused && screenshot_path == nullptr;
    state.alpha = blending ? state.accumulator * state.step_rate : 1.0f;
    if (example.update) example.update(state, delta_time);
    state.click = {};
    if (state.world != nullptr) draw_world(*state.world);

    ui(examples::at(current_example));

    sg_pass pass = {};
    pass.action.colors[0].load_action = SG_LOADACTION_CLEAR;
    pass.action.colors[0].clear_value = {0.1f, 0.1f, 0.11f, 1.0f};
    pass.swapchain = sglue_swapchain();
    sg_begin_pass(&pass);
    renderer::render(state.renderer, state.camera);
    simgui_render();
    sg_end_pass();
    sg_commit();

    frame_count++;
    if (screenshot_path && frame_count == screenshot_frame) save_screenshot();
}

static void cleanup() {
    end_example();
    state.shapes.release();
    state.hulls.release();
    state.mesh_data.release();
    state.body_colors.release();
    state.step_poses.release();
    state.step_sequences.release();
    debug::release(state.debug);
    renderer::destroy(state.renderer);
    simgui_shutdown();
    sg_shutdown();
}

static void event(const sapp_event *e) {
    if (simgui_handle_event(e) && ImGui::GetIO().WantCaptureMouse) {
        input.down = false;
        return;
    }

    switch (e->type) {
        case SAPP_EVENTTYPE_MOUSE_DOWN:
            input = {true, false, e->mouse_button, e->mouse_x, e->mouse_y, e->mouse_x, e->mouse_y};
            break;
        case SAPP_EVENTTYPE_MOUSE_MOVE: {
            const float dx = e->mouse_x - input.x;
            const float dy = e->mouse_y - input.y;
            input.x = e->mouse_x;
            input.y = e->mouse_y;
            state.pointer.x = e->mouse_x;
            state.pointer.y = e->mouse_y;
            if (!input.down) break;
            const float moved_x = e->mouse_x - input.down_x;
            const float moved_y = e->mouse_y - input.down_y;
            if (moved_x * moved_x + moved_y * moved_y > 16) input.dragging = true;
            if (!input.dragging) break;
            if (input.button == SAPP_MOUSEBUTTON_LEFT && (e->modifiers & SAPP_MODIFIER_SHIFT) == 0) {
                camera::orbit(state.camera, dx, dy);
            } else {
                camera::pan(state.camera, dx, dy);
            }
            break;
        }
        case SAPP_EVENTTYPE_MOUSE_UP:
            if (input.down && !input.dragging) {
                state.click = {true, e->mouse_button != SAPP_MOUSEBUTTON_LEFT, (e->modifiers & SAPP_MODIFIER_SHIFT) != 0,
                               camera::screen_ray(state.camera, e->mouse_x, e->mouse_y)};
            }
            input.down = false;
            break;
        case SAPP_EVENTTYPE_KEY_DOWN:
            if (!ImGui::GetIO().WantCaptureKeyboard && e->key_code >= 0 && e->key_code < 512) state.keys[e->key_code] = true;
            break;
        case SAPP_EVENTTYPE_KEY_UP:
            if (e->key_code >= 0 && e->key_code < 512) state.keys[e->key_code] = false;
            break;
        case SAPP_EVENTTYPE_MOUSE_SCROLL:
            camera::zoom(state.camera, e->scroll_y);
            break;
        default:
            break;
    }
}

sapp_desc sokol_main(int argc, char **argv) {
    for (int arg = 1; arg < argc; arg++) {
        if (strcmp(argv[arg], "--screenshot") == 0 && arg + 1 < argc) {
            screenshot_path = argv[++arg];
            continue;
        }
        if (strcmp(argv[arg], "--frames") == 0 && arg + 1 < argc) {
            screenshot_frame = atoi(argv[++arg]);
            continue;
        }
        if (strcmp(argv[arg], "--frame-time") == 0 && arg + 1 < argc) {
            frame_time = (float)atof(argv[++arg]);
            continue;
        }
        for (int i = 0; i < examples::count(); i++) {
            if (strcmp(examples::at(i).name, argv[arg]) == 0) initial_example = i;
        }
    }

    sapp_desc desc = {};
    desc.init_cb = init;
    desc.frame_cb = frame;
    desc.cleanup_cb = cleanup;
    desc.event_cb = event;
    desc.width = 1280;
    desc.height = 800;
    desc.sample_count = 4;
    desc.high_dpi = true;
    desc.window_title = "tumble labs";
    desc.logger.func = slog_func;
    return desc;
}
