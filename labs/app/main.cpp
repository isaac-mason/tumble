#include <stdio.h>
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

/** at most this many steps a frame, so a slow step cannot snowball */
static constexpr int MAX_STEPS_PER_FRAME = 4;

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

void set_color(App &app, const rigid_body::RigidBody *body, Color color) {
    const uint32_t index = body_id::index(body->id);
    while (app.body_colors.length <= index) app.body_colors.push({0, 0, 0, 0});
    app.body_colors[index] = color;
}

DrawList &frame(App &app) { return app.renderer.frame; }

double now_milliseconds() { return stm_ms(stm_now()); }

}

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
        default: shapes::release(shape); break;
    }
}

/** the example's world, then its shapes, then what they were built from */
static void end_example() {
    if (current_example >= 0 && examples::at(current_example).shutdown) examples::at(current_example).shutdown(state);
    if (state.world != nullptr) world::destroy(state.world);
    state.world = nullptr;
    for (uint32_t i = 0; i < state.shapes.length; i++) destroy_shape(state.shapes[i]);
    for (uint32_t i = 0; i < state.hulls.length; i++) convex_hull::release(state.hulls[i]);
    for (uint32_t i = 0; i < state.mesh_data.length; i++) triangle_mesh_data::destroy(state.mesh_data[i]);
    state.shapes.length = 0;
    state.hulls.length = 0;
    state.mesh_data.length = 0;
    // a freed shape's memory may be reused by the next example's
    renderer::forget_shapes(state.renderer);
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
    state.drawing = {true, true, false, false, false, false, true, debug::COLOR_MOTION_TYPE};

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
            renderer::mesh(state.renderer, mesh, matrix::compose(body.position, body.quaternion), body_color(body));
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
        if (example.step) example.step(state, step);
        const double start = app::now_milliseconds();
        world::step(*state.world, step);
        state.step_milliseconds = app::now_milliseconds() - start;
    }
}

static void save_screenshot() {
#if !defined(__EMSCRIPTEN__)
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
    const float delta_time = (float)sapp_frame_duration();

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
