#include "app/app.h"
#include "app/examples.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/math/mat4f.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/scaled.h"
#include "tumble/physics/shapes/transformed.h"

using namespace tumble;

static constexpr int SHAPE_COUNT = 4;
static const char *const SHAPE_NAMES[SHAPE_COUNT] = {"Box", "Scaled Box", "Rotated Box", "Convex Hull (Octahedron)"};
static const char *const SHAPE_OPTION_NAMES[SHAPE_COUNT] = {"Box", "Scaled Box", "Rotated Box", "Convex Hull"};
static constexpr uint32_t MAX_FACE_POINTS = 8;

static struct {
    shapes::Shape *shapes[SHAPE_COUNT];
    int shape;
    Float3 direction_point;
    Float3 direction;
    /** the arrow keeps its last length while the point sits on the origin */
    float arrow_length;
    face::Face face;
} state;

static Color hex(uint32_t rgb, float alpha = 1.0f) {
    return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, alpha};
}

static Float3 to_float3(Vec3f v) {
    Float3 out;
    vec3f::store(out, v);
    return out;
}

static void init(App &app) {
    state.shape = 0;
    state.direction_point = {1.0f, 0.0f, 0.0f};
    state.arrow_length = 2.0f;
    state.shapes[0] = app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}));
    shapes::Shape *unit_box = app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}));
    state.shapes[1] = app::keep(app, scaled::create({.shape = unit_box, .scale = {2.0f, 1.0f, 1.0f}}));
    shapes::Shape *turned_box = app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}));
    const float half_turn = scalar::PI / 8.0f;
    state.shapes[2] = app::keep(app, transformed::create({.shape = turned_box, .position = {0.0f, 0.0f, 0.0f}, .quaternion = {0.0f, 0.0f, sinf(half_turn), cosf(half_turn)}}));
    const float octahedron[] = {0.0f, 1.5f, 0.0f, 0.0f, -1.5f, 0.0f, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -1.0f};
    convex_hull::ConvexHullData *hull = app::keep(app, convex_hull::build({.positions = octahedron, .num_positions = 6, .convex_radius = 0.0f}));
    state.shapes[3] = app::keep(app, convex_hull::create({.data = hull}));
    camera::look_at(app.camera, {4.0f, 4.0f, 4.0f}, {0.0f, 0.0f, 0.0f});
}

static void update(App &app, float) {
    const float length = vec::length(state.direction_point);
    state.direction = length > 0.0f ? vec::scale(state.direction_point, 1.0f / length) : Float3{0.0f, 0.0f, 0.0f};
    if (length > 0.0f) state.arrow_length = fminf(length, 3.0f);

    // the shapes sit at the origin, unturned
    shapes::get_shape_supporting_face(state.face, state.shapes[state.shape], sub_shape::EMPTY_SUB_SHAPE_ID, vec3f::load(state.direction),
                                      mat4f::identity(), vec3f::splat(1.0f));

    DrawList &frame = app::frame(app);
    renderer::mesh(app.renderer, renderer::mesh_of(app.renderer, state.shapes[state.shape]), matrix::identity(), hex(0x00ff00));
    renderer::point(frame, state.direction_point, 0.2f, hex(0xffff00));
    renderer::arrow(frame, {0.0f, 0.0f, 0.0f}, vec::scale(state.direction, state.arrow_length), hex(0x00ffff));
    for (uint32_t i = 0; i < state.face.num_vertices && i < MAX_FACE_POINTS; i++) {
        renderer::point(frame, to_float3(state.face.vertices[i]), 0.15f, hex(0xff00ff));
    }
    if (state.face.num_vertices >= 2) {
        for (uint32_t i = 0; i < state.face.num_vertices; i++) {
            renderer::line(frame, to_float3(state.face.vertices[i]), to_float3(state.face.vertices[(i + 1) % state.face.num_vertices]), hex(0xff00ff));
        }
    }
}

static void ui(App &) {
    ImGui::Combo("shape", &state.shape, SHAPE_OPTION_NAMES, SHAPE_COUNT);
    ImGui::PushID("direction");
    ImGui::TextUnformatted("direction point");
    ImGui::DragFloat3("position", &state.direction_point.x, 0.02f);
    ImGui::PopID();

    ImGui::Separator();
    const Float3 &direction = state.direction;
    const Float3 &point = state.direction_point;
    ImGui::Text("computeSupportingFace");
    ImGui::Text("Shape: %s", SHAPE_NAMES[state.shape]);
    ImGui::Text("Query Direction: (%.3f, %.3f, %.3f)", direction.x, direction.y, direction.z);
    ImGui::Text("Gizmo Position: (%.3f, %.3f, %.3f)", point.x, point.y, point.z);
    ImGui::Text("Face Vertices: %u", state.face.num_vertices);
    ImGui::Text("Note: Returns the face perpendicular to the query direction");
    if (state.face.num_vertices == 0) return;
    ImGui::Text("Vertex Positions:");
    for (uint32_t i = 0; i < state.face.num_vertices; i++) {
        const Float3 vertex = to_float3(state.face.vertices[i]);
        ImGui::Text("[%u]: (%.3f, %.3f, %.3f)", i, vertex.x, vertex.y, vertex.z);
    }
}

static const Example EXAMPLE_GET_SUPPORTING_FACE = {"Get Supporting Face", init, nullptr, update, ui, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_GET_SUPPORTING_FACE);
