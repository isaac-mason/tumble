#include "app/app.h"
#include "app/examples.h"
#include "app/model.h"
#include "app/random.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/physics/shapes/utils/convex_hull_builder.h"

using namespace tumble;

enum InputShape : int {
    INPUT_CUBE,
    INPUT_ICOSPHERE,
    INPUT_RANDOM_SPHERE,
    INPUT_RANDOM_CUBE,
    INPUT_BUNNY,
    INPUT_CAT,
    INPUT_SUZI,
    INPUT_COUNT,
};

static const char *const INPUT_NAMES[INPUT_COUNT] = {"Cube", "Icosphere", "Random Points (Sphere)", "Random Points (Cube)",
                                                     "Bunny (GLTF)", "Cat (GLTF)", "Suzi (GLTF)"};

static const char *const RESULT_NAMES[] = {"Success", "MaxVerticesReached", "TooFewPoints", "TooFewFaces", "Degenerate"};

static struct {
    int shape;
    bool show_input_points;
    bool show_hull_wireframe;
    bool show_hull_solid;
    bool show_center_of_mass;
    float solid_opacity;
    int max_vertices;
    float tolerance;
    Random random;
    collections::Array<Vec3f> points;
    ConvexHullBuilder builder;
    ConvexHullBuilder::Result result;
    int32_t num_vertices;
    uint32_t num_faces;
    Vec3f center_of_mass;
    float volume;
} state;

static Color hex(uint32_t rgb, float alpha = 1.0f) {
    return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, alpha};
}

static Float3 to_float3(Vec3f v) {
    Float3 out;
    vec3f::store(out, v);
    return out;
}

static void generate_cube() {
    for (float x = -1.0f; x <= 1.0f; x += 0.5f) {
        for (float y = -1.0f; y <= 1.0f; y += 0.5f) {
            for (float z = -1.0f; z <= 1.0f; z += 0.5f) state.points.push(vec3f::make(x, y, z));
        }
    }
}

/** three.js's IcosahedronGeometry(2, 2): each face cut into 9 triangles, every triangle's corners pushed out to the sphere */
static void generate_icosphere() {
    const float t = (1.0f + sqrtf(5.0f)) / 2.0f;
    const Float3 corners[12] = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                                {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    const int faces[20][3] = {{0, 11, 5}, {0, 5, 1},  {0, 1, 7},   {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
                              {11, 10, 2}, {10, 7, 6}, {7, 1, 8},  {3, 9, 4},  {3, 4, 2},   {3, 2, 6}, {3, 6, 8},
                              {3, 8, 9},  {4, 9, 5},  {2, 4, 11}, {6, 2, 10}, {8, 6, 7},   {9, 8, 1}};
    constexpr int COLUMNS = 3;
    const auto lerp = [](Float3 a, Float3 b, float s) { return vec::add(a, vec::scale(vec::subtract(b, a), s)); };
    const auto push = [](Float3 p) { state.points.push(vec3f::load(vec::scale(vec::normalize(p), 2.0f))); };
    for (const int *face : faces) {
        const Float3 a = corners[face[0]], b = corners[face[1]], c = corners[face[2]];
        Float3 grid[COLUMNS + 1][COLUMNS + 1];
        for (int i = 0; i <= COLUMNS; i++) {
            const Float3 aj = lerp(a, c, (float)i / COLUMNS);
            const Float3 bj = lerp(b, c, (float)i / COLUMNS);
            const int rows = COLUMNS - i;
            for (int j = 0; j <= rows; j++) grid[i][j] = j == 0 && i == COLUMNS ? aj : lerp(aj, bj, (float)j / (float)rows);
        }
        for (int i = 0; i < COLUMNS; i++) {
            for (int j = 0; j < 2 * (COLUMNS - i) - 1; j++) {
                const int k = j / 2;
                if (j % 2 == 0) {
                    push(grid[i][k + 1]);
                    push(grid[i + 1][k]);
                    push(grid[i][k]);
                } else {
                    push(grid[i][k + 1]);
                    push(grid[i + 1][k + 1]);
                    push(grid[i + 1][k]);
                }
            }
        }
    }
}

static void generate_random_in_sphere(int count, float radius) {
    for (int i = 0; i < count; i++) {
        float x, y, z;
        do {
            x = (seeded::next(state.random) - 0.5f) * 2.0f * radius;
            y = (seeded::next(state.random) - 0.5f) * 2.0f * radius;
            z = (seeded::next(state.random) - 0.5f) * 2.0f * radius;
        } while (x * x + y * y + z * z > radius * radius);
        state.points.push(vec3f::make(x, y, z));
    }
}

static void generate_random_in_cube(int count, float size) {
    for (int i = 0; i < count; i++) {
        const float x = (seeded::next(state.random) - 0.5f) * 2.0f * size;
        const float y = (seeded::next(state.random) - 0.5f) * 2.0f * size;
        const float z = (seeded::next(state.random) - 0.5f) * 2.0f * size;
        state.points.push(vec3f::make(x, y, z));
    }
}

static void load_points(const char *name) {
    Model model = model::load(name);
    for (uint32_t i = 0; i + 2 < model.positions.length; i += 3) {
        state.points.push(vec3f::make(model.positions[i], model.positions[i + 1], model.positions[i + 2]));
    }
    model::release(model);
}

/** crashcat's updateVisualization: new input points, a new hull */
static void rebuild() {
    state.points.length = 0;
    switch (state.shape) {
        case INPUT_CUBE: generate_cube(); break;
        case INPUT_ICOSPHERE: generate_icosphere(); break;
        case INPUT_RANDOM_SPHERE: generate_random_in_sphere(100, 2.0f); break;
        case INPUT_RANDOM_CUBE: generate_random_in_cube(100, 2.0f); break;
        case INPUT_BUNNY: load_points("bunny.glb"); break;
        case INPUT_CAT: load_points("cat.gltf"); break;
        case INPUT_SUZI: load_points("suzi.glb"); break;
    }
    state.builder = convex_hull_builder::create(state.points.data, state.points.length);
    state.result = convex_hull_builder::initialize(state.builder, state.max_vertices, state.tolerance);
    if (state.result != ConvexHullBuilder::Result::SUCCESS) return;
    state.num_vertices = convex_hull_builder::get_num_vertices_used(state.builder);
    state.num_faces = 0;
    for (uint32_t i = 0; i < state.builder.faces.length; i++) {
        if (!state.builder.face_pool[state.builder.faces[i]].removed) state.num_faces++;
    }
    convex_hull_builder::get_center_of_mass_and_volume(state.builder, state.center_of_mass, state.volume);
}

static void init(App &app) {
    state.shape = INPUT_CUBE;
    state.show_input_points = true;
    state.show_hull_wireframe = true;
    state.show_hull_solid = true;
    state.show_center_of_mass = true;
    state.solid_opacity = 0.3f;
    state.max_vertices = 1000;
    state.tolerance = 1e-4f;
    state.random = {1};
    state.points.length = 0;
    state.builder = {};
    rebuild();
    camera::look_at(app.camera, {5.0f, 5.0f, 8.0f}, {0.0f, 0.0f, 0.0f});
}

static void update(App &app, float) {
    DrawList &frame = app::frame(app);
    if (state.show_input_points) {
        for (uint32_t i = 0; i < state.points.length; i++) renderer::point(frame, to_float3(state.points[i]), 0.025f, hex(0x00ffff));
    }
    if (state.result != ConvexHullBuilder::Result::SUCCESS) return;

    const ConvexHullBuilder &builder = state.builder;
    const Color solid = hex(0x44ff44, state.solid_opacity);
    for (uint32_t i = 0; i < builder.faces.length; i++) {
        const ConvexHullBuilder::Face &face = builder.face_pool[builder.faces[i]];
        if (face.removed || face.first_edge < 0) continue;
        // the polygon's edges, and a fan of triangles over it
        const int32_t first = face.first_edge;
        const Float3 start = to_float3(builder.positions[builder.edge_pool[first].start_idx]);
        int32_t edge = first;
        do {
            const int32_t next = builder.edge_pool[edge].next_edge;
            const Float3 from = to_float3(builder.positions[builder.edge_pool[edge].start_idx]);
            const Float3 to = to_float3(builder.positions[builder.edge_pool[next].start_idx]);
            if (state.show_hull_wireframe) renderer::line(frame, from, to, hex(0x00ff00));
            if (state.show_hull_solid && edge != first && next != first) renderer::triangle(frame, start, from, to, solid);
            edge = next;
        } while (edge != first);
    }

    if (state.show_center_of_mass) {
        const Float3 center = to_float3(state.center_of_mass);
        renderer::point(frame, center, 0.1f, hex(0xff0000));
        renderer::line(frame, center, vec::add(center, {0.5f, 0.0f, 0.0f}), hex(0xff0000));
        renderer::line(frame, center, vec::add(center, {0.0f, 0.5f, 0.0f}), hex(0x00ff00));
        renderer::line(frame, center, vec::add(center, {0.0f, 0.0f, 0.5f}), hex(0x0000ff));
    }
}

static void ui(App &) {
    bool changed = ImGui::Combo("shape", &state.shape, INPUT_NAMES, INPUT_COUNT);
    changed |= ImGui::Checkbox("showInputPoints", &state.show_input_points);
    changed |= ImGui::Checkbox("showHullWireframe", &state.show_hull_wireframe);
    changed |= ImGui::Checkbox("showHullSolid", &state.show_hull_solid);
    changed |= ImGui::Checkbox("showCenterOfMass", &state.show_center_of_mass);
    changed |= ImGui::SliderFloat("solidOpacity", &state.solid_opacity, 0.0f, 1.0f, "%.2f");
    changed |= ImGui::SliderInt("maxVertices", &state.max_vertices, 10, 5000);
    changed |= ImGui::SliderFloat("tolerance", &state.tolerance, 1e-6f, 1e-2f, "%.2e", ImGuiSliderFlags_Logarithmic);
    if (changed) rebuild();

    ImGui::Separator();
    ImGui::Text("Input Shape: %s", INPUT_NAMES[state.shape]);
    ImGui::Text("Input Points: %u", state.points.length);
    ImGui::Text("Max Vertices: %d", state.max_vertices);
    ImGui::Text("Tolerance: %.2e", state.tolerance);
    if (state.result != ConvexHullBuilder::Result::SUCCESS) {
        ImGui::TextColored(ImVec4(1.0f, 0.27f, 0.27f, 1), "Build Result: %s", RESULT_NAMES[(uint32_t)state.result]);
        return;
    }
    ImGui::TextColored(ImVec4(0.27f, 1.0f, 0.27f, 1), "Build Result: Success");
    ImGui::Text("Hull Vertices: %d", state.num_vertices);
    ImGui::Text("Hull Faces: %u", state.num_faces);
    ImGui::Text("Volume: %.4f", state.volume);
    ImGui::Text("Center of Mass: (%.2f, %.2f, %.2f)", vec3f::x(state.center_of_mass), vec3f::y(state.center_of_mass), vec3f::z(state.center_of_mass));
}

static void shutdown(App &) {
    state.builder = {};
    state.points.release();
}

static const Example EXAMPLE_CONVEX_HULL_BUILDER = {"Convex Hull Builder", init, nullptr, update, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_CONVEX_HULL_BUILDER);
