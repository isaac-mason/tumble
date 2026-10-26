#include "app/app.h"
#include "app/examples.h"
#include "app/model.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/collision/gjk.h"
#include "tumble/physics/collision/support.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/sphere.h"

using namespace tumble;

enum ShapeOption : int {
    OPTION_SPHERE,
    OPTION_BOX,
    OPTION_CAPSULE,
    OPTION_TRIANGLE,
    OPTION_CONVEX_HULL,
    OPTION_COUNT,
};

static const char *const SHAPE_OPTION_NAMES[OPTION_COUNT] = {"Sphere", "Box", "Capsule", "Triangle", "Convex Hull (Suzanne)"};

static const char *const CONVEX_RADIUS_MODE_NAMES[] = {"Exclude Convex Radius", "Include Convex Radius", "Default"};
static const SupportFunctionMode CONVEX_RADIUS_MODES[] = {SupportFunctionMode::EXCLUDE_CONVEX_RADIUS, SupportFunctionMode::INCLUDE_CONVEX_RADIUS,
                                                                   SupportFunctionMode::DEFAULT};

static constexpr int MINKOWSKI_SAMPLE_COUNT = 100;
static const Float3 TRIANGLE[3] = {{0.0f, 0.0f, 0.0f}, {1.5f, 0.0f, 0.0f}, {0.75f, 1.3f, 0.0f}};

namespace {

/** what crashcat's transform gizmo edits: a position, and a rotation as xyz euler degrees */
struct Pose {
    Float3 position;
    Float3 rotation_degrees;
};

}  // namespace

static struct {
    /** null for the triangle, which is a support of its own and no shape */
    Shape *shapes[OPTION_COUNT];
    int shape_a;
    int shape_b;
    int convex_radius_mode;
    Pose pose_a;
    Pose pose_b;
    Support support_a;
    Support support_b;
    Float3 minkowski[MINKOWSKI_SAMPLE_COUNT];
    GjkClosestPoints result;
    bool colliding;
} state;

static Color hex(uint32_t rgb, float alpha = 1.0f) {
    return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, alpha};
}

static Float3 to_float3(Vec3f v) {
    Float3 out;
    vec3f::store(out, v);
    return out;
}

static Float4 quaternion_of(const Pose &pose) {
    Float4 out;
    quatf::store(out, quatf::from_euler({pose.rotation_degrees.x * scalar::DEGREES_TO_RADIANS, pose.rotation_degrees.y * scalar::DEGREES_TO_RADIANS,
                                         pose.rotation_degrees.z * scalar::DEGREES_TO_RADIANS, EulerOrder::XYZ}));
    return out;
}

static void edit_pose(const char *label, Pose &pose) {
    ImGui::PushID(label);
    ImGui::TextUnformatted(label);
    ImGui::DragFloat3("position", &pose.position.x, 0.02f);
    ImGui::DragFloat3("rotation (deg)", &pose.rotation_degrees.x, 0.5f);
    ImGui::PopID();
}

static void fill_support(Support &out, int option) {
    if (option == OPTION_TRIANGLE) {
        support::set_triangle_support(out, vec3f::load(TRIANGLE[0]), vec3f::load(TRIANGLE[1]), vec3f::load(TRIANGLE[2]), 0.0f);
    } else {
        shapes::set_shape_support(out, state.shapes[option], CONVEX_RADIUS_MODES[state.convex_radius_mode], vec3f::splat(1.0f));
    }
}

/** crashcat's setup: fresh supports, both shapes back where they started */
static void setup() {
    fill_support(state.support_a, state.shape_a);
    fill_support(state.support_b, state.shape_b);
    state.pose_a = {{-2.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    state.pose_b = {{2.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
}

static void init(App &app) {
    state.shape_a = OPTION_SPHERE;
    state.shape_b = OPTION_BOX;
    state.convex_radius_mode = 1;
    state.shapes[OPTION_SPHERE] = app::keep(app, sphere::create({.radius = 1.0f}));
    state.shapes[OPTION_BOX] = app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}));
    state.shapes[OPTION_CAPSULE] = app::keep(app, capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f}));
    state.shapes[OPTION_TRIANGLE] = nullptr;
    Model suzanne = model::load("suzi.glb");
    state.shapes[OPTION_CONVEX_HULL] = app::keep(app, convex_hull::create({.positions = suzanne.positions.data, .num_positions = suzanne.positions.length / 3, .convex_radius = 0.0f}));
    model::release(suzanne);
    setup();
    camera::look_at(app.camera, {5.0f, 5.0f, 5.0f}, {0.0f, 0.0f, 0.0f});
}

/** a point on a + (-b) furthest along the direction: a point on the minkowski difference's surface */
static Vec3f minkowski_support(Vec3f direction) {
    const Vec3f on_a = state.support_a.get_support(state.support_a, direction);
    const Vec3f on_b = state.support_b.get_support(state.support_b, vec3f::negate(direction));
    return vec3f::subtract(on_a, on_b);
}

static void draw_shape(App &app, int option, const Pose &pose, Color color) {
    const Float4 quaternion = quaternion_of(pose);
    if (option == OPTION_TRIANGLE) {
        Float3 corners[3];
        for (int i = 0; i < 3; i++) corners[i] = vec::add(pose.position, vec::rotate(quaternion, TRIANGLE[i]));
        renderer::triangle(app::frame(app), corners[0], corners[1], corners[2], color);
        return;
    }
    renderer::mesh(app.renderer, renderer::mesh_of(app.renderer, state.shapes[option]), matrix::compose(pose.position, quaternion), color);
}

static void update(App &app, float) {
    // each shape's pose onto its support
    state.support_a.has_transform = true;
    state.support_a.transform = mat4f::from_rotation_translation(quatf::load(quaternion_of(state.pose_a)), vec3f::load(state.pose_a.position));
    state.support_b.has_transform = true;
    state.support_b.transform = mat4f::from_rotation_translation(quatf::load(quaternion_of(state.pose_b)), vec3f::load(state.pose_b.position));

    // a fibonacci sphere of directions
    const float golden_angle = 2.0f * scalar::PI * (1.0f + sqrtf(5.0f)) / 2.0f;
    for (int i = 0; i < MINKOWSKI_SAMPLE_COUNT; i++) {
        const float phi = acosf(1.0f - 2.0f * (float)i / (float)MINKOWSKI_SAMPLE_COUNT);
        const float theta = golden_angle * (float)i;
        const Vec3f direction = vec3f::make(sinf(phi) * cosf(theta), sinf(phi) * sinf(theta), cosf(phi));
        state.minkowski[i] = to_float3(minkowski_support(direction));
    }

    const float tolerance = 1e-4f;
    state.result = gjk::create_gjk_closest_points();
    gjk::closest_points(state.result, state.support_a, state.support_b, tolerance, vec3f::axis_x(), scalar::FLOAT_MAX);
    state.colliding = state.result.squared_distance <= tolerance * tolerance;

    const Color shape_color = hex(state.colliding ? 0x00ff00 : 0xff0000);
    draw_shape(app, state.shape_a, state.pose_a, shape_color);
    draw_shape(app, state.shape_b, state.pose_b, shape_color);

    DrawList &frame = app::frame(app);
    renderer::point(frame, {0.0f, 0.0f, 0.0f}, 0.15f, hex(0xff0000));
    for (int i = 0; i < MINKOWSKI_SAMPLE_COUNT; i++) renderer::point(frame, state.minkowski[i], 0.03f, hex(0x00ffff, 0.6f));

    const Simplex &simplex = state.result.simplex;
    Float3 corners[4];
    for (uint32_t i = 0; i < simplex.size; i++) {
        corners[i] = to_float3(simplex.y[i]);
        renderer::point(frame, corners[i], 0.04f, hex(0xffff00, 0.5f));
    }
    // every edge of the simplex: a segment, a triangle or a tetrahedron
    for (uint32_t i = 0; i < simplex.size; i++) {
        for (uint32_t j = i + 1; j < simplex.size; j++) renderer::line(frame, corners[i], corners[j], hex(0xffff00));
    }
}

static void ui(App &) {
    bool changed = ImGui::Combo("Shape A Type", &state.shape_a, SHAPE_OPTION_NAMES, OPTION_COUNT);
    changed |= ImGui::Combo("Shape B Type", &state.shape_b, SHAPE_OPTION_NAMES, OPTION_COUNT);
    changed |= ImGui::Combo("Convex Radius Mode", &state.convex_radius_mode, CONVEX_RADIUS_MODE_NAMES, 3);
    if (changed) setup();
    edit_pose("shape A", state.pose_a);
    edit_pose("shape B", state.pose_b);

    ImGui::Separator();
    ImGui::Text("GJK Collision Detection");
    ImGui::TextColored(state.colliding ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1), "%s, distance %.3f, simplex of %u",
                       state.colliding ? "colliding" : "not colliding", sqrtf(state.result.squared_distance), state.result.simplex.size);
    ImGui::Text("Shapes:");
    ImGui::Text("Sphere, Box, Triangle");
    ImGui::Text("Convex Hull (Suzanne): 3D model loaded from GLB");
    ImGui::Text("Visualization:");
    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Green: Colliding");
    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Red: Not colliding");
    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Yellow points/lines: Final simplex");
    ImGui::TextColored(ImVec4(0, 1, 1, 1), "Cyan points: Minkowski difference (A - B)");
    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Red sphere: Origin");
    ImGui::Text("Convex Radius Modes:");
    ImGui::Text("Exclude: GJK mode - radius stored separately");
    ImGui::Text("Include: Ray cast mode - radius in surface points");
    ImGui::Text("Default: Same as Exclude");
}

static const Example EXAMPLE_GJK = {"GJK Collision Detection", init, nullptr, update, ui, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_GJK);
