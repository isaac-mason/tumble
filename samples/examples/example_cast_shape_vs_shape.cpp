#include "app/app.h"
#include "app/examples.h"
#include "app/model.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/collision/narrowphase.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/static_compound.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"

using namespace tumble;

enum ShapeOption : int {
    OPTION_SPHERE,
    OPTION_BOX,
    OPTION_CAPSULE,
    OPTION_TRIANGLE_MESH,
    OPTION_COMPOUND,
    OPTION_STATIC_COMPOUND,
    OPTION_CONVEX_HULL,
    OPTION_TRANSFORMED_BOX,
    OPTION_COUNT,
};

static const char *const SHAPE_OPTION_NAMES[OPTION_COUNT] = {
    "Sphere", "Box", "Capsule", "Triangle Mesh", "Compound (Two Boxes)", "Static Compound (Two Boxes)", "Convex Hull (Suzanne)",
    "Transformed (Rotated Box)",
};
/** crashcat's config values, which its info panel shows */
static const char *const SHAPE_OPTION_IDS[OPTION_COUNT] = {
    "sphere", "box", "capsule", "triangle-mesh", "compound-two-boxes", "static-compound-two-boxes", "convexhull", "transformed-box",
};

static const char *const COLLECTOR_NAMES[] = {"Closest", "Any", "All"};
static const CollectorKind COLLECTOR_KINDS[] = {CollectorKind::CLOSEST, CollectorKind::ANY, CollectorKind::ALL};

namespace {

/** what crashcat's transform gizmo edits: a position, and a rotation as xyz euler degrees */
struct Pose {
    Float3 position;
    Float3 rotation_degrees;
};

}  // namespace

static struct {
    Shape *shapes[OPTION_COUNT];
    int shape_a;
    int shape_b;
    int collector_mode;
    bool collide_with_backfaces_triangles;
    bool collide_with_backfaces_convex;
    Pose pose_a;
    Pose pose_b;
    Float3 displacement_endpoint;
    CastShapeCollector collector;
    /** the hit shown: the collector's, or for all, its first */
    const CastShapeHit *hit;
} state;

static Color hex(uint32_t rgb, float alpha = 1.0f) {
    return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, alpha};
}

static Float3 to_float3(Vec3f v) {
    Float3 out;
    vec3f::store(out, v);
    return out;
}

static Float4 quat_from_degrees(float x, float y, float z, EulerOrder order) {
    Float4 out;
    quatf::store(out, quatf::from_euler({x * scalar::DEGREES_TO_RADIANS, y * scalar::DEGREES_TO_RADIANS, z * scalar::DEGREES_TO_RADIANS, order}));
    return out;
}

static Float4 quaternion_of(const Pose &pose) { return quat_from_degrees(pose.rotation_degrees.x, pose.rotation_degrees.y, pose.rotation_degrees.z, EulerOrder::XYZ); }

static Transformf transform_of(const Pose &pose) { return {pose.position, quaternion_of(pose), {1.0f, 1.0f, 1.0f}}; }

static void edit_pose(const char *label, Pose &pose) {
    ImGui::PushID(label);
    ImGui::TextUnformatted(label);
    ImGui::DragFloat3("position", &pose.position.x, 0.02f);
    ImGui::DragFloat3("rotation (deg)", &pose.rotation_degrees.x, 0.5f);
    ImGui::PopID();
}

static Shape *two_boxes(App &app, bool is_static) {
    Shape *upper = app::keep(app, box::create({.half_extents = {0.8f, 0.5f, 0.8f}}));
    Shape *lower = app::keep(app, box::create({.half_extents = {0.6f, 0.7f, 0.6f}}));
    const CompoundShapeSettings::Child children[] = {
        {.position = {-1.2f, 0.8f, 0.0f}, .quaternion = quat_from_degrees(0.0f, 45.0f, 0.0f, EulerOrder::ZYX), .shape = upper},
        {.position = {1.0f, -0.6f, 0.0f}, .quaternion = quat_from_degrees(30.0f, -30.0f, 15.0f, EulerOrder::ZYX), .shape = lower},
    };
    if (is_static) return app::keep(app, static_compound::create({.children = children, .num_children = 2}));
    return app::keep(app, compound::create({.children = children, .num_children = 2}));
}

static void make_shapes(App &app) {
    state.shapes[OPTION_SPHERE] = app::keep(app, sphere::create({.radius = 1.0f}));
    state.shapes[OPTION_BOX] = app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}));
    state.shapes[OPTION_CAPSULE] = app::keep(app, capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f}));

    // suzanne is both the triangle mesh and the hull here
    Model suzanne = model::load("suzi.glb");
    state.shapes[OPTION_TRIANGLE_MESH] = app::keep(app, triangle_mesh::create({.positions = suzanne.positions.data,
                                                                                                   .num_positions = suzanne.positions.length / 3,
                                                                                                   .indices = suzanne.indices.data,
                                                                                                   .num_indices = suzanne.indices.length}));
    state.shapes[OPTION_CONVEX_HULL] = app::keep(app, convex_hull::create({.positions = suzanne.positions.data, .num_positions = suzanne.positions.length / 3, .convex_radius = 0.0f}));
    model::release(suzanne);

    state.shapes[OPTION_COMPOUND] = two_boxes(app, false);
    state.shapes[OPTION_STATIC_COMPOUND] = two_boxes(app, true);

    Shape *rotated = app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}));
    state.shapes[OPTION_TRANSFORMED_BOX] = app::keep(
        app, transformed::create({.shape = rotated, .position = {0.0f, 0.0f, 0.0f}, .quaternion = quat_from_degrees(25.0f, 30.0f, 45.0f, EulerOrder::ZYX)}));
}

/** crashcat's setup: both shapes back where they started, the sweep 10 m along x from a */
static void reset_poses() {
    state.pose_a = {{-3.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    state.pose_b = {{3.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    state.displacement_endpoint = vec::add(state.pose_a.position, {10.0f, 0.0f, 0.0f});
}

static void init(App &app) {
    state.shape_a = OPTION_SPHERE;
    state.shape_b = OPTION_SPHERE;
    state.collector_mode = 0;
    state.collide_with_backfaces_triangles = false;
    state.collide_with_backfaces_convex = false;
    state.collector = cast_shape::create_collector(CollectorKind::CLOSEST);
    state.hit = nullptr;
    make_shapes(app);
    reset_poses();
    camera::look_at(app.camera, {5.0f, 5.0f, 8.0f}, {0.0f, 0.0f, 0.0f});
}

static void draw_face(DrawList &frame, const SupportingFace &face, Color color) {
    for (uint32_t i = 0; i < face.num_vertices; i++) {
        renderer::line(frame, to_float3(face.vertices[i]), to_float3(face.vertices[(i + 1) % face.num_vertices]), color);
    }
}

static void update(App &app, float) {
    const Transformf transform_a = transform_of(state.pose_a);
    const Transformf transform_b = transform_of(state.pose_b);
    const Shape *shape_a = state.shapes[state.shape_a];
    const Shape *shape_b = state.shapes[state.shape_b];
    const Vec3f start = vec3f::load(state.pose_a.position);
    const Vec3f displacement = vec3f::subtract(vec3f::load(state.displacement_endpoint), start);

    CastShapeSettings settings = cast_shape::default_settings();
    settings.collect_faces = true;
    settings.collide_with_backfaces_triangles = state.collide_with_backfaces_triangles;
    settings.collide_with_backfaces_convex = state.collide_with_backfaces_convex;
    settings.active_edge_movement_direction = displacement;

    cast_shape::reset(state.collector);
    state.collector.kind = COLLECTOR_KINDS[state.collector_mode];
    narrowphase::cast_shape_vs_shape(state.collector, settings, shape_a, sub_shape::builder(), transform_a, displacement, shape_b,
                                     sub_shape::builder(), transform_b);
    state.hit = state.collector.hits.length > 0 && state.collector.hits[0].status == CastShapeStatus::COLLIDING ? &state.collector.hits[0] : nullptr;

    DrawList &frame = app::frame(app);
    const Mat4 model_a = matrix::compose(transform_a.position, transform_a.quaternion);
    renderer::mesh(app.renderer, renderer::mesh_of(app.renderer, shape_a), model_a, hex(state.hit ? 0x00ff00 : 0x4488ff));
    renderer::mesh(app.renderer, renderer::mesh_of(app.renderer, shape_b), matrix::compose(transform_b.position, transform_b.quaternion),
                   hex(state.hit ? 0x00ff00 : 0xff8844));
    renderer::point(frame, state.displacement_endpoint, 0.2f, hex(0x00ff00));
    renderer::line(frame, state.pose_a.position, state.displacement_endpoint, hex(0x00ff00));

    if (state.hit == nullptr) return;
    const CastShapeHit &hit = *state.hit;
    const Vec3f hit_position = vec3f::scale_and_add(start, displacement, hit.fraction);
    renderer::mesh(app.renderer, renderer::mesh_of(app.renderer, shape_a), matrix::compose(to_float3(hit_position), transform_a.quaternion),
                   hex(0xffff00, 0.3f));
    draw_face(frame, hit.face_a, hex(0xff0000));
    draw_face(frame, hit.face_b, hex(0x00ff00));
    const float arrow_length = 1.5f;
    renderer::line(frame, to_float3(hit_position), to_float3(vec3f::scale_and_add(hit_position, hit.penetration_axis, arrow_length)), hex(0x00ffff));
    renderer::line(frame, to_float3(hit_position), to_float3(vec3f::scale_and_add(hit_position, hit.normal, arrow_length)), hex(0x00ff00));
}

static void ui(App &) {
    bool changed = ImGui::Combo("Shape A Type", &state.shape_a, SHAPE_OPTION_NAMES, OPTION_COUNT);
    changed |= ImGui::Combo("Shape B Type", &state.shape_b, SHAPE_OPTION_NAMES, OPTION_COUNT);
    if (changed) reset_poses();
    ImGui::Combo("Collector Mode", &state.collector_mode, COLLECTOR_NAMES, 3);
    ImGui::Checkbox("Collide With Triangle Backfaces", &state.collide_with_backfaces_triangles);
    ImGui::Checkbox("Collide With Convex Backfaces", &state.collide_with_backfaces_convex);
    edit_pose("shape A", state.pose_a);
    edit_pose("shape B", state.pose_b);
    ImGui::PushID("displacement");
    ImGui::TextUnformatted("displacement endpoint");
    ImGui::DragFloat3("position", &state.displacement_endpoint.x, 0.02f);
    ImGui::PopID();

    ImGui::Separator();
    const Float3 &a = state.pose_a.position;
    const Float3 &b = state.pose_b.position;
    const Float3 displacement = vec::subtract(state.displacement_endpoint, a);
    ImGui::Text("Continuous Collision Detection");
    ImGui::Text("Shape A: %s", SHAPE_OPTION_IDS[state.shape_a]);
    ImGui::Text("Position: (%.3f, %.3f, %.3f)", a.x, a.y, a.z);
    ImGui::Text("Shape B: %s", SHAPE_OPTION_IDS[state.shape_b]);
    ImGui::Text("Position: (%.3f, %.3f, %.3f)", b.x, b.y, b.z);
    ImGui::Text("Displacement: (%.3f, %.3f, %.3f)", displacement.x, displacement.y, displacement.z);
    ImGui::Text("Length: %.3f", vec::length(displacement));
    if (state.hit == nullptr) {
        ImGui::Text("No collision");
        return;
    }
    const CastShapeHit &hit = *state.hit;
    const Float3 hit_position = vec::scale_and_add(a, displacement, hit.fraction);
    ImGui::Text("COLLISION DETECTED");
    ImGui::Text("Collision Fraction: %.3f", hit.fraction);
    ImGui::Text("(0 = overlapping now, 1 = at end of displacement)");
    ImGui::Text("Hit Position: (%.3f, %.3f, %.3f)", hit_position.x, hit_position.y, hit_position.z);
    ImGui::TextColored(ImVec4(0, 1, 1, 1), "Penetration Axis: (%.2f, %.2f, %.2f)", vec3f::x(hit.penetration_axis), vec3f::y(hit.penetration_axis),
                       vec3f::z(hit.penetration_axis));
    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Normal: (%.2f, %.2f, %.2f)", vec3f::x(hit.normal), vec3f::y(hit.normal), vec3f::z(hit.normal));
    ImGui::Text("Face Extraction:");
    ImGui::Text("Face A vertices: %u", hit.face_a.num_vertices);
    ImGui::Text("Face B vertices: %u", hit.face_b.num_vertices);
    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Red lines = Face A");
    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Green lines = Face B");
}

static void shutdown(App &) { state.collector = {}; }

static const Example EXAMPLE_CAST_SHAPE_VS_SHAPE = {"Cast Shape vs Shape", init, nullptr, update, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_CAST_SHAPE_VS_SHAPE);
