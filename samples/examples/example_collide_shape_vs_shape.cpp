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

static const char *const COLLECTOR_NAMES[] = {"Closest", "Any", "All"};
static const CollectorKind COLLECTOR_KINDS[] = {CollectorKind::CLOSEST, CollectorKind::ANY, CollectorKind::ALL};

static constexpr int MAX_CONTACT_POINTS = 10;

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
    bool collect_faces;
    Pose pose_a;
    Pose pose_b;
    CollideShapeCollector collector;
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
        {.position = {1.0f, -0.6f, 0.0f}, .quaternion = quat_from_degrees(0.0f, 45.0f, 0.0f, EulerOrder::ZYX), .shape = lower},
    };
    if (is_static) return app::keep(app, static_compound::create({.children = children, .num_children = 2}));
    return app::keep(app, compound::create({.children = children, .num_children = 2}));
}

static void make_shapes(App &app) {
    state.shapes[OPTION_SPHERE] = app::keep(app, sphere::create({.radius = 1.0f}));
    state.shapes[OPTION_BOX] = app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}));
    state.shapes[OPTION_CAPSULE] = app::keep(app, capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f}));

    Model bunny = model::load("bunny.glb");
    state.shapes[OPTION_TRIANGLE_MESH] = app::keep(app, triangle_mesh::create({.positions = bunny.positions.data,
                                                                                                    .num_positions = bunny.positions.length / 3,
                                                                                                    .indices = bunny.indices.data,
                                                                                                    .num_indices = bunny.indices.length}));
    model::release(bunny);

    state.shapes[OPTION_COMPOUND] = two_boxes(app, false);
    state.shapes[OPTION_STATIC_COMPOUND] = two_boxes(app, true);

    Model suzanne = model::load("suzi.glb");
    state.shapes[OPTION_CONVEX_HULL] = app::keep(app, convex_hull::create({.positions = suzanne.positions.data, .num_positions = suzanne.positions.length / 3, .convex_radius = 0.0f}));
    model::release(suzanne);

    Shape *rotated = app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}));
    state.shapes[OPTION_TRANSFORMED_BOX] = app::keep(
        app, transformed::create({.shape = rotated, .position = {0.0f, 0.0f, 0.0f}, .quaternion = quat_from_degrees(25.0f, 30.0f, 45.0f, EulerOrder::ZYX)}));
}

/** crashcat's setup: both shapes back where they started */
static void reset_poses() {
    state.pose_a = {{-3.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    state.pose_b = {{3.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
}

static void init(App &app) {
    state.shape_a = OPTION_SPHERE;
    state.shape_b = OPTION_BOX;
    state.collector_mode = 0;
    state.collect_faces = true;
    state.collector = collide_shape::create_collector(CollectorKind::CLOSEST);
    make_shapes(app);
    reset_poses();
    camera::look_at(app.camera, {5.0f, 5.0f, 5.0f}, {0.0f, 0.0f, 0.0f});
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

    CollideShapeSettings settings = collide_shape::default_settings();
    settings.collect_faces = state.collect_faces;
    // no movement for a static overlap: the direction from b to a stands in for it
    settings.active_edge_movement_direction = vec3f::subtract(vec3f::load(state.pose_a.position), vec3f::load(state.pose_b.position));

    collide_shape::reset(state.collector);
    state.collector.kind = COLLECTOR_KINDS[state.collector_mode];
    narrowphase::collide_shape_vs_shape(state.collector, settings, shape_a, sub_shape::builder(), transform_a, shape_b, sub_shape::builder(),
                                        transform_b);
    const collections::Array<CollideShapeHit> &hits = state.collector.hits;
    const bool colliding = hits.length > 0;

    DrawList &frame = app::frame(app);
    const Color shape_color = hex(colliding ? 0x00ff00 : 0xff0000, 0.5f);
    renderer::mesh(app.renderer, renderer::mesh_of(app.renderer, shape_a), matrix::compose(transform_a.position, transform_a.quaternion),
                   shape_color);
    renderer::mesh(app.renderer, renderer::mesh_of(app.renderer, shape_b), matrix::compose(transform_b.position, transform_b.quaternion),
                   shape_color);

    const float arrow_length = 0.8f;
    for (uint32_t i = 0; i < hits.length; i++) {
        const CollideShapeHit &hit = hits[i];
        const Float3 point_a = to_float3(hit.point_a);
        if ((int)i < MAX_CONTACT_POINTS) {
            renderer::point(frame, point_a, 0.15f, hex(0xffaa00));
            renderer::point(frame, to_float3(hit.point_b), 0.15f, hex(0xff00ff));
        }
        const Float3 axis_end = to_float3(vec3f::scale_and_add(hit.point_a, hit.penetration_axis, arrow_length));
        renderer::line(frame, point_a, axis_end, hex(0x00ffff));
        renderer::line(frame, point_a, axis_end, hex(0x00ff00));
    }

    if (colliding && state.collect_faces) {
        draw_face(frame, hits[0].face_a, hex(0xff0000));
        draw_face(frame, hits[0].face_b, hex(0x00ff00));
    }
}

static void ui(App &) {
    if (ImGui::Combo("Shape A Type", &state.shape_a, SHAPE_OPTION_NAMES, OPTION_COUNT)) reset_poses();
    if (ImGui::Combo("Shape B Type", &state.shape_b, SHAPE_OPTION_NAMES, OPTION_COUNT)) reset_poses();
    ImGui::Combo("Collector Mode", &state.collector_mode, COLLECTOR_NAMES, 3);
    ImGui::Checkbox("Collect Faces", &state.collect_faces);
    edit_pose("shape A", state.pose_a);
    edit_pose("shape B", state.pose_b);

    ImGui::Separator();
    const collections::Array<CollideShapeHit> &hits = state.collector.hits;
    const bool colliding = hits.length > 0;
    const Float3 &a = state.pose_a.position;
    const Float3 &b = state.pose_b.position;
    ImGui::Text("collideShapeVsShape");
    ImGui::Text("Mode: %s", COLLECTOR_NAMES[state.collector_mode]);
    ImGui::TextColored(colliding ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1), "Colliding: %s", colliding ? "YES" : "NO");
    ImGui::Text("Contacts: %u", hits.length);
    ImGui::Text("Distance: %.3f", vec::length(vec::subtract(b, a)));
    for (uint32_t i = 0; i < hits.length && i < 5; i++) {
        const CollideShapeHit &hit = hits[i];
        ImGui::Text("Contact %u:", i);
        ImGui::Text("Penetration: %.6f", hit.penetration);
        ImGui::TextColored(ImVec4(0, 1, 1, 1), "Penetration Axis: (%.2f, %.2f, %.2f)", vec3f::x(hit.penetration_axis),
                           vec3f::y(hit.penetration_axis), vec3f::z(hit.penetration_axis));
    }
    if (hits.length > 5) ImGui::Text("... and %u more contacts", hits.length - 5);
    if (state.collect_faces && colliding) {
        ImGui::Text("Face Extraction:");
        ImGui::Text("Face A vertices: %u", hits[0].face_a.num_vertices);
        ImGui::Text("Face B vertices: %u", hits[0].face_b.num_vertices);
        ImGui::TextColored(ImVec4(1, 0, 0, 1), "Red lines = Face A");
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "Green lines = Face B");
    }
}

static void shutdown(App &) { state.collector = {}; }

static const Example EXAMPLE_COLLIDE_SHAPE_VS_SHAPE = {"Collide Shape vs Shape", init, nullptr, update, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_COLLIDE_SHAPE_VS_SHAPE);
