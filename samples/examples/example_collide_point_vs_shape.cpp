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

static const char *const COLLECTOR_NAMES[] = {"Any", "All"};
static const CollectorKind COLLECTOR_KINDS[] = {CollectorKind::ANY, CollectorKind::ALL};

namespace {

/** what crashcat's transform gizmo edits: a position, and a rotation as xyz euler degrees */
struct Pose {
    Float3 position;
    Float3 rotation_degrees;
};

}  // namespace

static struct {
    Shape *shapes[OPTION_COUNT];
    int shape;
    int collector_mode;
    Pose pose;
    Float3 point;
    CollidePointCollector collector;
} state;

static Color hex(uint32_t rgb, float alpha = 1.0f) {
    return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, alpha};
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

/** crashcat's setup: the shape and the point back where they started */
static void reset_poses() {
    state.pose = {{2.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    state.point = {-2.0f, 0.0f, 0.0f};
}

static void init(App &app) {
    state.shape = OPTION_BOX;
    state.collector_mode = 0;
    state.collector = collide_point::create_collector(CollectorKind::ANY);
    make_shapes(app);
    reset_poses();
    camera::look_at(app.camera, {5.0f, 5.0f, 5.0f}, {0.0f, 0.0f, 0.0f});
}

static void update(App &app, float) {
    const Transformf transform = transform_of(state.pose);
    const Shape *shape = state.shapes[state.shape];

    collide_point::reset(state.collector);
    state.collector.kind = COLLECTOR_KINDS[state.collector_mode];
    narrowphase::collide_point_vs_shape(state.collector, collide_point::default_settings(), vec3f::load(state.point), shape, sub_shape::builder(),
                                        transform);
    const bool colliding = state.collector.hits.length > 0;

    renderer::mesh(app.renderer, renderer::mesh_of(app.renderer, shape), matrix::compose(transform.position, transform.quaternion),
                   hex(colliding ? 0x00ff00 : 0xff0000, 0.5f));
    renderer::point(app::frame(app), state.point, 0.2f, hex(colliding ? 0x00ff00 : 0xffaa00));
}

static void ui(App &) {
    if (ImGui::Combo("Shape Type", &state.shape, SHAPE_OPTION_NAMES, OPTION_COUNT)) reset_poses();
    ImGui::Combo("Collector Mode", &state.collector_mode, COLLECTOR_NAMES, 2);
    edit_pose("shape", state.pose);
    ImGui::PushID("point");
    ImGui::TextUnformatted("point");
    ImGui::DragFloat3("position", &state.point.x, 0.02f);
    ImGui::PopID();

    ImGui::Separator();
    const collections::Array<CollidePointHit> &hits = state.collector.hits;
    const bool colliding = hits.length > 0;
    const Float3 &point = state.point;
    ImGui::Text("collidePointVsShape");
    ImGui::Text("Mode: %s", COLLECTOR_NAMES[state.collector_mode]);
    ImGui::TextColored(colliding ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1), "Colliding: %s", colliding ? "YES" : "NO");
    ImGui::Text("Hits: %u", hits.length);
    ImGui::Text("Distance: %.3f", vec::length(vec::subtract(state.pose.position, point)));
    ImGui::Text("Point: (%.2f, %.2f, %.2f)", point.x, point.y, point.z);
    for (uint32_t i = 0; i < hits.length && i < 5; i++) {
        ImGui::Text("Hit %u:", i);
        ImGui::Text("SubShape ID: %u", hits[i].sub_shape_id_b);
        ImGui::Text("Body ID: %u", hits[i].body_id_b);
    }
    if (hits.length > 5) ImGui::Text("... and %u more hits", hits.length - 5);
}

static void shutdown(App &) { state.collector = {}; }

static const Example EXAMPLE_COLLIDE_POINT_VS_SHAPE = {"Collide Point vs Shape", init, nullptr, update, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_COLLIDE_POINT_VS_SHAPE);
