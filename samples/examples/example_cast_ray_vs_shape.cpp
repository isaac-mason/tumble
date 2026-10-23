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
static const collector::CollectorKind COLLECTOR_KINDS[] = {collector::COLLECT_CLOSEST, collector::COLLECT_ANY, collector::COLLECT_ALL};

/** what crashcat's transform gizmo edits: a position, and a rotation as xyz euler degrees */
struct Pose {
    Float3 position;
    Float3 rotation_degrees;
};

struct RayHit {
    float fraction;
    Float3 normal;
    sub_shape::SubShapeId sub_shape_id;
};

static struct {
    shapes::Shape *shapes[OPTION_COUNT];
    int shape;
    int collector_mode;
    Pose pose;
    Float3 ray_origin;
    Float3 ray_endpoint;
    cast_ray::CastRayCollector collector;
    collections::Vec<RayHit> hits;
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

static Float4 quaternion_of(const Pose &pose) { return quat_from_degrees(pose.rotation_degrees.x, pose.rotation_degrees.y, pose.rotation_degrees.z, EULER_XYZ); }

static Transformf transform_of(const Pose &pose) { return {pose.position, quaternion_of(pose), {1.0f, 1.0f, 1.0f}}; }

static void edit_pose(const char *label, Pose &pose) {
    ImGui::PushID(label);
    ImGui::TextUnformatted(label);
    ImGui::DragFloat3("position", &pose.position.x, 0.02f);
    ImGui::DragFloat3("rotation (deg)", &pose.rotation_degrees.x, 0.5f);
    ImGui::PopID();
}

static void edit_point(const char *label, Float3 &point) {
    ImGui::PushID(label);
    ImGui::TextUnformatted(label);
    ImGui::DragFloat3("position", &point.x, 0.02f);
    ImGui::PopID();
}

static shapes::Shape *two_boxes(App &app, bool is_static) {
    shapes::Shape *upper = app::keep(app, box::create({.half_extents = {0.8f, 0.5f, 0.8f}}));
    shapes::Shape *lower = app::keep(app, box::create({.half_extents = {0.6f, 0.7f, 0.6f}}));
    const compound::CompoundShapeChild children[] = {
        {.position = {-1.2f, 0.8f, 0.0f}, .quaternion = quat_from_degrees(15.0f, 45.0f, -10.0f, EULER_ZYX), .shape = upper},
        {.position = {1.0f, -0.6f, 0.0f}, .quaternion = quat_from_degrees(-20.0f, 30.0f, 25.0f, EULER_ZYX), .shape = lower},
    };
    if (is_static) return app::keep(app, static_compound::create({.children = children, .num_children = 2}));
    return app::keep(app, compound::create({.children = children, .num_children = 2}));
}

static void make_shapes(App &app) {
    state.shapes[OPTION_SPHERE] = app::keep(app, sphere::create({.radius = 1.0f}));
    state.shapes[OPTION_BOX] = app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}));
    state.shapes[OPTION_CAPSULE] = app::keep(app, capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f}));

    Model bunny = model::load("bunny.glb");
    triangle_mesh_data::TriangleMeshData *bunny_data = app::keep(app, triangle_mesh_builder::build({.positions = bunny.positions.data,
                                                                                                    .num_positions = bunny.positions.length / 3,
                                                                                                    .indices = bunny.indices.data,
                                                                                                    .num_indices = bunny.indices.length}));
    state.shapes[OPTION_TRIANGLE_MESH] = app::keep(app, triangle_mesh::create({.data = bunny_data}));
    model::release(bunny);

    state.shapes[OPTION_COMPOUND] = two_boxes(app, false);
    state.shapes[OPTION_STATIC_COMPOUND] = two_boxes(app, true);

    Model suzanne = model::load("suzi.glb");
    convex_hull::ConvexHullData *hull = app::keep(
        app, convex_hull::build({.positions = suzanne.positions.data, .num_positions = suzanne.positions.length / 3, .convex_radius = 0.0f}));
    state.shapes[OPTION_CONVEX_HULL] = app::keep(app, convex_hull::create({.data = hull}));
    model::release(suzanne);

    shapes::Shape *rotated = app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}));
    state.shapes[OPTION_TRANSFORMED_BOX] = app::keep(
        app, transformed::create({.shape = rotated, .position = {0.0f, 0.0f, 0.0f}, .quaternion = quat_from_degrees(25.0f, 30.0f, 45.0f, EULER_ZYX)}));
}

/** crashcat's setup: the shape and both ends of the ray back where they started */
static void reset_poses() {
    state.pose = {{2.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    state.ray_origin = {-3.0f, 0.0f, 0.0f};
    state.ray_endpoint = {7.0f, 0.0f, 0.0f};
}

static void init(App &app) {
    state.shape = OPTION_SPHERE;
    state.collector_mode = 2;
    state.collector = cast_ray::create_collector(collector::COLLECT_ALL);
    state.hits.length = 0;
    make_shapes(app);
    reset_poses();
    camera::look_at(app.camera, {5.0f, 5.0f, 8.0f}, {0.0f, 0.0f, 0.0f});
}

static void update(App &app, float) {
    const Transformf transform = transform_of(state.pose);
    const shapes::Shape *shape = state.shapes[state.shape];
    const Vec3f origin = vec3f::load(state.ray_origin);
    const Vec3f to_endpoint = vec3f::subtract(vec3f::load(state.ray_endpoint), origin);
    const float ray_length = vec3f::length(to_endpoint);
    const Vec3f direction = ray_length > 0.0f ? vec3f::scale(to_endpoint, 1.0f / ray_length) : vec3f::zero();

    cast_ray::CastRaySettings settings = cast_ray::default_settings();
    settings.collide_with_backfaces_triangles = true;
    cast_ray::reset(state.collector);
    state.collector.kind = COLLECTOR_KINDS[state.collector_mode];
    narrowphase::cast_ray_vs_shape(state.collector, settings, {origin, direction, ray_length}, shape, sub_shape::builder(), transform);
    if (state.collector.kind == collector::COLLECT_ALL) cast_ray::sort(state.collector);

    // each hit's normal from the shape's surface there, in its local space and back
    const Quatf rotation = quatf::load(transform.quaternion);
    const Vec3f shape_position = vec3f::load(transform.position);
    state.hits.length = 0;
    for (uint32_t i = 0; i < state.collector.hits.length; i++) {
        const cast_ray::CastRayHit &hit = state.collector.hits[i];
        if (hit.status != cast_ray::COLLIDING) continue;
        const Vec3f point = vec3f::scale_and_add(origin, direction, hit.fraction * ray_length);
        const Vec3f local_point = vec3f::transform_quat(vec3f::subtract(point, shape_position), quatf::conjugate(rotation));
        const Vec3f normal = vec3f::transform_quat(shapes::get_shape_surface_normal(shape, local_point, hit.sub_shape_id), rotation);
        state.hits.push({hit.fraction, to_float3(normal), hit.sub_shape_id});
    }

    DrawList &frame = app::frame(app);
    renderer::mesh(app.renderer, renderer::mesh_of(app.renderer, shape), matrix::compose(transform.position, transform.quaternion),
                   hex(state.hits.length > 0 ? 0x00ff00 : 0x4488ff));
    renderer::point(frame, state.ray_origin, 0.2f, hex(0xff00ff));
    renderer::point(frame, state.ray_endpoint, 0.2f, hex(0x00ff00));
    renderer::line(frame, state.ray_origin, state.ray_endpoint, hex(0x00ff00));
    for (uint32_t i = 0; i < state.hits.length; i++) {
        const Float3 point = to_float3(vec3f::scale_and_add(origin, direction, state.hits[i].fraction * ray_length));
        renderer::point(frame, point, 0.15f, hex(0xffaa00));
        renderer::arrow(frame, point, vec::add(point, state.hits[i].normal), hex(0x00ff00));
    }
}

static void ui(App &) {
    if (ImGui::Combo("Shape Type", &state.shape, SHAPE_OPTION_NAMES, OPTION_COUNT)) reset_poses();
    ImGui::Combo("Collector Mode", &state.collector_mode, COLLECTOR_NAMES, 3);
    edit_pose("shape", state.pose);
    edit_point("ray origin", state.ray_origin);
    edit_point("ray endpoint", state.ray_endpoint);

    ImGui::Separator();
    const Float3 &origin = state.ray_origin;
    const Float3 to_endpoint = vec::subtract(state.ray_endpoint, origin);
    const float ray_length = vec::length(to_endpoint);
    const Float3 direction = vec::normalize(to_endpoint);
    ImGui::Text("Ray Info");
    ImGui::Text("Origin: (%.3f, %.3f, %.3f)", origin.x, origin.y, origin.z);
    ImGui::Text("Direction: (%.3f, %.3f, %.3f)", direction.x, direction.y, direction.z);
    ImGui::Text("Length: %.3f", ray_length);
    if (state.hits.length == 0) {
        ImGui::Text("No Hits");
        return;
    }
    ImGui::Text("Hits: %u", state.hits.length);
    for (uint32_t i = 0; i < state.hits.length && i < 5; i++) {
        const RayHit &hit = state.hits[i];
        const Float3 point = vec::scale_and_add(origin, direction, hit.fraction * ray_length);
        ImGui::Text("Hit %u:", i + 1);
        ImGui::Text("  Pos: (%.2f, %.2f, %.2f)", point.x, point.y, point.z);
        ImGui::Text("  Normal: (%.3f, %.3f, %.3f)", hit.normal.x, hit.normal.y, hit.normal.z);
        ImGui::Text("  SubShapeId: %u", hit.sub_shape_id);
    }
    if (state.hits.length > 5) ImGui::Text("... and %u more", state.hits.length - 5);
}

static void shutdown(App &) {
    cast_ray::release(state.collector);
    state.hits.release();
}

static const Example EXAMPLE_CAST_RAY_VS_SHAPE = {"Cast Ray vs Shape", init, nullptr, update, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_CAST_RAY_VS_SHAPE);
