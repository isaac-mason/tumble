#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/random.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/scaled.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"

using namespace tumble;

/** each cast starts this far from the centre and sweeps to it */
static constexpr float POINT_DISTANCE = 5.0f;
static constexpr float PI = 3.14159265f;

enum ShapeType : int { TORUS_KNOT, SPHERE, BOX, CAPSULE, SHAPE_TYPE_COUNT };
static const char *const SHAPE_NAMES[SHAPE_TYPE_COUNT] = {"Torus Knot Triangle Mesh", "Sphere", "Box", "Capsule"};

enum CastShapeType : int { CAST_SPHERE, CAST_BOX, CAST_CAPSULE, CAST_SHAPE_TYPE_COUNT };
static const char *const CAST_SHAPE_NAMES[CAST_SHAPE_TYPE_COUNT] = {"Sphere", "Box", "Capsule"};

struct Spinner {
    Float4 quaternion;
    /** radians per millisecond about x, y and z, as crashcat's frame time is in milliseconds */
    Float3 rotation_speed;
};

struct MeshObject {
    rigid_body::RigidBody *body;
    /** in the container's space */
    Float3 position;
    Spinner spin;
};

static struct {
    app::Layers layers;
    shapes::Shape *shapes[SHAPE_TYPE_COUNT];
    shapes::Shape *cast_shapes[CAST_SHAPE_TYPE_COUNT];
    filter::Filter filter;
    cast_shape::CastShapeCollector collector;
    collections::Vec<MeshObject> meshes;
    collections::Vec<Spinner> shapecasters;
    /** the container the meshes turn in, as euler angles about x and y */
    float container_x;
    float container_y;
    int shapecaster_count;
    float shapecaster_speed;
    int cast_shape_type;
    int shape_type;
    int mesh_count;
    float mesh_speed;
    Random random;
} state;

static Color hex(uint32_t rgb, float alpha = 1.0f) {
    return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, alpha};
}

/** three.js's 'XYZ' euler order */
static Float4 euler_xyz(float x, float y, float z) {
    const float c1 = cosf(x * 0.5f), c2 = cosf(y * 0.5f), c3 = cosf(z * 0.5f);
    const float s1 = sinf(x * 0.5f), s2 = sinf(y * 0.5f), s3 = sinf(z * 0.5f);
    return {s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3, c1 * c2 * c3 - s1 * s2 * s3};
}

static Spinner random_spinner(float z_range) {
    Spinner spin;
    spin.rotation_speed = {(seeded::next(state.random) - 0.5f) * 0.0001f, (seeded::next(state.random) - 0.5f) * 0.0001f,
                           (seeded::next(state.random) - 0.5f) * 0.0001f};
    spin.quaternion = euler_xyz(seeded::next(state.random) * 10.0f, seeded::next(state.random) * 10.0f, seeded::next(state.random) * z_range);
    return spin;
}

static void turn(Spinner &spin, float speed, float milliseconds) {
    const Float3 &r = spin.rotation_speed;
    spin.quaternion = vec::quat_multiply(spin.quaternion, euler_xyz(r.x * speed * milliseconds, r.y * speed * milliseconds, r.z * speed * milliseconds));
}

/** three.js's TorusKnotGeometry(1, 0.4, 40, 10), p = 2, q = 3 */
static shapes::Shape *make_torus_knot(App &app) {
    constexpr int TUBULAR = 40, RADIAL = 10;
    constexpr float RADIUS = 1.0f, TUBE = 0.4f, P = 2.0f, Q = 3.0f;
    const auto curve = [](float u) {
        const float quotient = Q / P * u;
        const float cs = cosf(quotient);
        return Float3{RADIUS * (2.0f + cs) * 0.5f * cosf(u), RADIUS * (2.0f + cs) * sinf(u) * 0.5f, RADIUS * sinf(quotient) * 0.5f};
    };
    float positions[(TUBULAR + 1) * (RADIAL + 1) * 3];
    uint32_t indices[TUBULAR * RADIAL * 6];
    int at = 0;
    for (int i = 0; i <= TUBULAR; i++) {
        const float u = (float)i / TUBULAR * P * PI * 2.0f;
        const Float3 p1 = curve(u);
        const Float3 p2 = curve(u + 0.01f);
        const Float3 tangent = vec::subtract(p2, p1);
        Float3 normal = vec::add(p2, p1);
        const Float3 binormal = vec::normalize(vec::cross(tangent, normal));
        normal = vec::normalize(vec::cross(binormal, tangent));
        for (int j = 0; j <= RADIAL; j++) {
            const float v = (float)j / RADIAL * PI * 2.0f;
            const float cx = -TUBE * cosf(v);
            const float cy = TUBE * sinf(v);
            const Float3 vertex = vec::add(p1, vec::add(vec::scale(normal, cx), vec::scale(binormal, cy)));
            positions[at++] = vertex.x;
            positions[at++] = vertex.y;
            positions[at++] = vertex.z;
        }
    }
    at = 0;
    for (uint32_t j = 1; j <= TUBULAR; j++) {
        for (uint32_t i = 1; i <= RADIAL; i++) {
            const uint32_t a = (RADIAL + 1) * (j - 1) + (i - 1);
            const uint32_t b = (RADIAL + 1) * j + (i - 1);
            const uint32_t c = (RADIAL + 1) * j + i;
            const uint32_t d = (RADIAL + 1) * (j - 1) + i;
            const uint32_t triangles[6] = {a, b, d, b, c, d};
            for (uint32_t index : triangles) indices[at++] = index;
        }
    }
    triangle_mesh_data::TriangleMeshData *data = app::keep(
        app, triangle_mesh_builder::build({.positions = positions, .num_positions = (TUBULAR + 1) * (RADIAL + 1), .indices = indices, .num_indices = TUBULAR * RADIAL * 6}));
    return app::keep(app, triangle_mesh::create({.data = data}));
}

/** the shapecasters and the meshes made again: crashcat's updateFromOptions */
static void update_from_options(App &app) {
    state.shapecasters.length = 0;
    while ((int)state.shapecasters.length < state.shapecaster_count) state.shapecasters.push(random_spinner(10.0f));

    for (uint32_t i = 0; i < state.meshes.length; i++) rigid_body::remove(*app.world, state.meshes[i].body);
    state.meshes.length = 0;

    // more meshes spread further out, and smaller
    const float amount = (float)(state.mesh_count - 1) / (300.0f - 1.0f);
    const float distance = 2.0f * amount;
    const float scale = 1.0f + (0.2f - 1.0f) * amount;
    shapes::Shape *shape = state.shapes[state.shape_type];
    if (state.mesh_count > 1) shape = app::keep(app, scaled::create({.shape = shape, .scale = {scale, scale, scale}}));

    for (int i = 0; i < state.mesh_count; i++) {
        MeshObject mesh;
        mesh.spin = random_spinner(0.0f);
        const Float3 up = {0.0f, 1.0f, 0.0f};
        const Float3 tilted = vec::rotate(vec::quat_axis_angle({1.0f, 0.0f, 0.0f}, PI * seeded::next(state.random)), up);
        mesh.position = vec::scale(vec::rotate(vec::quat_axis_angle({0.0f, 1.0f, 0.0f}, 2.0f * PI * seeded::next(state.random)), tilted), distance);
        rigid_body::RigidBodySettings settings;
        settings.shape = shape;
        settings.object_layer = state.layers.moving;
        settings.motion_type = motion_type::MOTION_STATIC;
        settings.position = mesh.position;
        settings.quaternion = mesh.spin.quaternion;
        mesh.body = rigid_body::create(*app.world, settings);
        app::set_color(app, mesh.body, hex(0xe91e63));
        state.meshes.push(mesh);
    }
}

static void init(App &app) {
    state.meshes.length = 0;
    state.shapecasters.length = 0;
    state.container_x = 10.989999999999943f;
    state.container_y = 10.989999999999943f;
    state.shapecaster_count = 10;
    state.shapecaster_speed = 1.0f;
    state.cast_shape_type = CAST_BOX;
    state.shape_type = TORUS_KNOT;
    state.mesh_count = 1;
    state.mesh_speed = 1.0f;
    state.random = {5};

    app.world = app::create_world(state.layers);
    state.filter = filter::create(app.world->settings.layers);
    state.collector = cast_shape::create_collector(collector::COLLECT_CLOSEST);

    state.shapes[TORUS_KNOT] = make_torus_knot(app);
    state.shapes[SPHERE] = app::keep(app, sphere::create({.radius = 1.0f}));
    state.shapes[BOX] = app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}}));
    state.shapes[CAPSULE] = app::keep(app, capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.5f}));
    state.cast_shapes[CAST_SPHERE] = app::keep(app, sphere::create({.radius = 0.1f}));
    state.cast_shapes[CAST_BOX] = app::keep(app, box::create({.half_extents = {0.1f, 0.1f, 0.1f}}));
    state.cast_shapes[CAST_CAPSULE] = app::keep(app, capsule::create({.half_height_of_cylinder = 0.1f, .radius = 0.075f}));
    update_from_options(app);

    // no ground here
    app.drawing.grid = false;
    camera::look_at(app.camera, {0.0f, 0.0f, 10.0f}, {0.0f, 0.0f, 0.0f});
}

static void step(App &app, float delta_time) {
    const float milliseconds = delta_time * 1000.0f;
    state.container_x += 0.0001f * state.mesh_speed * milliseconds;
    state.container_y += 0.0001f * state.mesh_speed * milliseconds;
    const Float4 container = euler_xyz(state.container_x, state.container_y, 0.0f);
    for (uint32_t i = 0; i < state.meshes.length; i++) {
        MeshObject &mesh = state.meshes[i];
        turn(mesh.spin, state.mesh_speed, milliseconds);
        const Float3 position = vec::rotate(container, mesh.position);
        const Float4 quaternion = vec::quat_multiply(container, mesh.spin.quaternion);
        rigid_body::set_transform(*app.world, mesh.body, vec3f::load(position), quatf::load(quaternion), false);
    }
    for (uint32_t i = 0; i < state.shapecasters.length; i++) turn(state.shapecasters[i], state.shapecaster_speed, milliseconds);
}

static void update(App &app, float) {
    DrawList &frame = app::frame(app);
    const shapes::Shape *cast = state.cast_shapes[state.cast_shape_type];
    const GpuMesh cast_mesh = renderer::mesh_of(app.renderer, cast);
    // the cast shape keeps its own orientation as its origin circles the centre
    const Float4 identity = {0.0f, 0.0f, 0.0f, 1.0f};
    const Color shape_color = hex(0xffffff, 0.6f);
    for (uint32_t i = 0; i < state.shapecasters.length; i++) {
        const Float3 origin = vec::rotate(state.shapecasters[i].quaternion, {POINT_DISTANCE, 0.0f, 0.0f});
        const Float3 displacement = vec::scale(vec::normalize(vec::scale(origin, -1.0f)), POINT_DISTANCE);
        const Transformf transform = {origin, identity, {1.0f, 1.0f, 1.0f}};

        cast_shape::reset(state.collector);
        query::cast_shape(*app.world, state.collector, cast_shape::default_settings(), cast, transform, vec3f::load(displacement), state.filter);
        const cast_shape::CastShapeHit *hit = cast_shape::hit(state.collector);
        // the shape's centre where it first touches, or the end of the cast without a hit
        const Float3 end = vec::scale_and_add(origin, displacement, hit != nullptr ? hit->fraction : 1.0f);

        renderer::mesh(app.renderer, cast_mesh, matrix::compose(origin, identity), shape_color);
        renderer::mesh(app.renderer, cast_mesh, matrix::compose(end, identity), shape_color);
        renderer::line(frame, origin, end, hex(0xffffff, 0.25f));
    }
}

static void ui(App &app) {
    if (ImGui::CollapsingHeader("Shape Casters", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::SliderInt("count##shapecasters", &state.shapecaster_count, 1, 1000)) update_from_options(app);
        ImGui::SliderFloat("speed##shapecasters", &state.shapecaster_speed, 0.0f, 20.0f);
        if (ImGui::Combo("shapeType##shapecasters", &state.cast_shape_type, CAST_SHAPE_NAMES, CAST_SHAPE_TYPE_COUNT)) update_from_options(app);
    }
    if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Combo("shapeType##mesh", &state.shape_type, SHAPE_NAMES, SHAPE_TYPE_COUNT)) update_from_options(app);
        if (ImGui::SliderInt("count##mesh", &state.mesh_count, 1, 300)) update_from_options(app);
        ImGui::SliderFloat("speed##mesh", &state.mesh_speed, 0.0f, 20.0f);
    }
}

static void shutdown(App &) {
    state.meshes.release();
    state.shapecasters.release();
    cast_shape::release(state.collector);
    filter::release(state.filter);
}

static const Example EXAMPLE_CAST_SHAPE = {"Cast Shape", init, step, update, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_CAST_SHAPE);
