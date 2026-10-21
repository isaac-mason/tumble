#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/random.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"

using namespace tumble;

static constexpr int CHUNK_BITS = 4;
static constexpr int CHUNK_SIZE = 1 << CHUNK_BITS;
static constexpr int CHUNK_MASK = CHUNK_SIZE - 1;
static constexpr int TERRAIN_SIZE = 32;
static constexpr int TERRAIN_HEIGHT = 16;
static constexpr int BASE_HEIGHT = 4;
static constexpr float SPAWN_HEIGHT = 25.0f;
static constexpr float SPAWN_AREA = 10.0f;
static constexpr int KIND_COUNT = 6;
/** one body respawned every this many steps, half a second at 60 */
static constexpr int RESPAWN_STEPS = 30;

struct Chunk {
    int cx, cy, cz;
    uint8_t solid[CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE];
};

struct Kind {
    shapes::Shape *shape;
    Color color;
};

static struct {
    app::Layers layers;
    collections::Vec<Chunk *> chunks;
    Kind kinds[KIND_COUNT];
    collections::Vec<rigid_body::RigidBody *> spawned;
    int count;
    int respawn_steps;
    int steps;
    uint32_t next_respawn;
    Random random;
} state;

static Color hex(uint32_t rgb) { return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, 1.0f}; }

static const Float4 IDENTITY = {0.0f, 0.0f, 0.0f, 1.0f};

/* simplex noise, as crashcat's examples make their terrain */

struct Simplex2d {
    uint8_t perm[512];
    int8_t gradients[512][2];
};

static void simplex2d_create(Simplex2d &out, int seed) {
    static const uint8_t P[256] = {
        151, 160, 137, 91,  90,  15,  131, 13,  201, 95,  96,  53,  194, 233, 7,   225, 140, 36,  103, 30,  69,  142, 8,   99,  37,  240,
        21,  10,  23,  190, 6,   148, 247, 120, 234, 75,  0,   26,  197, 62,  94,  252, 219, 203, 117, 35,  11,  32,  57,  177, 33,  88,
        237, 149, 56,  87,  174, 20,  125, 136, 171, 168, 68,  175, 74,  165, 71,  134, 139, 48,  27,  166, 77,  146, 158, 231, 83,  111,
        229, 122, 60,  211, 133, 230, 220, 105, 92,  41,  55,  46,  245, 40,  244, 102, 143, 54,  65,  25,  63,  161, 1,   216, 80,  73,
        209, 76,  132, 187, 208, 89,  18,  169, 200, 196, 135, 130, 116, 188, 159, 86,  164, 100, 109, 198, 173, 186, 3,   64,  52,  217,
        226, 250, 124, 123, 5,   202, 38,  147, 118, 126, 255, 82,  85,  212, 207, 206, 59,  227, 47,  16,  58,  17,  182, 189, 28,  42,
        223, 183, 170, 213, 119, 248, 152, 2,   44,  154, 163, 70,  221, 153, 101, 155, 167, 43,  172, 9,   129, 22,  39,  253, 19,  98,
        108, 110, 79,  113, 224, 232, 178, 185, 112, 104, 218, 246, 97,  228, 251, 34,  242, 193, 238, 210, 144, 12,  191, 179, 162, 241,
        81,  51,  145, 235, 249, 14,  239, 107, 49,  192, 214, 31,  181, 199, 106, 157, 184, 84,  204, 176, 115, 121, 50,  45,  127, 4,
        150, 254, 138, 236, 205, 93,  222, 114, 67,  29,  24,  72,  243, 141, 128, 195, 78,  66,  215, 61,  156, 180,
    };
    // the x and y of the 12 cube edge directions
    static const int8_t GRADIENTS[12][2] = {{1, 1}, {-1, 1}, {1, -1}, {-1, -1}, {1, 0}, {-1, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {0, 1}, {0, -1}};
    if (seed < 256) seed |= seed << 8;
    for (int i = 0; i < 256; i++) {
        const uint8_t v = (uint8_t)(P[i] ^ ((i & 1) ? (seed & 255) : ((seed >> 8) & 255)));
        out.perm[i] = out.perm[i + 256] = v;
        out.gradients[i][0] = out.gradients[i + 256][0] = GRADIENTS[v % 12][0];
        out.gradients[i][1] = out.gradients[i + 256][1] = GRADIENTS[v % 12][1];
    }
}

/** in [-1, 1] */
static double simplex2d_sample(const Simplex2d &noise, double x, double y) {
    const double F2 = 0.5 * (sqrt(3.0) - 1.0);
    const double G2 = (3.0 - sqrt(3.0)) / 6.0;
    const double s = (x + y) * F2;
    int i = (int)floor(x + s);
    int j = (int)floor(y + s);
    const double t = (i + j) * G2;
    const double x0 = x - i + t;
    const double y0 = y - j + t;
    const int i1 = x0 > y0 ? 1 : 0;
    const int j1 = x0 > y0 ? 0 : 1;
    const double x1 = x0 - i1 + G2;
    const double y1 = y0 - j1 + G2;
    const double x2 = x0 - 1.0 + 2.0 * G2;
    const double y2 = y0 - 1.0 + 2.0 * G2;
    i &= 255;
    j &= 255;
    const int8_t *g0 = noise.gradients[i + noise.perm[j]];
    const int8_t *g1 = noise.gradients[i + i1 + noise.perm[j + j1]];
    const int8_t *g2 = noise.gradients[i + 1 + noise.perm[j + 1]];
    const auto corner = [](double tx, double ty, const int8_t *g) {
        double falloff = 0.5 - tx * tx - ty * ty;
        if (falloff < 0.0) return 0.0;
        falloff *= falloff;
        return falloff * falloff * (g[0] * tx + g[1] * ty);
    };
    return 70.0 * (corner(x0, y0, g0) + corner(x1, y1, g1) + corner(x2, y2, g2));
}

/* voxels in chunks of 16^3, made as they are first set */

static Chunk *find_chunk(int cx, int cy, int cz) {
    for (uint32_t i = 0; i < state.chunks.length; i++) {
        Chunk *chunk = state.chunks[i];
        if (chunk->cx == cx && chunk->cy == cy && chunk->cz == cz) return chunk;
    }
    return nullptr;
}

static int voxel_index(int x, int y, int z) { return (x & CHUNK_MASK) + (y & CHUNK_MASK) * CHUNK_SIZE + (z & CHUNK_MASK) * CHUNK_SIZE * CHUNK_SIZE; }

static void set_voxel(int x, int y, int z, bool solid) {
    Chunk *chunk = find_chunk(x >> CHUNK_BITS, y >> CHUNK_BITS, z >> CHUNK_BITS);
    if (chunk == nullptr) {
        chunk = util::alloc<Chunk>();
        *chunk = {x >> CHUNK_BITS, y >> CHUNK_BITS, z >> CHUNK_BITS, {}};
        state.chunks.push(chunk);
    }
    chunk->solid[voxel_index(x, y, z)] = solid ? 1 : 0;
}

static bool get_voxel(int x, int y, int z) {
    const Chunk *chunk = find_chunk(x >> CHUNK_BITS, y >> CHUNK_BITS, z >> CHUNK_BITS);
    return chunk != nullptr && chunk->solid[voxel_index(x, y, z)] != 0;
}

/** a quad wherever a solid voxel of the chunk meets an empty one, counter clockwise seen from the empty side */
static void mesh_chunk(collections::Vec<float> &positions, collections::Vec<uint32_t> &indices, const Chunk &chunk) {
    const int origin[3] = {chunk.cx * CHUNK_SIZE, chunk.cy * CHUNK_SIZE, chunk.cz * CHUNK_SIZE};
    for (int x = -1; x < CHUNK_SIZE; x++) {
        for (int y = -1; y < CHUNK_SIZE; y++) {
            for (int z = -1; z < CHUNK_SIZE; z++) {
                const int at[3] = {origin[0] + x, origin[1] + y, origin[2] + z};
                const bool solid = get_voxel(at[0], at[1], at[2]);
                for (int axis = 0; axis < 3; axis++) {
                    int next[3] = {at[0], at[1], at[2]};
                    next[axis]++;
                    if (solid == get_voxel(next[0], next[1], next[2])) continue;
                    // the face belongs to whichever of the two is solid, and only a voxel of this chunk's
                    int owner[3] = {x, y, z};
                    if (!solid) owner[axis]++;
                    if (owner[0] < 0 || owner[0] >= CHUNK_SIZE || owner[1] < 0 || owner[1] >= CHUNK_SIZE || owner[2] < 0 || owner[2] >= CHUNK_SIZE) continue;
                    float u[3] = {0.0f, 0.0f, 0.0f};
                    float v[3] = {0.0f, 0.0f, 0.0f};
                    u[(axis + (solid ? 1 : 2)) % 3] = 1.0f;
                    v[(axis + (solid ? 2 : 1)) % 3] = 1.0f;
                    const uint32_t base = positions.length / 3;
                    const float corner[3] = {(float)next[0], (float)next[1], (float)next[2]};
                    for (int k = 0; k < 3; k++) positions.push(corner[k]);
                    for (int k = 0; k < 3; k++) positions.push(corner[k] + u[k]);
                    for (int k = 0; k < 3; k++) positions.push(corner[k] + u[k] + v[k]);
                    for (int k = 0; k < 3; k++) positions.push(corner[k] + v[k]);
                    const uint32_t quad[6] = {base, base + 1, base + 3, base + 1, base + 2, base + 3};
                    for (uint32_t index : quad) indices.push(index);
                }
            }
        }
    }
}

static void build_chunks(App &app) {
    collections::Vec<float> positions = {};
    collections::Vec<uint32_t> indices = {};
    for (uint32_t i = 0; i < state.chunks.length; i++) {
        positions.length = 0;
        indices.length = 0;
        mesh_chunk(positions, indices, *state.chunks[i]);
        if (positions.length == 0) continue;
        triangle_mesh_builder::TriangleMeshBuilderSettings mesh;
        mesh.positions = positions.data;
        mesh.num_positions = positions.length / 3;
        mesh.indices = indices.data;
        mesh.num_indices = indices.length;
        mesh.bvh_split_strategy = triangle_mesh_bvh::BVH_SPLIT_STRATEGY_CENTER;
        rigid_body::RigidBodySettings settings;
        settings.shape = app::keep(app, triangle_mesh::create({.data = app::keep(app, triangle_mesh_builder::build(mesh))}));
        settings.object_layer = state.layers.not_moving;
        settings.motion_type = motion_type::MOTION_STATIC;
        settings.restitution = 0.2f;
        settings.friction = 0.7f;
        rigid_body::create(*app.world, settings);
    }
    positions.release();
    indices.release();
}

static void make_terrain() {
    Simplex2d noise;
    simplex2d_create(noise, 42);
    for (int x = -TERRAIN_SIZE; x < TERRAIN_SIZE; x++) {
        for (int z = -TERRAIN_SIZE; z < TERRAIN_SIZE; z++) {
            // large hills, medium detail and small detail
            const double hills = simplex2d_sample(noise, x * 0.02, z * 0.02) * 0.5 + 0.5;
            const double medium = simplex2d_sample(noise, x * 0.08, z * 0.08) * 0.5 + 0.5;
            const double small = simplex2d_sample(noise, x * 0.2, z * 0.2) * 0.5 + 0.5;
            const int height = (int)floor(BASE_HEIGHT + (hills * 0.6 + medium * 0.3 + small * 0.1) * TERRAIN_HEIGHT);
            for (int y = 0; y < height; y++) set_voxel(x, y, z, true);
        }
    }
}

/* the bodies dropped on it, as the shapes example drops them */

static void make_kinds(App &app) {
    shapes::Shape *ball = app::keep(app, sphere::create({.radius = 0.5f}));
    state.kinds[0] = {ball, hex(0xff6b6b)};
    state.kinds[1] = {app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.05f})), hex(0x4ecdc4)};
    state.kinds[2] = {app::keep(app, capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f})), hex(0xdfe6e9)};

    shapes::Shape *bar = app::keep(app, capsule::create({.half_height_of_cylinder = 0.8f, .radius = 0.3f}));
    const compound::CompoundShapeChild dumbbell[] = {
        {.position = {-1.1f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = ball},
        {.position = {1.1f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = ball},
        {.position = {0.0f, 0.0f, 0.0f}, .quaternion = vec::quat_axis_angle({0.0f, 0.0f, 1.0f}, 0.5f * 3.14159265f), .shape = bar},
    };
    state.kinds[3] = {app::keep(app, compound::create({.children = dumbbell, .num_children = 3})), hex(0x95e1d3)};

    shapes::Shape *handle = app::keep(app, box::create({.half_extents = {0.2f, 1.5f, 0.2f}, .convex_radius = 0.05f}));
    shapes::Shape *head_box = app::keep(app, box::create({.half_extents = {0.8f, 0.3f, 0.3f}, .convex_radius = 0.05f, .density = 10000.0f}));
    shapes::Shape *head = app::keep(app, transformed::create({.shape = head_box, .position = {0.0f, 1.5f, 0.0f}, .quaternion = IDENTITY}));
    const compound::CompoundShapeChild hammer[] = {
        {.position = {0.0f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = handle},
        {.position = {0.0f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = head},
    };
    state.kinds[4] = {app::keep(app, compound::create({.children = hammer, .num_children = 2})), hex(0xf39c12)};

    const float pyramid[] = {-0.5f, 0.0f, -0.5f, 0.5f, 0.0f, -0.5f, 0.5f, 0.0f, 0.5f, -0.5f, 0.0f, 0.5f, 0.0f, 1.0f, 0.0f};
    convex_hull::ConvexHullData *hull = app::keep(app, convex_hull::build({.positions = pyramid, .num_positions = 5, .convex_radius = 0.05f}, nullptr));
    state.kinds[5] = {app::keep(app, convex_hull::create({.data = hull})), hex(0x9b59b6)};
}

static rigid_body::RigidBody *spawn(App &app, float restitution) {
    const Kind &kind = state.kinds[(uint32_t)(seeded::next(state.random) * KIND_COUNT) % KIND_COUNT];
    rigid_body::RigidBodySettings settings;
    settings.shape = kind.shape;
    settings.object_layer = state.layers.moving;
    settings.motion_type = motion_type::MOTION_DYNAMIC;
    settings.position = {seeded::range(state.random, -SPAWN_AREA, SPAWN_AREA), SPAWN_HEIGHT, seeded::range(state.random, -SPAWN_AREA, SPAWN_AREA)};
    const Float3 axis = {seeded::next(state.random) - 0.5f, seeded::next(state.random) - 0.5f, seeded::next(state.random) - 0.5f};
    settings.quaternion = vec::quat_axis_angle(axis, seeded::next(state.random) * 2.0f * 3.14159265f);
    settings.restitution = restitution;
    settings.friction = 0.5f;
    rigid_body::RigidBody *body = rigid_body::create(*app.world, settings);
    app::set_color(app, body, kind.color);
    return body;
}

static void match_count(App &app) {
    while ((int)state.spawned.length < state.count) state.spawned.push(spawn(app, 0.0f));
    while ((int)state.spawned.length > state.count) rigid_body::remove(*app.world, state.spawned[--state.spawned.length]);
}

static void init(App &app) {
    state.spawned.length = 0;
    state.count = 50;
    state.respawn_steps = RESPAWN_STEPS;
    state.steps = 0;
    state.next_respawn = 0;
    state.random = {11};
    app.world = app::create_world(state.layers);
    make_terrain();
    build_chunks(app);
    make_kinds(app);
    match_count(app);
    app.drawing.grid = false;
    // above the terrain: crashcat's eye, (0, 10, 30), is inside a column 10 high
    camera::look_at(app.camera, {0.0f, 40.0f, 60.0f}, {0.0f, 5.0f, 0.0f});
}

static void step(App &app, float) {
    // round robin: the oldest body goes, a new one of any kind drops in
    if (++state.steps < state.respawn_steps || state.spawned.length == 0) return;
    state.steps = 0;
    rigid_body::RigidBody *&oldest = state.spawned[state.next_respawn++ % state.spawned.length];
    rigid_body::remove(*app.world, oldest);
    oldest = spawn(app, 0.3f);
}

static void ui(App &app) {
    if (ImGui::SliderInt("bodies", &state.count, 0, 1000)) match_count(app);
    ImGui::SliderInt("respawn every (steps)", &state.respawn_steps, 6, 300);
}

static void shutdown(App &) {
    for (uint32_t i = 0; i < state.chunks.length; i++) util::free(state.chunks[i]);
    state.chunks.release();
    state.spawned.release();
}

static const Example EXAMPLE_VOXEL_TRIANGLE_MESH = {"Voxel Triangle Mesh", init, step, nullptr, ui, shutdown, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_VOXEL_TRIANGLE_MESH);
