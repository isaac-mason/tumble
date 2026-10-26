#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/random.h"
#include "app/vec.h"
#include "imgui.h"
#include "sokol_app.h"
#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"

using namespace tumble;

static constexpr int CHUNK_BITS = 4;
static constexpr int CHUNK_SIZE = 1 << CHUNK_BITS;
static constexpr int CHUNK_MASK = CHUNK_SIZE - 1;
static constexpr int CHUNK_VOXEL_COUNT = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE;
static constexpr int TERRAIN_SIZE = 32;
static constexpr int TERRAIN_HEIGHT = 16;
static constexpr int BASE_HEIGHT = 4;
static constexpr float SPAWN_HEIGHT = 25.0f;
static constexpr float SPAWN_AREA = 10.0f;
static constexpr int KIND_COUNT = 3;
/** one body respawned every this many steps, half a second at 60 */
static constexpr int RESPAWN_STEPS = 30;
static constexpr float RAY_LENGTH = 100.0f;
static constexpr int MAX_BRUSH_SIZE = 10;
/** chunks remeshed a frame after an edit */
static constexpr int REMESHES_PER_FRAME = 3;
static constexpr float PI = 3.14159265f;

/* a voxel shape in user slot 1: chunks of 16^3 voxels in a fixed box of chunks, which the shape points at */

static constexpr ShapeType SHAPE_VOXELS = ShapeType::USER_1;

/** a voxel's six faces: a hit's sub shape id is the face it hit, which is all the shape's functions need */
enum VoxelFace : uint32_t { NEGATIVE_X, POSITIVE_X, NEGATIVE_Y, POSITIVE_Y, NEGATIVE_Z, POSITIVE_Z, VOXEL_FACE_COUNT };

static const float FACE_NORMALS[VOXEL_FACE_COUNT][3] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};

namespace {

struct Chunk {
    int cx, cy, cz;
    /** filled voxels */
    int sum;
    uint8_t voxels[CHUNK_VOXEL_COUNT];
};

}  // namespace

struct Voxels {
    /** in chunks: min x y z, then max x y z, exclusive */
    int chunk_bounds[6];
    int chunk_count[3];
    /** one per chunk in the bounds, null until a voxel in it is set */
    collections::Array<Chunk *> chunks;
};

struct VoxelsShape : Shape {
    static constexpr ShapeType TYPE = SHAPE_VOXELS;
    Voxels *voxels;
};

/** a chunk's surface as quads, for drawing */
struct Quad {
    Float3 corners[4];
    uint32_t face;
};

namespace {

struct Kind {
    Shape *shape;
    Color color;
};

}  // namespace

static struct {
    app::Layers layers;
    Voxels voxels;
    Ref<Shape> voxel_shape;
    /** what a convex shape is collided against, one voxel at a time */
    Shape *voxel_box;
    collections::Array<collections::Array<Quad>> chunk_quads;
    collections::Array<bool> dirty;
    Kind kinds[KIND_COUNT];
    collections::Array<RigidBody *> spawned;
    int count;
    int respawn_steps;
    int steps;
    uint32_t next_respawn;
    Random random;
    Filter terrain_only;
    Filter everything;
    CastRayCollector ray_hit;
    bool hovering;
    Float3 hit_position;
    Float3 hit_normal;
    int brush_size;
    bool brush_keys_down[2];
} state;

static Color hex(uint32_t rgb) { return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, 1.0f}; }

static const Voxels &voxels_of(const Shape *shape) { return *shapes::as<VoxelsShape>(shape).voxels; }

static int floor_div_chunk(int voxel) { return voxel >> CHUNK_BITS; }

/** the chunk's slot, or -1 outside the bounds */
static int chunk_index(const Voxels &voxels, int cx, int cy, int cz) {
    const int *bounds = voxels.chunk_bounds;
    if (cx < bounds[0] || cx >= bounds[3] || cy < bounds[1] || cy >= bounds[4] || cz < bounds[2] || cz >= bounds[5]) return -1;
    return (cz - bounds[2]) * voxels.chunk_count[1] * voxels.chunk_count[0] + (cy - bounds[1]) * voxels.chunk_count[0] + (cx - bounds[0]);
}

static int voxel_index(int x, int y, int z) { return (z & CHUNK_MASK) * CHUNK_SIZE * CHUNK_SIZE + (y & CHUNK_MASK) * CHUNK_SIZE + (x & CHUNK_MASK); }

static bool get_voxel(const Voxels &voxels, int x, int y, int z) {
    const int index = chunk_index(voxels, floor_div_chunk(x), floor_div_chunk(y), floor_div_chunk(z));
    if (index < 0) return false;
    const Chunk *chunk = voxels.chunks[index];
    return chunk != nullptr && chunk->voxels[voxel_index(x, y, z)] != 0;
}

static void set_voxel(Voxels &voxels, int x, int y, int z, bool filled) {
    const int index = chunk_index(voxels, floor_div_chunk(x), floor_div_chunk(y), floor_div_chunk(z));
    if (index < 0) return;
    Chunk *&chunk = voxels.chunks[index];
    if (chunk == nullptr) {
        chunk = util::alloc<Chunk>();
        *chunk = {floor_div_chunk(x), floor_div_chunk(y), floor_div_chunk(z), 0, {}};
    }
    uint8_t &voxel = chunk->voxels[voxel_index(x, y, z)];
    if (voxel != 0 && !filled) chunk->sum--;
    if (voxel == 0 && filled) chunk->sum++;
    voxel = filled ? 1 : 0;
}

static VoxelFace face_from_normal(float x, float y, float z) {
    const float ax = fabsf(x), ay = fabsf(y), az = fabsf(z);
    if (ax > ay && ax > az) return x > 0.0f ? POSITIVE_X : NEGATIVE_X;
    if (ay > az) return y > 0.0f ? POSITIVE_Y : NEGATIVE_Y;
    return z > 0.0f ? POSITIVE_Z : NEGATIVE_Z;
}

static VoxelFace face_of(SubShapeId sub_shape_id) {
    SubShapeIdBuilder::PopResult popped;
    sub_shape::pop_index(popped, sub_shape_id, VOXEL_FACE_COUNT);
    return (VoxelFace)popped.value;
}

static SubShapeId with_face(const SubShapeIdBuilder &parent, VoxelFace face) {
    SubShapeIdBuilder out;
    sub_shape::push_index(out, parent, face, VOXEL_FACE_COUNT);
    return out.value;
}

/* the shape's functions */

// static only: no mass
static void voxels_mass_properties(MassProperties &out, const Shape *) { out = mass_properties::create(); }

static void voxels_surface_normal(SurfaceNormalResult &io, const Shape *, SubShapeId sub_shape_id) {
    const float *normal = FACE_NORMALS[face_of(sub_shape_id)];
    io.normal = vec3f::make(normal[0], normal[1], normal[2]);
}

static int32_t voxels_material(const Shape *, SubShapeId) { return -1; }

/** the voxel holding a point on its face: a point on a positive face sits on the boundary with the next voxel */
static void voxel_at_face(int out[3], const float hit[3], VoxelFace face) {
    for (int axis = 0; axis < 3; axis++) {
        out[axis] = (int)floorf(hit[axis]);
        if (face == (VoxelFace)(2 * axis + 1) && fabsf(hit[axis] - roundf(hit[axis])) < 1e-5f) out[axis] = (int)roundf(hit[axis]) - 1;
    }
}

// the quad of the face named, about the voxel at the transform's translation
static void voxels_supporting_face(SupportingFaceResult &io, Vec3f, const Shape *, SubShapeId sub_shape_id) {
    const VoxelFace face = face_of(sub_shape_id);
    const Vec3f at = mat4f::get_translation(io.transform);
    const float hit[3] = {vec3f::x(at), vec3f::y(at), vec3f::z(at)};
    int voxel[3];
    voxel_at_face(voxel, hit, face);
    const float x = (float)voxel[0], y = (float)voxel[1], z = (float)voxel[2];
    // counter clockwise seen from outside
    static const float CORNERS[VOXEL_FACE_COUNT][4][3] = {
        {{0, 0, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}}, {{1, 0, 0}, {1, 0, 1}, {1, 1, 1}, {1, 1, 0}},
        {{0, 0, 0}, {0, 0, 1}, {1, 0, 1}, {1, 0, 0}}, {{0, 1, 0}, {1, 1, 0}, {1, 1, 1}, {0, 1, 1}},
        {{0, 0, 0}, {0, 1, 0}, {1, 1, 0}, {1, 0, 0}}, {{0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}},
    };
    for (int i = 0; i < 4; i++) io.face.vertices[i] = vec3f::make(x + CORNERS[face][i][0], y + CORNERS[face][i][1], z + CORNERS[face][i][2]);
    io.face.num_vertices = 4;
    face::transform_with_mat4_scale(io.face, io.transform, io.scale);
}

/** a dda walk from the ray's entry into the bounds, jumping over empty chunks whole */
static bool cast_ray_dda(CastRayCollector &collector, const Rayf &ray, const Voxels &voxels,
                         const SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    const Quatf inverse = quatf::conjugate(quatf::load(transform.quaternion));
    const Vec3f scale = vec3f::abs(vec3f::load(transform.scale));
    const Vec3f origin = vec3f::divide(vec3f::transform_quat(vec3f::subtract(ray.origin, vec3f::load(transform.position)), inverse), scale);
    Vec3f direction = vec3f::divide(vec3f::transform_quat(ray.direction, inverse), scale);
    const float direction_length = vec3f::length(direction);
    if (direction_length < 1e-10f) return false;
    direction = vec3f::scale(direction, 1.0f / direction_length);
    const float max_distance = ray.length * direction_length;

    const float o[3] = {vec3f::x(origin), vec3f::y(origin), vec3f::z(origin)};
    const float d[3] = {vec3f::x(direction), vec3f::y(direction), vec3f::z(direction)};
    float inverse_d[3], t_min[3], t_max_bounds[3];
    int step[3];
    for (int axis = 0; axis < 3; axis++) {
        inverse_d[axis] = fabsf(d[axis]) > 1e-10f ? 1.0f / d[axis] : INFINITY;
        const float t1 = ((float)(voxels.chunk_bounds[axis] * CHUNK_SIZE) - o[axis]) * inverse_d[axis];
        const float t2 = ((float)(voxels.chunk_bounds[3 + axis] * CHUNK_SIZE) - o[axis]) * inverse_d[axis];
        t_min[axis] = fminf(t1, t2);
        t_max_bounds[axis] = fmaxf(t1, t2);
        step[axis] = d[axis] > 0.0f ? 1 : d[axis] < 0.0f ? -1 : 0;
    }
    const float t_enter = fmaxf(0.0f, fmaxf(t_min[0], fmaxf(t_min[1], t_min[2])));
    const float t_exit = fminf(max_distance, fminf(t_max_bounds[0], fminf(t_max_bounds[1], t_max_bounds[2])));
    if (t_enter >= t_exit || t_exit < 0.0f) return false;

    int v[3];
    for (int axis = 0; axis < 3; axis++) {
        const float start = o[axis] + d[axis] * t_enter;
        v[axis] = (int)floorf(start);
        // entering on a boundary going down: the voxel inside is the one below
        if (t_enter > 0.0f && step[axis] < 0 && start - (float)v[axis] < 1e-8f) v[axis]--;
    }
    float t_max[3], t_delta[3];
    const auto next_boundary = [&](int axis) {
        if (step[axis] > 0) return ((float)(v[axis] + 1) - o[axis]) * inverse_d[axis];
        if (step[axis] < 0) return ((float)v[axis] - o[axis]) * inverse_d[axis];
        return INFINITY;
    };
    for (int axis = 0; axis < 3; axis++) {
        t_max[axis] = next_boundary(axis);
        t_delta[axis] = fabsf(inverse_d[axis]);
    }

    float distance = t_enter;
    // the axis last crossed, which names the face hit; -1 for a ray starting inside
    int last_axis = -1;
    if (t_enter > 0.0f) last_axis = t_min[0] >= t_min[1] && t_min[0] >= t_min[2] ? 0 : t_min[1] >= t_min[2] ? 1 : 2;

    const float travel = max_distance - t_enter;
    int max_steps = 1;
    for (int axis = 0; axis < 3; axis++) {
        if (t_delta[axis] < INFINITY) max_steps += (int)ceilf(travel / t_delta[axis]);
    }

    for (int i = 0; i < max_steps; i++) {
        const int index = chunk_index(voxels, floor_div_chunk(v[0]), floor_div_chunk(v[1]), floor_div_chunk(v[2]));
        if (index < 0) return false;
        const Chunk *chunk = voxels.chunks[index];
        if (chunk == nullptr || chunk->sum == 0) {
            // to where the ray leaves this chunk
            float exit_at[3] = {INFINITY, INFINITY, INFINITY};
            for (int axis = 0; axis < 3; axis++) {
                const int chunk_min = floor_div_chunk(v[axis]) * CHUNK_SIZE;
                float t = INFINITY;
                if (d[axis] > 0.0f) t = ((float)(chunk_min + CHUNK_SIZE) - o[axis]) * inverse_d[axis];
                else if (d[axis] < 0.0f) t = ((float)chunk_min - o[axis]) * inverse_d[axis];
                if (t > distance) exit_at[axis] = t;
            }
            const float t = fminf(exit_at[0], fminf(exit_at[1], exit_at[2]));
            if (t >= max_distance || t == INFINITY) return false;
            last_axis = t == exit_at[0] ? 0 : t == exit_at[1] ? 1 : 2;
            for (int axis = 0; axis < 3; axis++) v[axis] = (int)floorf(o[axis] + d[axis] * (t + 0.0001f));
            for (int axis = 0; axis < 3; axis++) {
                if (fabsf(d[axis]) > 1e-10f) t_max[axis] = next_boundary(axis);
            }
            distance = t;
            continue;
        }

        if (chunk->voxels[voxel_index(v[0], v[1], v[2])] != 0) {
            const float fraction = fmaxf(0.0f, distance) / max_distance;
            if (fraction > collector.early_out_fraction) return false;
            int axis = last_axis;
            if (axis < 0) {
                // started inside: the face it would have come in by
                const float ax = fabsf(d[0]), ay = fabsf(d[1]), az = fabsf(d[2]);
                axis = ax > ay && ax > az ? 0 : ay > az ? 1 : 2;
            }
            // the face against the ray on the axis crossed
            const VoxelFace face = (VoxelFace)(2 * axis + (step[axis] > 0 ? 0 : 1));
            CastRayHit hit = cast_ray::create_hit();
            hit.status = CastRayStatus::COLLIDING;
            hit.fraction = fraction;
            hit.sub_shape_id = with_face(sub_shape_id, face);
            hit.body_id_b = collector.body_id_b;
            cast_ray::add_hit(collector, hit);
            return true;
        }

        int axis = t_max[0] < t_max[1] ? (t_max[0] < t_max[2] ? 0 : 2) : (t_max[1] < t_max[2] ? 1 : 2);
        if (t_max[axis] > max_distance) return false;
        v[axis] += step[axis];
        distance = t_max[axis];
        t_max[axis] += t_delta[axis];
        last_axis = axis;
    }
    return false;
}

static void voxels_cast_ray(CastRayCollector &collector, const CastRaySettings &, const Rayf &ray,
                            const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    if (!cast_ray_dda(collector, ray, voxels_of(shape), sub_shape_id, transform)) cast_ray::add_miss(collector);
}

static void voxels_collide_point(CollidePointCollector &collector, const CollidePointSettings &, Vec3f point,
                                 const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    const Quatf inverse = quatf::conjugate(quatf::load(transform.quaternion));
    const Vec3f local = vec3f::divide(vec3f::transform_quat(vec3f::subtract(point, vec3f::load(transform.position)), inverse),
                                      vec3f::abs(vec3f::load(transform.scale)));
    const float p[3] = {vec3f::x(local), vec3f::y(local), vec3f::z(local)};
    const int v[3] = {(int)floorf(p[0]), (int)floorf(p[1]), (int)floorf(p[2])};
    if (!get_voxel(voxels_of(shape), v[0], v[1], v[2])) {
        collide_point::add_miss(collector);
        return;
    }
    // the face nearest the point
    float distance[3];
    for (int axis = 0; axis < 3; axis++) distance[axis] = 0.5f - fabsf(p[axis] - ((float)v[axis] + 0.5f));
    const int axis = distance[0] <= distance[1] && distance[0] <= distance[2] ? 0 : distance[1] <= distance[2] ? 1 : 2;
    const VoxelFace face = (VoxelFace)(2 * axis + (p[axis] - ((float)v[axis] + 0.5f) > 0.0f ? 1 : 0));
    CollidePointHit hit = collide_point::create_hit();
    hit.sub_shape_id_b = with_face(sub_shape_id, face);
    hit.body_id_b = collector.body_id_b;
    collide_point::add_hit(collector, hit);
}

/** a convex shape against each filled voxel its bounds reach, each voxel a box */
static void collide_voxels_vs_convex(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                     const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                     const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const Voxels &voxels = voxels_of(shape_a);
    const Quatf rotation_a = quatf::load(transform_a.quaternion);
    const Vec3f position_a = vec3f::load(transform_a.position);
    const Quatf inverse_a = quatf::conjugate(rotation_a);
    const Vec3f position_b_in_a = vec3f::divide(vec3f::transform_quat(vec3f::subtract(vec3f::load(transform_b.position), position_a), inverse_a),
                                                vec3f::abs(vec3f::load(transform_a.scale)));
    const Quatf rotation_b_in_a = quatf::multiply(inverse_a, quatf::load(transform_b.quaternion));
    const Vec3f scale_b = vec3f::load(transform_b.scale);
    const AABox reach = aabox::expand_by_margin(
        aabox::transform_mat4(aabox::load(shape_b->aabb), mat4f::from_rotation_translation_scale(rotation_b_in_a, position_b_in_a, scale_b)),
        settings.max_separation_distance);
    const int min[3] = {(int)floorf(vec3f::x(reach.min)), (int)floorf(vec3f::y(reach.min)), (int)floorf(vec3f::z(reach.min))};
    const int max[3] = {(int)ceilf(vec3f::x(reach.max)), (int)ceilf(vec3f::y(reach.max)), (int)ceilf(vec3f::z(reach.max))};
    for (int z = min[2]; z <= max[2]; z++) {
        for (int y = min[1]; y <= max[1]; y++) {
            for (int x = min[0]; x <= max[0]; x++) {
                if (!get_voxel(voxels, x, y, z)) continue;
                const Vec3f centre = vec3f::make((float)x + 0.5f, (float)y + 0.5f, (float)z + 0.5f);
                const Vec3f b_from_centre = vec3f::subtract(position_b_in_a, centre);
                // which face, by where b is from the voxel's centre: a guess, as the contact normal may differ a little
                const VoxelFace face = face_from_normal(vec3f::x(b_from_centre), vec3f::y(b_from_centre), vec3f::z(b_from_centre));
                const Mat4f box_in_world = mat4f::from_rotation_translation(rotation_a, vec3f::add(position_a, vec3f::transform_quat(centre, rotation_a)));
                const Mat4f b_in_box = mat4f::from_rotation_translation(rotation_b_in_a, b_from_centre);
                convex::collide_convex_vs_convex_local(collector, settings, state.voxel_box, with_face(sub_shape_id_a, face), shape_b,
                                                       sub_shape_id_b.value, b_in_box, box_in_world, vec3f::make(1.0f, 1.0f, 1.0f), scale_b);
                if (collide_shape::should_early_out(collector)) return;
            }
        }
    }
}

/** frees the shape when the last Ref to it lets go: the terrain body's, then the example's */
static void destroy_voxel_shape(Shape *shape) { shapes::release<VoxelsShape>(shape); }

static ShapeDef voxels_def() {
    ShapeDef def = {};
    def.type = SHAPE_VOXELS;
    def.destroy = destroy_voxel_shape;
    def.compute_mass_properties = voxels_mass_properties;
    def.get_surface_normal = voxels_surface_normal;
    def.get_supporting_face = voxels_supporting_face;
    def.get_material = voxels_material;
    def.cast_ray = voxels_cast_ray;
    def.collide_point = voxels_collide_point;
    return shapes::define_shape(def);
}

/** the voxel type beside the built-ins, colliding with every convex type both ways; no casts against it */
static void register_voxels() {
    const ShapeDef def = voxels_def();
    dispatch::init(&def, 1);
    for (uint32_t type = 0; type < shapes::SHAPE_TYPE_COUNT; type++) {
        if (shapes::shape_defs[type].set_support == nullptr) continue;
        shapes::set_collide_shape_fn(SHAPE_VOXELS, (ShapeType)type, collide_voxels_vs_convex);
        shapes::set_collide_shape_fn((ShapeType)type, SHAPE_VOXELS, collide_shape::reversed_collide_shape_vs_shape<collide_voxels_vs_convex>);
    }
}

static Shape *create_voxel_shape(Voxels *voxels) {
    VoxelsShape *shape = shapes::allocate<VoxelsShape>();
    shape->voxels = voxels;
    const int *bounds = voxels->chunk_bounds;
    shape->aabb = {{(float)(bounds[0] * CHUNK_SIZE), (float)(bounds[1] * CHUNK_SIZE), (float)(bounds[2] * CHUNK_SIZE)},
                   {(float)(bounds[3] * CHUNK_SIZE), (float)(bounds[4] * CHUNK_SIZE), (float)(bounds[5] * CHUNK_SIZE)}};
    shape->center_of_mass = {0.0f, 0.0f, 0.0f};
    shape->volume = 0.0f;
    shape->material_id = -1;
    return shape;
}

/* simplex noise, as crashcat's examples make their terrain */

namespace {

struct Simplex2d {
    uint8_t perm[512];
    int8_t gradients[512][2];
};

}  // namespace

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

static void make_terrain() {
    Voxels &voxels = state.voxels;
    // chunks to cover x and z from -32 to 32, and y from 0 to the highest hill
    const int bounds[6] = {-TERRAIN_SIZE / CHUNK_SIZE, 0, -TERRAIN_SIZE / CHUNK_SIZE, TERRAIN_SIZE / CHUNK_SIZE,
                           (BASE_HEIGHT + TERRAIN_HEIGHT + CHUNK_SIZE - 1) / CHUNK_SIZE, TERRAIN_SIZE / CHUNK_SIZE};
    for (int i = 0; i < 6; i++) voxels.chunk_bounds[i] = bounds[i];
    for (int axis = 0; axis < 3; axis++) voxels.chunk_count[axis] = bounds[3 + axis] - bounds[axis];
    const uint32_t total = (uint32_t)(voxels.chunk_count[0] * voxels.chunk_count[1] * voxels.chunk_count[2]);
    voxels.chunks.length = 0;
    for (uint32_t i = 0; i < total; i++) voxels.chunks.push(nullptr);

    Simplex2d noise;
    simplex2d_create(noise, 42);
    for (int x = -TERRAIN_SIZE; x < TERRAIN_SIZE; x++) {
        for (int z = -TERRAIN_SIZE; z < TERRAIN_SIZE; z++) {
            // large hills, medium detail and small detail
            const double hills = simplex2d_sample(noise, x * 0.02, z * 0.02) * 0.5 + 0.5;
            const double medium = simplex2d_sample(noise, x * 0.08, z * 0.08) * 0.5 + 0.5;
            const double small = simplex2d_sample(noise, x * 0.2, z * 0.2) * 0.5 + 0.5;
            const int height = (int)floor(BASE_HEIGHT + (hills * 0.6 + medium * 0.3 + small * 0.1) * TERRAIN_HEIGHT);
            for (int y = 0; y < height; y++) set_voxel(voxels, x, y, z, true);
        }
    }
}

/* drawing: each chunk's culled quads, remeshed a few chunks a frame after an edit */

/** a quad wherever a filled voxel of the chunk meets an empty one, counter clockwise seen from the empty side */
static void mesh_chunk(collections::Array<Quad> &out, const Chunk &chunk) {
    out.length = 0;
    const int origin[3] = {chunk.cx * CHUNK_SIZE, chunk.cy * CHUNK_SIZE, chunk.cz * CHUNK_SIZE};
    for (int x = -1; x < CHUNK_SIZE; x++) {
        for (int z = -1; z < CHUNK_SIZE; z++) {
            for (int y = -1; y < CHUNK_SIZE; y++) {
                const int at[3] = {origin[0] + x, origin[1] + y, origin[2] + z};
                const bool solid = get_voxel(state.voxels, at[0], at[1], at[2]);
                for (int axis = 0; axis < 3; axis++) {
                    int next[3] = {at[0], at[1], at[2]};
                    next[axis]++;
                    if (solid == get_voxel(state.voxels, next[0], next[1], next[2])) continue;
                    int owner[3] = {x, y, z};
                    if (!solid) owner[axis]++;
                    if (owner[0] < 0 || owner[0] >= CHUNK_SIZE || owner[1] < 0 || owner[1] >= CHUNK_SIZE || owner[2] < 0 || owner[2] >= CHUNK_SIZE) continue;
                    float u[3] = {0.0f, 0.0f, 0.0f};
                    float v[3] = {0.0f, 0.0f, 0.0f};
                    u[(axis + (solid ? 1 : 2)) % 3] = 1.0f;
                    v[(axis + (solid ? 2 : 1)) % 3] = 1.0f;
                    const Float3 corner = {(float)next[0], (float)next[1], (float)next[2]};
                    const Float3 du = {u[0], u[1], u[2]};
                    const Float3 dv = {v[0], v[1], v[2]};
                    out.push({{corner, vec::add(corner, du), vec::add(vec::add(corner, du), dv), vec::add(corner, dv)}, (uint32_t)(2 * axis + (solid ? 1 : 0))});
                }
            }
        }
    }
}

static void mark_dirty(int cx, int cy, int cz) {
    const int index = chunk_index(state.voxels, cx, cy, cz);
    if (index >= 0) state.dirty[index] = true;
}

/** the voxel's chunk, and the chunk beside it where the voxel is on its edge */
static void mark_voxel_dirty(int x, int y, int z) {
    const int at[3] = {x, y, z};
    const int chunk[3] = {floor_div_chunk(x), floor_div_chunk(y), floor_div_chunk(z)};
    mark_dirty(chunk[0], chunk[1], chunk[2]);
    for (int axis = 0; axis < 3; axis++) {
        const int local = at[axis] & CHUNK_MASK;
        if (local != 0 && local != CHUNK_SIZE - 1) continue;
        int beside[3] = {chunk[0], chunk[1], chunk[2]};
        beside[axis] += local == 0 ? -1 : 1;
        mark_dirty(beside[0], beside[1], beside[2]);
    }
}

static void remesh_dirty(int most) {
    for (uint32_t i = 0; i < state.dirty.length && most > 0; i++) {
        if (!state.dirty[i]) continue;
        state.dirty[i] = false;
        const Chunk *chunk = state.voxels.chunks[i];
        if (chunk == nullptr || chunk->sum == 0) state.chunk_quads[i].length = 0;
        else mesh_chunk(state.chunk_quads[i], *chunk);
        most--;
    }
}

static void draw_terrain(DrawList &list) {
    const Float3 light = vec::normalize({0.4f, 1.0f, 0.3f});
    const Color base = hex(0x999999);
    Color shaded[VOXEL_FACE_COUNT];
    for (uint32_t face = 0; face < VOXEL_FACE_COUNT; face++) {
        // as the app's meshes are lit
        const float shade = powf(0.35f + 0.65f * (0.5f + 0.5f * vec::dot({FACE_NORMALS[face][0], FACE_NORMALS[face][1], FACE_NORMALS[face][2]}, light)), 1.0f / 2.2f);
        shaded[face] = {base.r * shade, base.g * shade, base.b * shade, 1.0f};
    }
    for (uint32_t i = 0; i < state.chunk_quads.length; i++) {
        const collections::Array<Quad> &quads = state.chunk_quads[i];
        for (uint32_t q = 0; q < quads.length; q++) {
            const Quad &quad = quads[q];
            renderer::triangle(list, quad.corners[0], quad.corners[1], quad.corners[3], shaded[quad.face]);
            renderer::triangle(list, quad.corners[1], quad.corners[2], quad.corners[3], shaded[quad.face]);
        }
    }
}

/* editing: the pointer's ray against the terrain, a click breaks or builds a cube of voxels */

static void wake_body(BodyVisitor *visitor, RigidBody *body) {
    rigid_body::wake(*static_cast<World *>(visitor->context), body);
}

static void edit(App &app, bool build) {
    const int half = state.brush_size / 2;
    // breaking takes the voxel just behind the surface, building the one just in front
    const float along = build ? 0.5f : -0.01f;
    const Float3 at = vec::scale_and_add(state.hit_position, state.hit_normal, along);
    const int centre[3] = {(int)floorf(at.x), (int)floorf(at.y), (int)floorf(at.z)};
    for (int dx = -half; dx <= half; dx++) {
        for (int dy = -half; dy <= half; dy++) {
            for (int dz = -half; dz <= half; dz++) {
                set_voxel(state.voxels, centre[0] + dx, centre[1] + dy, centre[2] + dz, build);
                mark_voxel_dirty(centre[0] + dx, centre[1] + dy, centre[2] + dz);
            }
        }
    }
    const float reach = (float)(half + 1);
    const Box3f around = {{(float)centre[0] - reach, (float)centre[1] - reach, (float)centre[2] - reach},
                          {(float)centre[0] + reach, (float)centre[1] + reach, (float)centre[2] + reach}};
    BodyVisitor visitor = body_visitor::create(wake_body, app.world);
    broadphase::intersect_aabb(app.world->broadphase, app.world->bodies, around, state.everything, visitor);
}

static void cast_pointer(App &app) {
    cast_ray::reset(state.ray_hit);
    const Vec3f origin = vec3f::load(app.pointer.ray.origin);
    const Vec3f direction = vec3f::load(app.pointer.ray.direction);
    query::cast_ray(*app.world, state.ray_hit, cast_ray::default_settings(), {origin, direction, RAY_LENGTH}, state.terrain_only);
    const CastRayHit *hit = cast_ray::hit(state.ray_hit);
    state.hovering = hit != nullptr;
    if (hit == nullptr) return;
    const Vec3f position = vec3f::scale_and_add(origin, direction, RAY_LENGTH * hit->fraction);
    const Vec3f normal = shapes::get_shape_surface_normal(state.voxel_shape, position, hit->sub_shape_id);
    vec3f::store(state.hit_position, position);
    vec3f::store(state.hit_normal, normal);
}

/** the hit point, its normal, and the cube of voxels a break would take */
static void draw_brush(DrawList &list) {
    if (!state.hovering) return;
    renderer::point(list, state.hit_position, 0.3f, {1.0f, 0.0f, 0.0f, 1.0f});
    renderer::arrow(list, state.hit_position, vec::scale_and_add(state.hit_position, state.hit_normal, 2.0f), {0.0f, 1.0f, 0.0f, 1.0f});
    const Float3 inside = vec::scale_and_add(state.hit_position, state.hit_normal, -0.01f);
    const Float3 centre = {floorf(inside.x) + 0.5f, floorf(inside.y) + 0.5f, floorf(inside.z) + 0.5f};
    const float h = 0.5f * (float)state.brush_size;
    const Color white = {1.0f, 1.0f, 1.0f, 0.8f};
    for (int axis = 0; axis < 3; axis++) {
        for (int corner = 0; corner < 4; corner++) {
            float from[3], to[3];
            const int u = (axis + 1) % 3, v = (axis + 2) % 3;
            from[axis] = -h;
            to[axis] = h;
            from[u] = to[u] = (corner & 1) ? h : -h;
            from[v] = to[v] = (corner & 2) ? h : -h;
            renderer::line(list, vec::add(centre, {from[0], from[1], from[2]}), vec::add(centre, {to[0], to[1], to[2]}), white);
        }
    }
}

/* the bodies dropped on it */

static void make_kinds(App &app) {
    state.kinds[0] = {app::keep(app, sphere::create({.radius = 0.5f})), hex(0xff6b6b)};
    state.kinds[1] = {app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.05f})), hex(0x4ecdc4)};
    state.kinds[2] = {app::keep(app, capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f})), hex(0xdfe6e9)};
}

static RigidBody *spawn(App &app, float restitution) {
    const Kind &kind = state.kinds[(uint32_t)(seeded::next(state.random) * KIND_COUNT) % KIND_COUNT];
    RigidBodySettings settings;
    settings.shape = kind.shape;
    settings.object_layer = state.layers.moving;
    settings.motion_type = MotionType::DYNAMIC;
    settings.position = {seeded::range(state.random, -SPAWN_AREA, SPAWN_AREA), SPAWN_HEIGHT, seeded::range(state.random, -SPAWN_AREA, SPAWN_AREA)};
    const Float3 axis = {seeded::next(state.random) - 0.5f, seeded::next(state.random) - 0.5f, seeded::next(state.random) - 0.5f};
    settings.quaternion = vec::quat_axis_angle(axis, seeded::next(state.random) * 2.0f * PI);
    settings.restitution = restitution;
    settings.friction = 0.5f;
    RigidBody *body = rigid_body::create(*app.world, settings);
    app::set_color(app, body, kind.color);
    return body;
}

static void match_count(App &app) {
    while ((int)state.spawned.length < state.count) state.spawned.push(spawn(app, 0.0f));
    while ((int)state.spawned.length > state.count) rigid_body::remove(*app.world, state.spawned[--state.spawned.length]);
}

static void init(App &app) {
    register_voxels();
    state.spawned.length = 0;
    state.count = 50;
    state.respawn_steps = RESPAWN_STEPS;
    state.steps = 0;
    state.next_respawn = 0;
    state.random = {5};
    state.hovering = false;
    state.brush_size = 1;
    state.brush_keys_down[0] = state.brush_keys_down[1] = false;
    app.world = app::create_world(state.layers);
    state.terrain_only = filter::create(app.world->settings.layers);
    filter::disable_object_layer(state.terrain_only, app.world->settings.layers, state.layers.moving);
    state.everything = filter::create(app.world->settings.layers);
    state.ray_hit = cast_ray::create_collector(CollectorKind::CLOSEST);

    make_terrain();
    // the example's own: app::keep frees built-in types alone
    state.voxel_shape = create_voxel_shape(&state.voxels);
    state.voxel_box = app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.05f}));
    RigidBodySettings terrain;
    terrain.shape = state.voxel_shape;
    terrain.object_layer = state.layers.not_moving;
    terrain.motion_type = MotionType::STATIC;
    rigid_body::create(*app.world, terrain);

    state.chunk_quads.length = 0;
    state.dirty.length = 0;
    for (uint32_t i = 0; i < state.voxels.chunks.length; i++) {
        state.chunk_quads.push({});
        state.dirty.push(true);
    }
    remesh_dirty((int)state.voxels.chunks.length);

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
    RigidBody *&oldest = state.spawned[state.next_respawn++ % state.spawned.length];
    rigid_body::remove(*app.world, oldest);
    oldest = spawn(app, 0.3f);
}

static void update(App &app, float) {
    const bool smaller = app.keys[SAPP_KEYCODE_LEFT_BRACKET];
    const bool larger = app.keys[SAPP_KEYCODE_RIGHT_BRACKET];
    if (smaller && !state.brush_keys_down[0] && state.brush_size > 1) state.brush_size--;
    if (larger && !state.brush_keys_down[1] && state.brush_size < MAX_BRUSH_SIZE) state.brush_size++;
    state.brush_keys_down[0] = smaller;
    state.brush_keys_down[1] = larger;

    cast_pointer(app);
    // left click breaks, right click builds
    if (app.click.clicked && state.hovering) edit(app, app.click.secondary);
    remesh_dirty(REMESHES_PER_FRAME);

    DrawList &list = app::frame(app);
    draw_terrain(list);
    draw_brush(list);
}

static void ui(App &app) {
    if (ImGui::SliderInt("bodies", &state.count, 0, 1000)) match_count(app);
    ImGui::SliderInt("respawn every (steps)", &state.respawn_steps, 6, 300);
    ImGui::Text("brush %d ([ and ]): left click breaks, right click builds", state.brush_size);
}

static void shutdown(App &app) {
    // the world before the shape its terrain is made of
    app::clear_world(app);
    state.voxel_shape = nullptr;
    for (uint32_t i = 0; i < state.voxels.chunks.length; i++) {
        if (state.voxels.chunks[i] != nullptr) util::free(state.voxels.chunks[i]);
    }
    state.voxels.chunks.release();
    state.chunk_quads.release();
    state.dirty.release();
    state.spawned.release();
    state.ray_hit = {};
    state.terrain_only = {};
    // back to the built-in types alone
    dispatch::init();
    state.everything = {};
}

static const Example EXAMPLE_VOXEL_CUSTOM_SHAPE = {"Voxel Custom Shape", init, step, update, ui, shutdown, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_VOXEL_CUSTOM_SHAPE);
