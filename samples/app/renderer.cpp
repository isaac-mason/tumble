#include "app/renderer.h"

#include <math.h>

#include "app/shape_mesh.h"
#include "app/vec.h"

using namespace tumble;
using namespace vec;

namespace renderer {

#if defined(SOKOL_GLES3)
#define SHADER_HEADER "#version 300 es\nprecision highp float;\n"
#else
#define SHADER_HEADER "#version 410\n"
#endif

static const char *COLOR_VERTEX_SHADER = SHADER_HEADER
    "uniform mat4 mvp;\n"
    "layout(location = 0) in vec3 position;\n"
    "layout(location = 1) in vec4 color0;\n"
    "out vec4 color;\n"
    "void main() {\n"
    "    gl_Position = mvp * vec4(position, 1.0);\n"
    "    color = color0;\n"
    "}\n";

static const char *COLOR_FRAGMENT_SHADER = SHADER_HEADER
    "in vec4 color;\n"
    "layout(location = 0) out vec4 frag_color;\n"
    "void main() {\n"
    "    frag_color = color;\n"
    "}\n";

static const char *MESH_VERTEX_SHADER = SHADER_HEADER
    "uniform mat4 mvp;\n"
    "uniform mat4 model;\n"
    "uniform vec4 tint;\n"
    "uniform vec4 light;\n"
    "layout(location = 0) in vec3 position;\n"
    "layout(location = 1) in vec3 normal;\n"
    "out vec4 color;\n"
    "void main() {\n"
    "    gl_Position = mvp * model * vec4(position, 1.0);\n"
    "    vec3 n = normalize(mat3(model) * normal);\n"
    "    float shade = 0.35 + 0.65 * (0.5 + 0.5 * dot(n, light.xyz));\n"
    "    color = vec4(pow(pow(tint.rgb, vec3(2.2)) * shade, vec3(1.0 / 2.2)), tint.a);\n"
    "}\n";

struct ColorUniforms {
    float mvp[16];
};

struct MeshUniforms {
    float mvp[16];
    float model[16];
    float tint[4];
    float light[4];
};

static uint32_t pack(Color color) {
    const auto byte = [](float value) { return (uint32_t)(fminf(fmaxf(value, 0.0f), 1.0f) * 255.0f + 0.5f); };
    return byte(color.r) | (byte(color.g) << 8) | (byte(color.b) << 16) | (byte(color.a) << 24);
}

static sg_shader make_color_shader() {
    sg_shader_desc desc = {};
    desc.vertex_func.source = COLOR_VERTEX_SHADER;
    desc.fragment_func.source = COLOR_FRAGMENT_SHADER;
    desc.attrs[0].glsl_name = "position";
    desc.attrs[1].glsl_name = "color0";
    desc.uniform_blocks[0].stage = SG_SHADERSTAGE_VERTEX;
    desc.uniform_blocks[0].size = sizeof(ColorUniforms);
    desc.uniform_blocks[0].glsl_uniforms[0].type = SG_UNIFORMTYPE_MAT4;
    desc.uniform_blocks[0].glsl_uniforms[0].glsl_name = "mvp";
    desc.label = "samples color shader";
    return sg_make_shader(&desc);
}

static sg_shader make_mesh_shader() {
    sg_shader_desc desc = {};
    desc.vertex_func.source = MESH_VERTEX_SHADER;
    desc.fragment_func.source = COLOR_FRAGMENT_SHADER;
    desc.attrs[0].glsl_name = "position";
    desc.attrs[1].glsl_name = "normal";
    desc.uniform_blocks[0].stage = SG_SHADERSTAGE_VERTEX;
    desc.uniform_blocks[0].size = sizeof(MeshUniforms);
    desc.uniform_blocks[0].glsl_uniforms[0] = {SG_UNIFORMTYPE_MAT4, 0, "mvp"};
    desc.uniform_blocks[0].glsl_uniforms[1] = {SG_UNIFORMTYPE_MAT4, 0, "model"};
    desc.uniform_blocks[0].glsl_uniforms[2] = {SG_UNIFORMTYPE_FLOAT4, 0, "tint"};
    desc.uniform_blocks[0].glsl_uniforms[3] = {SG_UNIFORMTYPE_FLOAT4, 0, "light"};
    desc.label = "samples mesh shader";
    return sg_make_shader(&desc);
}

static void blend(sg_pipeline_desc &desc) {
    desc.colors[0].blend.enabled = true;
    desc.colors[0].blend.src_factor_rgb = SG_BLENDFACTOR_SRC_ALPHA;
    desc.colors[0].blend.dst_factor_rgb = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
    desc.colors[0].blend.src_factor_alpha = SG_BLENDFACTOR_ONE;
    desc.colors[0].blend.dst_factor_alpha = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
}

static sg_pipeline make_color_pipeline(sg_shader shader, sg_primitive_type primitive_type, bool blended, bool depth_write, const char *label) {
    sg_pipeline_desc desc = {};
    desc.shader = shader;
    desc.layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3;
    desc.layout.attrs[1].format = SG_VERTEXFORMAT_UBYTE4N;
    desc.primitive_type = primitive_type;
    desc.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
    desc.depth.write_enabled = depth_write;
    desc.cull_mode = SG_CULLMODE_NONE;
    if (blended) blend(desc);
    desc.label = label;
    return sg_make_pipeline(&desc);
}

static sg_pipeline make_mesh_pipeline(sg_shader shader, bool blended, const char *label) {
    sg_pipeline_desc desc = {};
    desc.shader = shader;
    desc.layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3;
    desc.layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT3;
    desc.primitive_type = SG_PRIMITIVETYPE_TRIANGLES;
    desc.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
    desc.depth.write_enabled = !blended;
    // the wireframes drawn over a mesh win the depth test
    desc.depth.bias = 1.0f;
    desc.depth.bias_slope_scale = 1.0f;
    desc.cull_mode = SG_CULLMODE_NONE;
    if (blended) blend(desc);
    desc.label = label;
    return sg_make_pipeline(&desc);
}

Renderer create() {
    Renderer renderer = {};
    renderer.color_shader = make_color_shader();
    renderer.mesh_shader = make_mesh_shader();
    renderer.opaque_pipeline = make_color_pipeline(renderer.color_shader, SG_PRIMITIVETYPE_TRIANGLES, false, true, "samples opaque");
    renderer.transparent_pipeline = make_color_pipeline(renderer.color_shader, SG_PRIMITIVETYPE_TRIANGLES, true, false, "samples transparent");
    renderer.line_pipeline = make_color_pipeline(renderer.color_shader, SG_PRIMITIVETYPE_LINES, true, false, "samples lines");
    renderer.mesh_pipeline = make_mesh_pipeline(renderer.mesh_shader, false, "samples meshes");
    renderer.transparent_mesh_pipeline = make_mesh_pipeline(renderer.mesh_shader, true, "samples transparent meshes");
    return renderer;
}

void forget_shapes(Renderer &renderer) {
    collections::Map<ShapeKey, GpuMesh> &meshes = renderer.shape_meshes;
    for (uint32_t i = 0; i < meshes.capacity; i++) {
        if (meshes.slots[i].used && meshes.slots[i].value.buffer.id != SG_INVALID_ID) sg_destroy_buffer(meshes.slots[i].value.buffer);
    }
    meshes.release();
}

void destroy(Renderer &renderer) {
    forget_shapes(renderer);
    for (TransientBuffer &transient : renderer.transient) {
        if (transient.buffer.id != SG_INVALID_ID) sg_destroy_buffer(transient.buffer);
    }
    renderer.frame.opaque_triangles.release();
    renderer.frame.transparent_triangles.release();
    renderer.frame.lines.release();
    renderer.meshes.release();
    sg_destroy_pipeline(renderer.opaque_pipeline);
    sg_destroy_pipeline(renderer.transparent_pipeline);
    sg_destroy_pipeline(renderer.line_pipeline);
    sg_destroy_pipeline(renderer.mesh_pipeline);
    sg_destroy_pipeline(renderer.transparent_mesh_pipeline);
    sg_destroy_shader(renderer.color_shader);
    sg_destroy_shader(renderer.mesh_shader);
}

GpuMesh mesh_of(Renderer &renderer, const Shape *shape) {
    if (const GpuMesh *known = renderer.shape_meshes.get({shape})) return *known;
    collections::Array<MeshVertex> vertices = {};
    shape_mesh::build(vertices, shape);
    GpuMesh mesh = {};
    if (vertices.length > 0) {
        sg_buffer_desc desc = {};
        desc.data = {vertices.data, (size_t)vertices.length * sizeof(MeshVertex)};
        desc.label = "samples shape mesh";
        mesh = {sg_make_buffer(&desc), (int)vertices.length};
    }
    vertices.release();
    renderer.shape_meshes.set({shape}, mesh);
    return mesh;
}

void mesh(Renderer &renderer, GpuMesh mesh, const Mat4 &model, Color color) {
    if (mesh.vertex_count > 0) renderer.meshes.push({mesh, model, color});
}

void clear(Renderer &renderer) {
    renderer.frame.opaque_triangles.length = 0;
    renderer.frame.transparent_triangles.length = 0;
    renderer.frame.lines.length = 0;
    renderer.meshes.length = 0;
}

static void push(collections::Array<ColorVertex> &vertices, Float3 p, uint32_t color) { vertices.push({p.x, p.y, p.z, color}); }

void line(DrawList &list, Float3 a, Float3 b, Color color) {
    const uint32_t packed = pack(color);
    push(list.lines, a, packed);
    push(list.lines, b, packed);
}

void triangle(DrawList &list, Float3 a, Float3 b, Float3 c, Color color) {
    collections::Array<ColorVertex> &vertices = color.a < 1 ? list.transparent_triangles : list.opaque_triangles;
    const uint32_t packed = pack(color);
    push(vertices, a, packed);
    push(vertices, b, packed);
    push(vertices, c, packed);
}

void point(DrawList &list, Float3 position, float radius, Color color) {
    const Float3 axes[6] = {{radius, 0, 0}, {-radius, 0, 0}, {0, radius, 0}, {0, -radius, 0}, {0, 0, radius}, {0, 0, -radius}};
    const int faces[8][3] = {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}, {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}};
    for (const int *face : faces) triangle(list, add(position, axes[face[0]]), add(position, axes[face[1]]), add(position, axes[face[2]]), color);
}

void arrow(DrawList &list, Float3 from, Float3 to, Color color) {
    line(list, from, to, color);
    Float3 direction = subtract(to, from);
    const float l = length(direction);
    if (l < 1e-4f) return;
    direction = scale(direction, 1.0f / l);
    const Float3 reference = fabsf(direction.y) > 0.99f ? Float3{1, 0, 0} : Float3{0, 1, 0};
    const Float3 side = normalize(cross(direction, reference));
    const float head = fminf(0.2f, l * 0.4f);
    const Float3 back = scale_and_add(to, direction, -head);
    line(list, to, scale_and_add(back, side, head * 0.5f), color);
    line(list, to, scale_and_add(back, side, -head * 0.5f), color);
}

void debug(DrawList &list, const DebugDraw &buffers) {
    const auto at = [](const float *v) { return Float3{v[0], v[1], v[2]}; };
    const auto color = [](const float *v) { return Color{v[3], v[4], v[5], 1.0f}; };
    for (uint32_t i = 0; i < buffers.line_count; i++) {
        const float *v = buffers.lines.data + i * debug::LINE_STRIDE;
        push(list.lines, at(v), pack(color(v)));
        push(list.lines, at(v + debug::VERTEX_STRIDE), pack(color(v + debug::VERTEX_STRIDE)));
    }
    for (uint32_t i = 0; i < buffers.triangle_count; i++) {
        const float *v = buffers.triangles.data + i * debug::TRIANGLE_STRIDE;
        triangle(list, at(v), at(v + debug::VERTEX_STRIDE), at(v + 2 * debug::VERTEX_STRIDE), color(v));
    }
}

static GpuMesh write_transient(TransientBuffer &transient, const collections::Array<ColorVertex> &vertices) {
    if (vertices.length == 0) return {};

    const uint32_t size = vertices.length * (uint32_t)sizeof(ColorVertex);
    if (size > transient.capacity) {
        if (transient.buffer.id != SG_INVALID_ID) sg_destroy_buffer(transient.buffer);
        uint32_t capacity = transient.capacity > 0 ? transient.capacity : 64 * 1024;
        while (capacity < size) capacity *= 2;
        sg_buffer_desc desc = {};
        desc.size = capacity;
        desc.usage.vertex_buffer = true;
        desc.usage.immutable = false;
        desc.usage.write_transient = true;
        desc.label = "samples frame vertices";
        transient.buffer = sg_make_buffer(&desc);
        transient.capacity = capacity;
    }

    sg_write_buffer_desc write = {};
    write.src.data = {vertices.data, size};
    write.dst.buffer = transient.buffer;
    sg_write_buffer_transient(&write);
    return {transient.buffer, (int)vertices.length};
}

static void draw(sg_pipeline pipeline, const GpuMesh &batch, const ColorUniforms &uniforms) {
    if (batch.vertex_count == 0) return;
    sg_apply_pipeline(pipeline);
    sg_bindings bindings = {};
    bindings.vertex_buffers[0] = batch.buffer;
    sg_apply_bindings(&bindings);
    const sg_range range = {&uniforms, sizeof(uniforms)};
    sg_apply_uniforms(0, &range);
    sg_draw(0, batch.vertex_count, 1);
}

static void draw_meshes(Renderer &renderer, const Camera &camera, bool transparent) {
    const Float3 light = normalize({0.4f, 1.0f, 0.3f});
    bool applied = false;
    for (uint32_t i = 0; i < renderer.meshes.length; i++) {
        const MeshDraw &draw = renderer.meshes[i];
        if ((draw.color.a < 1.0f) != transparent) continue;
        if (!applied) {
            sg_apply_pipeline(transparent ? renderer.transparent_mesh_pipeline : renderer.mesh_pipeline);
            applied = true;
        }
        MeshUniforms uniforms = {};
        for (int k = 0; k < 16; k++) {
            uniforms.mvp[k] = camera.view_projection.m[k];
            uniforms.model[k] = draw.model.m[k];
        }
        uniforms.tint[0] = draw.color.r;
        uniforms.tint[1] = draw.color.g;
        uniforms.tint[2] = draw.color.b;
        uniforms.tint[3] = draw.color.a;
        uniforms.light[0] = light.x;
        uniforms.light[1] = light.y;
        uniforms.light[2] = light.z;
        sg_bindings bindings = {};
        bindings.vertex_buffers[0] = draw.mesh.buffer;
        sg_apply_bindings(&bindings);
        const sg_range range = {&uniforms, sizeof(uniforms)};
        sg_apply_uniforms(0, &range);
        sg_draw(0, draw.mesh.vertex_count, 1);
    }
}

void render(Renderer &renderer, const Camera &camera) {
    ColorUniforms uniforms = {};
    for (int i = 0; i < 16; i++) uniforms.mvp[i] = camera.view_projection.m[i];

    const GpuMesh opaque = write_transient(renderer.transient[0], renderer.frame.opaque_triangles);
    const GpuMesh lines = write_transient(renderer.transient[1], renderer.frame.lines);
    const GpuMesh transparent = write_transient(renderer.transient[2], renderer.frame.transparent_triangles);

    draw_meshes(renderer, camera, false);
    draw(renderer.opaque_pipeline, opaque, uniforms);
    draw(renderer.line_pipeline, lines, uniforms);
    draw_meshes(renderer, camera, true);
    draw(renderer.transparent_pipeline, transparent, uniforms);
}

}
