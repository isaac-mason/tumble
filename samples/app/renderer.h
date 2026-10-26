#pragma once

#include <stdint.h>

#include "app/camera.h"
#include "app/matrix.h"
#include "sokol_gfx.h"
#include "tumble/math/types.h"
#include "tumble/physics/debug.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/util/collections.h"

struct ColorVertex {
    float x, y, z;
    uint32_t color;
};

struct Color {
    float r, g, b, a;
};

struct DrawList {
    tumble::collections::Array<ColorVertex> opaque_triangles;
    tumble::collections::Array<ColorVertex> transparent_triangles;
    tumble::collections::Array<ColorVertex> lines;
};

struct GpuMesh {
    sg_buffer buffer;
    int vertex_count;
};

struct MeshDraw {
    GpuMesh mesh;
    Mat4 model;
    Color color;
};

struct ShapeKey {
    const tumble::Shape *shape;
};

inline uint32_t hash_of(const ShapeKey &key) {
    const uint64_t bits = (uint64_t)(uintptr_t)key.shape;
    return (uint32_t)((bits >> 4) * 2654435761u) ^ (uint32_t)(bits >> 32);
}
inline bool operator==(const ShapeKey &a, const ShapeKey &b) { return a.shape == b.shape; }

struct TransientBuffer {
    sg_buffer buffer;
    uint32_t capacity;
};

struct Renderer {
    sg_shader color_shader;
    sg_shader mesh_shader;
    sg_pipeline opaque_pipeline;
    sg_pipeline transparent_pipeline;
    sg_pipeline line_pipeline;
    sg_pipeline mesh_pipeline;
    sg_pipeline transparent_mesh_pipeline;
    TransientBuffer transient[3];
    /** what this frame draws, cleared each frame */
    DrawList frame;
    tumble::collections::Array<MeshDraw> meshes;
    /** each shape's mesh, made the first time a body of it is drawn */
    tumble::collections::Map<ShapeKey, GpuMesh> shape_meshes;
};

namespace renderer {

Renderer create();
void destroy(Renderer &renderer);

/** the shape's mesh, made once and kept until forget_shapes */
GpuMesh mesh_of(Renderer &renderer, const tumble::Shape *shape);
/** drops every shape's mesh: after the shapes are freed, before a freed shape's memory is reused */
void forget_shapes(Renderer &renderer);
void mesh(Renderer &renderer, GpuMesh mesh, const Mat4 &model, Color color);

void clear(Renderer &renderer);
void line(DrawList &list, tumble::Float3 a, tumble::Float3 b, Color color);
void triangle(DrawList &list, tumble::Float3 a, tumble::Float3 b, tumble::Float3 c, Color color);
void point(DrawList &list, tumble::Float3 position, float radius, Color color);
void arrow(DrawList &list, tumble::Float3 from, tumble::Float3 to, Color color);
/** the library's debug lines and triangles */
void debug(DrawList &list, const tumble::DebugDraw &buffers);

void render(Renderer &renderer, const Camera &camera);

}
