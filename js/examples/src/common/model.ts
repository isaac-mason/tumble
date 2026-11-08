import * as g from 'gpucat';
import { mat4, type Quat, type Vec3, vec3 } from 'math';
import type { Renderer } from './renderer';

// a glb scene's triangles in world space, with each vertex's material colour, as the c++ samples' model::load gives
// them. triangles only, from plain accessors: what the samples' models use

export type Model = {
    /** xyz per vertex */
    positions: number[];
    /** rgb per vertex, in [0, 1] */
    colors: number[];
    /** three per triangle */
    indices: number[];
};

type Accessor = { bufferView: number; byteOffset?: number; componentType: number; count: number; type: string };
type BufferView = { byteOffset?: number; byteLength: number; byteStride?: number };
type Primitive = { attributes: { POSITION?: number }; indices?: number; material?: number; mode?: number };
type Material = {
    pbrMetallicRoughness?: { baseColorFactor?: number[] };
    extensions?: { KHR_materials_pbrSpecularGlossiness?: { diffuseFactor?: number[] } };
};
type Node = { mesh?: number; children?: number[]; matrix?: number[]; translation?: Vec3; rotation?: Quat; scale?: Vec3 };
type Gltf = {
    accessors: Accessor[];
    bufferViews: BufferView[];
    meshes: { primitives: Primitive[] }[];
    materials?: Material[];
    nodes: Node[];
    scenes: { nodes: number[] }[];
    scene?: number;
};

const COMPONENTS: Record<string, number> = { SCALAR: 1, VEC2: 2, VEC3: 3, VEC4: 4 };
const BYTES: Record<number, number> = { 5121: 1, 5123: 2, 5125: 4, 5126: 4 };

/** an accessor's values, each `components` long, in order */
function read(gltf: Gltf, bin: DataView, index: number): { values: number[]; components: number } {
    const accessor = gltf.accessors[index]!;
    const view = gltf.bufferViews[accessor.bufferView]!;
    const components = COMPONENTS[accessor.type]!;
    const size = BYTES[accessor.componentType]!;
    const stride = view.byteStride ?? components * size;
    const start = (view.byteOffset ?? 0) + (accessor.byteOffset ?? 0);
    const values: number[] = [];
    for (let i = 0; i < accessor.count; i++) {
        for (let c = 0; c < components; c++) {
            const at = start + i * stride + c * size;
            switch (accessor.componentType) {
                case 5126:
                    values.push(bin.getFloat32(at, true));
                    break;
                case 5125:
                    values.push(bin.getUint32(at, true));
                    break;
                case 5123:
                    values.push(bin.getUint16(at, true));
                    break;
                default:
                    values.push(bin.getUint8(at));
            }
        }
    }
    return { values, components };
}

function colorOf(gltf: Gltf, material: number | undefined): Vec3 {
    const found = material === undefined ? undefined : gltf.materials?.[material];
    const factor =
        found?.pbrMetallicRoughness?.baseColorFactor ?? found?.extensions?.KHR_materials_pbrSpecularGlossiness?.diffuseFactor;
    return factor === undefined ? [0.8, 0.8, 0.8] : [factor[0]!, factor[1]!, factor[2]!];
}

function appendNode(model: Model, gltf: Gltf, bin: DataView, index: number, parent: mat4.Mat4): void {
    const node = gltf.nodes[index]!;
    const local = node.matrix
        ? (node.matrix as mat4.Mat4)
        : mat4.fromRotationTranslationScale(
              mat4.create(),
              node.rotation ?? [0, 0, 0, 1],
              node.translation ?? [0, 0, 0],
              node.scale ?? [1, 1, 1],
          );
    const world = mat4.multiply(mat4.create(), parent, local);
    if (node.mesh !== undefined) {
        for (const primitive of gltf.meshes[node.mesh]!.primitives) {
            if ((primitive.mode ?? 4) !== 4 || primitive.attributes.POSITION === undefined) continue;
            const { values: positions } = read(gltf, bin, primitive.attributes.POSITION);
            const color = colorOf(gltf, primitive.material);
            const base = model.positions.length / 3;
            const count = positions.length / 3;
            for (let i = 0; i < count; i++) {
                const x = positions[i * 3]!;
                const y = positions[i * 3 + 1]!;
                const z = positions[i * 3 + 2]!;
                model.positions.push(
                    world[0]! * x + world[4]! * y + world[8]! * z + world[12]!,
                    world[1]! * x + world[5]! * y + world[9]! * z + world[13]!,
                    world[2]! * x + world[6]! * y + world[10]! * z + world[14]!,
                );
                model.colors.push(color[0], color[1], color[2]);
            }
            if (primitive.indices === undefined) for (let i = 0; i < count; i++) model.indices.push(base + i);
            else for (const index of read(gltf, bin, primitive.indices).values) model.indices.push(base + index);
        }
    }
    for (const child of node.children ?? []) appendNode(model, gltf, bin, child, world);
}

/**
 * the model drawn where it is, in its own colours and lit from above, flat a triangle: for a static body made of it,
 * drawn so in place of the body's shape
 */
export function addModelToScene(renderer: Renderer, model: Model): g.Mesh {
    const count = model.indices.length;
    const positions = new Float32Array(count * 3);
    const normals = new Float32Array(count * 3);
    const colors = new Float32Array(count * 3);
    const a: Vec3 = [0, 0, 0];
    const b: Vec3 = [0, 0, 0];
    const c: Vec3 = [0, 0, 0];
    const normal: Vec3 = [0, 0, 0];
    for (let t = 0; t < count; t += 3) {
        const corners = [a, b, c];
        for (let k = 0; k < 3; k++) {
            const index = model.indices[t + k]!;
            vec3.set(corners[k]!, model.positions[index * 3]!, model.positions[index * 3 + 1]!, model.positions[index * 3 + 2]!);
            colors.set([model.colors[index * 3]!, model.colors[index * 3 + 1]!, model.colors[index * 3 + 2]!], (t + k) * 3);
            positions.set(corners[k]!, (t + k) * 3);
        }
        vec3.normalize(normal, vec3.cross(normal, vec3.subtract([0, 0, 0], b, a), vec3.subtract([0, 0, 0], c, a)));
        for (let k = 0; k < 3; k++) normals.set(normal, (t + k) * 3);
    }
    const geometry = new g.Geometry();
    geometry.setBuffer('position', g.createVertexBuffer(g.d.vec3f, positions));
    geometry.setBuffer('normal', g.createVertexBuffer(g.d.vec3f, normals));
    geometry.setBuffer('color', g.createVertexBuffer(g.d.vec3f, colors));
    const order = new Uint32Array(count);
    for (let i = 0; i < count; i++) order[i] = i;
    geometry.setIndex(g.createIndexBuffer(order));

    const clip = g.mul(g.cameraProjectionMatrix, g.mul(g.cameraViewMatrix, g.vec4(g.attribute('position', g.d.vec3f), g.f32(1))));
    const lit = g.varying(g.attribute('normal', g.d.vec3f), 'v_normal');
    const tint = g.varying(g.attribute('color', g.d.vec3f), 'v_tint');
    const light = g.vec3(g.f32(0.4), g.f32(1), g.f32(0.3)).normalize();
    const shade = g.f32(0.45).add(g.f32(0.55).mul(lit.normalize().dot(light).max(g.f32(0))));
    const mesh = new g.Mesh(geometry, new g.Material({ vertex: clip, fragment: g.vec4(tint.mul(shade), g.f32(1)) }));
    mesh.frustumCulled = false;
    renderer.scene.add(mesh);
    return mesh;
}

export async function loadModel(url: string): Promise<Model> {
    const data = await (await fetch(url)).arrayBuffer();
    const view = new DataView(data);
    // a header, then chunks of a length and a type: the json, then the binary buffer
    let offset = 12;
    let gltf: Gltf | null = null;
    let bin: DataView | null = null;
    while (offset < data.byteLength) {
        const length = view.getUint32(offset, true);
        const type = view.getUint32(offset + 4, true);
        const body = new Uint8Array(data, offset + 8, length);
        if (type === 0x4e4f534a) gltf = JSON.parse(new TextDecoder().decode(body)) as Gltf;
        if (type === 0x004e4942) bin = new DataView(data, offset + 8, length);
        offset += 8 + length;
    }
    if (gltf === null || bin === null) throw new Error(`${url}: not a glb with a json and a binary chunk`);
    const model: Model = { positions: [], colors: [], indices: [] };
    const scene = gltf.scenes[gltf.scene ?? 0]!;
    for (const node of scene.nodes) appendNode(model, gltf, bin, node, mat4.create());
    return model;
}
