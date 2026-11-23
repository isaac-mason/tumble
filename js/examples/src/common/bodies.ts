import * as g from 'gpucat';
import { type Quat, quat, type Vec3, vec3 } from 'math';
import { type BodyId, MotionType, rigidBody, type Shape, type World } from 'tumble.js';
import type { Renderer } from './renderer';
import { buildShapeTriangles } from './shape-mesh';
import { palette, rgb, spectrum } from './theme';

// every body drawn solid from its shape: one instanced mesh per shape, each instance a body's pose and colour. a body's
// pose is read after each step, and drawn between the last two by how far the frame is past the last step

/** an instance's floats: position and 1, quaternion, colour and 1 */
const INSTANCE_FLOATS = 12;

const INSTANCES = g.d.array(g.d.vec4f);

type ShapeGroup = {
    shape: Shape;
    mesh: g.Mesh;
    geometry: g.Geometry;
    instances: Float32Array;
    storage: g.StorageNode<typeof INSTANCES>;
    capacity: number;
    /** bodies of the shape */
    members: number;
};

type DrawnBody = {
    body: BodyId;
    group: ShapeGroup;
    color: [number, number, number];
    visible: boolean;
    beforePosition: Vec3;
    beforeQuaternion: Quat;
    nowPosition: Vec3;
    nowQuaternion: Quat;
};

export type Bodies = {
    renderer: Renderer;
    world: World;
    groups: Map<Shape, ShapeGroup>;
    drawn: Map<BodyId, DrawnBody>;
    /** for the next body without a colour of its own */
    nextColor: number;
};

export function createBodies(renderer: Renderer, world: World): Bodies {
    return { renderer, world, groups: new Map(), drawn: new Map(), nextColor: 0 };
}

/** lit from above, the instance's colour, the shape's geometry turned and moved by the instance's pose */
function createMaterial(instances: g.StorageNode<typeof INSTANCES>): g.Material {
    const first = g.instanceIndex.mul(g.u32(3));
    const position = g.index(instances, first);
    const rotation = g.index(instances, first.add(g.u32(1)));
    const color = g.index(instances, first.add(g.u32(2)));
    // v + 2 q.xyz x (q.xyz x v + q.w v)
    const turn = (v: g.Node<g.d.vec3f>) => v.add(rotation.xyz.cross(rotation.xyz.cross(v).add(v.mul(rotation.w))).mul(g.f32(2)));
    const local = g.attribute('position', g.d.vec3f);
    const world = turn(local).add(position.xyz);
    const clip = g.mul(g.cameraProjectionMatrix, g.mul(g.cameraViewMatrix, g.vec4(world, g.f32(1))));
    const normal = g.varying(turn(g.attribute('normal', g.d.vec3f)), 'v_normal');
    const tint = g.varying(color.xyz, 'v_tint');
    const light = g.vec3(g.f32(0.4), g.f32(1), g.f32(0.3)).normalize();
    const shade = g.f32(0.45).add(g.f32(0.55).mul(normal.normalize().dot(light).max(g.f32(0))));
    return new g.Material({ vertex: clip, fragment: g.vec4(tint.mul(shade), g.f32(1)) });
}

function createGroup(bodies: Bodies, shape: Shape): ShapeGroup {
    const triangles = buildShapeTriangles(shape);
    const vertexCount = triangles.positions.length / 3;
    const geometry = new g.Geometry();
    geometry.setBuffer('position', g.createVertexBuffer(g.d.vec3f, new Float32Array(triangles.positions)));
    geometry.setBuffer('normal', g.createVertexBuffer(g.d.vec3f, new Float32Array(triangles.normals)));
    const indices = new Uint32Array(vertexCount);
    for (let i = 0; i < vertexCount; i++) indices[i] = i;
    geometry.setIndex(g.createIndexBuffer(indices));
    const capacity = 16;
    const instances = new Float32Array(capacity * INSTANCE_FLOATS);
    const storage = g.storage(g.createStorageBuffer(INSTANCES, instances));
    const mesh = new g.Mesh(geometry, createMaterial(storage));
    // the instances are anywhere, so the shape's own bounds cannot cull them
    mesh.frustumCulled = false;
    mesh.count = 0;
    bodies.renderer.scene.add(mesh);
    return { shape, mesh, geometry, instances, storage, capacity, members: 0 };
}

function join(bodies: Bodies, shape: Shape): ShapeGroup {
    let group = bodies.groups.get(shape);
    if (group === undefined) {
        group = createGroup(bodies, shape);
        bodies.groups.set(shape, group);
    }
    group.members++;
    if (group.members > group.capacity) {
        // a bigger buffer, which the material reads in place of the last
        group.capacity *= 2;
        group.instances = new Float32Array(group.capacity * INSTANCE_FLOATS);
        group.storage.value!.dispose();
        group.storage.value = g.createStorageBuffer(INSTANCES, group.instances);
    }
    return group;
}

/** the last body of a shape gone, its mesh goes too: the shape may be freed, and its address reused by another */
function leave(bodies: Bodies, group: ShapeGroup): void {
    if (--group.members > 0) return;
    bodies.renderer.scene.remove(group.mesh);
    group.geometry.dispose();
    group.storage.value!.dispose();
    bodies.groups.delete(group.shape);
}

function read(bodies: Bodies, drawn: DrawnBody): void {
    rigidBody.getTransform(drawn.nowPosition, drawn.nowQuaternion, bodies.world, drawn.body);
}

/** draw a body, in its own colour or the next of the spectrum; a static body in the ground's colour */
export function addBody(bodies: Bodies, body: BodyId, color?: string): void {
    const shape = rigidBody.getShape(bodies.world, body);
    if (shape === null) return;
    const isStatic = rigidBody.getMotionType(bodies.world, body) === MotionType.STATIC;
    const hex = color ?? (isStatic ? palette.dim : spectrum[bodies.nextColor++ % spectrum.length]!);
    const drawn: DrawnBody = {
        body,
        group: join(bodies, shape),
        color: rgb(hex),
        visible: true,
        beforePosition: [0, 0, 0],
        beforeQuaternion: [0, 0, 0, 1],
        nowPosition: [0, 0, 0],
        nowQuaternion: [0, 0, 0, 1],
    };
    read(bodies, drawn);
    vec3.copy(drawn.beforePosition, drawn.nowPosition);
    quat.copy(drawn.beforeQuaternion, drawn.nowQuaternion);
    bodies.drawn.set(body, drawn);
}

/** stop drawing a body, before it is removed from the world */
export function removeBody(bodies: Bodies, body: BodyId): void {
    const drawn = bodies.drawn.get(body);
    if (drawn === undefined) return;
    leave(bodies, drawn.group);
    bodies.drawn.delete(body);
}

export function setBodyColor(bodies: Bodies, body: BodyId, color: string): void {
    const drawn = bodies.drawn.get(body);
    if (drawn !== undefined) drawn.color = rgb(color);
}

export function setBodyVisible(bodies: Bodies, body: BodyId, visible: boolean): void {
    const drawn = bodies.drawn.get(body);
    if (drawn !== undefined) drawn.visible = visible;
}

/** after rigidBody.setShape: draw the body with its new shape */
export function refreshBodyShape(bodies: Bodies, body: BodyId): void {
    const drawn = bodies.drawn.get(body);
    if (drawn === undefined) return;
    const shape = rigidBody.getShape(bodies.world, body);
    if (shape !== null && shape !== drawn.group.shape) {
        const old = drawn.group;
        drawn.group = join(bodies, shape);
        leave(bodies, old);
    }
}

/** after moving a body somewhere new with setPosition or setTransform: drawn there, not swept from where it was */
export function snapBody(bodies: Bodies, body: BodyId): void {
    const drawn = bodies.drawn.get(body);
    if (drawn === undefined) return;
    read(bodies, drawn);
    vec3.copy(drawn.beforePosition, drawn.nowPosition);
    quat.copy(drawn.beforeQuaternion, drawn.nowQuaternion);
}

/** after each step: what was now is as the step began, and the step's result is now */
export function readBodies(bodies: Bodies): void {
    for (const drawn of bodies.drawn.values()) {
        vec3.copy(drawn.beforePosition, drawn.nowPosition);
        quat.copy(drawn.beforeQuaternion, drawn.nowQuaternion);
        read(bodies, drawn);
    }
}

const _position: Vec3 = [0, 0, 0];
const _quaternion: Quat = [0, 0, 0, 1];

/** each body between the last two steps, by alpha */
export function drawBodies(bodies: Bodies, alpha: number): void {
    for (const group of bodies.groups.values()) group.mesh.count = 0;
    for (const drawn of bodies.drawn.values()) {
        if (!drawn.visible) continue;
        vec3.lerp(_position, drawn.beforePosition, drawn.nowPosition, alpha);
        quat.slerp(_quaternion, drawn.beforeQuaternion, drawn.nowQuaternion, alpha);
        const group = drawn.group;
        const at = group.mesh.count * INSTANCE_FLOATS;
        const instances = group.instances;
        instances[at] = _position[0];
        instances[at + 1] = _position[1];
        instances[at + 2] = _position[2];
        instances[at + 3] = 1;
        instances[at + 4] = _quaternion[0];
        instances[at + 5] = _quaternion[1];
        instances[at + 6] = _quaternion[2];
        instances[at + 7] = _quaternion[3];
        instances[at + 8] = drawn.color[0];
        instances[at + 9] = drawn.color[1];
        instances[at + 10] = drawn.color[2];
        instances[at + 11] = 1;
        group.mesh.count++;
    }
    for (const group of bodies.groups.values()) {
        group.mesh.visible = group.mesh.count > 0;
        if (group.mesh.count > 0) group.storage.value!.needsUpdate = true;
    }
}
