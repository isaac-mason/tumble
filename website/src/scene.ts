import * as g from 'gpucat';
import type { Mat4 } from 'math';
import { DEBUG_VERTEX_BYTES, DEBUG_VERTEX_COLOR_OFFSET } from 'tumble.js';
import type { Primitive } from './demo';
import { rgb, SKY_HAZE, SKY_HORIZON, SKY_ZENITH } from './theme';

// the landing page's drawing: every look is a box, cylinder or sphere, so each is drawn as one instanced mesh with a
// matrix and a colour per instance, lit from above and fading into the sky. a sky around it all, and the library's
// debug lines

/** a sun behind the camera and to the side, so the runner and what faces it are lit */
const SUN_DIRECTION: [number, number, number] = [0.35, 0.8, 0.6];
/** light from the sky on what faces up, bounced warm off the ground onto what faces down, and the sun on top */
const SKY_LIGHT = rgb('#d4d4d9');
const GROUND_LIGHT = rgb('#a9a9ae');
const SUN_LIGHT = rgb('#ffffff');
const SUN_STRENGTH = 0.32;
/** metres from the camera where the fog starts, and where it hides everything in the sky's colour */
const FOG_NEAR = 45;
const FOG_FAR = 170;
/** inside the camera's far plane, past everything else */
const SKY_RADIUS = 450;
/** metres from the camera where things start thinning out, dithered, and where they are gone, so what is right in front of it never hides the runner */
const NEAR_FADE_START = 3.5;
const NEAR_FADE_END = 1.2;
/** the most instances of one primitive drawn at once */
const CAPACITY = 32768;

/** a convex shape about the origin as flat shaded triangles, each wound to face outward */
function flatGeometry(positions: number[], indices: number[]): g.Geometry {
    const flat = new Float32Array(indices.length * 3);
    const normals = new Float32Array(indices.length * 3);
    for (let t = 0; t < indices.length; t += 3) {
        let a = indices[t]! * 3;
        let b = indices[t + 1]! * 3;
        const c = indices[t + 2]! * 3;
        const abx = positions[b]! - positions[a]!;
        const aby = positions[b + 1]! - positions[a + 1]!;
        const abz = positions[b + 2]! - positions[a + 2]!;
        const acx = positions[c]! - positions[a]!;
        const acy = positions[c + 1]! - positions[a + 1]!;
        const acz = positions[c + 2]! - positions[a + 2]!;
        let nx = aby * acz - abz * acy;
        let ny = abz * acx - abx * acz;
        let nz = abx * acy - aby * acx;
        // facing away from the middle, else turned around
        const middleX = positions[a]! + positions[b]! + positions[c]!;
        const middleY = positions[a + 1]! + positions[b + 1]! + positions[c + 1]!;
        const middleZ = positions[a + 2]! + positions[b + 2]! + positions[c + 2]!;
        if (nx * middleX + ny * middleY + nz * middleZ < 0) {
            [a, b] = [b, a];
            nx = -nx;
            ny = -ny;
            nz = -nz;
        }
        const length = Math.hypot(nx, ny, nz) || 1;
        const corners = [a, b, c];
        for (let k = 0; k < 3; k++) {
            const from = corners[k]!;
            const to = (t + k) * 3;
            flat[to] = positions[from]!;
            flat[to + 1] = positions[from + 1]!;
            flat[to + 2] = positions[from + 2]!;
            normals[to] = nx / length;
            normals[to + 1] = ny / length;
            normals[to + 2] = nz / length;
        }
    }
    const geometry = new g.Geometry();
    geometry.setBuffer('position', g.createVertexBuffer(g.d.vec3f, flat));
    geometry.setBuffer('normal', g.createVertexBuffer(g.d.vec3f, normals));
    const order = new Uint32Array(indices.length);
    for (let i = 0; i < order.length; i++) order[i] = i;
    geometry.setIndex(g.createIndexBuffer(order));
    return geometry;
}

/** from -1 to 1 on every axis */
function unitBox(): g.Geometry {
    const positions: number[] = [];
    for (let i = 0; i < 8; i++) positions.push(i & 1 ? 1 : -1, i & 2 ? 1 : -1, i & 4 ? 1 : -1);
    const faces = [
        [0, 2, 6, 4],
        [1, 3, 7, 5],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [0, 1, 3, 2],
        [4, 5, 7, 6],
    ];
    return flatGeometry(
        positions,
        faces.flatMap(([a, b, c, d]) => [a!, b!, c!, a!, c!, d!]),
    );
}

/** radius 1 about y, from y -1 to 1 */
function unitCylinder(segments: number): g.Geometry {
    const positions: number[] = [0, -1, 0, 0, 1, 0];
    for (let i = 0; i < segments; i++) {
        const angle = (i / segments) * Math.PI * 2;
        positions.push(Math.cos(angle), -1, Math.sin(angle), Math.cos(angle), 1, Math.sin(angle));
    }
    const indices: number[] = [];
    for (let i = 0; i < segments; i++) {
        const bottom = 2 + i * 2;
        const next = 2 + ((i + 1) % segments) * 2;
        indices.push(0, bottom, next, 1, bottom + 1, next + 1, bottom, next, next + 1, bottom, next + 1, bottom + 1);
    }
    return flatGeometry(positions, indices);
}

/** radius 1: an icosahedron, each face split in four and pushed out onto the sphere */
function unitSphere(): g.Geometry {
    const t = (1 + Math.sqrt(5)) / 2;
    const corners = [
        [-1, t, 0],
        [1, t, 0],
        [-1, -t, 0],
        [1, -t, 0],
        [0, -1, t],
        [0, 1, t],
        [0, -1, -t],
        [0, 1, -t],
        [t, 0, -1],
        [t, 0, 1],
        [-t, 0, -1],
        [-t, 0, 1],
    ];
    const faces = [
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    const positions: number[] = [];
    const push = (x: number, y: number, z: number) => {
        const length = Math.hypot(x, y, z);
        positions.push(x / length, y / length, z / length);
        return positions.length / 3 - 1;
    };
    const indices: number[] = [];
    for (const [a, b, c] of faces) {
        const [ax, ay, az] = corners[a!]!;
        const [bx, by, bz] = corners[b!]!;
        const [cx, cy, cz] = corners[c!]!;
        const pa = push(ax!, ay!, az!);
        const pb = push(bx!, by!, bz!);
        const pc = push(cx!, cy!, cz!);
        const ab = push(ax! + bx!, ay! + by!, az! + bz!);
        const bc = push(bx! + cx!, by! + cy!, bz! + cz!);
        const ca = push(cx! + ax!, cy! + ay!, cz! + az!);
        indices.push(pa, ab, ca, pb, bc, ab, pc, ca, bc, ab, bc, ca);
    }
    return flatGeometry(positions, indices);
}

/** the colour, or nothing where the fade near the camera thins out what fades, in a pattern fixed to the screen */
const nearFade = g.Fn(
    (color, screen, distance, fades) => {
        const threshold = g.fract(g.f32(52.9829189).mul(g.fract(g.dot(screen, g.vec2(g.f32(0.06711056), g.f32(0.00583715))))));
        const opacity = g.mix(g.f32(1), g.smoothstep(g.f32(NEAR_FADE_END), g.f32(NEAR_FADE_START), distance), fades);
        g.If(opacity.lessThan(threshold), () => {
            g.Discard();
        });
        return color;
    },
    {
        name: 'nearFade',
        params: [
            { name: 'color', type: g.d.vec4f },
            { name: 'screen', type: g.d.vec2f },
            { name: 'distance', type: g.d.f32 },
            { name: 'fades', type: g.d.f32 },
        ],
    },
);

/** the sky's colour looking along a unit direction: deep blue overhead, pale toward the horizon, and warm along it */
function skyColor(direction: g.Node<g.d.vec3f>): g.Node<g.d.vec3f> {
    const up = direction.y;
    const sky = g.mix(g.vec3(...SKY_HORIZON), g.vec3(...SKY_ZENITH), g.smoothstep(g.f32(0), g.f32(0.55), up));
    const haze = g.f32(1).sub(g.smoothstep(g.f32(0), g.f32(0.12), g.abs(up)));
    return g.mix(sky, g.vec3(...SKY_HAZE), haze.mul(g.f32(0.6)));
}

/** a sphere about the camera, drawn from inside, its colour from the direction to each point */
export function createSky(scene: g.Scene): g.Mesh {
    const world = g.mul(g.modelWorldMatrix, g.vec4(g.attribute('position', g.d.vec3f).mul(g.f32(SKY_RADIUS)), g.f32(1)));
    const clip = g.mul(g.cameraProjectionMatrix, g.mul(g.cameraViewMatrix, world));
    const at = g.varying(world.xyz, 'v_world');
    const material = new g.Material({
        vertex: clip,
        fragment: g.vec4(skyColor(g.normalize(at.sub(g.cameraPosition))), g.f32(1)),
        cullMode: 'none',
        depthWrite: false,
    });
    const sky = new g.Mesh(unitSphere(), material);
    sky.frustumCulled = false;
    scene.add(sky);
    return sky;
}

/** one primitive's instances: a matrix each, and a colour with whether it fades near the camera, filled afresh every frame */
type Batch = { mesh: g.Mesh; matrices: g.GpuBuffer<g.d.vec4f>; colors: g.GpuBuffer<g.d.vec4f>; count: number };

/** the instances' material: each vertex placed by its instance's matrix, lit by the sky, the ground and the sun, and fogged */
function instancedMaterial(matrices: g.GpuBuffer<g.d.vec4f>, colors: g.GpuBuffer<g.d.vec4f>): g.Material {
    const column = (offset: number) => g.attribute(matrices, { stride: 64, offset, instanced: true });
    const instance = g.mat4(column(0), column(16), column(32), column(48));
    const world = g.mul(instance, g.vec4(g.attribute('position', g.d.vec3f), g.f32(1)));
    const clip = g.mul(g.cameraProjectionMatrix, g.mul(g.cameraViewMatrix, world));
    // the matrix scales each axis on its own, which leaves an axis' normal along the same turned axis
    const normal = g.varying(g.normalize(g.mul(instance, g.vec4(g.attribute('normal', g.d.vec3f), g.f32(0))).xyz), 'v_normal');
    const at = g.varying(world.xyz, 'v_world');
    const tint = g.varying(g.attribute(colors, { instanced: true }), 'v_color');
    const color = tint.xyz;
    const sun = g.vec3(...SUN_DIRECTION).normalize();
    const unit = normal.normalize();
    const facing = g.clamp(unit.dot(sun), g.f32(0), g.f32(1));
    const ambient = g.mix(g.vec3(...GROUND_LIGHT), g.vec3(...SKY_LIGHT), unit.y.mul(g.f32(0.5)).add(g.f32(0.5)));
    const shaded = color.mul(ambient.add(g.vec3(...SUN_LIGHT).mul(facing.mul(g.f32(SUN_STRENGTH)))));
    const distance = g.length(at.sub(g.cameraPosition));
    const fog = g.smoothstep(g.f32(FOG_NEAR), g.f32(FOG_FAR), distance);
    const fragment = g.mix(shaded, skyColor(g.normalize(at.sub(g.cameraPosition))), fog);
    return new g.Material({ vertex: clip, fragment: nearFade(g.vec4(fragment, g.f32(1)), g.fragCoord.xy, distance, tint.w) });
}

function createBatch(scene: g.Scene, geometry: g.Geometry): Batch {
    const matrices = g.createVertexBuffer(g.d.vec4f, new Float32Array(CAPACITY * 16), 'instance matrices');
    const colors = g.createVertexBuffer(g.d.vec4f, new Float32Array(CAPACITY * 4), 'instance colors');
    const mesh = new g.Mesh(geometry, instancedMaterial(matrices, colors));
    mesh.frustumCulled = false;
    mesh.count = 0;
    scene.add(mesh);
    return { mesh, matrices, colors, count: 0 };
}

export type Instances = Record<Primitive, Batch>;

export function createInstances(scene: g.Scene): Instances {
    return {
        box: createBatch(scene, unitBox()),
        cylinder: createBatch(scene, unitCylinder(10)),
        sphere: createBatch(scene, unitSphere()),
    };
}

export function beginInstances(instances: Instances): void {
    for (const batch of Object.values(instances)) batch.count = 0;
}

/** one more instance of the primitive this frame, if there is room for it */
export function addInstance(
    instances: Instances,
    primitive: Primitive,
    matrix: Mat4,
    color: [number, number, number],
    fades: boolean,
): void {
    const batch = instances[primitive];
    if (batch.count >= CAPACITY) return;
    batch.matrices.array!.set(matrix, batch.count * 16);
    const colors = batch.colors.array!;
    colors.set(color, batch.count * 4);
    colors[batch.count * 4 + 3] = fades ? 1 : 0;
    batch.count++;
}

/** the instances added since the last begin, drawn */
export function endInstances(instances: Instances): void {
    for (const batch of Object.values(instances)) {
        batch.mesh.count = batch.count;
        if (batch.count === 0) continue;
        batch.matrices.addUpdateRange(0, batch.count * 16);
        batch.colors.addUpdateRange(0, batch.count * 4);
    }
}

/** a frame's lines of one colour */
type DebugBucket = { points: Float32Array; count: number; geometry: g.LineSegmentsGeometry; lines: g.LineSegments };

/** the library's debug lines, shown in the scene as line segments, one set per colour */
export type DebugLines = { scene: g.Scene; buckets: Map<number, DebugBucket> };

export function createDebugLines(scene: g.Scene): DebugLines {
    return { scene, buckets: new Map() };
}

function bucketOf(debugLines: DebugLines, r: number, green: number, b: number): DebugBucket {
    const key = (r << 16) | (green << 8) | b;
    let bucket = debugLines.buckets.get(key);
    if (bucket === undefined) {
        // it grows as an update needs
        const geometry = new g.LineSegmentsGeometry(new Float32Array(6), 2048);
        const lines = new g.LineSegments(
            geometry,
            new g.LineMaterial({ color: g.vec4f(r / 255, green / 255, b / 255, 1) as never, lineWidth: 1.5 }),
        );
        debugLines.scene.add(lines);
        bucket = { points: new Float32Array(6 * 1024), count: 0, geometry, lines };
        debugLines.buckets.set(key, bucket);
    }
    return bucket;
}

/** lines as debugDraw.read gives them, in place of the last: each vertex x y z and then the colour's four bytes */
export function showDebugLines(debugLines: DebugLines, lines: Float32Array, lineCount: number): void {
    for (const bucket of debugLines.buckets.values()) bucket.count = 0;
    const colors = new Uint8Array(lines.buffer, lines.byteOffset, lines.byteLength);
    const floats = DEBUG_VERTEX_BYTES / 4;
    for (let i = 0; i < lineCount; i++) {
        const a = i * 2 * floats;
        const b = a + floats;
        const color = a * 4 + DEBUG_VERTEX_COLOR_OFFSET;
        const bucket = bucketOf(debugLines, colors[color]!, colors[color + 1]!, colors[color + 2]!);
        if ((bucket.count + 2) * 3 > bucket.points.length) {
            const grown = new Float32Array(bucket.points.length * 2);
            grown.set(bucket.points);
            bucket.points = grown;
        }
        const points = bucket.points;
        const to = bucket.count * 3;
        points[to] = lines[a]!;
        points[to + 1] = lines[a + 1]!;
        points[to + 2] = lines[a + 2]!;
        points[to + 3] = lines[b]!;
        points[to + 4] = lines[b + 1]!;
        points[to + 5] = lines[b + 2]!;
        bucket.count += 2;
    }
    for (const bucket of debugLines.buckets.values()) {
        bucket.lines.visible = bucket.count > 0;
        if (bucket.count > 0) bucket.geometry.update(bucket.points.subarray(0, bucket.count * 3));
    }
}
