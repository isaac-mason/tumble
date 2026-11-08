import * as g from 'gpucat';
import { DEBUG_VERTEX_BYTES, DEBUG_VERTEX_COLOR_OFFSET } from 'tumble.js';
import type { ShapeMesh } from './demo';
import { rgb } from './theme';

// the landing page's drawing: a mesh for each body from its shape, lit from above, and the library's debug lines

const materials = new Map<string, g.Material>();

/** lit by a light from above, one material per colour */
function lit(color: string): g.Material {
    const known = materials.get(color);
    if (known) return known;
    const world = g.mul(g.modelWorldMatrix, g.vec4(g.attribute('position', g.d.vec3f), g.f32(1)));
    const clip = g.mul(g.cameraProjectionMatrix, g.mul(g.cameraViewMatrix, world));
    const normal = g.varying(g.normalize(g.mul(g.modelNormalMatrix, g.attribute('normal', g.d.vec3f))), 'v_normal');
    const light = g.vec3(g.f32(0.4), g.f32(1), g.f32(0.3)).normalize();
    const shade = g.f32(0.45).add(g.f32(0.55).mul(normal.dot(light).max(g.f32(0))));
    const material = new g.Material({ vertex: clip, fragment: g.vec4(g.vec3(...rgb(color)).mul(shade), g.f32(1)) });
    materials.set(color, material);
    return material;
}

export function meshOf(shape: ShapeMesh, color: string): g.Mesh {
    const [x, y, z] = shape.halfExtents;
    return new g.Mesh(g.createBoxGeometry(2 * x, 2 * y, 2 * z), lit(color));
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
