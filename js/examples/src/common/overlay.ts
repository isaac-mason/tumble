import type { Panel } from 'dashcat';
import * as g from 'gpucat';
import type { Vec3 } from 'math';
import {
    type Color,
    DEBUG_VERTEX_BYTES,
    DEBUG_VERTEX_COLOR_OFFSET,
    type DebugDraw,
    debug,
    debugDraw,
    type World,
} from 'tumble.js';
import type { Renderer } from './renderer';

// the library's debug drawing, and an example's own lines, shown as line segments: one set per colour, refilled each
// frame. begin draws what the options ask of the world; an example adds its own lines with the helpers here, hidden by
// the bodies in front of them or drawn over everything; end shows them

export type OverlayOptions = {
    wireframes: boolean;
    velocities: boolean;
    contacts: boolean;
    joints: boolean;
    colorMode: debug.BodyColorMode;
};

type Bucket = { points: Float32Array; count: number; geometry: g.LineSegmentsGeometry; lines: g.LineSegments };

type Layer = { draw: DebugDraw; buckets: Map<number, Bucket>; depthTest: boolean };

export type Overlay = {
    renderer: Renderer;
    world: World;
    options: OverlayOptions;
    /** behind what is in front of them */
    inScene: Layer;
    /** over everything */
    onTop: Layer;
};

function createLayer(depthTest: boolean): Layer {
    return { draw: debugDraw.create(), buckets: new Map(), depthTest };
}

export function createOverlay(renderer: Renderer, world: World, options: Partial<OverlayOptions> = {}): Overlay {
    return {
        renderer,
        world,
        options: {
            wireframes: false,
            velocities: false,
            contacts: false,
            joints: false,
            colorMode: debug.BodyColorMode.MOTION_TYPE,
            ...options,
        },
        inScene: createLayer(true),
        onTop: createLayer(false),
    };
}

/** a folder of the options in the panel */
export function addOverlayControls(overlay: Overlay, panel: Panel): void {
    const folder = panel.folder('debug drawing');
    folder.add(overlay.options, 'wireframes');
    folder.add(overlay.options, 'colorMode', {
        label: 'colour by',
        options: {
            'motion type': debug.BodyColorMode.MOTION_TYPE,
            sleeping: debug.BodyColorMode.SLEEPING,
            island: debug.BodyColorMode.ISLAND,
        },
    });
    folder.add(overlay.options, 'velocities');
    folder.add(overlay.options, 'contacts');
    folder.add(overlay.options, 'joints');
}

/** empty the lines, then draw the world as the options ask */
export function beginOverlay(overlay: Overlay): void {
    const { inScene, onTop, world, options } = overlay;
    debugDraw.clear(inScene.draw);
    debugDraw.clear(onTop.draw);
    if (options.wireframes || options.velocities) {
        debug.bodies(inScene.draw, world, {
            colorMode: options.colorMode,
            showLinearVelocity: options.velocities,
            showAngularVelocity: options.velocities,
        });
    }
    if (options.contacts) debug.contactConstraints(onTop.draw, world);
    if (options.joints) debug.joints(onTop.draw, world);
}

/** where an example's lines go */
export function overlayDraw(overlay: Overlay, onTop: boolean): DebugDraw {
    return onTop ? overlay.onTop.draw : overlay.inScene.draw;
}

export function drawLine(overlay: Overlay, a: Vec3, b: Vec3, color: Color, onTop = false): void {
    debug.line(overlayDraw(overlay, onTop), a, b, color);
}

const _a: Vec3 = [0, 0, 0];
const _b: Vec3 = [0, 0, 0];

/** three short lines crossing at the point */
export function drawPoint(overlay: Overlay, point: Vec3, color: Color, size = 0.1, onTop = false): void {
    const draw = overlayDraw(overlay, onTop);
    for (let axis = 0; axis < 3; axis++) {
        _a[0] = point[0] - (axis === 0 ? size : 0);
        _a[1] = point[1] - (axis === 1 ? size : 0);
        _a[2] = point[2] - (axis === 2 ? size : 0);
        _b[0] = point[0] + (axis === 0 ? size : 0);
        _b[1] = point[1] + (axis === 1 ? size : 0);
        _b[2] = point[2] + (axis === 2 ? size : 0);
        debug.line(draw, _a, _b, color);
    }
}

const _tip: Vec3 = [0, 0, 0];

/** a line from a point along a direction, scaled by length, with a point at its tip */
export function drawArrow(overlay: Overlay, from: Vec3, direction: Vec3, length: number, color: Color, onTop = false): void {
    _tip[0] = from[0] + direction[0] * length;
    _tip[1] = from[1] + direction[1] * length;
    _tip[2] = from[2] + direction[2] * length;
    debug.line(overlayDraw(overlay, onTop), from, _tip, color);
    drawPoint(overlay, _tip, color, 0.05, onTop);
}

function bucketOf(overlay: Overlay, layer: Layer, r: number, green: number, b: number): Bucket {
    const key = (r << 16) | (green << 8) | b;
    let bucket = layer.buckets.get(key);
    if (bucket === undefined) {
        // the geometry grows as an update needs
        const geometry = new g.LineSegmentsGeometry(new Float32Array(6), 2048);
        const material = new g.LineMaterial({ color: g.vec4f(r / 255, green / 255, b / 255, 1) as never, lineWidth: 1.5 });
        material.depthTest = layer.depthTest;
        material.depthWrite = layer.depthTest;
        const lines = new g.LineSegments(geometry, material);
        lines.frustumCulled = false;
        // drawn after the scene's solids
        lines.renderOrder = layer.depthTest ? 1 : 2;
        overlay.renderer.scene.add(lines);
        bucket = { points: new Float32Array(6 * 1024), count: 0, geometry, lines };
        layer.buckets.set(key, bucket);
    }
    return bucket;
}

function showLayer(overlay: Overlay, layer: Layer): void {
    for (const bucket of layer.buckets.values()) bucket.count = 0;
    // views onto the wasm's memory, read now before anything draws again
    const { lines, lineCount } = debugDraw.read(layer.draw);
    const colors = new Uint8Array(lines.buffer, lines.byteOffset, lines.byteLength);
    const floats = DEBUG_VERTEX_BYTES / 4;
    for (let i = 0; i < lineCount; i++) {
        const a = i * 2 * floats;
        const b = a + floats;
        const color = a * 4 + DEBUG_VERTEX_COLOR_OFFSET;
        const bucket = bucketOf(overlay, layer, colors[color]!, colors[color + 1]!, colors[color + 2]!);
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
    for (const bucket of layer.buckets.values()) {
        bucket.lines.visible = bucket.count > 0;
        if (bucket.count > 0) bucket.geometry.update(bucket.points.subarray(0, bucket.count * 3));
    }
}

/** the lines drawn since begin, into the scene in place of the last frame's */
export function endOverlay(overlay: Overlay): void {
    showLayer(overlay, overlay.inScene);
    showLayer(overlay, overlay.onTop);
}
