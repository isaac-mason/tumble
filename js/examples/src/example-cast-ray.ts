import { type Quat, quat, type Vec3, vec3 } from 'math';
import { mulberry32 } from 'math/random';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    CastRayStatus,
    capsule,
    castRay,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    init,
    MotionType,
    releaseShape,
    rigidBody,
    type Shape,
    scaled,
    sphere,
    triangleMesh,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies, removeBody } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, drawLine, drawPoint, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { debugColor, palette, spectrum } from './common/theme';
import { torusKnot } from './common/torus-knot';

await init();
const renderer = await createRenderer([0, 0, 10], [0, 0, 0]);
// nothing falls: the targets are static, turned by hand
const worldSettings = createWorldSettings();
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
worldSettings.gravity = [0, 0, 0];
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const rng = mulberry32.create(13);
const random = () => mulberry32.sample(rng);

// rays from points on a sphere of this radius, towards its centre, as long as the radius
const RADIUS = 5;
const knot = torusKnot(1, 0.4, 40, 10);
const targetShapes: Record<string, Shape> = {
    'torus knot mesh': triangleMesh.create(knot),
    sphere: sphere.create({ radius: 1 }),
    box: box.create({ halfExtents: [1, 1, 1] }),
    capsule: capsule.create({ halfHeightOfCylinder: 1, radius: 0.5 }),
};

const settings = { rays: 150, raySpeed: 1, near: 0, far: RADIUS, target: 'torus knot mesh', targets: 1, targetSpeed: 1 };

// the targets, spread through a ball and turned with it, each spinning too. many are scaled down to fit
type Target = { body: BodyId; offset: Vec3; quaternion: Quat; spin: Vec3 };
let targets: Target[] = [];
/** the scaled shape made for the targets, let go of when they are placed again */
let scaledShape: Shape | null = null;
const turn: Quat = [0, 0, 0, 1];

function placeTargets(): void {
    for (const target of targets) {
        removeBody(bodies, target.body);
        rigidBody.remove(world, target.body);
    }
    if (scaledShape !== null) releaseShape(scaledShape);
    const t = (settings.targets - 1) / 299;
    const spread = 2 * t;
    const scale = 1 - 0.8 * t;
    const shape = targetShapes[settings.target]!;
    scaledShape = scale === 1 ? null : scaled.create({ shape, scale: [scale, scale, scale] });
    const targetShape = scaledShape ?? shape;
    targets = Array.from({ length: settings.targets }, () => {
        const offset = vec3.normalize([0, 0, 0], [random() - 0.5, random() - 0.5, random() - 0.5]);
        vec3.scale(offset, offset, spread * random());
        const body = rigidBody.create(world, {
            shape: targetShape,
            objectLayer: notMoving,
            motionType: MotionType.STATIC,
            position: offset,
        });
        addBody(bodies, body, spectrum[1]);
        const quaternion = quat.fromEuler([0, 0, 0, 1], [random() * 10, random() * 10, 0, 'xyz']);
        return { body, offset, quaternion, spin: [random() - 0.5, random() - 0.5, random() - 0.5] };
    });
}

const _spin: Quat = [0, 0, 0, 1];
const _position: Vec3 = [0, 0, 0];
const _quaternion: Quat = [0, 0, 0, 1];

function turnTargets(deltaTime: number): void {
    const angle = 0.1 * settings.targetSpeed * deltaTime;
    quat.multiply(turn, turn, quat.fromEuler(_spin, [angle, angle, 0, 'xyz']));
    for (const target of targets) {
        const amount = 0.1 * settings.targetSpeed * deltaTime;
        quat.multiply(
            target.quaternion,
            target.quaternion,
            quat.fromEuler(_spin, [target.spin[0] * amount, target.spin[1] * amount, target.spin[2] * amount, 'xyz']),
        );
        vec3.transformQuat(_position, target.offset, turn);
        quat.multiply(_quaternion, turn, target.quaternion);
        rigidBody.setTransform(world, target.body, _position, _quaternion, false);
    }
}

// each ray's start circles the sphere, turning by its own small rotation each step
type Ray = { quaternion: Quat; spin: Vec3; origin: Vec3; direction: Vec3; hit: boolean; end: Vec3 };
const rays: Ray[] = [];
function matchRays(): void {
    while (rays.length < settings.rays) {
        rays.push({
            quaternion: quat.fromEuler([0, 0, 0, 1], [random() * 10, random() * 10, random() * 10, 'xyz']),
            spin: [random() - 0.5, random() - 0.5, random() - 0.5],
            origin: [0, 0, 0],
            direction: [0, 0, 0],
            hit: false,
            end: [0, 0, 0],
        });
    }
    rays.length = settings.rays;
}

const collector = createClosestCastRayCollector();
const raySettings = createDefaultCastRaySettings();

function castRays(deltaTime: number): void {
    const amount = 0.1 * settings.raySpeed * deltaTime;
    for (const ray of rays) {
        quat.multiply(
            ray.quaternion,
            ray.quaternion,
            quat.fromEuler(_spin, [ray.spin[0] * amount, ray.spin[1] * amount, ray.spin[2] * amount, 'xyz']),
        );
        vec3.transformQuat(ray.origin, [RADIUS, 0, 0], ray.quaternion);
        vec3.normalize(ray.direction, vec3.negate(ray.direction, ray.origin));
        // from near to far along the ray
        const start = vec3.scaleAndAdd([0, 0, 0], ray.origin, ray.direction, settings.near);
        const length = Math.max(settings.far - settings.near, 0);
        collector.reset();
        castRay(world, collector, raySettings, start, ray.direction, length);
        ray.hit = collector.hit.status === CastRayStatus.COLLIDING;
        vec3.scaleAndAdd(ray.end, start, ray.direction, ray.hit ? collector.hit.fraction * length : length);
    }
}

placeTargets();
matchRays();

const panel = createPanel('cast ray');
const rayFolder = panel.folder('rays');
rayFolder.add(settings, 'rays', { min: 1, max: 1000, step: 1 }).onChange(matchRays);
rayFolder.add(settings, 'raySpeed', { min: 0, max: 20, label: 'speed' });
rayFolder.add(settings, 'near', { min: 0, max: RADIUS });
rayFolder.add(settings, 'far', { min: 0, max: RADIUS });
const targetFolder = panel.folder('targets');
targetFolder.add(settings, 'target', { label: 'shape', options: Object.keys(targetShapes) }).onChange(placeTargets);
targetFolder.add(settings, 'targets', { min: 1, max: 300, step: 1, label: 'count' }).onChange(placeTargets);
targetFolder.add(settings, 'targetSpeed', { min: 0, max: 20, label: 'speed' });
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('rays are cast from all around towards the middle, and each one stops at the first shape it hits');

const rayColor = debugColor(palette.muted);
const pointColor = debugColor(palette.light);

loop(
    stepTime,
    (deltaTime) => {
        turnTargets(deltaTime);
        updateWorld(world, undefined, deltaTime);
        castRays(deltaTime);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        for (const ray of rays) {
            drawLine(overlay, ray.origin, ray.end, rayColor);
            drawPoint(overlay, ray.origin, pointColor, 0.03);
            if (ray.hit) drawPoint(overlay, ray.end, pointColor, 0.03);
        }
        endOverlay(overlay);
        render(renderer);
    },
);
