import { type Quat, quat, type Vec3, vec3 } from 'math';
import { mulberry32 } from 'math/random';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    CastShapeStatus,
    capsule,
    castShape,
    createClosestCastShapeCollector,
    createDefaultCastShapeSettings,
    createWorld,
    createWorldSettings,
    debug,
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
import { addOverlayControls, beginOverlay, createOverlay, drawLine, endOverlay, overlayDraw } from './common/overlay';
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

const rng = mulberry32.create(17);
const random = () => mulberry32.sample(rng);

// shapes swept from points on a sphere of this radius towards its centre, as far as the radius
const RADIUS = 5;
const targetShapes: Record<string, Shape> = {
    'torus knot mesh': triangleMesh.create(torusKnot(1, 0.4, 40, 10)),
    sphere: sphere.create({ radius: 1 }),
    box: box.create({ halfExtents: [1, 1, 1] }),
    capsule: capsule.create({ halfHeightOfCylinder: 1, radius: 0.5 }),
};
const sweptShapes: Record<string, Shape> = {
    box: box.create({ halfExtents: [0.1, 0.1, 0.1] }),
    sphere: sphere.create({ radius: 0.1 }),
    capsule: capsule.create({ halfHeightOfCylinder: 0.1, radius: 0.075 }),
};

const settings = { casts: 10, castSpeed: 1, swept: 'box', target: 'torus knot mesh', targets: 1, targetSpeed: 1 };

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

// each cast's start circles the sphere, turning by its own small rotation each step. the swept shape keeps its own
// orientation: only where it starts turns
type Cast = { quaternion: Quat; spin: Vec3; origin: Vec3; displacement: Vec3; hit: boolean; end: Vec3 };
const casts: Cast[] = [];
function matchCasts(): void {
    while (casts.length < settings.casts) {
        casts.push({
            quaternion: quat.fromEuler([0, 0, 0, 1], [random() * 10, random() * 10, random() * 10, 'xyz']),
            spin: [random() - 0.5, random() - 0.5, random() - 0.5],
            origin: [0, 0, 0],
            displacement: [0, 0, 0],
            hit: false,
            end: [0, 0, 0],
        });
    }
    casts.length = settings.casts;
}

const collector = createClosestCastShapeCollector();
const castSettings = createDefaultCastShapeSettings();
const IDENTITY: Quat = [0, 0, 0, 1];
const UNIT_SCALE: Vec3 = [1, 1, 1];

function castShapes(deltaTime: number): void {
    const amount = 0.1 * settings.castSpeed * deltaTime;
    const swept = sweptShapes[settings.swept]!;
    for (const cast of casts) {
        quat.multiply(
            cast.quaternion,
            cast.quaternion,
            quat.fromEuler(_spin, [cast.spin[0] * amount, cast.spin[1] * amount, cast.spin[2] * amount, 'xyz']),
        );
        vec3.transformQuat(cast.origin, [RADIUS, 0, 0], cast.quaternion);
        // to the centre
        vec3.negate(cast.displacement, cast.origin);
        collector.reset();
        castShape(world, collector, castSettings, swept, cast.origin, IDENTITY, UNIT_SCALE, cast.displacement);
        cast.hit = collector.hit.status === CastShapeStatus.COLLIDING;
        // where the shape's centre was as it touched, or at the end
        vec3.scaleAndAdd(cast.end, cast.origin, cast.displacement, cast.hit ? collector.hit.fraction : 1);
    }
}

placeTargets();
matchCasts();

const panel = createPanel('cast shape');
const castFolder = panel.folder('casts');
castFolder.add(settings, 'casts', { min: 1, max: 200, step: 1, label: 'count' }).onChange(matchCasts);
castFolder.add(settings, 'castSpeed', { min: 0, max: 20, label: 'speed' });
castFolder.add(settings, 'swept', { label: 'shape', options: Object.keys(sweptShapes) });
const targetFolder = panel.folder('targets');
targetFolder.add(settings, 'target', { label: 'shape', options: Object.keys(targetShapes) }).onChange(placeTargets);
targetFolder.add(settings, 'targets', { min: 1, max: 300, step: 1, label: 'count' }).onChange(placeTargets);
targetFolder.add(settings, 'targetSpeed', { min: 0, max: 20, label: 'speed' });
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('small shapes are swept towards the middle and drawn where they start and where they first touch');

const lineColor = debugColor(palette.muted);
const shapeColor = debugColor(palette.light);

loop(
    stepTime,
    (deltaTime) => {
        turnTargets(deltaTime);
        updateWorld(world, undefined, deltaTime);
        castShapes(deltaTime);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        const swept = sweptShapes[settings.swept]!;
        for (const cast of casts) {
            drawLine(overlay, cast.origin, cast.end, lineColor);
            debug.shape(overlayDraw(overlay, false), swept, cast.origin, IDENTITY, { color: shapeColor });
            debug.shape(overlayDraw(overlay, false), swept, cast.end, IDENTITY, { color: shapeColor });
        }
        endOverlay(overlay);
        render(renderer);
    },
);
