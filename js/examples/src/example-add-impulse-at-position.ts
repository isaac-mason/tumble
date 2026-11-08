import { type Vec3, vec3 } from 'math';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    CastRayStatus,
    castRay,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    enableCollision,
    init,
    MotionType,
    rigidBody,
    sphere,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, drawArrow, drawPoint, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createPointerRay, onClick, pointerRay } from './common/pointer';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { debugColor, spectrum } from './common/theme';

await init();
const renderer = await createRenderer([0, 10, 15], [0, 2, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const ground = rigidBody.create(world, {
    shape: box.create({ halfExtents: [30, 0.5, 30] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
    restitution: 0.3,
});
addBody(bodies, ground);

for (const [shape, x] of [
    [box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 }), -3],
    [sphere.create({ radius: 1 }), 3],
] as const) {
    const body = rigidBody.create(world, {
        shape,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [x, 2, 0],
        restitution: 0.3,
        mass: 10,
    });
    addBody(bodies, body);
}

const settings = { impulse: 20 };

// a click pushes the body under the pointer along the ray, where the ray meets it: off centre, it turns as well as moves
const ray = createPointerRay();
const collector = createClosestCastRayCollector();
const raySettings = createDefaultCastRaySettings();
const RAY_LENGTH = 100;
const hitPoint: Vec3 = [0, 0, 0];
const hitDirection: Vec3 = [0, 0, 0];
// how many more frames to show the last push
let showFor = 0;

onClick(renderer, (event) => {
    pointerRay(ray, renderer, event.clientX, event.clientY);
    collector.reset();
    castRay(world, collector, raySettings, ray.origin, ray.direction, RAY_LENGTH);
    if (collector.hit.status !== CastRayStatus.COLLIDING) return;
    const body = collector.hit.bodyIdB;
    if (rigidBody.getMotionType(world, body) !== MotionType.DYNAMIC) return;
    vec3.scaleAndAdd(hitPoint, ray.origin, ray.direction, collector.hit.fraction * RAY_LENGTH);
    vec3.copy(hitDirection, ray.direction);
    rigidBody.addImpulseAtPosition(world, body, vec3.scale([0, 0, 0], ray.direction, settings.impulse), hitPoint);
    showFor = 12;
});

const panel = createPanel('add impulse at position');
panel.add(settings, 'impulse', { min: 1, max: 100, step: 1, label: 'impulse (N s)' });
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('click a body to push it at that point, along the ray from the camera');

const pushColor = debugColor(spectrum[4]);

loop(
    stepTime,
    (deltaTime) => {
        updateWorld(world, undefined, deltaTime);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        if (showFor > 0) {
            showFor--;
            drawPoint(overlay, hitPoint, pushColor, 0.15, true);
            drawArrow(overlay, hitPoint, hitDirection, 2, pushColor, true);
        }
        endOverlay(overlay);
        render(renderer);
    },
);
