import type { Quat, Vec3 } from 'math';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    collideShape,
    createAllCollideShapeCollector,
    createDefaultCollideShapeSettings,
    createWorld,
    createWorldSettings,
    debug,
    init,
    MotionType,
    rigidBody,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { createLabel, placeLabels } from './common/labels';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, drawArrow, drawPoint, endOverlay, overlayDraw } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { debugColor, palette, spectrum } from './common/theme';

await init();
const renderer = await createRenderer([0, 5, 15], [0, 2, 0]);
const worldSettings = createWorldSettings();
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
worldSettings.gravity = [0, 0, 0];
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const block = rigidBody.create(world, {
    shape: box.create({ halfExtents: [2, 2, 2] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, 2, 0],
});
addBody(bodies, block);

// a box swung side to side through the block: collideShape finds every body it overlaps, with where they touch and
// the way out
const AMPLITUDE = 6;
const query = box.create({ halfExtents: [0.5, 0.5, 0.5] });
const position: Vec3 = [0, 2, 0];
const quaternion: Quat = [0, 0, 0, 1];
const scale: Vec3 = [1, 1, 1];
const collector = createAllCollideShapeCollector();
const settings = createDefaultCollideShapeSettings();
const label = createLabel('', [0, 4.5, 0]);
let time = 0;

const panel = createPanel('collide shape');
panel.monitor(() => collector.hits.length, { label: 'hits' });
panel.monitor(() => collector.hits[0]?.penetration ?? 0, { label: 'penetration', format: (value) => `${value.toFixed(3)} m` });
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('a box swings through a block, and collideShape reports the deepest points and the way to separate them');

const hitColor = debugColor(spectrum[4]);
const clearColor = debugColor(spectrum[1]);
const pointColor = debugColor(palette.light);
const axis: Vec3 = [0, 0, 0];

loop(
    stepTime,
    (deltaTime) => {
        time += deltaTime;
        position[0] = Math.sin(time) * AMPLITUDE;
        updateWorld(world, undefined, deltaTime);
        collector.reset();
        collideShape(world, collector, settings, query, position, quaternion, scale);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        const touching = collector.hits.length > 0;
        debug.shape(overlayDraw(overlay, true), query, position, quaternion, { color: touching ? hitColor : clearColor });
        for (const hit of collector.hits) {
            drawPoint(overlay, hit.pointA, pointColor, 0.1, true);
            drawPoint(overlay, hit.pointB, pointColor, 0.1, true);
            // the penetration axis moves the block out of the box, so the box moves out the other way
            const length = Math.hypot(hit.penetrationAxis[0], hit.penetrationAxis[1], hit.penetrationAxis[2]);
            if (length > 0) {
                axis[0] = -hit.penetrationAxis[0] / length;
                axis[1] = -hit.penetrationAxis[1] / length;
                axis[2] = -hit.penetrationAxis[2] / length;
                drawArrow(overlay, hit.pointA, axis, 1, pointColor, true);
            }
        }
        endOverlay(overlay);
        render(renderer);
        label.element.textContent = touching ? 'overlapping' : 'clear';
        label.position[0] = position[0];
        placeLabels(renderer, [label]);
    },
);
