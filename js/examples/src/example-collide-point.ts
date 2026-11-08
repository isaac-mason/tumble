import type { Vec3 } from 'math';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    collidePoint,
    createAllCollidePointCollector,
    createDefaultCollidePointSettings,
    createWorld,
    createWorldSettings,
    init,
    MotionType,
    rigidBody,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { createLabel, placeLabels } from './common/labels';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, drawPoint, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { debugColor, spectrum } from './common/theme';

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

// a point swung side to side through the block: collidePoint finds every body it is inside
const AMPLITUDE = 6;
const point: Vec3 = [0, 2, 0];
const collector = createAllCollidePointCollector();
const settings = createDefaultCollidePointSettings();
const label = createLabel('', [0, 4.5, 0]);
let inside = false;
let time = 0;

const panel = createPanel('collide point');
panel.monitor(() => (inside ? 'inside' : 'outside'), { label: 'the point is' });
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('a point swings through a block, and collidePoint tells you whether it is inside');

const insideColor = debugColor(spectrum[4]);
const outsideColor = debugColor(spectrum[1]);

loop(
    stepTime,
    (deltaTime) => {
        time += deltaTime;
        point[0] = Math.sin(time) * AMPLITUDE;
        updateWorld(world, undefined, deltaTime);
        collector.reset();
        collidePoint(world, collector, settings, point);
        inside = collector.hits.length > 0;
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        drawPoint(overlay, point, inside ? insideColor : outsideColor, 0.3, true);
        endOverlay(overlay);
        render(renderer);
        label.element.textContent = inside ? 'inside' : 'outside';
        label.position[0] = point[0];
        placeLabels(renderer, [label]);
    },
);
