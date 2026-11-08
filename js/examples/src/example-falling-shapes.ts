import { quat, type Vec3, vec3 } from 'math';
import { mulberry32 } from 'math/random';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    createWorld,
    createWorldSettings,
    cylinder,
    enableCollision,
    init,
    MotionType,
    rigidBody,
    type Shape,
    sphere,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { onClick } from './common/pointer';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { spectrum } from './common/theme';

await init();
const renderer = await createRenderer([18, 14, 22], [0, 2, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const rng = mulberry32.create(1);
const random = () => mulberry32.sample(rng);

const ground = rigidBody.create(world, {
    shape: box.create({ halfExtents: [15, 0.5, 15] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
});
addBody(bodies, ground);

// each kind's shape made once and shared by every body of it, drawn as one instanced mesh
const kinds: { shape: Shape; color: string }[] = [
    { shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }), color: spectrum[5] },
    { shape: sphere.create({ radius: 0.5 }), color: spectrum[1] },
    { shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.35 }), color: spectrum[3] },
    { shape: cylinder.create({ halfHeight: 0.4, radius: 0.5 }), color: spectrum[6] },
    { shape: box.create({ halfExtents: [1, 0.2, 0.4] }), color: spectrum[2] },
];

const axis: Vec3 = [0, 0, 0];
function drop(count: number): void {
    for (let i = 0; i < count; i++) {
        const kind = kinds[Math.floor(random() * kinds.length)]!;
        vec3.normalize(axis, vec3.set(axis, random() - 0.5, random() - 0.5, random() - 0.5));
        const body = rigidBody.create(world, {
            shape: kind.shape,
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: [(random() - 0.5) * 8, 8 + random() * 10, (random() - 0.5) * 8],
            quaternion: quat.setAxisAngle([0, 0, 0, 1], axis, random() * Math.PI),
            friction: 0.5,
        });
        addBody(bodies, body, kind.color);
    }
}
drop(150);
onClick(renderer, () => drop(20));

const panel = createPanel('falling shapes');
panel.button('drop 20 more', () => drop(20));
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('each shape is drawn between its last two steps. click to drop more');

loop(
    stepTime,
    (deltaTime) => {
        updateWorld(world, undefined, deltaTime);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        endOverlay(overlay);
        render(renderer);
    },
);
