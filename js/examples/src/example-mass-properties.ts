import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    compound,
    createWorld,
    createWorldSettings,
    enableCollision,
    init,
    MotionType,
    rigidBody,
    type Shape,
    updateWorld,
    type Vec3,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies, removeBody } from './common/bodies';
import { createInfo } from './common/info';
import { createLabel, type Label, placeLabels } from './common/labels';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, drawPoint, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { debugColor, palette } from './common/theme';

await init();
const renderer = await createRenderer([0, 5, 15], [0, 2, 0]);
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
    restitution: 0,
});
addBody(bodies, ground);

const SPACING = 3;
const stem = box.create({ halfExtents: [0.25, 0.75, 0.25], convexRadius: 0.05 });
const arm = box.create({ halfExtents: [0.75, 0.25, 0.25], convexRadius: 0.05 });

// a compound's mass, centre of mass and inertia come from its children, so where its parts are decides how it stands
const shapes: { shape: Shape; x: number; label: string }[] = [
    // its centre of mass over its base: it stands
    { shape: box.create({ halfExtents: [0.5, 1.5, 0.5], convexRadius: 0.05 }), x: -SPACING, label: 'a box' },
    // an upside down L, its centre of mass out past the stem's foot: it tips over
    {
        shape: compound.create({
            children: [
                { shape: stem, position: [0, 0.75, 0] },
                { shape: arm, position: [0.75, 1.25, 0] },
            ],
        }),
        x: 0,
        label: 'an L',
    },
    // a second L above, turned the other way, brings the centre of mass back over the stem: it stands
    {
        shape: compound.create({
            children: [
                { shape: stem, position: [0, 0.75, 0] },
                { shape: arm, position: [0.75, 1.25, 0] },
                { shape: stem, position: [0, 2.25, 0] },
                { shape: arm, position: [-0.75, 1.75, 0] },
            ],
        }),
        x: SPACING,
        label: 'two Ls',
    },
];

let placed: BodyId[] = [];
const labels: Label[] = shapes.map(({ x, label }) => createLabel(label, [x, 4.5, 0]));

function place(): void {
    for (const body of placed) {
        removeBody(bodies, body);
        rigidBody.remove(world, body);
    }
    placed = shapes.map(({ shape, x }) => {
        const body = rigidBody.create(world, {
            shape,
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: [x, 1.5, 0],
            restitution: 0,
        });
        addBody(bodies, body);
        return body;
    });
}
place();

const panel = createPanel('mass properties');
panel.button('again', place);
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('the cross on each body marks its centre of mass. a body stays up when that is over its base');

const centerColor = debugColor(palette.light);
const center: Vec3 = [0, 0, 0];

loop(
    stepTime,
    (deltaTime) => {
        updateWorld(world, undefined, deltaTime);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        for (const body of placed) {
            rigidBody.getCenterOfMassPosition(center, world, body);
            drawPoint(overlay, center, centerColor, 0.3, true);
        }
        endOverlay(overlay);
        render(renderer);
        placeLabels(renderer, labels);
    },
);
