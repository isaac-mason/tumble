import { type Quat, quat, type Vec3, vec3 } from 'math';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    type ContactEvent,
    type ContactSettings,
    createWorld,
    createWorldSettings,
    cylinder,
    enableCollision,
    init,
    type Listener,
    MotionType,
    rigidBody,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { createLabel, type Label, placeLabels } from './common/labels';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { palette, spectrum } from './common/theme';

await init();
const renderer = await createRenderer([60, 20, 50], [0, 0, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

// a belt is a body whose surface moves though the body does not: a contact hook gives the contact the surface's
// velocity, in the belt's own space, turned into the world's, and the solver drags what touches it along
type Belt = { linear?: Vec3; angular?: Vec3 };
const belts = new Map<BodyId, Belt>();

const _rotation: Quat = [0, 0, 0, 1];
const _linearA: Vec3 = [0, 0, 0];
const _linearB: Vec3 = [0, 0, 0];
const _angularA: Vec3 = [0, 0, 0];
const _angularB: Vec3 = [0, 0, 0];
const _centerA: Vec3 = [0, 0, 0];
const _centerB: Vec3 = [0, 0, 0];

/** a belt's surface velocity in the world, or nothing for a body that is no belt */
function surfaceVelocity(out: Vec3, body: BodyId, local: Vec3 | undefined): Vec3 {
    if (local === undefined) return vec3.zero(out);
    rigidBody.getQuaternion(_rotation, world, body);
    return vec3.transformQuat(out, local, _rotation);
}

function moveSurfaces(contact: ContactEvent, settings: ContactSettings): void {
    const beltA = belts.get(contact.bodyIdA);
    const beltB = belts.get(contact.bodyIdB);
    if (beltA === undefined && beltB === undefined) return;
    if (beltA?.linear !== undefined || beltB?.linear !== undefined) {
        // b's surface relative to a's
        surfaceVelocity(_linearA, contact.bodyIdA, beltA?.linear);
        surfaceVelocity(_linearB, contact.bodyIdB, beltB?.linear);
        vec3.subtract(settings.relativeLinearSurfaceVelocity, _linearB, _linearA);
    }
    if (beltA?.angular !== undefined || beltB?.angular !== undefined) {
        surfaceVelocity(_angularA, contact.bodyIdA, beltA?.angular);
        surfaceVelocity(_angularB, contact.bodyIdB, beltB?.angular);
        // the angular velocity is about a's centre of mass, so b's spin moves b's centre as well
        vec3.zero(settings.relativeLinearSurfaceVelocity);
        if (beltB?.angular !== undefined) {
            rigidBody.getCenterOfMassPosition(_centerA, world, contact.bodyIdA);
            rigidBody.getCenterOfMassPosition(_centerB, world, contact.bodyIdB);
            vec3.cross(settings.relativeLinearSurfaceVelocity, _angularB, vec3.subtract(_centerA, _centerA, _centerB));
        }
        vec3.subtract(settings.relativeAngularSurfaceVelocity, _angularB, _angularA);
    }
}

const listener: Listener = { onContactAdded: moveSurfaces, onContactPersisted: moveSurfaces };

const floor = rigidBody.create(world, {
    shape: box.create({ halfExtents: [100, 0.5, 100] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
    restitution: 0,
    friction: 1,
});
addBody(bodies, floor);

const labels: Label[] = [];
const cargo = box.create({ halfExtents: [2, 2, 2] });
function addCargo(position: Vec3, friction: number, color?: string): void {
    const body = rigidBody.create(world, {
        shape: cargo,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position,
        restitution: 0,
        friction,
    });
    addBody(bodies, body, color);
}

// four static belts in a square, each tilted up a degree, each of more friction than the last, all moving along their
// own -z at 10 m/s
const BELT_WIDTH = 10;
const BELT_LENGTH = 50;
const beltShape = box.create({ halfExtents: [BELT_WIDTH, 0.1, BELT_LENGTH] });
const tilt = quat.fromEuler([0, 0, 0, 1], [Math.PI / 180, 0, 0, 'xyz']);
for (let i = 0; i < 4; i++) {
    const rotation = quat.multiply([0, 0, 0, 1], quat.fromEuler([0, 0, 0, 1], [0, 0.5 * Math.PI * i, 0, 'xyz']), tilt);
    const belt = rigidBody.create(world, {
        shape: beltShape,
        objectLayer: notMoving,
        motionType: MotionType.STATIC,
        position: vec3.transformQuat([0, 0, 0], [BELT_LENGTH, 6, BELT_WIDTH], rotation),
        quaternion: rotation,
        restitution: 0,
        friction: 0.25 * (i + 1),
    });
    addBody(bodies, belt, palette.muted);
    belts.set(belt, { linear: [0, 0, -10] });
}

// cargo of less and less friction along the first belt
for (let i = 0; i <= 10; i++) {
    const friction = 1 - 0.1 * i;
    const position: Vec3 = [-BELT_LENGTH + i * 10, 10, -BELT_LENGTH];
    addCargo(position, friction);
    labels.push(createLabel(friction.toFixed(1), [position[0], 14, position[2]]));
}

// a dynamic belt on two loose rollers: its moving surface drives the cargo on it, and it on the rollers
const roller = cylinder.create({ halfHeight: 6, radius: 1 });
const sideways: Quat = [0, 0, Math.SQRT1_2, Math.SQRT1_2];
for (const z of [-20, 20]) {
    const body = rigidBody.create(world, {
        shape: roller,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [-25, 1, z],
        quaternion: sideways,
        restitution: 0,
        friction: 0.1,
    });
    addBody(bodies, body, palette.light);
}
const dynamicBelt = rigidBody.create(world, {
    shape: box.create({ halfExtents: [5, 0.1, 25] }),
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [-25, 3, 0],
    restitution: 0,
    friction: 1,
});
addBody(bodies, dynamicBelt, palette.muted);
belts.set(dynamicBelt, { linear: [0, 0, -10] });
addCargo([-25, 6, 15], 1);

// a turntable: a static belt whose surface turns at 10 degrees a second, and cargo of less and less friction on it
const turntable = rigidBody.create(world, {
    shape: box.create({ halfExtents: [20, 0.1, 20] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [10, 3, 0],
    restitution: 0,
    friction: 1,
});
addBody(bodies, turntable, palette.muted);
belts.set(turntable, { angular: [0, (10 * Math.PI) / 180, 0] });
for (let i = 0; i <= 6; i++) {
    const friction = 1 - 0.1 * i;
    const position: Vec3 = [10, 10, -15 + 5 * i];
    addCargo(position, friction, spectrum[i % spectrum.length]);
    labels.push(createLabel(friction.toFixed(1), [position[0], 14, position[2]]));
}

const panel = createPanel('conveyor belt');
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('these belts move their surface rather than themselves, with a contact hook setting the surface velocity');

loop(
    stepTime,
    (deltaTime) => {
        updateWorld(world, listener, deltaTime);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        endOverlay(overlay);
        render(renderer);
        placeLabels(renderer, labels);
    },
);
