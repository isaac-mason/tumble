import { quat, type Vec3, vec3 } from 'math';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    capsule,
    createWorld,
    createWorldSettings,
    type DistanceConstraintSettings,
    debug,
    distanceConstraint,
    emptyShape,
    enableCollision,
    filter,
    init,
    type KCC,
    kcc,
    kccPolicy,
    MotionType,
    type Quat,
    rigidBody,
    type Shape,
    SpringMode,
    sphere,
    transformed,
    triangleMesh,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies, removeBody } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, drawArrow, drawPoint, endOverlay, overlayDraw } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { debugColor, palette, spectrum } from './common/theme';

await init();
const SPAWN: Vec3 = [0, 10, 0];
const renderer = await createRenderer([0, 20, 20], SPAWN);
const GRAVITY: Vec3 = [0, -25, 0];
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
worldSettings.gravity = GRAVITY;
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

function addStatic(shape: Shape, position: Vec3, quaternion: Quat = [0, 0, 0, 1], color?: string): BodyId {
    const body = rigidBody.create(world, { shape, objectLayer: notMoving, motionType: MotionType.STATIC, position, quaternion });
    addBody(bodies, body, color);
    return body;
}
function addMoving(shape: Shape, position: Vec3, motionType: MotionType, color?: string): BodyId {
    const body = rigidBody.create(world, { shape, objectLayer: moving, motionType, position });
    addBody(bodies, body, color);
    return body;
}
const boxOf = (halfExtents: Vec3) => box.create({ halfExtents });

/* the level */

// lava far below: falling in sends the character back to the start
const lava = addStatic(boxOf([1000, 2, 1000]), [0, -50, 0], undefined, spectrum[1]);
const conveyor = addStatic(boxOf([10, 0.25, 2]), [0, 0, -10], undefined, spectrum[6]);
addStatic(boxOf([50, 0.5, 50]), [0, -0.5, 0]);
const wall = boxOf([0.5, 2, 45]);
addStatic(wall, [-45, 1, 0]);
addStatic(wall, [45, 1, 0]);

// five flights of stairs, their steps 0.3 to 0.7 m high
for (let j = 0; j < 5; j++) {
    const height = 0.3 + 0.1 * j;
    const step = boxOf([2, height / 2, 2]);
    for (let i = 1; i < 10; i++)
        addStatic(step, [15 + 5 * j, i * height - 0.5 + height / 2, -20 - i * 3], undefined, palette.muted);
}

// slopes from 70 down to 25 degrees
const slope = boxOf([2.5, 0.6, 8]);
for (let i = 0; i < 10; i++) {
    addStatic(
        slope,
        [-40 + 5 * i, 2, -25],
        quat.setAxisAngle([0, 0, 0, 1], [1, 0, 0], ((70 - i * 5) * Math.PI) / 180),
        palette.muted,
    );
}

// something to push
for (const [shape, position] of [
    [boxOf([0.75, 0.75, 0.75]), [-10, 5, 10]],
    [sphere.create({ radius: 0.75 }), [-8, 2, 10]],
] as const) {
    const body = rigidBody.create(world, {
        shape,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [...position],
        friction: 0.1,
        mass: 1,
    });
    addBody(bodies, body);
}

// platforms that carry the character: one spinning, one sliding, one going round a figure of eight
const spinning = addMoving(boxOf([4, 0.2, 4]), [0, 2, 15], MotionType.KINEMATIC, spectrum[4]);
const sliding = addMoving(boxOf([3, 0.2, 3]), [20, 2, -5], MotionType.KINEMATIC, spectrum[4]);
const diagonal = addMoving(boxOf([3, 0.2, 3]), [20, 3, 5], MotionType.KINEMATIC, spectrum[4]);

// rolling hills, a 16 by 16 grid of 2 m cells
{
    const GRID = 16;
    const CELL = 2;
    const heightAt = (x: number, z: number) =>
        Math.sin(x * 0.3) * Math.cos(z * 0.3) + Math.sin(x * 0.7 + 1) * Math.sin(z * 0.7) * 0.6 + Math.cos((x + z) * 0.15) * 0.4;
    const positions: number[] = [];
    const indices: number[] = [];
    for (let iz = 0; iz <= GRID; iz++) {
        for (let ix = 0; ix <= GRID; ix++) {
            const x = (ix - GRID / 2) * CELL;
            const z = (iz - GRID / 2) * CELL;
            positions.push(x, heightAt(x, z), z);
        }
    }
    const row = GRID + 1;
    for (let iz = 0; iz < GRID; iz++) {
        for (let ix = 0; ix < GRID; ix++) {
            const corner = iz * row + ix;
            indices.push(corner, corner + row, corner + 1, corner + 1, corner + row, corner + row + 1);
        }
    }
    addStatic(triangleMesh.create({ positions, indices }), [-30, 2, 30]);
}

// a bumpy floor: balls sunk into the ground, their tops showing
const BUMPS = [
    [25, 28, 0.8],
    [29, 32, 0.9],
    [23, 34, 0.7],
    [31, 28, 0.85],
    [27, 30, 1],
    [22, 30, 0.5],
    [32, 30, 0.55],
    [25, 26, 0.45],
    [29, 34, 0.6],
    [21, 32, 0.5],
    [33, 32, 0.48],
    [27, 35, 0.52],
    [27, 25, 0.55],
    [24, 31, 0.35],
    [30, 29, 0.3],
    [26, 33, 0.38],
    [28, 27, 0.32],
    [22, 28, 0.28],
    [32, 34, 0.34],
    [20, 30, 0.4],
    [34, 30, 0.36],
    [23, 36, 0.33],
    [31, 26, 0.31],
] as const;
for (const [x, z, radius] of BUMPS) addStatic(sphere.create({ radius }), [x, -radius / 2, z], undefined, palette.muted);

// boxes doubling in size, to step up onto
let boxX = 5;
for (let i = 0; i < 5; i++) {
    const half = 0.25 * 2 ** i;
    addStatic(boxOf([half, half, half]), [boxX, half, 30], undefined, palette.muted);
    boxX += half * 3;
}

// a platform hung from four springs
{
    const center: Vec3 = [-10, 5, 35];
    const half: Vec3 = [3, 0.3, 3];
    const platform = rigidBody.create(world, {
        shape: boxOf(half),
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: center,
        linearDamping: 0.5,
        angularDamping: 0.5,
    });
    addBody(bodies, platform, spectrum[2]);
    const nothing = emptyShape.create();
    for (const [cx, cz] of [
        [-1, -1],
        [1, -1],
        [-1, 1],
        [1, 1],
    ] as const) {
        const anchorAt: Vec3 = [center[0] + cx * half[0], 12, center[2] + cz * half[2]];
        const anchor = rigidBody.create(world, {
            shape: nothing,
            objectLayer: notMoving,
            motionType: MotionType.STATIC,
            position: anchorAt,
        });
        const spring: DistanceConstraintSettings = {
            pointA: anchorAt,
            pointB: [anchorAt[0], center[1] + half[1], anchorAt[2]],
            springSettings: { mode: SpringMode.FREQUENCY_AND_DAMPING, frequencyOrStiffness: 0.8, damping: 0.1 },
            minDistance: 2,
            maxDistance: 8,
        };
        distanceConstraint.create(world, anchor, platform, spring);
    }
}

let steps = 0;
const spinPosition: Vec3 = [0, 0, 0];
const spinQuaternion: Quat = [0, 0, 0, 1];
function movePlatforms(deltaTime: number): void {
    const time = steps * deltaTime;
    rigidBody.getTransform(spinPosition, spinQuaternion, world, spinning);
    quat.multiply(spinQuaternion, spinQuaternion, quat.setAxisAngle([0, 0, 0, 1], [0, 1, 0], deltaTime));
    rigidBody.moveKinematic(world, spinning, spinPosition, spinQuaternion, deltaTime);
    rigidBody.moveKinematic(world, sliding, [20 + Math.sin(time * 0.15 * 2 * Math.PI) * 8, 2, -5], [0, 0, 0, 1], deltaTime);
    const phase = time * 0.12 * 2 * Math.PI;
    rigidBody.moveKinematic(world, diagonal, [20 + Math.sin(phase) * 6, 3 + Math.sin(phase * 2) * 2, 5], [0, 0, 0, 1], deltaTime);
}

/* the character */

// its shapes stand on its feet: the character's position is the bottom of the shape
const HEIGHT_STANDING = 2;
const RADIUS_STANDING = 1;
const HEIGHT_CROUCHING = 1;
const RADIUS_CROUCHING = 0.8;
const raised = (shape: Shape, height: number, radius: number) =>
    transformed.create({ shape, position: [0, height / 2 + radius, 0] });
const shapes: Record<string, { standing: Shape; crouching: Shape }> = {
    capsule: {
        standing: raised(
            capsule.create({ halfHeightOfCylinder: HEIGHT_STANDING / 2, radius: RADIUS_STANDING }),
            HEIGHT_STANDING,
            RADIUS_STANDING,
        ),
        crouching: raised(
            capsule.create({ halfHeightOfCylinder: HEIGHT_CROUCHING / 2, radius: RADIUS_CROUCHING }),
            HEIGHT_CROUCHING,
            RADIUS_CROUCHING,
        ),
    },
    box: {
        standing: raised(
            boxOf([RADIUS_STANDING, HEIGHT_STANDING / 2 + RADIUS_STANDING, RADIUS_STANDING]),
            HEIGHT_STANDING,
            RADIUS_STANDING,
        ),
        crouching: raised(
            boxOf([RADIUS_CROUCHING, HEIGHT_CROUCHING / 2 + RADIUS_CROUCHING, RADIUS_CROUCHING]),
            HEIGHT_CROUCHING,
            RADIUS_CROUCHING,
        ),
    },
};

// how the character treats what it meets: the conveyor's surface moves along x at 5 m/s
const policy = kccPolicy.create();
kccPolicy.set(policy, conveyor, { surfaceLinearVelocity: [5, 0, 0] });

const settings = { shape: 'capsule', walkStairs: true, stickToFloor: true };
const show = { contacts: false, groundNormal: false, groundVelocity: false, velocity: false };
const queryFilter = filter.create(world);
let character: KCC;
let crouched = false;

/** the supporting plane through the bottom hemisphere's centre: only contacts below it hold the character up */
const supportingPlane = (radius: number): [number, number, number, number] => [0, 1, 0, -radius];

function createCharacter(): void {
    const shape = shapes[settings.shape]!.standing;
    character = kcc.create(
        {
            shape,
            innerRigidBody: { shape, objectLayer: moving },
            mass: 1000,
            maxSlopeAngle: (45 * Math.PI) / 180,
            maxStrength: 100,
            characterPadding: 0.02,
            penetrationRecoverySpeed: 1,
            predictiveContactDistance: 0.1,
            supportingVolumePlane: supportingPlane(RADIUS_STANDING),
        },
        SPAWN,
        [0, 0, 0, 1],
    );
    kcc.add(world, character);
    addBody(bodies, character.innerRigidBodyId, spectrum[2]);
    filter.setFromBody(queryFilter, world, character.innerRigidBodyId);
    crouched = false;
}

function releaseCharacter(): void {
    removeBody(bodies, character.innerRigidBodyId);
    kcc.remove(world, character);
    kcc.release(character);
}
createCharacter();

function setCrouched(want: boolean): void {
    if (want === crouched) return;
    const kind = shapes[settings.shape]!;
    // only where it fits
    if (!kcc.setShape(world, character, want ? kind.crouching : kind.standing, queryFilter, policy, 0.1)) return;
    crouched = want;
    kcc.setTuning(character, { supportingVolumePlane: supportingPlane(want ? RADIUS_CROUCHING : RADIUS_STANDING) });
}

/* input and steering, as crashcat's and jolt's character samples steer */

const SPEED = 10;
const JUMP_SPEED = 15;
const keys = new Set<string>();
window.addEventListener('keydown', (event) => keys.add(event.code));
window.addEventListener('keyup', (event) => keys.delete(event.code));
const desired: Vec3 = [0, 0, 0];

function steer(deltaTime: number): void {
    const forward = (keys.has('KeyW') ? 1 : 0) - (keys.has('KeyS') ? 1 : 0);
    const right = (keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0);
    // on the ground plane, relative to where the camera looks
    const look = vec3.subtract([0, 0, 0], renderer.controls.target as Vec3, renderer.camera.position as Vec3);
    const ahead = vec3.normalize([0, 0, 0], [look[0], 0, look[2]]);
    const side: Vec3 = [-ahead[2], 0, ahead[0]];
    const direction: Vec3 = [ahead[0] * forward + side[0] * right, 0, ahead[2] * forward + side[2] * right];
    const length = vec3.length(direction);
    if (length > 1e-6) vec3.scale(direction, direction, 1 / length);

    // no sliding down walkable slopes while standing still. eased, so starts and stops are not instant
    kccPolicy.setDefaults(policy, { slideOnWalkable: length > 1e-6 });
    vec3.scaleAndAdd(desired, vec3.scale(desired, desired, 0.75), direction, 0.25 * SPEED);

    kcc.updateGroundVelocity(world, character, policy);
    const up = character.up;
    const velocity = character.linearVelocity;
    const vertical = vec3.scale([0, 0, 0], up, vec3.dot(velocity, up));
    const ground = character.ground.velocity;
    const towardsGround = vec3.dot(vec3.subtract([0, 0, 0], vertical, ground), up) < 0.1;
    const next: Vec3 = [...vertical];
    if (character.ground.state === kcc.GroundState.ON_GROUND && towardsGround) {
        vec3.copy(next, ground);
        if (keys.has('Space')) vec3.scaleAndAdd(next, next, up, JUMP_SPEED);
    }
    vec3.scaleAndAdd(next, next, GRAVITY, deltaTime);
    character.linearVelocity = vec3.add([0, 0, 0], next, desired);
}

const updateSettings = kcc.createDefaultUpdateSettings();

/** true when the character went back to the start */
function step(deltaTime: number): boolean {
    updateSettings.stickToFloorStepDown = settings.stickToFloor ? [0, -0.5, 0] : [0, 0, 0];
    updateSettings.walkStairsStepUp = settings.walkStairs ? [0, 0.4, 0] : [0, 0, 0];
    setCrouched(keys.has('ShiftLeft') || keys.has('ShiftRight'));
    steer(deltaTime);
    movePlatforms(deltaTime);
    kcc.update(world, character, deltaTime, GRAVITY, updateSettings, policy, queryFilter);
    // the lava sends it back to the start
    const respawned = kcc.hasCollidedWith(character, lava);
    if (respawned) {
        kcc.setPosition(world, character, SPAWN);
        character.linearVelocity = [0, 0, 0];
    }
    updateWorld(world, undefined, deltaTime);
    steps++;
    return respawned;
}

/* panel and drawing */

const panel = createPanel('character controller');
panel.add(settings, 'shape', { options: Object.keys(shapes) }).onChange(() => {
    releaseCharacter();
    createCharacter();
});
panel.add(settings, 'walkStairs', { label: 'walk stairs' });
panel.add(settings, 'stickToFloor', { label: 'stick to floor' });
panel.monitor(() => ['on ground', 'on steep ground', 'not supported', 'in air'][character.ground.state], { label: 'ground' });
const characterFolder = panel.folder('character debug');
characterFolder.add(show, 'contacts');
characterFolder.add(show, 'groundNormal', { label: 'ground normal' });
characterFolder.add(show, 'groundVelocity', { label: 'ground velocity' });
characterFolder.add(show, 'velocity');
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('WASD to move, space to jump, shift to crouch and drag to look around');

const normalColor = debugColor(spectrum[4]);
const groundColor = debugColor(spectrum[6]);
const velocityColor = debugColor(spectrum[0]);

// the camera follows the character as drawn, keeping the turn and zoom it was given
const before: Vec3 = [...SPAWN];
const now: Vec3 = [...SPAWN];
const drawn: Vec3 = [...SPAWN];
const followed: Vec3 = [...SPAWN];

loop(
    stepTime,
    (deltaTime) => {
        vec3.copy(before, character.position);
        const respawned = step(deltaTime);
        vec3.copy(now, character.position);
        // drawn at the start, not swept back across the level
        if (respawned) vec3.copy(before, now);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        vec3.lerp(drawn, before, now, alpha);
        const moved = vec3.subtract([0, 0, 0], drawn, followed);
        vec3.add(renderer.camera.position as Vec3, renderer.camera.position as Vec3, moved);
        vec3.add(renderer.controls.target as Vec3, renderer.controls.target as Vec3, moved);
        vec3.copy(followed, drawn);

        beginOverlay(overlay);
        if (show.contacts) debug.characterContacts(overlayDraw(overlay, true), character);
        const at = character.position;
        if (character.ground.state !== kcc.GroundState.IN_AIR) {
            if (show.groundNormal) drawArrow(overlay, at, character.ground.normal, 2, normalColor, true);
            if (show.groundVelocity) drawArrow(overlay, at, character.ground.velocity, 1, groundColor, true);
            if (show.groundNormal) drawPoint(overlay, character.ground.position, normalColor, 0.15, true);
        }
        if (show.velocity) drawArrow(overlay, at, character.linearVelocity, 1, velocityColor, true);
        endOverlay(overlay);
        render(renderer);
    },
);
