import { type Vec3, vec3 } from 'math';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    CastRayStatus,
    type ConstraintId,
    ConstraintSpace,
    castRay,
    computeMassProperties,
    constraints,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    distanceConstraint,
    emptyShape,
    enableCollision,
    init,
    type Listener,
    type MassProperties,
    MotionType,
    rigidBody,
    type Shape,
    swingTwistConstraint,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, drawPoint, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createPointerRay, pointerRay } from './common/pointer';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { debugColor, palette, spectrum } from './common/theme';

await init();
const renderer = await createRenderer([0, 5, 15], [0, 0, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
worldSettings.gravity = [0, -20, 0];
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const ground = rigidBody.create(world, {
    shape: box.create({ halfExtents: [500, 1, 500] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -5, 0],
});
addBody(bodies, ground);

// a ragdoll: eleven boxes joined by swing twist constraints, each joint's swing and twist limited

const PELVIS = 0;
const UPPER_BODY = 1;
const HEAD = 2;
const UPPER_LEFT_ARM = 3;
const LOWER_LEFT_ARM = 4;
const UPPER_RIGHT_ARM = 5;
const LOWER_RIGHT_ARM = 6;
const UPPER_LEFT_LEG = 7;
const LOWER_LEFT_LEG = 8;
const UPPER_RIGHT_LEG = 9;
const LOWER_RIGHT_LEG = 10;
const PARTS = 11;

type Part = { halfExtents: Vec3; position: Vec3; parent: number };
type Joint = { a: number; b: number; pivotA: Vec3; pivotB: Vec3; axis: Vec3; angle: number; twist: number };

const SCALE = 1.8;
const ANGLE = Math.PI / 4;
const TWIST = 0;

function skeleton(): { parts: Part[]; joints: Joint[] } {
    const shoulders = 0.45 * SCALE;
    const upperArm = 0.4 * SCALE;
    const lowerArm = 0.4 * SCALE;
    const armSize = 0.15 * SCALE;
    const neck = 0.1 * SCALE;
    const head = 0.2 * SCALE;
    const upperBody = 0.6 * SCALE;
    const pelvis = 0.2 * SCALE;
    const pelvisSize = 0.25 * SCALE;
    const upperLeg = 0.5 * SCALE;
    const lowerLeg = 0.5 * SCALE;
    const legSize = 0.15 * SCALE;

    const lowerLegY = lowerLeg / 2;
    const upperLegY = lowerLegY + lowerLeg / 2 + upperLeg / 2;
    const pelvisY = upperLegY + upperLeg / 2 + pelvis / 2;
    const upperBodyY = pelvisY + pelvis / 2 + upperBody / 2;
    const headY = upperBodyY + upperBody / 2 + head / 2 + neck;
    const armY = upperBodyY + upperBody / 2;
    const upperArmX = shoulders / 2 + upperArm / 2;
    const lowerArmX = upperArmX + lowerArm / 2 + upperArm / 2;
    const legX = shoulders / 3;

    const parts: Part[] = [];
    parts[PELVIS] = { halfExtents: [shoulders / 2, pelvis / 2, pelvisSize / 2], position: [0, pelvisY, 0], parent: -1 };
    parts[UPPER_BODY] = {
        halfExtents: [shoulders / 2, upperBody / 2, armSize * 0.75],
        position: [0, upperBodyY, 0],
        parent: PELVIS,
    };
    parts[HEAD] = { halfExtents: [head * 0.6, head * 0.7, head * 0.6], position: [0, headY, 0], parent: UPPER_BODY };
    parts[UPPER_LEFT_ARM] = {
        halfExtents: [upperArm / 2, armSize / 2, armSize / 2],
        position: [-upperArmX, armY, 0],
        parent: UPPER_BODY,
    };
    parts[LOWER_LEFT_ARM] = {
        halfExtents: [lowerArm / 2, armSize / 2, armSize / 2],
        position: [-lowerArmX, armY, 0],
        parent: UPPER_LEFT_ARM,
    };
    parts[UPPER_RIGHT_ARM] = {
        halfExtents: [upperArm / 2, armSize / 2, armSize / 2],
        position: [upperArmX, armY, 0],
        parent: UPPER_BODY,
    };
    parts[LOWER_RIGHT_ARM] = {
        halfExtents: [lowerArm / 2, armSize / 2, armSize / 2],
        position: [lowerArmX, armY, 0],
        parent: UPPER_RIGHT_ARM,
    };
    parts[UPPER_LEFT_LEG] = {
        halfExtents: [legSize / 2, upperLeg / 2, legSize / 2],
        position: [-legX, upperLegY, 0],
        parent: PELVIS,
    };
    parts[LOWER_LEFT_LEG] = {
        halfExtents: [legSize / 2, lowerLeg / 2, legSize / 2],
        position: [-legX, lowerLegY, 0],
        parent: UPPER_LEFT_LEG,
    };
    parts[UPPER_RIGHT_LEG] = {
        halfExtents: [legSize / 2, upperLeg / 2, legSize / 2],
        position: [legX, upperLegY, 0],
        parent: PELVIS,
    };
    parts[LOWER_RIGHT_LEG] = {
        halfExtents: [legSize / 2, lowerLeg / 2, legSize / 2],
        position: [legX, lowerLegY, 0],
        parent: UPPER_RIGHT_LEG,
    };

    const up: Vec3 = [0, 1, 0];
    const side: Vec3 = [1, 0, 0];
    // pivots in each part's own space
    const joints: Joint[] = [
        {
            a: HEAD,
            b: UPPER_BODY,
            pivotA: [0, -head - neck / 2, 0],
            pivotB: [0, upperBody / 2, 0],
            axis: up,
            angle: ANGLE,
            twist: TWIST,
        },
        {
            a: LOWER_LEFT_LEG,
            b: UPPER_LEFT_LEG,
            pivotA: [0, lowerLeg / 2, 0],
            pivotB: [0, -upperLeg / 2, 0],
            axis: up,
            angle: ANGLE,
            twist: TWIST,
        },
        {
            a: LOWER_RIGHT_LEG,
            b: UPPER_RIGHT_LEG,
            pivotA: [0, lowerLeg / 2, 0],
            pivotB: [0, -upperLeg / 2, 0],
            axis: up,
            angle: ANGLE,
            twist: TWIST,
        },
        {
            a: UPPER_LEFT_LEG,
            b: PELVIS,
            pivotA: [0, upperLeg / 2, 0],
            pivotB: [-legX, -pelvis / 2, 0],
            axis: up,
            angle: ANGLE,
            twist: TWIST,
        },
        {
            a: UPPER_RIGHT_LEG,
            b: PELVIS,
            pivotA: [0, upperLeg / 2, 0],
            pivotB: [legX, -pelvis / 2, 0],
            axis: up,
            angle: ANGLE,
            twist: TWIST,
        },
        {
            a: PELVIS,
            b: UPPER_BODY,
            pivotA: [0, pelvis / 2, 0],
            pivotB: [0, -upperBody / 2, 0],
            axis: up,
            angle: ANGLE,
            twist: TWIST,
        },
        {
            a: UPPER_BODY,
            b: UPPER_LEFT_ARM,
            pivotA: [-shoulders / 2, upperBody / 2, 0],
            pivotB: [upperArm / 2, 0, 0],
            axis: side,
            angle: ANGLE,
            twist: TWIST,
        },
        {
            a: UPPER_BODY,
            b: UPPER_RIGHT_ARM,
            pivotA: [shoulders / 2, upperBody / 2, 0],
            pivotB: [-upperArm / 2, 0, 0],
            axis: side,
            angle: ANGLE,
            twist: TWIST,
        },
        {
            a: LOWER_LEFT_ARM,
            b: UPPER_LEFT_ARM,
            pivotA: [lowerArm / 2, 0, 0],
            pivotB: [-upperArm / 2, 0, 0],
            axis: side,
            angle: ANGLE,
            twist: TWIST,
        },
        {
            a: LOWER_RIGHT_ARM,
            b: UPPER_RIGHT_ARM,
            pivotA: [-lowerArm / 2, 0, 0],
            pivotB: [upperArm / 2, 0, 0],
            axis: side,
            angle: ANGLE,
            twist: TWIST,
        },
    ];
    return { parts, joints };
}

const { parts, joints } = skeleton();
const shapes: Shape[] = parts.map(({ halfExtents }) => box.create({ halfExtents, convexRadius: 0.05, density: SCALE }));

/**
 * jolt's ragdoll stabilisation ("stop my constraints from blowing up", oliver strunk): each child's mass kept within
 * [0.8, 1.2] of its parent's, and each parent's inertia raised towards the sum of its children's. every part is a box at
 * no rotation, so its inertia is diagonal and its principal moments are the diagonal
 */
function stabilizedMassProperties(): MassProperties[] {
    const masses = shapes.map((shape) =>
        computeMassProperties({ mass: 0, inertia: new Array(16).fill(0) as MassProperties['inertia'] }, shape),
    );
    // breadth first from the pelvis, every parent before its children
    const order = [PELVIS];
    for (let i = 0; i < order.length; i++)
        for (let part = 0; part < PARTS; part++) if (parts[part]!.parent === order[i]) order.push(part);

    const ratios: number[] = [];
    ratios[PELVIS] = 1;
    let totalRatio = 1;
    let totalMass = masses[PELVIS]!.mass;
    for (const part of order.slice(1)) {
        const parent = parts[part]!.parent;
        const ratio = masses[part]!.mass / masses[parent]!.mass;
        ratios[part] = ratios[parent]! * Math.max(0.8, Math.min(1.2, ratio));
        totalRatio += ratios[part]!;
        totalMass += masses[part]!.mass;
    }
    for (const part of order) {
        const properties = masses[part]!;
        const scale = (ratios[part]! * totalMass) / totalRatio / properties.mass;
        properties.mass *= scale;
        for (let i = 0; i < 16; i++) if (i % 4 !== 3 && i < 12) properties.inertia[i]! *= scale;
    }

    const largest = (part: number) => Math.max(masses[part]!.inertia[0]!, masses[part]!.inertia[5]!, masses[part]!.inertia[10]!);
    const childSums = new Array<number>(PARTS).fill(0);
    for (const part of order.slice(1).reverse()) childSums[parts[part]!.parent]! += largest(part) + childSums[part]!;
    for (const part of order) {
        if (childSums[part] === 0) continue;
        const least = Math.min(2 * largest(part), childSums[part]!);
        const inertia = masses[part]!.inertia;
        for (const diagonal of [0, 5, 10]) inertia[diagonal] = Math.max(inertia[diagonal]!, least);
    }
    return masses;
}
const massProperties = stabilizedMassProperties();

/** a unit direction perpendicular to the axis */
function tangent(axis: Vec3): Vec3 {
    const [x, y, z] = axis;
    const ax = Math.abs(x);
    const ay = Math.abs(y);
    const az = Math.abs(z);
    if (ax <= ay && ax <= az) return vec3.normalize([0, 0, 0], [0, -z, y]);
    if (ay <= az) return vec3.normalize([0, 0, 0], [z, 0, -x]);
    return vec3.normalize([0, 0, 0], [-y, x, 0]);
}

// the joined parts overlap at their joints, so the pairs a constraint joins are kept from colliding
const listener: Listener = { onBodyPairValidate: (a, b) => !rigidBody.bodiesShareConstraint(world, a, b) };

function createRagdoll(offset: Vec3, color: string): void {
    const ragdoll = parts.map((part, i) => {
        const body = rigidBody.create(world, {
            shape: shapes[i]!,
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: vec3.add([0, 0, 0], part.position, offset),
            linearDamping: 0.05,
            angularDamping: 0.05,
            restitution: 0,
            massPropertiesOverride: massProperties[i],
        });
        addBody(bodies, body, color);
        return body;
    });
    for (const joint of joints) {
        swingTwistConstraint.create(world, ragdoll[joint.a]!, ragdoll[joint.b]!, {
            space: ConstraintSpace.LOCAL,
            pointA: joint.pivotA,
            pointB: joint.pivotB,
            twistAxisA: joint.axis,
            twistAxisB: joint.axis,
            planeAxisA: tangent(joint.axis),
            planeAxisB: tangent(joint.axis),
            normalHalfConeAngle: joint.angle,
            planeHalfConeAngle: joint.angle,
            twistMinAngle: -joint.twist,
            twistMaxAngle: joint.twist,
        });
    }
    // thrown up, spinning
    for (const body of ragdoll) {
        rigidBody.addLinearVelocity(world, body, [0, 5, 0]);
        rigidBody.addAngularVelocity(world, body, [0, 0, 20]);
    }
}

for (let i = 0; i < 3; i++) createRagdoll([-5 + i * 4, i * 3, 0], spectrum[(i * 2 + 1) % spectrum.length]!);

// press on a part to grab it: a rope of up to 1 from a shapeless static anchor under the pointer, moved over a plane
// facing the camera as it was when grabbed. let go to drop it
const anchor = rigidBody.create(world, {
    shape: emptyShape.create(),
    objectLayer: moving,
    motionType: MotionType.STATIC,
    position: [0, 0, -10000],
});
const ray = createPointerRay();
const collector = createClosestCastRayCollector();
const raySettings = createDefaultCastRaySettings();
const RAY_LENGTH = 1000;
const planePoint: Vec3 = [0, 0, 0];
const planeNormal: Vec3 = [0, 0, 0];
const held: Vec3 = [0, 0, 0];
let rope: ConstraintId | null = null;

renderer.canvas.addEventListener('pointerdown', (event) => {
    pointerRay(ray, renderer, event.clientX, event.clientY);
    collector.reset();
    castRay(world, collector, raySettings, ray.origin, ray.direction, RAY_LENGTH);
    if (collector.hit.status !== CastRayStatus.COLLIDING) return;
    const body = collector.hit.bodyIdB;
    if (rigidBody.getMotionType(world, body) !== MotionType.DYNAMIC) return;
    vec3.scaleAndAdd(held, ray.origin, ray.direction, RAY_LENGTH * collector.hit.fraction);
    vec3.copy(planePoint, held);
    vec3.subtract(planeNormal, renderer.controls.target as Vec3, renderer.camera.position as Vec3);
    vec3.normalize(planeNormal, planeNormal);
    rigidBody.setPosition(world, anchor, held, false);
    rope = distanceConstraint.create(world, anchor, body, {
        pointA: held,
        pointB: held,
        minDistance: 0,
        maxDistance: 1,
        constraintPriority: 100,
    });
    rigidBody.wake(world, body);
    renderer.controls.enabled = false;
});
renderer.canvas.addEventListener('pointermove', (event) => {
    if (rope === null) return;
    pointerRay(ray, renderer, event.clientX, event.clientY);
    const facing = vec3.dot(ray.direction, planeNormal);
    if (Math.abs(facing) < 1e-6) return;
    const t = vec3.dot(vec3.subtract([0, 0, 0], planePoint, ray.origin), planeNormal) / facing;
    if (t <= 0) return;
    vec3.scaleAndAdd(held, ray.origin, ray.direction, t);
    rigidBody.setPosition(world, anchor, held, false);
});
window.addEventListener('pointerup', () => {
    if (rope === null) return;
    constraints.remove(world, rope);
    rope = null;
    renderer.controls.enabled = true;
});

const panel = createPanel('ragdoll');
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('three ragdolls thrown up spinning. press on a part to drag it');

const heldColor = debugColor(palette.light);

loop(
    stepTime,
    (deltaTime) => {
        updateWorld(world, listener, deltaTime);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        if (rope !== null) drawPoint(overlay, held, heldColor, 0.2, true);
        endOverlay(overlay);
        render(renderer);
    },
);
