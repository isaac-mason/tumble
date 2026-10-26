import { describe, expect, test } from 'vitest';
import {
    box,
    ConstraintType,
    coneConstraint,
    constraints,
    destroyWorld,
    distanceConstraint,
    emptyShape,
    fixedConstraint,
    hingeConstraint,
    INVALID_BODY_ID,
    init,
    MotionType,
    MotorState,
    pointConstraint,
    releaseShape,
    rigidBody,
    type Shape,
    SixDOFAxis,
    type SixDOFValues,
    SpringMode,
    sixDOFConstraint,
    sliderConstraint,
    sphere,
    swingTwistConstraint,
    type Vec3,
} from '../src';
import * as Low from '../src/generated/tumble';
import { makeScene, releaseScene, type Scene, steps } from './helpers';

type Parts = { nothing: Shape; door: Shape; ball: Shape; block: Shape };

function makeParts(): Parts {
    return {
        nothing: emptyShape.create(),
        door: box.create({ halfExtents: [0.5, 1, 0.05] }),
        ball: sphere.create({ radius: 0.25 }),
        block: box.create({ halfExtents: [0.5, 0.1, 0.5] }),
    };
}

function releaseParts(parts: Parts): void {
    releaseShape(parts.nothing);
    releaseShape(parts.door);
    releaseShape(parts.ball);
    releaseShape(parts.block);
}

/** a static body with no shape: a fixed anchor */
function anchor(scene: Scene, parts: Parts, position: Vec3): number {
    return rigidBody.create(scene.world, {
        shape: parts.nothing,
        objectLayer: scene.notMoving,
        motionType: MotionType.STATIC,
        position,
    });
}

function dynamic(scene: Scene, shape: Shape, position: Vec3): number {
    return rigidBody.create(scene.world, { shape, objectLayer: scene.moving, motionType: MotionType.DYNAMIC, position });
}

const position: Vec3 = [0, 0, 0];

function positionOf(scene: Scene, body: number): Vec3 {
    rigidBody.getPosition(position, scene.world, body);
    return position;
}

describe('constraints', () => {
    test('a hinge motor opens a door to its limit; a removed hinge is gone', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        // clear of the floor, whose friction would hold it shut
        const frame = anchor(scene, parts, [0, 3, 0]);
        const door = dynamic(scene, parts.door, [0.5, 3, 0]);
        const hinge = hingeConstraint.create(world, frame, door, {
            pointA: [0, 3, 0],
            pointB: [0, 3, 0],
            limitsMin: 0,
            limitsMax: 0.5 * Math.PI,
            motorSettings: { minTorqueLimit: -100, maxTorqueLimit: 100 },
        });
        expect(constraints.getType(hinge)).toBe(ConstraintType.HINGE);
        expect(hingeConstraint.setMotorState(world, hinge, MotorState.VELOCITY)).toBe(true);
        hingeConstraint.setTargetAngularVelocity(world, hinge, 2);
        steps(world, 90);
        expect(hingeConstraint.getCurrentAngle(world, hinge)).toBeCloseTo(0.5 * Math.PI, 1);

        expect(constraints.remove(world, hinge)).toBe(true);
        expect(constraints.exists(world, hinge)).toBe(false);
        expect(constraints.remove(world, hinge)).toBe(false);
        expect(hingeConstraint.setTargetAngle(world, hinge, 0)).toBe(false);
        expect(hingeConstraint.getCurrentAngle(world, hinge)).toBe(0);
        releaseScene(scene);
        releaseParts(parts);
    });

    test('motor settings read back, and set in part over what the motor has', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const frame = anchor(scene, parts, [0, 3, 0]);
        const door = dynamic(scene, parts.door, [0.5, 3, 0]);
        const hinge = hingeConstraint.create(world, frame, door, {
            pointA: [0, 3, 0],
            pointB: [0, 3, 0],
            motorSettings: { spring: { frequencyOrStiffness: 7, damping: 0.5 } },
        });
        // only the torque given: the spring stays as created
        expect(hingeConstraint.setMotorSettings(world, hinge, { minTorqueLimit: -40, maxTorqueLimit: 40 })).toBe(true);
        const read = {
            spring: { mode: SpringMode.FREQUENCY_AND_DAMPING, frequencyOrStiffness: 0, damping: 0 },
            minForceLimit: 0,
            maxForceLimit: 0,
            minTorqueLimit: 0,
            maxTorqueLimit: 0,
        };
        expect(hingeConstraint.getMotorSettings(read, world, hinge)).toBe(true);
        expect(read.maxTorqueLimit).toBe(40);
        expect(read.minTorqueLimit).toBe(-40);
        expect(read.spring.frequencyOrStiffness).toBe(7);
        expect(read.spring.damping).toBe(0.5);
        expect(() => hingeConstraint.setMotorSettings(world, hinge, { minTorqueLimit: 50 })).toThrow(RangeError);

        constraints.remove(world, hinge);
        expect(hingeConstraint.getMotorSettings(read, world, hinge)).toBe(false);
        expect(hingeConstraint.setMotorSettings(world, hinge, { maxTorqueLimit: 1 })).toBe(false);
        releaseScene(scene);
        releaseParts(parts);
    });

    test('a pendulum swings at its length; a bungee stops at its most', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const pivot = anchor(scene, parts, [10, 5, 0]);
        const bob = dynamic(scene, parts.ball, [12, 5, 0]);
        pointConstraint.create(world, pivot, bob, { pointA: [10, 5, 0], pointB: [10, 5, 0] });
        const jumper = dynamic(scene, parts.ball, [14, 5, 0]);
        distanceConstraint.create(world, pivot, jumper, {
            pointA: [10, 5, 0],
            pointB: [14, 5, 0],
            minDistance: 0,
            maxDistance: 3,
            springSettings: { mode: SpringMode.FREQUENCY_AND_DAMPING, frequencyOrStiffness: 2, damping: 0.5 },
        });
        steps(world, 90);
        const [x, y] = positionOf(scene, bob);
        expect(Math.hypot(x - 10, y - 5)).toBeCloseTo(2, 1);
        expect(y).toBeLessThan(4.9);
        steps(world, 150);
        const [jx, jy] = positionOf(scene, jumper);
        expect(Math.hypot(jx - 10, jy - 5)).toBeLessThan(3.3);
        releaseScene(scene);
        releaseParts(parts);
    });

    test('a lift holds on its position motor, a partial spring keeping the other motor defaults', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const rail = anchor(scene, parts, [20, 0, 0]);
        const platform = dynamic(scene, parts.block, [20, 0, 0]);
        // only the spring's frequency and damping given: its mode and the unbounded force limits are the C++ defaults
        const lift = sliderConstraint.create(world, rail, platform, {
            sliderAxisA: [0, 1, 0],
            sliderAxisB: [0, 1, 0],
            normalAxisA: [1, 0, 0],
            normalAxisB: [1, 0, 0],
            autoDetectPoint: true,
            limitsMin: 0,
            limitsMax: 5,
            motorSettings: { spring: { frequencyOrStiffness: 10, damping: 1 } },
        });
        sliderConstraint.setMotorState(world, lift, MotorState.POSITION);
        sliderConstraint.setTargetPosition(world, lift, 2);
        steps(world, 240);
        expect(Math.abs(sliderConstraint.getCurrentPosition(world, lift) - 2)).toBeLessThan(0.005);
        expect(Math.abs(positionOf(scene, platform)[1] - 2)).toBeLessThan(0.005);
        releaseScene(scene);
        releaseParts(parts);
    });

    test('a weld moves as one', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const left = dynamic(scene, parts.block, [30, 3, 0]);
        const right = dynamic(scene, parts.block, [31, 3, 0]);
        fixedConstraint.create(world, left, right, { autoDetectPoint: true });
        steps(world, 60);
        const [lx, ly] = positionOf(scene, left);
        const [rx, ry] = positionOf(scene, right);
        expect(rx - lx).toBeCloseTo(1, 1);
        expect(ry - ly).toBeCloseTo(0, 1);
        expect(ly).toBeLessThan(2.5);
        releaseScene(scene);
        releaseParts(parts);
    });

    test('a shoulder and a cone stop the arm 45 degrees below the twist axis', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const torso = anchor(scene, parts, [40, 3, 0]);
        const arm = dynamic(scene, parts.ball, [40.5, 3, 0]);
        swingTwistConstraint.create(world, torso, arm, {
            pointA: [40, 3, 0],
            pointB: [40, 3, 0],
            twistAxisA: [1, 0, 0],
            planeAxisA: [0, 0, 1],
            twistAxisB: [1, 0, 0],
            planeAxisB: [0, 0, 1],
            normalHalfConeAngle: 0.25 * Math.PI,
            planeHalfConeAngle: 0.25 * Math.PI,
            twistMinAngle: -Math.PI / 6,
            twistMaxAngle: Math.PI / 6,
        });
        const coneAnchor = anchor(scene, parts, [40, 6, 0]);
        const coneArm = dynamic(scene, parts.ball, [40.5, 6, 0]);
        const cone = coneConstraint.create(world, coneAnchor, coneArm, {
            pointA: [40, 6, 0],
            twistAxisA: [1, 0, 0],
            pointB: [40, 6, 0],
            twistAxisB: [1, 0, 0],
            halfConeAngle: 0.25 * Math.PI,
        });
        expect(coneConstraint.getHalfConeAngle(world, cone)).toBeCloseTo(0.25 * Math.PI, 5);
        steps(world, 120);
        const [ax, ay] = positionOf(scene, arm);
        expect(Math.atan2(-(ay - 3), ax - 40)).toBeCloseTo(0.25 * Math.PI, 1);
        const [cx, cy] = positionOf(scene, coneArm);
        expect(Math.atan2(-(cy - 6), cx - 40)).toBeCloseTo(0.25 * Math.PI, 1);
        releaseScene(scene);
        releaseParts(parts);
    });

    test('a six dof cart slides along x within its rail, held up against gravity', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const base = anchor(scene, parts, [50, 3, 0]);
        const cart = dynamic(scene, parts.block, [50, 3, 0]);
        const limitMin: SixDOFValues = [0, 0, 0, 0, 0, 0];
        const limitMax: SixDOFValues = [0, 0, 0, 0, 0, 0];
        limitMin[SixDOFAxis.TRANSLATION_X] = -1;
        limitMax[SixDOFAxis.TRANSLATION_X] = 1;
        limitMin[SixDOFAxis.ROTATION_Y] = -Math.PI;
        limitMax[SixDOFAxis.ROTATION_Y] = Math.PI;
        sixDOFConstraint.create(world, base, cart, { pointA: [50, 3, 0], pointB: [50, 3, 0], limitMin, limitMax });
        rigidBody.setLinearVelocity(world, cart, [5, 0, 0]);
        steps(world, 60);
        const [x, y] = positionOf(scene, cart);
        expect(Math.abs(y - 3)).toBeLessThan(0.02);
        expect(x).toBeLessThan(51.05);
        releaseScene(scene);
        releaseParts(parts);
    });

    test('a disabled constraint holds nothing; enabled again, it holds', async () => {
        const scene = await makeScene((settings) => {
            settings.gravity = [0, 0, 0];
        });
        const parts = makeParts();
        const { world } = scene;
        const pivot = anchor(scene, parts, [0, 5, 0]);
        const bob = dynamic(scene, parts.ball, [2, 5, 0]);
        const rod = pointConstraint.create(world, pivot, bob, { pointA: [0, 5, 0], pointB: [0, 5, 0] });
        expect(constraints.isEnabled(world, rod)).toBe(true);
        expect(constraints.setEnabled(world, rod, false)).toBe(true);
        rigidBody.setLinearVelocity(world, bob, [1, 0, 0]);
        steps(world, 60);
        expect(positionOf(scene, bob)[0]).toBeGreaterThan(2.9);
        constraints.setEnabled(world, rod, true);
        steps(world, 60);
        const [x, y] = positionOf(scene, bob);
        expect(Math.hypot(x, y - 5)).toBeCloseTo(2, 1);
        releaseScene(scene);
        releaseParts(parts);
    });

    test('a constraint goes with its body', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const pivot = anchor(scene, parts, [0, 5, 0]);
        const bob = dynamic(scene, parts.ball, [2, 5, 0]);
        const rod = pointConstraint.create(world, pivot, bob, { pointA: [0, 5, 0], pointB: [0, 5, 0] });
        rigidBody.remove(world, bob);
        expect(constraints.exists(world, rod)).toBe(false);
        expect(constraints.setEnabled(world, rod, false)).toBe(false);
        releaseScene(scene);
        releaseParts(parts);
    });

    test('what the C++ holds settings to throws, as does a body id naming no body', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const frame = anchor(scene, parts, [0, 1, 0]);
        const door = dynamic(scene, parts.door, [0.5, 1, 0]);
        expect(() => hingeConstraint.create(world, frame, door, { limitsMin: 0.1 })).toThrow(RangeError);
        expect(() => hingeConstraint.create(world, frame, door, { limitsMin: 0, limitsMax: 0 })).toThrow(RangeError);
        expect(() => hingeConstraint.create(world, frame, door, { motorSettings: { spring: { damping: -1 } } })).toThrow(
            RangeError,
        );
        expect(() => sliderConstraint.create(world, frame, door, { limitsMax: -1 })).toThrow(RangeError);
        expect(() => distanceConstraint.create(world, frame, door, { minDistance: 2, maxDistance: 1 })).toThrow(RangeError);
        expect(() => coneConstraint.create(world, frame, door, { halfConeAngle: 4 })).toThrow(RangeError);
        // a priority is jolt's uint32, which a negative or fractional one would not round trip through
        expect(() => pointConstraint.create(world, frame, door, { constraintPriority: -1 })).toThrow(RangeError);
        expect(() => sixDOFConstraint.create(world, frame, door, { constraintPriority: 1.5 })).toThrow(RangeError);
        expect(() => fixedConstraint.create(world, frame, door, { constraintPriority: 2 ** 32 })).toThrow(RangeError);
        const highest = fixedConstraint.create(world, frame, door, { constraintPriority: 0xffffffff });
        expect(constraints.exists(world, highest)).toBe(true);
        // equal limits on a soft spring are allowed
        const soft = hingeConstraint.create(world, frame, door, {
            limitsMin: 0,
            limitsMax: 0,
            limitsSpringSettings: { frequencyOrStiffness: 5 },
        });
        expect(constraints.exists(world, soft)).toBe(true);
        // pi as js has it is the C++'s float pi
        expect(hingeConstraint.setLimits(world, soft, -Math.PI, Math.PI)).toBe(true);
        const six = sixDOFConstraint.create(world, frame, door);
        expect(() => sixDOFConstraint.makeFixedAxis(world, six, 6 as SixDOFAxis)).toThrow(RangeError);
        expect(() =>
            sixDOFConstraint.setLimitsSpringSettings(world, six, SixDOFAxis.ROTATION_X, {
                mode: SpringMode.FREQUENCY_AND_DAMPING,
                frequencyOrStiffness: 1,
                damping: 1,
            }),
        ).toThrow(RangeError);
        expect(() => pointConstraint.create(world, frame, INVALID_BODY_ID)).toThrow('tumble: BAD_HANDLE');
        releaseScene(scene);
        releaseParts(parts);
    });

    test('a world with constraints, destroyed, leaves the heap as it was', async () => {
        await init();
        const before = Low.allocatedBytes();
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const pivot = anchor(scene, parts, [0, 5, 0]);
        for (let i = 0; i < 40; i++) {
            const bob = dynamic(scene, parts.ball, [i, 4, 0]);
            pointConstraint.create(world, pivot, bob, { pointA: [i, 5, 0], pointB: [i, 5, 0] });
            hingeConstraint.create(world, pivot, bob);
            sixDOFConstraint.create(world, pivot, bob);
        }
        steps(world, 10);
        destroyWorld(world);
        releaseShape(scene.floor);
        releaseParts(parts);
        expect(Low.allocatedBytes()).toBe(before);
    });
});
