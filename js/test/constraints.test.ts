import { describe, expect, test } from 'vitest';
import {
    Box,
    ConeConstraint,
    Constraint,
    ConstraintType,
    DistanceConstraint,
    Empty,
    FixedConstraint,
    HingeConstraint,
    INVALID_BODY_ID,
    init,
    MotionType,
    MotorState,
    PointConstraint,
    RigidBody,
    type Shape,
    SixDOFAxis,
    SixDOFConstraint,
    type SixDOFValues,
    SliderConstraint,
    Sphere,
    SpringMode,
    SwingTwistConstraint,
    type Vec3,
    World,
} from '../src';
import * as Low from '../src/generated/tumble';
import { makeScene, releaseScene, type Scene, steps } from './helpers';

type Parts = { nothing: Shape; door: Shape; ball: Shape; block: Shape };

function makeParts(): Parts {
    return {
        nothing: Empty.create(),
        door: Box.create({ halfExtents: [0.5, 1, 0.05] }),
        ball: Sphere.create({ radius: 0.25 }),
        block: Box.create({ halfExtents: [0.5, 0.1, 0.5] }),
    };
}

function releaseParts(parts: Parts): void {
    Empty.destroy(parts.nothing);
    Box.destroy(parts.door);
    Sphere.destroy(parts.ball);
    Box.destroy(parts.block);
}

/** a static body with no shape: a fixed anchor */
function anchor(scene: Scene, parts: Parts, position: Vec3): number {
    return RigidBody.create(scene.world, {
        shape: parts.nothing,
        objectLayer: scene.notMoving,
        motionType: MotionType.STATIC,
        position,
    });
}

function dynamic(scene: Scene, shape: Shape, position: Vec3): number {
    return RigidBody.create(scene.world, { shape, objectLayer: scene.moving, motionType: MotionType.DYNAMIC, position });
}

const position: Vec3 = [0, 0, 0];

function positionOf(scene: Scene, body: number): Vec3 {
    RigidBody.getPosition(position, scene.world, body);
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
        const hinge = HingeConstraint.create(world, frame, door, {
            pointA: [0, 3, 0],
            pointB: [0, 3, 0],
            limitsMin: 0,
            limitsMax: 0.5 * Math.PI,
            motorSettings: { minTorqueLimit: -100, maxTorqueLimit: 100 },
        });
        expect(Constraint.getType(hinge)).toBe(ConstraintType.HINGE);
        expect(HingeConstraint.setMotorState(world, hinge, MotorState.VELOCITY)).toBe(true);
        HingeConstraint.setTargetAngularVelocity(world, hinge, 2);
        steps(world, 90);
        expect(HingeConstraint.getCurrentAngle(world, hinge)).toBeCloseTo(0.5 * Math.PI, 1);

        expect(HingeConstraint.remove(world, hinge)).toBe(true);
        expect(Constraint.exists(world, hinge)).toBe(false);
        expect(HingeConstraint.remove(world, hinge)).toBe(false);
        expect(HingeConstraint.setTargetAngle(world, hinge, 0)).toBe(false);
        expect(HingeConstraint.getCurrentAngle(world, hinge)).toBe(0);
        releaseParts(parts);
        releaseScene(scene);
    });

    test('a pendulum swings at its length; a bungee stops at its most', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const pivot = anchor(scene, parts, [10, 5, 0]);
        const bob = dynamic(scene, parts.ball, [12, 5, 0]);
        PointConstraint.create(world, pivot, bob, { pointA: [10, 5, 0], pointB: [10, 5, 0] });
        const jumper = dynamic(scene, parts.ball, [14, 5, 0]);
        DistanceConstraint.create(world, pivot, jumper, {
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
        releaseParts(parts);
        releaseScene(scene);
    });

    test('a lift holds on its position motor, a partial spring keeping the other motor defaults', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const rail = anchor(scene, parts, [20, 0, 0]);
        const platform = dynamic(scene, parts.block, [20, 0, 0]);
        // only the spring's frequency and damping given: its mode and the unbounded force limits are the C++ defaults
        const lift = SliderConstraint.create(world, rail, platform, {
            sliderAxisA: [0, 1, 0],
            sliderAxisB: [0, 1, 0],
            normalAxisA: [1, 0, 0],
            normalAxisB: [1, 0, 0],
            autoDetectPoint: true,
            limitsMin: 0,
            limitsMax: 5,
            motorSettings: { spring: { frequencyOrStiffness: 10, damping: 1 } },
        });
        SliderConstraint.setMotorState(world, lift, MotorState.POSITION);
        SliderConstraint.setTargetPosition(world, lift, 2);
        steps(world, 240);
        expect(Math.abs(SliderConstraint.getCurrentPosition(world, lift) - 2)).toBeLessThan(0.005);
        expect(Math.abs(positionOf(scene, platform)[1] - 2)).toBeLessThan(0.005);
        releaseParts(parts);
        releaseScene(scene);
    });

    test('a weld moves as one', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const left = dynamic(scene, parts.block, [30, 3, 0]);
        const right = dynamic(scene, parts.block, [31, 3, 0]);
        FixedConstraint.create(world, left, right, { autoDetectPoint: true });
        steps(world, 60);
        const [lx, ly] = positionOf(scene, left);
        const [rx, ry] = positionOf(scene, right);
        expect(rx - lx).toBeCloseTo(1, 1);
        expect(ry - ly).toBeCloseTo(0, 1);
        expect(ly).toBeLessThan(2.5);
        releaseParts(parts);
        releaseScene(scene);
    });

    test('a shoulder and a cone stop the arm 45 degrees below the twist axis', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const torso = anchor(scene, parts, [40, 3, 0]);
        const arm = dynamic(scene, parts.ball, [40.5, 3, 0]);
        SwingTwistConstraint.create(world, torso, arm, {
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
        const cone = ConeConstraint.create(world, coneAnchor, coneArm, {
            pointA: [40, 6, 0],
            twistAxisA: [1, 0, 0],
            pointB: [40, 6, 0],
            twistAxisB: [1, 0, 0],
            halfConeAngle: 0.25 * Math.PI,
        });
        expect(ConeConstraint.getHalfConeAngle(world, cone)).toBeCloseTo(0.25 * Math.PI, 5);
        steps(world, 120);
        const [ax, ay] = positionOf(scene, arm);
        expect(Math.atan2(-(ay - 3), ax - 40)).toBeCloseTo(0.25 * Math.PI, 1);
        const [cx, cy] = positionOf(scene, coneArm);
        expect(Math.atan2(-(cy - 6), cx - 40)).toBeCloseTo(0.25 * Math.PI, 1);
        releaseParts(parts);
        releaseScene(scene);
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
        SixDOFConstraint.create(world, base, cart, { pointA: [50, 3, 0], pointB: [50, 3, 0], limitMin, limitMax });
        RigidBody.setLinearVelocity(world, cart, [5, 0, 0]);
        steps(world, 60);
        const [x, y] = positionOf(scene, cart);
        expect(Math.abs(y - 3)).toBeLessThan(0.02);
        expect(x).toBeLessThan(51.05);
        releaseParts(parts);
        releaseScene(scene);
    });

    test('a disabled constraint holds nothing; enabled again, it holds', async () => {
        const scene = await makeScene((settings) => {
            settings.gravity = [0, 0, 0];
        });
        const parts = makeParts();
        const { world } = scene;
        const pivot = anchor(scene, parts, [0, 5, 0]);
        const bob = dynamic(scene, parts.ball, [2, 5, 0]);
        const rod = PointConstraint.create(world, pivot, bob, { pointA: [0, 5, 0], pointB: [0, 5, 0] });
        expect(Constraint.isEnabled(world, rod)).toBe(true);
        expect(Constraint.setEnabled(world, rod, false)).toBe(true);
        RigidBody.setLinearVelocity(world, bob, [1, 0, 0]);
        steps(world, 60);
        expect(positionOf(scene, bob)[0]).toBeGreaterThan(2.9);
        Constraint.setEnabled(world, rod, true);
        steps(world, 60);
        const [x, y] = positionOf(scene, bob);
        expect(Math.hypot(x, y - 5)).toBeCloseTo(2, 1);
        releaseParts(parts);
        releaseScene(scene);
    });

    test('a constraint goes with its body', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const pivot = anchor(scene, parts, [0, 5, 0]);
        const bob = dynamic(scene, parts.ball, [2, 5, 0]);
        const rod = PointConstraint.create(world, pivot, bob, { pointA: [0, 5, 0], pointB: [0, 5, 0] });
        RigidBody.remove(world, bob);
        expect(Constraint.exists(world, rod)).toBe(false);
        expect(Constraint.setEnabled(world, rod, false)).toBe(false);
        releaseParts(parts);
        releaseScene(scene);
    });

    test('what the C++ holds settings to throws, as does a body id naming no body', async () => {
        const scene = await makeScene();
        const parts = makeParts();
        const { world } = scene;
        const frame = anchor(scene, parts, [0, 1, 0]);
        const door = dynamic(scene, parts.door, [0.5, 1, 0]);
        expect(() => HingeConstraint.create(world, frame, door, { limitsMin: 0.1 })).toThrow(RangeError);
        expect(() => HingeConstraint.create(world, frame, door, { limitsMin: 0, limitsMax: 0 })).toThrow(RangeError);
        expect(() => HingeConstraint.create(world, frame, door, { motorSettings: { spring: { damping: -1 } } })).toThrow(
            RangeError,
        );
        expect(() => SliderConstraint.create(world, frame, door, { limitsMax: -1 })).toThrow(RangeError);
        expect(() => DistanceConstraint.create(world, frame, door, { minDistance: 2, maxDistance: 1 })).toThrow(RangeError);
        expect(() => ConeConstraint.create(world, frame, door, { halfConeAngle: 4 })).toThrow(RangeError);
        // equal limits on a soft spring are allowed
        const soft = HingeConstraint.create(world, frame, door, {
            limitsMin: 0,
            limitsMax: 0,
            limitsSpringSettings: { frequencyOrStiffness: 5 },
        });
        expect(Constraint.exists(world, soft)).toBe(true);
        // pi as js has it is the C++'s float pi
        expect(HingeConstraint.setLimits(world, soft, -Math.PI, Math.PI)).toBe(true);
        const six = SixDOFConstraint.create(world, frame, door);
        expect(() => SixDOFConstraint.makeFixedAxis(world, six, 6 as SixDOFAxis)).toThrow(RangeError);
        expect(() =>
            SixDOFConstraint.setLimitsSpringSettings(world, six, SixDOFAxis.ROTATION_X, {
                mode: SpringMode.FREQUENCY_AND_DAMPING,
                frequencyOrStiffness: 1,
                damping: 1,
            }),
        ).toThrow(RangeError);
        expect(() => PointConstraint.create(world, frame, INVALID_BODY_ID)).toThrow('tumble: BAD_HANDLE');
        releaseParts(parts);
        releaseScene(scene);
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
            PointConstraint.create(world, pivot, bob, { pointA: [i, 5, 0], pointB: [i, 5, 0] });
            HingeConstraint.create(world, pivot, bob);
            SixDOFConstraint.create(world, pivot, bob);
        }
        steps(world, 10);
        World.destroy(world);
        Box.destroy(scene.floor);
        releaseParts(parts);
        expect(Low.allocatedBytes()).toBe(before);
    });
});
