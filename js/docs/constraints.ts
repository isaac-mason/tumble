import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    coneConstraint,
    constraints,
    createWorld,
    createWorldSettings,
    destroyWorld,
    distanceConstraint,
    emptyShape,
    enableCollision,
    fixedConstraint,
    hingeConstraint,
    init,
    MotionType,
    MotorState,
    pointConstraint,
    releaseShape,
    rigidBody,
    SixDOFAxis,
    type SixDOFValues,
    SpringMode,
    sixDOFConstraint,
    sliderConstraint,
    sphere,
    swingTwistConstraint,
    updateWorld,
    type Vec3,
} from 'tumble.js';

await init();
const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, notMoving);
const world = createWorld(settings);
const nothing = emptyShape.create();
const doorShape = box.create({ halfExtents: [0.5, 1, 0.05] });
const ballShape = sphere.create({ radius: 0.25 });
const blockShape = box.create({ halfExtents: [0.5, 0.1, 0.5] });
const steps = (count: number): void => {
    for (let i = 0; i < count; i++) updateWorld(world, undefined, 1 / 60);
};
const at: Vec3 = [0, 0, 0];
const positionOf = (body: number): Vec3 => {
    rigidBody.getPosition(at, world, body);
    return at;
};
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};
/** a static body with no shape: a fixed anchor */
const anchor = (position: Vec3): number =>
    rigidBody.create(world, { shape: nothing, objectLayer: notMoving, motionType: MotionType.STATIC, position });
const dynamic = (shape: typeof nothing, position: Vec3): number =>
    rigidBody.create(world, { shape, objectLayer: moving, motionType: MotionType.DYNAMIC, position });

/* SNIPPET_START: hinge */
// a door on a hinge along y at its left edge, opening a quarter turn at most, driven open by its motor. a motor's
// torque is unbounded unless limited, and an unbounded motor driving into a limit pushes past it: give it a limit
const frame = anchor([0, 1, 0]);
const door = dynamic(doorShape, [0.5, 1, 0]);
const hinge = hingeConstraint.create(world, frame, door, {
    pointA: [0, 1, 0],
    pointB: [0, 1, 0],
    hingeAxisA: [0, 1, 0],
    hingeAxisB: [0, 1, 0],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    limitsMin: 0,
    limitsMax: 0.5 * Math.PI,
    motorSettings: { minTorqueLimit: -100, maxTorqueLimit: 100 },
});
hingeConstraint.setMotorState(world, hinge, MotorState.VELOCITY);
hingeConstraint.setTargetAngularVelocity(world, hinge, 2);
/* SNIPPET_END: hinge */
steps(90);
check(Math.abs(hingeConstraint.getCurrentAngle(world, hinge) - 0.5 * Math.PI) < 0.05, 'opened a quarter turn');

/* SNIPPET_START: remove */
// gone, or with either body: a stale id names nothing
constraints.remove(world, hinge);
constraints.exists(world, hinge); // false
/* SNIPPET_END: remove */
check(!constraints.exists(world, hinge) && !constraints.remove(world, hinge), 'removed');
rigidBody.remove(world, door);

/* SNIPPET_START: point */
// a pendulum: a ball held 2 m from the point it swings about
const pivot = anchor([10, 5, 0]);
const bob = dynamic(ballShape, [12, 5, 0]);
pointConstraint.create(world, pivot, bob, { pointA: [10, 5, 0], pointB: [10, 5, 0] });
/* SNIPPET_END: point */
steps(90);
check(Math.abs(Math.hypot(positionOf(bob)[0] - 10, at[1] - 5) - 2) < 0.02 && at[1] < 4.9, 'swinging at 2 m');

/* SNIPPET_START: distance */
// a bungee: no further than 3 m, pulled back by a spring; minDistance and maxDistance of -1 keep where it starts
const jumper = dynamic(ballShape, [14, 5, 0]);
distanceConstraint.create(world, pivot, jumper, {
    pointA: [10, 5, 0],
    pointB: [14, 5, 0],
    minDistance: 0,
    maxDistance: 3,
    springSettings: { mode: SpringMode.FREQUENCY_AND_DAMPING, frequencyOrStiffness: 2, damping: 0.5 },
});
/* SNIPPET_END: distance */
steps(240);
check(Math.hypot(positionOf(jumper)[0] - 10, at[1] - 5) < 3.3, 'held within 3 m, give or take its spring');

/* SNIPPET_START: slider */
// a lift: a platform that slides along y only, its position motor holding it 2 m up the rail. a position motor drives
// through a spring, which a load sags: this one is stiff enough (10 Hz) to sag by millimetres
const rail = anchor([20, 0, 0]);
const platform = dynamic(blockShape, [20, 0, 0]);
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
/* SNIPPET_END: slider */
steps(240);
check(Math.abs(sliderConstraint.getCurrentPosition(world, lift) - 2) < 0.005, 'held 2 m up');

/* SNIPPET_START: fixed */
// a weld: two bodies held as they are, moving as one
const left = dynamic(blockShape, [30, 3, 0]);
const right = dynamic(blockShape, [31, 3, 0]);
fixedConstraint.create(world, left, right, { autoDetectPoint: true });
/* SNIPPET_END: fixed */
steps(60);
const leftAt: Vec3 = [...positionOf(left)];
check(Math.abs(positionOf(right)[0] - leftAt[0] - 1) < 0.01 && leftAt[1] < 2.5, 'falling as one');

/* SNIPPET_START: ragdoll */
// a shoulder: the arm swings within a 45 degree cone about the shoulder's twist axis (x here), twisting up to 30
// degrees either way. a cone constraint is the same without the twist limits
const torso = anchor([40, 3, 0]);
const arm = dynamic(ballShape, [40.5, 3, 0]);
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
/* SNIPPET_END: ragdoll */
const coneAnchor = anchor([40, 6, 0]);
const coneArm = dynamic(ballShape, [40.5, 6, 0]);
/* SNIPPET_START: cone */
coneConstraint.create(world, coneAnchor, coneArm, {
    pointA: [40, 6, 0],
    twistAxisA: [1, 0, 0],
    pointB: [40, 6, 0],
    twistAxisB: [1, 0, 0],
    halfConeAngle: 0.25 * Math.PI,
});
/* SNIPPET_END: cone */
steps(120);
// gravity pulls each arm down until its cone stops it 45 degrees below the twist axis
check(Math.abs(Math.atan2(-(positionOf(arm)[1] - 3), at[0] - 40) - 0.25 * Math.PI) < 0.05, 'the shoulder held at 45 degrees');
check(Math.abs(Math.atan2(-(positionOf(coneArm)[1] - 6), at[0] - 40) - 0.25 * Math.PI) < 0.05, 'the cone held at 45 degrees');

/* SNIPPET_START: six-dof */
// every axis its own: here free to slide along x within a metre either way and to turn about y, the rest locked
const base = anchor([50, 3, 0]);
const cart = dynamic(blockShape, [50, 3, 0]);
const limitMin: SixDOFValues = [0, 0, 0, 0, 0, 0];
const limitMax: SixDOFValues = [0, 0, 0, 0, 0, 0];
limitMin[SixDOFAxis.TRANSLATION_X] = -1;
limitMax[SixDOFAxis.TRANSLATION_X] = 1;
limitMin[SixDOFAxis.ROTATION_Y] = -Math.PI;
limitMax[SixDOFAxis.ROTATION_Y] = Math.PI;
sixDOFConstraint.create(world, base, cart, { pointA: [50, 3, 0], pointB: [50, 3, 0], limitMin, limitMax });
rigidBody.setLinearVelocity(world, cart, [5, 0, 0]);
/* SNIPPET_END: six-dof */
steps(60);
check(Math.abs(positionOf(cart)[1] - 3) < 0.02 && at[0] < 51.05, 'held up, stopped at the end of its rail');

/* SNIPPET_START: breaking */
// a joint that breaks: read the impulse it applied each step, and disable it past a strength. a disabled constraint
// holds nothing and joins nothing, until enabled again
const hook = anchor([60, 5, 0]);
const weight = rigidBody.create(world, {
    shape: blockShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [60, 4, 0],
    mass: 50,
});
const rope = pointConstraint.create(world, hook, weight, { pointA: [60, 5, 0], pointB: [60, 5, 0] });
const lambda: Vec3 = [0, 0, 0];
const STRENGTH = 15; // N s a step: 50 kg hangs at about 8
for (let i = 0; i < 120; i++) {
    if (i === 60) rigidBody.addImpulse(world, weight, [0, -2000, 0]);
    updateWorld(world, undefined, 1 / 60);
    pointConstraint.getTotalLambda(lambda, world, rope);
    if (Math.hypot(...lambda) > STRENGTH) constraints.setEnabled(world, rope, false);
    /* HIDE_START */
    if (i === 59) check(constraints.isEnabled(world, rope), 'holding the weight until the yank');
    /* HIDE_END */
}
/* SNIPPET_END: breaking */
check(!constraints.isEnabled(world, rope) && positionOf(weight)[1] < 3, 'snapped by the yank, the weight fallen');

destroyWorld(world);
releaseShape(nothing);
releaseShape(doorShape);
releaseShape(blockShape);
releaseShape(ballShape);
