import {
    Box,
    ConeConstraint,
    Constraint,
    DistanceConstraint,
    Empty,
    FixedConstraint,
    HingeConstraint,
    init,
    MotionType,
    MotorState,
    PointConstraint,
    RigidBody,
    SixDOFAxis,
    SixDOFConstraint,
    type SixDOFValues,
    SliderConstraint,
    Sphere,
    SpringMode,
    SwingTwistConstraint,
    type Vec3,
    World,
    WorldSettings,
} from 'tumble';

await init();
const settings = WorldSettings.create();
const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
const notMoving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
WorldSettings.enableCollision(settings, moving, notMoving);
const world = World.create(settings);
const nothing = Empty.create();
const doorShape = Box.create({ halfExtents: [0.5, 1, 0.05] });
const ballShape = Sphere.create({ radius: 0.25 });
const blockShape = Box.create({ halfExtents: [0.5, 0.1, 0.5] });
const steps = (count: number): void => {
    for (let i = 0; i < count; i++) World.step(world, undefined, 1 / 60);
};
const at: Vec3 = [0, 0, 0];
const positionOf = (body: number): Vec3 => {
    RigidBody.getPosition(at, world, body);
    return at;
};
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};
/** a static body with no shape: a fixed anchor */
const anchor = (position: Vec3): number =>
    RigidBody.create(world, { shape: nothing, objectLayer: notMoving, motionType: MotionType.STATIC, position });
const dynamic = (shape: typeof nothing, position: Vec3): number =>
    RigidBody.create(world, { shape, objectLayer: moving, motionType: MotionType.DYNAMIC, position });

/* SNIPPET_START: hinge */
// a door on a hinge along y at its left edge, opening a quarter turn at most, driven open by its motor. a motor's
// torque is unbounded unless limited, and an unbounded motor driving into a limit pushes past it: give it a limit
const frame = anchor([0, 1, 0]);
const door = dynamic(doorShape, [0.5, 1, 0]);
const hinge = HingeConstraint.create(world, frame, door, {
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
HingeConstraint.setMotorState(world, hinge, MotorState.VELOCITY);
HingeConstraint.setTargetAngularVelocity(world, hinge, 2);
/* SNIPPET_END: hinge */
steps(90);
check(Math.abs(HingeConstraint.getCurrentAngle(world, hinge) - 0.5 * Math.PI) < 0.05, 'opened a quarter turn');

/* SNIPPET_START: remove */
// gone, or with either body: a stale id names nothing
HingeConstraint.remove(world, hinge);
Constraint.exists(world, hinge); // false
/* SNIPPET_END: remove */
check(!Constraint.exists(world, hinge) && !HingeConstraint.remove(world, hinge), 'removed');
RigidBody.remove(world, door);

/* SNIPPET_START: point */
// a pendulum: a ball held 2 m from the point it swings about
const pivot = anchor([10, 5, 0]);
const bob = dynamic(ballShape, [12, 5, 0]);
PointConstraint.create(world, pivot, bob, { pointA: [10, 5, 0], pointB: [10, 5, 0] });
/* SNIPPET_END: point */
steps(90);
check(Math.abs(Math.hypot(positionOf(bob)[0] - 10, at[1] - 5) - 2) < 0.02 && at[1] < 4.9, 'swinging at 2 m');

/* SNIPPET_START: distance */
// a bungee: no further than 3 m, pulled back by a spring; minDistance and maxDistance of -1 keep where it starts
const jumper = dynamic(ballShape, [14, 5, 0]);
DistanceConstraint.create(world, pivot, jumper, {
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
/* SNIPPET_END: slider */
steps(240);
check(Math.abs(SliderConstraint.getCurrentPosition(world, lift) - 2) < 0.005, 'held 2 m up');

/* SNIPPET_START: fixed */
// a weld: two bodies held as they are, moving as one
const left = dynamic(blockShape, [30, 3, 0]);
const right = dynamic(blockShape, [31, 3, 0]);
FixedConstraint.create(world, left, right, { autoDetectPoint: true });
/* SNIPPET_END: fixed */
steps(60);
const leftAt: Vec3 = [...positionOf(left)];
check(Math.abs(positionOf(right)[0] - leftAt[0] - 1) < 0.01 && leftAt[1] < 2.5, 'falling as one');

/* SNIPPET_START: ragdoll */
// a shoulder: the arm swings within a 45 degree cone about the shoulder's twist axis (x here), twisting up to 30
// degrees either way. a cone constraint is the same without the twist limits
const torso = anchor([40, 3, 0]);
const arm = dynamic(ballShape, [40.5, 3, 0]);
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
/* SNIPPET_END: ragdoll */
const coneAnchor = anchor([40, 6, 0]);
const coneArm = dynamic(ballShape, [40.5, 6, 0]);
/* SNIPPET_START: cone */
ConeConstraint.create(world, coneAnchor, coneArm, {
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
SixDOFConstraint.create(world, base, cart, { pointA: [50, 3, 0], pointB: [50, 3, 0], limitMin, limitMax });
RigidBody.setLinearVelocity(world, cart, [5, 0, 0]);
/* SNIPPET_END: six-dof */
steps(60);
check(Math.abs(positionOf(cart)[1] - 3) < 0.02 && at[0] < 51.05, 'held up, stopped at the end of its rail');

/* SNIPPET_START: breaking */
// a joint that breaks: read the impulse it applied each step, and disable it past a strength. a disabled constraint
// holds nothing and joins nothing, until enabled again
const hook = anchor([60, 5, 0]);
const weight = RigidBody.create(world, {
    shape: blockShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [60, 4, 0],
    mass: 50,
});
const rope = PointConstraint.create(world, hook, weight, { pointA: [60, 5, 0], pointB: [60, 5, 0] });
const lambda: Vec3 = [0, 0, 0];
const STRENGTH = 15; // N s a step: 50 kg hangs at about 8
for (let i = 0; i < 120; i++) {
    if (i === 60) RigidBody.addImpulse(world, weight, [0, -2000, 0]);
    World.step(world, undefined, 1 / 60);
    PointConstraint.getTotalLambda(lambda, world, rope);
    if (Math.hypot(...lambda) > STRENGTH) Constraint.setEnabled(world, rope, false);
    /* HIDE_START */
    if (i === 59) check(Constraint.isEnabled(world, rope), 'holding the weight until the yank');
    /* HIDE_END */
}
/* SNIPPET_END: breaking */
check(!Constraint.isEnabled(world, rope) && positionOf(weight)[1] < 3, 'snapped by the yank, the weight fallen');

World.destroy(world);
Empty.destroy(nothing);
Box.destroy(doorShape);
Box.destroy(blockShape);
Sphere.destroy(ballShape);
