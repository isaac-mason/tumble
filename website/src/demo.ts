import { type Quat, quat, type Vec3, vec3 } from 'math';
import { mulberry32 } from 'math/random';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    createWorld,
    createWorldSettings,
    emptyShape,
    enableCollision,
    type Listener,
    ConstraintSpace,
    MotionType,
    pointConstraint,
    releaseShape,
    rigidBody,
    type Shape,
    updateWorld,
    type World,
} from 'tumble.js';
import { type Builder, buildRoom, type Material, ROOM_KINDS, ROOM_LENGTH, type RoomKind, type RoomSize, roomSize } from './rooms';
import { RUNNER_DARK, RUNNER_ORANGE, RUNNER_TRIM, rgb } from './theme';

// the landing page's world, without drawing it: a boxy runner that never stops, running on through room after room of
// things to knock over. the runner is kinematic, so nothing stops it. rooms are built ahead of it and taken away
// behind, and the world is moved back toward its origin now and then, so its numbers stay small however far it runs

export type Primitive = 'box' | 'cylinder' | 'sphere';

/**
 * a primitive in a colour, placed in its body's frame. size is the box's half extents, the cylinder's radius, half
 * height and radius again, or the sphere's radius three times
 */
export type Look = { primitive: Primitive; size: Vec3; offset: Vec3; rotation: Quat; color: [number, number, number] };

/**
 * a body and how it looks, all in the body's own frame, and whether it thins out near the camera so it never hides the
 * runner. a scale, kept up to date by the world, stretches how it is drawn but not the body
 */
export type Drawable = { body: BodyId; looks: Look[]; fades: boolean; scale?: Vec3 };

/** what the drawing must catch up with since it last looked: bodies come and go, and the world moves back */
export type Change = { type: 'add'; drawable: Drawable } | { type: 'remove'; body: BodyId } | { type: 'rebase'; by: Vec3 };

export const STEP = 1 / 60;

const GRAVITY = -14;
/** the runner's speed toward -z */
export const RUN_SPEED = 8;
/** how far the runner keeps from a room's walls, and how far either side of a doorway it is eased into the narrower room's reach */
const REACH_MARGIN = 1.05;
const REACH_BLEND = 8;
/** how much of the way to where it is steered the runner goes each second */
const STEER_FOLLOW = 4;
/** rooms built ahead of the runner, into the fog, and kept behind it until they are behind the camera */
const ROOMS_AHEAD = 4;
const ROOMS_BEHIND = 1;
/** a prop this far behind the runner is past the camera, and taken away */
const PROP_BEHIND = 14;
/** props made each step from those a new room is waiting for */
const PENDING_PER_STEP = 120;
/** how often, in steps, props are looked over for any left behind */
const SWEEP_EVERY = 15;
/** past this far from the origin, the world is moved back by it: a whole number of rooms */
const REBASE_DISTANCE = 10 * ROOM_LENGTH;

/* the runner: boxes posed each step by a run cycle, all kinematic */

const SCALE = 0.8;
const RUNNER_COLOR = RUNNER_ORANGE;
const RUNNER_ACCENT = RUNNER_DARK;
const SCARF_COLOR = RUNNER_TRIM;

const PELVIS = 0;
const TORSO = 1;
const HEAD = 2;
const LEFT_UPPER_ARM = 3;
const LEFT_LOWER_ARM = 4;
const RIGHT_UPPER_ARM = 5;
const RIGHT_LOWER_ARM = 6;
const LEFT_THIGH = 7;
const LEFT_SHIN = 8;
const LEFT_FOOT = 9;
const RIGHT_THIGH = 10;
const RIGHT_SHIN = 11;
const RIGHT_FOOT = 12;

const PART_HALF_EXTENTS: Vec3[] = [
    [0.24, 0.11, 0.15],
    [0.3, 0.3, 0.19],
    [0.25, 0.24, 0.24],
    [0.08, 0.15, 0.08],
    [0.09, 0.14, 0.09],
    [0.08, 0.15, 0.08],
    [0.09, 0.14, 0.09],
    [0.1, 0.19, 0.1],
    [0.09, 0.19, 0.09],
    [0.1, 0.065, 0.17],
    [0.1, 0.19, 0.1],
    [0.09, 0.19, 0.09],
    [0.1, 0.065, 0.17],
].map((halfExtents) => vec3.scale([0, 0, 0], halfExtents as Vec3, SCALE));
const PART_COLORS = [
    RUNNER_ACCENT,
    RUNNER_COLOR,
    RUNNER_COLOR,
    RUNNER_COLOR,
    RUNNER_COLOR,
    RUNNER_COLOR,
    RUNNER_COLOR,
    RUNNER_ACCENT,
    RUNNER_COLOR,
    RUNNER_ACCENT,
    RUNNER_ACCENT,
    RUNNER_COLOR,
    RUNNER_ACCENT,
];
const HIP_HEIGHT =
    2 * PART_HALF_EXTENTS[LEFT_THIGH]![1] + 2 * PART_HALF_EXTENTS[LEFT_SHIN]![1] + 2 * PART_HALF_EXTENTS[LEFT_FOOT]![1];
/** strides a second, two steps each, drifting a little either way over time */
const CADENCE = 1.6;
const CADENCE_DRIFT = 0.04;
const THIGH_LENGTH = 2 * PART_HALF_EXTENTS[LEFT_THIGH]![1];
const SHIN_LENGTH = 2 * PART_HALF_EXTENTS[LEFT_SHIN]![1];
/** how high the ankle is over the floor with the foot flat */
const ANKLE_HEIGHT = 2 * PART_HALF_EXTENTS[LEFT_FOOT]![1];
/** the thighs swing either way of a little forward of straight down, and the knee folds the heel up behind as the leg comes through */
const THIGH_SWING = 0.72;
const THIGH_FORWARD = 0.1;
const KNEE_EASE = 0.25;
const KNEE_FOLD = 1.9;
/** how far over the floor a foot keeps its ankle coming through, and how high it is before its toe points down */
const SWING_CLEARANCE = 0.05;
const TOE_CLEARANCE = 0.2;
/** how high the hips ride: low as each foot takes the weight, floating up between steps with both feet off the floor */
const HIP_RIDE = 0.68;
const HIP_BOUNCE = 0.045;
/** how far through a stride before the hips are lowest a foot lands */
const LANDING = 0.08;
/**
 * the hips turn a little with the thighs, the legs still swinging straight along the way it heads, and drop on the side
 * whose foot is in the air, and sway over the foot on the floor
 */
const HIP_TURN = 0.15;
const HIP_DROP = 0.07;
const HIP_SWAY = 0.03;
/** the feet land near the middle, not under the hips */
const FOOT_IN = 0.06;
/** how far it leans into a turn, for each unit of its sideways speed over its running speed */
const BANK = 0.9;
const LEAN = 0.28;
/** the shoulders turn against the hips */
const SHOULDER_TURN = 2;
const ARM_SWING = 0.95;
const ELBOW_BEND = 1.5;
/** how far behind what drives it each part follows through, in radians of the step: the torso after the hips, the head after the torso, the forearm after the arm */
const FOLLOW = 0.55;
/** the torso stretches as the hips float up and squashes as a foot lands */
const STRETCH = 0.045;
const SQUASH = 0.7;
/** each footstrike nods the torso forward a touch */
const STRIKE = 2.7;

/**
 * a scarf: a chain of thin boxes hung from the back of the neck on ball joints, moved only by the world, so it streams
 * behind and whips about. it collides with nothing
 */
const SCARF_SEGMENTS = 6;
const SCARF_HALF_EXTENTS: Vec3 = [0.07 * SCALE, 0.012 * SCALE, 0.09 * SCALE];
const SCARF_DRAG = 2.5;

/**
 * the pose shaken by springs, their values added to its joints: hits lean it back, and every footstrike jolts it. the
 * runner never slows, so this is all for show
 */
const LEAN_BACK_FREQUENCY = 2 * Math.PI * 2.6;
const LEAN_BACK_DAMPING = 0.3;
const STRIKE_FREQUENCY = 2 * Math.PI * 5;
const STRIKE_DAMPING = 0.45;

type Spring = { value: number; velocity: number; frequency: number; damping: number };

type Shake = {
    torsoPitch: Spring;
    torsoRoll: Spring;
    headPitch: Spring;
    dip: Spring;
    strike: Spring;
};

type Runner = {
    parts: BodyId[];
    /** where its pelvis is over the floor, in the world as it is now */
    x: number;
    z: number;
    /** where the pointer steers it across, from -1 at the left of the room's reach to 1 at the right */
    steer: number;
    sideways: number;
    /** how far through its stride, from 0 as the left foot lands */
    stride: number;
    time: number;
    scarf: BodyId[];
    shake: Shake;
    /** the hardest hit in the last step, which kicks the springs once: how hard, and the push on the runner */
    hit: { strength: number; push: Vec3 };
    positions: Vec3[];
    quaternions: Quat[];
    /** how each part is drawn stretched, the drawn box only */
    scales: Vec3[];
};

type Room = { index: number; kind: RoomKind; size: RoomSize; bodies: BodyId[] };
type Pending = { shape: Shape; position: Vec3; quaternion: Quat; looks: Look[]; material: Material };

export type Demo = {
    world: World;
    listener: Listener;
    moving: number;
    notMoving: number;
    /** the drawables there are now */
    drawables: Map<BodyId, Drawable>;
    /** since the drawing last took them */
    changes: Change[];
    /** where the world's origin is along the run, so a z plus it is along the whole run */
    origin: number;
    rooms: Map<number, Room>;
    /** every moving body but the runner's */
    props: BodyId[];
    /** props a room is waiting for, made a few each step */
    pending: Pending[];
    runner: Runner;
    steps: number;
    /** where the camera follows: the runner's pelvis */
    focus: Vec3;
};

const NO_ROTATION: Quat = [0, 0, 0, 1];
const _position: Vec3 = [0, 0, 0];
const _push: Vec3 = [0, 0, 0];

function addDrawable(demo: Demo, drawable: Drawable): void {
    demo.drawables.set(drawable.body, drawable);
    demo.changes.push({ type: 'add', drawable });
}

function removeBody(demo: Demo, body: BodyId): void {
    rigidBody.remove(demo.world, body);
    if (demo.drawables.delete(body)) demo.changes.push({ type: 'remove', body });
}

export function createDemo(): Demo {
    const settings = createWorldSettings();
    const movingBroadphase = addBroadphaseLayer(settings);
    const moving = addObjectLayer(settings, movingBroadphase);
    const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
    // for what hangs off the runner and touches nothing
    const trim = addObjectLayer(settings, movingBroadphase);
    enableCollision(settings, moving, moving);
    enableCollision(settings, moving, notMoving);
    settings.gravity = [0, GRAVITY, 0];
    const world = createWorld(settings);

    const runner: Runner = {
        parts: [],
        x: 0,
        z: -2,
        steer: 0,
        sideways: 0,
        stride: 0,
        time: 0,
        scarf: [],
        shake: {
            torsoPitch: spring(LEAN_BACK_FREQUENCY, LEAN_BACK_DAMPING),
            torsoRoll: spring(LEAN_BACK_FREQUENCY, LEAN_BACK_DAMPING),
            headPitch: spring(LEAN_BACK_FREQUENCY, LEAN_BACK_DAMPING),
            dip: spring(LEAN_BACK_FREQUENCY, LEAN_BACK_DAMPING),
            strike: spring(STRIKE_FREQUENCY, STRIKE_DAMPING),
        },
        hit: { strength: 0, push: [0, 0, 0] },
        positions: PART_HALF_EXTENTS.map(() => [0, 0, 0] as Vec3),
        quaternions: PART_HALF_EXTENTS.map(() => [0, 0, 0, 1] as Quat),
        scales: PART_HALF_EXTENTS.map(() => [1, 1, 1] as Vec3),
    };
    const demo: Demo = {
        world,
        listener: {
            // a hit on the runner, its parts known by their user data, leans it back
            onContactAdded: (contact) => {
                const runnerIsA = contact.userDataA > 0;
                if (!runnerIsA && contact.userDataB === 0) return;
                const other = runnerIsA ? contact.bodyIdB : contact.bodyIdA;
                const sign = runnerIsA ? -1 : 1;
                _push[0] = contact.normal[0] * sign;
                _push[1] = contact.normal[1] * sign;
                _push[2] = contact.normal[2] * sign;
                hitRunner(runner, rigidBody.getMass(world, other as BodyId), _push);
            },
        },
        moving,
        notMoving,
        drawables: new Map(),
        changes: [],
        origin: 0,
        rooms: new Map(),
        props: [],
        pending: [],
        runner,
        steps: 0,
        focus: [0, HIP_HEIGHT, runner.z],
    };

    poseRunner(runner);
    for (let i = 0; i < PART_HALF_EXTENTS.length; i++) {
        const halfExtents = PART_HALF_EXTENTS[i]!;
        const shape = box.create({ halfExtents });
        const body = rigidBody.create(world, {
            shape,
            objectLayer: moving,
            motionType: MotionType.KINEMATIC,
            userData: i + 1,
            position: runner.positions[i],
            quaternion: runner.quaternions[i],
        });
        releaseShape(shape);
        runner.parts.push(body);
        const looks: Look[] = [
            { primitive: 'box', size: halfExtents, offset: [0, 0, 0], rotation: NO_ROTATION, color: rgb(PART_COLORS[i]!) },
        ];
        // the scarf wrapped round the neck
        if (i === TORSO)
            looks.push({
                primitive: 'box',
                size: [halfExtents[0] * 0.75, 0.06 * SCALE, halfExtents[2] * 1.1],
                offset: [0, halfExtents[1] - 0.03 * SCALE, 0],
                rotation: NO_ROTATION,
                color: rgb(SCARF_COLOR),
            });
        addDrawable(demo, { body, looks, fades: false, scale: runner.scales[i] });
    }
    createScarf(demo, trim);

    updateRooms(demo);
    createPending(demo, demo.pending.length);
    return demo;
}

/** the scarf's segments, each hung by its front from the back of the last, the first from the back of the neck */
function createScarf(demo: Demo, layer: number): void {
    const runner = demo.runner;
    const torso = runner.parts[TORSO]!;
    const [, torsoHalfY, torsoHalfZ] = PART_HALF_EXTENTS[TORSO]!;
    const length = SCARF_HALF_EXTENTS[2];
    const neck: Vec3 = [0, torsoHalfY - 0.05 * SCALE, torsoHalfZ];
    const shape = box.create({ halfExtents: SCARF_HALF_EXTENTS, density: 300 });
    let previous = torso;
    let hook = neck;
    for (let i = 0; i < SCARF_SEGMENTS; i++) {
        const local: Vec3 = [neck[0], neck[1], neck[2] + length + i * 2 * length];
        const position = vec3.add(
            [0, 0, 0],
            runner.positions[TORSO]!,
            vec3.transformQuat([0, 0, 0], local, runner.quaternions[TORSO]!),
        );
        const body = rigidBody.create(demo.world, {
            shape,
            objectLayer: layer,
            motionType: MotionType.DYNAMIC,
            position,
            quaternion: runner.quaternions[TORSO],
            linearDamping: SCARF_DRAG,
            angularDamping: SCARF_DRAG,
            gravityFactor: 0.8,
            allowSleeping: false,
        });
        pointConstraint.create(demo.world, previous, body, {
            space: ConstraintSpace.LOCAL,
            pointA: hook,
            pointB: [0, 0, -length],
        });
        addDrawable(demo, {
            body,
            looks: [
                { primitive: 'box', size: SCARF_HALF_EXTENTS, offset: [0, 0, 0], rotation: NO_ROTATION, color: rgb(SCARF_COLOR) },
            ],
            fades: false,
        });
        runner.scarf.push(body);
        previous = body;
        hook = [0, 0, length];
    }
    releaseShape(shape);
}

/* the run cycle */

const _rotation: Quat = [0, 0, 0, 1];
const _turn: Quat = [0, 0, 0, 1];
const _joint: Vec3 = [0, 0, 0];
const X_AXIS: Vec3 = [1, 0, 0];
const Y_AXIS: Vec3 = [0, 1, 0];
const Z_AXIS: Vec3 = [0, 0, 1];

/**
 * a part hung from a joint on its parent, turned about x by the angle and then by the extra turn: the joint is in the
 * parent's frame, and the part's middle is reach from the joint in its own frame
 */
function hang(runner: Runner, part: number, parent: number, joint: Vec3, angle: number, reach: Vec3, extra: Quat = NO_ROTATION) {
    const parentPosition = runner.positions[parent]!;
    const parentRotation = runner.quaternions[parent]!;
    const rotation = runner.quaternions[part]!;
    quat.multiply(rotation, parentRotation, quat.setAxisAngle(_turn, X_AXIS, angle));
    quat.multiply(rotation, rotation, extra);
    vec3.add(_joint, parentPosition, vec3.transformQuat(_joint, joint, parentRotation));
    vec3.add(runner.positions[part]!, _joint, vec3.transformQuat([0, 0, 0], reach, rotation));
}

function spring(frequency: number, damping: number): Spring {
    return { value: 0, velocity: 0, frequency, damping };
}

function stepSpring(spring: Spring): void {
    const { frequency, damping } = spring;
    const acceleration = -frequency * frequency * spring.value - 2 * damping * frequency * spring.velocity;
    spring.velocity += acceleration * STEP;
    spring.value += spring.velocity * STEP;
}

function clamp(value: number, limit: number): number {
    return Math.max(-limit, Math.min(limit, value));
}

/**
 * the knee's angle, forward positive, that puts the ankle a drop under the hip with the thigh at its angle: bent just
 * enough, never straighter than a touch bent, and short of the floor where it cannot reach
 */
function kneeToReach(thigh: number, drop: number): number {
    const shin = -Math.acos(clamp((drop - THIGH_LENGTH * Math.cos(thigh)) / SHIN_LENGTH, 1));
    return Math.min(shin - thigh, -0.1);
}

const _splay: Quat = [0, 0, 0, 1];
const _twist: Quat = [0, 0, 0, 1];
const _extra: Quat = [0, 0, 0, 1];
/** each leg's thigh and knee angles, how much of the weight it takes, and how high its ankle is over the floor */
const _legs: [number, number, number, number][] = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
];

/** every part's pose for where the runner is, how far through its stride, and how shaken */
function poseRunner(runner: Runner): void {
    const half = PART_HALF_EXTENTS;
    const shake = runner.shake;

    // the hips ride lowest as each foot takes the weight, half a stride apart, and float up between
    const phase = 2 * Math.PI * runner.stride;
    const rise = -Math.cos(2 * phase);
    const hipY = HIP_RIDE + HIP_BOUNCE * rise - Math.min(0.1, Math.max(0, shake.dip.value)) - 0.03 * shake.strike.value;

    // each thigh swings on a sine, the right half a stride on from the left. a leg swinging back takes the weight,
    // bending at the knee just enough to keep its foot on the floor; coming through, it folds the heel up behind
    let roll = 0;
    let sway = 0;
    for (let leg = 0; leg < 2; leg++) {
        const side = leg === 0 ? -1 : 1;
        const legPhase = phase + leg * Math.PI;
        const thigh = THIGH_FORWARD + THIGH_SWING * Math.sin(legPhase);
        const drop = hipY - ANKLE_HEIGHT;
        let folded = -(KNEE_EASE + KNEE_FOLD * Math.max(0, Math.cos(legPhase + 0.4)) ** 1.2);
        // coming through, the foot folds up out of the way of the floor
        if (THIGH_LENGTH * Math.cos(thigh) + SHIN_LENGTH * Math.cos(thigh + folded) > drop - SWING_CLEARANCE)
            folded = kneeToReach(thigh, drop - SWING_CLEARANCE);
        const weight = smoothstep(-Math.cos(legPhase) / 0.25);
        const knee = folded + (kneeToReach(thigh, drop) - folded) * weight;
        const parts = _legs[leg]!;
        parts[0] = thigh;
        parts[1] = knee;
        parts[2] = weight;
        parts[3] = drop - THIGH_LENGTH * Math.cos(thigh) - SHIN_LENGTH * Math.cos(thigh + knee);
        const load = Math.max(0, -Math.cos(legPhase));
        roll += side * HIP_DROP * load;
        sway += side * HIP_SWAY * load;
    }
    const yaw = (HIP_TURN * (_legs[1]![0] - _legs[0]![0])) / 2;

    // turned toward where it is heading, the whole body leaning into the turn
    const heading = -Math.atan2(runner.sideways, RUN_SPEED);
    const bank = clamp((-BANK * runner.sideways) / RUN_SPEED, 0.45);
    const pelvisRotation = runner.quaternions[PELVIS]!;
    quat.setAxisAngle(pelvisRotation, Y_AXIS, heading);
    quat.multiply(pelvisRotation, pelvisRotation, quat.setAxisAngle(_rotation, Z_AXIS, bank + roll));
    vec3.set(runner.positions[PELVIS]!, runner.x + sway, hipY + half[PELVIS]![1], runner.z);

    // the legs hang from the hips before they turn, so they swing straight along the way it heads
    for (let leg = 0; leg < 2; leg++) {
        const side = leg === 0 ? -1 : 1;
        const [thighAngle, kneeAngle, weight, lift] = _legs[leg]!;

        // the thigh turned in a little so the foot lands near the middle; the foot flat on the floor, and pointed in the air
        const thigh = side < 0 ? LEFT_THIGH : RIGHT_THIGH;
        const shin = side < 0 ? LEFT_SHIN : RIGHT_SHIN;
        const foot = side < 0 ? LEFT_FOOT : RIGHT_FOOT;
        const footAngle = -(thighAngle + kneeAngle) - 0.7 * (1 - weight) ** 2 * smoothstep(lift / TOE_CLEARANCE);
        quat.setAxisAngle(_splay, Z_AXIS, -side * Math.atan2(FOOT_IN, THIGH_LENGTH + SHIN_LENGTH));
        hang(
            runner,
            thigh,
            PELVIS,
            [side * half[PELVIS]![0] * 0.55, -half[PELVIS]![1], 0],
            thighAngle,
            [0, -half[thigh]![1], 0],
            _splay,
        );
        hang(runner, shin, thigh, [0, -half[thigh]![1], 0], kneeAngle, [0, -half[shin]![1], 0]);
        hang(runner, foot, shin, [0, -half[shin]![1], 0], footAngle, [0, -half[foot]![1], -half[foot]![2] * 0.5]);
    }
    quat.multiply(pelvisRotation, quat.setAxisAngle(_rotation, Y_AXIS, yaw), pelvisRotation);

    // the torso follows the hips' bob a little late, nodding as they drop, stretching as they float and squashing as
    // a foot lands; it turns against the hips and keeps the shoulders level. the head follows later still, and keeps
    // looking ahead
    const torsoRise = -Math.cos(2 * phase - FOLLOW);
    const headRise = -Math.cos(2 * phase - 2 * FOLLOW);
    const stretch = Math.max(0.85, 1 + STRETCH * torsoRise - SQUASH * Math.max(0, shake.strike.value));
    const torsoScale = runner.scales[TORSO]!;
    vec3.set(torsoScale, 1 / Math.sqrt(stretch), stretch, 1 / Math.sqrt(stretch));
    const headStretch = 1 + (stretch - 1) * 0.5;
    vec3.set(runner.scales[HEAD]!, 1 / Math.sqrt(headStretch), headStretch, 1 / Math.sqrt(headStretch));
    const torsoHalfY = half[TORSO]![1] * stretch;

    const twist = -SHOULDER_TURN * yaw;
    quat.setAxisAngle(_twist, Y_AXIS, twist);
    quat.multiply(_extra, _twist, quat.setAxisAngle(_rotation, Z_AXIS, clamp(shake.torsoRoll.value, 0.4) - 0.6 * roll));
    const torsoPitch = -LEAN + 0.06 * torsoRise - shake.strike.value + clamp(shake.torsoPitch.value, 0.5);
    hang(runner, TORSO, PELVIS, [0, half[PELVIS]![1], 0], torsoPitch, [0, torsoHalfY, 0], _extra);
    quat.setAxisAngle(_extra, Y_AXIS, -(yaw + twist));
    const headPitch = LEAN * 0.85 - 0.1 * headRise + 0.7 * shake.strike.value + clamp(shake.headPitch.value, 0.5);
    hang(runner, HEAD, TORSO, [0, torsoHalfY, 0], headPitch, [0, half[HEAD]![1] * headStretch + 0.04, 0], _extra);

    for (let leg = 0; leg < 2; leg++) {
        const side = leg === 0 ? -1 : 1;
        const legPhase = phase + leg * Math.PI;

        // the arm against its leg, a little behind it, and the forearm behind the arm: the elbow closing as it comes
        // forward, the hand crossing toward the middle in front
        const upper = side < 0 ? LEFT_UPPER_ARM : RIGHT_UPPER_ARM;
        const lower = side < 0 ? LEFT_LOWER_ARM : RIGHT_LOWER_ARM;
        const forward = -Math.sin(legPhase - FOLLOW * 0.5);
        const forearm = -Math.sin(legPhase - FOLLOW * 1.5);
        const shoulder: Vec3 = [side * (half[TORSO]![0] + half[upper]![0]), torsoHalfY - half[upper]![0], 0];
        quat.setAxisAngle(_splay, Z_AXIS, side * (0.12 - 0.14 * Math.max(0, forward)));
        hang(runner, upper, TORSO, shoulder, 0.05 + ARM_SWING * forward, [0, -half[upper]![1], 0], _splay);
        hang(runner, lower, upper, [0, -half[upper]![1], 0], ELBOW_BEND + 0.35 * forearm, [0, -half[lower]![1], 0]);
    }
}

function stepRunner(demo: Demo): void {
    const runner = demo.runner;
    const before = runner.x;
    const reach = reachAt(demo, runner.z);
    runner.x += (runner.steer * reach - runner.x) * (1 - Math.exp(-STEER_FOLLOW * STEP));
    runner.x = clamp(runner.x, reach);
    runner.sideways = (runner.x - before) / STEP;
    runner.z -= RUN_SPEED * STEP;
    runner.time += STEP;
    const stride = runner.stride;
    runner.stride = (stride + CADENCE * (1 + CADENCE_DRIFT * Math.sin(runner.time * 0.7)) * STEP) % 1;
    // a foot lands every half stride, a little before the hips are lowest
    if (Math.floor((stride + LANDING) * 2) !== Math.floor((runner.stride + LANDING) * 2)) runner.shake.strike.velocity += STRIKE;
    leanBack(runner);
    for (const value of Object.values(runner.shake)) stepSpring(value);
    poseRunner(runner);
    for (let i = 0; i < runner.parts.length; i++)
        rigidBody.moveKinematic(demo.world, runner.parts[i]!, runner.positions[i]!, runner.quaternions[i]!, STEP);
}

/** a hit on the runner by a body of the mass given, pushing the runner along the direction, kept if it is the hardest this step */
function hitRunner(runner: Runner, mass: number, push: Vec3): void {
    const strength = Math.min(1, Math.sqrt(mass) / 6);
    if (strength <= runner.hit.strength) return;
    runner.hit.strength = strength;
    vec3.copy(runner.hit.push, push);
}

/** the springs kicked by the hardest hit of the last step: leaned back from the front, over from the side */
function leanBack(runner: Runner): void {
    const { strength, push } = runner.hit;
    if (strength === 0) return;
    const shake = runner.shake;
    shake.torsoPitch.velocity += strength * 3 * Math.max(0, push[2]);
    shake.torsoRoll.velocity -= strength * 2 * push[0];
    shake.headPitch.velocity += strength * 3 * Math.max(0, push[2]);
    shake.dip.velocity += strength * 0.3;
    runner.hit.strength = 0;
}

/** where the runner heads across, from -1 at the left to 1 at the right */
export function steer(demo: Demo, across: number): void {
    demo.runner.steer = clamp(across, 1);
}

function smoothstep(t: number): number {
    const clamped = Math.max(0, Math.min(1, t));
    return clamped * clamped * (3 - 2 * clamped);
}

/**
 * how far either side of the middle there is room to run at a z: inside the room's walls, and through a doorway no
 * wider than the narrower room either side of it, eased into either side
 */
export function reachAt(demo: Demo, z: number): number {
    const distance = -(z + demo.origin);
    const index = Math.max(0, Math.floor(distance / ROOM_LENGTH));
    const into = distance - index * ROOM_LENGTH;
    const own = clearOf(sizeAt(index));
    let reach = own;
    if (index > 0 && into < REACH_BLEND) {
        const door = Math.min(own, clearOf(sizeAt(index - 1)));
        reach = door + (own - door) * smoothstep(into / REACH_BLEND);
    } else if (into > ROOM_LENGTH - REACH_BLEND) {
        const door = Math.min(own, clearOf(sizeAt(index + 1)));
        reach = own + (door - own) * smoothstep((into - ROOM_LENGTH + REACH_BLEND) / REACH_BLEND);
    }
    return reach - REACH_MARGIN;
}

/* the rooms */

/** every kind once, shuffled the same way every time for the same cycle */
function shuffledKinds(cycle: number): RoomKind[] {
    const rng = mulberry32.create(cycle * 7727 + 3);
    const kinds = [...ROOM_KINDS];
    for (let i = kinds.length - 1; i > 0; i--) {
        const j = Math.floor(mulberry32.sample(rng) * (i + 1));
        [kinds[i], kinds[j]] = [kinds[j]!, kinds[i]!];
    }
    return kinds;
}

/**
 * the kind of the room at an index along the run, the same every time: the rooms go in cycles of every kind once,
 * shuffled, and a cycle never starts with the kind the last one ended on
 */
function roomKind(index: number): RoomKind {
    const cycle = Math.floor(index / ROOM_KINDS.length);
    const kinds = shuffledKinds(cycle);
    if (cycle > 0 && kinds[0] === shuffledKinds(cycle - 1)[ROOM_KINDS.length - 1]) [kinds[0], kinds[1]] = [kinds[1]!, kinds[0]!];
    return kinds[index % ROOM_KINDS.length]!;
}

/** how far either side of the middle a room is clear to run, inside what lines its walls */
function clearOf(size: RoomSize): number {
    return size.half - size.lining;
}

/** the size of the room at an index along the run, the same every time */
function sizeAt(index: number): RoomSize {
    const rng = mulberry32.create(index * 104729 + 7);
    return roomSize(roomKind(index), () => mulberry32.sample(rng));
}

function createRoom(demo: Demo, index: number): Room {
    const kind = roomKind(index);
    const size = sizeAt(index);
    const room: Room = { index, kind, size, bodies: [] };
    const rng = mulberry32.create(index * 7919 + 13);
    const zStart = -index * ROOM_LENGTH - demo.origin;
    const scenery: Look[] = [];
    const build: Builder = {
        random: () => mulberry32.sample(rng),
        prop(shape: Shape, position: Vec3, quaternion: Quat, looks: Look[], material: Material = {}) {
            demo.pending.push({ shape, position, quaternion, looks, material });
        },
        fixture(shape: Shape, position: Vec3, looks: Look[]) {
            const body = rigidBody.create(demo.world, {
                shape,
                objectLayer: demo.notMoving,
                motionType: MotionType.STATIC,
                position,
                friction: 0.6,
            });
            releaseShape(shape);
            room.bodies.push(body);
            addDrawable(demo, { body, looks, fades: false });
        },
        scenery(look: Look) {
            scenery.push(look);
        },
    };
    buildRoom(build, kind, zStart, size, index > 0 ? sizeAt(index - 1) : { ...size, height: 0 });
    // the scenery hangs off a body of its own, so it moves with a rebase and goes with its room
    const shape = emptyShape.create();
    const body = rigidBody.create(demo.world, { shape, objectLayer: demo.notMoving, motionType: MotionType.STATIC });
    releaseShape(shape);
    room.bodies.push(body);
    addDrawable(demo, { body, looks: scenery, fades: false });
    return room;
}

/** some of the props waiting to be made, so building a room is spread over steps */
function createPending(demo: Demo, count: number): void {
    for (const { shape, position, quaternion, looks, material } of demo.pending.splice(0, count)) {
        const body = rigidBody.create(demo.world, {
            shape,
            objectLayer: demo.moving,
            motionType: MotionType.DYNAMIC,
            position,
            quaternion,
            friction: material.friction ?? 0.5,
            restitution: material.restitution ?? 0.25,
        });
        releaseShape(shape);
        rigidBody.sleep(demo.world, body);
        demo.props.push(body);
        addDrawable(demo, { body, looks, fades: true });
    }
}

/** the rooms from one behind the runner to a few ahead, and no others */
function updateRooms(demo: Demo): void {
    const at = Math.floor(-(demo.runner.z + demo.origin) / ROOM_LENGTH);
    for (const [index, room] of demo.rooms) {
        if (index >= at - ROOMS_BEHIND && index <= at + ROOMS_AHEAD) continue;
        for (const body of room.bodies) removeBody(demo, body);
        demo.rooms.delete(index);
    }
    for (let index = Math.max(0, at - ROOMS_BEHIND); index <= at + ROOMS_AHEAD; index++)
        if (!demo.rooms.has(index)) demo.rooms.set(index, createRoom(demo, index));
}

/** props past the camera, or fallen out of the world, taken away */
function sweepProps(demo: Demo): void {
    const behind = demo.runner.z + PROP_BEHIND;
    demo.props = demo.props.filter((body) => {
        rigidBody.getPosition(_position, demo.world, body);
        const gone = _position[2] > behind || _position[1] < -20;
        if (gone) removeBody(demo, body);
        return !gone;
    });
}

/** everything moved back toward the origin by the rebase distance, so the numbers stay small */
function rebase(demo: Demo): void {
    const by: Vec3 = [0, 0, REBASE_DISTANCE];
    demo.origin -= REBASE_DISTANCE;
    const bodies = [
        ...[...demo.rooms.values()].flatMap((room) => room.bodies),
        ...demo.props,
        ...demo.runner.parts,
        ...demo.runner.scarf,
    ];
    for (const body of bodies) {
        rigidBody.getPosition(_position, demo.world, body);
        rigidBody.setPosition(demo.world, body, vec3.add(_position, _position, by), false);
    }
    for (const { position } of demo.pending) vec3.add(position, position, by);
    demo.runner.z += REBASE_DISTANCE;
    poseRunner(demo.runner);
    demo.changes.push({ type: 'rebase', by });
}

export function stepDemo(demo: Demo): void {
    stepRunner(demo);
    updateWorld(demo.world, demo.listener, STEP);
    demo.steps++;
    if (demo.steps % SWEEP_EVERY === 0) sweepProps(demo);
    updateRooms(demo);
    createPending(demo, PENDING_PER_STEP);
    if (demo.runner.z < -REBASE_DISTANCE) rebase(demo);
    vec3.set(demo.focus, demo.runner.x, HIP_HEIGHT, demo.runner.z);
}

/** the changes since the last call, for the drawing to catch up with */
export function takeChanges(demo: Demo): Change[] {
    return demo.changes.splice(0);
}
