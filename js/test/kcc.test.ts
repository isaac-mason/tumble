import { describe, expect, test } from 'vitest';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    createWorld,
    createWorldSettings,
    destroyWorld,
    enableCollision,
    type Filter,
    filter,
    INVALID_BODY_ID,
    init,
    type KCC,
    type KCCContact,
    kcc,
    kccPolicy,
    MotionType,
    releaseShape,
    rigidBody,
    type Shape,
    transformed,
    updateWorld,
    type Vec3,
    type World,
} from '../src';
import * as Low from '../src/generated/tumble';

const DT = 1 / 60;
const GRAVITY: Vec3 = [0, -10, 0];

type Scene = { world: World; moving: number; notMoving: number; shapes: Shape[]; filter: Filter };

async function makeScene(): Promise<Scene> {
    await init();
    const settings = createWorldSettings();
    const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
    const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
    enableCollision(settings, moving, moving);
    enableCollision(settings, moving, notMoving);
    const world = createWorld(settings);
    const floor = box.create({ halfExtents: [50, 0.5, 50] });
    rigidBody.create(world, { shape: floor, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
    return { world, moving, notMoving, shapes: [floor], filter: filter.create(world) };
}

function addBox(
    scene: Scene,
    halfExtents: Vec3,
    position: Vec3,
    motionType: MotionType = MotionType.STATIC,
    density?: number,
): number {
    const shape = box.create({ halfExtents, density });
    scene.shapes.push(shape);
    const objectLayer = motionType === MotionType.STATIC ? scene.notMoving : scene.moving;
    return rigidBody.create(scene.world, { shape, objectLayer, motionType, position });
}

/** a capsule on its feet: its origin at the character's position, the capsule raised 0.8 to stand on them */
function standingShape(scene: Scene): Shape {
    const capsuleShape = capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 });
    const standing = transformed.create({ shape: capsuleShape, position: [0, 0.8, 0] });
    scene.shapes.push(capsuleShape, standing);
    return standing;
}

function release(scene: Scene, characters: KCC[]): void {
    for (const character of characters) {
        kcc.remove(scene.world, character);
        kcc.release(character);
    }
    filter.destroy(scene.filter);
    destroyWorld(scene.world);
    for (const shape of scene.shapes.reverse()) releaseShape(shape);
}

/** a frame of a game: the velocity from the ground's when standing (else the vertical part), gravity and the input */
function velocityFor(character: KCC, input: Vec3): Vec3 {
    const { up } = character;
    const v = character.linearVelocity;
    const along = v[0] * up[0] + v[1] * up[1] + v[2] * up[2];
    const base: Vec3 =
        character.ground.state === kcc.GroundState.ON_GROUND
            ? [...character.ground.velocity]
            : [up[0] * along, up[1] * along, up[2] * along];
    return [base[0] + GRAVITY[0] * DT + input[0], base[1] + GRAVITY[1] * DT + input[1], base[2] + GRAVITY[2] * DT + input[2]];
}

const f = Math.fround;

/** a float32 dot, as vec3f's: (x x + y y) + z z */
function dot32(a: Vec3, b: Vec3): number {
    return f(f(f(a[0] * b[0]) + f(a[1] * b[1])) + f(a[2] * b[2]));
}

/** move from the toolkit, its own float32 arithmetic as the C++ does it, the contacts open to edit between collide and solve */
function toolkitMove(
    scene: Scene,
    character: KCC,
    edit?: (character: KCC) => void,
    skipHit?: (hit: KCCContact) => boolean,
): void {
    const position: Vec3 = [...character.position];
    const velocity: Vec3 = [...character.linearVelocity];
    let direction: Vec3 = [0, 0, 0];
    const speedSq = dot32(velocity, velocity);
    if (speedSq > f(1e-12)) {
        const inverse = f(1 / f(Math.sqrt(speedSq)));
        direction = [f(velocity[0] * inverse), f(velocity[1] * inverse), f(velocity[2] * inverse)];
    }
    const result = kcc.createSolveResult();
    const hit = kcc.createContact();
    let timeRemaining = f(DT);
    for (let iteration = 0; iteration < 5; iteration++) {
        if (timeRemaining < f(1e-4)) break;
        kcc.collide(scene.world, character, position, direction, scene.filter, undefined);
        edit?.(character);
        kcc.solve(result, scene.world, character, velocity, DT, timeRemaining, undefined);
        velocity[0] = result.velocity[0];
        velocity[1] = result.velocity[1];
        velocity[2] = result.velocity[2];
        const displacement: Vec3 = [...result.displacement];
        let timeSimulated = result.timeSimulated;
        if (kcc.cast(hit, scene.world, character, position, displacement, scene.filter, undefined) && !skipHit?.(hit)) {
            for (let i = 0; i < 3; i++) displacement[i] = f(displacement[i]! * hit.fraction);
            timeSimulated = f(timeSimulated * hit.fraction);
        }
        for (let i = 0; i < 3; i++) position[i] = f(position[i]! + displacement[i]!);
        timeRemaining = f(timeRemaining - timeSimulated);
        if (dot32(displacement, displacement) < f(1e-8)) break;
    }
    kcc.setPosition(scene.world, character, position);
    kcc.updateGround(scene.world, character, DT, false, undefined);
    kcc.applyWeight(scene.world, character, GRAVITY, DT);
}

describe('character controller', () => {
    test('a frame of update walks it over a step, on the ground throughout', async () => {
        const scene = await makeScene();
        addBox(scene, [1, 0.15, 2], [5, 0.15, 0]);
        const standing = standingShape(scene);
        const character = kcc.create(
            {
                shape: standing,
                maxSlopeAngle: (45 * Math.PI) / 180,
                innerRigidBody: { shape: standing, objectLayer: scene.moving },
            },
            [0, 0, 0],
            [0, 0, 0, 1],
        );
        kcc.add(scene.world, character);
        expect(character.innerRigidBodyId).not.toBe(INVALID_BODY_ID);
        const settings = kcc.createDefaultUpdateSettings();
        let topOfStep = 0;
        for (let frame = 0; frame < 180; frame++) {
            character.linearVelocity = velocityFor(character, [3, 0, 0]);
            kcc.update(scene.world, character, DT, GRAVITY, settings, undefined, scene.filter);
            updateWorld(scene.world, undefined, DT);
            if (character.position[0] > 4.5 && character.position[0] < 5.5)
                topOfStep = Math.max(topOfStep, character.position[1]);
        }
        // up onto the 30 cm step, off it, and on along the floor
        expect(topOfStep).toBeCloseTo(0.3, 2);
        expect(character.position[0]).toBeGreaterThan(8);
        expect(character.ground.state).toBe(kcc.GroundState.ON_GROUND);
        // its inner body follows it
        const inner: Vec3 = [0, 0, 0];
        rigidBody.getPosition(inner, scene.world, character.innerRigidBodyId);
        expect(inner[0]).toBeCloseTo(character.position[0], 4);
        release(scene, [character]);
    });

    test("the toolkit's phases in a loop of your own are move, bit for bit", async () => {
        const runs: { position: Vec3; velocity: Vec3; ground: number; crate: Vec3 }[] = [];
        for (const toolkit of [false, true]) {
            const scene = await makeScene();
            // a wall to slide along later
            addBox(scene, [3, 1.5, 0.2], [2, 1.5, -3]);
            // 10 kg: light enough for the character's 100 N
            const crate = addBox(scene, [0.4, 0.4, 0.4], [1.5, 0.4, 0], MotionType.DYNAMIC, 10);
            for (let i = 0; i < 20; i++) updateWorld(scene.world, undefined, DT);
            const character = kcc.create({ shape: standingShape(scene) }, [0, 0, 0], [0, 0, 0, 1]);
            for (let frame = 0; frame < 200; frame++) {
                character.linearVelocity = velocityFor(character, frame < 90 ? [2, 0, 0] : [1, 0, -3]);
                if (toolkit) toolkitMove(scene, character);
                else kcc.move(scene.world, character, DT, GRAVITY, undefined, scene.filter);
                updateWorld(scene.world, undefined, DT);
            }
            const cratePosition: Vec3 = [0, 0, 0];
            rigidBody.getPosition(cratePosition, scene.world, crate);
            runs.push({
                position: [...character.position],
                velocity: [...character.linearVelocity],
                ground: character.ground.state,
                crate: cratePosition,
            });
            release(scene, [character]);
        }
        const [moved, looped] = runs;
        expect(looped).toEqual(moved);
        // and it did things worth comparing: shoved the crate along, then slid along the wall, its face at z -2.8 less
        // the radius and the padding
        expect(moved!.crate[0]).toBeGreaterThan(4);
        expect(moved!.position[2]).toBeCloseTo(-2.48, 3);
    });

    test('a policy: carried by a conveyor, held by a crate it may not push, through a wall it ignores', async () => {
        const scene = await makeScene();
        const belt = addBox(scene, [3, 0.1, 1], [2, 0.1, 0]);
        const crate = addBox(scene, [0.4, 0.4, 0.4], [1.5, 0.4, 4], MotionType.DYNAMIC);
        const ghost = addBox(scene, [0.2, 1.5, 3], [2, 1.5, 8]);
        for (let i = 0; i < 20; i++) updateWorld(scene.world, undefined, DT);
        const policy = kccPolicy.create();
        kccPolicy.set(policy, belt, { surfaceLinearVelocity: [2, 0, 0] });
        kccPolicy.set(policy, crate, { canReceiveImpulses: false });
        kccPolicy.set(policy, ghost, { ignore: true });
        const settings = kcc.createDefaultUpdateSettings();
        const run = async (start: Vec3, input: Vec3, frames: number): Promise<KCC> => {
            const character = kcc.create({ shape: standingShape(scene) }, start, [0, 0, 0, 1]);
            for (let frame = 0; frame < frames; frame++) {
                kcc.updateGroundVelocity(scene.world, character, policy);
                character.linearVelocity = velocityFor(character, input);
                kcc.update(scene.world, character, DT, GRAVITY, settings, policy, scene.filter);
                updateWorld(scene.world, undefined, DT);
            }
            return character;
        };
        // two seconds standing still on the belt, carried at its 2 m/s
        const rider = await run([0, 0.2, 0], [0, 0, 0], 120);
        expect(rider.position[0]).toBeCloseTo(4, 1);
        // held at the crate's face, the crate unmoved
        const pusher = await run([0, 0, 4], [2, 0, 0], 90);
        expect(pusher.position[0]).toBeCloseTo(0.78, 2);
        const at: Vec3 = [0, 0, 0];
        rigidBody.getPosition(at, scene.world, crate);
        expect(at[0]).toBeCloseTo(1.5, 3);
        // through the wall at x 2
        const walker = await run([0, 0, 8], [3, 0, 0], 90);
        expect(walker.position[0]).toBeGreaterThan(3);
        const read = {
            ignore: false,
            canPushCharacter: true,
            canReceiveImpulses: true,
            slideOnWalkable: true,
            surfaceLinearVelocity: [0, 0, 0] as Vec3,
            surfaceAngularVelocity: [0, 0, 0] as Vec3,
        };
        expect(kccPolicy.get(read, policy, crate).canReceiveImpulses).toBe(false);
        expect(kccPolicy.remove(policy, crate)).toBe(true);
        expect(kccPolicy.get(read, policy, crate).canReceiveImpulses).toBe(true);
        kccPolicy.destroy(policy);
        release(scene, [rider, pusher, walker]);
    });

    test('contact events: the floor added then persisted, and removed when it goes', async () => {
        const scene = await makeScene();
        const ground = addBox(scene, [3, 0.25, 3], [0, 0.25, 0]);
        const character = kcc.create({ shape: standingShape(scene) }, [0, 0.6, 0], [0, 0, 0, 1]);
        const settings = kcc.createDefaultUpdateSettings();
        const kinds: number[] = [];
        for (let frame = 0; frame < 30; frame++) {
            character.linearVelocity = velocityFor(character, [0, 0, 0]);
            kcc.update(scene.world, character, DT, GRAVITY, settings, undefined, scene.filter);
            for (const event of character.contactEvents) if (event.bodyId === ground) kinds.push(event.kind);
            if (frame === 20) rigidBody.remove(scene.world, ground);
        }
        expect(kinds[0]).toBe(kcc.KCCContactEventKind.ADDED);
        expect(kinds).toContain(kcc.KCCContactEventKind.PERSISTED);
        expect(kinds.at(-1)).toBe(kcc.KCCContactEventKind.REMOVED);
        release(scene, [character]);
    });

    test('a one-way platform from your own loop: discarded contacts and a skipped hit, jumped up through', async () => {
        const scene = await makeScene();
        const platform = addBox(scene, [2, 0.1, 2], [0, 2, 0]);
        const character = kcc.create({ shape: standingShape(scene) }, [0, 0, 0], [0, 0, 0, 1]);
        const fromBelow = (character: KCC): void => {
            for (const contact of character.contacts) {
                if (contact.bodyId === platform && contact.contactNormal[1] < 0.5) contact.wasDiscarded = true;
            }
        };
        character.linearVelocity = [0, 9, 0];
        let highest = 0;
        for (let frame = 0; frame < 120; frame++) {
            const v = character.linearVelocity;
            character.linearVelocity = [v[0], v[1] + GRAVITY[1] * DT, v[2]];
            toolkitMove(scene, character, fromBelow, (hit) => hit.bodyId === platform && hit.contactNormal[1] < 0.5);
            if (character.ground.state === kcc.GroundState.ON_GROUND && character.linearVelocity[1] <= 0)
                character.linearVelocity = [0, 0, 0];
            highest = Math.max(highest, character.position[1]);
            updateWorld(scene.world, undefined, DT);
        }
        expect(highest).toBeGreaterThan(2.5);
        expect(character.ground.bodyId).toBe(platform);
        expect(character.position[1]).toBeCloseTo(2.1, 2);
        release(scene, [character]);
    });

    test('tuning after create: the slope it can walk, read back as set, up normalized', async () => {
        const scene = await makeScene();
        const character = kcc.create({ shape: standingShape(scene), maxSlopeAngle: 0.5 }, [0, 0, 0], [0, 0, 0, 1]);
        const tuning = kcc.getTuning(
            {
                shapeOffset: [0, 0, 0],
                mass: 0,
                maxStrength: 0,
                up: [0, 0, 0],
                maxSlopeAngle: 0,
                predictiveContactDistance: 0,
                characterPadding: 0,
                maxCollisionIterations: 0,
                maxConstraintIterations: 0,
                collisionTolerance: 0,
                penetrationRecoverySpeed: 0,
                minTimeRemaining: 0,
                maxNumHits: 0,
                hitReductionCosMaxAngle: 0,
                backFaceMode: kcc.BackFaceMode.COLLIDE,
                enhancedInternalEdgeRemoval: false,
                supportingVolumePlane: [0, 0, 0, 0],
            },
            character,
        );
        expect(tuning.maxSlopeAngle).toBe(Math.fround(0.5));
        // without a plane at create, anything below it can support it
        expect(tuning.supportingVolumePlane).toEqual([0, 1, 0, Math.fround(-1e10)]);
        // a 40 degree normal: too steep at 0.5 rad (29 degrees), walkable once the limit is 1
        const normal: Vec3 = [Math.sin((40 * Math.PI) / 180), Math.cos((40 * Math.PI) / 180), 0];
        expect(kcc.isSlopeTooSteep(character, normal)).toBe(true);
        kcc.setTuning(character, { maxSlopeAngle: 1, up: [0, 2, 0], enhancedInternalEdgeRemoval: true });
        expect(kcc.isSlopeTooSteep(character, normal)).toBe(false);
        kcc.getTuning(tuning, character);
        expect(tuning.up).toEqual([0, 1, 0]);
        expect(character.up).toEqual([0, 1, 0]);
        expect(tuning.enhancedInternalEdgeRemoval).toBe(true);
        // the rest as they were
        expect(tuning.supportingVolumePlane[3]).toBe(Math.fround(-1e10));
        release(scene, [character]);
    });

    test("a filter's body filter is refused: a character's queries run in the wasm", async () => {
        const scene = await makeScene();
        const character = kcc.create({ shape: standingShape(scene) }, [0, 0, 0], [0, 0, 0, 1]);
        scene.filter.bodyFilter = () => true;
        expect(() =>
            kcc.update(scene.world, character, DT, GRAVITY, kcc.createDefaultUpdateSettings(), undefined, scene.filter),
        ).toThrow(RangeError);
        scene.filter.bodyFilter = null;
        release(scene, [character]);
    });

    test('characters and policies, made and freed, leave the heap as it was', async () => {
        await init();
        const before = Low.allocatedBytes();
        const scene = await makeScene();
        const policy = kccPolicy.create();
        const standing = standingShape(scene);
        const character = kcc.create(
            { shape: standing, innerRigidBody: { shape: standing, objectLayer: scene.moving } },
            [0, 0, 0],
            [0, 0, 0, 1],
        );
        kcc.add(scene.world, character);
        for (let i = 0; i < 40; i++) kccPolicy.set(policy, 1000 + i, { ignore: true });
        for (let frame = 0; frame < 30; frame++) {
            character.linearVelocity = velocityFor(character, [1, 0, 0]);
            kcc.update(scene.world, character, DT, GRAVITY, kcc.createDefaultUpdateSettings(), policy, scene.filter);
        }
        kccPolicy.destroy(policy);
        release(scene, [character]);
        expect(Low.allocatedBytes()).toBe(before);
    });
});
