import { type Quat, quat, type Vec3 } from 'math';
import { box, type CompoundChild, compound, cylinder, releaseShape, type Shape, sphere } from 'tumble.js';
import type { Look, Primitive } from './demo';
import { rgb, STRUCTURE, THINGS, type Tone } from './theme';

// the places the runner runs through, one after another in no order: a supermarket, a library, a diner, a warehouse,
// a backyard, a kid's bedroom and a main street. each is its own width and height, at the scale of the real thing, and
// set in a town that reaches off to the hills either side, drawn only. every place opens with a wall across it, and
// is laid out like the real thing with heaps to knock over. x is across, y up, and the runner runs toward -z

const cream: Tone = 'cream';
const linen: Tone = 'linen';
const stone: Tone = 'stone';
const oak: Tone = 'oak';
const bark: Tone = 'bark';
const ink: Tone = 'ink';
const sage: Tone = 'sage';
const moss: Tone = 'moss';
const terracotta: Tone = 'terracotta';
const blush: Tone = 'blush';
const butter: Tone = 'butter';
const sky: Tone = 'sky';
const denim: Tone = 'denim';

export const ROOM_LENGTH = 32;

export type RoomKind = 'supermarket' | 'library' | 'diner' | 'warehouse' | 'backyard' | 'bedroom' | 'street';
export const ROOM_KINDS: RoomKind[] = ['supermarket', 'library', 'diner', 'warehouse', 'backyard', 'bedroom', 'street'];

/**
 * how far a room reaches either side of the middle, to its walls or its edges, how high its ceiling is, 0 outdoors, and
 * how deep what lines its walls is, which the runner keeps out of
 */
export type RoomSize = { half: number; height: number; lining: number };

const SIZES: Record<RoomKind, { half: [number, number]; height: [number, number]; lining: number }> = {
    supermarket: { half: [6, 9], height: [3.8, 4.6], lining: 0.95 },
    library: { half: [4.5, 7.5], height: [3.4, 5], lining: 0.7 },
    diner: { half: [3.2, 4.5], height: [2.9, 3.2], lining: 0 },
    warehouse: { half: [8, 11], height: [6, 7.5], lining: 0 },
    backyard: { half: [5, 9], height: [0, 0], lining: 0.85 },
    bedroom: { half: [3.2, 4.4], height: [2.8, 3], lining: 0 },
    street: { half: [5.5, 9], height: [0, 0], lining: 0 },
};

export function roomSize(kind: RoomKind, random: () => number): RoomSize {
    const { half, height, lining } = SIZES[kind];
    return { half: half[0] + random() * (half[1] - half[0]), height: height[0] + random() * (height[1] - height[0]), lining };
}

export type Material = { friction?: number; restitution?: number };

/** what a room is built with: bodies placed in the world as it is now. the builder takes each shape over */
export type Builder = {
    random: () => number;
    /** a body that moves, asleep until something touches it */
    prop(shape: Shape, position: Vec3, quaternion: Quat, looks: Look[], material?: Material): void;
    /** a body that never moves, taken away with its room */
    fixture(shape: Shape, position: Vec3, looks: Look[]): void;
    /** drawn only, placed in the world, taken away with its room */
    scenery(look: Look): void;
};

/** a room being built: where it starts along z, its size, and the size of the room before it */
type Place = { build: Builder; z: number; half: number; height: number; previous: RoomSize };

const NO_ROTATION: Quat = [0, 0, 0, 1];
const ORIGIN: Vec3 = [0, 0, 0];
const ACROSS = quat.setAxisAngle([0, 0, 0, 1], [0, 0, 1], Math.PI / 2);

function turned(angle: number): Quat {
    return quat.setAxisAngle([0, 0, 0, 1], [0, 1, 0], angle);
}

/** how something built in looks */
function look(primitive: Primitive, size: Vec3, tone: Tone, offset: Vec3 = ORIGIN, rotation: Quat = NO_ROTATION): Look {
    return { primitive, size, offset, rotation, color: rgb(STRUCTURE[tone]) };
}

/** how something that can be knocked over looks */
function thingLook(primitive: Primitive, size: Vec3, tone: Tone, offset: Vec3 = ORIGIN, rotation: Quat = NO_ROTATION): Look {
    return { primitive, size, offset, rotation, color: rgb(THINGS[tone]) };
}

function pick<T>(build: Builder, items: readonly T[]): T {
    return items[Math.floor(build.random() * items.length)]!;
}

function between(build: Builder, low: number, high: number): number {
    return low + build.random() * (high - low);
}

/** the middles of count equal spans side by side across from -half to half */
function spread(count: number, half: number): number[] {
    return Array.from({ length: count }, (_, i) => -half + ((i + 0.5) * 2 * half) / count);
}

/** a point turned about a centre by an angle about y: for placing a piece's parts as the piece is turned */
function placed(center: Vec3, angle: number, local: Vec3): Vec3 {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [center[0] + local[0] * c + local[2] * s, center[1] + local[1], center[2] - local[0] * s + local[2] * c];
}

/* props: bodies of one primitive, or of several parts together */

/** a primitive's part of a body: size as a look's */
type Part = { primitive: Primitive; size: Vec3; offset: Vec3; color: Tone; rotation?: Quat };

function shapeOf(primitive: Primitive, size: Vec3, density: number): Shape {
    if (primitive === 'box') return box.create({ halfExtents: size, density });
    if (primitive === 'cylinder') return cylinder.create({ radius: size[0], halfHeight: size[1], density });
    return sphere.create({ radius: size[0], density });
}

function prop(
    build: Builder,
    primitive: Primitive,
    size: Vec3,
    position: Vec3,
    color: Tone,
    density: number,
    angle = 0,
    material?: Material,
) {
    build.prop(shapeOf(primitive, size, density), position, turned(angle), [thingLook(primitive, size, color)], material);
}

function boxProp(
    build: Builder,
    halfExtents: Vec3,
    position: Vec3,
    color: Tone,
    density: number,
    angle = 0,
    material?: Material,
) {
    prop(build, 'box', halfExtents, position, color, density, angle, material);
}

function cylinderProp(build: Builder, radius: number, halfHeight: number, position: Vec3, color: Tone, density: number) {
    prop(build, 'cylinder', [radius, halfHeight, radius], position, color, density);
}

function sphereProp(build: Builder, radius: number, position: Vec3, color: Tone, density: number, material?: Material) {
    prop(build, 'sphere', [radius, radius, radius], position, color, density, 0, material);
}

function partsProp(build: Builder, parts: Part[], position: Vec3, density: number, angle = 0, material?: Material) {
    const children: CompoundChild[] = parts.map(({ primitive, size, offset, rotation }) => ({
        shape: shapeOf(primitive, size, density),
        position: offset,
        quaternion: rotation ?? NO_ROTATION,
    }));
    const shape = compound.create({ children });
    for (const child of children) releaseShape(child.shape);
    const looks = parts.map(({ primitive, size, offset, color, rotation }) =>
        thingLook(primitive, size, color, offset, rotation),
    );
    build.prop(shape, position, turned(angle), looks, material);
}

const part = (primitive: Primitive, size: Vec3, offset: Vec3, color: Tone, rotation?: Quat): Part => ({
    primitive,
    size,
    offset,
    color,
    rotation,
});

/* the town around every room, drawn only */

const BUILDING_TONES = [cream, linen, blush, butter, sage, sky];
const ROOF_TONES = [terracotta, bark, denim, stone];
const CARDBOARD = [oak, stone];
/** a ridge roof is a square prism turned on its edge, half buried in the building's top */
const ON_EDGE = quat.setAxisAngle([0, 0, 0, 1], [0, 0, 1], Math.PI / 4);

/** a block of a building under a ridge roof or a flat one, its face toward the middle of the town lined with windows */
function building(build: Builder, center: Vec3, half: Vec3, color: Tone, windows: boolean) {
    build.scenery(look('box', half, color, center));
    const top = center[1] + half[1];
    if (half[1] < 7 && build.random() < 0.65) {
        const side = half[0] / Math.SQRT2;
        build.scenery(look('box', [side, side, half[2] + 0.2], pick(build, ROOF_TONES), [center[0], top, center[2]], ON_EDGE));
    } else build.scenery(look('box', [half[0] + 0.15, 0.2, half[2] + 0.15], stone, [center[0], top + 0.2, center[2]]));
    if (!windows) return;
    const face = center[0] - Math.sign(center[0]) * (half[0] + 0.02);
    for (let y = 1.8; y < 2 * half[1] - 1; y += 3)
        for (let z = -half[2] + 1.4; z <= half[2] - 1.4; z += 2.6)
            build.scenery(look('box', [0.05, 0.65, 0.5], denim, [face, center[1] - half[1] + y, center[2] + z]));
}

/** ground to the hills, rows of buildings either side from the near edge out, hills beyond, and clouds */
function town(build: Builder, zStart: number, nearEdge: number) {
    const middle = zStart - ROOM_LENGTH / 2;
    build.scenery(look('box', [160, 0.1, ROOM_LENGTH / 2], sage, [0, -0.12, middle]));
    for (const side of [-1, 1]) {
        // a near row with windows, and a taller row behind it without
        for (const [edge, low, high, windows] of [
            [nearEdge, 5, 12, true],
            [nearEdge + 20, 10, 26, false],
        ] as const) {
            let along = between(build, 0.6, 3);
            while (along < ROOM_LENGTH) {
                const length = between(build, 6, 12);
                if (along + length > ROOM_LENGTH) break;
                const depth = between(build, 6, 11);
                const height = between(build, low, high);
                const x = side * (edge + depth / 2 + between(build, 0, 3));
                building(
                    build,
                    [x, height / 2, zStart - along - length / 2],
                    [depth / 2, height / 2, length / 2],
                    pick(build, BUILDING_TONES),
                    windows,
                );
                along += length + between(build, 1, 3);
            }
        }
        const hill = between(build, 30, 50);
        build.scenery(look('sphere', [hill, hill * 0.35, hill * 0.8], moss, [side * between(build, 110, 140), -4, middle]));
    }
    // now and then a cloud, a few flattened puffs
    if (build.random() < 0.6) {
        const center: Vec3 = [
            (build.random() < 0.5 ? -1 : 1) * between(build, 20, 90),
            between(build, 28, 45),
            zStart - between(build, 0, ROOM_LENGTH),
        ];
        for (let p = 0; p < 3; p++) {
            const size = between(build, 4, 7);
            build.scenery(
                look('sphere', [size, size * 0.4, size], cream, [
                    center[0] + (p - 1) * 5,
                    center[1],
                    center[2] + between(build, -2, 2),
                ]),
            );
        }
    }
}

/* floors and walls */

/** a box that is built in, as it is drawn, so nothing knocked about goes through it */
function solid(build: Builder, halfExtents: Vec3, color: Tone, center: Vec3) {
    build.fixture(box.create({ halfExtents }), center, [look('box', halfExtents, color)]);
}

function floor(place: Place, halfWidth: number, color: Tone) {
    solid(place.build, [halfWidth, 0.25, ROOM_LENGTH / 2], color, [0, -0.25, place.z - ROOM_LENGTH / 2]);
}

type Shell = {
    floor: Tone;
    wall: Tone;
    /** the building's front, around the way in */
    front: Tone;
    /** how high the windows start, higher where something lines the walls below */
    sill: number;
};

const WINDOW_EVERY = 4;
const WINDOW_HALF_WIDTH = 1.3;

/**
 * an indoor room: its floor, walls either side that keep what flies in, with a row of windows onto the town, a
 * ceiling, and the building's front around the way in, reaching past the room before so it closes that room off
 */
function shell(place: Place, { floor: floorColor, wall, front, sill }: Shell) {
    const { build, z, half, height, previous } = place;
    floor(place, half + 0.5, floorColor);
    town(build, z, half + 1.5);
    const middle = z - ROOM_LENGTH / 2;
    const lintel = Math.max(sill + 0.5, height - 0.5);
    for (const side of [-1, 1]) {
        const x = side * (half + 0.2);
        build.fixture(box.create({ halfExtents: [0.2, height / 2, ROOM_LENGTH / 2] }), [x, height / 2, middle], []);
        build.scenery(look('box', [0.2, sill / 2, ROOM_LENGTH / 2], wall, [x, sill / 2, middle]));
        build.scenery(look('box', [0.2, (height - lintel) / 2, ROOM_LENGTH / 2], wall, [x, (height + lintel) / 2, middle]));
        // the piers between the windows, half ones at the ends so neighbouring rooms' never overlap
        const pier = WINDOW_EVERY / 2 - WINDOW_HALF_WIDTH;
        for (let at = 0; at <= ROOM_LENGTH; at += WINDOW_EVERY) {
            const halfLength = at === 0 || at === ROOM_LENGTH ? pier / 2 : pier;
            const pierZ = at === 0 ? z - pier / 2 : at === ROOM_LENGTH ? z - ROOM_LENGTH + pier / 2 : z - at;
            build.scenery(look('box', [0.2, (lintel - sill) / 2, halfLength], wall, [x, (sill + lintel) / 2, pierZ]));
        }
    }
    solid(build, [half + 0.4, 0.15, ROOM_LENGTH / 2], cream, [0, height + 0.15, middle]);
    // the way out: the end wall over a doorway as wide as the room, so the building ends there rather than stops
    const doorway = Math.max(2.5, height * 0.55);
    if (height > doorway)
        solid(build, [half + 0.4, (height - doorway) / 2, 0.15], wall, [0, (height + doorway) / 2, z - ROOM_LENGTH + 0.15]);

    // the front: either side of the way in and over it, out past the room before and up over its ceiling
    const out = Math.max(half, previous.half) + 1.5;
    const top = Math.max(height, previous.height) + 1;
    const frontZ = z - 0.15;
    const sideHalf: Vec3 = [(out - half - 0.4) / 2, top / 2, 0.15];
    for (const side of [-1, 1]) solid(build, sideHalf, front, [side * (half + 0.4 + sideHalf[0]), top / 2, frontZ]);
    solid(build, [half + 0.4, (top - height - 0.3) / 2, 0.15], front, [0, (top + height + 0.3) / 2, frontZ]);
}

/** an outdoor place: its ground, and the town beyond */
function outdoors(place: Place, ground: Tone, nearEdge: number) {
    floor(place, nearEdge, ground);
    town(place.build, place.z, nearEdge);
}

/** a wall across the room of blocks laid in courses up to a height, each course half a block along from the last */
function blockWall(place: Place, halfExtents: Vec3, height: number, colors: Tone[], density: number) {
    const [hx, hy] = halfExtents;
    const gap = 0.004;
    const courses = Math.max(1, Math.round(height / (2 * hy + gap)));
    for (let course = 0; course < courses; course++) {
        const shift = course % 2 === 0 ? 0 : hx;
        const y = hy + course * (2 * hy + gap);
        for (let x = -place.half + hx + shift; x <= place.half - hx; x += 2 * hx + gap)
            boxProp(place.build, halfExtents, [x, y, place.z - 0.6], pick(place.build, colors), density);
    }
}

/** tall panels side by side across the room, a door's or a window's, to burst through like a shopfront */
function panelWall(place: Place, height: number, colors: Tone[]) {
    const count = Math.max(2, Math.round(place.half / 0.6));
    const halfWidth = place.half / count - 0.005;
    for (const x of spread(count, place.half))
        boxProp(place.build, [halfWidth, height / 2, 0.04], [x, height / 2, place.z - 0.6], pick(place.build, colors), 400);
}

/** the way in, floor to ceiling: a wall of blocks, or now and then a row of panels */
function entrance(place: Place, blocks: () => void, panels: Tone[]) {
    if (place.build.random() < 0.35) panelWall(place, place.height, panels);
    else blocks();
}

/* pieces found in many places */

/** a pyramid of whatever place puts at each spot, rows narrowing upward */
function pyramid(base: number, place: (row: number, column: number, columns: number) => void) {
    for (let row = 0; row < base; row++) {
        const columns = base - row;
        for (let column = 0; column < columns; column++) place(row, column, columns);
    }
}

function canPyramid(build: Builder, x: number, z: number, color: Tone) {
    const radius = 0.11;
    const halfHeight = 0.15;
    pyramid(5, (row, column, columns) => {
        const at = x + (column - (columns - 1) / 2) * (2 * radius + 0.01);
        cylinderProp(
            build,
            radius,
            halfHeight,
            [at, halfHeight + row * (2 * halfHeight + 0.004), z],
            row % 2 === 0 ? color : cream,
            300,
        );
    });
}

function boxStack(build: Builder, x: number, z: number, half: Vec3, layers: number) {
    for (let layer = 0; layer < layers; layer++)
        for (const sx of [-1, 1])
            for (const sz of [-1, 1])
                boxProp(
                    build,
                    half,
                    [x + sx * (half[0] + 0.005), half[1] + layer * (2 * half[1] + 0.004), z + sz * (half[2] + 0.005)],
                    pick(build, CARDBOARD),
                    120,
                );
}

type Stock = { primitive: Primitive; half: Vec3; density: number };

/**
 * shelves on uprights, turned by the angle: open both sides, or with a back, and stock in a row along every board but
 * the top, on each open side, a board's stock all one colour
 */
function shelves(
    build: Builder,
    center: Vec3,
    angle: number,
    halfLength: number,
    height: number,
    boards: number,
    frameColor: Tone,
    stock: Stock[],
    colors: Tone[],
    bothSides: boolean,
) {
    const halfDepth = 0.3;
    const parts: Part[] = [];
    for (const end of [-1, 1])
        parts.push(part('box', [0.04, height / 2, halfDepth], [end * halfLength, height / 2, 0], frameColor));
    parts.push(
        part('box', [halfLength, height / 2 - 0.05, 0.015], [0, height / 2, bothSides ? 0 : -halfDepth + 0.015], frameColor),
    );
    const boardHeights: number[] = [];
    for (let b = 0; b < boards; b++) {
        const y = 0.1 + (b * (height - 0.15)) / (boards - 1);
        boardHeights.push(y);
        parts.push(part('box', [halfLength, 0.025, halfDepth], [0, y, 0], frameColor));
    }
    partsProp(build, parts, center, 300, angle);

    const rows = bothSides ? [-halfDepth / 2 - 0.01, halfDepth / 2 + 0.01] : [0.02];
    for (let b = 0; b < boards - 1; b++) {
        const item = pick(build, stock);
        const color = pick(build, colors);
        const [hx, hy] = item.half;
        const top = boardHeights[b]! + 0.025;
        for (const rowZ of rows)
            for (let along = -halfLength + 0.06 + hx; along <= halfLength - 0.06 - hx; along += 2 * hx + 0.03)
                prop(
                    build,
                    item.primitive,
                    item.half,
                    placed(center, angle, [along, top + hy + 0.003, rowZ]),
                    color,
                    item.density,
                    angle,
                );
    }
}

/** a table on four legs, turned by the angle */
function table(build: Builder, center: Vec3, angle: number, halfWidth: number, halfDepth: number, height: number, color: Tone) {
    const parts = [part('box', [halfWidth, 0.04, halfDepth], [0, height - 0.04, 0], color)];
    for (const sx of [-1, 1])
        for (const sz of [-1, 1])
            parts.push(
                part(
                    'box',
                    [0.04, (height - 0.08) / 2, 0.04],
                    [sx * (halfWidth - 0.08), (height - 0.08) / 2, sz * (halfDepth - 0.08)],
                    color,
                ),
            );
    partsProp(build, parts, center, 400, angle);
}

/** a chair, its back away from where it faces: angle 0 faces +z */
function chair(build: Builder, center: Vec3, angle: number, color: Tone) {
    const seat = 0.45;
    const parts: Part[] = [
        part('box', [0.22, 0.03, 0.22], [0, seat, 0], color),
        part('box', [0.22, 0.25, 0.03], [0, seat + 0.28, -0.19], color),
    ];
    for (const sx of [-1, 1])
        for (const sz of [-1, 1])
            parts.push(part('box', [0.03, seat / 2 - 0.015, 0.03], [sx * 0.18, seat / 2 - 0.015, sz * 0.18], color));
    partsProp(build, parts, center, 400, angle);
}

/** a table with a chair at either side of each long edge, and whatever dress puts on its top at a height */
function diningSet(
    build: Builder,
    center: Vec3,
    halfWidth: number,
    tableColor: Tone,
    chairColor: Tone,
    dress: (top: number) => void,
) {
    const height = 0.76;
    table(build, center, 0, halfWidth, 0.5, height, tableColor);
    for (const side of [-1, 1])
        for (const along of [-0.5, 0.5])
            chair(build, [center[0] + along * halfWidth, 0, center[2] + side * 0.8], side > 0 ? Math.PI : 0, chairColor);
    dress(height);
}

/** an open box: a floor and four sides */
function crate(build: Builder, center: Vec3, half: Vec3, color: Tone) {
    const wall = 0.03;
    partsProp(
        build,
        [
            part('box', [half[0], wall, half[2]], [0, wall, 0], color),
            part('box', [wall, half[1], half[2]], [-half[0] + wall, half[1], 0], color),
            part('box', [wall, half[1], half[2]], [half[0] - wall, half[1], 0], color),
            part('box', [half[0], half[1], wall], [0, half[1], -half[2] + wall], color),
            part('box', [half[0], half[1], wall], [0, half[1], half[2] - wall], color),
        ],
        center,
        300,
    );
}

/** balls filling an open box, in layers */
function fillWithBalls(
    build: Builder,
    center: Vec3,
    half: Vec3,
    radius: number,
    layers: number,
    colors: Tone[],
    density: number,
) {
    for (let layer = 0; layer < layers; layer++)
        for (let x = -half[0] + 0.05 + radius; x <= half[0] - 0.05 - radius; x += 2 * radius + 0.01)
            for (let z = -half[2] + 0.05 + radius; z <= half[2] - 0.05 - radius; z += 2 * radius + 0.01)
                sphereProp(
                    build,
                    radius,
                    [center[0] + x, 0.07 + radius + layer * (2 * radius + 0.005), center[2] + z],
                    pick(build, colors),
                    density,
                );
}

function cone(build: Builder, center: Vec3) {
    partsProp(
        build,
        [
            part('box', [0.2, 0.025, 0.2], [0, 0.025, 0], terracotta),
            part('cylinder', [0.13, 0.18, 0.13], [0, 0.23, 0], terracotta),
            part('cylinder', [0.08, 0.12, 0.08], [0, 0.53, 0], cream),
        ],
        center,
        200,
    );
}

function barrier(build: Builder, center: Vec3) {
    partsProp(
        build,
        [
            part('box', [0.6, 0.12, 0.03], [0, 0.85, 0], cream),
            part('box', [0.2, 0.125, 0.04], [-0.3, 0.85, 0], terracotta),
            part('box', [0.2, 0.125, 0.04], [0.3, 0.85, 0], terracotta),
            part('box', [0.04, 0.48, 0.2], [-0.5, 0.48, 0], stone),
            part('box', [0.04, 0.48, 0.2], [0.5, 0.48, 0], stone),
        ],
        center,
        300,
    );
}

/* the places */

/** produce bins at the door, aisles of shelves with trolleys and offers in them, fridges along the walls, and the checkouts */
function supermarket(place: Place) {
    const { build, z, half } = place;
    const scheme = pick(build, [
        { wall: cream, floor: linen, front: sage, accents: [terracotta, butter] },
        { wall: cream, floor: linen, front: sky, accents: [denim, blush] },
        { wall: linen, floor: cream, front: butter, accents: [moss, terracotta] },
    ]);
    const colors = [...scheme.accents, cream];
    shell(place, { floor: scheme.floor, wall: scheme.wall, front: scheme.front, sill: 2.4 });
    entrance(place, () => blockWall(place, [0.5, 0.35, 0.35], place.height, CARDBOARD, 120), [sky, cream]);

    // fridges the length of both walls
    for (const side of [-1, 1]) {
        const x = side * (half - 0.45);
        const middle = z - ROOM_LENGTH / 2;
        solid(build, [0.45, 1.05, ROOM_LENGTH / 2 - 1], linen, [x, 1.05, middle]);
        build.scenery(look('box', [0.05, 0.9, ROOM_LENGTH / 2 - 1.2], denim, [x - side * 0.47, 1.1, middle]));
    }

    // produce: bins of fruit, one kind each
    const binX = spread(Math.max(2, Math.floor((2 * half - 2) / 2.8)), half - 1.2);
    binX.forEach((x, i) => {
        const center: Vec3 = [x, 0, z - 4];
        const binHalf: Vec3 = [0.6, 0.25, 0.45];
        crate(build, center, binHalf, oak);
        fillWithBalls(build, center, binHalf, i % 3 === 2 ? 0.2 : 0.1, i % 3 === 2 ? 1 : 2, [scheme.accents[i % 2]!], 500);
    });

    // aisles of shelves: two running with the runner in a wide shop, or rows across its way
    const stock: Stock[] = [
        { primitive: 'cylinder', half: [0.09, 0.13, 0.09], density: 400 },
        { primitive: 'box', half: [0.14, 0.2, 0.06], density: 200 },
    ];
    const along = half >= 7.2;
    if (along)
        for (const x of [-half * 0.42, half * 0.42])
            shelves(build, [x, 0, z - 15], Math.PI / 2, 5, 1.8, 4, stone, stock, colors, true);
    else {
        const halfLength = Math.min(2.6, half - 2.6);
        for (const at of [10, 15.5, 21])
            shelves(
                build,
                [between(build, -1, 1) * (half - halfLength - 1.6), 0, z - at],
                0,
                halfLength,
                1.8,
                4,
                stone,
                stock,
                colors,
                true,
            );
    }

    // in the aisles: trolleys and offers
    for (const x of along ? [-half * 0.75, 0, half * 0.75] : [-half * 0.5, half * 0.5]) {
        if (build.random() < 0.5)
            trolley(build, [x + between(build, -0.4, 0.4), 0, z - between(build, 12, 13.5)], between(build, -0.4, 0.4));
        else canPyramid(build, x, z - between(build, 12, 13.5), pick(build, scheme.accents));
        boxStack(
            build,
            x + between(build, -0.4, 0.4),
            z - between(build, 18, 19.5),
            [0.25, 0.2, 0.25],
            2 + Math.floor(build.random() * 2),
        );
    }

    // the checkouts, each a counter with shopping on its belt
    for (const x of spread(Math.max(2, Math.floor((2 * half) / 3.2)), half - 0.8)) {
        const at = z - 26.5;
        partsProp(
            build,
            [part('box', [0.4, 0.45, 1.3], [0, 0.45, 0], linen), part('box', [0.32, 0.02, 1.2], [0, 0.92, 0], ink)],
            [x, 0, at],
            300,
        );
        for (let i = 0; i < 4; i++) boxProp(build, [0.1, 0.12, 0.08], [x, 1.07, at + 0.9 - i * 0.4], pick(build, colors), 200);
    }
}

function trolley(build: Builder, center: Vec3, angle: number) {
    const parts: Part[] = [
        part('box', [0.3, 0.02, 0.45], [0, 0.45, 0], stone),
        part('box', [0.02, 0.22, 0.45], [-0.3, 0.67, 0], stone),
        part('box', [0.02, 0.22, 0.45], [0.3, 0.67, 0], stone),
        part('box', [0.3, 0.22, 0.02], [0, 0.67, 0.45], stone),
        part('box', [0.3, 0.22, 0.02], [0, 0.67, -0.45], stone),
        part('box', [0.3, 0.02, 0.02], [0, 1.0, 0.6], terracotta),
        part('box', [0.02, 0.2, 0.02], [0, 0.22, 0], stone),
    ];
    for (const sx of [-1, 1])
        for (const sz of [-1, 1]) parts.push(part('cylinder', [0.06, 0.02, 0.06], [sx * 0.25, 0.06, sz * 0.38], ink, ACROSS));
    partsProp(build, parts, center, 150, angle);
}

/** a front desk, bookcases along both walls, stacks either side of the middle, and a reading room at the back */
function library(place: Place) {
    const { build, z, half } = place;
    const scheme = pick(build, [
        { wall: linen, front: denim, books: [terracotta, denim, cream] },
        { wall: cream, front: terracotta, books: [moss, butter, linen] },
        { wall: linen, front: moss, books: [denim, blush, cream] },
    ]);
    shell(place, { floor: oak, wall: scheme.wall, front: scheme.front, sill: Math.min(2.4, place.height - 1) });
    entrance(place, () => blockWall(place, [0.45, 0.2, 0.25], place.height, [cream, linen], 500), [bark, oak]);
    const books = scheme.books;

    // bookcases along the walls under the windows, a shelf of books one colour
    for (const side of [-1, 1]) {
        const x = side * (half - 0.3);
        solid(build, [0.3, 1.1, ROOM_LENGTH / 2 - 0.5], bark, [x, 1.1, z - ROOM_LENGTH / 2]);
        for (const y of [0.15, 0.85, 1.55])
            build.scenery(
                look('box', [0.14, 0.26, ROOM_LENGTH / 2 - 0.9], pick(build, books), [
                    x - side * 0.24,
                    y + 0.26,
                    z - ROOM_LENGTH / 2,
                ]),
            );
    }

    // the front desk, with piles of returns
    const desk: Vec3 = [-half * 0.5, 0, z - 4];
    partsProp(build, [part('box', [1.4, 0.5, 0.4], [0, 0.5, 0], bark)], desk, 400);
    for (const at of [-0.5, 0.4])
        for (let b = 0; b < 5; b++)
            boxProp(
                build,
                [0.16, 0.035, 0.12],
                [desk[0] + at, 1.035 + b * 0.072, desk[2]],
                pick(build, books),
                500,
                build.random() * 0.4 - 0.2,
            );

    // stacks: rows across either side of the middle aisle in a wide library, or two long ones down it
    const stock: Stock[] = [
        { primitive: 'box', half: [0.06, 0.15, 0.11], density: 500 },
        { primitive: 'box', half: [0.08, 0.12, 0.1], density: 500 },
    ];
    const height = Math.min(2.2, place.height - 0.6);
    if (half >= 6)
        for (const at of [9, 13.5, 18])
            for (const side of [-1, 1])
                shelves(build, [side * (half / 2 + 0.4), 0, z - at], 0, half / 2 - 1.1, height, 4, bark, stock, books, false);
    else
        for (const side of [-1, 1])
            shelves(build, [side * half * 0.45, 0, z - 13.5], side * (Math.PI / 2), 4.5, height, 4, bark, stock, books, false);
    const cart: Vec3 = [between(build, -0.8, 0.8), 0, z - between(build, 11, 16)];
    partsProp(
        build,
        [part('box', [0.45, 0.04, 0.25], [0, 0.6, 0], bark), part('box', [0.45, 0.04, 0.25], [0, 0.25, 0], bark)],
        cart,
        400,
    );
    for (let b = 0; b < 7; b++)
        boxProp(build, [0.05, 0.13, 0.1], [cart[0] - 0.35 + b * 0.11, 0.77, cart[2]], pick(build, books), 500);

    // the reading room: tables with lamps and piles of books
    for (const [i, x] of spread(half >= 6 ? 3 : 2, half - 1.6).entries()) {
        const at: Vec3 = [x, 0, z - (i % 2 === 0 ? 24 : 27.5)];
        diningSet(build, at, 1.0, oak, bark, (top) => {
            partsProp(
                build,
                [
                    part('cylinder', [0.1, 0.02, 0.1], [0, 0.02, 0], ink),
                    part('cylinder', [0.015, 0.2, 0.015], [0, 0.22, 0], ink),
                    part('cylinder', [0.15, 0.1, 0.15], [0, 0.46, 0], butter),
                ],
                [at[0] - 0.6, top, at[2]],
                400,
            );
            for (let b = 0; b < 4; b++)
                boxProp(build, [0.16, 0.035, 0.12], [at[0] + 0.3, top + 0.035 + b * 0.072, at[2]], pick(build, books), 500);
        });
    }
}

/** a long narrow diner: booths down one wall, the counter and its stools down the other, and the aisle between */
function diner(place: Place) {
    const { build, z, half } = place;
    const scheme = pick(build, [
        { wall: blush, seat: terracotta, top: cream, front: butter },
        { wall: sage, seat: denim, top: cream, front: blush },
        { wall: sky, seat: terracotta, top: butter, front: cream },
    ]);
    shell(place, { floor: cream, wall: scheme.wall, front: scheme.front, sill: 1.1 });
    entrance(place, () => blockWall(place, [0.4, 0.15, 0.2], place.height, [terracotta, blush], 600), [sky, cream]);
    const dress = (center: Vec3, top: number, spread: number) => {
        for (const side of [-1, 1])
            cylinderProp(build, 0.16, 0.012, [center[0] + side * spread, top + 0.012, center[2]], cream, 900);
        cylinderProp(build, 0.05, 0.07, [center[0] + 0.1, top + 0.07, center[2] + 0.15], scheme.seat, 500);
        cylinderProp(build, 0.04, 0.1, [center[0] - 0.05, top + 0.1, center[2] - 0.12], terracotta, 500);
        cylinderProp(build, 0.04, 0.1, [center[0] + 0.05, top + 0.1, center[2] - 0.12], butter, 500);
    };

    // booths down the left: a table between two benches facing each other
    const boothX = -(half - 0.75);
    for (let at = 3; at < 27; at += 3.6) {
        for (const facing of [-1, 1])
            partsProp(
                build,
                [
                    part('box', [0.7, 0.22, 0.25], [0, 0.22, 0], scheme.seat),
                    part('box', [0.7, 0.4, 0.08], [0, 0.82, facing * 0.2], scheme.seat),
                ],
                [boothX, 0, z - at - 0.9 - facing * 0.9],
                300,
            );
        partsProp(
            build,
            [part('box', [0.65, 0.03, 0.4], [0, 0.74, 0], scheme.top), part('cylinder', [0.06, 0.36, 0.06], [0, 0.36, 0], stone)],
            [boothX, 0, z - at - 0.9],
            400,
        );
        dress([boothX, 0, z - at - 0.9], 0.77, 0.3);
    }

    // the counter down the right, with pies and cups along it and stools in front
    const counter: Vec3 = [half - 0.9, 0, z - 17];
    const halfLength = 10;
    partsProp(
        build,
        [
            part('box', [0.45, 0.5, halfLength], [0, 0.5, 0], scheme.top),
            part('box', [0.5, 0.04, halfLength + 0.1], [0, 1.04, 0], scheme.seat),
        ],
        counter,
        400,
    );
    for (let along = -halfLength + 0.7; along <= halfLength - 0.7; along += 1.4) {
        partsProp(
            build,
            [
                part('cylinder', [0.2, 0.05, 0.2], [0, 0.75, 0], scheme.seat),
                part('cylinder', [0.04, 0.35, 0.04], [0, 0.35, 0], stone),
            ],
            [counter[0] - 0.95, 0, counter[2] + along],
            400,
        );
        if (build.random() < 0.5) {
            cylinderProp(build, 0.2, 0.02, [counter[0], 1.1, counter[2] + along + 0.4], cream, 600);
            sphereProp(build, 0.16, [counter[0], 1.28, counter[2] + along + 0.4], butter, 300);
        } else cylinderProp(build, 0.05, 0.07, [counter[0], 1.15, counter[2] + along], cream, 500);
    }
}

/** racking down both walls stacked with boxes, pallets and a forklift on the floor, barrels and cones */
function warehouse(place: Place) {
    const { build, z, half } = place;
    shell(place, { floor: linen, wall: stone, front: pick(build, [denim, moss]), sill: 3 });
    entrance(place, () => blockWall(place, [0.5, 0.35, 0.35], place.height, CARDBOARD, 120), [stone, linen]);
    // lines down the floor
    for (const x of [-1.6, 1.6])
        build.scenery(look('box', [0.06, 0.01, ROOM_LENGTH / 2], butter, [x, 0.005, z - ROOM_LENGTH / 2]));

    // racking: tall frames with a level of boxes on each beam, down both walls, and down the middle of a wide one
    const racks: [number, number][] = [];
    for (const side of [-1, 1]) {
        for (const at of [5, 13, 21]) racks.push([side * (half - 1), at]);
        if (half >= 9.5) racks.push([side * 3, 9]);
    }
    for (const [x, at] of racks) {
        const parts: Part[] = [];
        for (const end of [-1, 1]) parts.push(part('box', [0.6, 2.15, 0.05], [0, 2.15, end * 3], denim));
        for (const y of [0.1, 1.4, 2.7, 4.0]) parts.push(part('box', [0.56, 0.05, 2.97], [0, y, 0], terracotta));
        partsProp(build, parts, [x, 0, z - at], 400);
        for (const y of [0.15, 1.45, 2.75])
            for (let along = -2.5; along <= 2.5; along += 1.25)
                boxProp(build, [0.4, 0.3, 0.3], [x, y + 0.31, z - at + along], pick(build, CARDBOARD), 120);
    }

    // pallets of boxes on the floor
    for (const at of [8, 17, 25]) {
        const x = between(build, -1, 1) * (half - 3.5);
        boxProp(build, [0.65, 0.07, 0.65], [x, 0.07, z - at], oak, 400);
        const layers = 2 + Math.floor(build.random() * 3);
        for (let layer = 0; layer < layers; layer++)
            for (const sx of [-1, 1])
                for (const sz of [-1, 1])
                    boxProp(
                        build,
                        [0.3, 0.24, 0.3],
                        [x + sx * 0.31, 0.38 + layer * 0.485, z - at + sz * 0.31],
                        pick(build, CARDBOARD),
                        120,
                    );
    }

    // a forklift, parked
    partsProp(
        build,
        [
            part('box', [0.55, 0.4, 0.9], [0, 0.6, 0], butter),
            part('box', [0.5, 0.05, 0.6], [0, 1.9, 0.2], ink),
            part('box', [0.04, 0.6, 0.04], [-0.45, 1.35, 0.75], ink),
            part('box', [0.04, 0.6, 0.04], [0.45, 1.35, 0.75], ink),
            part('box', [0.04, 0.95, 0.04], [-0.3, 1.0, -0.95], ink),
            part('box', [0.04, 0.95, 0.04], [0.3, 1.0, -0.95], ink),
            part('box', [0.08, 0.03, 0.55], [-0.25, 0.1, -1.5], stone),
            part('box', [0.08, 0.03, 0.55], [0.25, 0.1, -1.5], stone),
            part('cylinder', [0.25, 0.1, 0.25], [-0.6, 0.25, -0.55], ink, ACROSS),
            part('cylinder', [0.25, 0.1, 0.25], [0.6, 0.25, -0.55], ink, ACROSS),
            part('cylinder', [0.22, 0.1, 0.22], [-0.6, 0.22, 0.6], ink, ACROSS),
            part('cylinder', [0.22, 0.1, 0.22], [0.6, 0.22, 0.6], ink, ACROSS),
        ],
        [between(build, -2.5, 2.5), 0, z - 12.5],
        250,
        between(build, -0.5, 0.5),
    );

    // barrels, and cones across a bay
    const x0 = between(build, -2.5, 2.5);
    for (let i = 0; i < 4; i++)
        for (let j = 0; j < 2; j++) cylinderProp(build, 0.3, 0.45, [x0 - 0.93 + i * 0.62, 0.45, z - 28.5 - j * 0.62], denim, 300);
    for (const x of spread(Math.round(half / 0.65), half - 1.2)) cone(build, [x, 0, z - 20.5]);
}

/** a house along one side, a patio with a table under an umbrella, a barbecue, gnomes, a paddling pool, a fence across and a shed */
function backyard(place: Place) {
    const { build, z, half } = place;
    outdoors(place, sage, half + 8);
    const accents = pick(build, [
        [terracotta, butter],
        [denim, blush],
    ]);
    // the fence and the pool, one way round or the other
    const fenceFirst = build.random() < 0.5;
    // a tall board fence to burst through
    for (let x = -half + 0.08; x <= half - 0.08; x += 0.165)
        boxProp(build, [0.08, 0.9, 0.025], [x, 0.9, z - 0.6], pick(build, [oak, bark]), 500);

    // the house along the left: a wall with windows and a door, under a roof
    const middle = z - ROOM_LENGTH / 2;
    const house: Vec3 = [3, 2.5, ROOM_LENGTH / 2 - 1];
    building(build, [-(half + 3), 2.5, middle], house, pick(build, [cream, blush, butter]), true);
    build.fixture(box.create({ halfExtents: house }), [-(half + 3), 2.5, middle], []);
    build.scenery(look('box', [0.05, 1.1, 0.55], bark, [-half + 0.03, 1.1, middle]));
    // flower beds along it
    for (let along = 2; along < ROOM_LENGTH - 2; along += 0.8)
        build.scenery(look('sphere', [0.3, 0.25, 0.3], pick(build, [...accents, moss]), [-half + 0.5, 0.12, z - along]));
    // a hedge down the right
    solid(build, [0.6, 0.9, ROOM_LENGTH / 2], moss, [half + 0.6, 0.9, middle]);

    // the patio: a table under an umbrella, with chairs
    const patio: Vec3 = [-half * 0.45, 0, z - 7];
    build.scenery(look('box', [2.4, 0.02, 2.4], linen, [patio[0], 0.01, patio[2]]));
    partsProp(
        build,
        [
            part('cylinder', [0.6, 0.03, 0.6], [0, 0.72, 0], cream),
            part('cylinder', [0.04, 0.36, 0.04], [0, 0.36, 0], stone),
            part('cylinder', [0.03, 0.8, 0.03], [0, 1.55, 0], stone),
            part('cylinder', [1.3, 0.06, 1.3], [0, 2.3, 0], accents[0]!),
        ],
        patio,
        200,
    );
    for (const angle of [0, Math.PI / 2, Math.PI, -Math.PI / 2])
        chair(build, placed(patio, angle, [0, 0, 1]), angle + Math.PI, cream);

    // a barbecue on three legs
    partsProp(
        build,
        [
            part('sphere', [0.35, 0.35, 0.35], [0, 0.85, 0], ink),
            part('cylinder', [0.03, 0.4, 0.03], [0.2, 0.4, 0.15], ink),
            part('cylinder', [0.03, 0.4, 0.03], [-0.2, 0.4, 0.15], ink),
            part('cylinder', [0.03, 0.4, 0.03], [0, 0.4, -0.25], ink),
        ],
        [half * 0.4, 0, z - 9],
        300,
    );

    // gnomes on the lawn
    for (let g = 0; g < 4; g++)
        partsProp(
            build,
            [
                part('cylinder', [0.12, 0.15, 0.12], [0, 0.15, 0], denim),
                part('sphere', [0.1, 0.1, 0.1], [0, 0.4, 0], blush),
                part('box', [0.08, 0.12, 0.08], [0, 0.6, 0], terracotta),
                part('sphere', [0.07, 0.07, 0.07], [0, 0.3, 0.08], cream),
            ],
            [between(build, -1, 1) * (half - 1.5), 0, z - between(build, 12, 15)],
            400,
            build.random() * 6,
        );

    // a paddling pool full of balls
    const pool: Vec3 = [between(build, 0, half - 2.5), 0, z - (fenceFirst ? 20 : 13)];
    for (let i = 0; i < 12; i++) {
        const angle = (i / 12) * Math.PI * 2;
        cylinderProp(build, 0.12, 0.12, [pool[0] + Math.cos(angle) * 1.1, 0.12, pool[2] + Math.sin(angle) * 1.1], sky, 100);
    }
    for (let b = 0; b < 8; b++)
        sphereProp(
            build,
            0.18,
            [pool[0] + between(build, -0.6, 0.6), 0.2 + b * 0.05, pool[2] + between(build, -0.6, 0.6)],
            pick(build, [...accents, cream]),
            60,
            { restitution: 0.7 },
        );

    // a picket fence across, in panels
    for (const x of spread(Math.max(2, Math.round(half / 0.81)), half)) {
        const parts: Part[] = [
            part('box', [0.8, 0.04, 0.02], [0, 0.3, -0.04], cream),
            part('box', [0.8, 0.04, 0.02], [0, 0.8, -0.04], cream),
        ];
        for (let p = 0; p < 5; p++) parts.push(part('box', [0.06, 0.5, 0.02], [-0.64 + p * 0.32, 0.5, 0], cream));
        partsProp(build, parts, [x, 0, z - (fenceFirst ? 11 : 18)], 300);
    }

    // a shed of planks, with a roof, to run through
    const shed: Vec3 = [between(build, -1, 1) * (half - 3), 0, z - 26];
    for (let course = 0; course < 9; course++) {
        const y = 0.12 + course * 0.245;
        for (const side of [-1, 1]) {
            boxProp(build, [1.5, 0.12, 0.05], [shed[0], y, shed[2] + side * 1.45], oak, 400);
            boxProp(build, [0.05, 0.12, 1.4], [shed[0] + side * 1.45, y, shed[2]], oak, 400);
        }
    }
    boxProp(build, [1.7, 0.06, 1.7], [shed[0], 9 * 0.245 + 0.07, shed[2]], bark, 300);
}

/** a bed and a wardrobe against the walls, a rug in the middle with towers of blocks and a train, dominoes, a toy chest and a desk */
function bedroom(place: Place) {
    const { build, z, half } = place;
    const scheme = pick(build, [
        { wall: blush, floor: oak, front: cream, toys: [denim, butter, sage], rug: sky },
        { wall: sage, floor: linen, front: blush, toys: [terracotta, butter, sky], rug: cream },
        { wall: sky, floor: oak, front: butter, toys: [terracotta, cream, moss], rug: blush },
    ]);
    const toys = scheme.toys;
    shell(place, { floor: scheme.floor, wall: scheme.wall, front: scheme.front, sill: 1.0 });
    entrance(place, () => blockWall(place, [0.35, 0.35, 0.35], place.height, toys, 80), [cream, oak]);
    // the rug with its towers and train near the door and the toy chest further in, or the other way round
    const rugAt = build.random() < 0.5 ? 9 : 21;
    const toysAt = rugAt === 9 ? 20.5 : 9;
    const rug = Math.min(2.6, half - 0.6);
    build.scenery(look('cylinder', [rug, 0.01, rug], scheme.rug, [0, 0.012, z - rugAt]));

    // the bed, along the right wall
    partsProp(
        build,
        [
            part('box', [0.95, 0.18, 1.25], [0, 0.18, 0], bark),
            part('box', [0.9, 0.12, 1.2], [0, 0.48, 0], cream),
            part('box', [0.92, 0.04, 0.8], [0, 0.63, 0.38], toys[0]!),
            part('box', [0.4, 0.08, 0.2], [0, 0.68, -0.92], cream),
            part('box', [0.95, 0.55, 0.06], [0, 0.55, -1.28], bark),
        ],
        [half - 1.0, 0, z - 5.5],
        250,
    );
    // a wardrobe on the left
    partsProp(build, [part('box', [0.35, 1.0, 0.8], [0, 1.0, 0], oak)], [-half + 0.4, 0, z - 5], 200);

    // towers of blocks on the rug, and a train going round it
    for (let t = 0; t < 4; t++) {
        const angle = (t / 4) * Math.PI * 2 + build.random();
        const x = Math.cos(angle) * rug * 0.5;
        const at = z - rugAt + Math.sin(angle) * rug * 0.5;
        const height = 4 + Math.floor(build.random() * 4);
        for (let b = 0; b < height; b++)
            boxProp(build, [0.18, 0.18, 0.18], [x, 0.18 + b * 0.362, at], toys[b % toys.length]!, 150, b * 0.3);
    }
    for (let c = 0; c < 4; c++) {
        const angle = (c / 4) * 1.2;
        const center: Vec3 = [Math.cos(angle) * rug * 0.85, 0, z - rugAt + Math.sin(angle) * rug * 0.85];
        const parts: Part[] = [part('box', [0.16, 0.12, 0.26], [0, 0.2, 0], toys[c % toys.length]!)];
        if (c === 0) parts.push(part('box', [0.15, 0.1, 0.1], [0, 0.42, 0.13], ink));
        for (const sx of [-1, 1])
            for (const sz of [-1, 1]) parts.push(part('cylinder', [0.07, 0.03, 0.07], [sx * 0.18, 0.07, sz * 0.16], ink, ACROSS));
        partsProp(build, parts, center, 300, -angle);
    }

    // a line of dominoes snaking across
    const z0 = z - 15;
    for (let x = -half + 0.4; x <= half - 0.4; x += 0.36) {
        const at = z0 + 1.2 * Math.sin(x * 0.6);
        boxProp(
            build,
            [0.16, 0.34, 0.04],
            [x, 0.34, at],
            pick(build, [cream, ink]),
            300,
            Math.PI / 2 - Math.atan(0.72 * Math.cos(x * 0.6)),
        );
    }

    // a toy chest spilling over, teddies, and beach balls
    const chest: Vec3 = [-half * 0.5, 0, z - toysAt];
    const chestHalf: Vec3 = [0.55, 0.28, 0.38];
    crate(build, chest, chestHalf, terracotta);
    fillWithBalls(build, chest, chestHalf, 0.11, 2, toys, 80);
    for (let t = 0; t < 3; t++) {
        const fur = pick(build, [oak, butter, blush]);
        partsProp(
            build,
            [
                part('sphere', [0.2, 0.2, 0.2], [0, 0.2, 0], fur),
                part('sphere', [0.15, 0.15, 0.15], [0, 0.5, 0], fur),
                part('sphere', [0.055, 0.055, 0.055], [-0.1, 0.64, 0], fur),
                part('sphere', [0.055, 0.055, 0.055], [0.1, 0.64, 0], fur),
            ],
            [between(build, 0, half - 1), 0, z - toysAt + between(build, -1.5, 1.5)],
            100,
        );
    }
    for (let b = 0; b < 4; b++) {
        const radius = between(build, 0.25, 0.4);
        sphereProp(
            build,
            radius,
            [between(build, -1, 1) * (half - 0.6), radius, z - between(build, 23, 26)],
            pick(build, toys),
            60,
            { restitution: 0.75 },
        );
    }

    // a desk with a lamp and a chair, under the window at the back
    const desk: Vec3 = [between(build, -1, 1) * (half - 1.5), 0, z - 29];
    table(build, desk, 0, 0.9, 0.45, 0.75, cream);
    chair(build, [desk[0], 0, desk[2] + 0.75], Math.PI, toys[1]!);
    for (let b = 0; b < 4; b++)
        boxProp(build, [0.15, 0.03, 0.11], [desk[0] + 0.5, 0.78 + b * 0.062, desk[2]], pick(build, toys), 500);
}

/** a road between shopfronts, parked cars along both kerbs, lamp posts and bins, and roadworks across it */
function street(place: Place) {
    const { build, z, half } = place;
    const pavement = 3.5;
    outdoors(place, ink, half + pavement + 4);
    const middle = z - ROOM_LENGTH / 2;
    // barriers across the road to burst through
    for (const x of spread(Math.max(2, Math.round(half / 0.65)), half)) barrier(build, [x, 0, z - 0.6]);

    // lines down the road, pavements, and shopfronts with awnings
    for (let along = 1; along < ROOM_LENGTH; along += 4)
        build.scenery(look('box', [0.08, 0.01, 1], cream, [0, 0.005, z - along - 1]));
    for (const side of [-1, 1]) {
        build.scenery(look('box', [pavement / 2, 0.08, ROOM_LENGTH / 2], linen, [side * (half + pavement / 2), 0.08, middle]));
        let along = 0;
        while (along < ROOM_LENGTH - 4) {
            const length = between(build, 5, 8);
            const center: Vec3 = [side * (half + pavement + 1.5), 2.4, z - along - length / 2];
            solid(build, [1.5, 2.4, length / 2 - 0.05], pick(build, BUILDING_TONES), center);
            build.scenery(look('box', [0.05, 0.9, length / 2 - 0.8], sky, [center[0] - side * 1.52, 1.4, center[2]]));
            build.scenery(
                look(
                    'box',
                    [0.7, 0.04, length / 2 - 0.4],
                    pick(build, [terracotta, denim, moss, butter]),
                    [center[0] - side * 2.1, 2.7, center[2]],
                    quat.setAxisAngle([0, 0, 0, 1], [0, 0, 1], side * 0.3),
                ),
            );
            along += length;
        }
    }

    // cars parked along both kerbs
    for (const side of [-1, 1])
        for (let along = 3 + build.random() * 2; along < ROOM_LENGTH - 3; along += between(build, 6, 9))
            car(build, [side * (half - 1.1), 0, z - along]);

    // lamp posts and bins on the pavements' edges
    for (const side of [-1, 1])
        for (let along = 4; along < ROOM_LENGTH; along += 8) {
            partsProp(
                build,
                [
                    part('box', [0.12, 0.05, 0.12], [0, 0.05, 0], ink),
                    part('cylinder', [0.05, 2.0, 0.05], [0, 2.0, 0], ink),
                    part('box', [0.35, 0.04, 0.04], [-side * 0.3, 3.95, 0], ink),
                    part('sphere', [0.14, 0.1, 0.14], [-side * 0.6, 3.86, 0], butter),
                ],
                [side * (half + 0.4), 0.16, z - along],
                600,
            );
            cylinderProp(build, 0.22, 0.38, [side * (half + 0.4), 0.54, z - along - 2], moss, 200);
        }

    // roadworks across the road: cones and barriers
    const works = build.random() < 0.5 ? 10 : 18;
    for (const x of spread(Math.round(half / 0.6), half - 2.2)) cone(build, [x, 0, z - works]);
    for (const x of spread(Math.max(1, Math.round(half / 2.2)), half - 2.5)) barrier(build, [x, 0, z - works - 1.2]);
    // a postbox
    partsProp(
        build,
        [part('box', [0.22, 0.32, 0.22], [0, 0.95, 0], terracotta), part('box', [0.05, 0.32, 0.05], [0, 0.32, 0], ink)],
        [half + 0.9, 0.16, z - 24],
        400,
    );
}

function car(build: Builder, center: Vec3) {
    const paint = pick(build, [terracotta, sky, butter, sage, cream, denim]);
    const parts: Part[] = [
        part('box', [0.85, 0.32, 2.0], [0, 0.6, 0], paint),
        part('box', [0.75, 0.3, 1.05], [0, 1.2, 0.15], paint),
        part('box', [0.77, 0.22, 0.02], [0, 1.2, -0.92], ink),
        part('box', [0.77, 0.22, 0.02], [0, 1.2, 1.22], ink),
    ];
    for (const sx of [-1, 1])
        for (const sz of [-1, 1]) parts.push(part('cylinder', [0.3, 0.12, 0.3], [sx * 0.8, 0.3, sz * 1.3], ink, ACROSS));
    partsProp(build, parts, center, 250, between(build, -0.1, 0.1));
}

const BUILDERS: Record<RoomKind, (place: Place) => void> = {
    supermarket,
    library,
    diner,
    warehouse,
    backyard,
    bedroom,
    street,
};

function mirroredQuat(q: Quat): Quat {
    return [q[0], -q[1], -q[2], q[3]];
}

/** the same builder with everything it is given flipped left for right */
function mirrored(build: Builder): Builder {
    return {
        random: build.random,
        prop: (shape, position, quaternion, looks, material) =>
            build.prop(shape, [-position[0], position[1], position[2]], mirroredQuat(quaternion), looks, material),
        fixture: (shape, position, looks) => build.fixture(shape, [-position[0], position[1], position[2]], looks),
        scenery: (look) =>
            build.scenery({
                ...look,
                offset: [-look.offset[0], look.offset[1], look.offset[2]],
                rotation: mirroredQuat(look.rotation),
            }),
    };
}

/** a room of the kind and size given, its entrance at zStart and running on toward -z, as laid out or flipped left for right */
export function buildRoom(build: Builder, kind: RoomKind, zStart: number, size: RoomSize, previous: RoomSize): void {
    const placeBuild = build.random() < 0.5 ? mirrored(build) : build;
    BUILDERS[kind]({ build: placeBuild, z: zStart, half: size.half, height: size.height, previous });
}
