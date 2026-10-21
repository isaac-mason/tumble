import * as Low from './generated/tumble.js';
import type { Quat, Vec3 } from './math.js';
import type { BodyId } from './rigid-body.js';
import type { Shape } from './shapes.js';
import type { World } from './world.js';

export const CastRayStatus = Low.CastRayStatus;
export type CastRayStatus = Low.CastRayStatus;
export const CastShapeStatus = Low.CastShapeStatus;
export type CastShapeStatus = Low.CastShapeStatus;

/**
 * what a query may hit: the enabled layers, held in the wasm (free with Filter.destroy) and changed with Filter's
 * functions, then the group and mask a body must match, then the body filter
 */
export type Filter = {
    readonly layers: Low.Filter;
    /** the groups the query is in: a body hits when its mask has one of them, and its groups one of collisionMask's */
    collisionGroups: number;
    collisionMask: number;
    /**
     * false to pass over a body. a query with one collects every hit and keeps those the filter passes, so the closest
     * kept is the closest the filter passes, at the cost of the early out
     */
    bodyFilter: ((body: BodyId) => boolean) | null;
};

function readGroups(filter: Filter): void {
    filter.collisionGroups = Low.Filter.getCollisionGroups(filter.layers);
    filter.collisionMask = Low.Filter.getCollisionMask(filter.layers);
}

export const Filter = {
    /** every layer of the world's enabled, every group and mask bit set, no body filter */
    create(world: World): Filter {
        Low.errorClear();
        const filter: Filter = { layers: Low.Filter.create(world), collisionGroups: 0, collisionMask: 0, bodyFilter: null };
        readGroups(filter);
        return filter;
    },
    destroy(filter: Filter): void {
        Low.Filter.destroy(filter.layers);
    },
    /** as created: every layer enabled, every group and mask bit set, no body filter */
    setAllEnabled(filter: Filter, world: World): void {
        Low.Filter.enableAllLayers(filter.layers, world);
        filter.collisionGroups = 0xffffffff;
        filter.collisionMask = 0xffffffff;
        filter.bodyFilter = null;
    },
    enableAllLayers(filter: Filter, world: World): void {
        Low.Filter.enableAllLayers(filter.layers, world);
    },
    disableAllLayers(filter: Filter, world: World): void {
        Low.Filter.disableAllLayers(filter.layers, world);
    },
    /** the object layer and its broadphase layer; false for a layer the world does not have */
    enableObjectLayer(filter: Filter, world: World, objectLayer: number): boolean {
        return Low.Filter.enableObjectLayer(filter.layers, world, objectLayer);
    },
    /** the object layer, and its broadphase layer when no other enabled object layer is in it */
    disableObjectLayer(filter: Filter, world: World, objectLayer: number): boolean {
        return Low.Filter.disableObjectLayer(filter.layers, world, objectLayer);
    },
    /** the broadphase layer and every object layer in it */
    enableBroadphaseLayer(filter: Filter, world: World, broadphaseLayer: number): boolean {
        return Low.Filter.enableBroadphaseLayer(filter.layers, world, broadphaseLayer);
    },
    disableBroadphaseLayer(filter: Filter, world: World, broadphaseLayer: number): boolean {
        return Low.Filter.disableBroadphaseLayer(filter.layers, world, broadphaseLayer);
    },
    /** whether the object layer is enabled */
    filterObjectLayer(filter: Filter, objectLayer: number): boolean {
        return Low.Filter.isObjectLayerEnabled(filter.layers, objectLayer);
    },
    filterBroadphaseLayer(filter: Filter, broadphaseLayer: number): boolean {
        return Low.Filter.isBroadphaseLayerEnabled(filter.layers, broadphaseLayer);
    },
    /** what the body collides with: the layers its object layer does, and its groups and mask; false when the id names no body */
    setFromBody(filter: Filter, world: World, body: BodyId): boolean {
        if (!Low.Filter.setFromBody(filter.layers, world, body)) return false;
        readGroups(filter);
        return true;
    },
    copy(out: Filter, source: Filter): void {
        Low.errorClear();
        if (!Low.Filter.copy(out.layers, source.layers)) Low.getWasm().check();
        out.collisionGroups = source.collisionGroups;
        out.collisionMask = source.collisionMask;
        out.bodyFilter = source.bodyFilter;
    },
};

/** what a query given no filter hits: every layer, group and body */
const EVERYTHING: Filter = { layers: 0 as Low.Filter, collisionGroups: 0xffffffff, collisionMask: 0xffffffff, bodyFilter: null };

function copyVec3(out: Vec3, from: Vec3): void {
    out[0] = from[0];
    out[1] = from[1];
    out[2] = from[2];
}

/** hits kept by an all collector: the array, and the objects behind it, reused across resets */
class Pool<Hit> {
    readonly hits: Hit[] = [];
    readonly #spare: Hit[] = [];
    readonly #create: () => Hit;
    constructor(create: () => Hit) {
        this.#create = create;
    }
    /** a hit object at the end of hits, to fill */
    next(): Hit {
        let hit = this.#spare[this.hits.length];
        if (hit === undefined) {
            hit = this.#create();
            this.#spare.push(hit);
        }
        this.hits.push(hit);
        return hit;
    }
    reset(): void {
        this.hits.length = 0;
    }
}

// cast ray

export type CastRaySettings = Low.CastRaySettings;

export type CastRayHit = {
    status: CastRayStatus;
    /** along the ray: 0 at the origin, 1 at its length */
    fraction: number;
    subShapeId: number;
    bodyIdB: BodyId;
    /** -1 for none */
    materialId: number;
};

export function createDefaultCastRaySettings(): CastRaySettings {
    return Low.Query.defaultCastRaySettings(Low.CastRaySettings.create());
}

function createCastRayHit(): CastRayHit {
    return { status: CastRayStatus.NOT_COLLIDING, fraction: 1, subShapeId: 0, bodyIdB: 0, materialId: -1 };
}

function clearCastRayHit(hit: CastRayHit): void {
    hit.status = CastRayStatus.NOT_COLLIDING;
    hit.fraction = 1;
    hit.subShapeId = 0;
    hit.bodyIdB = 0;
    hit.materialId = -1;
}

function copyCastRayHit(out: CastRayHit, from: Low.CastRayHit): void {
    out.status = from.status as CastRayStatus;
    out.fraction = from.fraction;
    out.subShapeId = from.subShapeId;
    out.bodyIdB = from.bodyIdB;
    out.materialId = from.materialId;
}

export class ClosestCastRayCollector {
    readonly kind = Low.CollectorKind.CLOSEST;
    hit = createCastRayHit();
    earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    addHit(hit: Low.CastRayHit): void {
        if (this.hit.status === CastRayStatus.NOT_COLLIDING || hit.fraction < this.hit.fraction) {
            copyCastRayHit(this.hit, hit);
            this.earlyOutFraction = hit.fraction;
        }
    }
    shouldEarlyOut(): boolean {
        return false;
    }
    reset(): void {
        clearCastRayHit(this.hit);
        this.earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    }
}

export class AnyCastRayCollector {
    readonly kind = Low.CollectorKind.ANY;
    hit = createCastRayHit();
    earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    addHit(hit: Low.CastRayHit): void {
        if (this.hit.status === CastRayStatus.COLLIDING) return;
        copyCastRayHit(this.hit, hit);
        this.earlyOutFraction = 0;
    }
    shouldEarlyOut(): boolean {
        return this.hit.status === CastRayStatus.COLLIDING;
    }
    reset(): void {
        clearCastRayHit(this.hit);
        this.earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    }
}

export class AllCastRayCollector {
    readonly kind = Low.CollectorKind.ALL;
    readonly #pool = new Pool(createCastRayHit);
    readonly hits: readonly CastRayHit[] = this.#pool.hits;
    earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    addHit(hit: Low.CastRayHit): void {
        copyCastRayHit(this.#pool.next(), hit);
    }
    shouldEarlyOut(): boolean {
        return false;
    }
    /** closest first */
    sort(): void {
        this.#pool.hits.sort((a, b) => a.fraction - b.fraction);
    }
    reset(): void {
        this.#pool.reset();
        this.earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    }
}

export type CastRayCollector = ClosestCastRayCollector | AnyCastRayCollector | AllCastRayCollector;

export function createClosestCastRayCollector(): ClosestCastRayCollector {
    return new ClosestCastRayCollector();
}
export function createAnyCastRayCollector(): AnyCastRayCollector {
    return new AnyCastRayCollector();
}
export function createAllCastRayCollector(): AllCastRayCollector {
    return new AllCastRayCollector();
}

const rayHit = Low.CastRayHit.create();

/** the ray from origin along the unit direction for length, its hits into the collector, which keeps any it had */
export function castRay(
    world: World,
    collector: CastRayCollector,
    settings: CastRaySettings,
    origin: Vec3,
    direction: Vec3,
    length: number,
    filter: Filter = EVERYTHING,
): void {
    if (collector.shouldEarlyOut()) return;
    const { bodyFilter } = filter;
    const kind = bodyFilter === null ? collector.kind : Low.CollectorKind.ALL;
    Low.errorClear();
    const count = Low.Query.castRay(
        world,
        kind,
        collector.earlyOutFraction,
        settings,
        origin,
        direction,
        length,
        filter.layers,
        filter.collisionGroups >>> 0,
        filter.collisionMask >>> 0,
    );
    Low.getWasm().check();
    for (let i = 0; i < count && !collector.shouldEarlyOut(); i++) {
        Low.Query.readCastRayHit(rayHit, i);
        if (bodyFilter === null || bodyFilter(rayHit.bodyIdB)) collector.addHit(rayHit);
    }
}

// cast shape

export type CastShapeSettings = Low.CastShapeSettings;

export type CastShapeHit = {
    status: CastShapeStatus;
    /** along the sweep: 0 at the start, 1 at the end */
    fraction: number;
    /** world space, on the swept shape */
    pointA: Vec3;
    /** world space, on the body hit */
    pointB: Vec3;
    penetrationDepth: number;
    /** the way to move the body hit out of collision, unnormalized; the contact normal a to b when fraction is above 0 */
    penetrationAxis: Vec3;
    /** from b towards a, unit */
    normal: Vec3;
    subShapeIdA: number;
    subShapeIdB: number;
    materialIdA: number;
    materialIdB: number;
    bodyIdB: BodyId;
};

export function createDefaultCastShapeSettings(): CastShapeSettings {
    return Low.Query.defaultCastShapeSettings(Low.CastShapeSettings.create());
}

function createCastShapeHit(): CastShapeHit {
    return {
        status: CastShapeStatus.NOT_COLLIDING,
        fraction: 1,
        pointA: [0, 0, 0],
        pointB: [0, 0, 0],
        penetrationDepth: 0,
        penetrationAxis: [0, 0, 0],
        normal: [0, 0, 0],
        subShapeIdA: 0,
        subShapeIdB: 0,
        materialIdA: -1,
        materialIdB: -1,
        bodyIdB: 0,
    };
}

function clearCastShapeHit(hit: CastShapeHit): void {
    hit.status = CastShapeStatus.NOT_COLLIDING;
    hit.fraction = 1;
    hit.penetrationDepth = 0;
    hit.subShapeIdA = 0;
    hit.subShapeIdB = 0;
    hit.materialIdA = -1;
    hit.materialIdB = -1;
    hit.bodyIdB = 0;
}

function copyCastShapeHit(out: CastShapeHit, from: Low.CastShapeHit): void {
    out.status = from.status as CastShapeStatus;
    out.fraction = from.fraction;
    copyVec3(out.pointA, from.pointA);
    copyVec3(out.pointB, from.pointB);
    out.penetrationDepth = from.penetrationDepth;
    copyVec3(out.penetrationAxis, from.penetrationAxis);
    copyVec3(out.normal, from.normal);
    out.subShapeIdA = from.subShapeIdA;
    out.subShapeIdB = from.subShapeIdB;
    out.materialIdA = from.materialIdA;
    out.materialIdB = from.materialIdB;
    out.bodyIdB = from.bodyIdB;
}

/** a hit's place in the closest order: its fraction, or for one touching at the start, the deeper the earlier */
function castShapeEarlyOut(hit: { fraction: number; penetrationDepth: number }): number {
    return hit.fraction > 0 ? hit.fraction : -hit.penetrationDepth;
}

export class ClosestCastShapeCollector {
    readonly kind = Low.CollectorKind.CLOSEST;
    hit = createCastShapeHit();
    earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    addHit(hit: Low.CastShapeHit): void {
        if (this.hit.status === CastShapeStatus.NOT_COLLIDING || castShapeEarlyOut(hit) < castShapeEarlyOut(this.hit)) {
            copyCastShapeHit(this.hit, hit);
            this.earlyOutFraction = castShapeEarlyOut(hit);
        }
    }
    shouldEarlyOut(): boolean {
        return false;
    }
    reset(): void {
        clearCastShapeHit(this.hit);
        this.earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    }
}

export class AnyCastShapeCollector {
    readonly kind = Low.CollectorKind.ANY;
    hit = createCastShapeHit();
    earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    addHit(hit: Low.CastShapeHit): void {
        if (this.hit.status === CastShapeStatus.COLLIDING) return;
        copyCastShapeHit(this.hit, hit);
        this.earlyOutFraction = -Low.UNBOUNDED_EARLY_OUT;
    }
    shouldEarlyOut(): boolean {
        return this.hit.status === CastShapeStatus.COLLIDING;
    }
    reset(): void {
        clearCastShapeHit(this.hit);
        this.earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    }
}

export class AllCastShapeCollector {
    readonly kind = Low.CollectorKind.ALL;
    readonly #pool = new Pool(createCastShapeHit);
    readonly hits: readonly CastShapeHit[] = this.#pool.hits;
    earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    addHit(hit: Low.CastShapeHit): void {
        copyCastShapeHit(this.#pool.next(), hit);
    }
    shouldEarlyOut(): boolean {
        return false;
    }
    /** closest first */
    sort(): void {
        this.#pool.hits.sort((a, b) => castShapeEarlyOut(a) - castShapeEarlyOut(b));
    }
    reset(): void {
        this.#pool.reset();
        this.earlyOutFraction = Low.INITIAL_EARLY_OUT_FRACTION;
    }
}

export type CastShapeCollector = ClosestCastShapeCollector | AnyCastShapeCollector | AllCastShapeCollector;

export function createClosestCastShapeCollector(): ClosestCastShapeCollector {
    return new ClosestCastShapeCollector();
}
export function createAnyCastShapeCollector(): AnyCastShapeCollector {
    return new AnyCastShapeCollector();
}
export function createAllCastShapeCollector(): AllCastShapeCollector {
    return new AllCastShapeCollector();
}

const shapeHit = Low.CastShapeHit.create();

/** the shape at the pose swept along the displacement, its hits into the collector, which keeps any it had */
export function castShape(
    world: World,
    collector: CastShapeCollector,
    settings: CastShapeSettings,
    shape: Shape,
    position: Vec3,
    quaternion: Quat,
    scale: Vec3,
    displacement: Vec3,
    filter: Filter = EVERYTHING,
): void {
    if (collector.shouldEarlyOut()) return;
    const { bodyFilter } = filter;
    const kind = bodyFilter === null ? collector.kind : Low.CollectorKind.ALL;
    Low.errorClear();
    const count = Low.Query.castShape(
        world,
        kind,
        collector.earlyOutFraction,
        settings,
        shape,
        position,
        quaternion,
        scale,
        displacement,
        filter.layers,
        filter.collisionGroups >>> 0,
        filter.collisionMask >>> 0,
    );
    Low.getWasm().check();
    for (let i = 0; i < count && !collector.shouldEarlyOut(); i++) {
        Low.Query.readCastShapeHit(shapeHit, i);
        if (bodyFilter === null || bodyFilter(shapeHit.bodyIdB)) collector.addHit(shapeHit);
    }
}

// collide shape

export type CollideShapeSettings = Low.CollideShapeSettings;

export type CollideShapeHit = {
    /** world space, on the query shape */
    pointA: Vec3;
    /** world space, on the body hit */
    pointB: Vec3;
    /** the way to move the body hit out of collision, unnormalized */
    penetrationAxis: Vec3;
    /** above 0 when overlapping; below 0 for a separation maxSeparationDistance allows */
    penetration: number;
    subShapeIdA: number;
    subShapeIdB: number;
    materialIdA: number;
    materialIdB: number;
    bodyIdB: BodyId;
};

export function createDefaultCollideShapeSettings(): CollideShapeSettings {
    return Low.Query.defaultCollideShapeSettings(Low.CollideShapeSettings.create());
}

function createCollideShapeHit(): CollideShapeHit {
    return {
        pointA: [0, 0, 0],
        pointB: [0, 0, 0],
        penetrationAxis: [0, 0, 0],
        penetration: 0,
        subShapeIdA: 0,
        subShapeIdB: 0,
        materialIdA: -1,
        materialIdB: -1,
        bodyIdB: 0,
    };
}

function copyCollideShapeHit(out: CollideShapeHit, from: Low.CollideShapeHit): void {
    copyVec3(out.pointA, from.pointA);
    copyVec3(out.pointB, from.pointB);
    copyVec3(out.penetrationAxis, from.penetrationAxis);
    out.penetration = from.penetration;
    out.subShapeIdA = from.subShapeIdA;
    out.subShapeIdB = from.subShapeIdB;
    out.materialIdA = from.materialIdA;
    out.materialIdB = from.materialIdB;
    out.bodyIdB = from.bodyIdB;
}

/** the deepest hit */
export class ClosestCollideShapeCollector {
    readonly kind = Low.CollectorKind.CLOSEST;
    hit: CollideShapeHit | null = null;
    earlyOutFraction = Low.UNBOUNDED_EARLY_OUT;
    readonly #hit = createCollideShapeHit();
    addHit(hit: Low.CollideShapeHit): void {
        if (this.hit === null || hit.penetration > this.hit.penetration) {
            copyCollideShapeHit(this.#hit, hit);
            this.hit = this.#hit;
            this.earlyOutFraction = -hit.penetration;
        }
    }
    shouldEarlyOut(): boolean {
        return false;
    }
    reset(): void {
        this.hit = null;
        this.earlyOutFraction = Low.UNBOUNDED_EARLY_OUT;
    }
}

export class AnyCollideShapeCollector {
    readonly kind = Low.CollectorKind.ANY;
    hit: CollideShapeHit | null = null;
    earlyOutFraction = Low.UNBOUNDED_EARLY_OUT;
    readonly #hit = createCollideShapeHit();
    addHit(hit: Low.CollideShapeHit): void {
        if (this.hit !== null) return;
        copyCollideShapeHit(this.#hit, hit);
        this.hit = this.#hit;
        this.earlyOutFraction = -Low.UNBOUNDED_EARLY_OUT;
    }
    shouldEarlyOut(): boolean {
        return this.hit !== null;
    }
    reset(): void {
        this.hit = null;
        this.earlyOutFraction = Low.UNBOUNDED_EARLY_OUT;
    }
}

export class AllCollideShapeCollector {
    readonly kind = Low.CollectorKind.ALL;
    readonly #pool = new Pool(createCollideShapeHit);
    readonly hits: readonly CollideShapeHit[] = this.#pool.hits;
    earlyOutFraction = Low.UNBOUNDED_EARLY_OUT;
    addHit(hit: Low.CollideShapeHit): void {
        copyCollideShapeHit(this.#pool.next(), hit);
    }
    shouldEarlyOut(): boolean {
        return false;
    }
    reset(): void {
        this.#pool.reset();
        this.earlyOutFraction = Low.UNBOUNDED_EARLY_OUT;
    }
}

export type CollideShapeCollector = ClosestCollideShapeCollector | AnyCollideShapeCollector | AllCollideShapeCollector;

export function createClosestCollideShapeCollector(): ClosestCollideShapeCollector {
    return new ClosestCollideShapeCollector();
}
export function createAnyCollideShapeCollector(): AnyCollideShapeCollector {
    return new AnyCollideShapeCollector();
}
export function createAllCollideShapeCollector(): AllCollideShapeCollector {
    return new AllCollideShapeCollector();
}

const collideHit = Low.CollideShapeHit.create();

function collide(
    world: World,
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shape: Shape,
    position: Vec3,
    quaternion: Quat,
    scale: Vec3,
    internalEdgeRemoval: boolean,
    filter: Filter,
): void {
    if (collector.shouldEarlyOut()) return;
    const { bodyFilter } = filter;
    const kind = bodyFilter === null ? collector.kind : Low.CollectorKind.ALL;
    Low.errorClear();
    const count = Low.Query.collideShape(
        world,
        kind,
        collector.earlyOutFraction,
        settings,
        shape,
        position,
        quaternion,
        scale,
        internalEdgeRemoval,
        filter.layers,
        filter.collisionGroups >>> 0,
        filter.collisionMask >>> 0,
    );
    Low.getWasm().check();
    for (let i = 0; i < count && !collector.shouldEarlyOut(); i++) {
        Low.Query.readCollideShapeHit(collideHit, i);
        if (bodyFilter === null || bodyFilter(collideHit.bodyIdB)) collector.addHit(collideHit);
    }
}

/** the shape at the pose against the world, its hits into the collector, which keeps any it had */
export function collideShape(
    world: World,
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shape: Shape,
    position: Vec3,
    quaternion: Quat,
    scale: Vec3,
    filter: Filter = EVERYTHING,
): void {
    collide(world, collector, settings, shape, position, quaternion, scale, false, filter);
}

/** collideShape without the hits on edges inside a mesh's surface, which would snag a shape sliding over it */
export function collideShapeWithInternalEdgeRemoval(
    world: World,
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shape: Shape,
    position: Vec3,
    quaternion: Quat,
    scale: Vec3,
    filter: Filter = EVERYTHING,
): void {
    collide(world, collector, settings, shape, position, quaternion, scale, true, filter);
}

// collide point

export type CollidePointSettings = Low.CollidePointSettings;

export type CollidePointHit = {
    bodyIdB: BodyId;
    /** the empty sub shape id unless the shape is a compound */
    subShapeIdB: number;
    materialId: number;
};

export function createDefaultCollidePointSettings(): CollidePointSettings {
    return Low.Query.defaultCollidePointSettings(Low.CollidePointSettings.create());
}

function createCollidePointHit(): CollidePointHit {
    return { bodyIdB: 0, subShapeIdB: 0, materialId: -1 };
}

function copyCollidePointHit(out: CollidePointHit, from: Low.CollidePointHit): void {
    out.bodyIdB = from.bodyIdB;
    out.subShapeIdB = from.subShapeIdB;
    out.materialId = from.materialId;
}

export class AnyCollidePointCollector {
    readonly kind = Low.CollectorKind.ANY;
    hit: CollidePointHit | null = null;
    readonly #hit = createCollidePointHit();
    addHit(hit: Low.CollidePointHit): void {
        if (this.hit !== null) return;
        copyCollidePointHit(this.#hit, hit);
        this.hit = this.#hit;
    }
    shouldEarlyOut(): boolean {
        return this.hit !== null;
    }
    reset(): void {
        this.hit = null;
    }
}

export class AllCollidePointCollector {
    readonly kind = Low.CollectorKind.ALL;
    readonly #pool = new Pool(createCollidePointHit);
    readonly hits: readonly CollidePointHit[] = this.#pool.hits;
    addHit(hit: Low.CollidePointHit): void {
        copyCollidePointHit(this.#pool.next(), hit);
    }
    shouldEarlyOut(): boolean {
        return false;
    }
    reset(): void {
        this.#pool.reset();
    }
}

export type CollidePointCollector = AnyCollidePointCollector | AllCollidePointCollector;

export function createAnyCollidePointCollector(): AnyCollidePointCollector {
    return new AnyCollidePointCollector();
}
export function createAllCollidePointCollector(): AllCollidePointCollector {
    return new AllCollidePointCollector();
}

const pointHit = Low.CollidePointHit.create();

/** the bodies whose shapes hold the point, into the collector, which keeps any it had */
export function collidePoint(
    world: World,
    collector: CollidePointCollector,
    settings: CollidePointSettings,
    point: Vec3,
    filter: Filter = EVERYTHING,
): void {
    if (collector.shouldEarlyOut()) return;
    const { bodyFilter } = filter;
    const kind = bodyFilter === null ? collector.kind : Low.CollectorKind.ALL;
    Low.errorClear();
    const count = Low.Query.collidePoint(
        world,
        kind,
        settings,
        point,
        filter.layers,
        filter.collisionGroups >>> 0,
        filter.collisionMask >>> 0,
    );
    Low.getWasm().check();
    for (let i = 0; i < count && !collector.shouldEarlyOut(); i++) {
        Low.Query.readCollidePointHit(pointHit, i);
        if (bodyFilter === null || bodyFilter(pointHit.bodyIdB)) collector.addHit(pointHit);
    }
}
