import type { BodyId } from './body.js';
import * as Low from './generated/tumble.js';
import type { World } from './world.js';

/**
 * what a query may hit: the enabled layers, held in the wasm (free with filter.destroy) and changed with filter's
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

/** every layer of the world's enabled, every group and mask bit set, no body filter */
export function create(world: World): Filter {
    Low.errorClear();
    const filter: Filter = { layers: Low.Filter.create(world), collisionGroups: 0, collisionMask: 0, bodyFilter: null };
    readGroups(filter);
    return filter;
}

export function destroy(filter: Filter): void {
    Low.Filter.destroy(filter.layers);
}

/** as created: every layer enabled, every group and mask bit set, no body filter */
export function setAllEnabled(filter: Filter, world: World): void {
    Low.Filter.enableAllLayers(filter.layers, world);
    filter.collisionGroups = 0xffffffff;
    filter.collisionMask = 0xffffffff;
    filter.bodyFilter = null;
}

export function enableAllLayers(filter: Filter, world: World): void {
    Low.Filter.enableAllLayers(filter.layers, world);
}

export function disableAllLayers(filter: Filter, world: World): void {
    Low.Filter.disableAllLayers(filter.layers, world);
}

/** the object layer and its broadphase layer; false for a layer the world does not have */
export function enableObjectLayer(filter: Filter, world: World, objectLayer: number): boolean {
    return Low.Filter.enableObjectLayer(filter.layers, world, objectLayer);
}

/** the object layer, and its broadphase layer when no other enabled object layer is in it */
export function disableObjectLayer(filter: Filter, world: World, objectLayer: number): boolean {
    return Low.Filter.disableObjectLayer(filter.layers, world, objectLayer);
}

/** the broadphase layer and every object layer in it */
export function enableBroadphaseLayer(filter: Filter, world: World, broadphaseLayer: number): boolean {
    return Low.Filter.enableBroadphaseLayer(filter.layers, world, broadphaseLayer);
}

export function disableBroadphaseLayer(filter: Filter, world: World, broadphaseLayer: number): boolean {
    return Low.Filter.disableBroadphaseLayer(filter.layers, world, broadphaseLayer);
}

/** whether the object layer is enabled */
export function filterObjectLayer(filter: Filter, objectLayer: number): boolean {
    return Low.Filter.isObjectLayerEnabled(filter.layers, objectLayer);
}

export function filterBroadphaseLayer(filter: Filter, broadphaseLayer: number): boolean {
    return Low.Filter.isBroadphaseLayerEnabled(filter.layers, broadphaseLayer);
}

/** what the body collides with: the layers its object layer does, and its groups and mask; false when the id names no body */
export function setFromBody(filter: Filter, world: World, body: BodyId): boolean {
    if (!Low.Filter.setFromBody(filter.layers, world, body)) return false;
    readGroups(filter);
    return true;
}

export function copy(out: Filter, source: Filter): void {
    Low.Filter.copy(out.layers, source.layers);
    out.collisionGroups = source.collisionGroups;
    out.collisionMask = source.collisionMask;
    out.bodyFilter = source.bodyFilter;
}
