import { expect, test } from 'vitest';
import {
    Box,
    CastRayStatus,
    createAllCollidePointCollector,
    createClosestCastRayCollector,
    createClosestCastShapeCollector,
    createClosestCollideShapeCollector,
    init,
} from '../src';

// its own file, so the wasm is not up yet
test('collectors are made before init; anything that needs the wasm says to init first', async () => {
    const ray = createClosestCastRayCollector();
    const shape = createClosestCastShapeCollector();
    const overlap = createClosestCollideShapeCollector();
    const point = createAllCollidePointCollector();
    expect(ray.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
    expect(ray.earlyOutFraction).toBeGreaterThan(1);
    expect(shape.earlyOutFraction).toBe(ray.earlyOutFraction);
    expect(overlap.hit).toBe(null);
    expect(point.hits).toEqual([]);
    expect(() => Box.create({ halfExtents: [1, 1, 1] })).toThrow('tumble: await init() before anything else');
    await init();
    Box.destroy(Box.create({ halfExtents: [1, 1, 1] }));
});
