import * as Low from '../generated/tumble.js';
import type { Shape } from './shapes.js';

/** no volume and no collisions: a placeholder for a body whose shape comes later */
export function create(): Shape {
    Low.errorClear();
    return Low.Empty.create();
}
