import * as Low from '../generated/tumble.js';
import { type CompoundChild, type Shape, withChildren } from './shapes.js';

export type CompoundShapeSettings = { children: CompoundChild[] };

/** children placed together; it copies the list and holds the shapes */
export function create(settings: CompoundShapeSettings): Shape {
    return withChildren(settings.children, (address) => Low.Compound.create(address, settings.children.length));
}

/** its children replaced, holding the new and letting go of the old; each body using it takes it again with rigidBody.setShape */
export function update(shape: Shape, settings: CompoundShapeSettings): void {
    withChildren(settings.children, (address) => Low.Compound.setChildren(shape, address, settings.children.length));
}
