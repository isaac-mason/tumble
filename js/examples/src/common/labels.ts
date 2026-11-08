import type { Vec3 } from 'math';
import type { Renderer } from './renderer';

/** text over the canvas, following a point in the world */
export type Label = { element: HTMLDivElement; position: Vec3 };

export function createLabel(text: string, position: Vec3): Label {
    const element = document.createElement('div');
    element.className = 'label';
    element.textContent = text;
    document.body.appendChild(element);
    return { element, position };
}

/** each label over its point as the camera sees it now; hidden when the point is behind the camera */
export function placeLabels(renderer: Renderer, labels: Label[]): void {
    const view4 = renderer.camera.matrixWorldInverse;
    const projection = renderer.camera.projectionMatrix;
    for (const label of labels) {
        const [x, y, z] = label.position;
        // column-major: the view, then the projection
        const vx = view4[0]! * x + view4[4]! * y + view4[8]! * z + view4[12]!;
        const vy = view4[1]! * x + view4[5]! * y + view4[9]! * z + view4[13]!;
        const vz = view4[2]! * x + view4[6]! * y + view4[10]! * z + view4[14]!;
        const cx = projection[0]! * vx + projection[4]! * vy + projection[8]! * vz + projection[12]!;
        const cy = projection[1]! * vx + projection[5]! * vy + projection[9]! * vz + projection[13]!;
        const cw = projection[3]! * vx + projection[7]! * vy + projection[11]! * vz + projection[15]!;
        if (cw <= 0) {
            label.element.style.display = 'none';
            continue;
        }
        label.element.style.display = '';
        const left = ((cx / cw) * 0.5 + 0.5) * window.innerWidth;
        const top = (0.5 - (cy / cw) * 0.5) * window.innerHeight;
        label.element.style.transform = `translate(-50%, -50%) translate(${left}px, ${top}px)`;
    }
}
