import { type Vec3, vec3 } from 'math';

/**
 * a (2, 3) torus knot's surface: a tube around the curve that winds twice around the torus's axis and three times
 * through its hole. positions xyz and indices three a triangle, counter clockwise seen from outside
 */
export function torusKnot(
    radius: number,
    tube: number,
    tubularSegments: number,
    radialSegments: number,
): { positions: number[]; indices: number[] } {
    const P = 2;
    const Q = 3;
    const pointOnCurve = (out: Vec3, u: number): Vec3 => {
        const turn = (Q / P) * u;
        const spread = radius * (2 + Math.cos(turn)) * 0.5;
        return vec3.set(out, spread * Math.cos(u), spread * Math.sin(u), radius * Math.sin(turn) * 0.5);
    };
    const here: Vec3 = [0, 0, 0];
    const ahead: Vec3 = [0, 0, 0];
    const tangent: Vec3 = [0, 0, 0];
    const normal: Vec3 = [0, 0, 0];
    const binormal: Vec3 = [0, 0, 0];
    const positions: number[] = [];
    for (let j = 0; j <= tubularSegments; j++) {
        const u = (j / tubularSegments) * P * Math.PI * 2;
        pointOnCurve(here, u);
        pointOnCurve(ahead, u + 0.01);
        // a frame along the curve: its tangent, and two directions across it
        vec3.subtract(tangent, ahead, here);
        vec3.add(normal, ahead, here);
        vec3.normalize(binormal, vec3.cross(binormal, tangent, normal));
        vec3.normalize(normal, vec3.cross(normal, binormal, tangent));
        for (let i = 0; i <= radialSegments; i++) {
            const v = (i / radialSegments) * Math.PI * 2;
            const across = -tube * Math.cos(v);
            const up = tube * Math.sin(v);
            positions.push(
                here[0] + across * normal[0] + up * binormal[0],
                here[1] + across * normal[1] + up * binormal[1],
                here[2] + across * normal[2] + up * binormal[2],
            );
        }
    }
    const indices: number[] = [];
    for (let j = 1; j <= tubularSegments; j++) {
        for (let i = 1; i <= radialSegments; i++) {
            const a = (radialSegments + 1) * (j - 1) + (i - 1);
            const b = (radialSegments + 1) * j + (i - 1);
            const c = (radialSegments + 1) * j + i;
            const d = (radialSegments + 1) * (j - 1) + i;
            indices.push(a, b, d, b, c, d);
        }
    }
    return { positions, indices };
}
