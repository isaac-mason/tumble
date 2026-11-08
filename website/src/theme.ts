/** a hex colour as [r, g, b] in [0, 1] */
export function rgb(hex: string): [number, number, number] {
    return [
        Number.parseInt(hex.slice(1, 3), 16) / 255,
        Number.parseInt(hex.slice(3, 5), 16) / 255,
        Number.parseInt(hex.slice(5, 7), 16) / 255,
    ];
}

/** the canvas's colour behind the scene */
export const clearColor: [number, number, number, number] = [...rgb('#14120e'), 1];
