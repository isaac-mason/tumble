import type { Color } from 'tumble.js';

// the gallery's and every example's colours. mirrors theme.css

export const palette = {
    /** the canvas */
    base: '#131518',
    /** panels, a shade above the canvas */
    page: '#1c1f25',
    /** type, and neutral solids */
    light: '#e8eaed',
    /** secondary type */
    muted: '#8a95a1',
    /** the ground, and anything that should recede */
    dim: '#444b55',
} as const;

/** accents in spectrum order: violet, coral, orange, sun, mint, teal, sky */
export const spectrum = ['#a66cff', '#ff6b81', '#ff9f43', '#feca57', '#2ed573', '#1dd1c1', '#54a0ff'] as const;

/** a hex colour as [r, g, b] in [0, 1] */
export function rgb(hex: string): [number, number, number] {
    return [
        Number.parseInt(hex.slice(1, 3), 16) / 255,
        Number.parseInt(hex.slice(3, 5), 16) / 255,
        Number.parseInt(hex.slice(5, 7), 16) / 255,
    ];
}

/** a hex colour as the library's debug colour, opaque */
export function debugColor(hex: string): Color {
    return [
        Number.parseInt(hex.slice(1, 3), 16),
        Number.parseInt(hex.slice(3, 5), 16),
        Number.parseInt(hex.slice(5, 7), 16),
        255,
    ];
}

export const clearColor: [number, number, number, number] = [...rgb(palette.base), 1];
