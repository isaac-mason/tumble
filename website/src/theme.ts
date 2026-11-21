/** a hex colour as [r, g, b] in [0, 1] */
export function rgb(hex: string): [number, number, number] {
    return [
        Number.parseInt(hex.slice(1, 3), 16) / 255,
        Number.parseInt(hex.slice(3, 5), 16) / 255,
        Number.parseInt(hex.slice(5, 7), 16) / 255,
    ];
}

/** the tones a place is laid out in, each a slightly different shade of what it is drawn as */
export type Tone =
    | 'cream'
    | 'linen'
    | 'stone'
    | 'oak'
    | 'bark'
    | 'ink'
    | 'sage'
    | 'moss'
    | 'terracotta'
    | 'blush'
    | 'butter'
    | 'sky'
    | 'denim';

/** what is built in, the floors, walls and town, is near black */
export const STRUCTURE: Record<Tone, string> = {
    cream: '#2e2e33',
    linen: '#2a2a2f',
    stone: '#252529',
    oak: '#28282d',
    bark: '#1f1f23',
    ink: '#16161a',
    sage: '#2c2c31',
    moss: '#222226',
    terracotta: '#26262a',
    blush: '#303035',
    butter: '#323237',
    sky: '#29292e',
    denim: '#3c3c44',
};

/** what can be knocked over is white and grey, so it stands out from everything it can hit */
export const THINGS: Record<Tone, string> = {
    cream: '#f4f4f4',
    linen: '#e6e6e7',
    stone: '#cfcfd2',
    oak: '#bdbdc1',
    bark: '#a7a7ac',
    ink: '#8e8e94',
    sage: '#dcdcde',
    moss: '#b3b3b8',
    terracotta: '#c6c6ca',
    blush: '#ececed',
    butter: '#fafafa',
    sky: '#d5d6d9',
    denim: '#a0a0a6',
};

/** the runner, the one colour in the world: crashcat's car amber */
export const RUNNER_ORANGE = '#ffa400';
export const RUNNER_DARK = '#d17f00';
export const RUNNER_TRIM = '#f4f4f4';

/** the sky straight up, at the horizon, and along the horizon; the distance fades into the sky */
export const SKY_ZENITH = rgb('#0c0c0f');
export const SKY_HORIZON = rgb('#1c1c21');
export const SKY_HAZE = rgb('#232328');

/** the canvas's colour behind the scene, where the sky does not reach */
export const clearColor: [number, number, number, number] = [...SKY_HORIZON, 1];
