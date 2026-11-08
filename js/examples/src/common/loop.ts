/** the fixed step every example simulates at */
export const STEP = 1 / 60;

/** how long the last step took */
export type StepTime = { milliseconds: number };

/**
 * steps at a fixed 60 a second, at most 4 a frame, then draws. frame gets how far it is past the last step, 0 to 1, to
 * draw bodies between the pose the step began with and the one it left, so motion is even at any frame rate
 */
export function loop(stepTime: StepTime, step: (deltaTime: number) => void, frame: (alpha: number) => void): void {
    let last = performance.now();
    let owed = 0;
    const tick = (now: number) => {
        owed += Math.min((now - last) / 1000, 4 * STEP);
        last = now;
        while (owed >= STEP) {
            const started = performance.now();
            step(STEP);
            stepTime.milliseconds = performance.now() - started;
            owed -= STEP;
        }
        frame(owed / STEP);
        requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
}
