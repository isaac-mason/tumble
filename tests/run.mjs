// usage: node run.mjs <tumble-test.wasm>   (each test runs in an instance and memory of its own)
import { readFileSync } from 'node:fs';

const path = process.argv[2];
const module = await WebAssembly.compile(readFileSync(path));

/** a fresh instance: 4 MB of memory to start, grown as it allocates up to a 1 GB reservation */
async function instantiate() {
    const memory = new WebAssembly.Memory({ initial: 64, maximum: 16384 });
    const instance = await WebAssembly.instantiate(module, { env: { memory } });
    return { memory, wasm: instance.exports };
}

// views are made fresh per read: the buffer is replaced whenever the wasm grows the memory
const cstr = (memory, ptr) => {
    const u8 = new Uint8Array(memory.buffer);
    let end = ptr;
    while (u8[end] !== 0) end++;
    return new TextDecoder().decode(u8.subarray(ptr, end));
};

let failed = 0;

// the C++ harness
const { wasm } = await instantiate();
let failures = 0;
let checks = 0;
for (let index = 0, count = wasm.tests_count(); index < count; index++) {
    const run = await instantiate();
    failures += run.wasm.tests_run_one(index);
    checks += run.wasm.tests_checks();
    const recorded = run.wasm.tests_failure_count();
    const table = new Uint32Array(run.memory.buffer, run.wasm.tests_failures(), recorded * 4);
    for (let i = 0; i < recorded; i++) {
        const [file, line, expr, test] = table.subarray(i * 4, i * 4 + 4);
        console.error(`FAIL ${cstr(run.memory, test)}: ${cstr(run.memory, file)}:${line}: ${cstr(run.memory, expr)}`);
    }
}
console.log(`harness: ${checks} checks, ${failures} failures`);
failed += failures;

// build-test.sh compares this line between the simd backends; test_digest_matches_the_pin holds it to the pin
const digest = (await instantiate()).wasm.tests_digest();
console.log(`digest: ${(digest >>> 0).toString(16).padStart(8, '0')}`);

// the transcendental probes against node's Math, which is correctly rounded enough to serve as
// the reference at these tolerances. musl's single-precision routines are within 1 ulp; we assert
// a relative 2e-6 (a few ulps at f32) over sweeps that cross every reduction branch.
const REL = 2e-6;
const close = (got, want, abs = 1e-6) => Math.abs(got - want) <= Math.max(abs, REL * Math.abs(want));
// the boundary rounds the input to f32, so the reference sees the same rounded input; otherwise
// the input rounding alone (up to 4e-6 at x = 100) would fail steep functions.
const sweep = (label, probe, ref, from, to, steps, abs) => {
    let bad = 0;
    for (let i = 0; i <= steps; i++) {
        const x = Math.fround(from + ((to - from) * i) / steps);
        const got = probe(x);
        const want = Math.fround(ref(x));
        if (!close(got, want, abs)) {
            if (bad < 3) console.error(`FAIL ${label}(${x}): got ${got}, want ${want}`);
            bad++;
        }
    }
    console.log(`${label}: ${steps + 1} points, ${bad} bad`);
    return bad;
};
// a failed allocation traps, the error word saying why
{
    const run = await instantiate();
    let trapped = false;
    try {
        run.wasm.probe_out_of_memory();
    } catch (error) {
        trapped = error instanceof WebAssembly.RuntimeError;
    }
    // ERROR_OUT_OF_MEMORY
    if (!trapped || run.wasm.probe_error() !== 1) {
        console.error(`FAIL out of memory: trapped ${trapped}, error ${run.wasm.probe_error()}`);
        failed++;
    } else console.log('out of memory: traps');
}

if (wasm.probe_sin) {
    failed += sweep('sin', wasm.probe_sin, Math.sin, -100, 100, 20000);
    failed += sweep('sin large', wasm.probe_sin, Math.sin, -1e6, 1e6, 2000, 1e-4);
    failed += sweep('cos', wasm.probe_cos, Math.cos, -100, 100, 20000);
    failed += sweep('atan', wasm.probe_atan, Math.atan, -100, 100, 20000);
    failed += sweep('acos', wasm.probe_acos, Math.acos, -1, 1, 20000);
    let bad = 0;
    for (let yd = -10; yd <= 10; yd += 0.37) {
        for (let xd = -10; xd <= 10; xd += 0.41) {
            const y = Math.fround(yd);
            const x = Math.fround(xd);
            const got = wasm.probe_atan2(y, x);
            const want = Math.fround(Math.atan2(y, x));
            if (!close(got, want)) {
                if (bad < 3) console.error(`FAIL atan2(${y}, ${x}): got ${got}, want ${want}`);
                bad++;
            }
        }
    }
    console.log(`atan2 grid: ${bad} bad`);
    failed += bad;
}

if (failed > 0) {
    console.error(`${failed} failures`);
    process.exit(1);
}
console.log('ok');
