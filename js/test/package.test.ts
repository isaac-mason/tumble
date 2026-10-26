import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { expect, test } from 'vitest';

const root = fileURLToPath(new URL('..', import.meta.url));

// the package as published: built from src, imported by name in node, with no bundler between
test('the built package, as node imports it', () => {
    execFileSync('npx', ['tsc', '-p', 'tsconfig.build.json'], { cwd: root });
    const output = execFileSync('node', ['test/package/consumer.mjs'], { cwd: root, encoding: 'utf-8' });
    expect(output.trim()).toBe('the package works');
}, 60_000);
