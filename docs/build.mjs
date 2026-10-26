// usage: node docs/build.mjs [--check]
// renders docs/cpp.template.md to README.md and docs/js.template.md to js/README.md, the guides GitHub and npm show
// first. --check renders nothing and fails when a rendered doc is not what its template and snippets give.
//
// a template's <Snippet source="./cpp/file.cpp" select="name" /> takes the lines between `/* SNIPPET_START: name */`
// and `/* SNIPPET_END: name */` in that file, unindented; without `select` it takes the whole file less the markers.
// lines between `/* HIDE_START */` and `/* HIDE_END */` are left out: the checks a snippet's program makes of itself.
// a CMake file marks the same with `# SNIPPET_START: name`, `# SNIPPET_END: name`, `# HIDE_START` and `# HIDE_END`.
// <TOC /> lists the ## and ### headings.
import fs from 'node:fs';
import path from 'node:path';

const docs = path.dirname(new URL(import.meta.url).pathname);
const root = path.join(docs, '..');
const check = process.argv.includes('--check');

// a marker is a line of its own: /* SNIPPET_START: name */ in C++, # SNIPPET_START: name in CMake
const OPEN = String.raw`(?:\/\*|#)[ \t]*`;
const CLOSE = String.raw`[ \t]*(?:\*\/)?[ \t]*`;
const MARKER = new RegExp(String.raw`^[ \t]*${OPEN}SNIPPET_(START|END):[^*\n]*?${CLOSE}\n?`, 'gm');
const HIDDEN = new RegExp(String.raw`^[ \t]*${OPEN}HIDE_START${CLOSE}\n[\s\S]*?^[ \t]*${OPEN}HIDE_END${CLOSE}\n?`, 'gm');
const FENCE = { '.cpp': 'cpp', '.h': 'cpp', '.ts': 'ts', '.js': 'js', '.mjs': 'js', '.txt': 'cmake', '.cmake': 'cmake' };

function fail(message) {
    console.error(`docs: ${message}`);
    process.exit(1);
}

function unindent(text) {
    const lines = text.replace(/\s+$/, '').split('\n');
    const indents = lines.filter((line) => line.trim() !== '').map((line) => line.match(/^[ \t]*/)[0].length);
    const indent = indents.length > 0 ? Math.min(...indents) : 0;
    return lines.map((line) => line.slice(indent)).join('\n');
}

function snippet(template, source, select) {
    const file = path.join(docs, source);
    if (!fs.existsSync(file)) fail(`${template}: no snippet source ${source}`);
    const text = fs.readFileSync(file, 'utf-8');
    let code;
    if (select === undefined) {
        code = text.replace(MARKER, '');
    } else {
        const start = new RegExp(String.raw`^[ \t]*${OPEN}SNIPPET_START:[ \t]*${select}${CLOSE}\n`, 'm');
        const end = new RegExp(String.raw`^[ \t]*${OPEN}SNIPPET_END:[ \t]*${select}${CLOSE}$`, 'm');
        const from = text.match(start);
        if (from === null) fail(`${template}: no SNIPPET_START: ${select} in ${source}`);
        const rest = text.slice(from.index + from[0].length);
        const to = rest.match(end);
        if (to === null) fail(`${template}: no SNIPPET_END: ${select} in ${source}`);
        code = rest.slice(0, to.index).replace(MARKER, '');
    }
    code = code.replace(HIDDEN, '');
    if (/HIDE_(START|END)/.test(code)) fail(`${template}: an unmatched HIDE marker in ${source}`);
    const fence = FENCE[path.extname(source)] ?? '';
    return '```' + fence + '\n' + unindent(code) + '\n```';
}

function anchor(title) {
    return title
        .toLowerCase()
        .replace(/[^a-z0-9 _-]/g, '')
        .replace(/ /g, '-');
}

function toc(markdown) {
    const lines = [];
    let fenced = false;
    for (const line of markdown.split('\n')) {
        if (line.startsWith('```')) fenced = !fenced;
        const heading = !fenced && line.match(/^(#{2,3})\s+(.*)$/);
        if (heading) lines.push(`${'  '.repeat(heading[1].length - 2)}- [${heading[2]}](#${anchor(heading[2])})`);
    }
    return lines.join('\n');
}

function render(template) {
    let markdown = fs.readFileSync(path.join(docs, template), 'utf-8');
    markdown = markdown.replace(/<Snippet\s+source=["'](.+?)["'](?:\s+select=["'](.+?)["'])?\s*\/>/g, (_, source, select) =>
        snippet(template, source, select),
    );
    markdown = markdown.replace(/<TOC\s*\/>/g, () => toc(markdown));
    return `<!-- rendered by docs/build.mjs from docs/${template}: edit that, not this -->\n\n${markdown}`;
}

const OUTPUTS = { 'cpp.template.md': 'README.md', 'js.template.md': 'js/README.md' };

let stale = 0;
for (const [template, output] of Object.entries(OUTPUTS)) {
    const out = path.join(root, output);
    const rendered = render(template);
    if (check) {
        const current = fs.existsSync(out) ? fs.readFileSync(out, 'utf-8') : '';
        if (current !== rendered) {
            console.error(`docs: ${output} is not what docs/${template} renders; run node docs/build.mjs`);
            stale++;
        }
    } else {
        fs.writeFileSync(out, rendered);
        console.log(`docs: ${output}`);
    }
}
if (stale > 0) process.exit(1);
