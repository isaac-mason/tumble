// usage: node docs/build.mjs [--check]
// renders docs/README.template.md to README.md and docs/js.template.md to js/README.md, the guides GitHub and npm show
// first. --check renders nothing and fails when a rendered doc is not what its template and snippets give.
//
// a template's <Snippet source="./cpp/file.cpp" select="name" /> takes the lines between `/* SNIPPET_START: name */`
// and `/* SNIPPET_END: name */` in that file, unindented; without `select` it takes the whole file less the markers.
// lines between `/* HIDE_START */` and `/* HIDE_END */` are left out: the checks a snippet's program makes of itself.
// a CMake file marks the same with `# SNIPPET_START: name`, `# SNIPPET_END: name`, `# HIDE_START` and `# HIDE_END`.
// <TOC /> lists the ## and ### headings.
// <Examples /> is every sample's screenshot, and <ExamplesTable ids="example_cube_heap,example_ragdoll" /> some of them,
// each linking to the sample on the web and to its source. samples/screenshots.sh takes the screenshots
import fs from 'node:fs';
import path from 'node:path';

const docs = path.dirname(new URL(import.meta.url).pathname);
const root = path.join(docs, '..');
const check = process.argv.includes('--check');

const SAMPLES_URL = 'https://isaac-mason.github.io/tumble/samples/';

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

function snippet(template, source, select, width) {
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
    code = unindent(code);
    if (width !== undefined) {
        for (const line of code.split('\n')) {
            if (line.length > width) fail(`${template}: ${source} ${select ?? ''} has a line over ${width} columns:\n${line}`);
        }
    }
    const fence = FENCE[path.extname(source)] ?? '';
    return '```' + fence + '\n' + code + '\n```';
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

// the samples in the app's order, each from the file that declares it: `static const Example EXAMPLE_X = {"Title", ...`
function readSamples() {
    const order = fs.readFileSync(path.join(root, 'samples/app/examples.cpp'), 'utf-8');
    const titles = [...order.match(/ORDER\[\] = \{([\s\S]*?)\};/)[1].matchAll(/"([^"]+)"/g)].map((match) => match[1]);
    const titleToSource = new Map();
    for (const file of fs.readdirSync(path.join(root, 'samples/examples'))) {
        const text = fs.readFileSync(path.join(root, 'samples/examples', file), 'utf-8');
        const declared = text.match(/static const Example EXAMPLE_\w+ = \{"([^"]+)"/);
        if (declared === null) fail(`samples/examples/${file} declares no example`);
        titleToSource.set(declared[1], `samples/examples/${file}`);
    }
    const idToSample = new Map();
    for (const title of titles) {
        const source = titleToSource.get(title);
        if (source === undefined) fail(`no file in samples/examples declares "${title}"`);
        // as shell::example_id names it
        const id = `example_${title.toLowerCase().replace(/[^a-z0-9]/g, '_')}`;
        const screenshot = `samples/screenshots/${id}.png`;
        if (!fs.existsSync(path.join(root, screenshot))) fail(`no ${screenshot}: run samples/screenshots.sh ${id}`);
        idToSample.set(id, { title, source, screenshot });
    }
    return idToSample;
}

function sampleCell(id, sample, width, height, strong) {
    const label = strong ? `<strong>${sample.title}</strong>` : sample.title;
    return `    <td align="center">
      <a href="${SAMPLES_URL}?example=${id}">
        <img src="./${sample.screenshot}" width="${width}" height="${height}" style="object-fit:cover;"/><br/>
        ${label}
      </a><br/>
      <sub><a href="./${sample.source}">source</a></sub>
    </td>`;
}

function sampleRows(cells, columns) {
    let html = '<table>\n';
    for (let i = 0; i < cells.length; i += columns) html += `  <tr>\n${cells.slice(i, i + columns).join('\n')}\n  </tr>\n`;
    return `${html}</table>`;
}

function render(template, width) {
    let markdown = fs.readFileSync(path.join(docs, template), 'utf-8');
    markdown = markdown.replace(/<Snippet\s+source=["'](.+?)["'](?:\s+select=["'](.+?)["'])?\s*\/>/g, (_, source, select) =>
        snippet(template, source, select, width),
    );
    if (/<Examples(Table)?\b/.test(markdown)) {
        const idToSample = readSamples();
        markdown = markdown.replace(/<Examples\s*\/>/g, () =>
            sampleRows(
                [...idToSample].map(([id, sample]) => sampleCell(id, sample, 180, 120, false)),
                4,
            ),
        );
        markdown = markdown.replace(/<ExamplesTable\s+ids=["'](.+?)["']\s*\/>/g, (_, ids) => {
            const cells = ids.split(',').map((text) => {
                const id = text.trim();
                const sample = idToSample.get(id);
                if (sample === undefined) fail(`${template}: no sample ${id}`);
                return sampleCell(id, sample, 200, 133, true);
            });
            return sampleRows(cells, 4);
        });
    }
    markdown = markdown.replace(/<TOC\s*\/>/g, () => toc(markdown));
    const leftover = markdown.match(/<(TOC|Examples|ExamplesTable|Snippet)\b[^>]*\/>/);
    if (leftover) fail(`${template}: an unexpanded ${leftover[0]}`);
    return `<!-- rendered by docs/build.mjs from docs/${template}: edit that, not this -->\n\n${markdown}`;
}

// the C++ snippets' lines fit the width; the js snippets are biome's
const OUTPUTS = [
    { template: 'README.template.md', output: 'README.md', width: 100 },
    { template: 'js.template.md', output: 'js/README.md' },
];

let stale = 0;
for (const { template, output, width } of OUTPUTS) {
    const out = path.join(root, output);
    const rendered = render(template, width);
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
