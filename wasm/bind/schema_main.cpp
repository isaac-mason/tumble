// the schema runner: runs the manifest and writes the description as json into memory for generate.ts to read. built
// by build-schema.sh, never shipped.

#include <stdint.h>

#include "tumble/util/alloc.h"
#include "tumble/util/mem.h"
#include "wasm/bind/bind.h"
#include "wasm/runtime/export.h"
#include "wasm/runtime/instance.h"

using namespace tumble;

void bind_all(bind::Binder &b);
extern unsigned char __heap_base[];

namespace {

struct Out {
    char *text;
    uint32_t length;
    uint32_t capacity;
};

void put(Out &o, const char *s) {
    for (; *s != 0; s++) {
        if (o.length + 1 >= o.capacity) {
            const uint32_t capacity = o.capacity * 2;
            char *text = util::alloc<char>(capacity);
            util::copy(text, o.text, o.length);
            util::free(o.text, o.capacity);
            o.text = text;
            o.capacity = capacity;
        }
        o.text[o.length++] = *s;
    }
}

void put_u32(Out &o, uint32_t v) {
    char digits[11];
    uint32_t n = 0;
    do {
        digits[n++] = (char)('0' + v % 10);
        v /= 10;
    } while (v != 0);
    char s[12];
    for (uint32_t i = 0; i < n; i++) s[i] = digits[n - 1 - i];
    s[n] = 0;
    put(o, s);
}

void put_bool(Out &o, bool v) { put(o, v ? "true" : "false"); }

void put_str(Out &o, const char *s) {
    put(o, "\"");
    for (; *s != 0; s++) {
        if (*s == '"' || *s == '\\') put(o, "\\");
        const char one[2] = {*s, 0};
        put(o, one);
    }
    put(o, "\"");
}

void put_opt_str(Out &o, const char *s) {
    if (s == nullptr) put(o, "null");
    else put_str(o, s);
}

/** the resolved kind of a type, now that every registration has run. */
void put_type(Out &o, const bind::TypeRef &t) {
    put(o, "{\"kind\":");
    switch (t.base) {
        case bind::BASE_VOID:
            put(o, "\"void\"");
            break;
        case bind::BASE_SCALAR:
            put_str(o, t.pointers == 0 ? "scalar" : "ptr-scalar");
            put(o, ",\"scalar\":");
            put_str(o, t.scalar);
            if (t.bits) put(o, ",\"bits\":true");
            break;
        case bind::BASE_STRUCT: {
            const bool value = t.is_value();
            const char *kind = t.pointers == 2 ? "ptr-array"
                               : t.pointers == 1 ? (value ? (t.is_const ? "value-in" : "value-out") : "handle")
                                                 : (value ? "value" : "embedded");
            put_str(o, kind);
            put(o, ",\"cpp\":");
            put_opt_str(o, t.cpp());
            break;
        }
    }
    put(o, ",\"count\":");
    put_u32(o, t.count);
    put(o, ",\"ref\":");
    put_bool(o, t.is_ref);
    put(o, ",\"const\":");
    put_bool(o, t.is_const);
    put(o, "}");
}

void put_function(Out &o, const bind::FunctionDesc &f) {
    put(o, "{\"cpp\":");
    put_str(o, f.cpp);
    put(o, ",\"returns\":");
    put_type(o, f.returns);
    put(o, ",\"params\":[");
    for (uint32_t i = 0; i < f.param_count; i++) {
        if (i > 0) put(o, ",");
        put(o, "{\"name\":");
        put_opt_str(o, f.param_names[i]);
        put(o, ",\"type\":");
        put_type(o, f.params[i]);
        put(o, "}");
    }
    put(o, "],\"grows\":");
    put_bool(o, (f.flags & bind::GROWS) != 0);
    put(o, ",\"doc\":");
    put_opt_str(o, f.doc);
    put(o, ",\"jsNamespace\":");
    put_opt_str(o, f.js_namespace);
    put(o, "}");
}

bind::StructDesc structs[bind::MAX_STRUCTS];
bind::EnumDesc enums[bind::MAX_ENUMS];
bind::FunctionDesc functions[bind::MAX_FUNCTIONS];
bind::Binder binder;
Out shipped;

/** how far a binder had registered: a section is what lies between two of these. */
struct Counts {
    uint32_t includes, structs, enums, functions, constants;
};

Counts counts_of(const bind::Binder &b) {
    return {b.include_count, b.struct_count, b.enum_count, b.function_count, b.constant_count};
}

/** the description of what was registered between from and to, nul-terminated in out. */
void describe(Out &out, const Counts &from, const Counts &to) {
    const bind::Binder &b = binder;
    out.capacity = 65536;
    out.text = util::alloc<char>(out.capacity);
    out.length = 0;

    put(out, "{\"includes\":[");
    for (uint32_t i = from.includes; i < to.includes; i++) {
        if (i > from.includes) put(out, ",");
        put_str(out, b.includes[i]);
    }
    put(out, "],\"structs\":[");
    for (uint32_t i = from.structs; i < to.structs; i++) {
        const bind::StructDesc &s = structs[i];
        if (i > from.structs) put(out, ",");
        put(out, "{\"cpp\":");
        put_str(out, s.cpp);
        put(out, ",\"js\":");
        put_opt_str(out, s.js);
        put(out, ",\"value\":");
        put_bool(out, s.is_value);
        put(out, ",\"tuple\":");
        put_bool(out, s.is_tuple);
        put(out, ",\"size\":");
        put_u32(out, s.size);
        put(out, ",\"align\":");
        put_u32(out, s.align);
        put(out, ",\"doc\":");
        put_opt_str(out, s.doc);
        put(out, ",\"fields\":[");
        for (uint32_t j = 0; j < s.field_count; j++) {
            const bind::FieldDesc &f = s.fields[j];
            if (j > 0) put(out, ",");
            put(out, "{\"name\":");
            put_str(out, f.name);
            put(out, ",\"offset\":");
            put_u32(out, f.offset);
            put(out, ",\"type\":");
            put_type(out, f.type);
            put(out, ",\"writable\":");
            put_bool(out, (f.flags & bind::WRITABLE) != 0);
            put(out, ",\"doc\":");
            put_opt_str(out, f.doc);
            put(out, "}");
        }
        put(out, "]}");
    }
    put(out, "],\"enums\":[");
    for (uint32_t i = from.enums; i < to.enums; i++) {
        const bind::EnumDesc &e = enums[i];
        if (i > from.enums) put(out, ",");
        put(out, "{\"name\":");
        put_str(out, e.name);
        put(out, ",\"doc\":");
        put_opt_str(out, e.doc);
        put(out, ",\"values\":[");
        for (uint32_t j = 0; j < e.value_count; j++) {
            if (j > 0) put(out, ",");
            put(out, "{\"name\":");
            put_str(out, e.values[j].name);
            put(out, ",\"value\":");
            put_u32(out, e.values[j].value);
            put(out, ",\"doc\":");
            put_opt_str(out, e.values[j].doc);
            put(out, "}");
        }
        put(out, "]}");
    }
    put(out, "],\"functions\":[");
    for (uint32_t i = from.functions; i < to.functions; i++) {
        if (i > from.functions) put(out, ",");
        put_function(out, functions[i]);
    }
    put(out, "],\"constants\":[");
    for (uint32_t i = from.constants; i < to.constants; i++) {
        const bind::ConstantDesc &c = b.constants[i];
        if (i > from.constants) put(out, ",");
        put(out, "{\"name\":");
        put_str(out, c.name);
        put(out, ",\"value\":");
        put_u32(out, c.value);
        put(out, ",\"float\":");
        put_bool(out, c.is_float);
        put(out, ",\"doc\":");
        put_opt_str(out, c.doc);
        put(out, "}");
    }
    put(out, "]}");
    out.text[out.length] = 0;
}

/** runs the manifest once. */
void describe_all() {
    if (shipped.text != nullptr) return;
    runtime::init(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__heap_base)));
    binder = {structs, 0, enums, 0, functions, 0, {}, 0, {}, 0};
    const Counts start = counts_of(binder);
    bind_all(binder);
    describe(shipped, start, counts_of(binder));
}

}  // namespace

/** the shipped binary's description; its address, nul-terminated. */
WASM_EXPORT const char *schema_json() {
    describe_all();
    return shipped.text;
}
