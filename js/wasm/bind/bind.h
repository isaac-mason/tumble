#pragma once

// the binder: js/wasm/bindings.cpp is a manifest of what the wasm exposes, written against this. the schema build
// (js/wasm/toolchain/build-schema.sh) runs it once and writes a json description, from which generate.ts writes the typed
// plumbing module (js/src/generated/tumble.ts) and the extern "C" layer (js/wasm/generated/exports.cpp). none of this is
// compiled into the shipped binary.
//
// a function's js namespace and name are its innermost C++ namespace and its name with the case changed
// (`adapters::rigid_body::get_position` is `RigidBody.getPosition`), its C symbol joins them under tumble
// (`tumble_rigid_body_get_position`); a record's js name is its type name. the manifest adds only what C++ cannot say:
// which functions and fields are exposed, parameter names, docs, whether a call may grow the memory, and the js
// namespace of a function whose C++ one is not what js calls it (`.js_namespace("wasm")` binds `runtime::init` as
// `Wasm.init`, `tumble_wasm_init`).
//
// four kinds of type cross the boundary:
//   handles  `RECORD(World)`   objects the wasm owns, passed by pointer; js holds the
//            address as a type of its own. exposed fields are read through the views; WRITABLE
//            ones written too.
//   values   `VALUE(Transformf)` / `TUPLE_AS(Float3, "Vec3")`   plain data copied across as a js object or
//            tuple (a value's js name drops a trailing f: Transform). a function takes one by
//            `const V &` / `const V *` (an in-param: js writes it into scratch and passes the
//            address) or `V &` / `V *` (an out-param: the core writes it and js reads it back into
//            an object the caller supplied). values never come back by return. `LANES_AS(Vec3f,
//            "Vec3f", 3)` is a SIMD register type crossing as a tuple of its lanes.
//   arrays   a record's field holding a growable array (bind::array_of), read where it lies: its length and
//            its elements.
//   scalars  u8/u16/u32/i32/f32/bool, and pointers to them as addresses typed by the scalar
//            (a `void *` as a plain number, which any address converts to). an enum crosses as
//            its underlying scalar; an f64 only as a field, read as the js number it is; a u64
//            only as a field too, read as a number, exact up to 2^53. a struct marked
//            CROSSES_AS_BITS (a typed handle, one u32 `bits`) crosses as that u32.
//
//   TUMBLE_BINDINGS {
//       bind::include(b, "tumble/physics/world.h");
//       RECORD(World);
//       bind::fn(b, BIND(world::step), names("world", "deltaTime"), GROWS, "one step answering nothing.");
//   }

#include <stdint.h>

#define BIND(f) #f, f
#define FIELD(T, m) #m, &T::m
#define RECORD(T) bind::record<T>(b, #T)
#define RECORD_AS(T, js_name) bind::record<T>(b, #T, js_name)
#define VALUE(T) bind::value<T>(b, #T)
#define VALUE_AS(T, js_name) bind::value<T>(b, #T, js_name)
#define TUPLE(T) bind::tuple<T>(b, #T)
#define TUPLE_AS(T, js_name) bind::tuple<T>(b, #T, js_name)
#define LANES_AS(T, js_name, count) bind::lanes<T>(b, #T, js_name, count)

namespace bind {

// ---- type naming, filled by registration and read when the json is written ----

template <typename T>
struct TypeInfo {
    /** the C++ spelling, qualified, as the generated exports need it. */
    static const char *cpp;
    static bool is_value;
    static bool is_tuple;
};
template <typename T>
const char *TypeInfo<T>::cpp = nullptr;
template <typename T>
bool TypeInfo<T>::is_value = false;
template <typename T>
bool TypeInfo<T>::is_tuple = false;

enum BaseKind : uint8_t { BASE_VOID, BASE_SCALAR, BASE_STRUCT, BASE_ARRAY };

struct TypeRef {
    BaseKind base;
    /** 0 for by value, 1 for a pointer, 2 for a pointer to pointers. */
    uint8_t pointers;
    bool is_const;
    /** a reference parameter; crosses as a pointer, dereferenced in the generated wrapper. */
    bool is_ref;
    /** elements of an array member; 1 for a plain one. */
    uint32_t count;
    const char *scalar;
    const char *(*cpp)();
    bool (*is_value)();
    bool (*is_tuple)();
    /** a one-word handle struct (`{ uint32_t bits; }`, marked CROSSES_AS_BITS) that crosses as its u32. */
    bool bits;
    /** BASE_ARRAY: the element's type, and the byte offsets of the array's data pointer and length; zero otherwise. */
    TypeRef (*element)();
    uint32_t data_offset;
    uint32_t length_offset;
};

template <typename T>
const char *cpp_of() {
    return TypeInfo<T>::cpp;
}
template <typename T>
bool value_of() {
    return TypeInfo<T>::is_value;
}
template <typename T>
bool tuple_of() {
    return TypeInfo<T>::is_tuple;
}

template <typename T>
struct TypeOf {
    static TypeRef get() {
        if constexpr (__is_enum(T)) return TypeOf<__underlying_type(T)>::get();
        else if constexpr (requires { T::CROSSES_AS_BITS; }) return {BASE_SCALAR, 0, false, false, 1, "u32", nullptr, nullptr, nullptr, true, nullptr, 0, 0};
        else return {BASE_STRUCT, 0, false, false, 1, nullptr, cpp_of<T>, value_of<T>, tuple_of<T>, false, nullptr, 0, 0};
    }
};
template <>
struct TypeOf<void> {
    static TypeRef get() { return {BASE_VOID, 0, false, false, 1, nullptr, nullptr, nullptr, nullptr, false, nullptr, 0, 0}; }
};
template <typename T>
struct TypeOf<const T> {
    static TypeRef get() {
        TypeRef r = TypeOf<T>::get();
        r.is_const = true;
        return r;
    }
};
template <typename T>
struct TypeOf<T *> {
    static TypeRef get() {
        TypeRef r = TypeOf<T>::get();
        r.pointers++;
        return r;
    }
};
template <typename T>
struct TypeOf<T &> {
    static TypeRef get() {
        TypeRef r = TypeOf<T>::get();
        r.pointers++;
        r.is_ref = true;
        return r;
    }
};
template <typename T, unsigned long N>
struct TypeOf<T[N]> {
    static TypeRef get() {
        TypeRef r = TypeOf<T>::get();
        r.count = (uint32_t)N;
        return r;
    }
};
#define BIND_SCALAR(cpp_type, scalar_name)                                                                        \
    template <>                                                                                                   \
    struct TypeOf<cpp_type> {                                                                                     \
        static TypeRef get() { return {BASE_SCALAR, 0, false, false, 1, scalar_name, nullptr, nullptr, nullptr, false, nullptr, 0, 0}; } \
    };
BIND_SCALAR(uint8_t, "u8")
BIND_SCALAR(uint16_t, "u16")
BIND_SCALAR(uint32_t, "u32")
BIND_SCALAR(int32_t, "i32")
BIND_SCALAR(float, "f32")
BIND_SCALAR(bool, "bool")
BIND_SCALAR(double, "f64")
BIND_SCALAR(uint64_t, "u64")
#undef BIND_SCALAR
/** a raw address, as the heap hands out. */
template <>
struct TypeOf<void *> {
    static TypeRef get() { return {BASE_SCALAR, 1, false, false, 1, "void", nullptr, nullptr, nullptr, false, nullptr, 0, 0}; }
};

// ---- the description ----

constexpr uint32_t MAX_PARAMS = 12;
constexpr uint32_t MAX_FIELDS = 48;
constexpr uint32_t MAX_STRUCTS = 128;
constexpr uint32_t MAX_ENUMS = 64;
constexpr uint32_t MAX_VALUES = 64;
constexpr uint32_t MAX_FUNCTIONS = 512;
constexpr uint32_t MAX_INCLUDES = 32;
constexpr uint32_t MAX_CONSTANTS = 64;

/** the next free slot of a fixed table, trapping when it is full: the schema build keeps function names, so the trap
 *  names the binder call that overflowed instead of writing past the table. */
inline uint32_t take_slot(uint32_t &count, uint32_t capacity) {
    if (count >= capacity) __builtin_trap();
    return count++;
}

enum FunctionFlags : uint32_t {
    /** the call may allocate, so js re-derives its views after it; a handle it returns is never null, it throws instead. */
    GROWS = 1u << 0,
};

enum FieldFlags : uint32_t {
    /** js may write the field; by default a record's fields are the core's to change. */
    WRITABLE = 1u << 0,
};

/**
 * parameter names for a function, in order; `names("world", "body", "position")`. an unnamed handle
 * or value is named after its type, so a function of one handle needs none. a trailing ? marks a
 * handle that may be null (`"registry?"`); every other handle param is non-null in js.
 */
struct Names {
    const char *list[MAX_PARAMS];
    uint32_t count;
};

template <typename... S>
inline Names names(S... parts) {
    Names n = {{parts...}, sizeof...(S)};
    return n;
}

struct FunctionDesc {
    /** the qualified C++ name, `adapters::rigid_body::get_position`. */
    const char *cpp;
    TypeRef returns;
    TypeRef params[MAX_PARAMS];
    /** the js names of the params, from `names(...)`; null entries fall back to a, b, c. */
    const char *param_names[MAX_PARAMS];
    uint32_t param_count;
    uint32_t flags;
    const char *doc;
    /** the js namespace and C symbol prefix, snake case, when they are not the C++ namespace's; null otherwise. */
    const char *js_namespace;
};

struct FieldDesc {
    /** the C++ member name. */
    const char *name;
    uint32_t offset;
    TypeRef type;
    uint32_t flags;
    const char *doc;
};

/** how js finds a record by an id with no call; see StructBinder::id_lookup. */
struct IdLookup {
    /** the record js passes to start from, World. */
    const char *(*root)();
    /** byte offsets from the root of the slots' array's data pointer and length. */
    uint32_t data;
    uint32_t length;
    /** the record's field holding the id it answers to. */
    uint32_t id;
    uint32_t index_mask;
};

struct StructDesc {
    /** the qualified C++ name. */
    const char *cpp;
    /** the js name when it isn't the type's own. */
    const char *js;
    uint32_t size;
    uint32_t align;
    bool is_value;
    bool is_tuple;
    /** a SIMD register type crossing as its lanes (bind::lanes) */
    bool is_lanes;
    FieldDesc fields[MAX_FIELDS];
    uint32_t field_count;
    const char *doc;
    bool has_id_lookup;
    IdLookup id_lookup;
};

struct EnumValue {
    const char *name;
    uint32_t value;
    const char *doc;
};

struct EnumDesc {
    const char *name;
    EnumValue values[MAX_VALUES];
    uint32_t value_count;
    const char *doc;
};

/** a C++ constant js reads as a module-level const of the same value. */
struct ConstantDesc {
    const char *name;
    /** a float's bits when is_float */
    uint32_t value;
    bool is_float;
    const char *doc;
};

struct Binder {
    StructDesc *structs;
    uint32_t struct_count;
    EnumDesc *enums;
    uint32_t enum_count;
    FunctionDesc *functions;
    uint32_t function_count;
    /** headers the generated exports include, so every bound function is declared there. */
    const char *includes[MAX_INCLUDES];
    uint32_t include_count;
    ConstantDesc constants[MAX_CONSTANTS];
    uint32_t constant_count;
};

inline void include(Binder &b, const char *header) { b.includes[take_slot(b.include_count, MAX_INCLUDES)] = header; }

inline void constant(Binder &b, const char *name, uint32_t value, const char *doc = nullptr) {
    b.constants[take_slot(b.constant_count, MAX_CONSTANTS)] = {name, value, false, doc};
}

inline void float_constant(Binder &b, const char *name, float value, const char *doc = nullptr) {
    b.constants[take_slot(b.constant_count, MAX_CONSTANTS)] = {name, __builtin_bit_cast(uint32_t, value), true, doc};
}

template <typename... A>
inline void collect_params(FunctionDesc &f) {
    const TypeRef params[] = {TypeOf<A>::get()..., TypeOf<void>::get()};
    static_assert(sizeof...(A) <= MAX_PARAMS, "a bound function takes more parameters than MAX_PARAMS");
    f.param_count = sizeof...(A);
    for (uint32_t i = 0; i < f.param_count; i++) f.params[i] = params[i];
}

struct FunctionBinder {
    FunctionDesc &desc;

    /** binds the function under this js namespace (snake case) instead of its C++ one; its C symbol follows. */
    FunctionBinder &js_namespace(const char *name) {
        desc.js_namespace = name;
        return *this;
    }
};

/** fills a description from the signature's types alone; the pointer only proves the function exists. */
template <typename R, typename... A>
inline FunctionBinder describe(Binder &b, const char *cpp, R (*)(A...), const Names &param_names, uint32_t flags, const char *doc) {
    FunctionDesc &f = b.functions[take_slot(b.function_count, MAX_FUNCTIONS)];
    f.cpp = cpp;
    f.returns = TypeOf<R>::get();
    f.flags = flags;
    f.doc = doc;
    f.js_namespace = nullptr;
    collect_params<A...>(f);
    for (uint32_t i = 0; i < MAX_PARAMS; i++) f.param_names[i] = i < param_names.count ? param_names.list[i] : nullptr;
    return {f};
}

template <typename R, typename... A>
inline FunctionBinder fn(Binder &b, const char *cpp, R (*f)(A...), const Names &param_names, uint32_t flags = 0, const char *doc = nullptr) {
    return describe(b, cpp, f, param_names, flags, doc);
}
template <typename R, typename... A>
inline FunctionBinder fn(Binder &b, const char *cpp, R (*f)(A...), const Names &param_names, const char *doc) {
    return describe(b, cpp, f, param_names, 0, doc);
}
template <typename R, typename... A>
inline FunctionBinder fn(Binder &b, const char *cpp, R (*f)(A...), uint32_t flags, const char *doc = nullptr) {
    return describe(b, cpp, f, {}, flags, doc);
}
template <typename R, typename... A>
inline FunctionBinder fn(Binder &b, const char *cpp, R (*f)(A...), const char *doc = nullptr) {
    return describe(b, cpp, f, {}, 0, doc);
}

/** the member's offset in a would-be object at address zero; no object is made. */
template <typename T, typename M>
inline uint32_t offset_of(M T::*member) {
    alignas(T) static char storage[sizeof(T)];
    const T *base = reinterpret_cast<const T *>(storage);
    return (uint32_t)(reinterpret_cast<const char *>(&(base->*member)) - reinterpret_cast<const char *>(base));
}

/**
 * a field holding a growable array (a data pointer and a length, as tumble's collections::Array), which js reads where it
 * lies: its length and its elements. the manifest specializes TypeOf for its array type with this.
 */
template <typename A, typename E>
inline TypeRef array_of() {
    return {BASE_ARRAY, 0, false, false, 1, nullptr, nullptr, nullptr, nullptr, false, TypeOf<E>::get, offset_of<A>(&A::data),
            offset_of<A>(&A::length)};
}

template <typename T>
struct StructBinder {
    StructDesc &desc;

    template <typename M>
    StructBinder &field(const char *name, M T::*member, uint32_t flags, const char *doc = nullptr) {
        desc.fields[take_slot(desc.field_count, MAX_FIELDS)] = {name, offset_of<T, M>(member), TypeOf<M>::get(), flags, doc};
        return *this;
    }
    template <typename M>
    StructBinder &field(const char *name, M T::*member, const char *doc = nullptr) {
        return field(name, member, 0, doc);
    }

    StructBinder &doc(const char *text) {
        desc.doc = text;
        return *this;
    }

    /**
     * js finds the record by an id with no call: the id's index (`index_mask`) is a slot of the array of record pointers
     * at root->*container.*array (one with data and length), and the slot's record is the id's while its id field holds
     * it. `RigidBody.get(world, id)`.
     */
    template <typename Root, typename Container, typename Array, typename Id>
    StructBinder &id_lookup(Container Root::*container, Array Container::*array, Id T::*id_field, uint32_t index_mask) {
        static_assert(__is_same(decltype(Array::data), T **), "an id lookup's array holds pointers to the record");
        static_assert(__is_same(decltype(Array::length), uint32_t), "an id lookup's array has a uint32_t length");
        const uint32_t at = offset_of<Root, Container>(container) + offset_of<Container, Array>(array);
        desc.has_id_lookup = true;
        desc.id_lookup = {cpp_of<Root>, at + offset_of<Array>(&Array::data), at + offset_of<Array>(&Array::length),
                          offset_of<T, Id>(id_field), index_mask};
        return *this;
    }
};

struct EnumBinder {
    EnumDesc &desc;
    EnumBinder &value(const char *name, uint32_t v, const char *doc = nullptr) {
        desc.values[take_slot(desc.value_count, MAX_VALUES)] = {name, v, doc};
        return *this;
    }
    /** an enum's value, as its underlying integer */
    template <typename E>
        requires __is_enum(E)
    EnumBinder &value(const char *name, E v, const char *doc = nullptr) {
        return value(name, (uint32_t)v, doc);
    }
    EnumBinder &doc(const char *text) {
        desc.doc = text;
        return *this;
    }
};

template <typename T>
inline StructDesc &register_struct(Binder &b, const char *cpp, const char *js) {
    TypeInfo<T>::cpp = cpp;
    StructDesc &desc = b.structs[take_slot(b.struct_count, MAX_STRUCTS)];
    desc.cpp = cpp;
    desc.js = js;
    desc.size = sizeof(T);
    desc.align = alignof(T);
    return desc;
}

/** a handle: an object the core owns, which js holds by address. */
template <typename T>
inline StructBinder<T> record(Binder &b, const char *cpp, const char *js = nullptr) {
    return {register_struct<T>(b, cpp, js)};
}

/** a value crossing as a js object with named fields. */
template <typename T>
inline StructBinder<T> value(Binder &b, const char *cpp, const char *js = nullptr) {
    StructDesc &desc = register_struct<T>(b, cpp, js);
    desc.is_value = true;
    TypeInfo<T>::is_value = true;
    return {desc};
}

/** a value crossing as a js tuple, its fields in order; an array member contributes its elements. */
template <typename T>
inline StructBinder<T> tuple(Binder &b, const char *cpp, const char *js = nullptr) {
    StructDesc &desc = register_struct<T>(b, cpp, js);
    desc.is_value = true;
    desc.is_tuple = true;
    TypeInfo<T>::is_value = true;
    TypeInfo<T>::is_tuple = true;
    return {desc};
}

/**
 * a SIMD register type (Vec3f, Quatf, Mat4f) crossing as a js tuple of its first `count` f32 lanes, lane i at byte 4i:
 * x, y, z, w up to four, one array of them past that. a three lane type (Vec3f) is written with its z again in lane 3,
 * as the core keeps it.
 */
template <typename T>
inline void lanes(Binder &b, const char *cpp, const char *js, uint32_t count) {
    static const char *const LANE_NAMES[4] = {"x", "y", "z", "w"};
    if (count * sizeof(float) > sizeof(T)) __builtin_trap();
    StructDesc &desc = register_struct<T>(b, cpp, js);
    desc.is_value = true;
    desc.is_tuple = true;
    desc.is_lanes = true;
    TypeInfo<T>::is_value = true;
    TypeInfo<T>::is_tuple = true;
    if (count > 4) {
        TypeRef all = TypeOf<float>::get();
        all.count = count;
        desc.fields[take_slot(desc.field_count, MAX_FIELDS)] = {"m", 0, all, 0, nullptr};
        return;
    }
    for (uint32_t i = 0; i < count; i++) {
        desc.fields[take_slot(desc.field_count, MAX_FIELDS)] = {LANE_NAMES[i], i * (uint32_t)sizeof(float), TypeOf<float>::get(), 0, nullptr};
    }
}

template <typename E>
inline EnumBinder enumeration(Binder &b, const char *name) {
    EnumDesc &desc = b.enums[take_slot(b.enum_count, MAX_ENUMS)];
    desc.name = name;
    return {desc};
}

}  // namespace bind

/** the one bindings block, js/wasm/bindings.cpp. */
#define TUMBLE_BINDINGS void bind_all(bind::Binder &b)
