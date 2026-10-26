/** settings with every field optional at every depth: a value or a tuple of numbers is given whole */
export type Given<T> = T extends number | boolean | string | null
    ? T
    : T extends readonly (infer Element)[]
      ? Element extends number
          ? T
          : readonly (Given<Element> | undefined)[]
      : { [K in keyof T]?: Given<T[K]> };

/**
 * the given settings over the defaults: an object merges field by field and an array of objects element by element,
 * at every depth, and anything else given replaces the default; an undefined field takes the default
 */
export function over<T>(defaults: T, given: NoInfer<Given<T>> | undefined): T {
    return merge(defaults, given) as T;
}

function merge(defaults: unknown, given: unknown): unknown {
    if (given === undefined) return defaults;
    if (Array.isArray(defaults)) {
        if (typeof defaults[0] !== 'object') return given;
        const elements = given as readonly unknown[];
        return defaults.map((element, index) => merge(element, elements[index]));
    }
    if (typeof defaults !== 'object' || defaults === null) return given;
    const merged: Record<string, unknown> = { ...defaults };
    const fields = given as Record<string, unknown>;
    for (const key in fields) merged[key] = merge(merged[key], fields[key]);
    return merged;
}

/** the defaults the C++ gives, read once */
export function defaultsOf<T>(read: (out: T) => void, create: () => T): () => T {
    let value: T | null = null;
    return () => {
        if (value === null) {
            value = create();
            read(value);
        }
        return value;
    };
}
