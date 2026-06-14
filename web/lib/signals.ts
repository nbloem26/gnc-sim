/**
 * Telemetry-explorer signal helpers.
 *
 * The result JSON exposes every channel as a columnar array under `SimResult.series`
 * (one array per channel, all the same length as `series.t`). The explorer is
 * deliberately *data-driven* over whatever keys actually exist in `series` — newer
 * scenarios emit channels (e.g. discrim_*, track_*, thrust, selected_obj) that are
 * not all present in the static `SimSeries` type. We therefore introspect the
 * object at runtime rather than relying on the type.
 */

import type { SimResult } from './types';

/** A plottable channel: numeric array keyed by name, excluding the time base `t`. */
export interface Channel {
  /** Channel key as it appears in `series` (e.g. "veh_x"). */
  key: string;
  /** Group bucket for the tree (e.g. "vehicle"). */
  group: string;
  /** Per-sample values (same length as the time base). */
  values: number[];
}

export interface SignalGroup {
  name: string;
  channels: Channel[];
}

/**
 * Ordered prefix → group rules. First match wins; anything unmatched falls into
 * "other". Order matters: more specific prefixes precede generic ones.
 */
const GROUP_RULES: ReadonlyArray<readonly [RegExp, string]> = [
  [/^veh_/, 'vehicle'],
  [/^(roll|pitch|yaw)$/, 'vehicle'],
  [/^(mass|mach|thrust)$/, 'vehicle'],
  [/^tgt_/, 'target'],
  [/^(accel_cmd_|los_|v_closing$|range$)/, 'gnc'],
  [/^nav_/, 'navigation'],
  [/^(imu_|seeker_)/, 'sensors'],
  [/^track_/, 'track'],
  [/^(discrim_|selected_obj$)/, 'discrimination'],
  [/^(env_|atmo_|wind_|grav_|air_|rho|temp|press)/, 'environment'],
];

/** Stable display order for known groups; unknown groups append alphabetically. */
const GROUP_ORDER = [
  'vehicle',
  'target',
  'gnc',
  'navigation',
  'sensors',
  'track',
  'discrimination',
  'environment',
  'other',
];

function groupFor(key: string): string {
  for (const [re, group] of GROUP_RULES) {
    if (re.test(key)) return group;
  }
  return 'other';
}

/** True when `v` is a numeric array we can plot against time. */
function isNumericArray(v: unknown): v is number[] {
  return Array.isArray(v) && (v.length === 0 || typeof v[0] === 'number');
}

/**
 * Build the flat channel list from a result, excluding the time base. Channels
 * are returned in deterministic order (group order, then key alpha within group).
 */
export function buildChannels(result: SimResult): Channel[] {
  const series = result.series as unknown as Record<string, unknown>;
  const channels: Channel[] = [];
  for (const key of Object.keys(series)) {
    if (key === 't') continue;
    const v = series[key];
    if (!isNumericArray(v)) continue;
    channels.push({ key, group: groupFor(key), values: v });
  }
  channels.sort((a, b) => {
    const ga = GROUP_ORDER.indexOf(a.group);
    const gb = GROUP_ORDER.indexOf(b.group);
    const oa = ga === -1 ? GROUP_ORDER.length : ga;
    const ob = gb === -1 ? GROUP_ORDER.length : gb;
    if (oa !== ob) return oa - ob;
    if (a.group !== b.group) return a.group.localeCompare(b.group);
    return a.key.localeCompare(b.key);
  });
  return channels;
}

/** Group channels into tree buckets, preserving the deterministic order. */
export function groupChannels(channels: Channel[]): SignalGroup[] {
  const map = new Map<string, Channel[]>();
  for (const c of channels) {
    const arr = map.get(c.group);
    if (arr) arr.push(c);
    else map.set(c.group, [c]);
  }
  return Array.from(map.entries()).map(([name, chans]) => ({ name, channels: chans }));
}

/** The time base for a result (the `t` channel). */
export function timeBase(result: SimResult): number[] {
  return result.series.t;
}

// ---------------------------------------------------------------------------
// Derived-signal expression evaluator (stretch).
//
// A tiny, safe recursive-descent evaluator over the channel namespace. It is
// element-wise: every channel reference resolves to its array, scalars/operators
// broadcast, and the result is a same-length array. No `eval`, no globals — only
// the identifiers that name real channels plus a small whitelist of math fns.
// ---------------------------------------------------------------------------

type NumFn = (...a: number[]) => number;

const MATH_FNS: Record<string, NumFn> = {
  sqrt: Math.sqrt,
  abs: Math.abs,
  sin: Math.sin,
  cos: Math.cos,
  tan: Math.tan,
  atan: Math.atan,
  asin: Math.asin,
  acos: Math.acos,
  exp: Math.exp,
  log: Math.log,
  log10: Math.log10,
  min: Math.min,
  max: Math.max,
  pow: Math.pow,
  hypot: Math.hypot,
  sign: Math.sign,
  floor: Math.floor,
  ceil: Math.ceil,
  round: Math.round,
  atan2: Math.atan2,
  deg: (x: number) => (x * 180) / Math.PI,
  rad: (x: number) => (x * Math.PI) / 180,
};

const MATH_CONSTS: Record<string, number> = {
  pi: Math.PI,
  e: Math.E,
};

type Tok =
  | { t: 'num'; v: number }
  | { t: 'id'; v: string }
  | { t: 'op'; v: string }
  | { t: 'lp' }
  | { t: 'rp' }
  | { t: 'comma' };

function tokenize(src: string): Tok[] {
  const toks: Tok[] = [];
  let i = 0;
  while (i < src.length) {
    const c = src[i];
    if (c === ' ' || c === '\t' || c === '\n') {
      i++;
      continue;
    }
    if (c >= '0' && c <= '9') {
      let j = i + 1;
      while (j < src.length && /[0-9.eE+\-]/.test(src[j])) {
        // Allow exponent sign only right after e/E.
        if ((src[j] === '+' || src[j] === '-') && !/[eE]/.test(src[j - 1])) break;
        j++;
      }
      const num = Number(src.slice(i, j));
      if (!Number.isFinite(num)) throw new Error(`Invalid number near "${src.slice(i, j)}"`);
      toks.push({ t: 'num', v: num });
      i = j;
      continue;
    }
    if (/[A-Za-z_]/.test(c)) {
      let j = i + 1;
      while (j < src.length && /[A-Za-z0-9_]/.test(src[j])) j++;
      toks.push({ t: 'id', v: src.slice(i, j) });
      i = j;
      continue;
    }
    if (c === '(') {
      toks.push({ t: 'lp' });
      i++;
      continue;
    }
    if (c === ')') {
      toks.push({ t: 'rp' });
      i++;
      continue;
    }
    if (c === ',') {
      toks.push({ t: 'comma' });
      i++;
      continue;
    }
    if ('+-*/^%'.includes(c)) {
      toks.push({ t: 'op', v: c });
      i++;
      continue;
    }
    throw new Error(`Unexpected character "${c}"`);
  }
  return toks;
}

/**
 * Parse + evaluate `expr` element-wise against `lookup` (channel-name → array)
 * and a known length `n`. Returns a same-length array. Throws on syntax errors
 * or unknown identifiers (the UI surfaces the message).
 */
export function evalExpression(
  expr: string,
  lookup: (name: string) => number[] | undefined,
  n: number,
): number[] {
  const toks = tokenize(expr);
  let pos = 0;

  const peek = () => toks[pos];
  const next = () => toks[pos++];

  // Values flow as length-n arrays; scalars are broadcast lazily via const arrays.
  const constArr = (v: number): number[] => {
    const a = new Array<number>(n);
    a.fill(v);
    return a;
  };
  const ewUnary = (a: number[], f: (x: number) => number): number[] => {
    const out = new Array<number>(n);
    for (let k = 0; k < n; k++) out[k] = f(a[k]);
    return out;
  };
  const ewBinary = (a: number[], b: number[], f: (x: number, y: number) => number): number[] => {
    const out = new Array<number>(n);
    for (let k = 0; k < n; k++) out[k] = f(a[k], b[k]);
    return out;
  };

  function parsePrimary(): number[] {
    const tk = peek();
    if (!tk) throw new Error('Unexpected end of expression');
    if (tk.t === 'num') {
      next();
      return constArr(tk.v);
    }
    if (tk.t === 'lp') {
      next();
      const v = parseAdd();
      if (peek()?.t !== 'rp') throw new Error('Missing ")"');
      next();
      return v;
    }
    if (tk.t === 'op' && (tk.v === '-' || tk.v === '+')) {
      next();
      const v = parsePrimary();
      return tk.v === '-' ? ewUnary(v, (x) => -x) : v;
    }
    if (tk.t === 'id') {
      next();
      // Function call?
      if (peek()?.t === 'lp') {
        const fn = MATH_FNS[tk.v];
        if (!fn) throw new Error(`Unknown function "${tk.v}"`);
        next(); // consume (
        const args: number[][] = [];
        if (peek()?.t !== 'rp') {
          args.push(parseAdd());
          while (peek()?.t === 'comma') {
            next();
            args.push(parseAdd());
          }
        }
        if (peek()?.t !== 'rp') throw new Error('Missing ")" after arguments');
        next();
        const out = new Array<number>(n);
        for (let k = 0; k < n; k++) out[k] = fn(...args.map((a) => a[k]));
        return out;
      }
      // Constant?
      if (tk.v in MATH_CONSTS) return constArr(MATH_CONSTS[tk.v]);
      // Channel reference.
      const arr = lookup(tk.v);
      if (!arr) throw new Error(`Unknown signal "${tk.v}"`);
      return arr;
    }
    throw new Error('Unexpected token in expression');
  }

  function parsePow(): number[] {
    let base = parsePrimary();
    if (peek()?.t === 'op' && (peek() as { v: string }).v === '^') {
      next();
      const exp = parsePow(); // right-assoc
      base = ewBinary(base, exp, (a, b) => Math.pow(a, b));
    }
    return base;
  }

  function parseMul(): number[] {
    let left = parsePow();
    while (peek()?.t === 'op' && '*/%'.includes((peek() as { v: string }).v)) {
      const op = (next() as { v: string }).v;
      const right = parsePow();
      left = ewBinary(left, right, (a, b) =>
        op === '*' ? a * b : op === '/' ? a / b : a % b,
      );
    }
    return left;
  }

  function parseAdd(): number[] {
    let left = parseMul();
    while (peek()?.t === 'op' && '+-'.includes((peek() as { v: string }).v)) {
      const op = (next() as { v: string }).v;
      const right = parseMul();
      left = ewBinary(left, right, (a, b) => (op === '+' ? a + b : a - b));
    }
    return left;
  }

  const result = parseAdd();
  if (pos !== toks.length) throw new Error('Trailing characters in expression');
  return result;
}

/** Distinct, stable color for each curve index (cycles through a dark-theme palette). */
export const CURVE_PALETTE = [
  '#4fd1c5',
  '#f6ad55',
  '#90cdf4',
  '#fc8181',
  '#68d391',
  '#b794f4',
  '#f687b3',
  '#76e4f7',
  '#fbd38d',
  '#9ae6b4',
  '#d6bcfa',
  '#feb2b2',
];

export function curveColor(index: number): string {
  return CURVE_PALETTE[index % CURVE_PALETTE.length];
}
