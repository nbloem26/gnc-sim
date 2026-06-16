/**
 * Studio contract — the reusable interface every subsystem "studio" implements
 * to plug into the generic <StudioShell>. This is the keystone the whole studio
 * suite (issue #103) builds on, so the API is intentionally small and ergonomic.
 *
 * A studio is a self-contained, interactive exploration tool: a set of typed
 * parameters (rendered automatically as a control panel) plus a `compute`
 * function that maps the current params to an array of plots.
 *
 * `compute` is just an async function returning plot specs — that keeps the
 * compute *source* open: it can be a pure client-side math function (see
 * studios/exampleStudio.ts), or it can `await runSim(config)` from
 * `@/lib/wasmRunner` to drive the C++ core compiled to WASM, or call a future
 * analysis entry. The shell does not care which; it only awaits PlotSpec[].
 */

import type { Data, Layout } from 'plotly.js-dist-min';

// ----------------------------------------------------------------------------
// Param schema — typed descriptors the shell renders into controls
// ----------------------------------------------------------------------------

/**
 * A numeric parameter rendered as a labelled slider + number input.
 *
 * Physical quantities follow the repo naming convention (units in the name):
 * the `key` should carry an SI unit suffix (e.g. `freq_hz`, `damping_per_s`),
 * and `unit` is the human-facing label shown next to the control.
 */
export interface NumberParam {
  kind: 'number';
  /** Object key this control binds to. Carry a unit suffix for physical vars. */
  key: string;
  /** Human-facing label. */
  label: string;
  /** Unit shown beside the label (e.g. "Hz", "1/s", "m"). Dimensionless: omit. */
  unit?: string;
  min_value: number;
  max_value: number;
  /** Slider/input granularity. */
  step: number;
  /** Initial value (must lie within [min_value, max_value]). */
  default_value: number;
  /** Optional one-line help shown under the control. */
  help?: string;
}

/** A boolean parameter rendered as a checkbox toggle. */
export interface BooleanParam {
  kind: 'boolean';
  key: string;
  label: string;
  default_value: boolean;
  help?: string;
}

/** A single option in an enum parameter. */
export interface EnumOption {
  value: string;
  label: string;
}

/** An enumerated parameter rendered as a <select>. */
export interface EnumParam {
  kind: 'enum';
  key: string;
  label: string;
  options: EnumOption[];
  default_value: string;
  help?: string;
}

/** One typed parameter descriptor. */
export type ParamDescriptor = NumberParam | BooleanParam | EnumParam;

/** An ordered array of typed parameter descriptors = a studio's control schema. */
export type ParamSchema = ParamDescriptor[];

/**
 * The live parameter values, keyed by descriptor `key`. Numeric params hold a
 * number, booleans a boolean, enums a string. Studios narrow as needed in their
 * `compute`.
 */
export type ParamValues = Record<string, number | boolean | string>;

// ----------------------------------------------------------------------------
// Plot spec — a Plotly-compatible plot, rendered via the shared <Plot> wrapper
// ----------------------------------------------------------------------------

/**
 * A single plot to render. `data` and `layout` are passed straight to the
 * existing <Plot> wrapper (`@/components/Plot`), so they are Plotly-compatible
 * and inherit the shared dark theme.
 */
export interface PlotSpec {
  /** Stable id for React keys; falls back to `title` if omitted. */
  id?: string;
  /** Shown as a caption above the plot. */
  title: string;
  data: Data[];
  layout?: Partial<Layout>;
}

// ----------------------------------------------------------------------------
// Studio definition — what a studio registers
// ----------------------------------------------------------------------------

/** A named bundle of param values a studio can offer as a quick starting point. */
export interface StudioPreset {
  id: string;
  label: string;
  values: ParamValues;
}

/**
 * A studio definition. Register one via `@/lib/studio/registry`.
 *
 * `compute` receives the current param values and returns the plots to show.
 * It MAY be async (e.g. it awaits the WASM core) — the shell always awaits it
 * and debounces invocations on param change.
 */
export interface StudioDef {
  /** Unique, URL/DOM-safe id. */
  id: string;
  /** Short label shown in the studio picker. */
  label: string;
  /** One- or two-line description / notes shown in the shell header. */
  description: string;
  /** Typed parameter schema → rendered as the left control panel. */
  params: ParamSchema;
  /** Map current params → plots. Pure client fn, WASM call, or analysis entry. */
  compute: (params: ParamValues) => PlotSpec[] | Promise<PlotSpec[]>;
  /** Optional named starting points. */
  presets?: StudioPreset[];
}

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------

/** Build the initial ParamValues object from a schema's defaults. */
export function defaultParamValues(schema: ParamSchema): ParamValues {
  const values: ParamValues = {};
  for (const p of schema) values[p.key] = p.default_value;
  return values;
}
