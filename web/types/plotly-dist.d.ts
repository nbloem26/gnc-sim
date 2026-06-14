/**
 * `plotly.js-dist-min` ships no types. Re-map it to the full `@types/plotly.js`
 * declarations (the runtime API is identical) and expose the default export used
 * by our dynamic `import('plotly.js-dist-min')`.
 */
declare module 'plotly.js-dist-min' {
  import * as Plotly from 'plotly.js';
  export = Plotly;
  export as namespace Plotly;
}
