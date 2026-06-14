'use client';

/**
 * Thin client-only Plotly wrapper. Plotly touches `window`/`document`, so it is
 * imported lazily inside an effect (never at module scope) to avoid SSR crashes
 * during static export.
 */

import { useEffect, useRef } from 'react';
import type { Data, Layout, Config } from 'plotly.js-dist-min';

export interface PlotProps {
  data: Data[];
  layout?: Partial<Layout>;
  config?: Partial<Config>;
  className?: string;
  style?: React.CSSProperties;
}

// Shared dark theme for all plots.
const baseLayout: Partial<Layout> = {
  paper_bgcolor: '#0f141b',
  plot_bgcolor: '#0f141b',
  font: { color: '#c8d2dc', family: 'ui-monospace, monospace', size: 12 },
  margin: { l: 56, r: 16, t: 36, b: 44 },
  xaxis: { gridcolor: '#1f2a36', zerolinecolor: '#2a3744' },
  yaxis: { gridcolor: '#1f2a36', zerolinecolor: '#2a3744' },
  legend: { bgcolor: 'rgba(0,0,0,0)', orientation: 'h', y: -0.2 },
};

const baseConfig: Partial<Config> = {
  displayModeBar: true,
  displaylogo: false,
  responsive: true,
  modeBarButtonsToRemove: ['lasso2d', 'select2d'],
};

export default function Plot({ data, layout, config, className, style }: PlotProps) {
  const ref = useRef<HTMLDivElement>(null);

  useEffect(() => {
    let cancelled = false;
    const el = ref.current;
    if (!el) return;

    (async () => {
      const Plotly = (await import('plotly.js-dist-min')).default;
      if (cancelled || !ref.current) return;
      const mergedLayout: Partial<Layout> = {
        ...baseLayout,
        ...layout,
        xaxis: { ...baseLayout.xaxis, ...(layout?.xaxis as object) },
        yaxis: { ...baseLayout.yaxis, ...(layout?.yaxis as object) },
      };
      await Plotly.react(ref.current, data, mergedLayout, {
        ...baseConfig,
        ...config,
      });
    })();

    return () => {
      cancelled = true;
      if (el) {
        import('plotly.js-dist-min').then((m) => m.default.purge(el)).catch(() => {});
      }
    };
  }, [data, layout, config]);

  return (
    <div
      ref={ref}
      className={className}
      style={{ width: '100%', height: 360, ...style }}
    />
  );
}
