'use client';

/**
 * StudiosArea — the top-level "Studios" surface. Lists every registered studio
 * and renders the selected one inside <StudioShell>.
 *
 * This whole module (and therefore the studio registry + Plotly-backed shell) is
 * lazy-mounted by page.tsx via `dynamic(ssr:false)`, and only when the Studios
 * tab is active. Importing `@/lib/studio/studios` here (rather than at page
 * scope) keeps the registry and every studio out of the first-load chunk.
 */

import { useMemo, useState } from 'react';
import StudioShell from './StudioShell';
import { listStudios } from '@/lib/studio/registry';
// Side-effect import: registers the built-in studios into the registry. Lives
// here so it lands in the lazy Studios chunk, not the first-load bundle.
import '@/lib/studio/studios';

export default function StudiosArea() {
  const studios = useMemo(() => listStudios(), []);
  const [activeId, setActiveId] = useState<string>(() => studios[0]?.id ?? '');

  const active = studios.find((s) => s.id === activeId) ?? studios[0];

  if (!active) {
    return <div className="card placeholder">No studios registered.</div>;
  }

  return (
    <div className="studiosArea">
      {studios.length > 1 ? (
        <div className="studioPicker" role="tablist" aria-label="Studios">
          {studios.map((s) => {
            const selected = s.id === active.id;
            return (
              <button
                key={s.id}
                type="button"
                role="tab"
                aria-selected={selected}
                className={`studioPickerBtn${selected ? ' studioPickerActive' : ''}`}
                onClick={() => setActiveId(s.id)}
              >
                {s.label}
              </button>
            );
          })}
        </div>
      ) : null}

      <StudioShell studio={active} />
    </div>
  );
}
