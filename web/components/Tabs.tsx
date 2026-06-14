'use client';

/**
 * Hand-rolled accessible tab strip (no external dep).
 *
 * Implements the WAI-ARIA tabs pattern: role=tablist/tab/tabpanel, roving
 * tabindex, aria-selected, and arrow-key / Home / End navigation.
 *
 * Crucially this is a *lazy* tab host: only the active panel's `render()` is
 * invoked, so inactive tabs mount nothing. That is what keeps every Plotly /
 * Leaflet chart from instantiating at once on first load (which caused the
 * Canvas2D readback storm + setTimeout-handler violation).
 */

import { useRef } from 'react';

export interface TabDef {
  id: string;
  label: string;
  /** Rendered only while this tab is active (lazy mount). */
  render: () => React.ReactNode;
}

interface TabsProps {
  tabs: TabDef[];
  active: string;
  onChange: (id: string) => void;
}

export default function Tabs({ tabs, active, onChange }: TabsProps) {
  const tabRefs = useRef<Record<string, HTMLButtonElement | null>>({});
  const activeIndex = Math.max(
    0,
    tabs.findIndex((t) => t.id === active),
  );

  function focusTab(index: number) {
    const t = tabs[index];
    if (!t) return;
    onChange(t.id);
    // Move DOM focus to the newly selected tab (roving tabindex).
    requestAnimationFrame(() => tabRefs.current[t.id]?.focus());
  }

  function onKeyDown(e: React.KeyboardEvent) {
    switch (e.key) {
      case 'ArrowRight':
      case 'ArrowDown':
        e.preventDefault();
        focusTab((activeIndex + 1) % tabs.length);
        break;
      case 'ArrowLeft':
      case 'ArrowUp':
        e.preventDefault();
        focusTab((activeIndex - 1 + tabs.length) % tabs.length);
        break;
      case 'Home':
        e.preventDefault();
        focusTab(0);
        break;
      case 'End':
        e.preventDefault();
        focusTab(tabs.length - 1);
        break;
      default:
        break;
    }
  }

  const activeTab = tabs[activeIndex];

  return (
    <div className="tabs">
      <div
        className="tablist"
        role="tablist"
        aria-label="Analysis perspectives"
        onKeyDown={onKeyDown}
      >
        {tabs.map((t) => {
          const selected = t.id === active;
          return (
            <button
              key={t.id}
              ref={(el) => {
                tabRefs.current[t.id] = el;
              }}
              id={`tab-${t.id}`}
              role="tab"
              type="button"
              aria-selected={selected}
              aria-controls={`tabpanel-${t.id}`}
              tabIndex={selected ? 0 : -1}
              className={`tab${selected ? ' tabActive' : ''}`}
              onClick={() => onChange(t.id)}
            >
              {t.label}
            </button>
          );
        })}
      </div>

      <div
        id={`tabpanel-${activeTab.id}`}
        role="tabpanel"
        aria-labelledby={`tab-${activeTab.id}`}
        tabIndex={0}
        className="tabpanel"
      >
        {/* Lazy: only the active tab's content is constructed/mounted. */}
        {activeTab.render()}
      </div>
    </div>
  );
}
