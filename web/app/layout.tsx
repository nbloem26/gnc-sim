import type { Metadata } from 'next';
import Link from 'next/link';
import './globals.css';

export const metadata: Metadata = {
  title: 'GNC Sim — Browser WASM',
  description:
    'Interactive guidance, navigation & control simulation running in-browser via WebAssembly.',
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body>
        <header className="topbar">
          <div className="brand">
            <span className="title">GNC&nbsp;SIM</span>
            <span className="sub">proportional-navigation homing · in-browser WASM</span>
          </div>
          <nav className="nav">
            <Link href="/">Simulator</Link>
            <Link href="/validation">Engineering Validation</Link>
          </nav>
        </header>
        {children}
      </body>
    </html>
  );
}
