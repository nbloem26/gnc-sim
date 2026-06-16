import type { Metadata } from 'next';
import Link from 'next/link';
import './globals.css';

export const metadata: Metadata = {
  title: 'GNC Studio — guidance, navigation & control simulator',
  description:
    'GNC Studio — an interactive guided-interceptor guidance, navigation & control simulator running entirely in your browser via WebAssembly. Flight dynamics, multi-sensor tracking & fusion, optimal guidance, ballistic & hypersonic threats, and many-on-many engagement campaigns.',
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
            <span className="title">GNC&nbsp;Studio</span>
            <span className="sub">guided-interceptor GNC · dynamics · estimation · fusion · guidance · threats · campaigns · in-browser WASM</span>
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
