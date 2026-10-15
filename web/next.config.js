/** @type {import('next').NextConfig} */
const nextConfig = {
  // The simulation runs entirely client-side (WASM); no server routes are needed,
  // so we emit a static SPA that drops straight onto Vercel / any static host.
  output: 'export',
  reactStrictMode: true,
  // Cesium embeds a WASM module as a binary JS string; Next's SWC minifier serialized it
  // as a template literal with octal escapes (illegal -> ChunkLoadError at runtime). Terser
  // escapes it correctly. See the Cesium 3D Globe / Scenario Author chunks.
  swcMinify: false,
  images: {
    // next/image optimization needs a server; static export uses raw <img>/unoptimized.
    unoptimized: true,
  },
  // Allow importing the Emscripten glue at runtime without bundler interference.
  webpack: (config) => {
    config.resolve.fallback = {
      ...config.resolve.fallback,
      fs: false,
      path: false,
      crypto: false,
    };
    return config;
  },
};

module.exports = nextConfig;
