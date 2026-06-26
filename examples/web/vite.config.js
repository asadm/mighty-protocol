import { defineConfig } from "vite";
import { createReadStream, existsSync, readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const loopclosureWasmPath = resolve(here, "../../lib/loopclosure/wasm/lib/mighty_loopclosure_device.wasm");

function loopclosureWasmAsset() {
  return {
    name: "mighty-loopclosure-wasm-asset",
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        const url = new URL(req.url || "", "http://localhost");
        if (req.method !== "GET" || url.pathname !== "/mighty_loopclosure_device.wasm" || !existsSync(loopclosureWasmPath)) {
          next();
          return;
        }
        res.setHeader("Content-Type", "application/wasm");
        res.setHeader("Cache-Control", "no-store");
        createReadStream(loopclosureWasmPath).pipe(res);
      });
    },
    generateBundle() {
      if (!existsSync(loopclosureWasmPath)) return;
      this.emitFile({
        type: "asset",
        fileName: "mighty_loopclosure_device.wasm",
        source: readFileSync(loopclosureWasmPath),
      });
    },
  };
}

export default defineConfig({
  plugins: [loopclosureWasmAsset()],
  build: {
    rollupOptions: {
      input: {
        index: resolve(here, "index.html"),
        loopclosure: resolve(here, "loopclosure.html"),
        mapper: resolve(here, "mapper.html"),
      },
    },
  },
  server: {
    port: 8090,
    fs: {
      allow: [resolve(here, "../..")],
    },
    proxy: {
      "/stream": {
        target: "http://localhost:8080",
        changeOrigin: true,
      },
      "/command": {
        target: "http://localhost:8080",
        changeOrigin: true,
      },
    },
  },
});
