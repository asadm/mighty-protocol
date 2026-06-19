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
      server.middlewares.use("/mighty_loopclosure_device.wasm", (req, res, next) => {
        if (req.method !== "GET" || !existsSync(loopclosureWasmPath)) {
          next();
          return;
        }
        res.setHeader("Content-Type", "application/wasm");
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
      },
    },
  },
  server: {
    port: 8090,
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
