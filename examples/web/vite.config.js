import { defineConfig } from "vite";

export default defineConfig({
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
