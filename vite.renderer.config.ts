import { defineConfig } from "vite";

export default defineConfig({
    root: "launcher",
    base: "./",
    build: {
        outDir: "../.vite/renderer/main_window",
        emptyOutDir: true
    }
});
