// @lovable.dev/vite-tanstack-config already includes the following — do NOT add them manually
// or the app will break with duplicate plugins:
//   - TanStack devtools (dev-only, first), tanstackStart, viteReact, tailwindcss, tsConfigPaths,
//     nitro (build-only using cloudflare as a default target), VITE_* env injection, @ path alias,
//     React/TanStack dedupe, error logger plugins, and sandbox detection (port/host/strictPort).
// Never set ssr.external or resolve.external: the Worker has no runtime module resolution, so
// the build fails. Never add compression (compressPublicAssets, vite-plugin-compression):
// hosting already compresses at the edge.
// You can pass additional config via defineConfig({ vite: { ... }, etc... }) if needed.
import { defineConfig } from "@lovable.dev/vite-tanstack-config";
import { existsSync, readdirSync, readFileSync, statSync } from "node:fs";
import { extname, join, normalize } from "node:path";
import type { Plugin } from "vite";

// Dev-only: serve the car's real dashboard folder at /dashboard/ so the preview
// shows exactly the files the car serves (no copy).
const DASHBOARD_DIR = join(process.cwd(), "car/src/peaceofmine_operator/dashboard");
const TYPES: Record<string, string> = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".svg": "image/svg+xml",
};

function carDashboard(): Plugin {
  return {
    name: "car-dashboard",
    apply: "serve",
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        const url = (req.url || "").split("?")[0] ?? "";
        // Pretend car cameras: /camera/<id> -> public/mock/<id>.jpg (a still frame).
        const cam = /^\/camera\/([A-Za-z0-9_-]+)$/.exec(url);
        if (cam) {
          const still = join(process.cwd(), "public/mock", `${cam[1]}.jpg`);
          if (!existsSync(still)) return next();
          res.setHeader("Content-Type", "image/jpeg");
          res.setHeader("Cache-Control", "no-store");
          return res.end(readFileSync(still));
        }
        if (!url.startsWith("/dashboard/")) return next();
        let rel = url.slice("/dashboard/".length) || "index.html";
        if (rel.startsWith("assets/")) rel = rel.slice("assets/".length);
        const file = normalize(join(DASHBOARD_DIR, rel));
        if (!file.startsWith(DASHBOARD_DIR) || !existsSync(file) || !statSync(file).isFile()) return next();
        res.setHeader("Content-Type", TYPES[extname(file)] || "application/octet-stream");
        res.setHeader("Cache-Control", "no-store");
        res.end(readFileSync(file));
      });
    },
  };
}

// Build: copy the same dashboard files into the client output so the published
// preview serves /dashboard/ too (dev middleware above only runs in `vite dev`).
function carDashboardBuild(): Plugin {
  return {
    name: "car-dashboard-build",
    apply: "build",
    applyToEnvironment: (env) => env.name === "client",
    generateBundle() {
      for (const name of readdirSync(DASHBOARD_DIR)) {
        const file = join(DASHBOARD_DIR, name);
        if (!statSync(file).isFile() || !TYPES[extname(file)]) continue;
        const fileName = name === "index.html" ? "dashboard/index.html" : `dashboard/assets/${name}`;
        this.emitFile({ type: "asset", fileName, source: readFileSync(file) });
      }
    },
  };
}

export default defineConfig({
  tanstackStart: {
    // Redirect TanStack Start's bundled server entry to src/server.ts (our SSR error wrapper).
    // nitro/vite builds from this
    server: { entry: "server" },
  },
  vite: {
    plugins: [carDashboard(), carDashboardBuild()],
    server: { watch: { ignored: ["**/car/**"] } },
  },
});
