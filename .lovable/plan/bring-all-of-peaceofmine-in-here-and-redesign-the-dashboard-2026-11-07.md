# Bring all of PeaceOfMine in here and redesign the dashboard

## Goal
This project holds your whole PeaceOfMine repository, so I can change any part of it. First job: redesign the car dashboard so it uses one consistent look and fits a laptop screen with no scrolling. I can test the website here. Car code I can edit and check for obvious mistakes, but I can't run it on ROS or the hardware.

## How the repository works
```text
this project (new GitHub repo once you connect it)
  car/          <- full PeaceOfMine repo (ROS packages, util/, DevTools/, docs ...)
  src/, public/ <- small Lovable web shell that previews car/.../dashboard
```
- The Lovable project has to stay a buildable website, so PeaceOfMine goes in a `car/` folder rather than at the top level.
- The dashboard is edited in place at `car/src/peaceofmine_operator/dashboard/`. The preview shows those same files, so there's no second copy.
- Syncing: connect this project to GitHub, which creates a new repo. Then pick one option:
  - **A (simplest):** the new repo becomes the car's home. On the car: clone it, `cd car`, then `util/build` / `util/run` as today.
  - **B (keep the old repo):** from your laptop, push `car/` back to PeaceOfMine with one command: `git subtree push --prefix car <PeaceOfMine-url> main`. I'll write it as a script.
- I'll leave out `.git` folders, build output and very large binary files when importing. I'll list anything left out.

## Dashboard redesign (first task)
- **One screen, no scrolling (1280×720 and larger):** a fixed grid.
  - Top bar: connection status, Take control, a red Stop that's always visible, and Settings.
  - Left (largest): the camera, with drive status overlaid.
  - Right column: the task workbench (search, probe and alignment actions).
  - Bottom strip: the detector chart, map, power and target log, shown as compact tiles. Longer content scrolls inside its own tile.
- **Consistent design language:** one set of colors, spacing, corner radius, type sizes, button styles and status chips (ok / warning / blocked / error). The look is a dark, high-contrast control-room style.
- **Less confusing:** related controls are grouped under clear headings. Blocking reasons are shown the same way on every button. Drive, probe and detector diagnostics become tabs inside one panel.
- **Settings dialog:** restyled to match, with the same tabs and fields.
- No change to safety logic, ROS topics or messages in this task.

## Testing
- Here: a pretend-car data source, shaped from what `web_bridge.py` really sends, so every panel shows live-looking data in the preview. It only switches on in the preview.
- On the real car: update the code (option A or B), then run `util/build`, `util/run` and the launch command. Open the usual Tailscale URL on port 8080.

## Technical details
- Import the repo tree (via GitHub tarball) into `car/`, excluding `.git`, `build/`, `install/`, `log/` and files over 5 MB.
- Add a small Vite middleware in `vite.config.ts` (dev only) that serves `car/src/peaceofmine_operator/dashboard/` at `/dashboard/`. For production preview, copy that folder into the build output.
- Add `car/src/peaceofmine_operator/dashboard/mock-car.js`, loaded only with `?mock=1` (harmless on the car). It patches `WebSocket` for `/ws`.
- Rewrite `style.css` around CSS custom properties and a `100vh` grid. Restructure `index.html` while keeping every `id`/`data-*` hook that `app.js` uses.
- `src/routes/index.tsx` becomes a full-screen page embedding `/dashboard/index.html?mock=1`, with a proper title and meta.
- Exclude `car/` from the web app's lint, typecheck and test globs.
- Check with Playwright at 1280×720 and 1920×1080: no document scroll, no overlaps, no console errors.
- Add `scripts/push-car.sh` for option B.
- Record in `AGENTS.md`:
  - `car/` is the vendored PeaceOfMine repo.
  - The web shell only previews it.
  - The dashboard's DOM hooks must be preserved.
