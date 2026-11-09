<!-- LOVABLE:BEGIN -->
> [!IMPORTANT]
> This project is connected to [Lovable](https://lovable.dev). Avoid rewriting
> published git history — force pushing, or rebasing/amending/squashing commits
> that are already pushed — as it rewrites history on Lovable's side and the
> user will likely lose their project history.
>
> Commits you push to the connected branch sync back to Lovable and show up in
> the editor, so keep the branch in a working state.

## What Lovable manages

Rules outside this block are the project's own. Change the pieces below in
Lovable, not by hand. With the [Lovable MCP](https://mcp.lovable.dev)
connected, `send_message` asks Lovable to do it; otherwise ask the user to
use the editor.

- **Database.** Do not add or edit `supabase/migrations/` or `drizzle/`.
  Ask Lovable to change the schema.
- **Generated files.** Do not edit a file whose header says it is automatically
  generated.
- **AI gateway.** Keep model calls on Lovable's gateway, and keep
  `LOVABLE_API_KEY` out of browser code.
- **Connectors.** Lovable provides native connectors. `list_connectors` shows
  which are enabled; `add_connector` only returns the dashboard link. When a
  connector is enabled, calls go through the connector gateway and need
  `LOVABLE_API_KEY`. When Lovable does not provide one, users can bring their
  own key and store it in Secrets.
- **Secrets.** Lovable manages them. Do not put secret values in the repo.
<!-- LOVABLE:END -->

## Project structure

- `car/` is the full PeaceOfMine ROS 2 repository, vendored here; the car runs from it (`cd car && util/build && util/run`). The web app in `src/` is only a preview shell. Why: Lovable can only edit files inside this project.
- The operator dashboard lives once, at `car/src/peaceofmine_operator/dashboard/`, and the car's `operator_web_node.py` serves it from there. A Vite middleware serves the same folder at `/dashboard/` in dev, and a build plugin emits it into the client output for the published site. Never copy it into `public/`. Why: one source of truth for both the car and the preview.
- Dashboard edits must keep every `id` / `data-*` hook that `app.js`, `adc.js`, `actuator-settings.js` and the Python browser tests use. New dashboard asset files must be added to the allow-list in `operator_web_node.py`. Why: the car refuses to serve files that aren't on the list.
- Dashboard colors, spacing and type come only from the tokens at the top of `dashboard/style.css`. Why: one consistent design language.
- `dashboard/layout.js` is presentation-only. It reads the `state` global from app.js and never sends commands. It sets the mission stage (drive / confirm / probe) from `state.inspection.phase` and `state.probe.cycle_active`, and it swaps which camera is large using CSS grid areas, never by moving DOM nodes. Why: keyboard focus and the camera code in app.js depend on fixed elements.
- `dashboard/mock-car.js` (loaded only with `?mock=1`, and not on the car allow-list) fakes `/ws` and `/api/adc` for the preview. Keep its message shapes in line with `operator_control_node.py`, `web_bridge.py` and `adc_telemetry.py`.
- The car homes the connected probe before arc confirmation and once before each probe scan. Inspection owns this action and reports its progress to every dashboard.
- Dashboard asset links in `index.html` are relative (`assets/...`) so the page works at the car's `/` and the preview's `/dashboard/`; the Python browser tests accept both forms. Why: one HTML file for both hosts.
- Every Setup window panel has the same shape: `panel-head` (title with an (i) tip, one action or badge), an optional `status-bar`, a `panel-grid` of `set-card`s (numbered `step`s where order matters), then `set-more` drawers for rarely used settings. Inside Setup, spacing uses only `--s1` (label to control), `--s2` (siblings in a row) and `--s3` (everything else). Sections are grouped in the left nav (Driving / Arm & probe / Sensors). Why: one place to look per task, with the same layout on every panel.
- Dashboard explanations go in `data-tip` attributes (`<span class="tip">` for the small (i)), not in running text. `layout.js` renders one shared tooltip, and appends it inside an open `<dialog>` so it shows above the modal. Why: short, scannable screens.
- The live search alert, operator preferences, inspection and mission log are car-owned. The preview uses a local mock. A browser refresh or lease transfer must preserve completed results and stop ongoing actuation.
- `car/` is excluded from the web app's lint, prettier and typecheck.
