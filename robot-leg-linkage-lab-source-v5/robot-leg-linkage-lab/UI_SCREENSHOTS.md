# High-resolution UI screenshots

The UI screenshot harness lives in `scripts/capture-ui.mjs`. It is intentionally **local-only**: the repository does not run screenshot generation in GitHub Actions and does not upload screenshot artifacts.

The harness uses a pinned Playwright runtime (`1.62.0`) installed into the gitignored `.ui-screenshot-runtime/` directory, so the application dependency lockfile does not need to include browser automation tooling.

## What it captures

For both **S-curve** and **Sinusoidal** motion profiles, the harness captures:

- full-page screenshots
- 2560×1440, 1920×1080 and 1440×1000 desktop layouts at 2× device scale
- 768×1024 tablet layout at 2× device scale
- 430×932 and 390×844 phone layouts at 3× device scale
- the simulator at 0%, 25%, 50%, 75% and 100% of the cycle
- every main input tab
- input tabs with visible disclosure sections expanded
- major metric/result sections
- individual detail cards
- every plot card
- all `<details>` sections opened in a full-page capture
- fullscreen plot dialogs on a representative desktop and phone viewport

Pass `--maximal` to capture every fullscreen plot at every viewport.

A `manifest.json` file is written beside the screenshots with the source URL, browser version, viewport/DPR, motion profile and every generated image path.

## Local capture from the development server

From the app directory, start the site on the expected port:

```bash
npm run dev -- --host 127.0.0.1 --port 4173
```

In another terminal:

```bash
npm run screenshots
```

The first run installs the pinned Playwright runtime and Chromium automatically. Screenshots are written to `screenshots/ui/`.

Use a custom URL or output directory with:

```bash
node scripts/capture-ui.mjs --url http://127.0.0.1:4173/ --out screenshots/custom
```

## Capture the deployed GitHub Pages site from your local machine

This still runs Playwright and Chromium on your own computer; GitHub only serves the already-deployed website.

```bash
npm run screenshots:live
```

For the largest possible local review set:

```bash
npm run screenshots:maximal
```

## GitHub Actions and artifacts

Screenshot generation is deliberately **not** configured as a GitHub Actions workflow. Running the local commands above does not consume GitHub-hosted runner time and does not create GitHub Actions artifacts.

The normal project workflows for unrelated tasks, such as the existing regression checks or Pages deployment, are separate from this screenshot harness.

## Notes

- CSS animations/transitions and carets are disabled during capture to improve reproducibility.
- The harness drives the playhead directly instead of running real-time animation, so screenshots are deterministic.
- Full-page screenshots attempt device-pixel resolution first. If Chromium rejects an extremely tall high-DPI bitmap, only that full-page image falls back to CSS-pixel scale; individual section, simulator and plot screenshots remain high-DPI.
- Generated screenshots and the Playwright tooling runtime are intentionally gitignored.
