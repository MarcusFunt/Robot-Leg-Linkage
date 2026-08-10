import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const root = new URL("../", import.meta.url);
const source = (path) => readFile(new URL(path, root), "utf8");

test("mobile analysis UX keeps traceability, navigation, camera fits and accessible reactions", async () => {
  const [page, ui, css] = await Promise.all([
    source("app/page.tsx"),
    source("app/linkage-ui.tsx"),
    source("app/mobile-enhancements.css"),
  ]);

  assert.match(page, /Fit mechanism/);
  assert.match(page, /Fit motion window/);
  assert.match(page, /Fit full path/);
  assert.match(page, /Peak joint reaction/);
  assert.match(page, /peakReactionJoint/);
  assert.match(page, /focusResult/);
  assert.match(page, /nav-badge/);
  assert.match(page, /reaction-mobile/);
  assert.match(page, /link-label/);
  assert.match(ui, /plot-modal/);
  assert.match(ui, /compact && !detail \? \[0, 0\.5, 1\]/);
  assert.match(css, /display:\s*grid\s*!important/);
  assert.match(css, /\.plot-svg[\s\S]*min-width:\s*0\s*!important/);
  assert.match(css, /\.reaction-desktop\{display:none\}/);
});
