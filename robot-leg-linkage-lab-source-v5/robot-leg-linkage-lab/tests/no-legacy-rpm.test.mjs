import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const root = new URL("../", import.meta.url);

test("legacy constant-RPM/input-acceleration controls are removed from active UI and solver", async () => {
  const [page, ui, core] = await Promise.all([
    readFile(new URL("app/page.tsx", root), "utf8"),
    readFile(new URL("app/linkage-ui.tsx", root), "utf8"),
    readFile(new URL("lib/linkage-core.mjs", root), "utf8"),
  ]);
  assert.doesNotMatch(page, /config\.rpm|config\.inputAccel/);
  assert.doesNotMatch(ui, /key:\s*["']rpm["']|key:\s*["']inputAccel["']/);
  assert.doesNotMatch(core, /config\.rpm|config\.inputAccel/);
});
