import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import {
  motionWindowReachability,
  sampleAngleRange,
  solvePose,
} from "../lib/linkage-analysis.mjs";

const preset = JSON.parse(
  readFileSync(new URL("../configs/ascento-v1-diagonal.json", import.meta.url), "utf8"),
);
const config = preset.labConfig;

test("Ascento V1 preset keeps the reconstructed 0.5:1:1:0.25:1 geometry", () => {
  assert.ok(Math.abs(Math.hypot(config.groundX, config.groundY) - 50) < 1e-9);
  assert.equal(config.rocker, 100); // H-I, driven input
  assert.equal(config.crank, 100); // P-K
  assert.equal(config.coupler, 25); // K-I
  assert.equal(config.toolAlong, 125); // K-I plus I-W
  assert.equal(config.toolOffset, 0);
  assert.equal(config.branch, -1);
});

test("fixed pivots use the corrected, near-vertical Ascento mounting direction", () => {
  // Lab coordinates are O2=P and O4=H. Therefore H->P is the negative of
  // the stored ground vector and should point about +55 degrees from +x.
  const hToP = Math.atan2(-config.groundY, -config.groundX) * 180 / Math.PI;
  assert.ok(Math.abs(hToP - 55) < 1e-9);
});

test("reference pose has the intended Ascento-like topology", () => {
  const pose = solvePose(config, preset.referencePose.theta4_deg);
  assert.ok(pose);

  // O2=P is above/right of O4=H in the physical drawing.
  assert.ok(pose.O2.x > pose.O4.x);
  assert.ok(pose.O2.y > pose.O4.y);

  // B=I is below H, A=K is above I, and T=W is below I.
  assert.ok(pose.B.y < pose.O4.y);
  assert.ok(pose.A.y > pose.B.y);
  assert.ok(pose.T.y < pose.B.y);
});

test("configured motion window is fully reachable and predominantly vertical", () => {
  const reachability = motionWindowReachability(config);
  assert.equal(reachability.fullyReachable, true);

  const points = sampleAngleRange(config.minAngle, config.maxAngle, 0.5)
    .map((angle) => solvePose(config, angle)?.T)
    .filter(Boolean);
  assert.ok(points.length > 2);

  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  const horizontalSpan = Math.max(...xs) - Math.min(...xs);
  const verticalSpan = Math.max(...ys) - Math.min(...ys);

  assert.ok(verticalSpan > 2 * horizontalSpan);
});
