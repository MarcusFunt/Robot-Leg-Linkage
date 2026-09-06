import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import {
  classifyScreeningCheck,
  hipQToSolverAngle,
  solverAngleToHipQ,
  summarizeDesign,
  takeoffMetrics,
  targetDiagonal,
  wheelDerivative,
  wheelPathMetrics,
} from "../lib/robot-workbench.mjs";

const ascento = {
  groundX: -28.6788218176,
  groundY: -40.9576022144,
  crank: 100,
  coupler: 25,
  rocker: 100,
  toolAlong: 125,
  toolOffset: 0,
  minAngle: 310,
  maxAngle: 355,
  motionProfile: "sinusoidal",
  maxVelocity: 360,
  maxAcceleration: 1500,
  maxJerk: 10000,
  cycleTime: 4.2,
  branch: -1,
  supportForce: 100,
  gravity: true,
  crankMass: 0.12,
  legMass: 0.28,
  rockerMass: 0.15,
  toolMass: 0.08,
  pinDiameter: 8,
  linkThickness: 4,
  shearPlanes: 2,
  allowableShear: 120,
  allowableBearing: 80,
  gearRatio: 10,
  gearEfficiency: 90,
  motorContinuous: 0.6,
  motorPeak: 1.2,
};

test("Ascento robot hip angle convention round-trips", () => {
  assert.equal(solverAngleToHipQ(337), 78);
  assert.equal(hipQToSolverAngle(78), 337);
  for (const theta of [310, 337, 355]) {
    assert.equal(hipQToSolverAngle(solverAngleToHipQ(theta)), theta);
  }
});

test("robot workbench computes a usable diagonal wheel target", () => {
  const target = targetDiagonal(ascento, 0.5);
  assert.ok(target);
  assert.ok(target.length > 10);
  const metrics = wheelPathMetrics(ascento, ascento, 0.5);
  assert.equal(metrics.valid, true);
  assert.ok(metrics.samples.length > 20);
  assert.ok(metrics.verticalStroke > 0);
  assert.ok(metrics.foreAftExcursion > 0);
  assert.ok(metrics.targetRms >= 0);
  assert.ok(metrics.straightnessRms >= 0);
  assert.ok(metrics.minTransmission > 0);
});

test("robot-frame derivatives and takeoff metrics are finite", () => {
  const derivative = wheelDerivative(ascento, 337);
  assert.ok(derivative);
  assert.ok(Number.isFinite(derivative.dxDq));
  assert.ok(Number.isFinite(derivative.dzDq));
  const takeoff = takeoffMetrics(ascento, ascento, { comOffsetX: 0, comOffsetZ: 0 });
  assert.ok(takeoff);
  assert.ok(Number.isFinite(takeoff.wheelMinusComX));
  assert.ok(Number.isFinite(takeoff.dxDq));
  assert.ok(Number.isFinite(takeoff.dzDq));
});

test("design summaries support A/B comparison metrics", () => {
  const summary = summarizeDesign(ascento, ascento);
  assert.ok(summary.verticalStroke > 0);
  assert.ok(summary.foreAftExcursion > 0);
  assert.ok(summary.minTransmission > 0);
  assert.ok(summary.targetRms >= 0);
});

test("screening status classification is structural rather than copy-driven", () => {
  assert.deepEqual(
    classifyScreeningCheck({ index: 0, danger: true }),
    { category: "dynamic", state: "critical" },
  );
  assert.deepEqual(
    classifyScreeningCheck({ index: 2, warning: true }),
    { category: "geometry", state: "warning" },
  );
  assert.deepEqual(
    classifyScreeningCheck({ index: 3, pass: true }),
    { category: "static", state: "pass" },
  );
  assert.deepEqual(
    classifyScreeningCheck({ index: 0, blocked: true }),
    { category: "geometry", state: "critical" },
  );
});

test("layout loads one consolidated workbench enhancement layer", () => {
  const layout = readFileSync(new URL("../app/layout.tsx", import.meta.url), "utf8");
  assert.match(layout, /import "\.\/workbench\.css"/);
  assert.match(layout, /RobotWorkbench/);
  assert.doesNotMatch(layout, /mobile-enhancements\.css/);
  assert.doesNotMatch(layout, /motion-profile\.css/);
  assert.doesNotMatch(layout, /ux-enhancements\.css/);
  assert.doesNotMatch(layout, /UXEnhancements/);
});

test("workbench exposes the robot-leg design workflow", () => {
  const source = readFileSync(new URL("../app/robot-workbench.tsx", import.meta.url), "utf8");
  for (const expected of [
    "Robot-leg workbench",
    "Ascento-inspired V1",
    "Hip link HI",
    "Rocker PK",
    "Coupler IK",
    "Wheel offset IW",
    "Hip q",
    "Vertical stroke",
    "Takeoff W−COM x",
    "Capture A",
    "Capture B",
    "Export JSON",
    "Import JSON",
    "COM reference",
  ]) assert.match(source, new RegExp(expected.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")));
});
