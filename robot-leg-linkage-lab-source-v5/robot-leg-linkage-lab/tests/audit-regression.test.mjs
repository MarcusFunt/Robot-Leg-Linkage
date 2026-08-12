import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { inverseDynamics, solvePose } from "../lib/linkage-analysis.mjs";

const root = new URL("../", import.meta.url);
const source = async (path) => readFile(new URL(path, root), "utf8");

test("input drafts are explicitly reset after model resets and profile changes", async () => {
  const [page, ui] = await Promise.all([
    source("app/page.tsx"),
    source("app/linkage-ui.tsx"),
  ]);
  assert.match(page, /inputResetVersion/);
  assert.match(page, /setInputResetVersion/);
  assert.match(ui, /resetVersion/);
});

test("motion controls are unavailable when the requested path is invalid", async () => {
  const page = await source("app/page.tsx");
  assert.match(page, /motionPlayable\s*=\s*Boolean/);
  assert.match(page, /disabled=\{!motionPlayable\}/);
  assert.match(page, /Motion window unreachable/);
});

test("status consolidates a motion-path blocker and exposes model scope", async () => {
  const [page, ux] = await Promise.all([
    source("app/page.tsx"),
    source("app/ux-enhancements.tsx"),
  ]);
  assert.match(page, /Preliminary planar model/);
  assert.match(ux, /Geometry blocker/);
  assert.match(ux, /invalid motion path/);
});

test("plots support keyboard scrubbing and modal focus recovery", async () => {
  const ui = await source("app/linkage-ui.tsx");
  assert.match(ui, /closeRef/);
  assert.match(ui, /event\.key\s*===\s*"Escape"/);
  assert.match(ui, /ArrowRight/);
  assert.match(ui, /aria-describedby/);
});

test("mobile uses a horizontal, touch-sized motion control strip", async () => {
  const [page, css] = await Promise.all([
    source("app/page.tsx"),
    source("app/globals.css"),
  ]);
  assert.match(page, /mobile-motion-controls/);
  assert.match(css, /\.mobile-motion-controls/);
  assert.match(css, /min-height:\s*44px/);
});

test("inverse dynamics agrees with an independent virtual-work inertia calculation", () => {
  const config = {
    groundX: 45,
    groundY: -40,
    crank: 40,
    coupler: 45,
    rocker: 60,
    toolAlong: 120,
    toolOffset: 0,
    minAngle: 165,
    maxAngle: 250,
    motionProfile: "s-curve",
    maxVelocity: 360,
    maxAcceleration: 1500,
    maxJerk: 10000,
    cycleTime: 2,
    branch: -1,
    supportForce: 0,
    gravity: false,
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
  const omega = 3.2,
    alpha = -17,
    h = 1e-4;
  const dot = (a, b) => a.x * b.x + a.y * b.y;
  const sub = (a, b) => ({ x: a.x - b.x, y: a.y - b.y });
  const scale = (a, k) => ({ x: a.x * k, y: a.y * k });
  const center = (a, b, ratio) => ({
    x: a.x + (b.x - a.x) * ratio,
    y: a.y + (b.y - a.y) * ratio,
  });
  const angle = (a, b) => Math.atan2(b.y - a.y, b.x - a.x);
  const signed = (a, b) => Math.atan2(Math.sin(a - b), Math.cos(a - b));
  const state = (q) => {
    const p = solvePose(config, (q * 180) / Math.PI);
    const legCenter = center(p.A, p.T, 0.5),
      m3 = config.legMass + config.toolMass;
    const G3 = {
      x: (legCenter.x * config.legMass + p.T.x * config.toolMass) / m3,
      y: (legCenter.y * config.legMass + p.T.y * config.toolMass) / m3,
    };
    return {
      G2: center(p.O2, p.A, 0.5),
      G3,
      G4: center(p.O4, p.B, 0.5),
      legCenter,
      T: p.T,
      m3,
      t3: angle(p.A, p.B),
      t4: angle(p.O4, p.B),
    };
  };
  for (const q of [175, 200, 230].map((degrees) => (degrees * Math.PI) / 180)) {
    const now = state(q),
      plus = state(q + h),
      minus = state(q - h);
    const first = (key) => scale(sub(plus[key], minus[key]), 1 / (2 * h));
    const second = (key) =>
      scale(
        {
          x: plus[key].x - 2 * now[key].x + minus[key].x,
          y: plus[key].y - 2 * now[key].y + minus[key].y,
        },
        1 / (h * h),
      );
    const theta3q = signed(plus.t3, minus.t3) / (2 * h),
      theta4q = signed(plus.t4, minus.t4) / (2 * h);
    const theta3qq =
        (signed(plus.t3, now.t3) - signed(now.t3, minus.t3)) / (h * h),
      theta4qq = (signed(plus.t4, now.t4) - signed(now.t4, minus.t4)) / (h * h);
    const acceleration = (key) => ({
      x: second(key).x * omega * omega + first(key).x * alpha,
      y: second(key).y * omega * omega + first(key).y * alpha,
    });
    const I2 = (config.crankMass * config.crank ** 2) / 12,
      I3 =
        (config.legMass * config.toolAlong ** 2) / 12 +
        config.legMass *
          dot(sub(now.legCenter, now.G3), sub(now.legCenter, now.G3)) +
        config.toolMass * dot(sub(now.T, now.G3), sub(now.T, now.G3)),
      I4 = (config.rockerMass * config.rocker ** 2) / 12;
    const virtualTorque =
      (config.crankMass * dot(acceleration("G2"), first("G2")) +
        now.m3 * dot(acceleration("G3"), first("G3")) +
        config.rockerMass * dot(acceleration("G4"), first("G4")) +
        I2 * alpha +
        I3 * (theta3qq * omega * omega + theta3q * alpha) * theta3q +
        I4 * (theta4qq * omega * omega + theta4q * alpha) * theta4q) /
      1e6;
    const solved = inverseDynamics(
      config,
      (q * 180) / Math.PI,
      { omega, alpha },
      { x: 0, y: 0 },
    );
    assert.ok(solved);
    assert.ok(
      Math.abs(solved.torque - virtualTorque) < 2e-8,
      `q=${q}: expected ${virtualTorque}, got ${solved.torque}`,
    );
  }
});
