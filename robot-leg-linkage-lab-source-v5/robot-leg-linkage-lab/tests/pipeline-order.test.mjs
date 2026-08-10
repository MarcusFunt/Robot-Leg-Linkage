import test from "node:test";
import assert from "node:assert/strict";
import {
  analyzeMotion,
  analyzeStaticSupport,
  dynamicsBreakdown,
  kinematicStateAtAngle,
  motionStateAtTime,
} from "../lib/linkage-analysis.mjs";

const config = {
  groundX: 45, groundY: -40, crank: 40, coupler: 45, rocker: 60, toolAlong: 120, toolOffset: 0,
  minAngle: 0, maxAngle: 360, motionProfile: "s-curve", maxVelocity: 360, maxAcceleration: 1500, maxJerk: 10000, cycleTime: 2,
  branch: -1, supportForce: 100, gravity: true, crankMass: 0.12, legMass: 0.28, rockerMass: 0.15, toolMass: 0.08,
  pinDiameter: 8, linkThickness: 4, shearPlanes: 2, allowableShear: 120, allowableBearing: 80,
  gearRatio: 10, gearEfficiency: 90, motorContinuous: 0.6, motorPeak: 1.2,
};

function close(a, b, tolerance = 1e-10) {
  assert.ok(Math.abs(a - b) <= tolerance, `expected ${b}, got ${a}`);
}

test("inverse dynamics accepts the reusable kinematic state without changing results", () => {
  const angle = 180;
  const state = kinematicStateAtAngle(config, angle);
  assert.ok(state);
  const trajectory = { omega: 4.5, alpha: -12 };
  const fromAngle = dynamicsBreakdown(config, angle, trajectory, { x: 0, y: 0 });
  const fromState = dynamicsBreakdown(config, state, trajectory, { x: 0, y: 0 });
  assert.ok(fromAngle.total && fromState.total);
  close(fromState.total.torque, fromAngle.total.torque);
  close(fromState.total.AReaction.x, fromAngle.total.AReaction.x);
  close(fromState.total.BReaction.y, fromAngle.total.BReaction.y);
});

test("motion branch materializes trajectory then kinematic state", () => {
  const motion = analyzeMotion(config, { sampleCount: 101 });
  assert.equal(motion.status, "valid");
  for (const sample of motion.samples) {
    assert.ok(sample.kinematicState);
    close(sample.kinematicState.angle, sample.angle, 1e-9);
    assert.equal(sample.kinematicState.pose, sample.pose);
  }
  const trajectory = motionStateAtTime(config, motion.profile.duration * 0.25);
  assert.ok(Number.isFinite(trajectory.omega) && Number.isFinite(trajectory.alpha));
});

test("static branch is angle/geometry driven and carries its own kinematic state", () => {
  const analysis = analyzeStaticSupport(config, { stepDeg: 5 });
  assert.equal(analysis.status, "valid");
  for (const sample of analysis.samples) {
    assert.ok(sample.kinematicState && sample.support);
    close(sample.kinematicState.angle, sample.angle, 1e-9);
    assert.equal(sample.support.kinematicState, sample.kinematicState);
  }
});
