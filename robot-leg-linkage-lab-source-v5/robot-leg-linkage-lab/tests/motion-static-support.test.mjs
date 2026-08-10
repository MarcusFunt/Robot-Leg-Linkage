import test from "node:test";
import assert from "node:assert/strict";
import {
  DEG, analyzeMotion, analyzeStaticSupport, motionProfileInfo, motionStateAtTime,
  staticInputTorqueFromJacobian, staticSupportAtAngle,
} from "../lib/linkage-analysis.mjs";

const base = {
  groundX: 45, groundY: -40, crank: 40, coupler: 45, rocker: 60, toolAlong: 120, toolOffset: 0,
  minAngle: 0, maxAngle: 360, motionProfile: "s-curve", maxVelocity: 360, maxAcceleration: 1500, maxJerk: 10000, cycleTime: 2,
  branch: -1, supportForce: 100, gravity: true, crankMass: 0.12, legMass: 0.28, rockerMass: 0.15, toolMass: 0.08,
  pinDiameter: 8, linkThickness: 4, shearPlanes: 2, allowableShear: 120, allowableBearing: 80,
  gearRatio: 10, gearEfficiency: 90, motorContinuous: 0.6, motorPeak: 1.2,
};

function close(actual, expected, tolerance) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `expected ${expected}, got ${actual}`);
}

test("sinusoid reaches min/max at the specified cycle phases", () => {
  const config = { ...base, motionProfile: "sinusoidal", minAngle: 20, maxAngle: 100, cycleTime: 2.5 };
  const info = motionProfileInfo(config);
  close(motionStateAtTime(config, 0).angle, 20, 1e-10);
  close(motionStateAtTime(config, info.duration / 2).angle, 100, 1e-10);
  close(motionStateAtTime(config, info.duration).angle, 20, 1e-10);
});

test("static vertical support leverage is consistent with virtual work", () => {
  const result = staticSupportAtAngle(base, 180, 100);
  assert.ok(result);
  close(result.supportTorque, staticInputTorqueFromJacobian(base, 180, { x: 0, y: 100 }), 1e-8);
  close(result.verticalSupportPerInputTorque, 1000 / result.effectiveMomentArmMm, 1e-10);
  close(result.normalizedMechanicalAdvantage, base.crank / result.effectiveMomentArmMm, 1e-10);
});

test("dynamic and static analyses explicitly reject an unreachable requested window", () => {
  const config = { ...base, groundX: 100, groundY: 0, crank: 10, coupler: 20, rocker: 20, minAngle: 0, maxAngle: 40 };
  assert.equal(analyzeMotion(config).status, "invalid motion path");
  assert.equal(analyzeStaticSupport(config).status, "invalid motion path");
});

test("S-curve reported peaks stay inside configured limits", () => {
  const info = motionProfileInfo(base);
  assert.ok(info.peakVelocityDegS <= base.maxVelocity + 1e-8);
  assert.ok(info.peakAccelerationDegS2 <= base.maxAcceleration + 1e-8);
  assert.ok(info.peakJerkDegS3 <= base.maxJerk + 1e-8);
  assert.ok(info.duration > 0);
});
