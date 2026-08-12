import test from "node:test";
import assert from "node:assert/strict";
import { staticSupportAtAngle } from "../lib/linkage-analysis.mjs";

const config = {
  groundX: 45, groundY: -40, crank: 40, coupler: 45, rocker: 60, toolAlong: 120, toolOffset: 0,
  minAngle: 165, maxAngle: 225, motionProfile: "s-curve", maxVelocity: 360, maxAcceleration: 1500, maxJerk: 10000, cycleTime: 2,
  branch: -1, supportForce: 100, gravity: false, crankMass: 0, legMass: 0, rockerMass: 0, toolMass: 0,
  pinDiameter: 8, linkThickness: 4, shearPlanes: 2, allowableShear: 120, allowableBearing: 80,
  gearRatio: 10, gearEfficiency: 90, motorContinuous: 0.6, motorPeak: 1.2,
};

test("doubling vertical support force doubles static input torque away from singularity", () => {
  const a = staticSupportAtAngle(config, 180, 50);
  const b = staticSupportAtAngle(config, 180, 100);
  assert.ok(a && b && Math.abs(a.supportTorque) > 1e-9);
  assert.ok(Math.abs(b.supportTorque / a.supportTorque - 2) < 1e-10);
});
