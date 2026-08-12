import test from "node:test";
import assert from "node:assert/strict";
import {
  analysisToCsv, analyzeMotion, analyzeStaticSupport, dynamicsBreakdown,
  staticSupportAtAngle, summarizeAnalysis,
} from "../lib/linkage-analysis.mjs";

const config = {
  groundX: 45, groundY: -40, crank: 40, coupler: 45, rocker: 60,
  toolAlong: 120, toolOffset: 0, minAngle: 165, maxAngle: 225,
  motionProfile: "s-curve", maxVelocity: 360, maxAcceleration: 1500,
  maxJerk: 10000, cycleTime: 2, branch: -1, supportForce: 100,
  gravity: true, crankMass: 0.12, legMass: 0.28, rockerMass: 0.15,
  toolMass: 0.08, pinDiameter: 8, linkThickness: 4, shearPlanes: 2,
  allowableShear: 120, allowableBearing: 80, gearRatio: 10,
  gearEfficiency: 90, motorContinuous: 0.6, motorPeak: 1.2,
};

function close(actual, expected, tolerance = 1e-8) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `expected ${expected}, got ${actual}`);
}
test("motion analysis propagates configured external load through torque and reactions", () => {
  const motion = analyzeMotion(config, { sampleCount: 481 });
  assert.equal(motion.status, "valid");
  const sample = motion.samples.reduce((best, item) =>
    Math.abs(item.angle - 200) < Math.abs(best.angle - 200) ? item : best,
  );
  assert.ok(sample.dynamics && sample.selfDynamics && sample.externalDynamics);
  close(sample.dynamics.torque,
    sample.selfDynamics.torque + sample.externalDynamics.torque, 5e-7);
  const support = staticSupportAtAngle(config, sample.angle, config.supportForce);
  assert.ok(support && sample.inertiaTorque !== null);
  close(sample.dynamics.torque, support.holdingTorque + sample.inertiaTorque, 5e-7);
  assert.ok(Math.hypot(sample.dynamics.O4Reaction.x, sample.dynamics.O4Reaction.y) > 50);
});

test("actuator summary sizes from loaded trajectory with time-weighted RMS", () => {
  const motion = analyzeMotion(config, { sampleCount: 481 });
  const statics = analyzeStaticSupport(config, { stepDeg: 0.5 });
  const sample = motion.samples[Math.floor(motion.samples.length / 4)];
  const current = dynamicsBreakdown(config, sample.kinematicState, sample,
    { x: 0, y: config.supportForce });
  const support = staticSupportAtAngle(config, sample.angle, config.supportForce);
  const summary = summarizeAnalysis(config, motion, statics, current, support);
  assert.ok(summary.peakTorque > 4);
  assert.ok(summary.rmsMotorTorque > 0.3);
  close(summary.peakTorque, motion.peaks.peakTorque.value, 1e-10);
});
test("CSV names loaded and self-only quantities explicitly and has unique exported times", () => {
  const motion = analyzeMotion(config, { sampleCount: 481 });
  const csv = analysisToCsv(config, motion);
  const [header, ...rows] = csv.trim().split("\n").map((line) => line.split(","));
  for (const field of [
    "external_load_N", "loaded_input_torque_Nm", "linkage_self_dynamic_torque_Nm",
    "loaded_input_power_W", "loaded_O2_reaction_Fx_N", "loaded_O2_reaction_Fy_N",
    "loaded_O2_reaction_N", "motion_direction", "segment_id", "dt_s",
    "solver_status", "refinement_status",
  ]) assert.ok(header.includes(field), `missing ${field}`);
  assert.ok(!header.includes("dynamic_input_torque_Nm"));
  const timeIndex = header.indexOf("time_s");
  const times = rows.map((row) => row[timeIndex]);
  assert.equal(new Set(times).size, times.length);
});

test("external-load torque contribution vanishes when configured load is zero", () => {
  const zero = analyzeMotion({ ...config, supportForce: 0 }, { sampleCount: 121 });
  for (const sample of zero.samples) {
    if (!sample.dynamics || !sample.selfDynamics || !sample.externalDynamics) continue;
    close(sample.externalDynamics.torque, 0, 1e-9);
    close(sample.dynamics.torque, sample.selfDynamics.torque, 5e-8);
  }
});
test("worst-case loaded and static peaks carry local refinement provenance", () => {
  const motion = analyzeMotion(config, { sampleCount: 121 });
  const statics = analyzeStaticSupport(config, { stepDeg: 2 });
  assert.equal(motion.peaks.peakTorque.refinement, "local_golden_section");
  assert.equal(motion.peaks.peakLoadedPower.refinement, "local_golden_section");
  assert.ok(motion.peaks.peakTorque.resolution > 0 && motion.peaks.peakTorque.resolution < 1e-4);
  assert.equal(statics.peaks.peakHoldingTorque.refinement, "local_golden_section");
  assert.ok(statics.peaks.peakHoldingTorque.resolution > 0 && statics.peaks.peakHoldingTorque.resolution < 1e-3);
});
