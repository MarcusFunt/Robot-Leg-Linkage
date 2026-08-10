import test from "node:test";
import assert from "node:assert/strict";
import {
  adaptiveAnalyze, analysisToCsv, dynamicsBreakdown, inverseDynamics, mechanismClass, sampleAngleRange,
  solvePose, staticInputTorqueFromJacobian, summarizeAnalysis,
} from "../lib/linkage-analysis.mjs";
import { validateConfig, validateNumericValue } from "../lib/input-validation.mjs";

const config = {
  groundX: 45, groundY: -40, crank: 40, coupler: 45, rocker: 60, toolAlong: 120, toolOffset: 0,
  angle: 180, minAngle: 0, maxAngle: 360, rpm: 60, inputAccel: 0, branch: -1,
  forceX: 0, forceY: 100, gravity: true, crankMass: 0.12, legMass: 0.28, rockerMass: 0.15, toolMass: 0.08,
  pinDiameter: 8, linkThickness: 4, shearPlanes: 2, allowableShear: 120, allowableBearing: 80,
  gearRatio: 10, gearEfficiency: 90, motorContinuous: 0.6, motorPeak: 1.2,
};

function close(actual, expected, tolerance, message = "") {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${message} expected ${expected}, got ${actual}`);
}

test("four-bar pose preserves link lengths on both assembly branches", () => {
  for (const branch of [-1, 1]) {
    const pose = solvePose({ ...config, branch }, 180);
    assert.ok(pose);
    close(Math.hypot(pose.B.x - pose.A.x, pose.B.y - pose.A.y), config.coupler, 1e-8, "coupler");
    close(Math.hypot(pose.B.x - pose.O4.x, pose.B.y - pose.O4.y), config.rocker, 1e-8, "rocker");
  }
  const lower = solvePose({ ...config, branch: -1 }, 180);
  const upper = solvePose({ ...config, branch: 1 }, 180);
  assert.ok(lower && upper);
  assert.notEqual(Math.sign(lower.B.y - upper.B.y), 0);
});

test("unreachable geometry returns null rather than stale coordinates", () => {
  assert.equal(solvePose({ ...config, coupler: 5, rocker: 5 }, 0), null);
});

test("Grashof classifier distinguishes rotating and non-Grashof cases", () => {
  const rotating = mechanismClass({ ...config, groundX: 30, groundY: 0, crank: 10, coupler: 30, rocker: 30 });
  assert.equal(rotating.grashof, true);
  assert.equal(rotating.inputRotates, true);
  const non = mechanismClass({ ...config, groundX: 40, groundY: 0, crank: 30, coupler: 20, rocker: 15 });
  assert.equal(non.grashof, false);
});

test("static inverse dynamics matches the tool-force virtual-work moment arm", () => {
  const staticConfig = { ...config, gravity: false, rpm: 0, inputAccel: 0, crankMass: 0, legMass: 0, rockerMass: 0, toolMass: 0, forceX: 37, forceY: 83 };
  const torque = inverseDynamics(staticConfig, 180)?.torque;
  const jacobianTorque = staticInputTorqueFromJacobian(staticConfig, 180);
  assert.ok(torque !== undefined && jacobianTorque !== null);
  close(torque, jacobianTorque, 2e-4);
});

test("gravity-only breakdown equals total when no external load or inertia is prescribed", () => {
  const gravityConfig = { ...config, forceX: 0, forceY: 0, rpm: 0, inputAccel: 0 };
  const breakdown = dynamicsBreakdown(gravityConfig, 180);
  assert.ok(breakdown.total && breakdown.gravity);
  close(breakdown.total.torque, breakdown.gravity.torque, 1e-10);
});

test("adaptive peak and minimum transmission converge against a dense reference", () => {
  const adaptive = adaptiveAnalyze(config, { coarseStepDeg: 12, angleToleranceDeg: 0.04, maxDepth: 12 });
  const denseAngles = sampleAngleRange(config.minAngle, config.maxAngle, 0.1);
  const dense = denseAngles.map((angle) => ({ angle, pose: solvePose(config, angle), dynamics: inverseDynamics(config, angle) }));
  const valid = dense.filter((sample) => sample.pose && sample.dynamics);
  const densePeak = Math.max(...valid.map((sample) => Math.abs(sample.dynamics.torque)));
  const denseMinTransmission = Math.min(...valid.map((sample) => sample.pose.transmission));
  close(adaptive.peaks.peakTorque.value, densePeak, Math.max(0.01, densePeak * 0.01));
  close(adaptive.peaks.minTransmission.value, denseMinTransmission, 0.15);
  assert.ok(adaptive.peaks.peakTorque.angle !== null);
  assert.ok(["converged", "not converged", "indeterminate near toggle"].includes(adaptive.peaks.peakTorque.convergence));
});

test("RMS continuous sizing is separate from peak rating sizing", () => {
  const analysis = adaptiveAnalyze(config);
  const summary = summarizeAnalysis(config, analysis, dynamicsBreakdown(config, config.angle));
  assert.ok(summary.rmsMotorTorque !== null);
  assert.ok(summary.peakMotorTorque >= summary.rmsMotorTorque);
  close(summary.continuousUse, summary.rmsMotorTorque / config.motorContinuous * 100, 1e-10);
  close(summary.peakUse, summary.peakMotorTorque / config.motorPeak * 100, 1e-10);
  close(summary.peakMotorSpeedRpm, config.rpm * config.gearRatio, 1e-12);
  assert.ok(summary.peakMechanicalPowerW >= 0);
});

test("unresolved reachability boundary is reported as indeterminate near toggle and suppresses safety factors", () => {
  const toggleConfig = { ...config, groundX: 40, groundY: 0, crank: 30, coupler: 20, rocker: 15, toolAlong: 50, gravity: false, crankMass: 0, legMass: 0, rockerMass: 0, toolMass: 0 };
  const analysis = adaptiveAnalyze(toggleConfig);
  assert.equal(analysis.convergence.status, "indeterminate near toggle");
  const summary = summarizeAnalysis(toggleConfig, analysis, dynamicsBreakdown(toggleConfig, toggleConfig.angle));
  assert.equal(summary.shearSafety, null);
  assert.equal(summary.bearingSafety, null);
});

test("numeric validation rejects non-finite values, invalid efficiency, fractional shear planes, and inverted motor ratings", () => {
  assert.match(validateNumericValue(Number.NaN), /finite/);
  assert.match(validateNumericValue(101, { max: 100 }), /≤ 100/);
  const errors = validateConfig({ ...config, gearEfficiency: 101, shearPlanes: 1.5, motorContinuous: 2, motorPeak: 1 });
  assert.ok(errors.gearEfficiency);
  assert.ok(errors.shearPlanes);
  assert.ok(errors.motorPeak);
});

test("CSV export declares engineering units explicitly", () => {
  const csv = analysisToCsv(adaptiveAnalyze({ ...config, minAngle: 170, maxAngle: 190 }));
  const header = csv.split("\n", 1)[0];
  for (const column of ["angle_deg", "tool_x_mm", "transmission_deg", "link_torque_Nm", "O2_reaction_N"]) assert.ok(header.includes(column));
});
