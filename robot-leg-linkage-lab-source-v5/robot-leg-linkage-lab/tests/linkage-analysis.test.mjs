import test from "node:test";
import assert from "node:assert/strict";
import {
  DEG,
  analyzeMotion,
  analyzeStaticSupport,
  dynamicsBreakdown,
  inverseDynamics,
  mechanismClass,
  motionProfileInfo,
  motionStateAtTime,
  motionWindowReachability,
  sampleMotionProfile,
  solvePose,
  staticInputTorqueFromJacobian,
  staticSupportAtAngle,
  summarizeAnalysis,
  timeAtAngle,
} from "../lib/linkage-analysis.mjs";
import {
  validateConfig,
  validateNumericValue,
} from "../lib/input-validation.mjs";
const config = {
  groundX: 45,
  groundY: -40,
  crank: 40,
  coupler: 45,
  rocker: 60,
  toolAlong: 120,
  toolOffset: 0,
  minAngle: 165,
  maxAngle: 225,
  motionProfile: "s-curve",
  maxVelocity: 360,
  maxAcceleration: 1500,
  maxJerk: 10000,
  cycleTime: 2,
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
function close(a, e, t, m = "") {
  assert.ok(Math.abs(a - e) <= t, `${m} expected ${e}, got ${a}`);
}
test("four-bar pose preserves link lengths on both assembly branches", () => {
  for (const branch of [-1, 1]) {
    const p = solvePose({ ...config, branch }, 180);
    assert.ok(p);
    close(Math.hypot(p.B.x - p.A.x, p.B.y - p.A.y), config.coupler, 1e-8);
    close(Math.hypot(p.B.x - p.O4.x, p.B.y - p.O4.y), config.rocker, 1e-8);
  }
});
test("continuous motion-window reachability catches impossible windows", () => {
  assert.equal(motionWindowReachability(config).fullyReachable, true);
  const impossible = {
    ...config,
    groundX: 100,
    groundY: 0,
    crank: 10,
    coupler: 20,
    rocker: 20,
    minAngle: 0,
    maxAngle: 40,
  };
  assert.equal(motionWindowReachability(impossible).fullyReachable, false);
  assert.equal(analyzeMotion(impossible).status, "invalid motion path");
});
test("Grashof classifier distinguishes rotating and non-Grashof cases", () => {
  assert.equal(
    mechanismClass({
      ...config,
      groundX: 30,
      groundY: 0,
      crank: 10,
      coupler: 30,
      rocker: 10,
    }).inputRotates,
    true,
  );
  assert.equal(
    mechanismClass({
      ...config,
      groundX: 40,
      groundY: 0,
      crank: 30,
      coupler: 20,
      rocker: 15,
    }).grashof,
    false,
  );
});
test("S-curve respects limits and endpoint states", () => {
  const info = motionProfileInfo(config),
    start = motionStateAtTime(config, 0),
    half = motionStateAtTime(config, info.halfDuration),
    end = motionStateAtTime(config, info.duration);
  close(start.angle, config.minAngle, 1e-8);
  close(half.angle, config.maxAngle, 1e-7);
  close(end.angle, config.minAngle, 1e-7);
  close(start.omega, 0, 1e-10);
  close(half.omega, 0, 1e-10);
  const samples = sampleMotionProfile(config, { sampleCount: 1001 });
  assert.ok(
    Math.max(...samples.map((s) => Math.abs(s.omega * DEG))) <=
      config.maxVelocity + 1e-7,
  );
  assert.ok(
    Math.max(...samples.map((s) => Math.abs(s.alpha * DEG))) <=
      config.maxAcceleration + 1e-7,
  );
  assert.ok(
    Math.max(...samples.map((s) => Math.abs(s.jerk * DEG))) <=
      config.maxJerk + 1e-7,
  );
});
test("short S-curve becomes jerk-limited", () => {
  const short = {
      ...config,
      minAngle: 100,
      maxAngle: 101,
      maxVelocity: 1000,
      maxAcceleration: 5000,
      maxJerk: 2000,
    },
    info = motionProfileInfo(short);
  assert.ok(info.peakVelocityDegS < short.maxVelocity);
  assert.ok(info.peakAccelerationDegS2 < short.maxAcceleration);
});
test("sinusoidal reciprocation timing and derivatives", () => {
  const c = {
      ...config,
      motionProfile: "sinusoidal",
      minAngle: 20,
      maxAngle: 100,
      cycleTime: 2.5,
    },
    info = motionProfileInfo(c),
    start = motionStateAtTime(c, 0),
    q = motionStateAtTime(c, info.duration / 4),
    half = motionStateAtTime(c, info.duration / 2),
    end = motionStateAtTime(c, info.duration);
  close(start.angle, 20, 1e-10);
  close(half.angle, 100, 1e-10);
  close(end.angle, 20, 1e-10);
  close(q.omega * DEG, info.peakVelocityDegS, 1e-9);
});
test("angle-to-time lookup returns outbound occurrence", () => {
  for (const angle of [165, 180, 200, 225])
    close(
      motionStateAtTime(config, timeAtAngle(config, angle)).angle,
      angle,
      1e-7,
    );
});
test("static vertical support matches virtual work", () => {
  const support = staticSupportAtAngle(config, 180, config.supportForce);
  assert.ok(support);
  close(
    support.supportTorque,
    staticInputTorqueFromJacobian(config, 180, {
      x: 0,
      y: config.supportForce,
    }),
    1e-8,
  );
  close(
    support.verticalSupportPerInputTorque,
    1000 / support.effectiveMomentArmMm,
    1e-10,
  );
  close(
    support.normalizedMechanicalAdvantage,
    config.rocker / support.effectiveMomentArmMm,
    1e-10,
  );
  close(
    support.holdingTorque,
    support.supportTorque + support.gravityTorque,
    1e-8,
  );
});
test("inverse dynamics uses explicit trajectory omega and alpha", () => {
  const still = inverseDynamics(
      config,
      180,
      { omega: 0, alpha: 0 },
      { x: 0, y: 0 },
    ),
    moving = inverseDynamics(
      config,
      180,
      { omega: 5, alpha: 20 },
      { x: 0, y: 0 },
    );
  assert.ok(still && moving);
  assert.notEqual(still.torque, moving.torque);
  const b = dynamicsBreakdown(
    config,
    180,
    { omega: 5, alpha: 20 },
    { x: 0, y: 0 },
  );
  close(b.total.torque, b.gravity.torque + b.inertia.torque, 2e-7);
});
test("time-weighted RMS sizing follows trajectory", () => {
  const motion = analyzeMotion(config),
    staticAnalysis = analyzeStaticSupport(config, { stepDeg: 1 }),
    state = motionStateAtTime(config, motion.profile.duration * 0.2),
    current = dynamicsBreakdown(config, state.angle, state, { x: 0, y: 0 }),
    support = staticSupportAtAngle(config, state.angle, config.supportForce),
    summary = summarizeAnalysis(
      config,
      motion,
      staticAnalysis,
      current,
      support,
    );
  assert.ok(
    summary.rmsMotorTorque !== null &&
      summary.peakMotorTorque >= summary.rmsMotorTorque,
  );
  close(
    summary.continuousUse,
    (summary.rmsMotorTorque / config.motorContinuous) * 100,
    1e-10,
  );
});
test("validation rejects bad profile inputs", () => {
  assert.match(validateNumericValue(Number.NaN), /finite/);
  const errors = validateConfig({
    ...config,
    groundX: 0,
    groundY: 0,
    maxJerk: 0,
    gearEfficiency: 101,
    shearPlanes: 1.5,
    motorContinuous: 2,
    motorPeak: 1,
  });
  assert.ok(
    errors.ground &&
      errors.maxJerk &&
      errors.gearEfficiency &&
      errors.shearPlanes &&
      errors.motorPeak,
  );
});
