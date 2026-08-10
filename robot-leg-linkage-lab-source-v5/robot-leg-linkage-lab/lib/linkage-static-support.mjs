import { RAD, motionWindowReachability, sampleAngleRange, solvePose, mul, sub } from "./linkage-geometry.mjs";
import { kinematicStateAtAngle } from "./linkage-kinematics.mjs";
import { inverseDynamics } from "./linkage-dynamics.mjs";

function finitePeak(samples, metric, mode = "max") {
  const candidates = samples
    .map((sample) => ({ sample, value: metric(sample) }))
    .filter((item) => Number.isFinite(item.value));
  if (!candidates.length) return { value: null, time: null, angle: null, resolution: null };
  let best = candidates[0];
  for (const candidate of candidates.slice(1)) {
    if ((mode === "max" && candidate.value > best.value) || (mode === "min" && candidate.value < best.value)) best = candidate;
  }
  const index = samples.indexOf(best.sample);
  const before = samples[index - 1];
  const after = samples[index + 1];
  const resolution = before && after ? Math.max(best.sample.angle - before.angle, after.angle - best.sample.angle) : null;
  return { value: best.value, time: null, angle: best.sample.angle, resolution };
}

export function staticInputTorqueFromJacobian(config, angleDegrees, force = { x: 0, y: 0 }) {
  const hDegrees = 1e-3;
  const before = solvePose(config, angleDegrees - hDegrees);
  const after = solvePose(config, angleDegrees + hDegrees);
  if (!before || !after) return null;
  const dTheta = 2 * hDegrees * RAD;
  const derivative = mul(sub(after.T, before.T), 1 / dTheta);
  return -(force.x * derivative.x + force.y * derivative.y) / 1000;
}

export function staticSupportFromKinematicState(config, state, supportForce = config.supportForce ?? 0) {
  if (!state) return null;
  const signedVerticalArmMm = -state.kinematics.toolDerivative.y;
  const effectiveMomentArmMm = Math.abs(signedVerticalArmMm);
  const singularVertical = effectiveMomentArmMm < 1e-6;
  const verticalSupportPerInputTorque = singularVertical ? Infinity : 1000 / effectiveMomentArmMm;
  const normalizedMechanicalAdvantage = singularVertical ? Infinity : config.crank / effectiveMomentArmMm;
  const supportTorque = staticInputTorqueFromJacobian(config, state.angle, { x: 0, y: supportForce });
  const gravityTorque = inverseDynamics(config, state, { omega: 0, alpha: 0 }, { x: 0, y: 0 })?.torque ?? null;
  const holdingTorque = inverseDynamics(config, state, { omega: 0, alpha: 0 }, { x: 0, y: supportForce })?.torque ?? null;
  return {
    pose: state.pose,
    kinematicState: state,
    effectiveMomentArmMm,
    signedVerticalArmMm,
    verticalSupportPerInputTorque,
    normalizedMechanicalAdvantage,
    singularVertical,
    supportForce,
    supportTorque,
    gravityTorque,
    holdingTorque,
  };
}

export function staticSupportAtAngle(config, angleDegrees, supportForce = config.supportForce ?? 0) {
  return staticSupportFromKinematicState(config, kinematicStateAtAngle(config, angleDegrees), supportForce);
}

export function analyzeStaticSupport(config, options = {}) {
  const reachability = motionWindowReachability(config);
  const stepDeg = Math.max(0.05, options.stepDeg ?? 0.25);
  const angles = sampleAngleRange(config.minAngle, config.maxAngle, stepDeg);
  const samples = angles.map((angle) => {
    const kinematicState = kinematicStateAtAngle(config, angle);
    return {
      angle,
      pose: kinematicState?.pose ?? solvePose(config, angle),
      kinematicState,
      support: kinematicState ? staticSupportFromKinematicState(config, kinematicState, config.supportForce) : null,
    };
  });
  const status = !reachability.fullyReachable || samples.some((sample) => !sample.kinematicState || !sample.support)
    ? "invalid motion path"
    : "valid";
  return {
    status,
    reachability,
    stepDeg,
    samples,
    peaks: {
      minTransmission: finitePeak(samples, (sample) => sample.pose?.transmission ?? NaN, "min"),
      peakHoldingTorque: finitePeak(samples, (sample) => sample.support?.holdingTorque == null ? NaN : Math.abs(sample.support.holdingTorque)),
      peakGravityTorque: finitePeak(samples, (sample) => sample.support?.gravityTorque == null ? NaN : Math.abs(sample.support.gravityTorque)),
      maxMechanicalAdvantage: finitePeak(samples, (sample) => Number.isFinite(sample.support?.normalizedMechanicalAdvantage) ? sample.support.normalizedMechanicalAdvantage : NaN),
    },
  };
}
