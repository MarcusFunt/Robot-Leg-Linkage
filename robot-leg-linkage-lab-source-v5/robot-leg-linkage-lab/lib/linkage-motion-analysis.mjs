import { magnitude, motionWindowReachability } from "./linkage-geometry.mjs";
import { kinematicStateAtAngle } from "./linkage-kinematics.mjs";
import { dynamicsBreakdown } from "./linkage-dynamics.mjs";
import { motionProfileInfo, motionStateAtTime, sampleMotionProfile } from "./motion-profile.mjs";

function evaluateTrajectory(config, trajectory) {
  const externalLoad = { x: 0, y: config.supportForce ?? 0 };
  const kinematicState = kinematicStateAtAngle(config, trajectory.angle);
  if (!kinematicState) return {
    ...trajectory, pose: null, kinematicState: null, dynamics: null, selfDynamics: null,
    externalDynamics: null, externalLoad, gravityTorque: null, inertiaTorque: null,
    jointReaction: null, linkPowerW: null, loadedPowerW: null,
  };
  const breakdown = dynamicsBreakdown(config, kinematicState, trajectory, externalLoad);
  const selfBreakdown = dynamicsBreakdown(config, kinematicState, trajectory, { x: 0, y: 0 });
  const dynamics = breakdown.total;
  const selfDynamics = selfBreakdown.total;
  const jointReaction = dynamics ? Math.max(
    magnitude(dynamics.O2Reaction), magnitude(dynamics.AReaction),
    magnitude(dynamics.BReaction), magnitude(dynamics.O4Reaction),
  ) : null;
  return {
    ...trajectory, pose: kinematicState.pose, kinematicState, dynamics, selfDynamics,
    externalDynamics: breakdown.external, externalLoad,
    gravityTorque: breakdown.gravity?.torque ?? null, inertiaTorque: breakdown.inertia?.torque ?? null,
    jointReaction, linkPowerW: selfDynamics ? selfDynamics.torque * trajectory.omega : null,
    loadedPowerW: dynamics ? dynamics.torque * trajectory.omega : null,
  };
}
function finitePeak(samples, metric, mode = "max") {
  const candidates = samples.map((sample) => ({ sample, value: metric(sample) }))
    .filter((item) => Number.isFinite(item.value));
  if (!candidates.length) return { value: null, time: null, angle: null, resolution: null, refinement: "none" };
  let best = candidates[0];
  for (const candidate of candidates.slice(1)) {
    if ((mode === "max" && candidate.value > best.value) || (mode === "min" && candidate.value < best.value)) best = candidate;
  }
  const index = samples.indexOf(best.sample);
  const before = samples[index - 1], after = samples[index + 1];
  const resolution = before && after ? Math.max(best.sample.time - before.time, after.time - best.sample.time) : null;
  return { value: best.value, time: best.sample.time, angle: best.sample.angle, resolution, refinement: "sampled" };
}

function refineTimePeak(config, samples, basePeak, metric, mode = "max") {
  if (basePeak.time == null) return basePeak;
  const index = samples.findIndex((sample) => sample.time === basePeak.time);
  if (index <= 0 || index >= samples.length - 1) return { ...basePeak, refinement: "sampled_boundary" };
  let lo = samples[index - 1].time, hi = samples[index + 1].time;
  const score = (time) => {
    const sample = evaluateTrajectory(config, motionStateAtTime(config, time));
    const value = metric(sample);
    return Number.isFinite(value) ? (mode === "min" ? -value : value) : -Infinity;
  };
  const ratio = (Math.sqrt(5) - 1) / 2;
  let x1 = hi - ratio * (hi - lo), x2 = lo + ratio * (hi - lo);
  let f1 = score(x1), f2 = score(x2);
  const targetResolution = Math.max(1e-7, (samples.at(-1).time - samples[0].time) * 1e-6);
  for (let iteration = 0; iteration < 32 && hi - lo > targetResolution; iteration += 1) {
    if (f1 < f2) {
      lo = x1; x1 = x2; f1 = f2; x2 = lo + ratio * (hi - lo); f2 = score(x2);
    } else {
      hi = x2; x2 = x1; f2 = f1; x1 = hi - ratio * (hi - lo); f1 = score(x1);
    }
  }
  const candidates = [basePeak.time, x1, x2, (lo + hi) / 2].map((time) => {
    const sample = evaluateTrajectory(config, motionStateAtTime(config, time));
    return { sample, value: metric(sample) };
  }).filter((item) => Number.isFinite(item.value));
  if (!candidates.length) return basePeak;
  let best = candidates[0];
  for (const candidate of candidates.slice(1)) {
    if ((mode === "max" && candidate.value > best.value) || (mode === "min" && candidate.value < best.value)) best = candidate;
  }
  return { value: best.value, time: best.sample.time, angle: best.sample.angle, resolution: hi - lo, refinement: "local_golden_section" };
}

export function analyzeMotion(config, options = {}) {
  const profile = motionProfileInfo(config);
  const reachability = motionWindowReachability(config);
  const trajectoryStates = sampleMotionProfile(config, { sampleCount: options.sampleCount ?? 481 });
  const samples = trajectoryStates.map((trajectory) => evaluateTrajectory(config, trajectory));
  const hasUnsolvedDynamics = samples.some((sample) => sample.pose && !sample.dynamics);
  const status = !reachability.fullyReachable ? "invalid motion path"
    : hasUnsolvedDynamics ? "indeterminate near singularity" : "valid";
  const sampled = {
    peakTorque: finitePeak(samples, (sample) => sample.dynamics ? Math.abs(sample.dynamics.torque) : NaN),
    peakJointReaction: finitePeak(samples, (sample) => sample.jointReaction ?? NaN),
    peakLoadedPower: finitePeak(samples, (sample) => sample.loadedPowerW === null ? NaN : Math.abs(sample.loadedPowerW)),
    peakLinkPower: finitePeak(samples, (sample) => sample.linkPowerW === null ? NaN : Math.abs(sample.linkPowerW)),
    peakSpeed: finitePeak(samples, (sample) => Math.abs(sample.omega)),
  };
  const refine = options.refinePeaks !== false && status === "valid";
  const peaks = refine ? {
    peakTorque: refineTimePeak(config, samples, sampled.peakTorque, (sample) => sample.dynamics ? Math.abs(sample.dynamics.torque) : NaN),
    peakJointReaction: refineTimePeak(config, samples, sampled.peakJointReaction, (sample) => sample.jointReaction ?? NaN),
    peakLoadedPower: refineTimePeak(config, samples, sampled.peakLoadedPower, (sample) => sample.loadedPowerW === null ? NaN : Math.abs(sample.loadedPowerW)),
    peakLinkPower: refineTimePeak(config, samples, sampled.peakLinkPower, (sample) => sample.linkPowerW === null ? NaN : Math.abs(sample.linkPowerW)),
    peakSpeed: sampled.peakSpeed,
  } : sampled;
  return { profile, reachability, status, samples, peaks };
}
