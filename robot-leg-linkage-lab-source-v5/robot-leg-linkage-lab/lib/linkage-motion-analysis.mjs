import { magnitude, motionWindowReachability } from "./linkage-geometry.mjs";
import { kinematicStateAtAngle } from "./linkage-kinematics.mjs";
import { dynamicsBreakdown } from "./linkage-dynamics.mjs";
import { motionProfileInfo, sampleMotionProfile } from "./motion-profile.mjs";

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
  const resolution = before && after ? Math.max(best.sample.time - before.time, after.time - best.sample.time) : null;
  return { value: best.value, time: best.sample.time, angle: best.sample.angle, resolution };
}

export function analyzeMotion(config, options = {}) {
  const profile = motionProfileInfo(config);
  const reachability = motionWindowReachability(config);
  const trajectoryStates = sampleMotionProfile(config, { sampleCount: options.sampleCount ?? 481 });
  const samples = trajectoryStates.map((trajectory) => {
    const kinematicState = kinematicStateAtAngle(config, trajectory.angle);
    if (!kinematicState) {
      return {
        ...trajectory,
        pose: null,
        kinematicState: null,
        dynamics: null,
        gravityTorque: null,
        inertiaTorque: null,
        jointReaction: null,
        linkPowerW: null,
      };
    }

    // No external motion-load model exists yet. The dynamic branch currently solves
    // only the configured gravity plus rigid-body inertia from this trajectory.
    const breakdown = dynamicsBreakdown(config, kinematicState, trajectory, { x: 0, y: 0 });
    const dynamics = breakdown.total;
    const jointReaction = dynamics ? Math.max(
      magnitude(dynamics.O2Reaction),
      magnitude(dynamics.AReaction),
      magnitude(dynamics.BReaction),
      magnitude(dynamics.O4Reaction),
    ) : null;
    return {
      ...trajectory,
      pose: kinematicState.pose,
      kinematicState,
      dynamics,
      gravityTorque: breakdown.gravity?.torque ?? null,
      inertiaTorque: breakdown.inertia?.torque ?? null,
      jointReaction,
      linkPowerW: dynamics ? dynamics.torque * trajectory.omega : null,
    };
  });

  const hasUnsolvedDynamics = samples.some((sample) => sample.pose && !sample.dynamics);
  const status = !reachability.fullyReachable
    ? "invalid motion path"
    : hasUnsolvedDynamics
      ? "indeterminate near singularity"
      : "valid";
  return {
    profile,
    reachability,
    status,
    samples,
    peaks: {
      peakTorque: finitePeak(samples, (sample) => sample.dynamics ? Math.abs(sample.dynamics.torque) : NaN),
      peakJointReaction: finitePeak(samples, (sample) => sample.jointReaction ?? NaN),
      peakLinkPower: finitePeak(samples, (sample) => sample.linkPowerW === null ? NaN : Math.abs(sample.linkPowerW)),
      peakSpeed: finitePeak(samples, (sample) => Math.abs(sample.omega)),
    },
  };
}
