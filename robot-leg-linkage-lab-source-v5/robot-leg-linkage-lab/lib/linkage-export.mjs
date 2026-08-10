import { magnitude } from "./linkage-geometry.mjs";
import { staticSupportAtAngle } from "./linkage-static-support.mjs";

const close = (a, b, tolerance) => Math.abs(a - b) <= tolerance;

function sampleMeta(profile, sample, index, samples) {
  const previous = samples[index - 1];
  const next = samples[index + 1];
  const dt = next ? next.time - sample.time : 0;
  const direction = sample.omega > 1e-10 ? "increasing" : sample.omega < -1e-10 ? "decreasing" : "boundary";
  let segment = direction === "decreasing" ? "return" : "outbound";
  let refinement = "base_grid";
  if (profile.type === "s-curve" && profile.plan) {
    const boundaries = [...profile.plan.boundaries, ...profile.plan.boundaries.map((t) => profile.halfDuration + t)];
    const tolerance = Math.max(1e-10, profile.duration * 1e-9);
    const boundaryIndex = boundaries.findIndex((t) => close(sample.time, t, tolerance));
    if (boundaryIndex >= 0) refinement = "exact_profile_boundary";
    const halfOffset = sample.time <= profile.halfDuration ? 0 : profile.plan.boundaries.length - 1;
    const local = sample.time <= profile.halfDuration ? sample.time : sample.time - profile.halfDuration;
    let localSegment = profile.plan.boundaries.findIndex((t, i) => i < profile.plan.boundaries.length - 1 && local >= t - tolerance && local <= profile.plan.boundaries[i + 1] + tolerance);
    if (localSegment < 0) localSegment = profile.plan.boundaries.length - 2;
    segment = `s${halfOffset + localSegment + 1}`;
  }
  return { dt, direction, segment, refinement };
}
export function analysisToCsv(config, motionAnalysis) {
  const header = [
    "sample_index", "time_s", "dt_s", "phase", "motion_direction", "segment_id",
    "solver_status", "refinement_status", "angle_deg", "omega_rad_s", "alpha_rad_s2", "jerk_rad_s3",
    "reachable", "tool_x_mm", "tool_y_mm", "transmission_deg", "external_load_N",
    "loaded_input_torque_Nm", "linkage_self_dynamic_torque_Nm", "external_load_torque_Nm",
    "gravity_torque_Nm", "inertia_torque_Nm", "loaded_input_power_W", "linkage_self_power_W",
    "loaded_O2_reaction_Fx_N", "loaded_O2_reaction_Fy_N", "loaded_O2_reaction_N",
    "loaded_A_reaction_Fx_N", "loaded_A_reaction_Fy_N", "loaded_A_reaction_N",
    "loaded_B_reaction_Fx_N", "loaded_B_reaction_Fy_N", "loaded_B_reaction_N",
    "loaded_O4_reaction_Fx_N", "loaded_O4_reaction_Fy_N", "loaded_O4_reaction_N",
    "vertical_effective_arm_mm", "vertical_support_N_per_input_Nm",
    "normalized_vertical_mechanical_advantage", "static_hold_torque_Nm",
  ];
  const fmt = (value, digits = 6) => Number.isFinite(value) ? Number(value).toFixed(digits) : "";
  const reaction = (dynamics, key) => dynamics?.[key] ?? null;
  const rows = motionAnalysis.samples.map((sample, index, samples) => {
    const meta = sampleMeta(motionAnalysis.profile, sample, index, samples);
    const support = sample.kinematicState ? staticSupportAtAngle(config, sample.angle, config.supportForce) : null;
    const solved = Boolean(sample.pose && sample.dynamics);
    const dyn = sample.dynamics;
    const self = sample.selfDynamics;
    const ext = sample.externalDynamics;
    const O2 = reaction(dyn, "O2Reaction"), A = reaction(dyn, "AReaction");
    const B = reaction(dyn, "BReaction"), O4 = reaction(dyn, "O4Reaction");
    return [
      index, fmt(sample.time, 8), fmt(meta.dt, 8), fmt(sample.phase, 8), meta.direction, meta.segment,
      solved ? "solved" : sample.pose ? "dynamics_indeterminate" : "unreachable", meta.refinement,
      fmt(sample.angle, 8), fmt(sample.omega, 8), fmt(sample.alpha, 8), fmt(sample.jerk, 8),
      solved, fmt(sample.pose?.T.x), fmt(sample.pose?.T.y), fmt(sample.pose?.transmission), fmt(config.supportForce),
      fmt(dyn?.torque), fmt(self?.torque), fmt(ext?.torque), fmt(sample.gravityTorque), fmt(sample.inertiaTorque),
      fmt(sample.loadedPowerW), fmt(sample.linkPowerW),
      fmt(O2?.x), fmt(O2?.y), fmt(O2 ? magnitude(O2) : null),
      fmt(A?.x), fmt(A?.y), fmt(A ? magnitude(A) : null),
      fmt(B?.x), fmt(B?.y), fmt(B ? magnitude(B) : null),
      fmt(O4?.x), fmt(O4?.y), fmt(O4 ? magnitude(O4) : null),
      fmt(support?.effectiveMomentArmMm),
      support && Number.isFinite(support.verticalSupportPerInputTorque) ? fmt(support.verticalSupportPerInputTorque) : "inf",
      support && Number.isFinite(support.normalizedMechanicalAdvantage) ? fmt(support.normalizedMechanicalAdvantage) : "inf",
      fmt(support?.holdingTorque),
    ];
  });
  return [header, ...rows].map((row) => row.join(",")).join("\n");
}
