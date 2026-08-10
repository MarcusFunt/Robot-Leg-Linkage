import { magnitude } from "./linkage-geometry.mjs";
import { staticSupportAtAngle } from "./linkage-static-support.mjs";

export function analysisToCsv(config, motionAnalysis) {
  const header = [
    "time_s",
    "phase",
    "angle_deg",
    "omega_rad_s",
    "alpha_rad_s2",
    "jerk_rad_s3",
    "reachable",
    "tool_x_mm",
    "tool_y_mm",
    "transmission_deg",
    "dynamic_input_torque_Nm",
    "gravity_torque_Nm",
    "inertia_torque_Nm",
    "link_power_W",
    "O2_reaction_N",
    "A_reaction_N",
    "B_reaction_N",
    "O4_reaction_N",
    "vertical_effective_arm_mm",
    "vertical_support_N_per_input_Nm",
    "normalized_vertical_mechanical_advantage",
    "static_hold_torque_Nm",
  ];
  const rows = motionAnalysis.samples.map((sample) => {
    const support = sample.kinematicState
      ? staticSupportAtAngle(config, sample.angle, config.supportForce)
      : null;
    if (!sample.pose || !sample.dynamics) {
      return [sample.time, sample.phase, sample.angle, sample.omega, sample.alpha, sample.jerk, false, "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""];
    }
    return [
      sample.time.toFixed(8),
      sample.phase.toFixed(8),
      sample.angle.toFixed(8),
      sample.omega.toFixed(8),
      sample.alpha.toFixed(8),
      sample.jerk.toFixed(8),
      true,
      sample.pose.T.x.toFixed(6),
      sample.pose.T.y.toFixed(6),
      sample.pose.transmission.toFixed(6),
      sample.dynamics.torque.toFixed(6),
      sample.gravityTorque?.toFixed(6) ?? "",
      sample.inertiaTorque?.toFixed(6) ?? "",
      sample.linkPowerW?.toFixed(6) ?? "",
      magnitude(sample.dynamics.O2Reaction).toFixed(6),
      magnitude(sample.dynamics.AReaction).toFixed(6),
      magnitude(sample.dynamics.BReaction).toFixed(6),
      magnitude(sample.dynamics.O4Reaction).toFixed(6),
      support?.effectiveMomentArmMm.toFixed(6) ?? "",
      support && Number.isFinite(support.verticalSupportPerInputTorque) ? support.verticalSupportPerInputTorque.toFixed(6) : "inf",
      support && Number.isFinite(support.normalizedMechanicalAdvantage) ? support.normalizedMechanicalAdvantage.toFixed(6) : "inf",
      support?.holdingTorque?.toFixed(6) ?? "",
    ];
  });
  return [header, ...rows].map((row) => row.join(",")).join("\n");
}
