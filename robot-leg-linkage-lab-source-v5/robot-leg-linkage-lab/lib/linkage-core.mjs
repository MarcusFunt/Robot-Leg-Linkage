// Compatibility surface for callers that previously imported from linkage-core.
// New code should depend on the specific pipeline layer instead.
export * from "./linkage-geometry.mjs";
export * from "./linkage-kinematics.mjs";
export * from "./linkage-dynamics.mjs";
export {
  staticInputTorqueFromJacobian,
  staticSupportAtAngle,
  staticSupportFromKinematicState,
} from "./linkage-static-support.mjs";
