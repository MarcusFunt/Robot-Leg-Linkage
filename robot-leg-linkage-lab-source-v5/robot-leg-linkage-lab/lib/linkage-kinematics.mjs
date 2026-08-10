import { DEG, mul, signedAngleDifference, solvePose, sub } from "./linkage-geometry.mjs";

export function kinematics(config, angleDegrees = config.minAngle ?? 0, pose = solvePose(config, angleDegrees)) {
  if (!pose) return null;
  const h = 0.001;
  const minus = solvePose(config, angleDegrees - h * DEG);
  const plus = solvePose(config, angleDegrees + h * DEG);
  if (!minus || !plus) return null;
  return {
    omega3Ratio: signedAngleDifference(plus.theta3, minus.theta3) / (2 * h),
    omega4Ratio: signedAngleDifference(plus.theta4, minus.theta4) / (2 * h),
    toolDerivative: mul(sub(plus.T, minus.T), 1 / (2 * h)),
  };
}

export function kinematicStateAtAngle(config, angleDegrees = config.minAngle ?? 0) {
  const pose = solvePose(config, angleDegrees);
  if (!pose) return null;
  const derivatives = kinematics(config, angleDegrees, pose);
  if (!derivatives) return null;
  return { angle: angleDegrees, pose, kinematics: derivatives };
}
