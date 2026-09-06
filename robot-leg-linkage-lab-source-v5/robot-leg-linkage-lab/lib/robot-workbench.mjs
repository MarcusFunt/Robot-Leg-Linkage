import {
  analyzeMotion,
  analyzeStaticSupport,
  solvePose,
} from "./linkage-analysis.mjs";

export const ASCENTO_HIP_OFFSET_DEG = 415;

export function solverAngleToHipQ(theta4, offset = ASCENTO_HIP_OFFSET_DEG) {
  return offset - theta4;
}

export function hipQToSolverAngle(q, offset = ASCENTO_HIP_OFFSET_DEG) {
  return offset - q;
}

export function robotFrame(point) {
  return { x: point.x, z: -point.y };
}

export function wheelDerivative(config, angleDegrees, stepDegrees = 0.05) {
  const minus = solvePose(config, angleDegrees - stepDegrees);
  const plus = solvePose(config, angleDegrees + stepDegrees);
  if (!minus || !plus) return null;
  const scale = 1 / (2 * stepDegrees);
  return {
    dxDq: (plus.T.x - minus.T.x) * scale,
    dzDq: (-(plus.T.y) + minus.T.y) * scale,
  };
}

export function sampleWheelPath(config, stepDegrees = 1) {
  const minimum = Number(config.minAngle ?? 0);
  const maximum = Number(config.maxAngle ?? minimum);
  if (!(maximum > minimum)) return [];
  const count = Math.max(2, Math.ceil((maximum - minimum) / stepDegrees) + 1);
  const samples = [];
  for (let index = 0; index < count; index += 1) {
    const s = index / (count - 1);
    const angle = minimum + s * (maximum - minimum);
    const pose = solvePose(config, angle);
    if (!pose) continue;
    const point = robotFrame(pose.T);
    samples.push({
      s,
      angle,
      x: point.x,
      z: point.z,
      transmission: pose.transmission,
    });
  }
  return samples;
}

export function targetDiagonal(targetConfig, stepDegrees = 1) {
  const path = sampleWheelPath(targetConfig, stepDegrees);
  if (path.length < 2) return null;
  const start = path[0];
  const end = path[path.length - 1];
  return {
    start: { x: start.x, z: start.z },
    end: { x: end.x, z: end.z },
    dx: end.x - start.x,
    dz: end.z - start.z,
    length: Math.hypot(end.x - start.x, end.z - start.z),
    angleDeg: Math.atan2(end.z - start.z, end.x - start.x) * 180 / Math.PI,
  };
}

export function wheelPathMetrics(config, targetConfig = config, stepDegrees = 1) {
  const path = sampleWheelPath(config, stepDegrees);
  const target = targetDiagonal(targetConfig, stepDegrees);
  if (path.length < 2 || !target) {
    return {
      valid: false,
      samples: path,
      target,
      verticalStroke: null,
      foreAftExcursion: null,
      targetRms: null,
      straightnessRms: null,
      minTransmission: null,
    };
  }

  const xs = path.map((point) => point.x);
  const zs = path.map((point) => point.z);
  const ownStart = path[0];
  const ownEnd = path[path.length - 1];
  let targetError2 = 0;
  let straightnessError2 = 0;
  for (const point of path) {
    const tx = target.start.x + point.s * target.dx;
    const tz = target.start.z + point.s * target.dz;
    targetError2 += (point.x - tx) ** 2 + (point.z - tz) ** 2;

    const ox = ownStart.x + point.s * (ownEnd.x - ownStart.x);
    const oz = ownStart.z + point.s * (ownEnd.z - ownStart.z);
    straightnessError2 += (point.x - ox) ** 2 + (point.z - oz) ** 2;
  }

  return {
    valid: true,
    samples: path,
    target,
    verticalStroke: Math.max(...zs) - Math.min(...zs),
    foreAftExcursion: Math.max(...xs) - Math.min(...xs),
    targetRms: Math.sqrt(targetError2 / path.length),
    straightnessRms: Math.sqrt(straightnessError2 / path.length),
    minTransmission: Math.min(...path.map((point) => point.transmission)),
  };
}

export function referenceCom(config, offsetX = 0, offsetZ = 0) {
  // O2=P and O4=H in the robot-leg preset. The midpoint is deliberately a
  // reference COM, not a claim about the final robot mass distribution.
  return {
    x: Number(config.groundX ?? 0) / 2 + offsetX,
    z: -Number(config.groundY ?? 0) / 2 + offsetZ,
  };
}

export function takeoffMetrics(config, targetConfig = config, options = {}) {
  const angle = Number(options.angle ?? config.maxAngle ?? 0);
  const pose = solvePose(config, angle);
  if (!pose) return null;
  const wheel = robotFrame(pose.T);
  const com = referenceCom(
    config,
    Number(options.comOffsetX ?? 0),
    Number(options.comOffsetZ ?? 0),
  );
  const derivative = wheelDerivative(config, angle);
  const target = targetDiagonal(targetConfig);
  return {
    angle,
    wheel,
    com,
    wheelMinusComX: wheel.x - com.x,
    dxDq: derivative?.dxDq ?? null,
    dzDq: derivative?.dzDq ?? null,
    targetAngleDeg: target?.angleDeg ?? null,
  };
}

export function summarizeDesign(config, targetConfig = config) {
  const path = wheelPathMetrics(config, targetConfig, 0.5);
  let peakTorque = null;
  let peakStaticHoldTorque = null;
  try {
    const motion = analyzeMotion(config);
    if (motion?.status === "valid") peakTorque = motion.peaks?.peakTorque?.value ?? null;
  } catch {
    peakTorque = null;
  }
  try {
    const statics = analyzeStaticSupport(config);
    if (statics?.status === "valid") {
      peakStaticHoldTorque = statics.peaks?.peakHoldingTorque?.value ?? null;
    }
  } catch {
    peakStaticHoldTorque = null;
  }
  return {
    verticalStroke: path.verticalStroke,
    foreAftExcursion: path.foreAftExcursion,
    targetRms: path.targetRms,
    straightnessRms: path.straightnessRms,
    minTransmission: path.minTransmission,
    peakTorque,
    peakStaticHoldTorque,
  };
}

export function classifyScreeningCheck({
  index,
  blocked = false,
  disabled = false,
  warning = false,
  danger = false,
  pass = false,
}) {
  const categories = ["dynamic", "joint", "geometry", "static", "actuator"];
  const category = categories[index] ?? "analysis";
  if (blocked) return { category: "geometry", state: "critical" };
  if (disabled) return { category, state: "neutral" };
  if (danger) return { category, state: "critical" };
  if (warning) return { category, state: "warning" };
  if (pass) return { category, state: "pass" };
  return { category, state: "neutral" };
}
