import { EPS, evaluateCycleSample, magnitude, sampleAngleRange } from "./linkage-core.mjs";

function finiteMetric(sample, metric) {
  const value = metric(sample);
  return Number.isFinite(value) ? value : null;
}

function slopeChangesSign(a, m, b, metric) {
  const va = finiteMetric(a, metric); const vm = finiteMetric(m, metric); const vb = finiteMetric(b, metric);
  if (va === null || vm === null || vb === null) return false;
  const left = vm - va;
  const right = vb - vm;
  const scale = Math.max(1e-9, Math.abs(va), Math.abs(vm), Math.abs(vb));
  return Math.abs(left) > scale * 1e-5 && Math.abs(right) > scale * 1e-5 && Math.sign(left) !== Math.sign(right);
}

function highCurvature(a, m, b, metric, threshold) {
  const va = finiteMetric(a, metric); const vm = finiteMetric(m, metric); const vb = finiteMetric(b, metric);
  if (va === null || vm === null || vb === null) return false;
  const curvature = Math.abs(va - 2 * vm + vb);
  const scale = Math.max(1, Math.abs(va), Math.abs(vm), Math.abs(vb));
  return curvature / scale > threshold;
}

function intervalNeedsRefinement(a, m, b, options) {
  const reachabilityChanged = Boolean(a.pose) !== Boolean(m.pose) || Boolean(m.pose) !== Boolean(b.pose);
  const lowTransmission = [a, m, b].some((sample) => sample.pose && sample.pose.transmission < options.lowTransmissionDeg);
  const dynamicBoundary = [a, m, b].some((sample) => sample.pose && !sample.dynamics);
  const torqueMetric = (sample) => sample.dynamics ? Math.abs(sample.dynamics.torque) : NaN;
  const reactionMetric = (sample) => sample.jointReaction ?? NaN;
  const transmissionMetric = (sample) => sample.pose?.transmission ?? NaN;
  const extremum = slopeChangesSign(a, m, b, torqueMetric)
    || slopeChangesSign(a, m, b, reactionMetric)
    || slopeChangesSign(a, m, b, transmissionMetric);
  const curvature = highCurvature(a, m, b, torqueMetric, options.curvatureThreshold)
    || highCurvature(a, m, b, reactionMetric, options.curvatureThreshold)
    || highCurvature(a, m, b, transmissionMetric, options.curvatureThreshold);
  return { refine: reachabilityChanged || lowTransmission || dynamicBoundary || extremum || curvature, reachabilityChanged, lowTransmission, dynamicBoundary, extremum, curvature };
}

function peakFromSamples(samples, metric, mode, unresolved) {
  const candidates = samples.map((sample) => ({ sample, value: finiteMetric(sample, metric) })).filter((item) => item.value !== null);
  if (!candidates.length) return { value: null, angle: null, convergence: "indeterminate", resolutionDeg: null };
  let best = candidates[0];
  for (const candidate of candidates.slice(1)) {
    if ((mode === "max" && candidate.value > best.value) || (mode === "min" && candidate.value < best.value)) best = candidate;
  }
  const nearUnresolved = unresolved.some((interval) => best.sample.angle >= interval.start - interval.width && best.sample.angle <= interval.end + interval.width);
  const nearToggle = best.sample.pose && best.sample.pose.transmission < 2;
  const convergence = nearUnresolved && nearToggle ? "indeterminate near toggle" : nearUnresolved ? "not converged" : "converged";
  const index = samples.indexOf(best.sample);
  const before = samples[index - 1]; const after = samples[index + 1];
  const resolutionDeg = before && after ? Math.max(best.sample.angle - before.angle, after.angle - best.sample.angle) : null;
  return { value: best.value, angle: best.sample.angle, convergence, resolutionDeg };
}

export function adaptiveAnalyze(config, userOptions = {}) {
  const options = {
    coarseStepDeg: 10,
    angleToleranceDeg: 0.05,
    maxDepth: 12,
    lowTransmissionDeg: 35,
    singularityThresholdDeg: 0.5,
    curvatureThreshold: 0.04,
    ...userOptions,
  };
  const start = Math.min(config.minAngle, config.maxAngle);
  const end = Math.max(config.minAngle, config.maxAngle);
  const cache = new Map();
  const keyFor = (angle) => angle.toFixed(12);
  const sampleAt = (angle) => {
    const key = keyFor(angle);
    if (!cache.has(key)) cache.set(key, evaluateCycleSample(config, angle));
    return cache.get(key);
  };
  const unresolved = [];

  const refine = (aAngle, bAngle, depth) => {
    const a = sampleAt(aAngle);
    const b = sampleAt(bAngle);
    const midpointAngle = (aAngle + bAngle) / 2;
    const midpoint = sampleAt(midpointAngle);
    const reasons = intervalNeedsRefinement(a, midpoint, b, options);
    const width = bAngle - aAngle;
    if (!reasons.refine) return;
    if (width <= options.angleToleranceDeg) {
      const singularAtResolution = [a, midpoint, b].some((sample) => !sample.pose || !sample.dynamics || (sample.pose?.transmission ?? 90) < options.singularityThresholdDeg);
      if ((reasons.reachabilityChanged || reasons.dynamicBoundary || singularAtResolution)) {
        unresolved.push({ start: aAngle, end: bAngle, width, reasons: { ...reasons, singularAtResolution } });
      }
      return;
    }
    if (depth >= options.maxDepth) {
      unresolved.push({ start: aAngle, end: bAngle, width, reasons });
      return;
    }
    refine(aAngle, midpointAngle, depth + 1);
    refine(midpointAngle, bAngle, depth + 1);
  };

  const coarseAngles = sampleAngleRange(start, end, options.coarseStepDeg);
  coarseAngles.forEach(sampleAt);
  for (let index = 0; index < coarseAngles.length - 1; index += 1) refine(coarseAngles[index], coarseAngles[index + 1], 0);

  const samples = [...cache.values()].sort((a, b) => a.angle - b.angle);
  const peakTorque = peakFromSamples(samples, (sample) => sample.dynamics ? Math.abs(sample.dynamics.torque) : NaN, "max", unresolved);
  const peakJointReaction = peakFromSamples(samples, (sample) => sample.jointReaction ?? NaN, "max", unresolved);
  const minTransmission = peakFromSamples(samples, (sample) => sample.pose?.transmission ?? NaN, "min", unresolved);
  const nearToggleUnresolved = unresolved.some((interval) => {
    const inInterval = samples.filter((sample) => sample.angle >= interval.start && sample.angle <= interval.end);
    return inInterval.some((sample) => !sample.pose || !sample.dynamics || (sample.pose?.transmission ?? 90) < 2);
  });

  return {
    samples,
    peaks: { peakTorque, peakJointReaction, minTransmission },
    convergence: {
      status: nearToggleUnresolved ? "indeterminate near toggle" : unresolved.length ? "not fully converged" : "converged",
      unresolvedIntervals: unresolved,
      maxDepth: options.maxDepth,
      angleToleranceDeg: options.angleToleranceDeg,
    },
  };
}

function trapezoidRms(samples, valueForSample) {
  let weightedSquares = 0;
  let totalAngle = 0;
  for (let index = 0; index < samples.length - 1; index += 1) {
    const a = samples[index]; const b = samples[index + 1];
    const va = valueForSample(a); const vb = valueForSample(b);
    if (!Number.isFinite(va) || !Number.isFinite(vb)) continue;
    const width = b.angle - a.angle;
    if (!(width > 0)) continue;
    weightedSquares += width * (va ** 2 + vb ** 2) / 2;
    totalAngle += width;
  }
  return totalAngle > 0 ? Math.sqrt(weightedSquares / totalAngle) : null;
}

export function summarizeAnalysis(config, analysis, currentDynamics = null) {
  const valid = analysis.samples.filter((sample) => sample.pose && sample.dynamics);
  const peakTorque = analysis.peaks.peakTorque.value ?? 0;
  const peakJointForce = analysis.peaks.peakJointReaction.value ?? 0;
  const minTransmission = analysis.peaks.minTransmission.value ?? 0;
  const efficiency = config.gearEfficiency / 100;
  const ratio = config.gearRatio;
  const currentMotorTorque = currentDynamics?.total ? Math.abs(currentDynamics.total.torque) / (ratio * efficiency) : null;
  const peakMotorTorque = peakTorque / (ratio * efficiency);
  const rmsMotorTorque = trapezoidRms(analysis.samples, (sample) => sample.dynamics ? sample.dynamics.torque / (ratio * efficiency) : NaN);
  const peakMotorSpeedRpm = Math.abs(config.rpm * ratio);
  const motorAngularSpeed = peakMotorSpeedRpm * Math.PI / 30;
  const peakMechanicalPowerW = peakMotorTorque * motorAngularSpeed;
  const pinArea = Math.PI * config.pinDiameter ** 2 / 4;
  const shearStress = peakJointForce / (config.shearPlanes * pinArea);
  const bearingStress = peakJointForce / (config.pinDiameter * config.linkThickness);
  const indeterminate = analysis.convergence.status === "indeterminate near toggle";
  return {
    valid,
    peakTorque,
    peakJointForce,
    minTransmission,
    currentMotorTorque,
    peakMotorTorque,
    rmsMotorTorque,
    peakMotorSpeedRpm,
    peakMechanicalPowerW,
    shearStress,
    bearingStress,
    shearSafety: indeterminate ? null : (shearStress > EPS ? config.allowableShear / shearStress : Infinity),
    bearingSafety: indeterminate ? null : (bearingStress > EPS ? config.allowableBearing / bearingStress : Infinity),
    continuousUse: rmsMotorTorque !== null ? rmsMotorTorque / config.motorContinuous * 100 : null,
    peakUse: peakMotorTorque / config.motorPeak * 100,
    indeterminateNearToggle: indeterminate,
  };
}

export function analysisToCsv(analysis) {
  const header = [
    "angle_deg", "reachable", "tool_x_mm", "tool_y_mm", "transmission_deg",
    "link_torque_Nm", "external_torque_Nm", "gravity_torque_Nm", "inertia_torque_Nm",
    "O2_reaction_N", "A_reaction_N", "B_reaction_N", "O4_reaction_N",
  ];
  const rows = analysis.samples.map((sample) => {
    if (!sample.pose || !sample.dynamics) return [sample.angle.toFixed(8), false, "", "", "", "", "", "", "", "", "", "", ""];
    return [
      sample.angle.toFixed(8), true, sample.pose.T.x.toFixed(6), sample.pose.T.y.toFixed(6), sample.pose.transmission.toFixed(6),
      sample.dynamics.torque.toFixed(6), sample.externalTorque?.toFixed(6) ?? "", sample.gravityTorque?.toFixed(6) ?? "", sample.inertiaTorque?.toFixed(6) ?? "",
      magnitude(sample.dynamics.O2Reaction).toFixed(6), magnitude(sample.dynamics.AReaction).toFixed(6), magnitude(sample.dynamics.BReaction).toFixed(6), magnitude(sample.dynamics.O4Reaction).toFixed(6),
    ];
  });
  return [header, ...rows].map((row) => row.join(",")).join("\n");
}
