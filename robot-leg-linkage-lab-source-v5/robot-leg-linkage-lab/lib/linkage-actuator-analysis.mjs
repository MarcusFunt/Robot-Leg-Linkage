import { EPS } from "./linkage-geometry.mjs";

function timeRms(samples, valueForSample) {
  let weightedSquares = 0;
  let totalTime = 0;
  for (let index = 0; index < samples.length - 1; index += 1) {
    const a = samples[index];
    const b = samples[index + 1];
    const va = valueForSample(a);
    const vb = valueForSample(b);
    if (!Number.isFinite(va) || !Number.isFinite(vb)) return null;
    const width = b.time - a.time;
    if (!(width > 0)) continue;
    weightedSquares += width * (va ** 2 + vb ** 2) / 2;
    totalTime += width;
  }
  return totalTime > 0 ? Math.sqrt(weightedSquares / totalTime) : null;
}

export function summarizeAnalysis(config, motionAnalysis, staticAnalysis, currentDynamics = null, currentSupport = null) {
  const motionValid = motionAnalysis?.status === "valid";
  const staticValid = staticAnalysis?.status === "valid";
  const efficiency = config.gearEfficiency / 100;
  const ratio = config.gearRatio;
  const peakTorque = motionValid ? motionAnalysis.peaks.peakTorque.value : null;
  const peakJointForce = motionValid ? motionAnalysis.peaks.peakJointReaction.value : null;
  const minTransmission = staticValid ? staticAnalysis.peaks.minTransmission.value : null;
  const currentMotorTorque = currentDynamics?.total ? Math.abs(currentDynamics.total.torque) / (ratio * efficiency) : null;
  const peakMotorTorque = peakTorque !== null ? peakTorque / (ratio * efficiency) : null;
  const rmsMotorTorque = motionValid
    ? timeRms(motionAnalysis.samples, (sample) => sample.dynamics ? sample.dynamics.torque / (ratio * efficiency) : NaN)
    : null;
  const peakMotorSpeedRpm = motionValid
    ? Math.max(...motionAnalysis.samples.map((sample) => Math.abs(sample.omega))) * ratio * 60 / (2 * Math.PI)
    : null;
  const peakMechanicalPowerW = motionValid
    ? Math.max(...motionAnalysis.samples.map((sample) => sample.linkPowerW === null ? 0 : Math.abs(sample.linkPowerW))) / efficiency
    : null;
  const peakStaticHoldTorque = staticValid ? staticAnalysis.peaks.peakHoldingTorque.value : null;
  const peakStaticMotorTorque = peakStaticHoldTorque !== null ? peakStaticHoldTorque / (ratio * efficiency) : null;
  const currentStaticMotorTorque = currentSupport?.holdingTorque != null
    ? Math.abs(currentSupport.holdingTorque) / (ratio * efficiency)
    : null;

  // Existing simple joint screen retained as-is; full link structural analysis is not part of this refactor.
  const pinArea = Math.PI * config.pinDiameter ** 2 / 4;
  const shearStress = peakJointForce !== null ? peakJointForce / (config.shearPlanes * pinArea) : null;
  const bearingStress = peakJointForce !== null ? peakJointForce / (config.pinDiameter * config.linkThickness) : null;

  return {
    motionValid,
    staticValid,
    peakTorque,
    peakJointForce,
    minTransmission,
    currentMotorTorque,
    peakMotorTorque,
    rmsMotorTorque,
    peakMotorSpeedRpm,
    peakMechanicalPowerW,
    peakStaticHoldTorque,
    peakStaticMotorTorque,
    currentStaticMotorTorque,
    shearStress,
    bearingStress,
    shearSafety: shearStress !== null ? (shearStress > EPS ? config.allowableShear / shearStress : Infinity) : null,
    bearingSafety: bearingStress !== null ? (bearingStress > EPS ? config.allowableBearing / bearingStress : Infinity) : null,
    continuousUse: rmsMotorTorque !== null ? rmsMotorTorque / config.motorContinuous * 100 : null,
    peakUse: peakMotorTorque !== null ? peakMotorTorque / config.motorPeak * 100 : null,
    staticPeakUse: peakStaticMotorTorque !== null ? peakStaticMotorTorque / config.motorPeak * 100 : null,
  };
}
