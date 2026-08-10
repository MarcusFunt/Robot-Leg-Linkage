export function validateNumericValue(value, rules = {}) {
  if (!Number.isFinite(value)) return "Enter a finite number.";
  if (rules.min !== undefined && value < rules.min) return `Must be ≥ ${rules.min}.`;
  if (rules.max !== undefined && value > rules.max) return `Must be ≤ ${rules.max}.`;
  if (rules.integer && !Number.isInteger(value)) return "Must be a whole number.";
  if (rules.validate) return rules.validate(value) ?? null;
  return null;
}

export function validateConfig(config) {
  const errors = {};
  const positive = ["crank", "coupler", "rocker", "pinDiameter", "linkThickness", "gearRatio", "motorContinuous", "motorPeak"];
  for (const key of positive) if (!Number.isFinite(config[key]) || config[key] <= 0) errors[key] = "Must be greater than zero.";
  const nonNegative = ["rpm", "crankMass", "legMass", "rockerMass", "toolMass"];
  for (const key of nonNegative) if (!Number.isFinite(config[key]) || config[key] < 0) errors[key] = "Must be zero or greater.";
  for (const [key, value] of Object.entries(config)) if (typeof value === "number" && !Number.isFinite(value)) errors[key] = "Enter a finite number.";
  if (!(config.gearEfficiency > 0 && config.gearEfficiency <= 100)) errors.gearEfficiency = "Efficiency must be > 0% and ≤ 100%.";
  if (!(Number.isInteger(config.shearPlanes) && config.shearPlanes >= 1)) errors.shearPlanes = "Shear planes must be a whole number ≥ 1.";
  if (config.motorPeak < config.motorContinuous) errors.motorPeak = "Peak torque must be ≥ continuous torque.";
  if (!(config.minAngle >= 0 && config.maxAngle <= 360 && config.maxAngle - config.minAngle >= 0.5)) errors.motionWindow = "Motion window must be within 0–360° and at least 0.5° wide.";
  return errors;
}
