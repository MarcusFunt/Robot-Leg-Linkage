export const RAD = Math.PI / 180;
export const DEG = 180 / Math.PI;
export const EPS = 1e-9;

export function add(a, b) { return { x: a.x + b.x, y: a.y + b.y }; }
export function sub(a, b) { return { x: a.x - b.x, y: a.y - b.y }; }
export function mul(a, scalar) { return { x: a.x * scalar, y: a.y * scalar }; }
export function midpoint(a, b) { return mul(add(a, b), 0.5); }
export function cross(a, b) { return a.x * b.y - a.y * b.x; }
export function magnitude(vector) { return Math.hypot(vector.x, vector.y); }
export function wrapDegrees(value) { return ((value % 360) + 360) % 360; }
export function clamp(value, minimum, maximum) { return Math.min(maximum, Math.max(minimum, value)); }
export function signedAngleDifference(a, b) { return Math.atan2(Math.sin(a - b), Math.cos(a - b)); }

export function sampleAngleRange(minimum, maximum, step) {
  const start = Math.min(minimum, maximum);
  const end = Math.max(minimum, maximum);
  const increment = Math.max(0.001, Math.abs(step));
  const count = Math.floor((end - start) / increment);
  const samples = Array.from({ length: count + 1 }, (_, index) => start + index * increment);
  if (!samples.length || Math.abs(end - samples.at(-1)) > 1e-10) samples.push(end);
  return samples;
}

export function solvePose(config, angleDegrees = config.minAngle ?? 0) {
  const { groundX, groundY, crank, coupler, rocker, toolAlong, toolOffset } = config;
  if (![groundX, groundY, crank, coupler, rocker, toolAlong, toolOffset, angleDegrees].every(Number.isFinite)) return null;
  if ([crank, coupler, rocker].some((value) => value <= 0)) return null;

  // The driven member is B-04.  `angleDegrees` is therefore theta_4, not
  // theta_2; A-02 is the non-driven grounded rocker and T is the output point.
  const theta4 = angleDegrees * RAD;
  const O2 = { x: 0, y: 0 };
  const O4 = { x: groundX, y: groundY };
  const B = { x: O4.x + rocker * Math.cos(theta4), y: O4.y + rocker * Math.sin(theta4) };
  const delta = sub(O2, B);
  const distance = magnitude(delta);
  if (distance < EPS || distance > coupler + crank + EPS || distance < Math.abs(coupler - crank) - EPS) return null;

  const unit = mul(delta, 1 / distance);
  const normal = { x: -unit.y, y: unit.x };
  const along = (coupler ** 2 - crank ** 2 + distance ** 2) / (2 * distance);
  const heightSquared = Math.max(0, coupler ** 2 - along ** 2);
  const base = add(B, mul(unit, along));
  const A = add(base, mul(normal, config.branch * Math.sqrt(heightSquared)));
  const couplerUnit = mul(sub(B, A), 1 / coupler);
  const couplerNormal = { x: -couplerUnit.y, y: couplerUnit.x };
  const T = add(A, add(mul(couplerUnit, toolAlong), mul(couplerNormal, toolOffset)));
  const theta2 = Math.atan2(A.y - O2.y, A.x - O2.x);
  const theta3 = Math.atan2(B.y - A.y, B.x - A.x);
  let transmission = Math.abs(signedAngleDifference(theta3, theta2)) * DEG;
  if (transmission > 90) transmission = 180 - transmission;
  return { O2, O4, A, B, T, theta2, theta3, theta4, transmission };
}

function includeEquivalentAngle(candidates, angle, start, end) {
  for (let turn = -2; turn <= 2; turn += 1) {
    const candidate = angle + 360 * turn;
    if (candidate >= start - 1e-10 && candidate <= end + 1e-10) candidates.push(candidate);
  }
}

export function motionWindowReachability(config) {
  const start = Math.min(config.minAngle, config.maxAngle);
  const end = Math.max(config.minAngle, config.maxAngle);
  const ground = Math.hypot(config.groundX, config.groundY);
  const lowerLimit = Math.abs(config.coupler - config.crank);
  const upperLimit = config.coupler + config.crank;
  if (!(ground > EPS) || !(config.crank > 0) || !(config.coupler > 0) || !(config.rocker > 0)) {
    return { fullyReachable: false, degenerate: true, minDistance: 0, maxDistance: 0, lowerLimit, upperLimit };
  }

  const phase = Math.atan2(config.groundY, config.groundX) * DEG;
  const candidates = [start, end];
  includeEquivalentAngle(candidates, phase, start, end);
  includeEquivalentAngle(candidates, phase + 180, start, end);
  const distances = candidates.map((angle) => {
    const theta = angle * RAD;
    const bx = config.groundX + config.rocker * Math.cos(theta);
    const by = config.groundY + config.rocker * Math.sin(theta);
    return Math.hypot(bx, by);
  });
  const minDistance = Math.min(...distances);
  const maxDistance = Math.max(...distances);
  return {
    fullyReachable: minDistance >= lowerLimit - 1e-8 && maxDistance <= upperLimit + 1e-8,
    degenerate: false,
    minDistance,
    maxDistance,
    lowerLimit,
    upperLimit,
  };
}

export function mechanismClass(config) {
  const ground = Math.hypot(config.groundX, config.groundY);
  const named = [
    { name: "ground", value: ground },
    { name: "input crank", value: config.rocker },
    { name: "coupler", value: config.coupler },
    { name: "output rocker", value: config.crank },
  ].sort((a, b) => a.value - b.value);
  const shortest = named[0];
  const longest = named[3];
  const margin = named[1].value + named[2].value - shortest.value - longest.value;
  const grashof = margin >= -1e-7;
  const changePoint = Math.abs(margin) < 1e-7;
  let type = "Non-Grashof double rocker";
  let inputRotates = false;
  if (grashof) {
    if (changePoint) type = "Grashof change-point mechanism";
    else if (shortest.name === "ground") type = "Grashof double crank";
    else if (shortest.name === "coupler") type = "Grashof double rocker";
    else type = `Grashof crank-rocker · ${shortest.name} rotates`;
    inputRotates = shortest.name === "ground" || shortest.name === "input crank";
  }
  return { ground, margin, grashof, type, inputRotates };
}
