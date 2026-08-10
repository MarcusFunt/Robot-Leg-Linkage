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

export function solvePose(config, angleDegrees = config.angle) {
  const { groundX, groundY, crank, coupler, rocker, toolAlong, toolOffset } = config;
  if (![groundX, groundY, crank, coupler, rocker, toolAlong, toolOffset, angleDegrees].every(Number.isFinite)) return null;
  if ([crank, coupler, rocker].some((value) => value <= 0)) return null;

  const theta2 = angleDegrees * RAD;
  const O2 = { x: 0, y: 0 };
  const O4 = { x: groundX, y: groundY };
  const A = { x: crank * Math.cos(theta2), y: crank * Math.sin(theta2) };
  const delta = sub(O4, A);
  const distance = magnitude(delta);

  if (distance < EPS || distance > coupler + rocker + EPS || distance < Math.abs(coupler - rocker) - EPS) return null;

  const unit = mul(delta, 1 / distance);
  const normal = { x: -unit.y, y: unit.x };
  const along = (coupler ** 2 - rocker ** 2 + distance ** 2) / (2 * distance);
  const heightSquared = Math.max(0, coupler ** 2 - along ** 2);
  const base = add(A, mul(unit, along));
  const B = add(base, mul(normal, config.branch * Math.sqrt(heightSquared)));
  const couplerUnit = mul(sub(B, A), 1 / coupler);
  const couplerNormal = { x: -couplerUnit.y, y: couplerUnit.x };
  const T = add(A, add(mul(couplerUnit, toolAlong), mul(couplerNormal, toolOffset)));
  const theta3 = Math.atan2(B.y - A.y, B.x - A.x);
  const theta4 = Math.atan2(B.y - O4.y, B.x - O4.x);
  let transmission = Math.abs(signedAngleDifference(theta3, theta4)) * DEG;
  if (transmission > 90) transmission = 180 - transmission;
  return { O2, O4, A, B, T, theta2, theta3, theta4, transmission };
}

export function kinematics(config, pose = solvePose(config)) {
  if (!pose) return null;
  const h = 0.001;
  const angle = config.angle ?? pose.theta2 * DEG;
  const minus = solvePose(config, angle - h * DEG);
  const plus = solvePose(config, angle + h * DEG);
  if (!minus || !plus) return null;
  return {
    omega3Ratio: signedAngleDifference(plus.theta3, minus.theta3) / (2 * h),
    omega4Ratio: signedAngleDifference(plus.theta4, minus.theta4) / (2 * h),
    toolDerivative: mul(sub(plus.T, minus.T), 1 / (2 * h)),
  };
}

function bodyGeometry(config, pose) {
  const m2 = config.crankMass;
  const legMass = config.legMass;
  const toolMass = config.toolMass;
  const m3 = legMass + toolMass;
  const m4 = config.rockerMass;
  const G2 = midpoint(pose.O2, pose.A);
  const legCenter = midpoint(pose.A, pose.T);
  const G3 = m3 > EPS ? mul(add(mul(legCenter, legMass), mul(pose.T, toolMass)), 1 / m3) : legCenter;
  const G4 = midpoint(pose.O4, pose.B);
  const legLength = magnitude(sub(pose.T, pose.A));
  const I2 = m2 * config.crank ** 2 / 12;
  const I3 = legMass * legLength ** 2 / 12
    + legMass * magnitude(sub(legCenter, G3)) ** 2
    + toolMass * magnitude(sub(pose.T, G3)) ** 2;
  const I4 = m4 * config.rocker ** 2 / 12;
  return { G2, G3, G4, I2, I3, I4, m2, m3, m4 };
}

export function solveLinearSystem(matrix, rhs) {
  const augmented = matrix.map((row, index) => [...row, rhs[index]]);
  const size = rhs.length;
  for (let column = 0; column < size; column += 1) {
    let pivot = column;
    for (let row = column + 1; row < size; row += 1) {
      if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column])) pivot = row;
    }
    if (Math.abs(augmented[pivot][column]) < 1e-10) return null;
    [augmented[column], augmented[pivot]] = [augmented[pivot], augmented[column]];
    const divisor = augmented[column][column];
    for (let index = column; index <= size; index += 1) augmented[column][index] /= divisor;
    for (let row = 0; row < size; row += 1) {
      if (row === column) continue;
      const factor = augmented[row][column];
      for (let index = column; index <= size; index += 1) augmented[row][index] -= factor * augmented[column][index];
    }
  }
  return augmented.map((row) => row[size]);
}

export function inverseDynamics(config, angleDegrees = config.angle) {
  const h = 0.0005;
  const pose = solvePose(config, angleDegrees);
  const minus = solvePose(config, angleDegrees - h * DEG);
  const plus = solvePose(config, angleDegrees + h * DEG);
  if (!pose || !minus || !plus) return null;

  const body = bodyGeometry(config, pose);
  const bodyMinus = bodyGeometry(config, minus);
  const bodyPlus = bodyGeometry(config, plus);
  const first = (before, after) => mul(sub(after, before), 1 / (2 * h));
  const second = (before, current, after) => mul(add(sub(after, mul(current, 2)), before), 1 / (h * h));
  const omega2 = config.rpm * Math.PI / 30;
  const alpha2 = config.inputAccel;
  const acceleration = (before, current, after) => add(mul(second(before, current, after), omega2 ** 2), mul(first(before, after), alpha2));

  const aG2 = acceleration(bodyMinus.G2, body.G2, bodyPlus.G2);
  const aG3 = acceleration(bodyMinus.G3, body.G3, bodyPlus.G3);
  const aG4 = acceleration(bodyMinus.G4, body.G4, bodyPlus.G4);
  const theta3First = signedAngleDifference(plus.theta3, minus.theta3) / (2 * h);
  const theta4First = signedAngleDifference(plus.theta4, minus.theta4) / (2 * h);
  const theta3Second = (signedAngleDifference(plus.theta3, pose.theta3) - signedAngleDifference(pose.theta3, minus.theta3)) / (h * h);
  const theta4Second = (signedAngleDifference(plus.theta4, pose.theta4) - signedAngleDifference(pose.theta4, minus.theta4)) / (h * h);
  const alpha3 = theta3Second * omega2 ** 2 + theta3First * alpha2;
  const alpha4 = theta4Second * omega2 ** 2 + theta4First * alpha2;
  const gravity = { x: 0, y: config.gravity ? -9810 : 0 };
  const externalForce = { x: config.forceX, y: config.forceY };

  const body3Force = sub(mul(sub(aG3, gravity), body.m3 / 1000), externalForce);
  const rA3 = sub(pose.A, body.G3);
  const rB3 = sub(pose.B, body.G3);
  const rT3 = sub(pose.T, body.G3);
  const body4Force = mul(sub(aG4, gravity), body.m4 / 1000);
  const rO4 = sub(pose.O4, body.G4);
  const rB4 = sub(pose.B, body.G4);
  const rockerArm = sub(rO4, rB4);

  const solution = solveLinearSystem([
    [-1, 0, 1, 0],
    [0, -1, 0, 1],
    [rA3.y, -rA3.x, -rB3.y, rB3.x],
    [0, 0, -rockerArm.y, rockerArm.x],
  ], [
    body3Force.x,
    body3Force.y,
    body.I3 * alpha3 / 1000 - cross(rT3, externalForce),
    body.I4 * alpha4 / 1000 - cross(rO4, body4Force),
  ]);
  if (!solution || solution.some((value) => !Number.isFinite(value))) return null;

  const AReaction = { x: solution[0], y: solution[1] };
  const BReaction = { x: solution[2], y: solution[3] };
  const O2Reaction = sub(mul(sub(aG2, gravity), body.m2 / 1000), AReaction);
  const O4Reaction = add(body4Force, BReaction);
  const torqueNmm = body.I2 * alpha2 / 1000
    - cross(sub(pose.O2, body.G2), O2Reaction)
    - cross(sub(pose.A, body.G2), AReaction);

  return { torque: torqueNmm / 1000, O2Reaction, AReaction, BReaction, O4Reaction, alpha3, alpha4 };
}

export function dynamicsBreakdown(config, angleDegrees = config.angle) {
  return {
    total: inverseDynamics(config, angleDegrees),
    external: inverseDynamics({ ...config, gravity: false, rpm: 0, inputAccel: 0, crankMass: 0, legMass: 0, rockerMass: 0, toolMass: 0 }, angleDegrees),
    gravity: inverseDynamics({ ...config, forceX: 0, forceY: 0, rpm: 0, inputAccel: 0 }, angleDegrees),
    inertia: inverseDynamics({ ...config, forceX: 0, forceY: 0, gravity: false }, angleDegrees),
  };
}

export function staticInputTorqueFromJacobian(config, angleDegrees = config.angle) {
  const hDegrees = 1e-3;
  const before = solvePose(config, angleDegrees - hDegrees);
  const after = solvePose(config, angleDegrees + hDegrees);
  if (!before || !after) return null;
  const dTheta = 2 * hDegrees * RAD;
  const dT = mul(sub(after.T, before.T), 1 / dTheta);
  return -(config.forceX * dT.x + config.forceY * dT.y) / 1000;
}

export function mechanismClass(config) {
  const ground = Math.hypot(config.groundX, config.groundY);
  const named = [
    { name: "ground", value: ground },
    { name: "input crank", value: config.crank },
    { name: "coupler", value: config.coupler },
    { name: "output rocker", value: config.rocker },
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

export function evaluateCycleSample(config, angle) {
  const pose = solvePose(config, angle);
  if (!pose) return { angle, pose: null, dynamics: null, externalTorque: null, gravityTorque: null, inertiaTorque: null, jointReaction: null };
  const breakdown = dynamicsBreakdown(config, angle);
  const dynamics = breakdown.total;
  const jointReaction = dynamics ? Math.max(
    magnitude(dynamics.O2Reaction), magnitude(dynamics.AReaction), magnitude(dynamics.BReaction), magnitude(dynamics.O4Reaction),
  ) : null;
  return {
    angle,
    pose,
    dynamics,
    externalTorque: breakdown.external?.torque ?? null,
    gravityTorque: breakdown.gravity?.torque ?? null,
    inertiaTorque: breakdown.inertia?.torque ?? null,
    jointReaction,
  };
}

