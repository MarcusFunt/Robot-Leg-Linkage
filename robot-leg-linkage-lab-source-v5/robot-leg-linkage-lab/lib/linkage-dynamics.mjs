import {
  DEG,
  EPS,
  add,
  cross,
  magnitude,
  midpoint,
  mul,
  signedAngleDifference,
  solvePose,
  sub,
} from "./linkage-geometry.mjs";
import { kinematicStateAtAngle } from "./linkage-kinematics.mjs";

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

function resolveKinematicState(config, angleOrState) {
  if (angleOrState && typeof angleOrState === "object" && Number.isFinite(angleOrState.angle) && angleOrState.pose) return angleOrState;
  const angle = Number.isFinite(angleOrState) ? angleOrState : config.minAngle ?? 0;
  return kinematicStateAtAngle(config, angle);
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

export function inverseDynamics(config, angleOrState = config.minAngle ?? 0, motion = {}, externalForce = { x: 0, y: 0 }) {
  const state = resolveKinematicState(config, angleOrState);
  if (!state) return null;
  const angleDegrees = state.angle;
  const pose = state.pose;
  const h = 0.0005;
  const minus = solvePose(config, angleDegrees - h * DEG);
  const plus = solvePose(config, angleDegrees + h * DEG);
  if (!minus || !plus) return null;

  const body = bodyGeometry(config, pose);
  const bodyMinus = bodyGeometry(config, minus);
  const bodyPlus = bodyGeometry(config, plus);
  const first = (before, after) => mul(sub(after, before), 1 / (2 * h));
  const second = (before, current, after) => mul(add(sub(after, mul(current, 2)), before), 1 / (h * h));
  const omega2 = Number.isFinite(motion.omega) ? motion.omega : 0;
  const alpha2 = Number.isFinite(motion.alpha) ? motion.alpha : 0;
  const acceleration = (before, current, after) => add(
    mul(second(before, current, after), omega2 ** 2),
    mul(first(before, after), alpha2),
  );

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
  const load = {
    x: Number.isFinite(externalForce.x) ? externalForce.x : 0,
    y: Number.isFinite(externalForce.y) ? externalForce.y : 0,
  };

  const body3Force = sub(mul(sub(aG3, gravity), body.m3 / 1000), load);
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
    body.I3 * alpha3 / 1000 - cross(rT3, load),
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

export function dynamicsBreakdown(config, angleOrState = config.minAngle ?? 0, motion = {}, externalForce = { x: 0, y: 0 }) {
  const zeroMotion = { omega: 0, alpha: 0 };
  return {
    total: inverseDynamics(config, angleOrState, motion, externalForce),
    external: inverseDynamics(
      { ...config, gravity: false, crankMass: 0, legMass: 0, rockerMass: 0, toolMass: 0 },
      angleOrState,
      zeroMotion,
      externalForce,
    ),
    gravity: inverseDynamics(config, angleOrState, zeroMotion, { x: 0, y: 0 }),
    inertia: inverseDynamics({ ...config, gravity: false }, angleOrState, motion, { x: 0, y: 0 }),
  };
}
