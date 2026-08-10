"use client";

import { useEffect, useMemo, useRef, useState } from "react";

type Vec = { x: number; y: number };
type Branch = -1 | 1;
type ControlTab = "geometry" | "loads" | "hardware";

type Config = {
  groundX: number;
  groundY: number;
  crank: number;
  coupler: number;
  rocker: number;
  toolAlong: number;
  toolOffset: number;
  angle: number;
  minAngle: number;
  maxAngle: number;
  rpm: number;
  inputAccel: number;
  branch: Branch;
  forceX: number;
  forceY: number;
  gravity: boolean;
  crankMass: number;
  legMass: number;
  rockerMass: number;
  toolMass: number;
  pinDiameter: number;
  linkThickness: number;
  shearPlanes: number;
  allowableShear: number;
  allowableBearing: number;
  gearRatio: number;
  gearEfficiency: number;
  motorContinuous: number;
  motorPeak: number;
};

type Pose = {
  O2: Vec;
  O4: Vec;
  A: Vec;
  B: Vec;
  T: Vec;
  theta2: number;
  theta3: number;
  theta4: number;
  transmission: number;
};

type BodyGeometry = {
  G2: Vec;
  G3: Vec;
  G4: Vec;
  I2: number;
  I3: number;
  I4: number;
  m2: number;
  m3: number;
  m4: number;
};

type Dynamics = {
  torque: number;
  O2Reaction: Vec;
  AReaction: Vec;
  BReaction: Vec;
  O4Reaction: Vec;
  alpha3: number;
  alpha4: number;
};

type CycleSample = {
  angle: number;
  pose: Pose | null;
  dynamics: Dynamics | null;
  externalTorque: number | null;
  gravityTorque: number | null;
  inertiaTorque: number | null;
};

const DEFAULT_CONFIG: Config = {
  groundX: 45,
  groundY: -40,
  crank: 40,
  coupler: 45,
  rocker: 60,
  toolAlong: 120,
  toolOffset: 0,
  angle: 180,
  minAngle: 0,
  maxAngle: 360,
  rpm: 60,
  inputAccel: 0,
  branch: -1,
  forceX: 0,
  forceY: 100,
  gravity: true,
  crankMass: 0.12,
  legMass: 0.28,
  rockerMass: 0.15,
  toolMass: 0.08,
  pinDiameter: 8,
  linkThickness: 4,
  shearPlanes: 2,
  allowableShear: 120,
  allowableBearing: 80,
  gearRatio: 10,
  gearEfficiency: 90,
  motorContinuous: 0.6,
  motorPeak: 1.2,
};

const RAD = Math.PI / 180;
const DEG = 180 / Math.PI;
const EPS = 1e-9;

function add(a: Vec, b: Vec): Vec {
  return { x: a.x + b.x, y: a.y + b.y };
}

function sub(a: Vec, b: Vec): Vec {
  return { x: a.x - b.x, y: a.y - b.y };
}

function mul(a: Vec, scalar: number): Vec {
  return { x: a.x * scalar, y: a.y * scalar };
}

function midpoint(a: Vec, b: Vec): Vec {
  return mul(add(a, b), 0.5);
}

function cross(a: Vec, b: Vec): number {
  return a.x * b.y - a.y * b.x;
}

function length(a: Vec): number {
  return Math.hypot(a.x, a.y);
}

function wrapDegrees(value: number): number {
  return ((value % 360) + 360) % 360;
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(maximum, Math.max(minimum, value));
}

function sampleAngleRange(minimum: number, maximum: number, step: number): number[] {
  const start = clamp(Math.min(minimum, maximum), 0, 360);
  const end = clamp(Math.max(minimum, maximum), 0, 360);
  const increment = Math.max(0.1, step);
  const samples = Array.from(
    { length: Math.floor((end - start) / increment) + 1 },
    (_, index) => start + index * increment,
  );
  if (!samples.length || end - samples[samples.length - 1] > 1e-7) samples.push(end);
  return samples;
}

function signedAngleDifference(a: number, b: number): number {
  return Math.atan2(Math.sin(a - b), Math.cos(a - b));
}

function solvePose(config: Config, angleDegrees = config.angle): Pose | null {
  const { groundX, groundY, crank, coupler, rocker, toolAlong, toolOffset } = config;
  if ([crank, coupler, rocker].some((value) => !Number.isFinite(value) || value <= 0)) return null;

  const theta2 = angleDegrees * RAD;
  const O2 = { x: 0, y: 0 };
  const O4 = { x: groundX, y: groundY };
  const A = { x: crank * Math.cos(theta2), y: crank * Math.sin(theta2) };
  const delta = sub(O4, A);
  const distance = length(delta);

  if (distance < EPS || distance > coupler + rocker + EPS || distance < Math.abs(coupler - rocker) - EPS) {
    return null;
  }

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

function kinematics(config: Config, pose: Pose | null) {
  if (!pose) return null;
  const h = 0.001;
  const minus = solvePose(config, config.angle - h * DEG);
  const plus = solvePose(config, config.angle + h * DEG);
  if (!minus || !plus) return null;

  const omega3Ratio = signedAngleDifference(plus.theta3, minus.theta3) / (2 * h);
  const omega4Ratio = signedAngleDifference(plus.theta4, minus.theta4) / (2 * h);
  const toolDerivative = mul(sub(plus.T, minus.T), 1 / (2 * h));
  return { omega3Ratio, omega4Ratio, toolDerivative };
}

function bodyGeometry(config: Config, pose: Pose): BodyGeometry {
  const m2 = Math.max(0, config.crankMass);
  const legMass = Math.max(0, config.legMass);
  const toolMass = Math.max(0, config.toolMass);
  const m3 = legMass + toolMass;
  const m4 = Math.max(0, config.rockerMass);
  const G2 = midpoint(pose.O2, pose.A);
  const legCenter = midpoint(pose.A, pose.T);
  const G3 = m3 > EPS
    ? mul(add(mul(legCenter, legMass), mul(pose.T, toolMass)), 1 / m3)
    : legCenter;
  const G4 = midpoint(pose.O4, pose.B);
  const legLength = length(sub(pose.T, pose.A));
  const I2 = m2 * config.crank ** 2 / 12;
  const I3 = legMass * legLength ** 2 / 12
    + legMass * length(sub(legCenter, G3)) ** 2
    + toolMass * length(sub(pose.T, G3)) ** 2;
  const I4 = m4 * config.rocker ** 2 / 12;
  return { G2, G3, G4, I2, I3, I4, m2, m3, m4 };
}

function solveLinearSystem(matrix: number[][], rhs: number[]): number[] | null {
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
      for (let index = column; index <= size; index += 1) {
        augmented[row][index] -= factor * augmented[column][index];
      }
    }
  }
  return augmented.map((row) => row[size]);
}

function inverseDynamics(config: Config, angleDegrees = config.angle): Dynamics | null {
  const h = 0.0005;
  const pose = solvePose(config, angleDegrees);
  const minus = solvePose(config, angleDegrees - h * DEG);
  const plus = solvePose(config, angleDegrees + h * DEG);
  if (!pose || !minus || !plus) return null;

  const body = bodyGeometry(config, pose);
  const bodyMinus = bodyGeometry(config, minus);
  const bodyPlus = bodyGeometry(config, plus);
  const first = (before: Vec, after: Vec) => mul(sub(after, before), 1 / (2 * h));
  const second = (before: Vec, current: Vec, after: Vec) => mul(add(sub(after, mul(current, 2)), before), 1 / (h * h));
  const omega2 = config.rpm * Math.PI / 30;
  const alpha2 = config.inputAccel;
  const acceleration = (before: Vec, current: Vec, after: Vec) =>
    add(mul(second(before, current, after), omega2 ** 2), mul(first(before, after), alpha2));

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

  const matrix = [
    [-1, 0, 1, 0],
    [0, -1, 0, 1],
    [rA3.y, -rA3.x, -rB3.y, rB3.x],
    [0, 0, -rockerArm.y, rockerArm.x],
  ];
  const rhs = [
    body3Force.x,
    body3Force.y,
    body.I3 * alpha3 / 1000 - cross(rT3, externalForce),
    body.I4 * alpha4 / 1000 - cross(rO4, body4Force),
  ];
  const solution = solveLinearSystem(matrix, rhs);
  if (!solution || solution.some((value) => !Number.isFinite(value))) return null;

  const AReaction = { x: solution[0], y: solution[1] };
  const BReaction = { x: solution[2], y: solution[3] };
  const O2Reaction = sub(mul(sub(aG2, gravity), body.m2 / 1000), AReaction);
  const O4Reaction = add(body4Force, BReaction);
  const torqueNmm = body.I2 * alpha2 / 1000
    - cross(sub(pose.O2, body.G2), O2Reaction)
    - cross(sub(pose.A, body.G2), AReaction);

  return {
    torque: torqueNmm / 1000,
    O2Reaction,
    AReaction,
    BReaction,
    O4Reaction,
    alpha3,
    alpha4,
  };
}

function dynamicsBreakdown(config: Config, angleDegrees = config.angle) {
  const total = inverseDynamics(config, angleDegrees);
  const external = inverseDynamics({
    ...config,
    gravity: false,
    rpm: 0,
    inputAccel: 0,
    crankMass: 0,
    legMass: 0,
    rockerMass: 0,
    toolMass: 0,
  }, angleDegrees);
  const gravity = inverseDynamics({
    ...config,
    forceX: 0,
    forceY: 0,
    rpm: 0,
    inputAccel: 0,
  }, angleDegrees);
  const inertia = inverseDynamics({
    ...config,
    forceX: 0,
    forceY: 0,
    gravity: false,
  }, angleDegrees);
  return { total, external, gravity, inertia };
}

function magnitude(vector: Vec): number {
  return Math.hypot(vector.x, vector.y);
}

function mechanismClass(config: Config) {
  const ground = Math.hypot(config.groundX, config.groundY);
  const named = [
    { name: "ground", value: ground },
    { name: "input crank", value: config.crank },
    { name: "coupler", value: config.coupler },
    { name: "output rocker", value: config.rocker },
  ].sort((a, b) => a.value - b.value);
  const shortest = named[0];
  const longest = named[3];
  const middleSum = named[1].value + named[2].value;
  const margin = middleSum - shortest.value - longest.value;
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

function NumericInput({
  label,
  value,
  unit,
  step = 1,
  min,
  max,
  onChange,
}: {
  label: string;
  value: number;
  unit: string;
  step?: number;
  min?: number;
  max?: number;
  onChange: (value: number) => void;
}) {
  return (
    <label className="field">
      <span>{label}</span>
      <span className="input-wrap">
        <input
          type="number"
          value={Number.isFinite(value) ? value : ""}
          step={step}
          min={min}
          max={max}
          onChange={(event) => onChange(Number(event.target.value))}
        />
        <small>{unit}</small>
      </span>
    </label>
  );
}

function RailLimitInput({
  label,
  value,
  min,
  max,
  onChange,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  onChange: (value: number) => void;
}) {
  const [draft, setDraft] = useState(String(value));

  const commit = () => {
    const next = Number(draft);
    if (draft.trim() === "" || !Number.isFinite(next)) {
      setDraft(String(value));
      return;
    }
    const committed = clamp(next, min, max);
    setDraft(String(committed));
    onChange(committed);
  };

  return (
    <label className="rail-limit">
      <span className="rail-limit-label">{label}</span>
      <span className="rail-limit-input">
        <input
          type="number"
          value={draft}
          step={0.5}
          min={min}
          max={max}
          inputMode="decimal"
          aria-label={`${label} angle in degrees`}
          onChange={(event) => {
            const nextDraft = event.target.value;
            const next = Number(nextDraft);
            setDraft(nextDraft);
            if (nextDraft.trim() !== "" && Number.isFinite(next) && next >= min && next <= max) onChange(next);
          }}
          onBlur={commit}
          onKeyDown={(event) => {
            if (event.key === "Enter") event.currentTarget.blur();
            if (event.key === "Escape") {
              setDraft(String(value));
              event.currentTarget.blur();
            }
          }}
        />
        <small>°</small>
      </span>
    </label>
  );
}

function Toggle({ label, checked, onChange }: { label: string; checked: boolean; onChange: (value: boolean) => void }) {
  return (
    <label className="toggle-field">
      <span>{label}</span>
      <input type="checkbox" checked={checked} onChange={(event) => onChange(event.target.checked)} />
    </label>
  );
}

type PlotSeries = {
  label: string;
  color: string;
  values: Array<number | null>;
};

function AnalysisPlot({
  title,
  unit,
  angles,
  series,
  currentAngle,
  fixedRange,
  alertBelow,
}: {
  title: string;
  unit: string;
  angles: number[];
  series: PlotSeries[];
  currentAngle: number;
  fixedRange?: [number, number];
  alertBelow?: number;
}) {
  const [activeIndex, setActiveIndex] = useState<number | null>(null);
  const width = 720;
  const height = 240;
  const margin = { left: 48, right: 16, top: 34, bottom: 32 };
  const allValues = series.flatMap((item) => item.values).filter((value): value is number => value !== null && Number.isFinite(value));
  let minValue = fixedRange?.[0] ?? Math.min(0, ...allValues);
  let maxValue = fixedRange?.[1] ?? Math.max(0, ...allValues);
  if (!fixedRange) {
    const span = Math.max(1e-6, maxValue - minValue);
    minValue -= span * 0.09;
    maxValue += span * 0.09;
  }
  const plotWidth = width - margin.left - margin.right;
  const plotHeight = height - margin.top - margin.bottom;
  const angleMin = angles.length ? Math.min(...angles) : 0;
  const angleMax = angles.length ? Math.max(...angles) : 360;
  const angleSpan = Math.max(1e-7, angleMax - angleMin);
  const x = (angle: number) => margin.left + (angle - angleMin) / angleSpan * plotWidth;
  const y = (value: number) => margin.top + (maxValue - value) / (maxValue - minValue || 1) * plotHeight;
  const pathFor = (values: Array<number | null>) => {
    let path = "";
    let active = false;
    values.forEach((value, index) => {
      if (value === null || !Number.isFinite(value)) {
        active = false;
        return;
      }
      path += `${active ? "L" : "M"}${x(angles[index]).toFixed(2)},${y(value).toFixed(2)} `;
      active = true;
    });
    return path.trim();
  };
  const yTicks = Array.from({ length: 5 }, (_, index) => minValue + (maxValue - minValue) * index / 4);
  const xTicks = Array.from({ length: 5 }, (_, index) => angleMin + angleSpan * index / 4);
  const clampedCurrentAngle = clamp(currentAngle, angleMin, angleMax);
  const currentIndex = angles.length
    ? angles.reduce((nearest, angle, index) => (
        Math.abs(angle - clampedCurrentAngle) < Math.abs(angles[nearest] - clampedCurrentAngle) ? index : nearest
      ), 0)
    : 0;
  const selectedIndex = activeIndex ?? currentIndex;
  const selectedAngle = angles[selectedIndex] ?? clampedCurrentAngle;
  const selectedValues = series.map((item) => ({ label: item.label, color: item.color, value: item.values[selectedIndex] }));
  const formatAngleTick = (angle: number) => Math.abs(angle - Math.round(angle)) < 0.05 ? angle.toFixed(0) : angle.toFixed(1);

  const selectFromPointer = (clientX: number, target: SVGRectElement) => {
    const bounds = target.getBoundingClientRect();
    const svgX = (clientX - bounds.left) / Math.max(1, bounds.width) * width;
    const candidateAngle = clamp(angleMin + (svgX - margin.left) / plotWidth * angleSpan, angleMin, angleMax);
    const nearest = angles.reduce((best, angle, index) => (
      Math.abs(angle - candidateAngle) < Math.abs(angles[best] - candidateAngle) ? index : best
    ), 0);
    setActiveIndex(nearest);
  };

  return (
    <article className="plot-card">
      <div className="plot-heading">
        <div>
          <h3>{title}</h3>
          <span>{unit}</span>
        </div>
        <div className="plot-legend">
          {series.map((item) => <span key={item.label}><i style={{ background: item.color }}></i>{item.label}</span>)}
        </div>
      </div>
      <div className="plot-readout" aria-live="polite">
        <strong>θ₂ {selectedAngle.toFixed(0)}°</strong>
        {selectedValues.map((item) => (
          <span key={item.label}>
            <i style={{ background: item.color }}></i>
            {item.label} <b>{item.value === null || item.value === undefined ? "—" : item.value.toFixed(2)}</b>
          </span>
        ))}
      </div>
      <div className="plot-scroll">
        <svg
          className="plot-svg"
          viewBox={`0 0 ${width} ${height}`}
          role="img"
          aria-label={`${title} from ${formatAngleTick(angleMin)} to ${formatAngleTick(angleMax)} degrees. Move a pointer across the graph or use the left and right arrow keys to inspect values by crank angle.`}
        >
          {alertBelow !== undefined && alertBelow > minValue ? (
            <rect
              x={margin.left}
              y={y(Math.min(alertBelow, maxValue))}
              width={plotWidth}
              height={Math.max(0, y(minValue) - y(Math.min(alertBelow, maxValue)))}
              className="plot-alert-band"
            />
          ) : null}
          {yTicks.map((tick) => (
            <g key={tick}>
              <line x1={margin.left} y1={y(tick)} x2={width - margin.right} y2={y(tick)} className="plot-grid-line" />
              <text x={margin.left - 8} y={y(tick) + 4} textAnchor="end" className="plot-tick">{tick.toFixed(Math.abs(tick) < 10 ? 1 : 0)}</text>
            </g>
          ))}
          {xTicks.map((tick) => (
            <g key={tick}>
              <line x1={x(tick)} y1={margin.top} x2={x(tick)} y2={height - margin.bottom} className="plot-grid-line vertical" />
              <text x={x(tick)} y={height - 10} textAnchor="middle" className="plot-tick">{formatAngleTick(tick)}°</text>
            </g>
          ))}
          {series.map((item) => (
            <path key={item.label} d={pathFor(item.values)} fill="none" stroke={item.color} className="plot-line" />
          ))}
          <line x1={x(selectedAngle)} y1={margin.top} x2={x(selectedAngle)} y2={height - margin.bottom} className="plot-cursor" />
          {selectedValues.map((item) => item.value !== null && item.value !== undefined ? (
            <circle key={item.label} cx={x(selectedAngle)} cy={y(item.value)} r="4.5" fill={item.color} className="plot-marker" />
          ) : null)}
          <rect
            x={margin.left}
            y={margin.top}
            width={plotWidth}
            height={plotHeight}
            className="plot-hit-area"
            tabIndex={0}
            role="slider"
            aria-label={`Inspect ${title} by crank angle`}
            aria-valuemin={angleMin}
            aria-valuemax={angleMax}
            aria-valuenow={selectedAngle}
            onPointerMove={(event) => selectFromPointer(event.clientX, event.currentTarget)}
            onPointerDown={(event) => selectFromPointer(event.clientX, event.currentTarget)}
            onPointerLeave={() => setActiveIndex(null)}
            onFocus={() => setActiveIndex(currentIndex)}
            onBlur={() => setActiveIndex(null)}
            onKeyDown={(event) => {
              if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") return;
              event.preventDefault();
              const direction = event.key === "ArrowRight" ? 1 : -1;
              setActiveIndex((current) => Math.max(0, Math.min(angles.length - 1, (current ?? currentIndex) + direction)));
            }}
          />
        </svg>
      </div>
    </article>
  );
}

export default function Home() {
  const [config, setConfig] = useState<Config>(DEFAULT_CONFIG);
  const [playing, setPlaying] = useState(false);
  const [activeControlTab, setActiveControlTab] = useState<ControlTab>("geometry");
  const [compactCanvas, setCompactCanvas] = useState(false);
  const [presetRevision, setPresetRevision] = useState(0);
  const lastFrame = useRef<number | null>(null);
  const motionDirection = useRef<1 | -1>(1);

  const update = <K extends keyof Config>(key: K, value: Config[K]) => {
    setConfig((current) => ({ ...current, [key]: value }));
  };

  const updateMinAngle = (value: number) => {
    setConfig((current) => {
      const minAngle = clamp(value, 0, current.maxAngle - 0.5);
      motionDirection.current = 1;
      return { ...current, minAngle, angle: clamp(current.angle, minAngle, current.maxAngle) };
    });
  };

  const updateMaxAngle = (value: number) => {
    setConfig((current) => {
      const maxAngle = clamp(value, current.minAngle + 0.5, 360);
      motionDirection.current = 1;
      return { ...current, maxAngle, angle: clamp(current.angle, current.minAngle, maxAngle) };
    });
  };

  useEffect(() => {
    const query = window.matchMedia("(max-width: 720px)");
    const syncCanvas = () => setCompactCanvas(query.matches);
    syncCanvas();
    query.addEventListener("change", syncCanvas);
    return () => query.removeEventListener("change", syncCanvas);
  }, []);

  useEffect(() => {
    if (!playing) {
      lastFrame.current = null;
      return;
    }

    let frame = 0;
    const animate = (time: number) => {
      if (lastFrame.current !== null) {
        const elapsedSeconds = Math.min(0.05, (time - lastFrame.current) / 1000);
        setConfig((current) => {
          const range = current.maxAngle - current.minAngle;
          if (range >= 359.5) {
            return { ...current, angle: wrapDegrees(current.angle + current.rpm * 6 * elapsedSeconds) };
          }

          const travel = motionDirection.current > 0
            ? current.angle - current.minAngle
            : range + current.maxAngle - current.angle;
          const phase = (travel + Math.abs(current.rpm) * 6 * elapsedSeconds) % (range * 2);
          if (phase <= range) {
            motionDirection.current = 1;
            return { ...current, angle: current.minAngle + phase };
          }
          motionDirection.current = -1;
          return { ...current, angle: current.maxAngle - (phase - range) };
        });
      }
      lastFrame.current = time;
      frame = requestAnimationFrame(animate);
    };
    frame = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(frame);
  }, [playing]);

  const pose = useMemo(() => solvePose(config), [config]);
  const velocity = useMemo(() => kinematics(config, pose), [config, pose]);
  const classification = useMemo(() => mechanismClass(config), [config]);

  const geometryConfig = useMemo<Config>(() => ({
    ...DEFAULT_CONFIG,
    groundX: config.groundX,
    groundY: config.groundY,
    crank: config.crank,
    coupler: config.coupler,
    rocker: config.rocker,
    toolAlong: config.toolAlong,
    toolOffset: config.toolOffset,
    branch: config.branch,
    angle: 0,
  }), [config.groundX, config.groundY, config.crank, config.coupler, config.rocker, config.toolAlong, config.toolOffset, config.branch]);

  const analysisConfig = useMemo<Config>(() => ({
    groundX: config.groundX,
    groundY: config.groundY,
    crank: config.crank,
    coupler: config.coupler,
    rocker: config.rocker,
    toolAlong: config.toolAlong,
    toolOffset: config.toolOffset,
    angle: 0,
    minAngle: config.minAngle,
    maxAngle: config.maxAngle,
    rpm: config.rpm,
    inputAccel: config.inputAccel,
    branch: config.branch,
    forceX: config.forceX,
    forceY: config.forceY,
    gravity: config.gravity,
    crankMass: config.crankMass,
    legMass: config.legMass,
    rockerMass: config.rockerMass,
    toolMass: config.toolMass,
    pinDiameter: config.pinDiameter,
    linkThickness: config.linkThickness,
    shearPlanes: config.shearPlanes,
    allowableShear: config.allowableShear,
    allowableBearing: config.allowableBearing,
    gearRatio: config.gearRatio,
    gearEfficiency: config.gearEfficiency,
    motorContinuous: config.motorContinuous,
    motorPeak: config.motorPeak,
  }), [
    config.groundX, config.groundY, config.crank, config.coupler, config.rocker,
    config.toolAlong, config.toolOffset, config.minAngle, config.maxAngle, config.rpm, config.inputAccel, config.branch,
    config.forceX, config.forceY, config.gravity, config.crankMass, config.legMass,
    config.rockerMass, config.toolMass, config.pinDiameter, config.linkThickness,
    config.shearPlanes, config.allowableShear, config.allowableBearing,
    config.gearRatio, config.gearEfficiency, config.motorContinuous, config.motorPeak,
  ]);

  const currentDynamics = useMemo(() => dynamicsBreakdown(config), [config]);
  const fullAnalysisCycle = config.maxAngle - config.minAngle >= 359.5;
  const analysisWindowName = fullAnalysisCycle ? "cycle" : "motion window";

  const cycleAnalysis = useMemo<CycleSample[]>(() => {
    return sampleAngleRange(analysisConfig.minAngle, analysisConfig.maxAngle, 5).map((angle) => {
      const samplePose = solvePose(analysisConfig, angle);
      if (!samplePose) {
        return { angle, pose: null, dynamics: null, externalTorque: null, gravityTorque: null, inertiaTorque: null };
      }
      const breakdown = dynamicsBreakdown(analysisConfig, angle);
      return {
        angle,
        pose: samplePose,
        dynamics: breakdown.total,
        externalTorque: breakdown.external?.torque ?? null,
        gravityTorque: breakdown.gravity?.torque ?? null,
        inertiaTorque: breakdown.inertia?.torque ?? null,
      };
    });
  }, [analysisConfig]);

  const loadSummary = useMemo(() => {
    const valid = cycleAnalysis.filter((sample) => sample.pose && sample.dynamics) as Array<CycleSample & { pose: Pose; dynamics: Dynamics }>;
    const peakTorque = valid.length ? Math.max(...valid.map((sample) => Math.abs(sample.dynamics.torque))) : 0;
    const peakJointForce = valid.length ? Math.max(...valid.flatMap((sample) => [
      magnitude(sample.dynamics.O2Reaction),
      magnitude(sample.dynamics.AReaction),
      magnitude(sample.dynamics.BReaction),
      magnitude(sample.dynamics.O4Reaction),
    ])) : 0;
    const minTransmission = valid.length ? Math.min(...valid.map((sample) => sample.pose.transmission)) : 0;
    const efficiency = Math.max(0.01, config.gearEfficiency / 100);
    const ratio = Math.max(0.01, config.gearRatio);
    const currentMotorTorque = Math.abs(currentDynamics.total?.torque ?? 0) / (ratio * efficiency);
    const peakMotorTorque = peakTorque / (ratio * efficiency);
    const pinArea = Math.PI * Math.max(0.01, config.pinDiameter) ** 2 / 4;
    const shearStress = peakJointForce / (Math.max(1, config.shearPlanes) * pinArea);
    const bearingStress = peakJointForce / (Math.max(0.01, config.pinDiameter) * Math.max(0.01, config.linkThickness));
    return {
      valid,
      peakTorque,
      peakJointForce,
      minTransmission,
      currentMotorTorque,
      peakMotorTorque,
      shearStress,
      bearingStress,
      shearSafety: shearStress > EPS ? config.allowableShear / shearStress : Infinity,
      bearingSafety: bearingStress > EPS ? config.allowableBearing / bearingStress : Infinity,
      continuousUse: config.motorContinuous > EPS ? peakMotorTorque / config.motorContinuous * 100 : Infinity,
      peakUse: config.motorPeak > EPS ? peakMotorTorque / config.motorPeak * 100 : Infinity,
    };
  }, [cycleAnalysis, config.gearEfficiency, config.gearRatio, config.pinDiameter, config.shearPlanes, config.linkThickness, config.allowableShear, config.allowableBearing, config.motorContinuous, config.motorPeak, currentDynamics.total]);

  const cycle = useMemo(() => {
    return Array.from({ length: 181 }, (_, index) => {
      const angle = index * 2;
      return { angle, pose: solvePose(geometryConfig, angle) };
    });
  }, [geometryConfig]);

  const motionCycle = useMemo(() => {
    return sampleAngleRange(config.minAngle, config.maxAngle, 2).map((angle) => ({
      angle,
      pose: solvePose(geometryConfig, angle),
    }));
  }, [geometryConfig, config.minAngle, config.maxAngle]);

  const reachable = cycle.filter((sample) => sample.pose);
  const coverage = reachable.length / cycle.length;
  const pathBounds = useMemo(() => {
    if (!reachable.length) return null;
    const xs = reachable.map((sample) => sample.pose!.T.x);
    const ys = reachable.map((sample) => sample.pose!.T.y);
    return {
      minX: Math.min(...xs),
      maxX: Math.max(...xs),
      minY: Math.min(...ys),
      maxY: Math.max(...ys),
    };
  }, [reachable]);

  const viewport = useMemo(() => {
    const points: Vec[] = [{ x: 0, y: 0 }, { x: config.groundX, y: config.groundY }];
    reachable.forEach((sample) => {
      if (sample.pose) points.push(sample.pose.A, sample.pose.B, sample.pose.T);
    });
    const xs = points.map((point) => point.x);
    const ys = points.map((point) => point.y);
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);
    const spanX = Math.max(50, maxX - minX);
    const spanY = Math.max(50, maxY - minY);
    const width = compactCanvas ? 620 : 920;
    const height = compactCanvas ? 760 : 610;
    const padding = compactCanvas ? 54 : 72;
    const scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY);
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;
    const map = (point: Vec) => ({
      x: width / 2 + (point.x - centerX) * scale,
      y: height / 2 - (point.y - centerY) * scale,
    });
    return { width, height, scale, map };
  }, [reachable, config.groundX, config.groundY, compactCanvas]);

  const pathData = useMemo(() => {
    let drawing = "";
    let previousReachable = false;
    cycle.forEach((sample) => {
      if (!sample.pose) {
        previousReachable = false;
        return;
      }
      const point = viewport.map(sample.pose.T);
      drawing += `${previousReachable ? "L" : "M"}${point.x.toFixed(2)},${point.y.toFixed(2)} `;
      previousReachable = true;
    });
    return drawing.trim();
  }, [cycle, viewport]);

  const motionPathData = useMemo(() => {
    if (fullAnalysisCycle) return "";
    let drawing = "";
    let previousReachable = false;
    motionCycle.forEach((sample) => {
      if (!sample.pose) {
        previousReachable = false;
        return;
      }
      const point = viewport.map(sample.pose.T);
      drawing += `${previousReachable ? "L" : "M"}${point.x.toFixed(2)},${point.y.toFixed(2)} `;
      previousReachable = true;
    });
    return drawing.trim();
  }, [motionCycle, viewport, fullAnalysisCycle]);

  const mapped = pose
    ? {
        O2: viewport.map(pose.O2),
        O4: viewport.map(pose.O4),
        A: viewport.map(pose.A),
        B: viewport.map(pose.B),
        T: viewport.map(pose.T),
      }
    : null;

  const forceMagnitude = Math.hypot(config.forceX, config.forceY);
  const forceEndpoint = pose && forceMagnitude > EPS
    ? add(pose.T, mul({ x: config.forceX, y: config.forceY }, 36 / forceMagnitude))
    : null;
  const mappedForceEndpoint = forceEndpoint ? viewport.map(forceEndpoint) : null;

  const downloadCycleCsv = () => {
    const header = [
      "angle_deg", "reachable", "tool_x_mm", "tool_y_mm", "transmission_deg",
      "link_torque_Nm", "external_torque_Nm", "gravity_torque_Nm", "inertia_torque_Nm",
      "O2_reaction_N", "A_reaction_N", "B_reaction_N", "O4_reaction_N",
    ];
    const rows = cycleAnalysis.map((sample) => {
      if (!sample.pose || !sample.dynamics) return [sample.angle, false, "", "", "", "", "", "", "", "", "", "", ""];
      return [
        sample.angle,
        true,
        sample.pose.T.x.toFixed(6),
        sample.pose.T.y.toFixed(6),
        sample.pose.transmission.toFixed(6),
        sample.dynamics.torque.toFixed(6),
        sample.externalTorque?.toFixed(6) ?? "",
        sample.gravityTorque?.toFixed(6) ?? "",
        sample.inertiaTorque?.toFixed(6) ?? "",
        magnitude(sample.dynamics.O2Reaction).toFixed(6),
        magnitude(sample.dynamics.AReaction).toFixed(6),
        magnitude(sample.dynamics.BReaction).toFixed(6),
        magnitude(sample.dynamics.O4Reaction).toFixed(6),
      ];
    });
    const csv = [header, ...rows].map((row) => row.join(",")).join("\n");
    const url = URL.createObjectURL(new Blob([csv], { type: "text/csv;charset=utf-8" }));
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = `robot-leg-linkage-${fullAnalysisCycle ? "cycle" : "motion-window"}.csv`;
    document.body.appendChild(anchor);
    anchor.click();
    anchor.remove();
    window.setTimeout(() => URL.revokeObjectURL(url), 1000);
  };

  const statusTone = !pose ? "danger" : pose.transmission < 20 ? "danger" : pose.transmission < 35 ? "warning" : "good";

  return (
    <main className="site-shell">
      <header className="topbar">
        <div className="brand-lockup">
          <p className="eyebrow">Planar mechanism workbench</p>
          <h1>Robot Leg Linkage Lab</h1>
        </div>
        <div className="header-status" aria-label="Mechanism status">
          <span className={`status-dot ${statusTone}`}></span>
          <span>{pose ? "Assembly solved" : "Unreachable pose"}</span>
          <span className="separator">/</span>
          <span>{classification.type}</span>
        </div>
      </header>

      <nav className="mobile-jump" aria-label="Page sections">
        <a href="#simulator">Model</a>
        <a href="#inputs">Inputs</a>
        <a href="#results">Results</a>
      </nav>

      <div className="workspace">
        <aside id="inputs" className="control-panel" aria-label="Linkage configuration">
          <div className="panel-heading">
            <div>
              <span className="panel-index">01</span>
              <div>
                <h2>Inputs</h2>
                <p>Configure one group at a time</p>
              </div>
            </div>
            <button
              className="text-button"
              type="button"
              onClick={() => {
                setConfig(DEFAULT_CONFIG);
                setPlaying(false);
                setActiveControlTab("geometry");
                setPresetRevision((current) => current + 1);
                motionDirection.current = 1;
              }}
            >
              Reset preset
            </button>
          </div>

          <div className="control-tabs" role="tablist" aria-label="Input groups">
            {([
              ["geometry", "Geometry", "Links & motion"],
              ["loads", "Loads", "Forces & mass"],
              ["hardware", "Hardware", "Motor & pins"],
            ] as Array<[ControlTab, string, string]>).map(([value, label, description]) => (
              <button
                key={value}
                id={`${value}-tab`}
                type="button"
                role="tab"
                aria-selected={activeControlTab === value}
                aria-controls={`${value}-panel`}
                tabIndex={activeControlTab === value ? 0 : -1}
                className={activeControlTab === value ? "active" : ""}
                onClick={() => setActiveControlTab(value)}
                onKeyDown={(event) => {
                  if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") return;
                  event.preventDefault();
                  const order: ControlTab[] = ["geometry", "loads", "hardware"];
                  const direction = event.key === "ArrowRight" ? 1 : -1;
                  const next = order[(order.indexOf(value) + direction + order.length) % order.length];
                  setActiveControlTab(next);
                  window.requestAnimationFrame(() => document.getElementById(`${next}-tab`)?.focus());
                }}
              >
                <span>{label}</span>
                <small>{description}</small>
              </button>
            ))}
          </div>

          {activeControlTab === "geometry" ? (
            <div id="geometry-panel" role="tabpanel" aria-labelledby="geometry-tab" className="control-tab-panel">
              <section className="control-section tab-section">
                <div className="section-label">
                  <h3>Four-bar geometry</h3>
                  <span>millimetres</span>
                </div>
                <div className="field-grid">
                  <NumericInput label="Ground Δx" value={config.groundX} unit="mm" step={1} onChange={(value) => update("groundX", value)} />
                  <NumericInput label="Ground Δy" value={config.groundY} unit="mm" step={1} onChange={(value) => update("groundY", value)} />
                  <NumericInput label="Input crank" value={config.crank} unit="mm" min={0.1} onChange={(value) => update("crank", value)} />
                  <NumericInput label="Coupler A–B" value={config.coupler} unit="mm" min={0.1} onChange={(value) => update("coupler", value)} />
                  <NumericInput label="Output rocker" value={config.rocker} unit="mm" min={0.1} onChange={(value) => update("rocker", value)} />
                  <NumericInput label="Tool along A–B" value={config.toolAlong} unit="mm" step={1} onChange={(value) => update("toolAlong", value)} />
                  <NumericInput label="Tool offset" value={config.toolOffset} unit="mm" step={1} onChange={(value) => update("toolOffset", value)} />
                  <label className="field">
                    <span>Assembly branch</span>
                    <span className="input-wrap select-wrap">
                      <select value={config.branch} onChange={(event) => update("branch", Number(event.target.value) as Branch)}>
                        <option value={-1}>Lower / drawing</option>
                        <option value={1}>Upper / alternate</option>
                      </select>
                    </span>
                  </label>
                </div>
              </section>
              <section className="control-section compact-section">
                <div className="section-label">
                  <h3>Motion</h3>
                  <span>animation window</span>
                </div>
                <div className="field-grid">
                  <NumericInput label="Input speed" value={config.rpm} unit="rpm" step={5} min={0} onChange={(value) => update("rpm", value)} />
                </div>
                <p className="control-hint">Set the minimum and maximum directly on the right-hand scrubber. Partial ranges reverse smoothly at each endpoint.</p>
              </section>
            </div>
          ) : null}

          {activeControlTab === "loads" ? (
            <div id="loads-panel" role="tabpanel" aria-labelledby="loads-tab" className="control-tab-panel">
              <section className="control-section tab-section">
                <div className="section-label">
                  <h3>External load</h3>
                  <span>at tool point T</span>
                </div>
                <div className="field-grid">
                  <NumericInput label="Tool force Fx" value={config.forceX} unit="N" step={10} onChange={(value) => update("forceX", value)} />
                  <NumericInput label="Tool force Fy" value={config.forceY} unit="N" step={10} onChange={(value) => update("forceY", value)} />
                  <NumericInput label="Input acceleration" value={config.inputAccel} unit="rad/s²" step={1} onChange={(value) => update("inputAccel", value)} />
                  <Toggle label="Gravity −y" checked={config.gravity} onChange={(value) => update("gravity", value)} />
                </div>
              </section>
              <section className="control-section compact-section">
                <div className="section-label">
                  <h3>Body model</h3>
                  <span>uniform links</span>
                </div>
                <div className="field-grid">
                  <NumericInput label="Crank mass" value={config.crankMass} unit="kg" step={0.01} min={0} onChange={(value) => update("crankMass", value)} />
                  <NumericInput label="Extended leg mass" value={config.legMass} unit="kg" step={0.01} min={0} onChange={(value) => update("legMass", value)} />
                  <NumericInput label="Rocker mass" value={config.rockerMass} unit="kg" step={0.01} min={0} onChange={(value) => update("rockerMass", value)} />
                  <NumericInput label="Tool / wheel mass" value={config.toolMass} unit="kg" step={0.01} min={0} onChange={(value) => update("toolMass", value)} />
                </div>
              </section>
            </div>
          ) : null}

          {activeControlTab === "hardware" ? (
            <div id="hardware-panel" role="tabpanel" aria-labelledby="hardware-tab" className="control-tab-panel">
              <section className="control-section tab-section">
                <div className="section-label">
                  <h3>Actuator</h3>
                  <span>motor side</span>
                </div>
                <div className="field-grid">
                  <NumericInput label="Gear ratio" value={config.gearRatio} unit=":1" step={0.5} min={0.01} onChange={(value) => update("gearRatio", value)} />
                  <NumericInput label="Gear efficiency" value={config.gearEfficiency} unit="%" step={1} min={1} onChange={(value) => update("gearEfficiency", value)} />
                  <NumericInput label="Motor continuous" value={config.motorContinuous} unit="N·m" step={0.1} min={0.01} onChange={(value) => update("motorContinuous", value)} />
                  <NumericInput label="Motor peak" value={config.motorPeak} unit="N·m" step={0.1} min={0.01} onChange={(value) => update("motorPeak", value)} />
                </div>
              </section>
              <section className="control-section compact-section">
                <div className="section-label">
                  <h3>Joint screening</h3>
                  <span>nominal stress</span>
                </div>
                <div className="field-grid">
                  <NumericInput label="Pin diameter" value={config.pinDiameter} unit="mm" step={0.5} min={0.1} onChange={(value) => update("pinDiameter", value)} />
                  <NumericInput label="Link thickness" value={config.linkThickness} unit="mm" step={0.5} min={0.1} onChange={(value) => update("linkThickness", value)} />
                  <NumericInput label="Shear planes" value={config.shearPlanes} unit="count" step={1} min={1} onChange={(value) => update("shearPlanes", value)} />
                  <NumericInput label="Allowable pin shear" value={config.allowableShear} unit="MPa" step={10} min={1} onChange={(value) => update("allowableShear", value)} />
                  <NumericInput label="Allowable bearing" value={config.allowableBearing} unit="MPa" step={10} min={1} onChange={(value) => update("allowableBearing", value)} />
                </div>
              </section>
            </div>
          ) : null}

          <section className="mechanism-summary" aria-label="Mechanism summary">
            <div>
              <span>Ground length</span>
              <strong>{classification.ground.toFixed(2)} mm</strong>
            </div>
            <div>
              <span>Grashof margin</span>
              <strong className={classification.margin < 0 ? "value-danger" : ""}>{classification.margin.toFixed(2)} mm</strong>
            </div>
            <div>
              <span>Full rotation</span>
              <strong>{classification.inputRotates && coverage > 0.99 ? "Yes" : "No"}</strong>
            </div>
            <div>
              <span>Reachable cycle</span>
              <strong>{(coverage * 100).toFixed(0)}%</strong>
            </div>
          </section>
        </aside>

        <section id="simulator" className="analysis-area">
          <article className="simulator-card">
            <div className="card-heading">
              <div>
                <span className="panel-index">02</span>
                <div>
                  <h2>Kinematic model</h2>
                  <p>O₂ origin · +y upward · dimensions in millimetres</p>
                </div>
              </div>
              <div className="legend" aria-label="Link legend">
                <span><i className="swatch crank"></i>Crank</span>
                <span><i className="swatch coupler"></i>Leg</span>
                <span><i className="swatch rocker"></i>Rocker</span>
                <span><i className="swatch path"></i>Full tool path</span>
                {!fullAnalysisCycle ? <span><i className="swatch motion"></i>Motion window</span> : null}
              </div>
            </div>

            <div className="simulator-body">
              <div className="simulator-stage">
              <svg viewBox={`0 0 ${viewport.width} ${viewport.height}`} role="img" aria-label="Interactive four-bar robot leg linkage">
                <defs>
                  <pattern id="grid" width="32" height="32" patternUnits="userSpaceOnUse">
                    <path d="M 32 0 L 0 0 0 32" className="grid-line" />
                  </pattern>
                  <filter id="joint-glow" x="-100%" y="-100%" width="300%" height="300%">
                    <feGaussianBlur stdDeviation="3" result="blur" />
                    <feMerge><feMergeNode in="blur" /><feMergeNode in="SourceGraphic" /></feMerge>
                  </filter>
                  <marker id="force-arrow" markerWidth="10" markerHeight="10" refX="8" refY="3" orient="auto" markerUnits="strokeWidth">
                    <path d="M0,0 L0,6 L9,3 z" className="force-arrow-head" />
                  </marker>
                </defs>
                <rect width="100%" height="100%" fill="url(#grid)" />
                <path d={pathData} className="tool-path" />
                {motionPathData ? <path d={motionPathData} className="motion-path" /> : null}

                {mapped && pose ? (
                  <>
                    <line x1={mapped.O2.x} y1={mapped.O2.y} x2={mapped.O4.x} y2={mapped.O4.y} className="ground-link" />
                    <circle cx={mapped.O2.x} cy={mapped.O2.y} r={Math.max(12, 11 * viewport.scale)} className="bearing-ring" />
                    <circle cx={mapped.O2.x} cy={mapped.O2.y} r={Math.max(5, 4 * viewport.scale)} className="bearing-bore" />
                    <circle cx={mapped.O4.x} cy={mapped.O4.y} r={Math.max(18, 18.5 * viewport.scale)} className="bearing-ring" />
                    <circle cx={mapped.O4.x} cy={mapped.O4.y} r={Math.max(9, 12.5 * viewport.scale)} className="bearing-bore" />
                    <line x1={mapped.O2.x} y1={mapped.O2.y} x2={mapped.A.x} y2={mapped.A.y} className="link crank-link" />
                    <line x1={mapped.A.x} y1={mapped.A.y} x2={mapped.T.x} y2={mapped.T.y} className="link coupler-link" />
                    <line x1={mapped.O4.x} y1={mapped.O4.y} x2={mapped.B.x} y2={mapped.B.y} className="link rocker-link" />
                    <circle cx={mapped.O2.x} cy={mapped.O2.y} r="8" className="fixed-joint" />
                    <circle cx={mapped.O4.x} cy={mapped.O4.y} r="8" className="fixed-joint" />
                    <circle cx={mapped.A.x} cy={mapped.A.y} r="8" className="moving-joint" />
                    <circle cx={mapped.B.x} cy={mapped.B.y} r="8" className="moving-joint" />
                    <circle cx={mapped.T.x} cy={mapped.T.y} r="10" className="tool-joint" filter="url(#joint-glow)" />
                    {mappedForceEndpoint ? (
                      <g className="force-vector">
                        <line x1={mapped.T.x} y1={mapped.T.y} x2={mappedForceEndpoint.x} y2={mappedForceEndpoint.y} markerEnd="url(#force-arrow)" />
                        <text x={mappedForceEndpoint.x + 10} y={mappedForceEndpoint.y - 8}>{forceMagnitude.toFixed(0)} N</text>
                      </g>
                    ) : null}
                    <g className="joint-labels">
                      <text x={mapped.O2.x + 13} y={mapped.O2.y - 14}>O₂</text>
                      <text x={mapped.A.x - 23} y={mapped.A.y - 13}>A</text>
                      <text x={mapped.B.x + 13} y={mapped.B.y - 13}>B</text>
                      <text x={mapped.O4.x + 13} y={mapped.O4.y - 14}>O₄</text>
                      <text x={mapped.T.x + 14} y={mapped.T.y - 13}>T</text>
                    </g>
                  </>
                ) : (
                  <g className="empty-state">
                    <circle cx="460" cy="280" r="38" />
                    <path d="M440 280h40M460 260v40" />
                    <text x="460" y="348" textAnchor="middle">No circle intersection at θ₂ = {config.angle.toFixed(1)}°</text>
                    <text x="460" y="374" textAnchor="middle">Adjust the link lengths, branch, or input angle.</text>
                  </g>
                )}
              </svg>
                <div className="stage-coordinate">θ₂ {config.angle.toFixed(1)}°</div>
              </div>

              <div className="motion-rail" aria-label="Animation controls">
                <div className="rail-readout">
                  <span>θ₂</span>
                  <output htmlFor="crank-angle">{config.angle.toFixed(1)}°</output>
                </div>
                <div className="rail-range">
                  <RailLimitInput
                    key={`maximum-angle-${presetRevision}`}
                    label="Max"
                    value={config.maxAngle}
                    min={config.minAngle + 0.5}
                    max={360}
                    onChange={updateMaxAngle}
                  />
                  <input
                    id="crank-angle"
                    className="vertical-angle-slider"
                    type="range"
                    min={config.minAngle}
                    max={config.maxAngle}
                    step="0.5"
                    value={config.angle}
                    aria-label={`Crank angle from ${config.minAngle.toFixed(1)} to ${config.maxAngle.toFixed(1)} degrees`}
                    aria-orientation="vertical"
                    onChange={(event) => {
                      setPlaying(false);
                      update("angle", Number(event.target.value));
                    }}
                  />
                  <RailLimitInput
                    key={`minimum-angle-${presetRevision}`}
                    label="Min"
                    value={config.minAngle}
                    min={0}
                    max={config.maxAngle - 0.5}
                    onChange={updateMinAngle}
                  />
                </div>
                <div className="rail-nudges" aria-label="Angle step controls">
                  <button
                    type="button"
                    onClick={() => {
                      setPlaying(false);
                      update("angle", Math.min(config.maxAngle, config.angle + 5));
                    }}
                    aria-label="Increase crank angle by 5 degrees"
                  >+5°</button>
                  <button
                    type="button"
                    onClick={() => {
                      setPlaying(false);
                      update("angle", Math.max(config.minAngle, config.angle - 5));
                    }}
                    aria-label="Decrease crank angle by 5 degrees"
                  >−5°</button>
                </div>
                <button
                  className={`simulator-play ${playing ? "active" : ""}`}
                  type="button"
                  aria-pressed={playing}
                  onClick={() => setPlaying((current) => !current)}
                >
                  <span aria-hidden="true">{playing ? "Ⅱ" : "▶"}</span>
                  {playing ? "Pause" : "Run"}
                </button>
              </div>
            </div>
          </article>

          <section className="metric-grid" aria-label="Current mechanism results">
            <article className={`metric-card ${statusTone === "danger" ? "metric-alert" : ""}`}>
              <span>Transmission angle</span>
              <strong>{pose ? `${pose.transmission.toFixed(2)}°` : "—"}</strong>
              <small>{!pose ? "unreachable" : pose.transmission < 20 ? "critical geometry" : pose.transmission < 35 ? "low" : "usable"}</small>
            </article>
            <article className="metric-card">
              <span>Tool point T</span>
              <strong>{pose ? `${pose.T.x.toFixed(1)}, ${pose.T.y.toFixed(1)}` : "—"}</strong>
              <small>x, y · mm</small>
            </article>
            <article className="metric-card metric-emphasis">
              <span>Required link torque</span>
              <strong>{currentDynamics.total ? `${currentDynamics.total.torque.toFixed(3)} N·m` : "—"}</strong>
              <small>signed at input O₂</small>
            </article>
            <article className={`metric-card ${loadSummary.currentMotorTorque > config.motorContinuous ? "metric-alert" : ""}`}>
              <span>Motor torque now</span>
              <strong>{currentDynamics.total ? `${loadSummary.currentMotorTorque.toFixed(3)} N·m` : "—"}</strong>
              <small>after ratio & efficiency</small>
            </article>
          </section>

          <section className="state-strip" aria-label="Secondary kinematic results">
            <div><span>Output θ₄</span><strong>{pose ? `${wrapDegrees(pose.theta4 * DEG).toFixed(2)}°` : "—"}</strong></div>
            <div><span>Velocity ratio ω₄/ω₂</span><strong>{velocity ? velocity.omega4Ratio.toFixed(3) : "—"}</strong></div>
            <div><span>Tool Jacobian</span><strong>{velocity ? `${velocity.toolDerivative.x.toFixed(1)}, ${velocity.toolDerivative.y.toFixed(1)} mm/rad` : "—"}</strong></div>
            <div><span>Path envelope</span><strong>{pathBounds ? `${(pathBounds.maxX - pathBounds.minX).toFixed(1)} × ${(pathBounds.maxY - pathBounds.minY).toFixed(1)} mm` : "—"}</strong></div>
          </section>

          <p className="analysis-note">
            {fullAnalysisCycle
              ? "The dashed curve is the full reachable path of T."
              : `The solid arc is the configured ${config.minAngle.toFixed(1)}°–${config.maxAngle.toFixed(1)}° motion window; the dashed curve is the full reachable path of T.`}
            {" "}Transmission below 20° is flagged because joint reactions and sensitivity rise rapidly near toggle geometry.
          </p>
        </section>
      </div>

      <section id="results" className="results-area">
        <div className="results-heading">
          <div>
            <span className="panel-index">03</span>
            <div>
              <h2>Load analysis</h2>
              <p>Planar inverse dynamics · prescribed crank motion · force at T</p>
            </div>
          </div>
          <button type="button" className="export-button" onClick={downloadCycleCsv}>Download analysis CSV</button>
        </div>

        <section className="load-metric-grid" aria-label="Peak load results">
          <article className={`load-metric primary-metric ${loadSummary.peakUse > 100 ? "load-warning" : ""}`}>
            <span>Peak {analysisWindowName} torque</span>
            <strong>{loadSummary.valid.length ? `${loadSummary.peakTorque.toFixed(2)} N·m` : "—"}</strong>
            <small>maximum |τ₂| over reachable {analysisWindowName}</small>
          </article>
          <article className="load-metric">
            <span>Peak joint reaction</span>
            <strong>{loadSummary.valid.length ? `${loadSummary.peakJointForce.toFixed(0)} N` : "—"}</strong>
            <small>largest O₂ / A / B / O₄ resultant</small>
          </article>
          <article className={`load-metric ${loadSummary.shearSafety < 1.5 ? "load-warning" : ""}`}>
            <span>Pin shear safety</span>
            <strong>{Number.isFinite(loadSummary.shearSafety) ? `${loadSummary.shearSafety.toFixed(1)}×` : "∞"}</strong>
            <small>{loadSummary.shearStress.toFixed(1)} MPa nominal peak</small>
          </article>
          <article className={`load-metric ${loadSummary.bearingSafety < 1.5 ? "load-warning" : ""}`}>
            <span>Link bearing safety</span>
            <strong>{Number.isFinite(loadSummary.bearingSafety) ? `${loadSummary.bearingSafety.toFixed(1)}×` : "∞"}</strong>
            <small>{loadSummary.bearingStress.toFixed(1)} MPa nominal peak</small>
          </article>
        </section>

        <section className="plot-grid" aria-label={`${fullAnalysisCycle ? "Cycle" : "Motion window"} analysis plots`}>
          <AnalysisPlot
            title={`Input torque over ${analysisWindowName}`}
            unit="N·m at O₂"
            angles={cycleAnalysis.map((sample) => sample.angle)}
            currentAngle={config.angle}
            series={[
              { label: "total", color: "var(--orange)", values: cycleAnalysis.map((sample) => sample.dynamics?.torque ?? null) },
              { label: "tool load", color: "var(--cyan)", values: cycleAnalysis.map((sample) => sample.externalTorque) },
              { label: "gravity + inertia", color: "var(--violet)", values: cycleAnalysis.map((sample) => sample.gravityTorque !== null && sample.inertiaTorque !== null ? sample.gravityTorque + sample.inertiaTorque : null) },
            ]}
          />
          <AnalysisPlot
            title="Transmission angle"
            unit="degrees · red zone below 20°"
            angles={cycleAnalysis.map((sample) => sample.angle)}
            currentAngle={config.angle}
            fixedRange={[0, 90]}
            alertBelow={20}
            series={[
              { label: "μ", color: "var(--lime)", values: cycleAnalysis.map((sample) => sample.pose?.transmission ?? null) },
            ]}
          />
        </section>

        <section className="load-detail-grid">
          <article className="detail-card">
            <div className="detail-heading">
              <div>
                <h3>Current joint reactions</h3>
                <span>signed components in global axes</span>
              </div>
              <span className="angle-chip">θ₂ {config.angle.toFixed(1)}°</span>
            </div>
            <div className="table-wrap">
              <table>
                <thead><tr><th>Joint / body</th><th>Fx</th><th>Fy</th><th>Resultant</th></tr></thead>
                <tbody>
                  {currentDynamics.total ? [
                    ["O₂ ground → crank", currentDynamics.total.O2Reaction],
                    ["A coupler → crank", currentDynamics.total.AReaction],
                    ["B rocker → coupler", currentDynamics.total.BReaction],
                    ["O₄ ground → rocker", currentDynamics.total.O4Reaction],
                  ].map(([label, vector]) => {
                    const item = vector as Vec;
                    return <tr key={label as string}><td>{label as string}</td><td>{item.x.toFixed(1)} N</td><td>{item.y.toFixed(1)} N</td><td><strong>{magnitude(item).toFixed(1)} N</strong></td></tr>;
                  }) : <tr><td colSpan={4}>Current configuration is not dynamically solvable.</td></tr>}
                </tbody>
              </table>
            </div>
            <div className="torque-breakdown">
              <div><span>Tool load</span><strong>{currentDynamics.external?.torque.toFixed(3) ?? "—"} N·m</strong></div>
              <div><span>Gravity</span><strong>{currentDynamics.gravity?.torque.toFixed(3) ?? "—"} N·m</strong></div>
              <div><span>Inertia</span><strong>{currentDynamics.inertia?.torque.toFixed(3) ?? "—"} N·m</strong></div>
            </div>
          </article>

          <article className="detail-card actuator-card">
            <div className="detail-heading">
              <div>
                <h3>Actuator & pin screen</h3>
                <span>peak values from 5° sampling plus endpoints</span>
              </div>
            </div>
            <div className="utilization-block">
              <div className="utilization-label"><span>Continuous motor capacity</span><strong>{Number.isFinite(loadSummary.continuousUse) ? `${loadSummary.continuousUse.toFixed(0)}%` : "—"}</strong></div>
              <div className="utilization-track"><i className={loadSummary.continuousUse > 100 ? "over" : ""} style={{ width: `${Math.min(100, loadSummary.continuousUse)}%` }}></i></div>
              <small>{loadSummary.peakMotorTorque.toFixed(3)} N·m required / {config.motorContinuous.toFixed(2)} N·m configured</small>
            </div>
            <div className="utilization-block">
              <div className="utilization-label"><span>Peak motor capacity</span><strong>{Number.isFinite(loadSummary.peakUse) ? `${loadSummary.peakUse.toFixed(0)}%` : "—"}</strong></div>
              <div className="utilization-track"><i className={loadSummary.peakUse > 100 ? "over" : ""} style={{ width: `${Math.min(100, loadSummary.peakUse)}%` }}></i></div>
              <small>{config.gearRatio.toFixed(2)}:1 ratio · {config.gearEfficiency.toFixed(0)}% efficiency</small>
            </div>
            <div className="screening-grid">
              <div><span>Minimum μ</span><strong className={loadSummary.minTransmission < 20 ? "screen-danger" : ""}>{loadSummary.minTransmission.toFixed(1)}°</strong></div>
              <div><span>Peak pin shear</span><strong>{loadSummary.shearStress.toFixed(1)} MPa</strong></div>
              <div><span>Peak bearing</span><strong>{loadSummary.bearingStress.toFixed(1)} MPa</strong></div>
              <div><span>Window samples</span><strong>{loadSummary.valid.length} / {cycleAnalysis.length}</strong></div>
            </div>
          </article>
        </section>

        <article className="assumption-strip">
          <strong>Model scope</strong>
          <span>Rigid planar links</span>
          <span>Ideal revolute joints</span>
          <span>Constant configured RPM</span>
          <span>Uniform crank, leg and rocker bars</span>
          <span>Tool mass concentrated at T</span>
          <span>Nominal shear and projected bearing stress only</span>
        </article>
      </section>
    </main>
  );
}
