"use client";

import { useEffect, useMemo, useRef, useState } from "react";
import { createPortal } from "react-dom";
import {
  ASCENTO_V1_CONFIG,
  GENERIC_DEMO_CONFIG,
  type Config,
} from "./linkage-ui";
import {
  solvePose,
  timeAtAngle,
} from "../lib/linkage-analysis.mjs";
import {
  classifyScreeningCheck,
  hipQToSolverAngle,
  referenceCom,
  sampleWheelPath,
  solverAngleToHipQ,
  summarizeDesign,
  takeoffMetrics,
  targetDiagonal,
  wheelDerivative,
  wheelPathMetrics,
} from "../lib/robot-workbench.mjs";
import type { DesignSummary } from "../lib/robot-workbench.mjs";

type Mode = "pose" | "animate";
type AnalysisView = "status" | "dynamic" | "static" | "actuator";
type ScreeningState = "critical" | "warning" | "pass" | "neutral";
type ScreeningCheck = {
  index: number;
  category: string;
  label: string;
  detail: string;
  state: ScreeningState;
};
type SavedPreset = {
  name: string;
  config: Config;
  angleConvention: "ascento" | "solver";
};
type Snapshot = {
  name: "A" | "B";
  config: Config;
  summary: DesignSummary;
};
type OverlayPoint = { x: number; y: number };
type OverlayData = {
  wheel: OverlayPoint;
  wheelRadius: number;
  com: OverlayPoint;
  body: string;
  targetPath: string;
  snapshotAPath: string;
  snapshotBPath: string;
  markers: Array<{ x: number; y: number; label: string }>;
};

const PRESET_STORAGE = "robot-leg-linkage:saved-presets-v1";
const SNAPSHOT_STORAGE = "robot-leg-linkage:design-snapshots-v1";
const COM_STORAGE = "robot-leg-linkage:com-reference-v1";
const DETAILS_STORAGE = "robot-leg-linkage:engineering-details-v1";

const numericKeys: Array<keyof Config> = [
  "groundX",
  "groundY",
  "crank",
  "coupler",
  "rocker",
  "toolAlong",
  "toolOffset",
  "minAngle",
  "maxAngle",
  "maxVelocity",
  "maxAcceleration",
  "maxJerk",
  "cycleTime",
  "supportForce",
  "crankMass",
  "legMass",
  "rockerMass",
  "toolMass",
  "pinDiameter",
  "linkThickness",
  "shearPlanes",
  "allowableShear",
  "allowableBearing",
  "gearRatio",
  "gearEfficiency",
  "motorContinuous",
  "motorPeak",
];

const fieldNames: Record<string, { original: string[]; robot: string; math: string }> = {
  groundX: { original: ["Ground Δx"], robot: "Fixed pivots P→H Δx", math: "solver x" },
  groundY: { original: ["Ground Δy"], robot: "Fixed pivots P→H Δy", math: "solver y · robot z = −y" },
  crank: { original: ["Rocker (A-02)"], robot: "Rocker PK", math: "O₂–A" },
  coupler: { original: ["Coupler A–B"], robot: "Coupler IK", math: "A–B" },
  rocker: { original: ["Input crank (B-04)"], robot: "Hip link HI", math: "O₄–B · driven" },
  toolAlong: { original: ["Tool along A–B"], robot: "Wheel carrier K→W", math: "A→T along coupler" },
  toolOffset: { original: ["Tool offset"], robot: "Wheel carrier offset", math: "normal to K→I" },
  supportForce: { original: ["Vertical external load at T"], robot: "Vertical load at wheel W", math: "external support load" },
  crankMass: { original: ["Rocker (A-02) mass"], robot: "Rocker PK mass", math: "O₂–A" },
  legMass: { original: ["Extended leg mass"], robot: "Coupler / leg mass", math: "K–I–W carrier" },
  rockerMass: { original: ["Input crank (B-04) mass"], robot: "Hip link HI mass", math: "O₄–B" },
  toolMass: { original: ["Tool / wheel mass"], robot: "Wheel W mass", math: "tool point T" },
};

const jointText: Record<string, string> = {
  "O₂": "P",
  "O₄": "H",
  A: "K",
  B: "I",
  T: "W",
};

function round(value: number, digits = 2) {
  const factor = 10 ** digits;
  return Math.round(value * factor) / factor;
}

function closeEnough(a: number, b: number, tolerance = 1e-5) {
  return Math.abs(a - b) <= tolerance * Math.max(1, Math.abs(a), Math.abs(b));
}

function configLooksLike(a: Config, b: Config) {
  return ["groundX", "groundY", "crank", "coupler", "rocker", "toolAlong", "toolOffset", "minAngle", "maxAngle"].every(
    (key) => closeEnough(Number(a[key as keyof Config]), Number(b[key as keyof Config]), 1e-3),
  );
}

function setNativeValue(input: HTMLInputElement | HTMLSelectElement, value: string) {
  const prototype = input instanceof HTMLInputElement ? HTMLInputElement.prototype : HTMLSelectElement.prototype;
  const setter = Object.getOwnPropertyDescriptor(prototype, "value")?.set;
  setter?.call(input, value);
  input.dispatchEvent(new Event("input", { bubbles: true }));
  input.dispatchEvent(new Event("change", { bubbles: true }));
}

function fieldInput(key: string) {
  return document.querySelector<HTMLInputElement | HTMLSelectElement>(
    `[data-workbench-key="${key}"] input, [data-workbench-key="${key}"] select`,
  );
}

function tagFieldLabels() {
  for (const label of Array.from(document.querySelectorAll<HTMLLabelElement>("#inputs label.field"))) {
    const first = label.querySelector<HTMLElement>(":scope > span:first-child");
    if (!first) continue;
    const current = first.textContent?.trim() ?? "";
    for (const [key, names] of Object.entries(fieldNames)) {
      const matches = label.dataset.workbenchKey === key || names.original.some((name) => current.startsWith(name)) || current.startsWith(names.robot);
      if (!matches) continue;
      label.dataset.workbenchKey = key;
      if (first.dataset.robotLabel !== key) {
        first.replaceChildren();
        const primary = document.createElement("span");
        primary.textContent = names.robot;
        primary.className = "robot-field-primary";
        const secondary = document.createElement("small");
        secondary.textContent = names.math;
        secondary.className = "robot-field-secondary";
        first.append(primary, secondary);
        first.dataset.robotLabel = key;
      }
      break;
    }
  }

  const motionLabels: Record<string, string[]> = {
    minAngle: ["Minimum angle"],
    maxAngle: ["Maximum angle"],
    maxVelocity: ["Max velocity"],
    maxAcceleration: ["Max acceleration"],
    maxJerk: ["Max jerk"],
    cycleTime: ["Cycle time"],
  };
  for (const label of Array.from(document.querySelectorAll<HTMLLabelElement>("#inputs .motion-profile-section label.field"))) {
    const first = label.querySelector<HTMLElement>(":scope > span:first-child");
    if (!first) continue;
    const current = first.textContent?.trim() ?? "";
    for (const [key, names] of Object.entries(motionLabels)) {
      if (label.dataset.workbenchKey === key || names.some((name) => current.startsWith(name))) {
        label.dataset.workbenchKey = key;
        break;
      }
    }
  }

  const simpleLabels: Record<string, string[]> = {
    pinDiameter: ["Pin diameter"],
    linkThickness: ["Link thickness"],
    shearPlanes: ["Shear planes"],
    allowableShear: ["Allowable pin shear"],
    allowableBearing: ["Allowable bearing"],
    gearRatio: ["Gear ratio"],
    gearEfficiency: ["Gear efficiency"],
    motorContinuous: ["Motor continuous"],
    motorPeak: ["Motor peak"],
  };
  for (const label of Array.from(document.querySelectorAll<HTMLLabelElement>("#inputs label.field"))) {
    if (label.dataset.workbenchKey) continue;
    const first = label.querySelector<HTMLElement>(":scope > span:first-child");
    const current = first?.textContent?.trim() ?? "";
    for (const [key, names] of Object.entries(simpleLabels)) {
      if (names.some((name) => current.startsWith(name))) {
        label.dataset.workbenchKey = key;
        break;
      }
    }
  }
}

function readNumber(key: keyof Config, fallback: number) {
  const input = fieldInput(String(key));
  if (!input) return fallback;
  const value = Number(input.value);
  return Number.isFinite(value) ? value : fallback;
}

function readConfigFromDom(): Config {
  tagFieldLabels();
  const fallback = ASCENTO_V1_CONFIG;
  const config = { ...fallback } as Config;
  for (const key of numericKeys) {
    const base = Number(fallback[key]);
    (config as Record<string, unknown>)[key] = readNumber(key, base);
  }
  const activeProfile = document.querySelector<HTMLButtonElement>(".motion-profile-selector button.active")?.textContent?.toLowerCase();
  config.motionProfile = activeProfile?.includes("sinus") ? "sinusoidal" : "s-curve";
  const branch = document.querySelector<HTMLSelectElement>("#inputs select")?.value;
  if (branch === "1" || branch === "-1") config.branch = Number(branch) as Config["branch"];
  const gravity = document.querySelector<HTMLInputElement>("#inputs .toggle-field input[type=checkbox]");
  if (gravity) config.gravity = gravity.checked;
  return config;
}

async function commitNumericField(key: keyof Config, value: number) {
  tagFieldLabels();
  const input = fieldInput(String(key));
  if (!(input instanceof HTMLInputElement)) return false;
  input.focus();
  setNativeValue(input, String(value));
  await new Promise<void>((resolve) => requestAnimationFrame(() => resolve()));
  input.blur();
  await new Promise<void>((resolve) => requestAnimationFrame(() => resolve()));
  return true;
}

async function applyConfig(config: Config) {
  tagFieldLabels();
  const current = readConfigFromDom();
  const keys = numericKeys.filter((key) => key !== "minAngle" && key !== "maxAngle");
  for (const key of keys) {
    if (Number.isFinite(Number(config[key]))) await commitNumericField(key, Number(config[key]));
  }
  if (config.minAngle <= current.maxAngle) {
    await commitNumericField("minAngle", config.minAngle);
    await commitNumericField("maxAngle", config.maxAngle);
  } else {
    await commitNumericField("maxAngle", config.maxAngle);
    await commitNumericField("minAngle", config.minAngle);
  }
  const profileButtons = Array.from(document.querySelectorAll<HTMLButtonElement>(".motion-profile-selector button"));
  profileButtons.find((button) => button.textContent?.toLowerCase().includes(config.motionProfile === "sinusoidal" ? "sinus" : "s-curve"))?.click();
  const branchSelect = Array.from(document.querySelectorAll<HTMLSelectElement>("#inputs select")).find((select) => Array.from(select.options).some((option) => option.value === "-1"));
  if (branchSelect) setNativeValue(branchSelect, String(config.branch));
  const gravity = document.querySelector<HTMLInputElement>("#inputs .toggle-field input[type=checkbox]");
  if (gravity && gravity.checked !== config.gravity) gravity.click();
}

function renameRenderedMechanism(angleConvention: "ascento" | "solver") {
  tagFieldLabels();
  for (const text of Array.from(document.querySelectorAll<SVGTextElement>(".joint-labels text"))) {
    const value = text.textContent?.trim() ?? "";
    const replacement = jointText[value];
    if (replacement) text.textContent = replacement;
  }
  for (const text of Array.from(document.querySelectorAll<SVGTextElement>(".link-label"))) {
    const value = text.textContent ?? "";
    if (value.includes("A-02")) text.textContent = "PK · rocker";
    else if (value.includes("B-04")) text.textContent = "HI · hip input";
    else if (value.trim() === "leg") text.textContent = "KIW · wheel carrier";
  }

  for (const card of Array.from(document.querySelectorAll<HTMLElement>(".metric-card"))) {
    const label = card.querySelector<HTMLElement>(":scope > span");
    const value = card.querySelector<HTMLElement>(":scope > strong");
    if (!label || !value) continue;
    if (label.textContent?.includes("Tool point T") || label.textContent?.includes("Wheel W position")) {
      label.textContent = "Wheel W position";
      const raw = value.textContent?.trim() ?? "";
      if (raw !== "—" && raw !== value.dataset.lastRobotValue) {
        const numbers = raw.match(/-?\d+(?:\.\d+)?/g)?.map(Number);
        if (numbers && numbers.length >= 2) {
          const converted = `${numbers[0].toFixed(1)}, ${(-numbers[1]).toFixed(1)}`;
          value.textContent = converted;
          value.dataset.lastRobotValue = converted;
        }
      }
      const small = card.querySelector<HTMLElement>("small");
      if (small) small.textContent = "x, z · mm · +z downward";
    }
  }

  for (const item of Array.from(document.querySelectorAll<HTMLElement>(".state-strip > div"))) {
    const label = item.querySelector<HTMLElement>("span");
    if (!label) continue;
    const value = label.textContent ?? "";
    if (value.includes("A-02 rocker θ₂")) label.textContent = "PK rocker angle";
    else if (value.includes("Input B-04 ω₄")) label.textContent = "HI input angular speed";
    else if (value.includes("Input B-04 α₄")) label.textContent = "HI input angular acceleration";
    else if (value.includes("A-02 rocker ω₂")) label.textContent = "PK rocker angular speed";
  }

  const coordinate = document.querySelector<HTMLElement>(".stage-coordinate");
  if (coordinate) {
    const text = coordinate.textContent ?? "";
    const match = text.match(/t\s+([\d.]+)\s+s\s+·\s+θ₄\s+(-?[\d.]+)/);
    if (match && angleConvention === "ascento") {
      const theta = Number(match[2]);
      coordinate.textContent = `t ${Number(match[1]).toFixed(3)} s · q ${solverAngleToHipQ(theta).toFixed(1)}° · θ₄ ${theta.toFixed(1)}°`;
    }
  }
}

function currentSolverAngle() {
  const text = document.querySelector<HTMLElement>(".stage-coordinate")?.textContent ?? "";
  const match = text.match(/θ₄\s+(-?[\d.]+)/);
  if (match) return Number(match[1]);
  return null;
}

function currentPlayState() {
  return Boolean(document.querySelector(".simulator-play.active"));
}

function setPlayheadForAngle(config: Config, angle: number) {
  const time = timeAtAngle(config, angle);
  if (!Number.isFinite(time)) return;
  const input = document.querySelector<HTMLInputElement>(".motion-rail input[type=range], .mobile-time-scrubber input[type=range]");
  if (!input) return;
  setNativeValue(input, String(time));
}

function affineTransform(model: OverlayPoint[], screen: OverlayPoint[]) {
  if (model.length < 3 || screen.length < 3) return null;
  const [p0, p1, p2] = model;
  const [s0, s1, s2] = screen;
  const v1 = { x: p1.x - p0.x, y: p1.y - p0.y };
  const v2 = { x: p2.x - p0.x, y: p2.y - p0.y };
  const w1 = { x: s1.x - s0.x, y: s1.y - s0.y };
  const w2 = { x: s2.x - s0.x, y: s2.y - s0.y };
  const det = v1.x * v2.y - v1.y * v2.x;
  if (Math.abs(det) < 1e-9) return null;
  const inv00 = v2.y / det;
  const inv01 = -v2.x / det;
  const inv10 = -v1.y / det;
  const inv11 = v1.x / det;
  const a = w1.x * inv00 + w2.x * inv10;
  const b = w1.x * inv01 + w2.x * inv11;
  const c = w1.y * inv00 + w2.y * inv10;
  const d = w1.y * inv01 + w2.y * inv11;
  return {
    point(point: OverlayPoint) {
      const x = point.x - p0.x;
      const y = point.y - p0.y;
      return { x: s0.x + a * x + b * y, y: s0.y + c * x + d * y };
    },
    scale: (Math.hypot(a, c) + Math.hypot(b, d)) / 2,
  };
}

function modelToOverlayPath(points: Array<{ x: number; z: number }>, transform: ReturnType<typeof affineTransform>) {
  if (!transform || points.length < 2) return "";
  return points
    .map((point, index) => {
      const mapped = transform.point({ x: point.x, y: -point.z });
      return `${index ? "L" : "M"}${mapped.x.toFixed(1)},${mapped.y.toFixed(1)}`;
    })
    .join(" ");
}

function buildOverlay(
  config: Config,
  angle: number,
  snapshotA: Snapshot | null,
  snapshotB: Snapshot | null,
  comOffsetX: number,
  comOffsetZ: number,
): OverlayData | null {
  const svg = document.querySelector<SVGSVGElement>(".simulator-stage svg");
  const fixed = Array.from(svg?.querySelectorAll<SVGCircleElement>("circle.fixed-joint") ?? []);
  const moving = Array.from(svg?.querySelectorAll<SVGCircleElement>("circle.moving-joint") ?? []);
  if (!svg || fixed.length < 2 || moving.length < 1) return null;
  const pose = solvePose(config, angle);
  if (!pose) return null;
  const transform = affineTransform(
    [pose.O2, pose.O4, pose.A],
    [
      { x: Number(fixed[0].getAttribute("cx")), y: Number(fixed[0].getAttribute("cy")) },
      { x: Number(fixed[1].getAttribute("cx")), y: Number(fixed[1].getAttribute("cy")) },
      { x: Number(moving[0].getAttribute("cx")), y: Number(moving[0].getAttribute("cy")) },
    ],
  );
  if (!transform) return null;

  const wheel = transform.point(pose.T);
  const comRobot = referenceCom(config, comOffsetX, comOffsetZ);
  const com = transform.point({ x: comRobot.x, y: -comRobot.z });

  const hp = { x: pose.O2.x - pose.O4.x, y: pose.O2.y - pose.O4.y };
  const hpLength = Math.hypot(hp.x, hp.y) || 1;
  const tangent = { x: hp.x / hpLength, y: hp.y / hpLength };
  const normal = { x: -tangent.y, y: tangent.x };
  const extension = 34;
  const halfWidth = 38;
  const h = pose.O4;
  const p = pose.O2;
  const corners = [
    { x: h.x - tangent.x * extension + normal.x * halfWidth, y: h.y - tangent.y * extension + normal.y * halfWidth },
    { x: p.x + tangent.x * extension + normal.x * halfWidth, y: p.y + tangent.y * extension + normal.y * halfWidth },
    { x: p.x + tangent.x * extension - normal.x * halfWidth, y: p.y + tangent.y * extension - normal.y * halfWidth },
    { x: h.x - tangent.x * extension - normal.x * halfWidth, y: h.y - tangent.y * extension - normal.y * halfWidth },
  ].map(transform.point);
  const body = corners.map((point) => `${point.x.toFixed(1)},${point.y.toFixed(1)}`).join(" ");

  const target = targetDiagonal(ASCENTO_V1_CONFIG, 0.5);
  const targetPath = target
    ? modelToOverlayPath([target.start, target.end], transform)
    : "";
  const samples = sampleWheelPath(config, 1);
  const labels = [
    [0, "Crouch"],
    [0.34, "Ride"],
    [0.68, "Pre-jump"],
    [1, "Takeoff"],
  ] as const;
  const markers = labels.map(([fraction, label]) => {
    const index = Math.min(samples.length - 1, Math.max(0, Math.round(fraction * (samples.length - 1))));
    const point = samples[index] ?? { x: pose.T.x, z: -pose.T.y };
    const mapped = transform.point({ x: point.x, y: -point.z });
    return { ...mapped, label };
  });

  return {
    wheel,
    wheelRadius: Math.max(8, 45 * transform.scale),
    com,
    body,
    targetPath,
    snapshotAPath: snapshotA ? modelToOverlayPath(sampleWheelPath(snapshotA.config, 1), transform) : "",
    snapshotBPath: snapshotB ? modelToOverlayPath(sampleWheelPath(snapshotB.config, 1), transform) : "",
    markers,
  };
}

function formatMetric(value: number | null | undefined, unit: string, digits = 1) {
  return value == null || !Number.isFinite(value) ? "—" : `${value.toFixed(digits)} ${unit}`;
}

function downloadJson(filename: string, value: unknown) {
  const blob = new Blob([JSON.stringify(value, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
  link.click();
  URL.revokeObjectURL(url);
}

function screeningChecks() {
  const blocker = Boolean(document.querySelector(".stage-blocker"));
  if (blocker) {
    return [{ index: -1, category: "geometry", label: "Geometry unreachable", detail: "Fix the requested motion window before interpreting loads.", state: "critical" as const }];
  }
  const buttons = Array.from(document.querySelectorAll<HTMLButtonElement>("#results .worst-case-grid .result-jump"));
  return buttons.map((button, index) => {
    const stateNode = button.querySelector<HTMLElement>(".result-state");
    const result = classifyScreeningCheck({
      index,
      disabled: button.disabled,
      warning: button.classList.contains("load-warning"),
      danger: Boolean(stateNode?.classList.contains("danger")),
      pass: Boolean(stateNode?.classList.contains("good")),
    });
    button.dataset.workbenchState = result.state;
    button.dataset.workbenchIndex = String(index);
    const label = button.querySelector<HTMLElement>(":scope > span")?.textContent?.trim() ?? `Check ${index + 1}`;
    const detail = stateNode?.textContent?.trim() ?? "";
    return { index, category: result.category, label, detail, state: result.state as ScreeningState };
  });
}

export default function RobotWorkbench() {
  const [topbar, setTopbar] = useState<HTMLElement | null>(null);
  const [analysisArea, setAnalysisArea] = useState<HTMLElement | null>(null);
  const [resultsArea, setResultsArea] = useState<HTMLElement | null>(null);
  const [svgTarget, setSvgTarget] = useState<SVGSVGElement | null>(null);
  const [config, setConfig] = useState<Config>(ASCENTO_V1_CONFIG);
  const [presetId, setPresetId] = useState("ascento-v1");
  const [angleConvention, setAngleConvention] = useState<"ascento" | "solver">("ascento");
  const [mode, setMode] = useState<Mode>("pose");
  const [solverAngle, setSolverAngle] = useState(337);
  const [playing, setPlaying] = useState(false);
  const [savedPresets, setSavedPresets] = useState<SavedPreset[]>([]);
  const [snapshotA, setSnapshotA] = useState<Snapshot | null>(null);
  const [snapshotB, setSnapshotB] = useState<Snapshot | null>(null);
  const [comOffsetX, setComOffsetX] = useState(0);
  const [comOffsetZ, setComOffsetZ] = useState(0);
  const [overlay, setOverlay] = useState<OverlayData | null>(null);
  const [checks, setChecks] = useState<ScreeningCheck[]>([]);
  const [showPassed, setShowPassed] = useState(false);
  const [analysisView, setAnalysisView] = useState<AnalysisView>("status");
  const [engineeringDetails, setEngineeringDetails] = useState(false);
  const importRef = useRef<HTMLInputElement>(null);
  const lastSignature = useRef("");

  useEffect(() => {
    setTopbar(document.querySelector<HTMLElement>(".topbar"));
    setAnalysisArea(document.querySelector<HTMLElement>(".analysis-area"));
    setResultsArea(document.querySelector<HTMLElement>("#results"));
    setSvgTarget(document.querySelector<SVGSVGElement>(".simulator-stage svg"));
    try {
      setSavedPresets(JSON.parse(localStorage.getItem(PRESET_STORAGE) ?? "[]"));
      const storedSnapshots = JSON.parse(localStorage.getItem(SNAPSHOT_STORAGE) ?? "{}");
      setSnapshotA(storedSnapshots.A ?? null);
      setSnapshotB(storedSnapshots.B ?? null);
      const com = JSON.parse(localStorage.getItem(COM_STORAGE) ?? "{}");
      setComOffsetX(Number(com.x ?? 0));
      setComOffsetZ(Number(com.z ?? 0));
      const details = localStorage.getItem(DETAILS_STORAGE) === "1";
      setEngineeringDetails(details);
    } catch {
      // Ignore stale local workbench state rather than blocking the simulator.
    }
  }, []);

  useEffect(() => {
    document.documentElement.classList.toggle("robot-engineering-open", engineeringDetails);
    localStorage.setItem(DETAILS_STORAGE, engineeringDetails ? "1" : "0");
  }, [engineeringDetails]);

  useEffect(() => {
    document.documentElement.classList.toggle("robot-show-passes", showPassed);
  }, [showPassed]);

  useEffect(() => {
    document.documentElement.dataset.robotMode = mode;
  }, [mode]);

  useEffect(() => {
    document.documentElement.dataset.analysisView = analysisView;
  }, [analysisView]);

  useEffect(() => {
    localStorage.setItem(COM_STORAGE, JSON.stringify({ x: comOffsetX, z: comOffsetZ }));
  }, [comOffsetX, comOffsetZ]);

  useEffect(() => {
    const sync = () => {
      renameRenderedMechanism(angleConvention);
      const nextConfig = readConfigFromDom();
      const nextAngle = currentSolverAngle() ?? solverAngle;
      const nextPlaying = currentPlayState();
      const signature = JSON.stringify({
        g: [nextConfig.groundX, nextConfig.groundY, nextConfig.crank, nextConfig.coupler, nextConfig.rocker, nextConfig.toolAlong, nextConfig.toolOffset],
        a: [nextConfig.minAngle, nextConfig.maxAngle, nextAngle],
        m: nextConfig.motionProfile,
        p: nextPlaying,
      });
      if (signature !== lastSignature.current) {
        lastSignature.current = signature;
        setConfig(nextConfig);
        setSolverAngle(nextAngle);
        setPlaying(nextPlaying);
        if (configLooksLike(nextConfig, ASCENTO_V1_CONFIG)) {
          setPresetId("ascento-v1");
          setAngleConvention("ascento");
        } else if (configLooksLike(nextConfig, GENERIC_DEMO_CONFIG)) {
          setPresetId("generic");
          setAngleConvention("solver");
        } else if (!presetId.startsWith("saved:")) {
          setPresetId("custom");
        }
      }
      setChecks(screeningChecks());
      setOverlay(buildOverlay(nextConfig, nextAngle, snapshotA, snapshotB, comOffsetX, comOffsetZ));
    };
    sync();
    const timer = window.setInterval(sync, 140);
    return () => window.clearInterval(timer);
  }, [angleConvention, snapshotA, snapshotB, comOffsetX, comOffsetZ, presetId, solverAngle]);

  useEffect(() => {
    if (!resultsArea) return;
    const headings = Array.from(resultsArea.querySelectorAll<HTMLElement>(".section-wide-heading"));
    for (const heading of headings) {
      const title = heading.querySelector("h2")?.textContent?.toLowerCase() ?? "";
      const group = title.includes("dynamic") ? "dynamic" : title.includes("static") ? "static" : null;
      if (!group) continue;
      heading.dataset.analysisGroup = group;
      const next = heading.nextElementSibling as HTMLElement | null;
      if (next?.classList.contains("plot-grid")) next.dataset.analysisGroup = group;
    }
    const detail = resultsArea.querySelector<HTMLElement>(".load-detail-grid");
    if (detail) detail.dataset.analysisGroup = "actuator";
    const assumptions = resultsArea.querySelector<HTMLElement>(".assumption-strip");
    if (assumptions) assumptions.dataset.analysisGroup = "actuator";
  }, [resultsArea, analysisView]);

  const pathMetrics = useMemo(() => wheelPathMetrics(config, ASCENTO_V1_CONFIG, 0.5), [config]);
  const takeoff = useMemo(
    () => takeoffMetrics(config, ASCENTO_V1_CONFIG, { angle: config.maxAngle, comOffsetX, comOffsetZ }),
    [config, comOffsetX, comOffsetZ],
  );
  const currentDerivative = useMemo(() => wheelDerivative(config, solverAngle), [config, solverAngle]);

  const groundLength = Math.hypot(config.groundX, config.groundY);
  const mountAngle = Math.atan2(config.groundY, -config.groundX) * 180 / Math.PI;
  const wheelAlong = config.toolAlong - config.coupler;
  const wheelRadius = Math.hypot(wheelAlong, config.toolOffset);
  const beta = Math.atan2(-config.toolOffset, -wheelAlong) * 180 / Math.PI;
  const q = solverAngleToHipQ(solverAngle);
  const qMinimum = solverAngleToHipQ(config.maxAngle);
  const qMaximum = solverAngleToHipQ(config.minAngle);

  const issues = checks.filter((check) => check.state === "critical" || check.state === "warning");
  const passed = checks.filter((check) => check.state === "pass");

  async function setGeometryValue(kind: string, value: number) {
    if (!Number.isFinite(value)) return;
    if (kind === "groundLength" || kind === "mountAngle") {
      const length = kind === "groundLength" ? value : groundLength;
      const alpha = (kind === "mountAngle" ? value : mountAngle) * Math.PI / 180;
      await commitNumericField("groundX", -length * Math.cos(alpha));
      await commitNumericField("groundY", length * Math.sin(alpha));
    } else if (kind === "hip") await commitNumericField("rocker", value);
    else if (kind === "rocker") await commitNumericField("crank", value);
    else if (kind === "coupler") await commitNumericField("coupler", value);
    else if (kind === "wheelRadius" || kind === "beta") {
      const radius = kind === "wheelRadius" ? value : wheelRadius;
      const angle = (kind === "beta" ? value : beta) * Math.PI / 180;
      await commitNumericField("toolAlong", config.coupler - radius * Math.cos(angle));
      await commitNumericField("toolOffset", -radius * Math.sin(angle));
    }
  }

  async function selectPreset(value: string) {
    setPresetId(value);
    if (value === "ascento-v1") {
      const reset = Array.from(document.querySelectorAll<HTMLButtonElement>("button")).find((button) => button.textContent?.trim() === "Reset preset");
      if (reset) reset.click();
      else await applyConfig(ASCENTO_V1_CONFIG);
      setAngleConvention("ascento");
      return;
    }
    if (value === "generic") {
      await applyConfig(GENERIC_DEMO_CONFIG);
      setAngleConvention("solver");
      return;
    }
    if (value.startsWith("saved:")) {
      const index = Number(value.slice(6));
      const preset = savedPresets[index];
      if (preset) {
        await applyConfig(preset.config);
        setAngleConvention(preset.angleConvention);
      }
    }
  }

  function savePreset() {
    const name = window.prompt("Name this linkage preset:", "Robot leg variant");
    if (!name?.trim()) return;
    const next = [...savedPresets, { name: name.trim(), config, angleConvention }];
    setSavedPresets(next);
    localStorage.setItem(PRESET_STORAGE, JSON.stringify(next));
    setPresetId(`saved:${next.length - 1}`);
  }

  function deletePreset() {
    if (!presetId.startsWith("saved:")) return;
    const index = Number(presetId.slice(6));
    const next = savedPresets.filter((_, itemIndex) => itemIndex !== index);
    setSavedPresets(next);
    localStorage.setItem(PRESET_STORAGE, JSON.stringify(next));
    setPresetId("custom");
  }

  function captureSnapshot(name: "A" | "B") {
    const snapshot: Snapshot = { name, config, summary: summarizeDesign(config, ASCENTO_V1_CONFIG) };
    const nextA = name === "A" ? snapshot : snapshotA;
    const nextB = name === "B" ? snapshot : snapshotB;
    setSnapshotA(nextA);
    setSnapshotB(nextB);
    localStorage.setItem(SNAPSHOT_STORAGE, JSON.stringify({ A: nextA, B: nextB }));
  }

  function clearSnapshots() {
    setSnapshotA(null);
    setSnapshotB(null);
    localStorage.removeItem(SNAPSHOT_STORAGE);
  }

  function exportCurrent() {
    downloadJson("robot-leg-linkage-config.json", {
      name: presetId === "ascento-v1" ? "Ascento-inspired V1" : "Robot leg custom configuration",
      status: presetId === "ascento-v1" ? "image-derived seed; not official Ascento CAD" : "user configuration",
      coordinateSystem: { x: "+forward", z: "+downward", solverY: "+up", relationship: "z=-y" },
      angleConvention: angleConvention === "ascento" ? { q: "415° - theta4", solver: "theta4" } : { solver: "theta4" },
      comReference: { offsetX: comOffsetX, offsetZ: comOffsetZ, base: "midpoint H-P" },
      labConfig: config,
    });
  }

  async function importJson(file: File) {
    const text = await file.text();
    const parsed = JSON.parse(text);
    const imported = (parsed.labConfig ?? parsed.config ?? parsed) as Partial<Config>;
    const next = { ...config, ...imported } as Config;
    await applyConfig(next);
    setPresetId("custom");
    if (parsed.angleConvention?.q || parsed.scaleReference?.ratio_HP_HI_PK_IK_IW) setAngleConvention("ascento");
  }

  function focusCheck(check: ScreeningCheck) {
    if (check.index < 0) {
      document.querySelector("#inputs")?.scrollIntoView({ behavior: "smooth", block: "start" });
      return;
    }
    const button = document.querySelector<HTMLButtonElement>(`#results .result-jump[data-workbench-index="${check.index}"]`);
    if (button && !button.disabled) button.click();
    else button?.scrollIntoView({ behavior: "smooth", block: "center" });
  }

  const presetName = presetId === "ascento-v1"
    ? "Ascento-inspired V1"
    : presetId === "generic"
      ? "Generic four-bar"
      : presetId.startsWith("saved:")
        ? savedPresets[Number(presetId.slice(6))]?.name ?? "Saved design"
        : "Custom linkage";

  return (
    <>
      {topbar
        ? createPortal(
            <>
              <nav className="robot-section-nav" aria-label="Primary sections">
                <a href="#simulator">Simulate</a>
                <a href="#inputs">Configure</a>
                <a href="#results">
                  Analyze
                  {issues.length ? <span>{issues.length}</span> : null}
                </a>
              </nav>
              <div className="robot-preset-chip" title="Current configuration identity">
                <strong>{presetName}</strong>
                <small>{presetId === "ascento-v1" ? "image-derived V1 seed" : "robot-leg workbench"}</small>
              </div>
            </>,
            topbar,
          )
        : null}

      {analysisArea
        ? createPortal(
            <aside className="robot-quick-panel" aria-label="Robot leg workbench controls">
              <header>
                <div>
                  <span>Robot-leg workbench</span>
                  <strong>{presetName}</strong>
                  <small>+x forward · +z downward · wheel W</small>
                </div>
                <button type="button" className={engineeringDetails ? "active" : ""} onClick={() => setEngineeringDetails((value) => !value)}>
                  {engineeringDetails ? "Less detail" : "Engineering detail"}
                </button>
              </header>

              <section className="robot-preset-row">
                <label>
                  <span>Preset</span>
                  <select value={presetId} onChange={(event) => void selectPreset(event.target.value)}>
                    <option value="ascento-v1">Ascento-inspired V1</option>
                    <option value="generic">Generic four-bar</option>
                    <option value="custom">Custom</option>
                    {savedPresets.map((preset, index) => (
                      <option key={`${preset.name}-${index}`} value={`saved:${index}`}>{preset.name}</option>
                    ))}
                  </select>
                </label>
                <div className="robot-inline-actions">
                  <button type="button" onClick={savePreset}>Save current</button>
                  <button type="button" disabled={!presetId.startsWith("saved:")} onClick={deletePreset}>Delete saved</button>
                </div>
                {presetId === "ascento-v1" ? (
                  <p className="robot-provenance">Approximate image-derived Ascento ratios for V1 simulation. Not official Ascento CAD geometry.</p>
                ) : null}
              </section>

              <section>
                <div className="robot-section-title"><strong>Geometry</strong><small>direct design variables</small></div>
                <div className="robot-field-grid">
                  <label><span>Fixed HP length</span><input type="number" step="1" value={round(groundLength, 3)} onChange={(event) => void setGeometryValue("groundLength", Number(event.target.value))}/><small>mm</small></label>
                  <label><span>Mount α H→P</span><input type="number" step="1" value={round(mountAngle, 2)} onChange={(event) => void setGeometryValue("mountAngle", Number(event.target.value))}/><small>°</small></label>
                  <label><span>Hip link HI</span><input type="number" step="1" value={round(config.rocker, 3)} onChange={(event) => void setGeometryValue("hip", Number(event.target.value))}/><small>mm</small></label>
                  <label><span>Rocker PK</span><input type="number" step="1" value={round(config.crank, 3)} onChange={(event) => void setGeometryValue("rocker", Number(event.target.value))}/><small>mm</small></label>
                  <label><span>Coupler IK</span><input type="number" step="1" value={round(config.coupler, 3)} onChange={(event) => void setGeometryValue("coupler", Number(event.target.value))}/><small>mm</small></label>
                  <label><span>Wheel offset IW</span><input type="number" step="1" value={round(wheelRadius, 3)} onChange={(event) => void setGeometryValue("wheelRadius", Number(event.target.value))}/><small>mm</small></label>
                  <label><span>Wheel angle β</span><input type="number" step="1" value={round(beta, 2)} onChange={(event) => void setGeometryValue("beta", Number(event.target.value))}/><small>°</small></label>
                </div>
              </section>

              <section className="robot-motion-mode">
                <div className="robot-section-title"><strong>Motion</strong><small>pose and animation are separate</small></div>
                <div className="robot-mode-toggle" role="group" aria-label="Motion interaction mode">
                  <button type="button" className={mode === "pose" ? "active" : ""} onClick={() => setMode("pose")}>Pose</button>
                  <button type="button" className={mode === "animate" ? "active" : ""} onClick={() => setMode("animate")}>Animate</button>
                </div>
                {mode === "pose" ? (
                  <label className="robot-pose-slider">
                    <span>{angleConvention === "ascento" ? `Hip q ${q.toFixed(1)}°` : `Input θ₄ ${solverAngle.toFixed(1)}°`}</span>
                    <input
                      type="range"
                      min={angleConvention === "ascento" ? Math.min(qMinimum, qMaximum) : config.minAngle}
                      max={angleConvention === "ascento" ? Math.max(qMinimum, qMaximum) : config.maxAngle}
                      step="0.1"
                      value={angleConvention === "ascento" ? q : solverAngle}
                      onChange={(event) => {
                        if (playing) document.querySelector<HTMLButtonElement>(".simulator-play.active")?.click();
                        const requested = Number(event.target.value);
                        const theta = angleConvention === "ascento" ? hipQToSolverAngle(requested) : requested;
                        setPlayheadForAngle(config, theta);
                      }}
                    />
                    <small>{angleConvention === "ascento" ? `solver θ₄ ${solverAngle.toFixed(1)}°` : "direct solver angle"}</small>
                  </label>
                ) : (
                  <div className="robot-animate-controls">
                    <button type="button" className={playing ? "active" : ""} onClick={() => document.querySelector<HTMLButtonElement>(".simulator-play")?.click()}>{playing ? "Pause" : "Run cycle"}</button>
                    <span>t-controlled motion · configured {config.motionProfile === "sinusoidal" ? "sinusoid" : "S-curve"}</span>
                  </div>
                )}
              </section>

              <section>
                <div className="robot-section-title"><strong>Leg objective</strong><small>V1 diagonal target</small></div>
                <div className="robot-kpi-grid">
                  <div><span>Vertical stroke</span><strong>{formatMetric(pathMetrics.verticalStroke, "mm")}</strong></div>
                  <div><span>Fore-aft excursion</span><strong>{formatMetric(pathMetrics.foreAftExcursion, "mm")}</strong></div>
                  <div><span>Target RMS</span><strong>{formatMetric(pathMetrics.targetRms, "mm")}</strong></div>
                  <div><span>Min transmission</span><strong>{formatMetric(pathMetrics.minTransmission, "°")}</strong></div>
                  <div><span>dx/dq now</span><strong>{formatMetric(currentDerivative?.dxDq, "mm/°", 2)}</strong></div>
                  <div><span>dz/dq now</span><strong>{formatMetric(currentDerivative?.dzDq, "mm/°", 2)}</strong></div>
                  <div><span>Takeoff W−COM x</span><strong>{formatMetric(takeoff?.wheelMinusComX, "mm", 2)}</strong></div>
                  <div><span>Takeoff dx/dq</span><strong>{formatMetric(takeoff?.dxDq, "mm/°", 2)}</strong></div>
                </div>
                <details className="robot-com-reference">
                  <summary>COM reference</summary>
                  <p>Reference COM starts at the midpoint of H–P. Move it here when the body mass model becomes known.</p>
                  <div className="robot-field-grid two">
                    <label><span>COM Δx</span><input type="number" value={comOffsetX} step="1" onChange={(event) => setComOffsetX(Number(event.target.value))}/><small>mm</small></label>
                    <label><span>COM Δz</span><input type="number" value={comOffsetZ} step="1" onChange={(event) => setComOffsetZ(Number(event.target.value))}/><small>mm</small></label>
                  </div>
                </details>
              </section>

              <section>
                <div className="robot-section-title"><strong>Compare designs</strong><small>overlay + key metrics</small></div>
                <div className="robot-inline-actions three">
                  <button type="button" onClick={() => captureSnapshot("A")}>Capture A</button>
                  <button type="button" onClick={() => captureSnapshot("B")}>Capture B</button>
                  <button type="button" disabled={!snapshotA && !snapshotB} onClick={clearSnapshots}>Clear</button>
                </div>
                {(snapshotA || snapshotB) ? (
                  <div className="robot-comparison-table" role="table" aria-label="Design comparison">
                    <div className="head"><span>Metric</span><strong>A</strong><strong>B</strong></div>
                    {[
                      ["Stroke", "verticalStroke", "mm"],
                      ["Fore/aft", "foreAftExcursion", "mm"],
                      ["Target RMS", "targetRms", "mm"],
                      ["Min μ", "minTransmission", "°"],
                      ["Peak torque", "peakTorque", "N·m"],
                    ].map(([label, key, unit]) => (
                      <div key={key}><span>{label}</span><strong>{formatMetric(snapshotA?.summary[key as keyof DesignSummary] as number | null, unit)}</strong><strong>{formatMetric(snapshotB?.summary[key as keyof DesignSummary] as number | null, unit)}</strong></div>
                    ))}
                  </div>
                ) : <p className="robot-muted">Capture A, change the linkage, then capture B. Both wheel paths will overlay on the simulator.</p>}
              </section>

              <section>
                <div className="robot-section-title"><strong>Reproducibility</strong><small>presets and files</small></div>
                <div className="robot-inline-actions three">
                  <button type="button" onClick={exportCurrent}>Export JSON</button>
                  <button type="button" onClick={() => importRef.current?.click()}>Import JSON</button>
                  <button type="button" onClick={() => downloadJson("robot-leg-design-comparison.json", { A: snapshotA, B: snapshotB })} disabled={!snapshotA && !snapshotB}>Export compare</button>
                </div>
                <input ref={importRef} type="file" accept="application/json,.json" hidden onChange={(event) => {
                  const file = event.target.files?.[0];
                  if (file) void importJson(file).catch((error) => window.alert(`Could not import configuration: ${String(error)}`));
                  event.target.value = "";
                }}/>
              </section>
            </aside>,
            analysisArea,
          )
        : null}

      {resultsArea
        ? createPortal(
            <section className={`robot-design-status ${issues.length ? "has-issues" : "all-good"}`}>
              <div>
                <span>Design status</span>
                <strong>{issues.length ? `${issues.length} issue${issues.length === 1 ? "" : "s"} need attention` : "All screened checks passed"}</strong>
                <small>Screening state is derived from structured result classes and reachability, not parsed status sentences.</small>
              </div>
              {issues.length ? (
                <div className="robot-issue-list">
                  {issues.map((issue) => (
                    <button key={`${issue.index}-${issue.label}`} type="button" className={issue.state} onClick={() => focusCheck(issue)}>
                      <span>{issue.state}</span><strong>{issue.label}</strong><small>{issue.detail || issue.category}</small>
                    </button>
                  ))}
                </div>
              ) : null}
              <div className="robot-status-actions">
                {passed.length ? <button type="button" onClick={() => setShowPassed((value) => !value)}>{showPassed ? "Hide passed" : `Show ${passed.length} passed`}</button> : null}
                <button type="button" className={engineeringDetails ? "active" : ""} onClick={() => setEngineeringDetails((value) => !value)}>{engineeringDetails ? "Compact results" : "Engineering detail"}</button>
              </div>
              <nav className="robot-analysis-tabs" aria-label="Analysis views">
                {(["status", "dynamic", "static", "actuator"] as AnalysisView[]).map((view) => (
                  <button key={view} type="button" className={analysisView === view ? "active" : ""} onClick={() => setAnalysisView(view)}>{view[0].toUpperCase() + view.slice(1)}</button>
                ))}
              </nav>
            </section>,
            resultsArea,
          )
        : null}

      {svgTarget && overlay
        ? createPortal(
            <g className="robot-physical-overlay" pointerEvents="none">
              <polygon points={overlay.body} className="robot-body-reference" />
              <path d={overlay.targetPath} className="robot-target-path" />
              {overlay.snapshotAPath ? <path d={overlay.snapshotAPath} className="robot-snapshot-path snapshot-a" /> : null}
              {overlay.snapshotBPath ? <path d={overlay.snapshotBPath} className="robot-snapshot-path snapshot-b" /> : null}
              <circle cx={overlay.wheel.x} cy={overlay.wheel.y} r={overlay.wheelRadius} className="robot-wheel" />
              <circle cx={overlay.com.x} cy={overlay.com.y} r="7" className="robot-com" />
              <line x1={overlay.com.x - 11} y1={overlay.com.y} x2={overlay.com.x + 11} y2={overlay.com.y} className="robot-com-cross" />
              <line x1={overlay.com.x} y1={overlay.com.y - 11} x2={overlay.com.x} y2={overlay.com.y + 11} className="robot-com-cross" />
              <text x={overlay.com.x + 12} y={overlay.com.y - 10} className="robot-overlay-label">COM ref</text>
              {overlay.markers.map((marker) => (
                <g key={marker.label}>
                  <circle cx={marker.x} cy={marker.y} r="4.5" className="robot-pose-marker" />
                  <text x={marker.x + 8} y={marker.y - 8} className="robot-pose-label">{marker.label}</text>
                </g>
              ))}
              <g className="robot-frame-axes">
                <line x1="38" y1="52" x2="104" y2="52" />
                <line x1="38" y1="52" x2="38" y2="118" />
                <text x="110" y="57">x →</text>
                <text x="27" y="137">z ↓</text>
              </g>
            </g>,
            svgTarget,
          )
        : null}
    </>
  );
}
