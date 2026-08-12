"use client";
import { useEffect, useRef, useState } from "react";
import type {
  ChangeEvent,
  KeyboardEvent as ReactKeyboardEvent,
  PointerEvent as ReactPointerEvent,
} from "react";
import { clamp } from "../lib/linkage-analysis.mjs";
import type {
  AnalysisSummary,
  MotionPeak,
  MotionProfileType,
} from "../lib/linkage-analysis.mjs";
import { validateNumericValue } from "../lib/input-validation.mjs";
type Branch = -1 | 1;
export type Config = {
  groundX: number;
  groundY: number;
  crank: number;
  coupler: number;
  rocker: number;
  toolAlong: number;
  toolOffset: number;
  minAngle: number;
  maxAngle: number;
  motionProfile: MotionProfileType;
  maxVelocity: number;
  maxAcceleration: number;
  maxJerk: number;
  cycleTime: number;
  branch: Branch;
  supportForce: number;
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
type NumberKey = {
  [K in keyof Config]: Config[K] extends number ? K : never;
}[keyof Config];
type Rule = {
  min?: number;
  max?: number;
  integer?: boolean;
  validate?: (value: number) => string | null;
};
type FieldSpec = Rule & {
  key: NumberKey;
  label: string;
  unit: string;
  step?: number;
};
type Group = { title: string; note: string; fields: FieldSpec[] };
export type Vec = { x: number; y: number };
export type PlotPoint = { x: number; y: number | null };
export const DEFAULT_CONFIG: Config = {
  groundX: 45,
  groundY: -40,
  crank: 40,
  coupler: 45,
  rocker: 60,
  toolAlong: 120,
  toolOffset: 0,
  minAngle: 165,
  maxAngle: 225,
  motionProfile: "s-curve",
  maxVelocity: 360,
  maxAcceleration: 1500,
  maxJerk: 10000,
  cycleTime: 2,
  branch: -1,
  supportForce: 100,
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
export function NumericInput({
  label,
  unit,
  value,
  onCommit,
  resetVersion,
  ...rules
}: Rule & {
  label: string;
  unit: string;
  value: number;
  step?: number;
  onCommit: (value: number) => void;
  resetVersion?: number;
}) {
  const [draft, setDraft] = useState(String(value)),
    [error, setError] = useState<string | null>(null);
  useEffect(() => {
    setDraft(String(value));
    setError(null);
  }, [value, resetVersion]);
  const inspect = (text: string) =>
      text.trim() === ""
        ? "A value is required."
        : validateNumericValue(Number(text), rules),
    commit = () => {
      const message = inspect(draft);
      if (message) {
        setError(`${message} Previous valid value remains active.`);
        return;
      }
      const next = Number(draft);
      onCommit(next);
      setDraft(String(next));
      setError(null);
    };
  return (
    <label className="field">
      <span>{label}</span>
      <span className="input-wrap">
        <input
          type="number"
          value={draft}
          step={rules.step ?? 1}
          min={rules.min}
          max={rules.max}
          aria-invalid={Boolean(error)}
          onChange={(event: ChangeEvent<HTMLInputElement>) => {
            setDraft(event.target.value);
            const message = inspect(event.target.value);
            setError(
              message
                ? `${message} Previous valid value remains active.`
                : null,
            );
          }}
          onBlur={commit}
          onKeyDown={(event: ReactKeyboardEvent<HTMLInputElement>) => {
            if (event.key === "Enter") event.currentTarget.blur();
            if (event.key === "Escape") {
              setDraft(String(value));
              setError(null);
              event.currentTarget.blur();
            }
          }}
        />
        <small>{unit}</small>
      </span>
      {error ? (
        <small className="value-danger" role="alert">
          {error}
        </small>
      ) : null}
    </label>
  );
}
function peakResolution(value: number | null, kind: "time" | "angle") {
  if (value === null) return "";
  const unit = kind === "time" ? "s" : "°",
    formatted =
      value < (kind === "time" ? 0.001 : 0.01)
        ? value.toExponential(1)
        : value.toFixed(kind === "time" ? 4 : 3);
  return ` · ${kind === "time" ? "Δt" : "Δθ"} ≤ ${formatted} ${unit}`;
}
function refinementLabel(peak: MotionPeak) {
  return peak.refinement === "local_golden_section"
    ? " · refined"
    : peak.refinement === "sampled_boundary"
      ? " · boundary"
      : "";
}
export function motionPeakMeta(peak: MotionPeak | undefined) {
  if (!peak || peak.angle === null || peak.time === null)
    return "no valid peak";
  return `t ${peak.time.toFixed(4)} s · θ₄ ${peak.angle.toFixed(3)}°${refinementLabel(peak)}${peakResolution(peak.resolution, "time")}`;
}
export function anglePeakMeta(peak: MotionPeak | undefined) {
  if (!peak || peak.angle === null) return "no valid point";
  return `θ₄ ${peak.angle.toFixed(3)}°${refinementLabel(peak)}${peakResolution(peak.resolution, "angle")}`;
}
export function safetyText(
  _summary: AnalysisSummary | null,
  value: number | null | undefined,
) {
  if (value == null) return "—";
  return Number.isFinite(value) ? `${value.toFixed(1)}×` : "∞";
}
type PlotProps = {
  title: string;
  unit: string;
  xUnit: string;
  points: PlotPoint[];
  currentX: number;
  range?: [number, number];
  alertBelow?: number;
  onXChange?: (x: number) => void;
};
type PlotTone = "current" | "dynamic" | "static" | "geometry";
function plotTone(title: string): PlotTone {
  const key = title.toLowerCase();
  if (
    key.includes("static") ||
    key.includes("support") ||
    key.includes("mechanical advantage") ||
    key.includes("moment arm")
  )
    return "static";
  if (key.includes("dynamic") || key.includes("torque")) return "dynamic";
  if (key.includes("transmission")) return "geometry";
  return "current";
}
function plotValue(value: number | undefined) {
  if (value === undefined || !Number.isFinite(value)) return "—";
  const abs = Math.abs(value);
  return abs >= 100
    ? value.toFixed(0)
    : abs >= 10
      ? value.toFixed(1)
      : value.toFixed(3);
}
export function Plot({
  title,
  unit,
  xUnit,
  points,
  currentX,
  range,
  alertBelow,
  onXChange,
}: PlotProps) {
  const [compact, setCompact] = useState(false),
    [expanded, setExpanded] = useState(false),
    closeRef = useRef<HTMLButtonElement>(null),
    triggerRef = useRef<HTMLButtonElement | null>(null);
  const close = () => {
    setExpanded(false);
    window.requestAnimationFrame(() => triggerRef.current?.focus());
  };
  useEffect(() => {
    const query = window.matchMedia("(max-width: 620px)"),
      sync = () => setCompact(query.matches);
    sync();
    query.addEventListener("change", sync);
    return () => query.removeEventListener("change", sync);
  }, []);
  useEffect(() => {
    if (expanded) closeRef.current?.focus();
  }, [expanded]);
  const width = 720,
    finite = points.filter(
      (point): point is { x: number; y: number } =>
        point.y !== null && Number.isFinite(point.y),
    ),
    x0 = points[0]?.x ?? 0,
    x1 = points.at(-1)?.x ?? 1,
    dx = Math.max(1e-9, x1 - x0);
  let y0 = range?.[0] ?? Math.min(0, ...finite.map((p) => p.y)),
    y1 = range?.[1] ?? Math.max(0, ...finite.map((p) => p.y));
  if (!Number.isFinite(y0) || !Number.isFinite(y1)) {
    y0 = 0;
    y1 = 1;
  }
  if (!range) {
    const pad = Math.max(1e-6, y1 - y0) * 0.08;
    y0 -= pad;
    y1 += pad;
  }
  const dy = Math.max(1e-9, y1 - y0);
  const currentPoint = finite.length
      ? finite.reduce(
          (best, point) =>
            Math.abs(point.x - currentX) < Math.abs(best.x - currentX)
              ? point
              : best,
          finite[0],
        )
      : undefined,
    minPoint = finite.length
      ? finite.reduce(
          (best, point) => (point.y < best.y ? point : best),
          finite[0],
        )
      : undefined,
    maxPoint = finite.length
      ? finite.reduce(
          (best, point) => (point.y > best.y ? point : best),
          finite[0],
        )
      : undefined,
    tone = plotTone(title);
  const renderSvg = (detail: boolean) => {
    const height = detail ? (compact ? 600 : 420) : 240,
      m = {
        l: detail ? 58 : 52,
        r: detail ? 22 : 16,
        t: detail ? 34 : 28,
        b: detail ? 46 : 38,
      },
      sx = (x: number) => m.l + ((x - x0) / dx) * (width - m.l - m.r),
      sy = (v: number) => m.t + ((y1 - v) / dy) * (height - m.t - m.b),
      fractions = compact && !detail ? [0, 0.5, 1] : [0, 0.25, 0.5, 0.75, 1];
    let path = "",
      drawing = false;
    for (const point of points) {
      if (point.y === null || !Number.isFinite(point.y)) {
        drawing = false;
        continue;
      }
      path += `${drawing ? "L" : "M"}${sx(point.x).toFixed(1)},${sy(point.y).toFixed(1)} `;
      drawing = true;
    }
    const selectX = (event: ReactPointerEvent<SVGRectElement>) => {
      if (!detail || !onXChange) return;
      const bounds = event.currentTarget.getBoundingClientRect(),
        fraction = clamp(
          (event.clientX - bounds.left) / Math.max(1, bounds.width),
          0,
          1,
        );
      onXChange(x0 + fraction * dx);
    };
    return (
      <svg
        viewBox={`0 0 ${width} ${height}`}
        className="plot-svg"
        role="img"
        aria-label={`${title}. Current cursor ${currentX.toFixed(3)} ${xUnit}.`}
      >
        {alertBelow !== undefined ? (
          <rect
            x={m.l}
            y={sy(Math.min(alertBelow, y1))}
            width={width - m.l - m.r}
            height={Math.max(0, sy(y0) - sy(Math.min(alertBelow, y1)))}
            className="plot-alert-band"
          />
        ) : null}
        {fractions.map((f) => {
          const yy = m.t + f * (height - m.t - m.b),
            v = y1 - f * dy;
          return (
            <g key={`y-${f}`}>
              <line
                x1={m.l}
                y1={yy}
                x2={width - m.r}
                y2={yy}
                className="plot-grid-line"
              />
              <text
                x={m.l - 7}
                y={yy + 4}
                textAnchor="end"
                className="plot-tick"
              >
                {v.toFixed(Math.abs(v) < 10 ? 1 : 0)}
              </text>
            </g>
          );
        })}
        {fractions.map((f) => {
          const x = x0 + f * dx,
            xx = sx(x);
          return (
            <g key={`x-${f}`}>
              <line
                x1={xx}
                y1={m.t}
                x2={xx}
                y2={height - m.b}
                className="plot-grid-line vertical"
              />
              <text
                x={xx}
                y={height - 13}
                textAnchor="middle"
                className="plot-tick"
              >
                {x.toFixed(compact && !detail ? 1 : 2)} {xUnit}
              </text>
            </g>
          );
        })}
        <path
          d={path.trim()}
          fill="none"
          stroke="var(--plot-tone,var(--orange))"
          className="plot-line"
        />
        <line
          x1={sx(clamp(currentX, x0, x1))}
          y1={m.t}
          x2={sx(clamp(currentX, x0, x1))}
          y2={height - m.b}
          className="plot-cursor"
        />
        {detail && onXChange ? (
          <rect
            x={m.l}
            y={m.t}
            width={width - m.l - m.r}
            height={height - m.t - m.b}
            className="plot-detail-hit"
            onPointerDown={(event) => {
              event.currentTarget.setPointerCapture(event.pointerId);
              selectX(event);
            }}
            onPointerMove={(event) => {
              if (event.buttons === 1) selectX(event);
            }}
            onPointerUp={(event) => {
              if (event.currentTarget.hasPointerCapture(event.pointerId))
                event.currentTarget.releasePointerCapture(event.pointerId);
            }}
          />
        ) : null}
      </svg>
    );
  };
  const open = (trigger: HTMLButtonElement) => {
    triggerRef.current = trigger;
    setExpanded(true);
  };
  const scrubWithKeyboard = (event: ReactKeyboardEvent<HTMLDivElement>) => {
    if (event.key === "Escape") {
      event.preventDefault();
      close();
      return;
    }
    if (!onXChange) return;
    const step = dx / 100;
    if (event.key === "ArrowRight" || event.key === "ArrowUp") {
      event.preventDefault();
      onXChange(clamp(currentX + step, x0, x1));
    } else if (event.key === "ArrowLeft" || event.key === "ArrowDown") {
      event.preventDefault();
      onXChange(clamp(currentX - step, x0, x1));
    } else if (event.key === "Home") {
      event.preventDefault();
      onXChange(x0);
    } else if (event.key === "End") {
      event.preventDefault();
      onXChange(x1);
    }
  };
  return (
    <>
      <article className="plot-card" data-tone={tone}>
        <div className="plot-heading">
          <div>
            <h3>{title}</h3>
            <span>{unit}</span>
          </div>
          <button
            type="button"
            className="plot-expand-button"
            onClick={(event) => open(event.currentTarget)}
          >
            Expand
          </button>
        </div>
        <div className="plot-readout">
          <span>
            <i />
            <strong>Now</strong>
            <b>{plotValue(currentPoint?.y)}</b>
          </span>
          <span>
            <strong>Min</strong>
            <b>{plotValue(minPoint?.y)}</b>
          </span>
          <span>
            <strong>Max</strong>
            <b>{plotValue(maxPoint?.y)}</b>
          </span>
        </div>
        <button
          type="button"
          className="plot-overview-button"
          aria-label={`Open detailed ${title} plot`}
          onClick={(event) => open(event.currentTarget)}
        >
          {renderSvg(false)}
        </button>
      </article>
      {expanded ? (
        <div
          className="plot-modal"
          role="dialog"
          aria-modal="true"
          aria-label={`${title} detailed plot`}
          aria-describedby="plot-modal-description"
          onClick={close}
          onKeyDown={scrubWithKeyboard}
        >
          <div
            className="plot-modal-panel"
            onClick={(event: { stopPropagation: () => void }) =>
              event.stopPropagation()
            }
          >
            <div className="plot-modal-heading">
              <div>
                <strong>{title}</strong>
                <span id="plot-modal-description">
                  {unit} · drag chart to scrub the synchronized mechanism state
                </span>
              </div>
              <button
                type="button"
                ref={closeRef}
                onClick={close}
                aria-label="Close detailed plot"
              >
                Close
              </button>
            </div>
            <div
              className="plot-modal-chart"
              tabIndex={0}
              aria-label="Detailed plot. Use arrow keys to scrub, Home or End for endpoints, and Escape to close."
            >
              {renderSvg(true)}
            </div>
          </div>
        </div>
      ) : null}
    </>
  );
}
export function fieldGroups(config: Config): Group[] {
  return [
    {
      title: "Four-bar geometry",
      note: "millimetres",
      fields: [
        { key: "groundX", label: "Ground Δx", unit: "mm" },
        { key: "groundY", label: "Ground Δy", unit: "mm" },
        { key: "crank", label: "Rocker (A-02)", unit: "mm", min: 0.1 },
        { key: "coupler", label: "Coupler A–B", unit: "mm", min: 0.1 },
        { key: "rocker", label: "Input crank (B-04)", unit: "mm", min: 0.1 },
        { key: "toolAlong", label: "Tool along A–B", unit: "mm" },
        { key: "toolOffset", label: "Tool offset", unit: "mm" },
      ],
    },
    {
      title: "External load",
      note: "applies during motion + static hold",
      fields: [
        {
          key: "supportForce",
          label: "Vertical external load at T",
          unit: "N",
          min: 0,
          step: 10,
        },
      ],
    },
    {
      title: "Mass properties",
      note: "simplified rigid masses",
      fields: [
        {
          key: "crankMass",
          label: "Rocker (A-02) mass",
          unit: "kg",
          min: 0,
          step: 0.01,
        },
        {
          key: "legMass",
          label: "Extended leg mass",
          unit: "kg",
          min: 0,
          step: 0.01,
        },
        {
          key: "rockerMass",
          label: "Input crank (B-04) mass",
          unit: "kg",
          min: 0,
          step: 0.01,
        },
        {
          key: "toolMass",
          label: "Tool / wheel mass",
          unit: "kg",
          min: 0,
          step: 0.01,
        },
      ],
    },
    {
      title: "Actuator & joint screening",
      note: "motor side / nominal stresses",
      fields: [
        {
          key: "gearRatio",
          label: "Gear ratio",
          unit: ":1",
          min: 0.01,
          step: 0.5,
        },
        {
          key: "gearEfficiency",
          label: "Gear efficiency",
          unit: "%",
          min: 0.01,
          max: 100,
        },
        {
          key: "motorContinuous",
          label: "Motor continuous",
          unit: "N·m",
          min: 0.01,
          max: config.motorPeak,
          step: 0.1,
          validate: (v) =>
            v <= config.motorPeak
              ? null
              : `Must be ≤ peak rating (${config.motorPeak} N·m).`,
        },
        {
          key: "motorPeak",
          label: "Motor peak",
          unit: "N·m",
          min: config.motorContinuous,
          step: 0.1,
          validate: (v) =>
            v >= config.motorContinuous
              ? null
              : `Must be ≥ continuous rating (${config.motorContinuous} N·m).`,
        },
        {
          key: "pinDiameter",
          label: "Pin diameter",
          unit: "mm",
          min: 0.1,
          step: 0.5,
        },
        {
          key: "linkThickness",
          label: "Link thickness",
          unit: "mm",
          min: 0.1,
          step: 0.5,
        },
        {
          key: "shearPlanes",
          label: "Shear planes",
          unit: "count",
          min: 1,
          integer: true,
        },
        {
          key: "allowableShear",
          label: "Allowable pin shear",
          unit: "MPa",
          min: 1,
          step: 10,
        },
        {
          key: "allowableBearing",
          label: "Allowable bearing",
          unit: "MPa",
          min: 1,
          step: 10,
        },
      ],
    },
  ];
}
