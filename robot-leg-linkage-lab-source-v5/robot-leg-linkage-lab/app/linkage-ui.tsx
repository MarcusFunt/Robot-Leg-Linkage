"use client";

import { useEffect, useState } from "react";
import type { ChangeEvent, KeyboardEvent as ReactKeyboardEvent, PointerEvent as ReactPointerEvent } from "react";
import { clamp } from "../lib/linkage-analysis.mjs";
import type { AnalysisSummary, CycleSample, PeakResult } from "../lib/linkage-analysis.mjs";
import { validateNumericValue } from "../lib/input-validation.mjs";

type Branch = -1 | 1;
export type Config = {
  groundX: number; groundY: number; crank: number; coupler: number; rocker: number; toolAlong: number; toolOffset: number;
  angle: number; minAngle: number; maxAngle: number; rpm: number; inputAccel: number; branch: Branch;
  forceX: number; forceY: number; gravity: boolean; crankMass: number; legMass: number; rockerMass: number; toolMass: number;
  pinDiameter: number; linkThickness: number; shearPlanes: number; allowableShear: number; allowableBearing: number;
  gearRatio: number; gearEfficiency: number; motorContinuous: number; motorPeak: number;
};
type NumberKey = { [K in keyof Config]: Config[K] extends number ? K : never }[keyof Config];
type Rule = { min?: number; max?: number; integer?: boolean; validate?: (value: number) => string | null };
type FieldSpec = Rule & { key: NumberKey; label: string; unit: string; step?: number };
type Group = { title: string; note: string; fields: FieldSpec[] };
export type Vec = { x: number; y: number };

export const DEFAULT_CONFIG: Config = {
  groundX: 45, groundY: -40, crank: 40, coupler: 45, rocker: 60, toolAlong: 120, toolOffset: 0,
  angle: 180, minAngle: 0, maxAngle: 360, rpm: 60, inputAccel: 0, branch: -1,
  forceX: 0, forceY: 100, gravity: true, crankMass: 0.12, legMass: 0.28, rockerMass: 0.15, toolMass: 0.08,
  pinDiameter: 8, linkThickness: 4, shearPlanes: 2, allowableShear: 120, allowableBearing: 80,
  gearRatio: 10, gearEfficiency: 90, motorContinuous: 0.6, motorPeak: 1.2,
};

export function NumericInput({ label, unit, value, onCommit, ...rules }: Rule & { label: string; unit: string; value: number; step?: number; onCommit: (value: number) => void }) {
  const [draft, setDraft] = useState(String(value));
  const [error, setError] = useState<string | null>(null);
  useEffect(() => { setDraft(String(value)); setError(null); }, [value]);
  const inspect = (text: string) => text.trim() === "" ? "A value is required." : validateNumericValue(Number(text), rules);
  const commit = () => {
    const message = inspect(draft);
    if (message) { setError(`${message} Previous valid value remains active.`); return; }
    const next = Number(draft); onCommit(next); setDraft(String(next)); setError(null);
  };
  return <label className="field"><span>{label}</span><span className="input-wrap"><input type="number" value={draft} step={rules.step ?? 1} min={rules.min} max={rules.max} aria-invalid={Boolean(error)} onChange={(e: ChangeEvent<HTMLInputElement>) => { setDraft(e.target.value); const message = inspect(e.target.value); setError(message ? `${message} Previous valid value remains active.` : null); }} onBlur={commit} onKeyDown={(e: ReactKeyboardEvent<HTMLInputElement>) => { if (e.key === "Enter") e.currentTarget.blur(); if (e.key === "Escape") { setDraft(String(value)); setError(null); e.currentTarget.blur(); } }} /><small>{unit}</small></span>{error ? <small className="value-danger" role="alert">{error}</small> : null}</label>;
}

export function RailLimit({ label, value, min, max, onCommit }: { label: string; value: number; min: number; max: number; onCommit: (value: number) => void }) {
  return <NumericInput label={label} unit="°" value={value} step={0.5} min={min} max={max} onCommit={onCommit} />;
}

export function peakMeta(peak: PeakResult | undefined) {
  if (!peak || peak.angle === null) return "no valid peak";
  const resolution = peak.resolutionDeg === null ? "" : ` · local Δθ ${peak.resolutionDeg.toFixed(3)}°`;
  return `θ₂ ${peak.angle.toFixed(3)}° · ${peak.convergence}${resolution}`;
}

export function safetyText(summary: AnalysisSummary | null, value: number | null | undefined) {
  if (summary?.indeterminateNearToggle) return "indeterminate near toggle";
  if (value === null || value === undefined) return "—";
  return Number.isFinite(value) ? `${value.toFixed(1)}×` : "∞";
}

type PlotProps = {
  title: string;
  unit: string;
  samples: CycleSample[];
  currentAngle: number;
  value: (sample: CycleSample) => number | null;
  range?: [number, number];
  alertBelow?: number;
  onAngleChange?: (angle: number) => void;
};

export function Plot({ title, unit, samples, currentAngle, value, range, alertBelow, onAngleChange }: PlotProps) {
  const [compact, setCompact] = useState(false);
  const [expanded, setExpanded] = useState(false);
  useEffect(() => {
    const query = window.matchMedia("(max-width: 620px)");
    const sync = () => setCompact(query.matches);
    sync(); query.addEventListener("change", sync);
    return () => query.removeEventListener("change", sync);
  }, []);

  const width = 720, height = 240, m = { l: 48, r: 16, t: 28, b: 36 };
  const finite = samples.map((sample) => ({ angle: sample.angle, v: value(sample) })).filter((point): point is { angle: number; v: number } => point.v !== null && Number.isFinite(point.v));
  const x0 = samples[0]?.angle ?? 0, x1 = samples.at(-1)?.angle ?? 360, dx = Math.max(1e-9, x1 - x0);
  let y0 = range?.[0] ?? Math.min(0, ...finite.map((point) => point.v));
  let y1 = range?.[1] ?? Math.max(0, ...finite.map((point) => point.v));
  if (!range) { const pad = Math.max(1e-6, y1 - y0) * 0.08; y0 -= pad; y1 += pad; }
  const dy = Math.max(1e-9, y1 - y0);
  const sx = (angle: number) => m.l + (angle - x0) / dx * (width - m.l - m.r);
  const sy = (v: number) => m.t + (y1 - v) / dy * (height - m.t - m.b);
  let path = "", drawing = false;
  for (const sample of samples) {
    const v = value(sample);
    if (v === null || !Number.isFinite(v)) { drawing = false; continue; }
    path += `${drawing ? "L" : "M"}${sx(sample.angle).toFixed(1)},${sy(v).toFixed(1)} `;
    drawing = true;
  }

  const renderSvg = (detail: boolean) => {
    const fractions = compact && !detail ? [0, 0.5, 1] : [0, 0.25, 0.5, 0.75, 1];
    const selectAngle = (event: ReactPointerEvent<SVGRectElement>) => {
      if (!detail || !onAngleChange) return;
      const bounds = event.currentTarget.getBoundingClientRect();
      const fraction = clamp((event.clientX - bounds.left) / Math.max(1, bounds.width), 0, 1);
      onAngleChange(x0 + fraction * dx);
    };
    return <svg viewBox={`0 0 ${width} ${height}`} className="plot-svg" role="img" aria-label={`${title}. Current crank angle ${currentAngle.toFixed(1)} degrees.`}>
      {alertBelow !== undefined ? <rect x={m.l} y={sy(Math.min(alertBelow, y1))} width={width-m.l-m.r} height={Math.max(0, sy(y0)-sy(Math.min(alertBelow,y1)))} className="plot-alert-band" /> : null}
      {fractions.map((f) => { const yy=m.t+f*(height-m.t-m.b); const val=y1-f*dy; return <g key={`y-${f}`}><line x1={m.l} y1={yy} x2={width-m.r} y2={yy} className="plot-grid-line"/><text x={m.l-7} y={yy+4} textAnchor="end" className="plot-tick">{val.toFixed(Math.abs(val)<10?1:0)}</text></g>; })}
      {fractions.map((f) => { const angle=x0+f*dx; const xx=sx(angle); return <g key={`x-${f}`}><line x1={xx} y1={m.t} x2={xx} y2={height-m.b} className="plot-grid-line vertical"/><text x={xx} y={height-11} textAnchor="middle" className="plot-tick">{angle.toFixed(compact&&!detail?0:1)}°</text></g>; })}
      <path d={path.trim()} fill="none" stroke="var(--orange)" className="plot-line"/>
      <line x1={sx(clamp(currentAngle,x0,x1))} y1={m.t} x2={sx(clamp(currentAngle,x0,x1))} y2={height-m.b} className="plot-cursor"/>
      {detail && onAngleChange ? <rect x={m.l} y={m.t} width={width-m.l-m.r} height={height-m.t-m.b} className="plot-detail-hit" onPointerDown={selectAngle}/> : null}
    </svg>;
  };

  return <>
    <article className="plot-card">
      <div className="plot-heading"><div><h3>{title}</h3><span>{unit}</span></div><button type="button" className="plot-expand-button" onClick={() => setExpanded(true)}>Fullscreen</button></div>
      <button type="button" className="plot-overview-button" aria-label={`Open detailed ${title} plot`} onClick={() => setExpanded(true)}>{renderSvg(false)}</button>
    </article>
    {expanded ? <div className="plot-modal" role="dialog" aria-modal="true" aria-label={`${title} detailed plot`} onClick={() => setExpanded(false)}>
      <div className="plot-modal-panel" onClick={(event) => event.stopPropagation()}>
        <div className="plot-modal-heading"><div><strong>{title}</strong><span>{unit} · tap chart to move θ₂</span></div><button type="button" onClick={() => setExpanded(false)} aria-label="Close detailed plot">Close</button></div>
        <div className="plot-modal-chart">{renderSvg(true)}</div>
      </div>
    </div> : null}
  </>;
}

export function fieldGroups(config: Config): Group[] {
  return [
    { title: "Four-bar geometry", note: "millimetres", fields: [
      { key:"groundX",label:"Ground Δx",unit:"mm"},{key:"groundY",label:"Ground Δy",unit:"mm"},{key:"crank",label:"Input crank",unit:"mm",min:.1},{key:"coupler",label:"Coupler A–B",unit:"mm",min:.1},{key:"rocker",label:"Output rocker",unit:"mm",min:.1},{key:"toolAlong",label:"Tool along A–B",unit:"mm"},{key:"toolOffset",label:"Tool offset",unit:"mm"},{key:"rpm",label:"Input speed",unit:"rpm",min:0,step:5},
    ]},
    { title:"External load & mass", note:"tool force / rigid-body masses", fields:[
      {key:"forceX",label:"Tool force Fx",unit:"N",step:10},{key:"forceY",label:"Tool force Fy",unit:"N",step:10},{key:"inputAccel",label:"Input acceleration",unit:"rad/s²"},{key:"crankMass",label:"Crank mass",unit:"kg",min:0,step:.01},{key:"legMass",label:"Extended leg mass",unit:"kg",min:0,step:.01},{key:"rockerMass",label:"Rocker mass",unit:"kg",min:0,step:.01},{key:"toolMass",label:"Tool / wheel mass",unit:"kg",min:0,step:.01},
    ]},
    { title:"Actuator & joint screening", note:"motor side / nominal stresses", fields:[
      {key:"gearRatio",label:"Gear ratio",unit:":1",min:.01,step:.5},{key:"gearEfficiency",label:"Gear efficiency",unit:"%",min:.01,max:100},{key:"motorContinuous",label:"Motor continuous",unit:"N·m",min:.01,max:config.motorPeak,step:.1,validate:(v)=>v<=config.motorPeak?null:`Must be ≤ peak rating (${config.motorPeak} N·m).`},{key:"motorPeak",label:"Motor peak",unit:"N·m",min:config.motorContinuous,step:.1,validate:(v)=>v>=config.motorContinuous?null:`Must be ≥ continuous rating (${config.motorContinuous} N·m).`},{key:"pinDiameter",label:"Pin diameter",unit:"mm",min:.1,step:.5},{key:"linkThickness",label:"Link thickness",unit:"mm",min:.1,step:.5},{key:"shearPlanes",label:"Shear planes",unit:"count",min:1,integer:true},{key:"allowableShear",label:"Allowable pin shear",unit:"MPa",min:1,step:10},{key:"allowableBearing",label:"Allowable bearing",unit:"MPa",min:1,step:10},
    ]},
  ];
}
