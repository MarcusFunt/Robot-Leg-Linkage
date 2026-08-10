"use client";

import { useEffect, useMemo, useRef, useState } from "react";
import type { ChangeEvent } from "react";
import {
  DEG, adaptiveAnalyze, add, analysisToCsv, clamp, dynamicsBreakdown, kinematics, magnitude,
  mechanismClass, mul, sampleAngleRange, solvePose, summarizeAnalysis, wrapDegrees,
} from "../lib/linkage-analysis.mjs";
import type { CycleSample, DynamicsBreakdown } from "../lib/linkage-analysis.mjs";
import { validateConfig } from "../lib/input-validation.mjs";
import { DEFAULT_CONFIG, NumericInput, Plot, RailLimit, fieldGroups, peakMeta, safetyText } from "./linkage-ui";
import type { Config, Vec } from "./linkage-ui";

type FitMode = "mechanism" | "window" | "path";
type PoseSample = { angle: number; pose: ReturnType<typeof solvePose> };

function toolPath(samples: PoseSample[], map: (point: Vec) => Vec) {
  let path = "", active = false;
  for (const sample of samples) {
    if (!sample.pose) { active = false; continue; }
    const point = map(sample.pose.T);
    path += `${active ? "L" : "M"}${point.x.toFixed(1)},${point.y.toFixed(1)} `;
    active = true;
  }
  return path.trim();
}

function midpoint(a: Vec, b: Vec): Vec { return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 }; }

export default function Home() {
  const [config, setConfig] = useState<Config>(DEFAULT_CONFIG);
  const [playing, setPlaying] = useState(false);
  const [compact, setCompact] = useState(false);
  const [fitMode, setFitMode] = useState<FitMode>("path");
  const [cameraZoom, setCameraZoom] = useState(1);
  const [showLabels, setShowLabels] = useState(true);
  const lastFrame = useRef<number | null>(null);
  const direction = useRef<1 | -1>(1);
  const initialFitSet = useRef(false);
  const update = <K extends keyof Config>(key: K, value: Config[K]) => setConfig((current) => ({ ...current, [key]: value }));

  useEffect(() => {
    const query = window.matchMedia("(max-width: 720px)");
    const sync = () => {
      const next = query.matches;
      setCompact(next);
      if (!initialFitSet.current) { setFitMode(next ? "mechanism" : "path"); initialFitSet.current = true; }
    };
    sync(); query.addEventListener("change", sync);
    return () => query.removeEventListener("change", sync);
  }, []);

  useEffect(() => {
    if (!playing) { lastFrame.current = null; return; }
    let frame = 0;
    const tick = (time: number) => {
      if (lastFrame.current !== null) setConfig((current) => {
        const dt = Math.min(0.05, (time - lastFrame.current!) / 1000), span = current.maxAngle - current.minAngle;
        if (span >= 359.5) return { ...current, angle: wrapDegrees(current.angle + current.rpm * 6 * dt) };
        let next = current.angle + direction.current * Math.abs(current.rpm) * 6 * dt;
        if (next >= current.maxAngle) { next = current.maxAngle; direction.current = -1; }
        if (next <= current.minAngle) { next = current.minAngle; direction.current = 1; }
        return { ...current, angle: next };
      });
      lastFrame.current = time; frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, [playing]);

  const errors = useMemo(() => validateConfig(config), [config]);
  const valid = Object.keys(errors).length === 0;
  const pose = useMemo(() => solvePose(config), [config]);
  const kin = useMemo(() => kinematics(config, pose), [config, pose]);
  const cls = useMemo(() => mechanismClass(config), [config]);
  const current = useMemo<DynamicsBreakdown>(() => valid ? dynamicsBreakdown(config) : { total: null, external: null, gravity: null, inertia: null }, [config, valid]);
  const adaptive = useMemo(() => valid ? adaptiveAnalyze(config, { coarseStepDeg: 10, angleToleranceDeg: 0.05, maxDepth: 12 }) : null, [config, valid]);
  const summary = useMemo(() => adaptive ? summarizeAnalysis(config, adaptive, current) : null, [config, adaptive, current]);
  const samples = adaptive?.samples ?? [];
  const geometry = useMemo<PoseSample[]>(() => sampleAngleRange(0, 360, 2).map((angle) => ({ angle, pose: solvePose(config, angle) })), [config.groundX, config.groundY, config.crank, config.coupler, config.rocker, config.toolAlong, config.toolOffset, config.branch]);
  const windowGeometry = useMemo<PoseSample[]>(() => sampleAngleRange(config.minAngle, config.maxAngle, 2).map((angle) => ({ angle, pose: solvePose(config, angle) })), [config.groundX, config.groundY, config.crank, config.coupler, config.rocker, config.toolAlong, config.toolOffset, config.branch, config.minAngle, config.maxAngle]);
  const reachable = geometry.filter((sample) => sample.pose), coverage = reachable.length / geometry.length;

  const fitSamples = fitMode === "path" ? reachable : fitMode === "window" ? windowGeometry.filter((sample) => sample.pose) : pose ? [{ angle: config.angle, pose }] : windowGeometry.filter((sample) => sample.pose).slice(0, 1);
  const cameraPoints: Vec[] = [{ x: 0, y: 0 }, { x: config.groundX, y: config.groundY }];
  for (const sample of fitSamples) if (sample.pose) cameraPoints.push(sample.pose.A, sample.pose.B, sample.pose.T);
  const xs = cameraPoints.map((point) => point.x), ys = cameraPoints.map((point) => point.y);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
  const W = 900, H = 580, pad = fitMode === "mechanism" ? 115 : 70;
  const baseScale = Math.min((W - 2 * pad) / Math.max(40, maxX - minX), (H - 2 * pad) / Math.max(40, maxY - minY));
  const scale = baseScale * cameraZoom, cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
  const map = (point: Vec): Vec => ({ x: W / 2 + (point.x - cx) * scale, y: H / 2 - (point.y - cy) * scale });
  const fullPath = toolPath(geometry, map), motionPath = toolPath(windowGeometry, map);
  const mapped = pose ? { O2: map(pose.O2), O4: map(pose.O4), A: map(pose.A), B: map(pose.B), T: map(pose.T) } : null;
  const forceMagnitude = Math.hypot(config.forceX, config.forceY);
  const forceEnd = pose && forceMagnitude > 1e-9 ? map(add(pose.T, mul({ x: config.forceX, y: config.forceY }, 36 / forceMagnitude))) : null;

  const setMin = (value: number) => setConfig((current) => ({ ...current, minAngle: value, angle: current.angle < value ? value : current.angle }));
  const setMax = (value: number) => setConfig((current) => ({ ...current, maxAngle: value, angle: current.angle > value ? value : current.angle }));
  const setAnalysisAngle = (angle: number) => { setPlaying(false); setConfig((current) => ({ ...current, angle: clamp(angle, current.minAngle, current.maxAngle) })); };
  const focusResult = (angle: number | null | undefined) => {
    if (angle === null || angle === undefined || !Number.isFinite(angle)) return;
    setAnalysisAngle(angle);
    window.requestAnimationFrame(() => document.getElementById("simulator")?.scrollIntoView({ behavior: "smooth", block: "start" }));
  };
  const selectFit = (mode: FitMode) => { setFitMode(mode); setCameraZoom(1); };
  const resetView = () => { setFitMode(compact ? "mechanism" : "path"); setCameraZoom(1); };
  const exportCsv = () => {
    if (!adaptive) return;
    const url = URL.createObjectURL(new Blob([analysisToCsv(adaptive)], { type: "text/csv;charset=utf-8" }));
    const anchor = document.createElement("a"); anchor.href = url; anchor.download = "robot-leg-linkage-analysis.csv"; anchor.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  };

  const reactions = current.total ? [["O₂", "ground → crank", current.total.O2Reaction], ["A", "coupler → crank", current.total.AReaction], ["B", "rocker → coupler", current.total.BReaction], ["O₄", "ground → rocker", current.total.O4Reaction]] as Array<[string, string, Vec]> : [];
  const nearestSample = (angle: number | null | undefined): CycleSample | null => angle === null || angle === undefined || !samples.length ? null : samples.reduce((best, sample) => Math.abs(sample.angle - angle) < Math.abs(best.angle - angle) ? sample : best, samples[0]);
  const peakReactionSample = nearestSample(adaptive?.peaks.peakJointReaction.angle);
  const peakReactionJoint = peakReactionSample?.dynamics ? [["O₂", magnitude(peakReactionSample.dynamics.O2Reaction)], ["A", magnitude(peakReactionSample.dynamics.AReaction)], ["B", magnitude(peakReactionSample.dynamics.BReaction)], ["O₄", magnitude(peakReactionSample.dynamics.O4Reaction)]].sort((a, b) => (b[1] as number) - (a[1] as number))[0][0] as string : "—";
  const peakUnsafe = (summary?.peakUse ?? 0) > 100, continuousUnsafe = (summary?.continuousUse ?? 0) > 100;
  const shearUnsafe = summary?.shearSafety !== null && summary?.shearSafety !== undefined && summary.shearSafety < 1.5;
  const bearingUnsafe = summary?.bearingSafety !== null && summary?.bearingSafety !== undefined && summary.bearingSafety < 1.5;
  const geometryUnsafe = (summary?.minTransmission ?? 90) < 20;
  const analysisWarning = !valid || !adaptive || adaptive.convergence.status !== "converged" || peakUnsafe || continuousUnsafe || shearUnsafe || bearingUnsafe || geometryUnsafe;
  const loadCase = "total · tool + gravity + inertia";
  const crankMid = mapped ? midpoint(mapped.O2, mapped.A) : null, legMid = mapped ? midpoint(mapped.A, mapped.T) : null, rockerMid = mapped ? midpoint(mapped.O4, mapped.B) : null;

  return <main className="site-shell">
    <header className="topbar"><div className="brand-lockup"><p className="eyebrow">Planar mechanism workbench</p><h1>Robot Leg Linkage Lab</h1></div><div className="header-status"><span className={`status-dot ${!pose ? "danger" : pose.transmission < 35 ? "warning" : "good"}`}/><span>{pose ? "Assembly solved" : "Unreachable pose"}</span><span className="separator">/</span><span>{cls.type}</span></div></header>
    <nav className="mobile-jump" aria-label="Section navigation"><a href="#simulator">Model</a><a href="#inputs">Inputs</a><a href="#results" className={analysisWarning ? "nav-warning" : ""}>Analysis{analysisWarning ? <span className="nav-badge" aria-label="Analysis warning">!</span> : null}</a></nav>
    <div className="workspace">
      <aside id="inputs" className="control-panel"><div className="panel-heading"><div><span className="panel-index">01</span><div><h2>Inputs</h2><p>Draft → validate → commit</p></div></div><button className="text-button" type="button" onClick={() => { setConfig(DEFAULT_CONFIG); setPlaying(false); resetView(); }}>Reset preset</button></div>
        {fieldGroups(config).map((group) => <section className="control-section" key={group.title}><div className="section-label"><h3>{group.title}</h3><span>{group.note}</span></div><div className="field-grid">{group.fields.map((field) => <NumericInput key={field.key} label={field.label} unit={field.unit} step={field.step} min={field.min} max={field.max} integer={field.integer} validate={field.validate} value={config[field.key]} onCommit={(value) => update(field.key, value)}/>)}</div></section>)}
        <section className="control-section compact-section"><div className="section-label"><h3>Assembly & gravity</h3><span>discrete settings</span></div><label className="field"><span>Assembly branch</span><span className="input-wrap select-wrap"><select value={config.branch} onChange={(event: ChangeEvent<HTMLSelectElement>) => update("branch", Number(event.target.value) as Config["branch"])}><option value={-1}>Lower / drawing</option><option value={1}>Upper / alternate</option></select></span></label><label className="toggle-field"><span>Gravity −y</span><input type="checkbox" checked={config.gravity} onChange={(event: ChangeEvent<HTMLInputElement>) => update("gravity", event.target.checked)}/></label></section>
        <section className="mechanism-summary"><div><span>Ground length</span><strong>{cls.ground.toFixed(2)} mm</strong></div><div><span>Grashof margin</span><strong className={cls.margin < 0 ? "value-danger" : ""}>{cls.margin.toFixed(2)} mm</strong></div><div><span>Full rotation</span><strong>{cls.inputRotates && coverage > .99 ? "Yes" : "No"}</strong></div><div><span>Reachable cycle</span><strong>{(coverage * 100).toFixed(0)}%</strong></div></section>
      </aside>
      <section id="simulator" className="analysis-area"><article className="simulator-card"><div className="card-heading"><div><span className="panel-index">02</span><div><h2>Kinematic preview</h2><p>Preview motion only · endpoint reversals excluded from dynamics</p></div></div></div><div className="simulator-body"><div className="simulator-stage">
        <svg viewBox={`0 0 ${W} ${H}`} role="img" aria-label={`Robot leg four-bar linkage. Camera fitted to ${fitMode}.`}><defs><pattern id="grid" width="32" height="32" patternUnits="userSpaceOnUse"><path d="M32 0H0V32" className="grid-line"/></pattern><marker id="force-arrow" markerWidth="10" markerHeight="10" refX="8" refY="3" orient="auto"><path d="M0,0 L0,6 L9,3 z" className="force-arrow-head"/></marker></defs><rect width="100%" height="100%" fill="url(#grid)"/>{fitMode === "path" ? <path d={fullPath} className="tool-path"/> : null}{fitMode === "window" ? <path d={motionPath} className="motion-path"/> : null}
          {mapped && pose ? <><line x1={mapped.O2.x} y1={mapped.O2.y} x2={mapped.O4.x} y2={mapped.O4.y} className="ground-link"/><line x1={mapped.O2.x} y1={mapped.O2.y} x2={mapped.A.x} y2={mapped.A.y} className="link crank-link"/><line x1={mapped.A.x} y1={mapped.A.y} x2={mapped.T.x} y2={mapped.T.y} className="link coupler-link"/><line x1={mapped.O4.x} y1={mapped.O4.y} x2={mapped.B.x} y2={mapped.B.y} className="link rocker-link"/>{[[mapped.O2,"fixed-joint"],[mapped.O4,"fixed-joint"],[mapped.A,"moving-joint"],[mapped.B,"moving-joint"],[mapped.T,"tool-joint"]].map(([point,className],index) => <circle key={index} cx={(point as Vec).x} cy={(point as Vec).y} r={index === 4 ? 10 : 8} className={className as string}/>)}{showLabels && crankMid && legMid && rockerMid ? <g className="link-labels"><text x={crankMid.x} y={crankMid.y - 14} className="link-label crank-label">crank</text><text x={legMid.x} y={legMid.y - 14} className="link-label leg-label">leg</text><text x={rockerMid.x} y={rockerMid.y - 14} className="link-label rocker-label">rocker</text></g> : null}{forceEnd ? <g className="force-vector"><line x1={mapped.T.x} y1={mapped.T.y} x2={forceEnd.x} y2={forceEnd.y} markerEnd="url(#force-arrow)"/><text x={forceEnd.x + 10} y={forceEnd.y - 8}>{forceMagnitude.toFixed(0)} N</text></g> : null}</> : null}
        </svg><div className="camera-toolbar" aria-label="Camera controls"><div className="camera-fit"><button type="button" className={fitMode === "mechanism" ? "active" : ""} onClick={() => selectFit("mechanism")}>Fit mechanism</button><button type="button" className={fitMode === "window" ? "active" : ""} onClick={() => selectFit("window")}>Fit motion window</button><button type="button" className={fitMode === "path" ? "active" : ""} onClick={() => selectFit("path")}>Fit full path</button></div><div className="camera-actions"><button type="button" onClick={() => setCameraZoom((zoom) => clamp(zoom / 1.2, .7, 3))} aria-label="Zoom out">−</button><button type="button" onClick={() => setCameraZoom((zoom) => clamp(zoom * 1.2, .7, 3))} aria-label="Zoom in">+</button><button type="button" onClick={resetView}>Reset</button><button type="button" className={showLabels ? "active" : ""} onClick={() => setShowLabels((shown) => !shown)}>Labels</button></div></div><div className="stage-coordinate">θ₂ {config.angle.toFixed(1)}° · {fitMode}</div></div>
        <div className="motion-rail"><div className="rail-readout"><span>θ₂</span><output>{config.angle.toFixed(1)}°</output></div><div className="rail-range"><RailLimit label="Max" value={config.maxAngle} min={config.minAngle + .5} max={360} onCommit={setMax}/><input id="crank-angle" className="vertical-angle-slider" type="range" min={config.minAngle} max={config.maxAngle} step="0.5" value={config.angle} aria-label="Crank angle" onChange={(event: ChangeEvent<HTMLInputElement>) => setAnalysisAngle(Number(event.target.value))}/><RailLimit label="Min" value={config.minAngle} min={0} max={config.maxAngle - .5} onCommit={setMin}/></div><div className="rail-nudges"><button type="button" onClick={() => setAnalysisAngle(Math.max(config.minAngle, config.angle - 5))}>−5°</button><button type="button" onClick={() => setAnalysisAngle(Math.min(config.maxAngle, config.angle + 5))}>+5°</button></div><button className={`simulator-play ${playing ? "active" : ""}`} type="button" onClick={() => setPlaying((active) => !active)}>{playing ? "Pause" : "Run"}</button></div></div></article>
        <section className="metric-grid"><article className={`metric-card ${pose && pose.transmission < 20 ? "metric-alert" : ""}`}><span>Transmission angle</span><strong>{pose ? `${pose.transmission.toFixed(2)}°` : "—"}</strong><small>{pose ? pose.transmission < 20 ? "Critical geometry" : pose.transmission < 35 ? "Low transmission" : "Geometry usable" : "Unreachable"}</small></article><article className="metric-card"><span>Tool point T</span><strong>{pose ? `${pose.T.x.toFixed(1)}, ${pose.T.y.toFixed(1)}` : "—"}</strong><small>x, y · mm</small></article><article className="metric-card metric-emphasis"><span>Required link torque</span><strong>{current.total ? `${current.total.torque.toFixed(3)} N·m` : "—"}</strong><small>prescribed RPM/α at O₂</small></article><article className="metric-card"><span>Motor torque now</span><strong>{summary?.currentMotorTorque !== null && summary?.currentMotorTorque !== undefined ? `${summary.currentMotorTorque.toFixed(3)} N·m` : "—"}</strong><small>{summary && Math.abs(summary.currentMotorTorque ?? 0) > config.motorPeak ? "Over peak rating" : "Within peak rating"}</small></article></section>
        <section className="state-strip"><div><span>Output θ₄</span><strong>{pose ? `${wrapDegrees(pose.theta4 * DEG).toFixed(2)}°` : "—"}</strong></div><div><span>Velocity ratio ω₄/ω₂</span><strong>{kin ? kin.omega4Ratio.toFixed(3) : "—"}</strong></div><div><span>Tool Jacobian</span><strong>{kin ? `${kin.toolDerivative.x.toFixed(1)}, ${kin.toolDerivative.y.toFixed(1)} mm/rad` : "—"}</strong></div></section>
      </section>
    </div>
    <section id="results" className="results-area"><div className="results-heading"><div><span className="panel-index">03</span><div><h2>Load analysis</h2><p>Adaptive planar inverse dynamics · convergence-aware peaks</p></div></div><button type="button" className="export-button" disabled={!adaptive} onClick={exportCsv}>Download analysis CSV</button></div>{!valid ? <p className="analysis-note value-danger">Analysis paused: {Object.values(errors).join(" ")}</p> : null}{adaptive?.convergence.status !== "converged" ? <p className="analysis-note value-danger">Adaptive solver status: {adaptive?.convergence.status}. Safety factors are withheld when a near-toggle region cannot be resolved.</p> : null}
      <section className="load-metric-grid worst-case-grid" aria-label="Worst-case analysis results">
        <button type="button" className={`load-metric result-jump primary-metric ${peakUnsafe ? "load-warning" : ""}`} disabled={adaptive?.peaks.peakTorque.angle === null || adaptive?.peaks.peakTorque.angle === undefined} onClick={() => focusResult(adaptive?.peaks.peakTorque.angle)}><span>Peak torque</span><strong>{summary ? `${summary.peakTorque.toFixed(2)} N·m` : "—"}</strong><small>{peakMeta(adaptive?.peaks.peakTorque)}</small><small>{loadCase}</small><em className={`result-state ${peakUnsafe ? "danger" : "good"}`}>{peakUnsafe ? "Over peak motor rating" : "Peak rating OK"}</em></button>
        <button type="button" className={`load-metric result-jump ${shearUnsafe || bearingUnsafe ? "load-warning" : ""}`} disabled={adaptive?.peaks.peakJointReaction.angle === null || adaptive?.peaks.peakJointReaction.angle === undefined} onClick={() => focusResult(adaptive?.peaks.peakJointReaction.angle)}><span>Peak joint reaction</span><strong>{summary ? `${summary.peakJointForce.toFixed(0)} N at ${peakReactionJoint}` : "—"}</strong><small>{peakMeta(adaptive?.peaks.peakJointReaction)}</small><small>{loadCase}</small><em className={`result-state ${shearUnsafe || bearingUnsafe ? "danger" : "good"}`}>{shearUnsafe || bearingUnsafe ? "Joint screening warning" : "Joint screen OK"}</em></button>
        <button type="button" className={`load-metric result-jump ${geometryUnsafe ? "load-warning" : ""}`} disabled={adaptive?.peaks.minTransmission.angle === null || adaptive?.peaks.minTransmission.angle === undefined} onClick={() => focusResult(adaptive?.peaks.minTransmission.angle)}><span>Minimum μ</span><strong>{summary ? `${summary.minTransmission.toFixed(2)}°` : "—"}</strong><small>{peakMeta(adaptive?.peaks.minTransmission)}</small><small>geometry · transmission angle</small><em className={`result-state ${geometryUnsafe ? "danger" : "good"}`}>{geometryUnsafe ? "Critical geometry" : "Geometry OK"}</em></button>
        <button type="button" className={`load-metric result-jump ${shearUnsafe ? "load-warning" : ""}`} disabled={adaptive?.peaks.peakJointReaction.angle === null || adaptive?.peaks.peakJointReaction.angle === undefined} onClick={() => focusResult(adaptive?.peaks.peakJointReaction.angle)}><span>Pin shear safety</span><strong>{safetyText(summary, summary?.shearSafety)}</strong><small>{summary ? `${summary.shearStress.toFixed(1)} MPa at reaction peak` : "—"}</small><em className={`result-state ${shearUnsafe ? "danger" : "good"}`}>{summary?.indeterminateNearToggle ? "Indeterminate" : shearUnsafe ? "Low margin" : "Margin OK"}</em></button>
        <button type="button" className={`load-metric result-jump ${bearingUnsafe ? "load-warning" : ""}`} disabled={adaptive?.peaks.peakJointReaction.angle === null || adaptive?.peaks.peakJointReaction.angle === undefined} onClick={() => focusResult(adaptive?.peaks.peakJointReaction.angle)}><span>Link bearing safety</span><strong>{safetyText(summary, summary?.bearingSafety)}</strong><small>{summary ? `${summary.bearingStress.toFixed(1)} MPa at reaction peak` : "—"}</small><em className={`result-state ${bearingUnsafe ? "danger" : "good"}`}>{summary?.indeterminateNearToggle ? "Indeterminate" : bearingUnsafe ? "Low margin" : "Margin OK"}</em></button>
      </section>
      {adaptive ? <section className="plot-grid"><Plot title="Input torque over motion window" unit="N·m at O₂" samples={samples} currentAngle={config.angle} value={(sample) => sample.dynamics?.torque ?? null} onAngleChange={setAnalysisAngle}/><Plot title="Transmission angle" unit="degrees · red zone below 20°" samples={samples} currentAngle={config.angle} value={(sample) => sample.pose?.transmission ?? null} range={[0, 90]} alertBelow={20} onAngleChange={setAnalysisAngle}/></section> : null}
      <section className="load-detail-grid"><article className="detail-card actuator-card"><div className="detail-heading"><div><h3>Actuator & pin screen</h3><span>adaptive coarse-to-fine sampling</span></div><span className="angle-chip">{adaptive?.convergence.status ?? "analysis paused"}</span></div><div className="utilization-block"><div className="utilization-label"><span>Continuous / RMS torque</span><strong>{summary?.continuousUse !== null && summary?.continuousUse !== undefined ? `${summary.continuousUse.toFixed(0)}%` : "—"}</strong></div><div className="utilization-track"><i className={continuousUnsafe ? "over" : ""} style={{ width: `${Math.min(100, Math.max(0, summary?.continuousUse ?? 0))}%` }}/></div><small>{summary?.rmsMotorTorque !== null && summary?.rmsMotorTorque !== undefined ? `${summary.rmsMotorTorque.toFixed(3)} N·m RMS / ${config.motorContinuous.toFixed(2)} N·m continuous` : "No valid RMS result"}</small></div><div className="utilization-block"><div className="utilization-label"><span>Peak torque</span><strong>{summary ? `${summary.peakUse.toFixed(0)}%` : "—"}</strong></div><div className="utilization-track"><i className={peakUnsafe ? "over" : ""} style={{ width: `${Math.min(100, Math.max(0, summary?.peakUse ?? 0))}%` }}/></div><small>{summary ? `${summary.peakMotorTorque.toFixed(3)} N·m peak / ${config.motorPeak.toFixed(2)} N·m peak rating` : "—"}</small></div><div className="screening-grid"><div><span>Peak motor speed</span><strong>{summary ? `${summary.peakMotorSpeedRpm.toFixed(0)} rpm` : "—"}</strong></div><div><span>Peak mechanical power</span><strong>{summary ? `${summary.peakMechanicalPowerW.toFixed(1)} W` : "—"}</strong></div><div><span>Adaptive samples</span><strong>{samples.length}</strong></div><div><span>Peak reaction joint</span><strong>{peakReactionJoint}</strong></div></div></article>
        <article className="detail-card reaction-detail"><div className="detail-heading"><div><h3>Current joint reactions</h3><span>resultant first · signed global components</span></div><span className="angle-chip">θ₂ {config.angle.toFixed(1)}°</span></div><div className="reaction-mobile" aria-label="Joint reactions mobile view">{reactions.length ? reactions.map(([joint, body, vector]) => <details className="reaction-card" key={joint}><summary><span><b>{joint}</b><small>{body}</small></span><span className="reaction-resultant"><small>Resultant</small><strong>{magnitude(vector).toFixed(1)} N</strong></span></summary><div className="reaction-components"><div><span>Fx</span><strong>{vector.x.toFixed(1)} N</strong></div><div><span>Fy</span><strong>{vector.y.toFixed(1)} N</strong></div></div></details>) : <p className="reaction-empty">Current configuration is not dynamically solvable.</p>}</div><div className="table-wrap reaction-desktop"><table><thead><tr><th>Joint / body</th><th>Fx</th><th>Fy</th><th>Resultant</th></tr></thead><tbody>{reactions.length ? reactions.map(([joint, body, vector]) => <tr key={joint}><td>{joint} {body}</td><td>{vector.x.toFixed(1)} N</td><td>{vector.y.toFixed(1)} N</td><td><strong>{magnitude(vector).toFixed(1)} N</strong></td></tr>) : <tr><td colSpan={4}>Current configuration is not dynamically solvable.</td></tr>}</tbody></table></div><div className="torque-breakdown"><div><span>Tool load</span><strong>{current.external?.torque.toFixed(3) ?? "—"} N·m</strong></div><div><span>Gravity</span><strong>{current.gravity?.torque.toFixed(3) ?? "—"} N·m</strong></div><div><span>Inertia</span><strong>{current.inertia?.torque.toFixed(3) ?? "—"} N·m</strong></div></div></article></section>
      <article className="assumption-strip"><strong>Model scope</strong><span>Rigid planar links</span><span>Ideal revolute joints</span><span>Constant configured RPM in dynamics</span><span>Configured constant input α</span><span>Preview endpoint reversal excluded from dynamics</span><span>RMS torque angle/time-weighted for constant speed</span><span>Nominal shear and projected bearing stress only</span></article>
    </section>
  </main>;
}
