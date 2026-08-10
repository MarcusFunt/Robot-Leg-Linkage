"use client";

import { useEffect, useState } from "react";
import { createPortal } from "react-dom";

type SectionId = "simulator" | "inputs" | "results";
type AnalysisView = "status" | "dynamic" | "static" | "actuator";
type CheckState = "critical" | "warning" | "pass" | "neutral";
type CheckSummary = {
  index: number;
  label: string;
  detail: string;
  state: CheckState;
};

const STORAGE_KEY = "robot-leg-linkage:advanced-inputs";

function resultChecks(): CheckSummary[] {
  const buttons = Array.from(
    document.querySelectorAll<HTMLButtonElement>("#results .worst-case-grid .result-jump"),
  );

  return buttons.map((button, index) => {
    const label = Array.from(button.children).find((child) => child.tagName === "SPAN")?.textContent?.trim() ?? "Analysis check";
    const stateNode = button.querySelector<HTMLElement>(".result-state");
    const detail = stateNode?.textContent?.trim() ?? "";
    const normalized = detail.toLowerCase();
    const explicitDanger = button.classList.contains("load-warning") || stateNode?.classList.contains("danger");
    const indeterminate = normalized.includes("indeterminate") || normalized.includes("paused") || normalized.includes("invalid");
    const critical = normalized.includes("critical") || normalized.includes("over ") || normalized.includes("overload") || indeterminate;
    const warning = explicitDanger || normalized.includes("warning") || normalized.includes("low margin");
    const pass = stateNode?.classList.contains("good") && !indeterminate;
    const state: CheckState = critical ? "critical" : warning ? "warning" : pass ? "pass" : "neutral";

    button.dataset.uxState = state;
    button.dataset.uxIndex = String(index);
    return { index, label, detail, state };
  });
}

export default function UXEnhancements() {
  const [navTarget, setNavTarget] = useState<HTMLElement | null>(null);
  const [inputTarget, setInputTarget] = useState<HTMLElement | null>(null);
  const [resultsTarget, setResultsTarget] = useState<HTMLElement | null>(null);
  const [activeSection, setActiveSection] = useState<SectionId>("simulator");
  const [advancedOpen, setAdvancedOpen] = useState(false);
  const [showPassed, setShowPassed] = useState(false);
  const [analysisView, setAnalysisView] = useState<AnalysisView>("status");
  const [profileInfoTarget, setProfileInfoTarget] = useState<HTMLElement | null>(null);
  const [checks, setChecks] = useState<CheckSummary[]>([]);

  useEffect(() => {
    setNavTarget(document.querySelector<HTMLElement>(".topbar"));
    setInputTarget(document.querySelector<HTMLElement>("#inputs .panel-heading"));
    setResultsTarget(document.querySelector<HTMLElement>("#results"));
    setProfileInfoTarget(document.querySelector<HTMLElement>(".motion-profile-section"));

    const saved = window.localStorage.getItem(STORAGE_KEY) === "1";
    setAdvancedOpen(saved);
    document.documentElement.classList.toggle("ux-advanced-open", saved);

    const updateActive = () => {
      const threshold = window.innerHeight * 0.34;
      let current: SectionId = "simulator";
      for (const id of ["simulator", "inputs", "results"] as SectionId[]) {
        const element = document.getElementById(id);
        if (element && element.getBoundingClientRect().top <= threshold) current = id;
      }
      setActiveSection(current);
    };

    updateActive();
    window.addEventListener("scroll", updateActive, { passive: true });
    window.addEventListener("resize", updateActive);
    return () => {
      window.removeEventListener("scroll", updateActive);
      window.removeEventListener("resize", updateActive);
      document.documentElement.classList.remove("ux-advanced-open", "ux-show-passes");
    };
  }, []);

  useEffect(() => {
    document.documentElement.classList.toggle("ux-show-passes", showPassed);
  }, [showPassed]);
  useEffect(() => {
    document.documentElement.dataset.analysisView = analysisView;
    const results = document.getElementById("results");
    if (!results) return;
    const headings = Array.from(results.querySelectorAll<HTMLElement>(".section-wide-heading"));
    for (const heading of headings) {
      const title = heading.querySelector("h2")?.textContent?.toLowerCase() ?? "";
      const group = title.includes("dynamic") ? "dynamic" : title.includes("static") ? "static" : null;
      if (group) {
        heading.dataset.analysisGroup = group;
        const next = heading.nextElementSibling as HTMLElement | null;
        if (next?.classList.contains("plot-grid")) next.dataset.analysisGroup = group;
      }
    }
    const detail = results.querySelector<HTMLElement>(".load-detail-grid");
    if (detail) detail.dataset.analysisGroup = "actuator";
    const assumptions = results.querySelector<HTMLElement>(".assumption-strip");
    if (assumptions) assumptions.dataset.analysisGroup = "actuator";
  }, [analysisView, resultsTarget]);


  useEffect(() => {
    if (!resultsTarget) return;
    const refresh = () => setChecks(resultChecks());
    refresh();
    const observer = new MutationObserver(refresh);
    observer.observe(resultsTarget, {
      subtree: true,
      childList: true,
      characterData: true,
      attributes: true,
      attributeFilter: ["class", "disabled"],
    });
    return () => observer.disconnect();
  }, [resultsTarget]);

  const toggleAdvanced = () => {
    const next = !advancedOpen;
    setAdvancedOpen(next);
    document.documentElement.classList.toggle("ux-advanced-open", next);
    window.localStorage.setItem(STORAGE_KEY, next ? "1" : "0");
  };

  const issues = checks.filter((check) => check.state === "critical" || check.state === "warning");
  const passed = checks.filter((check) => check.state === "pass");
  useEffect(() => {
    if (!issues.length || analysisView !== "status") return;
    const label = issues[0].label.toLowerCase();
    if (label.includes("transmission") || label.includes("minimum μ") || label.includes("geometry") || label.includes("static")) setAnalysisView("static");
    else if (label.includes("motor") || label.includes("shear") || label.includes("bearing")) setAnalysisView("actuator");
    else setAnalysisView("dynamic");
  }, [checks, analysisView]);

  const focusCheck = (index: number) => {
    const button = document.querySelector<HTMLButtonElement>(`#results .result-jump[data-ux-index="${index}"]`);
    if (button && !button.disabled) button.click();
    else button?.scrollIntoView({ behavior: "smooth", block: "center" });
  };

  return (
    <>
      {navTarget
        ? createPortal(
            <nav className="ux-section-nav" aria-label="Primary sections">
              {([
                ["simulator", "Simulate"],
                ["inputs", "Configure"],
                ["results", "Analyze"],
              ] as Array<[SectionId, string]>).map(([id, label]) => (
                <a key={id} href={`#${id}`} className={activeSection === id ? "active" : ""} aria-current={activeSection === id ? "page" : undefined}>
                  {label}
                  {id === "results" && issues.length > 0 ? <span className="ux-nav-count">{issues.length}</span> : null}
                </a>
              ))}
            </nav>,
            navTarget,
          )
        : null}

      {inputTarget
        ? createPortal(
            <button type="button" className="ux-advanced-toggle" aria-expanded={advancedOpen} onClick={toggleAdvanced}>
              {advancedOpen ? "Hide advanced" : "Advanced inputs"}
            </button>,
            inputTarget,
          )
        : null}

      {profileInfoTarget
        ? createPortal(
            <details className="ux-profile-info">
              <summary>About this motion profile</summary>
              <p>S-curve uses the minimum-time symmetric jerk-limited trajectory within the configured limits. Sinusoidal motion starts at the minimum angle, reaches the maximum at half-cycle, then returns.</p>
            </details>,
            profileInfoTarget,
          )
        : null}

      {resultsTarget
        ? createPortal(
            <section className={`ux-design-status ${issues.length ? "has-issues" : "all-good"}`} aria-live="polite">
              <div className="ux-status-copy">
                <span>Design status</span>
                <strong>{issues.length ? `${issues.length} issue${issues.length === 1 ? "" : "s"} · ${passed.length} checks passed` : "All screened checks passed"}</strong>
                <small>{issues.length ? "Review the highest-risk conditions first. Each issue jumps to its exact analyzed condition." : "No currently screened limit is reporting a warning or failure."}</small>
              </div>

              {issues.length ? (
                <div className="ux-issue-list" aria-label="Issues needing attention">
                  {issues.map((issue) => (
                    <button key={`${issue.index}-${issue.label}`} type="button" className={`ux-issue ${issue.state}`} onClick={() => focusCheck(issue.index)}>
                      <span>{issue.state === "critical" ? "Critical" : "Warning"}</span>
                      <strong>{issue.label}</strong>
                      <small>{issue.detail || "View condition"}</small>
                    </button>
                  ))}
                </div>
              ) : null}

              {passed.length ? (
                <button type="button" className="ux-passed-toggle" aria-expanded={showPassed} onClick={() => setShowPassed((current) => !current)}>
                  {showPassed ? "Hide passed checks" : `Show ${passed.length} passed check${passed.length === 1 ? "" : "s"}`}
                </button>
              ) : null}
              <nav className="ux-analysis-nav" aria-label="Analysis sections">
                {([['status','Status'],['dynamic','Dynamic'],['static','Static'],['actuator','Actuator']] as Array<[AnalysisView,string]>).map(([id,label]) => (
                  <button key={id} type="button" className={analysisView===id?'active':''} onClick={() => setAnalysisView(id)}>{label}</button>
                ))}
              </nav>
            </section>,
            resultsTarget,
          )
        : null}
    </>
  );
}
