// Compatibility surface for the pre-refactor analysis module.
// The implemented pipeline is now split into static, motion, actuator, and export layers.
export { analyzeMotion } from "./linkage-motion-analysis.mjs";
export { analyzeStaticSupport } from "./linkage-static-support.mjs";
export { summarizeAnalysis } from "./linkage-actuator-analysis.mjs";
export { analysisToCsv } from "./linkage-export.mjs";
