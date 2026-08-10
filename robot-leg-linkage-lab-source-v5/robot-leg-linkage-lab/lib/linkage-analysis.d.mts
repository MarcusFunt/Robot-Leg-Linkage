export type Vec = { x: number; y: number };
export type Pose = {
  O2: Vec; O4: Vec; A: Vec; B: Vec; T: Vec;
  theta2: number; theta3: number; theta4: number; transmission: number;
};
export type Dynamics = {
  torque: number; O2Reaction: Vec; AReaction: Vec; BReaction: Vec; O4Reaction: Vec; alpha3: number; alpha4: number;
};
export type DynamicsBreakdown = { total: Dynamics | null; external: Dynamics | null; gravity: Dynamics | null; inertia: Dynamics | null };
export type CycleSample = {
  angle: number; pose: Pose | null; dynamics: Dynamics | null; externalTorque: number | null; gravityTorque: number | null; inertiaTorque: number | null; jointReaction: number | null;
};
export type PeakResult = { value: number | null; angle: number | null; convergence: string; resolutionDeg: number | null };
export type AdaptiveAnalysis = {
  samples: CycleSample[];
  peaks: { peakTorque: PeakResult; peakJointReaction: PeakResult; minTransmission: PeakResult };
  convergence: { status: string; unresolvedIntervals: unknown[]; maxDepth: number; angleToleranceDeg: number };
};
export type AnalysisSummary = {
  valid: CycleSample[];
  peakTorque: number; peakJointForce: number; minTransmission: number;
  currentMotorTorque: number | null; peakMotorTorque: number; rmsMotorTorque: number | null;
  peakMotorSpeedRpm: number; peakMechanicalPowerW: number;
  shearStress: number; bearingStress: number; shearSafety: number | null; bearingSafety: number | null;
  continuousUse: number | null; peakUse: number; indeterminateNearToggle: boolean;
};
export type MechanismClass = { ground: number; margin: number; grashof: boolean; type: string; inputRotates: boolean };
export const DEG: number;
export function adaptiveAnalyze(config: object, options?: object): AdaptiveAnalysis;
export function add(a: Vec, b: Vec): Vec;
export function analysisToCsv(analysis: AdaptiveAnalysis): string;
export function clamp(value: number, minimum: number, maximum: number): number;
export function dynamicsBreakdown(config: object, angleDegrees?: number): DynamicsBreakdown;
export function kinematics(config: object, pose?: Pose | null): { omega3Ratio: number; omega4Ratio: number; toolDerivative: Vec } | null;
export function magnitude(vector: Vec): number;
export function mechanismClass(config: object): MechanismClass;
export function mul(a: Vec, scalar: number): Vec;
export function sampleAngleRange(minimum: number, maximum: number, step: number): number[];
export function solvePose(config: object, angleDegrees?: number): Pose | null;
export function summarizeAnalysis(config: object, analysis: AdaptiveAnalysis, currentDynamics?: DynamicsBreakdown | null): AnalysisSummary;
export function wrapDegrees(value: number): number;
