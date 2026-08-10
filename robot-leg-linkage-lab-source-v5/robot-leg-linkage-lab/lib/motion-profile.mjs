import { DEG, EPS, RAD, clamp } from "./linkage-geometry.mjs";

function integrateConstantJerk(state, jerk, dt) {
  return {
    position: state.position + state.velocity * dt + 0.5 * state.acceleration * dt ** 2 + jerk * dt ** 3 / 6,
    velocity: state.velocity + state.acceleration * dt + 0.5 * jerk * dt ** 2,
    acceleration: state.acceleration + jerk * dt,
  };
}

export function planSCurveHalf(config) {
  const distance = Math.max(EPS, (config.maxAngle - config.minAngle) * RAD);
  const velocityLimit = Math.max(EPS, config.maxVelocity * RAD);
  const accelerationLimit = Math.max(EPS, config.maxAcceleration * RAD);
  const jerkLimit = Math.max(EPS, config.maxJerk * RAD);

  const jerkTimeAtAccelerationLimit = accelerationLimit / jerkLimit;
  const velocityAtAccelerationLimit = accelerationLimit ** 2 / jerkLimit;
  let jerkTime;
  let accelerationTime = 0;
  let cruiseTime = 0;
  let peakVelocity;
  let peakAcceleration;

  if (velocityLimit <= velocityAtAccelerationLimit) {
    const jerkTimeAtVelocityLimit = Math.sqrt(velocityLimit / jerkLimit);
    const distanceAtVelocityLimit = 2 * jerkLimit * jerkTimeAtVelocityLimit ** 3;
    if (distance >= distanceAtVelocityLimit) {
      jerkTime = jerkTimeAtVelocityLimit;
      peakVelocity = velocityLimit;
      peakAcceleration = jerkLimit * jerkTime;
      cruiseTime = (distance - distanceAtVelocityLimit) / peakVelocity;
    } else {
      jerkTime = Math.cbrt(distance / (2 * jerkLimit));
      peakAcceleration = jerkLimit * jerkTime;
      peakVelocity = jerkLimit * jerkTime ** 2;
    }
  } else {
    const accelerationTimeAtVelocityLimit = velocityLimit / accelerationLimit - jerkTimeAtAccelerationLimit;
    const accelerationDurationAtVelocityLimit = accelerationTimeAtVelocityLimit + 2 * jerkTimeAtAccelerationLimit;
    const distanceAtVelocityLimit = velocityLimit * accelerationDurationAtVelocityLimit;
    const distanceAtAccelerationLimit = 2 * accelerationLimit ** 3 / jerkLimit ** 2;

    if (distance >= distanceAtVelocityLimit) {
      jerkTime = jerkTimeAtAccelerationLimit;
      accelerationTime = accelerationTimeAtVelocityLimit;
      peakAcceleration = accelerationLimit;
      peakVelocity = velocityLimit;
      cruiseTime = (distance - distanceAtVelocityLimit) / peakVelocity;
    } else if (distance >= distanceAtAccelerationLimit) {
      jerkTime = jerkTimeAtAccelerationLimit;
      accelerationTime = (-3 * jerkTime + Math.sqrt(jerkTime ** 2 + 4 * distance / accelerationLimit)) / 2;
      peakAcceleration = accelerationLimit;
      peakVelocity = accelerationLimit * (accelerationTime + jerkTime);
    } else {
      jerkTime = Math.cbrt(distance / (2 * jerkLimit));
      peakAcceleration = jerkLimit * jerkTime;
      peakVelocity = jerkLimit * jerkTime ** 2;
    }
  }

  const durations = [jerkTime, accelerationTime, jerkTime, cruiseTime, jerkTime, accelerationTime, jerkTime];
  const jerks = [jerkLimit, 0, -jerkLimit, 0, -jerkLimit, 0, jerkLimit];
  const boundaries = [0];
  for (const duration of durations) boundaries.push(boundaries.at(-1) + duration);
  const duration = boundaries.at(-1);

  return {
    distance,
    duration,
    jerkTime,
    accelerationTime,
    cruiseTime,
    peakVelocity,
    peakAcceleration,
    peakJerk: jerkLimit,
    durations,
    jerks,
    boundaries,
  };
}

function stateOnSCurveHalf(plan, time) {
  let remaining = clamp(time, 0, plan.duration);
  let state = { position: 0, velocity: 0, acceleration: 0 };
  let jerk = 0;
  for (let index = 0; index < plan.durations.length; index += 1) {
    const duration = plan.durations[index];
    jerk = plan.jerks[index];
    const dt = Math.min(remaining, duration);
    state = integrateConstantJerk(state, jerk, dt);
    remaining -= dt;
    if (remaining <= EPS) return { ...state, jerk };
  }
  return { position: plan.distance, velocity: 0, acceleration: 0, jerk: 0 };
}

export function motionProfileInfo(config) {
  if (config.motionProfile === "sinusoidal") {
    const duration = Math.max(EPS, config.cycleTime);
    const amplitude = (config.maxAngle - config.minAngle) * RAD / 2;
    const angularFrequency = 2 * Math.PI / duration;
    return {
      type: "sinusoidal",
      duration,
      halfDuration: duration / 2,
      peakVelocity: amplitude * angularFrequency,
      peakAcceleration: amplitude * angularFrequency ** 2,
      peakJerk: amplitude * angularFrequency ** 3,
      peakVelocityDegS: amplitude * angularFrequency * DEG,
      peakAccelerationDegS2: amplitude * angularFrequency ** 2 * DEG,
      peakJerkDegS3: amplitude * angularFrequency ** 3 * DEG,
      plan: null,
    };
  }

  const plan = planSCurveHalf(config);
  return {
    type: "s-curve",
    duration: 2 * plan.duration,
    halfDuration: plan.duration,
    peakVelocity: plan.peakVelocity,
    peakAcceleration: plan.peakAcceleration,
    peakJerk: plan.peakJerk,
    peakVelocityDegS: plan.peakVelocity * DEG,
    peakAccelerationDegS2: plan.peakAcceleration * DEG,
    peakJerkDegS3: plan.peakJerk * DEG,
    plan,
  };
}

export function motionStateAtTime(config, time) {
  const info = motionProfileInfo(config);
  const t = clamp(Number.isFinite(time) ? time : 0, 0, info.duration);
  const minRad = config.minAngle * RAD;
  const maxRad = config.maxAngle * RAD;

  if (info.type === "sinusoidal") {
    const center = (minRad + maxRad) / 2;
    const amplitude = (maxRad - minRad) / 2;
    const w = 2 * Math.PI / info.duration;
    const theta = center - amplitude * Math.cos(w * t);
    const omega = amplitude * w * Math.sin(w * t);
    const alpha = amplitude * w ** 2 * Math.cos(w * t);
    const jerk = -amplitude * w ** 3 * Math.sin(w * t);
    return { time: t, phase: t / info.duration, angle: theta * DEG, theta, omega, alpha, jerk };
  }

  const outbound = t <= info.halfDuration;
  const localTime = outbound ? t : t - info.halfDuration;
  const state = stateOnSCurveHalf(info.plan, localTime);
  const theta = outbound ? minRad + state.position : maxRad - state.position;
  const sign = outbound ? 1 : -1;
  return {
    time: t,
    phase: t / info.duration,
    angle: theta * DEG,
    theta,
    omega: sign * state.velocity,
    alpha: sign * state.acceleration,
    jerk: sign * state.jerk,
  };
}

export function sampleMotionProfile(config, options = {}) {
  const info = motionProfileInfo(config);
  const sampleCount = Math.max(81, Math.floor(options.sampleCount ?? 401));
  const times = new Set();
  for (let index = 0; index < sampleCount; index += 1) times.add(info.duration * index / (sampleCount - 1));
  times.add(0);
  times.add(info.halfDuration);
  times.add(info.duration);
  if (info.type === "s-curve" && info.plan) {
    for (const boundary of info.plan.boundaries) {
      times.add(boundary);
      times.add(info.halfDuration + boundary);
    }
  }
  const sorted = [...times]
    .filter((time) => time >= -EPS && time <= info.duration + EPS)
    .sort((a, b) => a - b);
  const unique = [];
  const tolerance = Math.max(1e-12, info.duration * 1e-12);
  for (const time of sorted) {
    const clamped = clamp(time, 0, info.duration);
    if (!unique.length || Math.abs(clamped - unique.at(-1)) > tolerance) unique.push(clamped);
  }
  return unique.map((time) => motionStateAtTime(config, time));
}

export function timeAtAngle(config, angleDegrees) {
  const info = motionProfileInfo(config);
  const target = clamp(angleDegrees, config.minAngle, config.maxAngle);
  if (Math.abs(target - config.minAngle) < 1e-10) return 0;
  if (Math.abs(target - config.maxAngle) < 1e-10) return info.halfDuration;
  let low = 0;
  let high = info.halfDuration;
  for (let iteration = 0; iteration < 64; iteration += 1) {
    const mid = (low + high) / 2;
    if (motionStateAtTime(config, mid).angle < target) low = mid;
    else high = mid;
  }
  return (low + high) / 2;
}
