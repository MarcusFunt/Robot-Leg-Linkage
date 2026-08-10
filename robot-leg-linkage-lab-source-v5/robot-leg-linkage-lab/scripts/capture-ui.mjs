#!/usr/bin/env node

import { existsSync } from "node:fs";
import { mkdir, writeFile } from "node:fs/promises";
import path from "node:path";
import { pathToFileURL } from "node:url";
import { spawnSync } from "node:child_process";

const argv = process.argv.slice(2);
const PLAYWRIGHT_VERSION = process.env.PLAYWRIGHT_VERSION ?? "1.62.0";
const RUNTIME_DIR = path.resolve(".ui-screenshot-runtime");
const INSTALL_ONLY = argv.includes("--install-only");
const MAXIMAL = argv.includes("--maximal");

function argValue(name, fallback) {
  const index = argv.indexOf(name);
  if (index === -1 || !argv[index + 1]) return fallback;
  return argv[index + 1];
}

const BASE_URL = argValue("--url", "http://127.0.0.1:4173/");
const OUTPUT_DIR = path.resolve(argValue("--out", "screenshots/ui"));

const TARGETS = [
  { name: "desktop-2560x1440", width: 2560, height: 1440, dpr: 2 },
  { name: "desktop-1920x1080", width: 1920, height: 1080, dpr: 2 },
  { name: "desktop-1440x1000", width: 1440, height: 1000, dpr: 2 },
  { name: "tablet-768x1024", width: 768, height: 1024, dpr: 2, isMobile: true, hasTouch: true },
  { name: "phone-430x932", width: 430, height: 932, dpr: 3, isMobile: true, hasTouch: true },
  { name: "phone-390x844", width: 390, height: 844, dpr: 3, isMobile: true, hasTouch: true },
];

const PROFILES = [
  { id: "s-curve", label: "S-curve" },
  { id: "sinusoidal", label: "Sinusoidal" },
];

const CORE_SECTIONS = [
  ["topbar", ".topbar"],
  ["simulator", ".simulator-card"],
  ["inputs", "#inputs"],
  ["current-metrics", ".metric-grid"],
  ["state-strip", ".state-strip"],
  ["profile-derived", ".profile-derived"],
  ["support-metrics", ".support-metric-grid"],
  ["design-status", ".ux-design-status"],
  ["results", ".results-area"],
  ["actuator-and-reactions", ".load-detail-grid"],
  ["assumptions", ".assumption-strip"],
];

const SCREENSHOT_STYLE = `
  *, *::before, *::after {
    animation: none !important;
    transition: none !important;
    caret-color: transparent !important;
  }
  html { scroll-behavior: auto !important; }
`;

const manifest = {
  generatedAt: new Date().toISOString(),
  source: BASE_URL,
  playwrightVersion: PLAYWRIGHT_VERSION,
  maximal: MAXIMAL,
  captures: [],
  failures: [],
};

function slug(value) {
  return String(value)
    .trim()
    .toLowerCase()
    .normalize("NFKD")
    .replace(/[^a-z0-9]+/g, "-")
    .replace(/^-+|-+$/g, "")
    .slice(0, 100) || "untitled";
}

function relativeOutput(file) {
  return path.relative(OUTPUT_DIR, file).split(path.sep).join("/");
}

function runCommand(command, args) {
  const result = spawnSync(command, args, { stdio: "inherit" });
  if (result.error) {
    throw new Error(`${command} failed to start: ${result.error.message}`, { cause: result.error });
  }
  if (result.status !== 0) {
    throw new Error(`${command} ${args.join(" ")} exited with status ${result.status ?? "unknown"}.`);
  }
}

function runNpm(args) {
  const npmCli = process.env.npm_execpath;
  if (npmCli && existsSync(npmCli)) {
    runCommand(process.execPath, [npmCli, ...args]);
    return;
  }

  if (process.platform === "win32") {
    const commandProcessor = process.env.ComSpec ?? "cmd.exe";
    runCommand(commandProcessor, ["/d", "/s", "/c", "npm.cmd", ...args]);
    return;
  }

  runCommand("npm", args);
}

async function ensurePlaywrightRuntime() {
  await mkdir(RUNTIME_DIR, { recursive: true });
  const runtimePackage = path.join(RUNTIME_DIR, "package.json");
  if (!existsSync(runtimePackage)) {
    await writeFile(runtimePackage, '{"private":true,"type":"module"}\n', "utf8");
  }

  const moduleEntry = path.join(RUNTIME_DIR, "node_modules", "playwright", "index.mjs");
  const cliEntry = path.join(RUNTIME_DIR, "node_modules", "playwright", "cli.js");

  if (!existsSync(moduleEntry)) {
    console.log(`Installing Playwright ${PLAYWRIGHT_VERSION} into ${RUNTIME_DIR}`);
    runNpm([
      "install",
      "--prefix",
      RUNTIME_DIR,
      "--no-package-lock",
      "--no-save",
      `playwright@${PLAYWRIGHT_VERSION}`,
    ]);
  }

  const playwright = await import(pathToFileURL(moduleEntry).href);
  const browserExecutable = playwright.chromium.executablePath();
  if (!existsSync(browserExecutable)) {
    console.log("Installing the Playwright Chromium browser...");
    runCommand(process.execPath, [cliEntry, "install", "chromium"]);
  }

  return playwright;
}

async function ensureParent(file) {
  await mkdir(path.dirname(file), { recursive: true });
}

async function stabilize(page) {
  await page.waitForLoadState("networkidle", { timeout: 5000 }).catch(() => {});
  await page.evaluate(async () => {
    if (document.fonts?.ready) await document.fonts.ready;
  });
  await page.waitForTimeout(150);
}

async function capturePage(page, file, metadata = {}) {
  await ensureParent(file);
  const common = {
    path: file,
    type: "png",
    fullPage: true,
    animations: "disabled",
    caret: "hide",
    style: SCREENSHOT_STYLE,
  };

  try {
    await page.screenshot({ ...common, scale: "device" });
    manifest.captures.push({ file: relativeOutput(file), scale: "device", ...metadata });
  } catch (error) {
    console.warn(`High-DPI full-page capture failed for ${file}; retrying at CSS scale.`);
    await page.screenshot({ ...common, scale: "css" });
    manifest.captures.push({ file: relativeOutput(file), scale: "css-fallback", ...metadata });
  }
}

async function captureElement(locator, file, metadata = {}) {
  if ((await locator.count()) === 0) return false;
  const element = locator.first();
  if (!(await element.isVisible())) return false;
  await ensureParent(file);

  const common = {
    path: file,
    type: "png",
    animations: "disabled",
    caret: "hide",
    style: SCREENSHOT_STYLE,
  };

  try {
    await element.screenshot({ ...common, scale: "device" });
    manifest.captures.push({ file: relativeOutput(file), scale: "device", ...metadata });
  } catch (error) {
    console.warn(`High-DPI element capture failed for ${file}; retrying at CSS scale.`);
    await element.screenshot({ ...common, scale: "css" });
    manifest.captures.push({ file: relativeOutput(file), scale: "css-fallback", ...metadata });
  }

  return true;
}

async function gotoBaseline(page) {
  const response = await page.goto(BASE_URL, { waitUntil: "domcontentloaded", timeout: 30000 });
  if (response && !response.ok()) {
    throw new Error(`Page returned HTTP ${response.status()} for ${BASE_URL}`);
  }
  await page.locator(".site-shell").waitFor({ state: "visible", timeout: 10000 });
  await stabilize(page);
}

async function ensureProfileSelectorVisible(page) {
  const selector = page.locator(".motion-profile-selector").first();
  if ((await selector.count()) > 0 && (await selector.isVisible())) return;

  const tabs = page.locator(".control-tabs button");
  const count = await tabs.count();
  for (let index = 0; index < count; index += 1) {
    await tabs.nth(index).click();
    await stabilize(page);
    if ((await selector.count()) > 0 && (await selector.isVisible())) return;
  }

  throw new Error("Could not reveal the motion-profile selector from the control tabs.");
}

async function selectProfile(page, profile) {
  await ensureProfileSelectorVisible(page);
  const buttons = page.locator(".motion-profile-selector button");
  const count = await buttons.count();
  for (let index = 0; index < count; index += 1) {
    const text = (await buttons.nth(index).innerText()).trim();
    if (text.toLowerCase().includes(profile.label.toLowerCase())) {
      await buttons.nth(index).click();
      await stabilize(page);
      return;
    }
  }
  throw new Error(`Could not find motion profile button: ${profile.label}`);
}

async function setNumericField(page, label, value) {
  const field = page.locator('.field').filter({ hasText: label }).first();
  if ((await field.count()) === 0) throw new Error(`Field not found: ${label}`);
  const input = field.locator('input[type="number"]').first();
  await input.fill(String(value));
  await input.press('Enter');
  await stabilize(page);
}

async function captureRiskStates(page, directory, metadata) {
  const scenarios = [
    { name: 'one-warning-geometry', values: [['Vertical support at T', 0], ['Maximum angle', 306]] },
    { name: 'multiple-warnings', values: [['Allowable bearing', 1], ['Pin diameter', 1], ['Motor continuous', 0.02], ['Motor peak', 0.05]] },
    { name: 'actuator-overload', values: [['Motor continuous', 0.02], ['Motor peak', 0.05]] },
    { name: 'invalid-motion', values: [['Input crank', 180]] },
  ];
  for (const scenario of scenarios) {
    await gotoBaseline(page);
    const advanced = page.getByRole('button', { name: /advanced inputs/i }).first();
    if ((await advanced.count()) > 0 && !(await advanced.getAttribute('aria-expanded') === 'true')) await advanced.click();
    for (const [label, value] of scenario.values) await setNumericField(page, label, value);
    await captureElement(page.locator('.ux-design-status'), path.join(directory, 'risk-states', `${scenario.name}.png`), { ...metadata, kind: 'risk-state', scenario: scenario.name });
  }
}

async function setTimeFraction(page, fraction) {
  const slider = page.locator('.time-rail input[type="range"], .motion-rail input[type="range"]').first();
  if ((await slider.count()) === 0) return;

  await slider.evaluate((element, requestedFraction) => {
    const min = Number(element.min || 0);
    const max = Number(element.max || 100);
    element.value = String(min + (max - min) * Number(requestedFraction));
    element.dispatchEvent(new Event("input", { bubbles: true }));
    element.dispatchEvent(new Event("change", { bubbles: true }));
  }, fraction);

  await stabilize(page);
}

async function captureSimulatorStates(page, directory, metadata) {
  const simulator = page.locator(".simulator-card");
  const states = [
    ["start", 0],
    ["quarter", 0.25],
    ["mid", 0.5],
    ["three-quarter", 0.75],
    ["end", 1],
  ];

  for (const [name, fraction] of states) {
    await setTimeFraction(page, fraction);
    await captureElement(simulator, path.join(directory, `simulator-${name}.png`), {
      ...metadata,
      kind: "simulator-state",
      timeFraction: fraction,
    });
  }
  await setTimeFraction(page, 0);
}

async function captureCoreSections(page, directory, metadata) {
  for (const [name, selector] of CORE_SECTIONS) {
    await captureElement(page.locator(selector), path.join(directory, `section-${name}.png`), {
      ...metadata,
      kind: "section",
      section: name,
    });
  }

  const details = page.locator(".detail-card");
  const count = await details.count();
  for (let index = 0; index < count; index += 1) {
    const card = details.nth(index);
    const heading = ((await card.locator("h3").first().textContent().catch(() => null)) ?? `detail-${index + 1}`).trim();
    await captureElement(card, path.join(directory, "detail-cards", `${String(index + 1).padStart(2, "0")}-${slug(heading)}.png`), {
      ...metadata,
      kind: "detail-card",
      detail: heading,
    });
  }
}

async function captureControlTabs(page, directory, metadata) {
  const tabs = page.locator(".control-tabs button");
  const count = await tabs.count();
  if (count === 0) return;

  for (let index = 0; index < count; index += 1) {
    const tab = tabs.nth(index);
    const label = ((await tab.locator("span").first().textContent().catch(() => null)) ?? (await tab.textContent()) ?? `tab-${index + 1}`).trim();
    await tab.click();
    await stabilize(page);
    await captureElement(page.locator(".control-panel"), path.join(directory, "input-tabs", `${String(index + 1).padStart(2, "0")}-${slug(label)}.png`), {
      ...metadata,
      kind: "input-tab",
      tab: label,
    });

    const visibleDetails = page.locator(".control-panel details:visible");
    if ((await visibleDetails.count()) > 0) {
      await visibleDetails.evaluateAll((elements) => {
        for (const element of elements) element.open = true;
      });
      await stabilize(page);
      await captureElement(page.locator(".control-panel"), path.join(directory, "input-tabs", `${String(index + 1).padStart(2, "0")}-${slug(label)}-details-open.png`), {
        ...metadata,
        kind: "input-tab-details-open",
        tab: label,
      });
    }
  }
}

async function captureAllDetailsOpen(page, directory, metadata) {
  await page.locator("details").evaluateAll((elements) => {
    for (const element of elements) element.open = true;
  });
  await stabilize(page);

  await capturePage(page, path.join(directory, "full-page-all-details-open.png"), {
    ...metadata,
    kind: "full-page-all-details-open",
  });

  await captureElement(page.locator("#inputs"), path.join(directory, "inputs-all-details-open.png"), {
    ...metadata,
    kind: "all-details-open",
    section: "inputs",
  });

  await captureElement(page.locator(".results-area"), path.join(directory, "results-all-details-open.png"), {
    ...metadata,
    kind: "all-details-open",
    section: "results",
  });
}

function shouldCaptureFullscreen(target) {
  if (MAXIMAL) return true;
  return target.name === "desktop-1440x1000" || target.name === "phone-390x844";
}

async function capturePlots(page, directory, metadata, captureFullscreen) {
  await page.evaluate(() => document.documentElement.removeAttribute("data-analysis-view"));
  const cards = page.locator(".plot-card");
  const count = await cards.count();

  for (let index = 0; index < count; index += 1) {
    const card = cards.nth(index);
    const heading = ((await card.locator("h3").first().textContent()) ?? `plot-${index + 1}`).trim();
    const name = `${String(index + 1).padStart(2, "0")}-${slug(heading)}`;

    await captureElement(card, path.join(directory, "plots", `${name}-card.png`), {
      ...metadata,
      kind: "plot-card",
      plot: heading,
    });

    if (!captureFullscreen) continue;

    const fullscreen = card.getByRole("button", { name: /fullscreen/i }).first();
    if ((await fullscreen.count()) === 0) continue;
    await fullscreen.click();

    const dialog = page.getByRole("dialog").last();
    await dialog.waitFor({ state: "visible", timeout: 5000 });
    await stabilize(page);

    await captureElement(dialog, path.join(directory, "plots", `${name}-fullscreen-dialog.png`), {
      ...metadata,
      kind: "plot-fullscreen-dialog",
      plot: heading,
    });

    await captureElement(dialog.locator(".plot-svg").first(), path.join(directory, "plots", `${name}-fullscreen-plot.png`), {
      ...metadata,
      kind: "plot-fullscreen-svg",
      plot: heading,
    });

    const close = dialog.getByRole("button", { name: /close/i }).first();
    if ((await close.count()) > 0) await close.click();
    else await page.keyboard.press("Escape");
    await dialog.waitFor({ state: "hidden", timeout: 5000 }).catch(() => {});
    await stabilize(page);
  }
}

async function captureTarget(browser, target) {
  console.log(`\n=== ${target.name}: ${target.width}×${target.height} @ ${target.dpr}× ===`);
  const context = await browser.newContext({
    viewport: { width: target.width, height: target.height },
    screen: { width: target.width, height: target.height },
    deviceScaleFactor: target.dpr,
    isMobile: Boolean(target.isMobile),
    hasTouch: Boolean(target.hasTouch),
    reducedMotion: "reduce",
    colorScheme: "light",
    locale: "en-US",
  });

  const page = await context.newPage();
  page.on("console", (message) => {
    if (message.type() === "error") console.error(`[browser console] ${message.text()}`);
  });
  page.on("pageerror", (error) => console.error(`[browser error] ${error.message}`));

  for (const profile of PROFILES) {
    await gotoBaseline(page);
    await selectProfile(page, profile);
    await setTimeFraction(page, 0);

    const directory = path.join(OUTPUT_DIR, target.name, profile.id);
    const metadata = {
      target: target.name,
      viewport: { width: target.width, height: target.height, deviceScaleFactor: target.dpr },
      profile: profile.id,
    };

    console.log(`Capturing ${target.name} / ${profile.id}`);
    await capturePage(page, path.join(directory, "full-page.png"), { ...metadata, kind: "full-page" });
    await captureCoreSections(page, directory, metadata);
    if (profile.id === "s-curve") await captureRiskStates(page, directory, metadata);
    await gotoBaseline(page);
    await selectProfile(page, profile);
    await captureControlTabs(page, directory, metadata);
    await ensureProfileSelectorVisible(page);
    await selectProfile(page, profile);
    await captureSimulatorStates(page, directory, metadata);
    await captureAllDetailsOpen(page, directory, metadata);

    await gotoBaseline(page);
    await selectProfile(page, profile);
    await capturePlots(page, directory, metadata, shouldCaptureFullscreen(target));
  }

  await context.close();
}

async function main() {
  const playwright = await ensurePlaywrightRuntime();
  if (INSTALL_ONLY) {
    console.log(`Playwright ${PLAYWRIGHT_VERSION} and Chromium are installed.`);
    return;
  }

  await mkdir(OUTPUT_DIR, { recursive: true });
  console.log(`Source: ${BASE_URL}`);
  console.log(`Output: ${OUTPUT_DIR}`);
  console.log(`Maximal: ${MAXIMAL}`);

  const browser = await playwright.chromium.launch({ headless: true });
  manifest.browserVersion = browser.version();

  try {
    for (const target of TARGETS) {
      try {
        await captureTarget(browser, target);
      } catch (error) {
        console.error(`Capture failed for ${target.name}:`, error);
        manifest.failures.push({
          target: target.name,
          message: error instanceof Error ? error.message : String(error),
        });
      }
    }
  } finally {
    await browser.close();
  }

  await writeFile(path.join(OUTPUT_DIR, "manifest.json"), `${JSON.stringify(manifest, null, 2)}\n`, "utf8");
  console.log(`\nGenerated ${manifest.captures.length} screenshots.`);
  console.log(`Manifest: ${path.join(OUTPUT_DIR, "manifest.json")}`);

  if (manifest.failures.length > 0) {
    console.error(`${manifest.failures.length} target(s) failed.`);
    process.exitCode = 1;
  }
}

await main();