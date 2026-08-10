# Design QA

## Evidence

- Source visual truth:
  - `/workspace/scratch/e5b13cf5ff42/qa/design-qa-desktop-first.jpg` — existing right-hand animation rail before the scoped control move.
  - `/workspace/scratch/e5b13cf5ff42/upload/01-9002.jpg` — mobile simulator baseline.
  - `/workspace/scratch/e5b13cf5ff42/upload/02-9004.jpg` — mobile inputs baseline.
  - `/workspace/scratch/e5b13cf5ff42/upload/03-9006.jpg` — mobile analysis baseline.
- Browser-rendered implementation:
  - `/workspace/scratch/e5b13cf5ff42/qa-angle/angle-rail-desktop-complete.jpg` — editable Max/Min controls integrated at the rail endpoints, tested at 45°–225°.
  - `/workspace/scratch/e5b13cf5ff42/qa/design-qa-desktop-first.jpg` — simulator-first landscape viewport.
  - `/workspace/scratch/e5b13cf5ff42/qa/design-qa-desktop-config.jpg` — configuration immediately below the simulator.
  - `/workspace/scratch/e5b13cf5ff42/qa/design-qa-desktop-results.jpg` — analysis below configuration.
- Combined comparison inputs:
  - `/workspace/scratch/e5b13cf5ff42/qa-angle/angle-rail-comparison.jpg` — same-size before/after comparison of the animation rail.
  - `/workspace/scratch/e5b13cf5ff42/qa/design-qa-comparison.jpg` — original mobile-to-landscape design comparison.
- Source pixels: 709 × 1536 each. The screenshots include Android and in-app-browser chrome, so they are used as the mobile visual-system and information-hierarchy reference rather than as a CSS-pixel density target.
- Implementation pixels and CSS viewport: 1348 × 926 at device-pixel ratio 1, rendered by the cloud browser in a 1363 × 936 viewport.
- State: default geometry with the editable rail shown at a tested 45°–225° motion window.

## Full-view comparison evidence

The rendered landscape implementation preserves the source's off-white paper, dark engineering canvas, cyan/orange/lime/violet linkage identity, Geist/monospace hierarchy, fine grid, rounded card language, and dense technical tone. The Max and Min values have moved from the configuration panel into compact editable endpoint controls on the rail without reducing the canvas or breaking the established hierarchy.

The 393 px phone breakpoint was checked from the implemented layout rules against the supplied phone reference: 58 px top bar, 8 px outer gutter, 54 px simulator heading, 64 px right rail, and a simulator stage calculated at about 384 px high. Its 620 × 760 viewBox matches the resulting narrow stage aspect ratio, so the linkage remains large and centered without changing the model scale. Direct phone emulation was unavailable in the cloud-browser surface; this remains a device-rendering test gap rather than an observed mismatch.

## Focused-region comparison evidence

- Simulator: the linkage is the dominant first-viewport object; the rail now carries the angle readout, editable Max/Min endpoints, vertical scrubber, ±5° nudges, and Run/Pause without reducing the canvas to a thumbnail.
- Inputs: existing dark-panel typography, tabs, fields, units, and summary strip are preserved. The Motion group now contains speed and a short pointer to the right-hand scrubber, avoiding duplicate angle controls.
- Analysis: peak metrics, plots, reactions, and actuator screening remain below the configuration. A partial angle window updates peak calculations, plot x-axes, sample count, CSV output, and the highlighted tool-path arc.

## Required fidelity surfaces

- Fonts and typography: passed. Existing Geist and Geist Mono families, weights, hierarchy, wrapping, and technical labels are preserved; no new font drift was introduced.
- Spacing and layout rhythm: passed. Both rail inputs fit within the existing 96 px desktop rail. The mobile rules retain the 64 px rail, compact 31 px endpoint inputs, and a reduced range-track minimum so the control stack stays within the stage.
- Colors and visual tokens: passed. Existing paper, navy, cyan, orange, lime, violet, danger, border, and muted tokens are reused consistently.
- Image quality and asset fidelity: passed. The mechanism remains a crisp semantic SVG; no raster placeholders, approximate logos, or substitute assets were added.
- Copy and content: passed. Labels distinguish the full tool path from the configured motion window, and analysis copy changes between full-cycle and partial-window modes.

## Interaction and accessibility checks

- Entered a 45° minimum and 225° maximum using the new rail endpoint controls.
- Confirmed the vertical slider adopted the same 45°–225° limits.
- Confirmed animation stayed inside the range during the new interaction pass; sampled values remained between 45° and 225°.
- Confirmed the motion-window path, load-analysis x-axis, peak values, CSV scope, and sample count updated with the limits.
- Confirmed Reset preset restores 0°–360° and remounts both angle editors with the restored values.
- Confirmed tab roles, native number/range controls, labels, keyboard nudges, and focus styling remain present.
- Confirmed there are no remaining minimum/maximum angle inputs in the configuration panel.
- Browser console contained no application-origin errors; observed messages came only from the browser's own extension.
- Production build, artifact validation, lint, and rendered-HTML test passed.

## Findings

- No actionable P0, P1, or P2 findings remain in the tested implementation.

## Open questions

- Exact rendering on the user's Android in-app-browser should be visually spot-checked after deployment because the cloud browser did not expose a phone-sized viewport.

## Implementation checklist

- [x] Simulator dominates the first viewport.
- [x] Vertical animation control sits on the right.
- [x] Configuration follows the simulator through normal scrolling.
- [x] Analysis follows configuration.
- [x] Landscape layout uses the available width.
- [x] Minimum and maximum angles constrain animation and analysis.
- [x] Minimum and maximum editors sit directly on the right-hand slider rail.
- [x] Partial ranges reverse smoothly at their endpoints.

## Follow-up polish

- P3: verify the final top-bar wrap and native vertical-range thumb on the user's specific Android WebView after deployment.

final result: passed
