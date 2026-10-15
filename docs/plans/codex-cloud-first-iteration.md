# Codex Cloud: core implementation plan

## Goal

Complete and review a buildable core-first foundation for `4everWZ/flight-damage-sim`, then improve it toward the preserved [original v0.1 specification](../spec/flight-sim-core-v0.1.original.md). [Current decisions](../decisions/current-decisions.md) own implementation rationale, measured evidence and model uncertainty. Preserve the original document unchanged.

## Working state

PR #1 was merged by the user after desktop fixes and passing CI. Main is `9f0bd7c` and includes user commit `1d6d837` (viewer performance, cleanup and corrected display). The user explicitly resumed automatic bounded core iterations on 2026-10-15. Current development starts from that main on `feature/detached-aero-systems`; preserve all user work, use draft PRs, and never merge automatically. Cloud agent configuration is not added to Git.

The repository-local APEX skills preserve source content/license and do not modify global permissions, credentials or other projects. Static installation checks do not constitute skill behavioral validation.

## Current bounded iteration

Fill the separation boundary: each detached component evaluates the existing cell law from its own CG/velocity/rates/altitude/wind and integrates 6DoF once per 120 Hz tick. Partition mapped control surfaces into local unpowered/jammed branches while keeping surviving parent branches operational. Validate disjoint force/moment reduction, birth conservation, control independence, passive work, step refinement and actual load-triggered flight separation. Expose component diagnostics through the existing live protocol and experiment CSV; preserve the user's optimized viewer and cleanup behavior.

This is an additive reduced-model mechanism. Frozen separation deformation, neutral relaxation of unpowered non-jammed surfaces, no inter-body wakes/collisions, no secondary fracture or detached engines are explicit limits. Material-removal mass bookkeeping and local spanwise beam stresses remain future core work. The old review checkpoint no longer pauses development; native human handling limits still apply. [Viewer operations](../viewer-operations.md) own startup and manual checks.

## Reproduce

```sh
make -j2
python3 tools/experiments.py
python3 tools/instructor_experiments.py
python3 tools/propulsion_experiments.py
python3 tools/rotor_experiments.py
python3 tools/detachment_experiments.py
python3 tools/viewer.py --smoke
python3 tools/plot_results.py
```

The headless path uses only GCC supporting C++20, GNU make and Python's standard library. The optional viewer/plots use Matplotlib. The [operations manual](../viewer-operations.md) contains the tested repository-local virtual environment and TkAgg startup, every input/damage command, telemetry interpretation and manual checks; Cloud uses `--smoke` instead of a GUI. Guidance and legacy propulsion comparisons remain available.
`build/flight-run data/validation.airframe <scenario> results/<prefix> [hz=120] [seconds=10] [controller=flightpath|flightpath-p|legacy|off] [bank_deg=40]` runs a single scenario; available names are defined in `src/runner.cpp`. The experimental 240 Hz option exists for convergence. Live mode is fixed at 120 Hz. Inputs and damage commands are applied at explicit tick boundaries; viewer rendering does not alter dt.

`results/summary.json`, `results/instructor-summary.json` and `results/propulsion-summary.json` and `results/rotor-summary.json` store fresh measurements and acceptance predicates; CSVs include flight, ownership, actual surfaces, stall counts and structural failure. `*.cells.csv`/`*.edges.csv` contain local end-state debug data. `experiments.png`, `instructor-comparison.png` and `rotor-governor.png` are scientific plots; `viewer-smoke.png` is a marked offline Agg rendering of live state; `viewer-input-summary.json` records input lifecycle checks with `human_gui_verified=false`. These generated results stay untracked.

## Original acceptance mapping

| Baseline requirement | State | Evidence / next work |
|---|---|---|
| §4–5, 6DoF and 120 Hz | Implemented and tested | Force-at-point, ballistic energy, full-inertia free-rotation invariants/convergence, covariance and tilted simultaneous split checks |
| §13–18, distributed/local aero | Implemented reduced model, tested | 152 cells, rotational flow, static stall entry/recovery, asymmetric and symmetric loss. Dynamic stall, flaps/slats/downwash and calibrated compressibility pending |
| TEST-FM-001 trim | Tested | Elastic equilibrium and force/moment residual; no historical validation |
| TEST-FM-002 elevator pull | Tested first fixture | AoA/G/energy telemetry; normal 70 m/s pull only ~1.76 G, high-speed entry reaches ~4.84 G; not a full envelope |
| TEST-FM-003 roll | Tested first speed | Intact/damaged roll response at 70 m/s; speed sweep pending |
| TEST-CTRL-001 90° aim | Tested basic guidance | Legacy prop variant: ~89.99° heading, ~18.6 m transient drop; signed prop: ~89.919°, ~17.074 m drop. Guidance comparisons retained |
| TEST-CTRL-002/003 override/release | Implemented and tested | Independent ownership, configurable handoff initial state, PI freeze and release; live adapter smoke |
| §19–22 engine/prop/trim | Partial, implemented and measured | Signed blade loads/induction, stopped drag/wind startup, coupled rotor inertia/energy/gyro, finite pitch governor/jam/travel and three trim axes. Legacy counterexamples retained. Calibrated data, feathering, richer damage/engine model pending |
| TEST-DMG-001 aero loss | Implemented and tested | Exactly 8/32 left outer-wing cells; force/moment and flight response, Instructor compensation |
| TEST-DMG-002 control damage | Implemented and tested disconnect | Reduced actual roll response; no Instructor bypass. Partial range/lag/friction/hinge coverage pending |
| TEST-DMG-003 spar weakening | Implemented reduced graph and tested | Low-load bench/flight survival, redistribution, delayed coupled high-load failure; material calibration pending |
| TEST-DMG-004 large skin loss | Partial | 25% aero loss does not detach; large-area fixture with trim difficulty pending |
| TEST-DMG-005 critical hit | Partial | Connectivity and mass split tested; dedicated small-entry-hole/high-load scenario pending |
| §29–30 separation/feedback | Implemented reduced model and tested | Graph connectivity, CG/full inertia and split momentum, elastic feedback. Detached cells now drive independent body motion; local control branches are unpowered/jammed. Secondary fracture, collisions, detached engines and inter-body wakes pending |
| §36–37 debugging | Partial, runnable and adapter-tested | Mouse/axes/throttle/three trim/prop/damage, per-axis ownership, lost-focus/pause/reset, CSV/JSON, forces/utilization. Full structural heatmap and native GUI/human handling validation pending |
| §33–35 projectiles/HE/fire | Not implemented | Follow core damage injection; do not replace it with HP |
| §38–42 aircraft reuse/fidelity | One data-defined synthetic fixture | Historical data and ≥3 substantially different aircraft remain pending |
| §45 replay | Tested same binary | No stochastic effects yet; explicit seed required when introduced |
| §48–49 first playable/full success | Not complete | Engineering viewer is available; finished sky/terrain camera and human flight-feel evaluation remain open |

## Next useful experiments

1. Calibrate and extend the signed prop boundary before historical or transonic performance claims: independent blade/engine data, compressibility/tip/swirl/Re limits and shutdown/feathering damage. Numerical signed power balance, finite pitch governance and rotor/body flight ledgers now pass; this does not validate the synthetic airframe. Preserve the legacy three failed hypotheses. Broaden physically useful speed/load cases rather than tuning away those failures. Detailed sources/equations/measurements belong in [current decisions](../decisions/current-decisions.md).
2. Add target-rate feedforward for continuously moving aim: current circle tracking lags by ~13°. Distinguish path tracking from absolute altitude hold; the latter is not the declared aim semantic. Keep legacy/off/P-only comparisons and the failed universal height-loss hypothesis visible.
3. Extend material mass bookkeeping and spanwise graph geometry/load mapping, then test large skin loss, critical spar hits and secondary detached-piece fracture. Separation momentum is checked, but local material removal does not yet update component mass. Current generalized graph statics are not calibrated local beam stresses.
4. Extend damaged range/friction/lag/hinge and saturation cases, and speed sweeps. Re-run impossible trim through actual available actuators rather than manufacturing equilibrium.
5. Run the viewer on a desktop using the documented startup modes and record human observations. Cloud smoke validates transport/rendering only. A future FlightGear adapter still needs explicit aim/ownership and cell/structure telemetry.

## Blockers

No known core build/test blocker. This iteration must observe CI for its own pushed head. The mechanics/Instructor revision passed all remote Actions steps; follow-up heads need their own observed result. The gh GraphQL PR-status call was forbidden; GitHub connector reads provide the remote workflow/status evidence instead. Cloud has no graphical display, so interactive human handling cannot be established here. The upstream APEX PowerShell package validator needs `pwsh` and the official `skill-creator` package; neither is installed, and neither is required to run this core. Historical aircraft data, detailed material calibration and full v0.1 acceptance remain project work, not completed claims.
