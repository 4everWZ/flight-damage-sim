# Codex Cloud: core implementation plan

## Goal

Complete and review a buildable core-first foundation for `4everWZ/flight-damage-sim`, then improve it toward the preserved [original v0.1 specification](../spec/flight-sim-core-v0.1.original.md). [Current decisions](../decisions/current-decisions.md) own implementation rationale, measured evidence and model uncertainty. Preserve the original document unchanged.

## Working state

Two tested core iterations and a Matplotlib engineering viewer are implemented on `feature/headless-core-harness`. Work stays within this repository. A temporary remote-write pause was explicitly lifted after the parent confirmed the earlier cancellation was a platform bug. Main contains the original design checkpoint; development must remain on this feature branch with a draft PR and no auto-merge.

The repository-local APEX skills preserve source content/license and do not modify global permissions, credentials or other projects. Static installation checks do not constitute skill behavioral validation.

## Reproduce

```sh
make -j2
python3 tools/experiments.py
python3 tools/instructor_experiments.py
python3 tools/viewer.py --smoke
python3 tools/plot_results.py
```

The headless path uses only GCC supporting C++20, GNU make and Python's standard library. The optional viewer/plots use Matplotlib (`python3 -m pip install -r requirements-viewer.txt` in a local isolated environment if absent). On a machine with a desktop backend, `python3 tools/viewer.py` opens the live engineering view. Click heading/elevation to set aim; arrows control pitch/roll, Q/E yaw, W/S throttle. F2–F6 inject cell/link/spar/engine damage. Space pauses, R resets while preserving controller choice, and M cycles new/legacy/off. `--controller legacy` or `--controller off` selects a startup comparison; these are automatic-input/transport tests, not human flight-feel validation.

`build/flight-run data/validation.airframe <scenario> results/<prefix> [hz=120] [seconds=10] [controller=flightpath|flightpath-p|legacy|off] [bank_deg=40]` runs a single scenario; available names are defined in `src/runner.cpp`. The experimental 240 Hz option exists for convergence. Live mode is fixed at 120 Hz. Inputs and damage commands are applied at explicit tick boundaries; viewer rendering does not alter dt.

`results/summary.json` and `results/instructor-summary.json` store fresh measurements and acceptance predicates; CSVs include flight, ownership, actual surfaces, stall counts and structural failure. `*.cells.csv`/`*.edges.csv` contain local end-state debug data. `experiments.png` and `instructor-comparison.png` are scientific plots; `viewer-smoke.png` is an Agg rendering of live state. These generated results stay untracked.

## Original acceptance mapping

| Baseline requirement | State | Evidence / next work |
|---|---|---|
| §4–5, 6DoF and 120 Hz | Implemented and tested | Force-at-point, ballistic energy, full-inertia free-rotation invariants/convergence, covariance and tilted simultaneous split checks |
| §13–18, distributed/local aero | Implemented reduced model, tested | 152 cells, rotational flow, static stall entry/recovery, asymmetric and symmetric loss. Dynamic stall, flaps/slats/downwash and calibrated compressibility pending |
| TEST-FM-001 trim | Tested | Elastic equilibrium and force/moment residual; no historical validation |
| TEST-FM-002 elevator pull | Tested first fixture | AoA/G/energy telemetry; normal 70 m/s pull only ~1.76 G, high-speed entry reaches ~4.84 G; not a full envelope |
| TEST-FM-003 roll | Tested first speed | Intact/damaged roll response at 70 m/s; speed sweep pending |
| TEST-CTRL-001 90° aim | Tested basic guidance | New mode ~89.99° velocity heading at 30 s, transient drop ~18.6 m; off/legacy/P-only and damaged comparisons retained |
| TEST-CTRL-002/003 override/release | Implemented and tested | Independent ownership, configurable handoff initial state, PI freeze and release; live adapter smoke |
| §19–22 engine/prop/trim | Partial, measured | Fixed-pitch shaft/RPM/thrust/torque/slipstream and three trim axes; constant-speed prop, feathering and richer damage pending |
| TEST-DMG-001 aero loss | Implemented and tested | Exactly 8/32 left outer-wing cells; force/moment and flight response, Instructor compensation |
| TEST-DMG-002 control damage | Implemented and tested disconnect | Reduced actual roll response; no Instructor bypass. Partial range/lag/friction/hinge coverage pending |
| TEST-DMG-003 spar weakening | Implemented reduced graph and tested | Low-load bench/flight survival, redistribution, delayed coupled high-load failure; material calibration pending |
| TEST-DMG-004 large skin loss | Partial | 25% aero loss does not detach; large-area fixture with trim difficulty pending |
| TEST-DMG-005 critical hit | Partial | Connectivity and mass split tested; dedicated small-entry-hole/high-load scenario pending |
| §29–30 separation/feedback | Implemented reduced model and tested | Graph connectivity, CG/full inertia and split momentum, elastic feedback. Detached-body aero/collisions pending |
| §36–37 debugging | Partial, runnable | Cell/link/spar/engine/prop events, CSV/JSON, live intent, force arrows/utilization. Full structural heatmap and GUI handling validation pending |
| §33–35 projectiles/HE/fire | Not implemented | Follow core damage injection; do not replace it with HP |
| §38–42 aircraft reuse/fidelity | One data-defined synthetic fixture | Historical data and ≥3 substantially different aircraft remain pending |
| §45 replay | Tested same binary | No stochastic effects yet; explicit seed required when introduced |
| §48–49 first playable/full success | Not complete | Engineering viewer is available; finished sky/terrain camera and human flight-feel evaluation remain open |

## Next useful experiments

1. Resolve propulsion/energy margin before promising sustained climb or tighter level turns. Compare fixed-pitch advance tables and a constant-speed governor on the same core. The current 8° climb is not fully trackable, and 50°/60° turn requests saturate pitch. Preserve keyboard authority and report actual rate/path error.
2. Add target-rate feedforward for continuously moving aim: current circle tracking lags by ~13°. Distinguish path tracking from absolute altitude hold; the latter is not the declared aim semantic. Keep legacy/off/P-only comparisons and the failed universal height-loss hypothesis visible.
3. Extend material mass bookkeeping and spanwise graph geometry/load mapping, then test large skin loss, critical spar hits and detached-piece aero. Separation momentum is checked, but local material removal does not yet update component mass. Current generalized graph statics are not calibrated local beam stresses.
4. Extend damaged range/friction/lag/hinge and saturation cases, and speed sweeps. Re-run impossible trim through actual available actuators rather than manufacturing equilibrium.
5. Run the viewer on a desktop using the documented startup modes and record human observations. Cloud smoke validates transport/rendering only. A future FlightGear adapter still needs explicit aim/ownership and cell/structure telemetry.

## Blockers

No core build/test blocker. The previous revision had no remote Actions checks; the added workflow must be observed on the pushed head. The gh GraphQL PR-status call was forbidden; GitHub connector reads provide the remote workflow/status evidence instead. Cloud has no graphical display, so interactive human handling cannot be established here. The upstream APEX PowerShell package validator needs `pwsh` and the official `skill-creator` package; neither is installed, and neither is required to run this core. Historical aircraft data, detailed material calibration and full v0.1 acceptance remain project work, not completed claims.
