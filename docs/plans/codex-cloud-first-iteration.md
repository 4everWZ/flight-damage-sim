# Codex Cloud: core implementation plan

## Goal

Complete and review a buildable core-first foundation for `4everWZ/flight-damage-sim`, then improve it toward the preserved [original v0.1 specification](../spec/flight-sim-core-v0.1.original.md). [Current decisions](../decisions/current-decisions.md) own implementation rationale, measured evidence and model uncertainty. Preserve the original document unchanged.

## Working state

First headless slice and a Matplotlib engineering viewer are implemented on `feature/headless-core-harness`. Work stays within this repository. A temporary remote-write pause was explicitly lifted after the parent confirmed the earlier cancellation was a platform bug. Main contains the original design checkpoint; development must remain on this feature branch with a draft PR and no auto-merge.

The repository-local APEX skills preserve source content/license and do not modify global permissions, credentials or other projects. Static installation checks do not constitute skill behavioral validation.

## Reproduce

```sh
make -j2
python3 tools/experiments.py
python3 tools/viewer.py --smoke
python3 tools/plot_results.py
```

The headless path uses only GCC supporting C++20, GNU make and Python's standard library. The optional viewer/plots use Matplotlib (`python3 -m pip install -r requirements-viewer.txt` in a local isolated environment if absent). On a machine with a desktop backend, `python3 tools/viewer.py` opens the live engineering view. Click heading/elevation to set aim; arrows control pitch/roll, Q/E yaw, W/S throttle. F2–F6 inject cell/link/spar/engine damage. Space pauses and R resets.

`build/flight-run data/validation.airframe <scenario> results/<prefix> [hz=120] [seconds=10]` runs a single scenario; available names are defined in `src/runner.cpp`. The experimental 240 Hz option exists for convergence. Live mode is fixed at 120 Hz. Inputs and damage commands are applied at explicit tick boundaries; viewer rendering does not alter dt.

`results/summary.json` stores fresh measurements and acceptance predicates; CSVs include flight, ownership, actual surfaces, stall counts and structural failure. `*.cells.csv`/`*.edges.csv` contain local end-state debug data. `experiments.png` is a scientific plot; `viewer-smoke.png` is an Agg rendering of live state. These generated results stay untracked.

## Original acceptance mapping

| Baseline requirement | State | Evidence / next work |
|---|---|---|
| §4–5, 6DoF and 120 Hz | Implemented and tested | Force-at-point, analytic ballistic energy, angular acceleration, quaternion motion; 120/240 comparisons |
| §13–18, distributed/local aero | Implemented reduced model, tested | 152 cells, rotational flow, static stall entry/recovery, asymmetric and symmetric loss. Dynamic stall, flaps/slats/downwash and calibrated compressibility pending |
| TEST-FM-001 trim | Tested | Elastic equilibrium and force/moment residual; no historical validation |
| TEST-FM-002 elevator pull | Tested first fixture | AoA/G/energy telemetry; normal 70 m/s pull only ~1.76 G, high-speed entry reaches ~4.84 G; not a full envelope |
| TEST-FM-003 roll | Tested first speed | Intact/damaged roll response at 70 m/s; speed sweep pending |
| TEST-CTRL-001 90° aim | Tested basic guidance | ~89.43° at 30 s; remaining altitude/coordination/handling improvements |
| TEST-CTRL-002/003 override/release | Implemented and tested | Independent roll ownership and 0.25 s handoff; live adapter smoke |
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

1. Improve the Instructor's sustained banked-turn altitude and mixed-roll recovery, measuring flight-path error, surface saturation and energy. It currently loses ~31 m during the 30 s turn. Preserve direct authority and actual control limitations.
2. Extend the propeller model to tabulated advance ratio and constant-speed governing. The 140 m/s case is explicitly transient, not trimmed cruise. Compare torque/thrust and windmilling shutdown behavior before claiming propulsion fidelity.
3. Extend structural coverage to multiple nodes/materials, large skin loss and critical spar hits, plus detached-piece aerodynamic loads. Check cell/structure/control partitioning and separation momentum at more attitudes/rates.
4. Add saturated-Instructor/damaged-chain scenarios and speed sweeps; test range/friction/lag/hinge modes independently. Treat natural stall and limited authority as observable outcomes.
5. Run the existing viewer locally with a GUI and record qualitative handling observations. Cloud smoke validates the transport/render path, not human playability. A later FlightGear adapter requires explicit aim/ownership input and distributed debug telemetry; native-ctrls/native-fdm alone are insufficient.

## Blockers

No core build/test blocker. Cloud has no graphical display, so interactive human handling cannot be established here. The upstream APEX PowerShell package validator needs `pwsh` and the official `skill-creator` package; neither is installed, and neither is required to run this core. Historical aircraft data, detailed material calibration and full v0.1 acceptance remain project work, not completed claims.
