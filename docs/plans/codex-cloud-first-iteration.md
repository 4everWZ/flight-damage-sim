# Codex Cloud: core implementation plan

## Goal

Complete and review a buildable core-first foundation for `4everWZ/flight-damage-sim`, then improve it toward the preserved [original v0.1 specification](../spec/flight-sim-core-v0.1.original.md). [Current decisions](../decisions/current-decisions.md) own implementation rationale, measured evidence and model uncertainty. Preserve the original document unchanged.

## Working state

PR #13 merged as `b3e0095` after explicit confirmation; all four post-merge main CI jobs passed. The current bounded core iteration is on isolated `feature/damaged-control-coupling`. Standing user/repository authorization permits integration after physical review and passing checks for changes needing no human test or material decision; honor actual approval-review results. GitHub owns live PR/check status. Production aerodynamic-data/default-model adoption remains separate. Preserve user changes, original spec and isolated environments; add no Cloud agent configuration to Git.

The repository-local APEX skills preserve source content/license and do not modify global permissions, credentials or other projects. Static installation checks do not constitute skill behavioral validation.

## Current bounded iteration

The lagged/rate-limited damaged-control reversal benchmark exposes a missed within-tick turn and wrong aerodynamic torque impulse. Integrate the existing exponential target plus bounded surface speed through its catch times; keep zero-lag behavior, force laws, damage/input meanings and old acceptance predicates. The analytic hardware/cell-wrench matrix and short production 6DoF comparisons are implemented; complete review and repository/remote checks before eligible integration, then verify main CI. See [baseline, correction and limits](../decisions/current-decisions.md#lagged-damaged-controls-within-tick-reversal-and-aerodynamic-impulse). The fixed default live step stays 120 Hz; no human handling or full v0.1 completion is claimed.

### Previous clock investigation, completed on the earlier controller

The final discriminating experiment is complete: keep the physical kernel/actuators/rotor/structure at 960 Hz; independently change internal controller update, sensor read and request output clocks, then close at the native force age. The coarse endpoint matches the previous fixed-age run and the high endpoint reproduces the default native 960 Hz chain exactly. A separate 240 Hz sampled-system cross-check reproduces ~0.12181 m of effect disagreement on identical physical stepping, versus ~0.12418 m in the original native comparison.

That investigation selected no production/gain/acceptance change, and its original 10% hypothesis remained false. No interpretation of that acceptance expectation is adopted. The new control correction is selected from the separate force-balance and moving-aim mechanisms above. Its vector-load dependencies require complete v3 sensor records; current native-clock identities are rechecked without changing their limits. Older records and findings retain their earlier-controller meaning. Evidence and the unresolved expectation belong to [current decisions](../decisions/current-decisions.md#final-clock-decomposition-and-the-remaining-semantics-choice).

## Reproduce

```sh
make -j2
python3 tools/experiments.py
python3 tools/instructor_experiments.py
python3 tools/aim_tracking_experiments.py
python3 tools/propulsion_experiments.py
python3 tools/rotor_experiments.py
python3 tools/detachment_experiments.py
python3 tools/material_experiments.py
python3 tools/control_experiments.py
python3 tools/sensor_history_experiments.py
python3 tools/load_sample_experiments.py
python3 tools/clock_experiments.py
python3 tools/finite_span_experiments.py
python3 tools/lifting_line_reference.py
python3 tools/section_transfer_reference.py
python3 tools/local_section_coupling.py
python3 tools/roll_authority_reference.py
python3 tools/control_fault_reference.py
python3 tools/control_transition_reference.py
python3 tools/viewer.py --smoke
python3 tools/plot_results.py
```

The headless path uses only GCC supporting C++20, GNU make and Python's standard library. The optional viewer/plots use Matplotlib. The [operations manual](../viewer-operations.md) contains the tested repository-local virtual environment and TkAgg startup, every input/damage command, telemetry interpretation and manual checks; Cloud uses `--smoke` instead of a GUI. Guidance and legacy propulsion comparisons remain available.
`build/flight-run data/validation.airframe <scenario> results/<prefix> [hz=120] [seconds=10] [controller=flightpath|flightpath-p|legacy|off] [bank_deg=40]` runs a single scenario; available names are defined in `src/runner.cpp`. The experimental 240 Hz option exists for convergence. Live mode is fixed at 120 Hz. Inputs and damage commands are applied at explicit tick boundaries; viewer rendering does not alter dt.

`results/summary.json`, `results/instructor-summary.json` and `results/propulsion-summary.json` and `results/rotor-summary.json` store fresh measurements and acceptance predicates; CSVs include flight, ownership, actual surfaces, stall counts and structural failure. `*.cells.csv`/`*.edges.csv` contain local end-state debug data. `experiments.png`, `instructor-comparison.png` and `rotor-governor.png` are scientific plots; `viewer-smoke.png` is a marked offline Agg rendering of live state; `viewer-input-summary.json` records input lifecycle checks with `human_gui_verified=false`. These generated results stay untracked.

## Original acceptance mapping

| Baseline requirement | State | Evidence / next work |
|---|---|---|
| §4–5, 6DoF and 120 Hz | Implemented and tested | Midpoint cell-flow work/passivity on parent and pieces; force-at-point, ballistic energy, full-inertia free-rotation invariants/convergence, covariance and tilted simultaneous split checks |
| §13–18, distributed/local aero | Implemented reduced model, tested | 152 cells, rotational flow, static stall entry/recovery, asymmetric and symmetric loss. Dynamic stall, flaps/slats/downwash and calibrated compressibility pending |
| TEST-FM-001 trim | Tested | Elastic equilibrium and force/moment residual; no historical validation |
| TEST-FM-002 elevator pull | Tested first fixture | AoA/G/energy telemetry; normal 70 m/s pull only ~1.76 G, high-speed entry reaches ~4.84 G; not a full envelope |
| TEST-FM-003 roll | Tested first speed | Intact/damaged roll response at 70 m/s; speed sweep pending |
| TEST-CTRL-001 90° aim | Tested basic guidance | Legacy prop variant: ~89.99° heading, ~18.6 m transient drop; signed prop: ~89.919°, ~17.074 m drop. Guidance comparisons retained |
| TEST-CTRL-002/003 override/release | Implemented and tested | Independent ownership, configurable handoff initial state, PI freeze and release; live adapter smoke |
| §19–22 engine/prop/trim | Partial, implemented and measured | Signed blade loads/induction, stopped drag/wind startup, coupled rotor inertia/energy/gyro, finite pitch governor/jam/travel and three trim axes. Legacy counterexamples retained. Calibrated data, feathering, richer damage/engine model pending |
| TEST-DMG-001 aero loss | Implemented and tested | Exactly 8/32 left outer-wing cells; force/moment and flight response, Instructor compensation |
| TEST-DMG-002 control damage | Implemented and tested reduced hardware | Disconnect plus combined lag/friction/range/hinge reversal, local cell loss and real surface-driven short flight; full damaged envelope/free-floating hinge physics pending |
| TEST-DMG-003 spar weakening | Implemented reduced graph and tested | Low-load bench/flight survival, redistribution, delayed coupled high-load failure; material calibration pending |
| TEST-DMG-004 large skin loss | Partial | 25% aero loss does not detach; large-area fixture with trim difficulty pending |
| TEST-DMG-005 critical hit | Partial | Connectivity and mass split tested; dedicated small-entry-hole/high-load scenario pending |
| §29–30 separation/feedback | Implemented reduced model and tested | Explicit material inventory now changes owner mass/CG/full inertia and tracks removed parcels conservatively. Graph connectivity, CG/full inertia and split momentum, elastic feedback. Detached cells now drive independent body motion; local control branches are unpowered/jammed. Secondary fracture, collisions, detached engines and inter-body wakes pending |
| §36–37 debugging | Partial, runnable and adapter-tested | Mouse/axes/throttle/three trim/prop/damage, per-axis ownership, lost-focus/pause/reset, CSV/JSON, forces/utilization. Full structural heatmap and native GUI/human handling validation pending |
| §33–35 projectiles/HE/fire | Not implemented | Follow core damage injection; do not replace it with HP |
| §38–42 aircraft reuse/fidelity | One data-defined synthetic fixture | Historical data and ≥3 substantially different aircraft remain pending |
| §45 replay | Tested same binary | No stochastic effects yet; explicit seed required when introduced |
| §48–49 first playable/full success | Not complete | Engineering viewer is available; finished sky/terrain camera and human flight-feel evaluation remain open |

## Next useful experiments

1. Finite-span/induced-flow reference is now measured independently. Next test an explicitly new opt-in section-data fixture and strip/cell load boundary, preserving real controls/structure; surface data meaning before any production adoption. Then prioritize local stall validation and calibrated propeller effects. The reference excludes arbitrary perforations, transient/stalled wakes and live 120 Hz cost. Clock experiments are finished: preserve the original failed 10% hypothesis and unresolved sampled-production versus continuous-limit interpretation; no new clock patch is selected.
2. Calibrate and extend the signed prop boundary before historical or transonic performance claims: independent blade/engine data, compressibility/tip/swirl/Re limits and shutdown/feathering damage. Numerical signed power balance, finite pitch governance and rotor/body flight ledgers now pass; this does not validate the synthetic airframe. Preserve the legacy three failed hypotheses. Broaden physically useful speed/load cases rather than tuning away those failures. Detailed sources/equations/measurements belong in [current decisions](../decisions/current-decisions.md).
3. Horizontal target-rate feedforward and consistent normal-load feedback merged in PR #11; retain their measured bounds. Keep legacy/off/P-only comparisons and failed altitude hypotheses visible. A later vertical-rate, stop/reversal or wider speed-envelope claim needs its own bounded evidence; no altitude-hold semantic is adopted.
4. Extend material mass bookkeeping and spanwise graph geometry/load mapping, then test large skin loss, critical spar hits and secondary detached-piece fracture. Explicit named material removal now updates the current component and accounts for removed parcels; automatic geometric damage-to-material mapping and full structural materialization remain pending. Current generalized graph statics are not calibrated local beam stresses.
5. Combined damaged range/friction/lag/hinge reversal now has an analytic cell-force bench and short speed/rate matrix. Next test that chain through real mouse/keyboard handoff or sustained control/structural-limit approach, distinguishing aerodynamic bias from incremental authority. Preserve impossible trim as an honest residual rather than manufactured equilibrium.
6. Run the viewer on a desktop using the documented startup modes and record human observations. Cloud smoke validates transport/rendering only. A future FlightGear adapter still needs explicit aim/ownership and cell/structure telemetry.

## Blockers

No known build/test blocker; PR #13's configuration fix is integrated. This core iteration must observe its own PR and post-merge workflows; earlier green CI is not current-head evidence. The gh GraphQL PR-status call was forbidden; GitHub connector reads provide the remote workflow/status evidence instead. Cloud has no graphical display, so interactive human handling cannot be established here. The upstream APEX PowerShell package validator needs `pwsh` and the official `skill-creator` package; neither is installed, and neither is required to run this core. Historical aircraft data, detailed material calibration and full v0.1 acceptance remain project work, not completed claims.
