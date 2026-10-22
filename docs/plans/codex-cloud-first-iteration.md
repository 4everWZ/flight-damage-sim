# Codex Cloud: core implementation plan

## Goal

Complete and review a buildable core-first foundation for `4everWZ/flight-damage-sim`, then improve it toward the preserved [original v0.1 specification](../spec/flight-sim-core-v0.1.original.md). [Current decisions](../decisions/current-decisions.md) own implementation rationale, measured evidence and model uncertainty. Preserve the original document unchanged.

## Working state

PR17 and PR18 were explicitly approved and merged on 2026-10-20, using normal merge commits to retain the stack ancestry. PR17 merge is `eac4edfe94fa28268af3f672d2b2256466e6f2b0`; PR18 merge/main baseline is `f0fa41e3190074b09a15b3cfae67ebb3d7b67eb5`. Their post-merge CI runs 39394128517 and 39394207777 succeeded, and main's tree matches the reviewed PR18 head. No force push or physics retuning was needed.

The user also approved the smallest optional two-dimensional control-polar implementation on `feature/control-polar-2d`, disabled unless cells explicitly bind a new table. That scope excludes downwash and dynamic stall. Its future implementation PR remains draft and needs separate merge authorization. Preserve the original specification, existing fixture files and source evidence; historical aircraft calibration remains separate.

The repository-local APEX skills preserve source content/license and do not modify global permissions, credentials or other projects. Static installation checks do not constitute skill behavioral validation.

## Current bounded iteration

The lagged/rate-limited reversal repair is integrated; preserve its [baseline and evidence boundary](../decisions/current-decisions.md#lagged-damaged-controls-within-tick-reversal-and-aerodynamic-impulse). The [damaged handoff](../decisions/current-decisions.md#damaged-mousekeyboard-handoff-bounded-flight-investigation) and [sustained-load continuation](../decisions/current-decisions.md#sustained-post-handoff-maneuver-and-actual-structural-loads) now provide 20 plus 16 dynamic simulated flights. Full pitch produces ≈1.608 G at low energy, versus ≈5.023 G and delayed weakened-graph failure in the explicitly non-equilibrium high-energy entry. Actual graph balance/material recurrence and handoff remain consistent; no force-law/threshold retuning or new core repair is selected. The user has now supplied the separate PR15/PR16 integration authorization. Preserve the diagnostic evidence and run its focused CI alongside the orientation implementation. The live step remains 120 Hz; human handling, historical validity and the original false 10% hypothesis remain separate.

The user's material-physics question and component-first clarification take priority in the current assessment. The original NASA L334/L353 reports are now extracted and read, including direct scan review of the main configuration and maximum-lift tables: Spitfire Mk Va W3119/Merlin XLV/Rotol, test weight/CG, geometry and candidate control/stall observables; see [material audit and source interpretation](../decisions/current-decisions.md#material-audit-and-component-first-historical-references) and the separate [reference proposal](../../data/spitfire-va-naca-reference.json). The proposal is not a runnable aircraft and no historical comparison has run. Preserve existing synthetic fixtures; declare a matching speed/atmosphere/linkage/configuration before selecting numerical tolerances or changing production data. Bf 109 F-4 stays the second reference. Next implementation remains a bounded local stall/control evidence pass followed by component load-path refinement; do not turn historical-data completeness or full FEA into a gate, and do not change aerodynamic/material semantics before surfacing the concrete choice. Weapons/performance may be tables; core flight/controls, functional damage, high-speed and overload breakup remain priorities.

The [reference-to-input gap investigation](../decisions/current-decisions.md#spitfire-reference-input-mapping-and-current-capability-gap) is now measured: existing-fixture trim flight, actuator pressure response, static local stall flags and a prescribed two-panel geometry counterexample. **No historical numerical performance comparison is currently admissible**; weight/geometry and output-shaft RPM mapping are available, while historical coefficients, atmosphere/airspeed calibration, installed propulsion and force-linkage data are not. The proposed per-cell undeformed orientation is now approved and implemented by PR16, with identity defaults and explicit elastic/reference/incidence composition. See the [current frame contract and measured evidence](../decisions/current-decisions.md#approved-per-cell-reference-frames). The reference proposal itself is still not a runnable historical aircraft. Keep later sectional/finite-span, stall/force-control semantics and beam/box refinement explicit rather than bundling them into that geometry change.

The optional fixed-throttle trim is now implemented, preserving the level API: a free air-relative flight-path angle replaces free throttle, and acceptance requires force/moment, rotor and governor equilibrium. Independent analytic glide, uniform-wind translation, actual mechanical restrictions, failed-solve reporting and byte-equal legacy telemetry are checked. The 70/140 m/s entries are actual descending equilibria rather than the old untrimmed high-speed level initialization. See [contract and measured evidence](../decisions/current-decisions.md#fixed-throttle-trim-and-glide-controlload-evidence).

The local-flow/control/load experiments retain the existing nonlinear laws. Six-second pulls do not trigger local stall or failure; a separate sustained 140 m/s pull with damaged linkage/skin/front section causes calculated delayed failure, while the same damage survives the 60 m/s run. Existing full-authority elevator travel does not drive the tested glides past the current 15-degree cell flag. Prescribed-flow sweeps are separate diagnostics, not successful flight-stall validation. No aircraft coefficients, material strengths or control laws are fitted to historical points or changed to force a desired outcome. The [controlled follow-up](../decisions/current-decisions.md#controlled-trim-and-post-stall-diagnosis) now attributes the negative roll gain to the existing effective-angle/post-stall table semantics and quantifies limited elevator headroom. No sign/interpolation bug was established. Any new control polar, downwash or changed authority/geometry must be surfaced before adoption; component load-path refinement remains open.

The geometry change passes independent local-flow/body-wrench, incidence/dihedral, work, symmetry and attached/detached tests. Explicit signed identities retain complete telemetry, and orientation-only synthetic flights retain actual controls and load-driven damage. The useful next bounded experiment is source-conditioned local stall progression and remaining aileron authority, preserving TAS/indicated speed, powered/closed-throttle and table semantics. Define any new table-specific stall/section law before adoption; do not fit one historical point or change material strengths to force a result. Component load-path geometry and material calibration remain separate open work.

The approved [optional two-dimensional control-polar contract](../decisions/current-decisions.md#optional-two-dimensional-control-polar) is now implemented through the shared attached/detached cell force path. Independent bilinear wrench/work, equal-legacy-angle discrimination, input validation, real mechanical faults, force-stage sampling, structural graph and detachment tests pass. Default and unbound-table flight/cell/edge telemetry in four scenarios and default live JSON match the accepted main core. The ignored enabled synthetic fixture runs actual trim, mixed input and load-driven damage. This establishes model expressiveness/integration, not historical accuracy or corrected post-stall handling. Keep the current static diagnostic proxy explicitly labeled; new control/separation data and induced flow remain distinct follow-up work.

The next EDIR increment preserves PR19's frozen implementation/evidence at `4d596b0` (nine successful CI jobs, run 39396239197). Runtime review selected no demonstrated core defect. The separate [fixed-condition calibration preparation](../decisions/current-decisions.md#fixed-condition-control-polar-calibration-preparation) now tests a concrete import pitfall: Mach-conditioned lift data would otherwise receive the existing core lift multiplier twice. It retains a published CAMRAD program-output neutral slice as software-reference evidence only, refuses to fabricate missing flap deflections, reports coarse-grid holdout error, and independently exercises complete synthetic 2D preparation. No new flights, runtime law, production fixture or historical aircraft calibration is added. Source-matched measured control data remains the useful next acquisition; incomplete data stops that validation claim, not other permitted prototype work.

The [measured-source increment](../decisions/current-decisions.md#measured-control-surface-subset-and-source-qualification) now retains five faired-curve readings from a fixed equal-linkage NACA 0009 double flap, with scan pixels and explicit reading/source uncertainty. Its four-corner coefficient-space holdout misses central CL/CM; unknown Mach and remaining convention/condition gaps block runtime export. The directly relevant NACA 0015 candidate is partial because inspected plots provide hinge moments and drag increments rather than an absolute CD baseline. A demonstrated preparation-tool mismatch now rejects contradictory declared flap configurations. No runtime/default change or historical aircraft fit is claimed; denser independent deflection readings and condition/sign qualification are the next useful experiment.

The bounded continuation verified `5d26fe8` full CI 39409371385 terminal success (all nine jobs), then completed the [preselected dense slice](../decisions/current-decisions.md#preselected-denser-measured-flap-slice); `7d47bcf` full CI 39412143995 also passed all nine jobs. The old center remains held out and its four-corner failure is frozen. Added delta-5 endpoint readings improve that center's alpha interpolation; the separately preselected, unseen delta-15 observation retains a CM discrepancy slightly outside reading-only bounds. The subsequent [bounded direct-reference qualification](../decisions/current-decisions.md#bounded-mach-and-native-cm-convention-qualification) maps horizontal section CM without a sign flip through NACA721's explicit axis convention, but neither L-290 nor the directly cited facility report establishes this run's Mach/static state. No assumed Mach, measured-data native replay, runtime/default change or merge is adopted. Coefficients, bounds and hold-outs remain unchanged. Stop this source slice: obtain a case-linked condition record, or select a different complete measured source with declared Mach rather than repeat broad searches. Other authorized prototype work can continue.

### Previous clock investigation, completed on the earlier controller

The final discriminating experiment is complete: keep the physical kernel/actuators/rotor/structure at 960 Hz; independently change internal controller update, sensor read and request output clocks, then close at the native force age. The coarse endpoint matches the previous fixed-age run and the high endpoint reproduces the default native 960 Hz chain exactly. A separate 240 Hz sampled-system cross-check reproduces ~0.12181 m of effect disagreement on identical physical stepping, versus ~0.12418 m in the original native comparison.

That investigation selected no production/gain/acceptance change, and its original 10% hypothesis remained false. No interpretation of that acceptance expectation is adopted. The new control correction is selected from the separate force-balance and moving-aim mechanisms above. Its vector-load dependencies require complete v3 sensor records; current native-clock identities are rechecked without changing their limits. Older records and findings retain their earlier-controller meaning. Evidence and the unresolved expectation belong to [current decisions](../decisions/current-decisions.md#final-clock-decomposition-and-the-remaining-semantics-choice).

## Reproduce

```sh
make -j2
python3 tools/experiments.py
python3 tools/instructor_experiments.py
python3 tools/aim_tracking_experiments.py
python3 tools/cell_frame_experiments.py
python3 tools/glide_experiments.py
python3 tools/trim_stall_diagnostics.py
python3 tools/control_polar_experiments.py
python3 tools/control_polar_calibration.py
python3 tools/control_polar_measured_subset.py
python3 tools/control_polar_dense_slice.py
python3 tools/control_polar_cr3439.py
python3 tools/control_polar_reference_transfer.py
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
python3 tools/spitfire_reference_summary.py
python3 tools/reference_gap_experiments.py
python3 tools/viewer.py --smoke
python3 tools/plot_results.py
```

The headless path uses only GCC supporting C++20, GNU make and Python's standard library. The optional viewer/plots use Matplotlib. The [operations manual](../viewer-operations.md) contains the tested repository-local virtual environment and TkAgg startup, every input/damage command, telemetry interpretation and manual checks; Cloud uses `--smoke` instead of a GUI. Guidance and legacy propulsion comparisons remain available.

Optional source research, separate from core verification: create `build/reference-venv`, install `pypdf==6.0.0 pymupdf==1.26.6` into that environment, then run `build/reference-venv/bin/python tools/extract_spitfire_sources.py --ocr-review`. It fetches only the declared public NASA reports and official Tesseract English recognition data, saving page-indexed text, selected scan renders and independent OCR under `results/spitfire-source/`; `--log-review-images` also transports two public scan pages in text-only CI logs. The `Reference source extraction` workflow retains these artifacts. This Cloud host returns HTTP403 for direct NASA downloads; the isolated GitHub runner has successfully fetched them. No global dependency/credential installation is needed.
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
| TEST-DMG-002 control damage | Implemented and tested reduced hardware | Disconnect plus combined lag/friction/range/hinge reversal, local cell loss and surface-driven dynamic simulated short flight; full damaged envelope/free-floating hinge physics pending |
| TEST-DMG-003 spar weakening | Implemented reduced graph and tested | Low-load survival and sustained post-handoff measured high-load failure; parallel-path redistribution and nodal balance checked. Reduced generalized graph, not calibrated local stress/material strength |
| TEST-DMG-004 large skin loss | Partial | 25% aero loss does not detach; large-area fixture with trim difficulty pending |
| TEST-DMG-005 critical hit | Partial | Connectivity and mass split tested; dedicated small-entry-hole/high-load scenario pending |
| §29–30 separation/feedback | Implemented reduced model and tested | Explicit material inventory now changes owner mass/CG/full inertia and tracks removed parcels conservatively. Graph connectivity, CG/full inertia and split momentum, elastic feedback. Detached cells now drive independent body motion; local control branches are unpowered/jammed. Secondary fracture, collisions, detached engines and inter-body wakes pending |
| §36–37 debugging | Partial, runnable and adapter-tested | Mouse/axes/throttle/three trim/prop/damage, per-axis ownership, lost-focus/pause/reset, CSV/JSON, forces/utilization. Full structural heatmap and native GUI/human handling validation pending |
| §33–35 projectiles/HE/fire | Not implemented | Follow core damage injection; do not replace it with HP |
| §38–42 aircraft reuse/fidelity | One data-defined synthetic fixture | Historical data and ≥3 substantially different aircraft remain pending |
| §45 replay | Tested same binary | No stochastic effects yet; explicit seed required when introduced |
| §48–49 first playable/full success | Not complete | Engineering viewer is available; finished sky/terrain camera and human flight-feel evaluation remain open |

## Next useful experiments

The approved [isolated CR-3439 section reference](../decisions/current-decisions.md#isolated-cr-3439-section-reference) is implemented and locally replayed through the unchanged optional native force kernel. Original graphs established a single open .005c gap series and explicit positive-down deflection. Commit `05ffae7` fixed six training stations and six observations on entirely withheld ±10-degree curves, with reading allowances, before transcription. The dataset retains source conditions, approximate pixel calibration and separate instrumental/systematic uncertainty. CM nose-up is a labeled inference from report load/moment summaries, not an explicit axis statement. Software/operator errors are ≤2.22e−16; four endpoint diagnostic flags represent only 5.56e−17-rad rounding and remain visible. Unseen-curve maximum CL/CD/CM residuals .0875/.0095/.0125 leave 6/3/3 readings outside reading-only envelopes. Preserve those discrepancies and all NACA0009 findings. No runtime/default parameter, WWII/stall calibration or merge is adopted. A further denser/reference-transfer experiment must preselect its meaning before reusing holdouts.

### CR-3439 reference-transfer diagnostic — completed locally

**Recovered state, 2026-10-22.** Local branch/remote PR19 head: `297a8d69e76745dbb7b28cf3d200f447e4d5a431`. Worktree was clean before this documentation edit. Read-only remote checks confirm PR19 open, ready, unmerged; main remains `f0fa41e3190074b09a15b3cfae67ebb3d7b67eb5`. At 06:39 UTC the parent verified [core CI](https://github.com/4everWZ/flight-damage-sim/actions/runs/39421924482) 9/9, [source CI](https://github.com/4everWZ/flight-damage-sim/actions/runs/39421924421) 1/1 and Codex review with no findings. Recovered local summary matches the [retained numerical evidence](https://github.com/4everWZ/flight-damage-sim/actions/runs/39421924482/artifacts/12263786805); nothing was rerun. Source conditions/deflection signs and the bounded measured subset are qualified to their stated scope. CM handedness remains conditional; physical reference transfer is unverified.

**Purpose (authorized 2026-10-22).** Test whether the existing isolated section force and quarter-chord couple produce the correct wrench about another reference point. Freeze the coefficient predictor. This tests load/reference bookkeeping against an independent calculation, not interpolation improvement or physical section-to-cell accuracy (original spec §§14–15, §50).

**Data split, unchanged.** Use only `data/control-polar-cr3439-section.json`, its fixed conditions/conventions and existing PG preparation. No new geometry, table, transcription or knots.

| Role | Alpha (degrees) | Delta (degrees) |
|---|---|---|
| Six training rows | −4, +4 | −20, 0, +20 |
| Six excluded holdouts | −4, 0, +4 | −10, +10 |

Previously inspected holdouts remain excluded from export/fitting. Reuse is a frozen regression comparison, **not fresh independent validation**; held values cannot choose coefficients, reference points or tolerances.

**Hypothesis and matrix.** Preserve the existing normalized unit-area/chord cell, physical location, uniform flow, zero angular rate and actual fixture control-chain settling. For each of twelve queries, observe its unchanged load about quarter chord `Q`, leading edge `R_LE = Q + .25*c*e_chord`, and trailing edge `R_TE = Q − .75*c*e_chord`: 36 wrench comparisons. Native +X points forward. These are mathematical moment origins, not new geometry, shifted aerodynamic centers or calibrated aircraft CGs.

```text
F = q*A*(CL*e_lift + CD*e_drag)
M_Q = q*A*c*CM*e_span
M_R = M_Q + (Q − R) cross F
```

The diagnostic probe now optionally exports actual native cell force, pure couple and `Wrench::add` result at each origin. Independent Python expectations use frozen-grid interpolation and these equations, without the native reducer. The previous coefficient-only CSV did not test a nonzero reference arm; this addition exercises native reduction rather than only transforming that CSV in Python.

**Transfer boundary.** CR-3439 describes the whole section. The normalized cell is an operator surrogate. Applying its polar to existing trailing-wing cells assumes an unsupported chordwise allocation and risks double counting. Integrated CL/CD/CM do not identify pressure distribution, local skin-loss consequences or damaged control authority. This round stops at reference translation. Later physical mapping needs area/chord/reference locations, conservative force/couple allocation and independent local-loss evidence; no production binding, subdivision or damage law is selected here.

**Criteria.** Retain existing coefficient/operator and actual-angle tolerances `1e−12` (angles in radians). Preselect normalized force error `abs(dF)/(q*A)` and moment error `abs(dM)/(q*A*c)` ≤ `1e−12` per component. Wrong arm sign, duplicate CM, meaningful extrapolation or missed mechanical angle is an implementation failure; investigate without retuning data. Preserve raw endpoint clamp flags and quantified excess.

Keep the measured ledger: maximum CL/CD/CM residuals `.0875/.0095/.0125`, `6/3/3` beyond unchanged reading-only envelopes `.05/.004/.010`. These remain discrepancies, not total physical pass/fail tolerances. A translated moment gets a separately labeled propagated reading envelope: for `s = dot(Q−R,e_chord)/c`, `CM_R = CM_Q + s*(CL*cos(alpha)+CD*sin(alpha))`, and `b_R = b_CM + abs(s)*(abs(cos(alpha))*b_CL+abs(sin(alpha))*b_CD)`. It cannot replace the original CM result. Source angle/systematic uncertainty and inferred CM sign remain separate.

**Local result.** `python3 tools/control_polar_reference_transfer.py` builds the existing diagnostic probe and runs exactly 12 queries × three reference origins. All 36 comparisons pass the preselected `1e−12` normalized tolerances. Worst force error is `1.0686931437748433e−16` (train alpha +4/delta 0, quarter origin, Z); pure-couple error is `1.6698330371481926e−17` (held alpha −4/delta −10, quarter origin, Y); translated-moment error is `1.0686931437748433e−16` (train alpha +4/delta 0, trailing origin, Y). Actual-angle error and maximum boundary excess remain `5.551115123125783e−17` rad, with four flagged unique queries. Native force/pure couple are unchanged across the three observers. The new coefficient CSV is byte-identical to the retained pre-change 12-query CSV; frozen residual maxima and outside counts match the old summary exactly.

New ignored outputs are `results/control-polar-transfer-summary.json`, `control-polar-transfer-comparisons.csv`, `control-polar-transfer-native.csv`, `control-polar-transfer-coefficients.csv` and `control-polar-transfer-queries.csv`. The existing probe accepts an optional fifth output argument for native wrenches; its original coefficient output format is unchanged. Only this focused run, the retained-output comparison, Python compilation and diff review were performed; old CI was not rerun and covers remote head `297a8d6`, not this addition.

**Interpretation and next boundary.** Same-load algebraic reference translation is supported. Physical section-to-cell mapping, source CM handedness and interpolation accuracy are not established by this pass. All old failure evidence, NACA0009 data, defaults and geometry remain intact. Denser fitting, physical cell allocation or new calibration needs its own defined experiment; none is adopted. The `609c8e6` implementation was subsequently synchronized through the authorized feature-branch route; its existing core/source CI passed, as confirmed by the parent at 2026-10-22 07:07 UTC. That workflow did not yet explicitly call the 36-case diagnostic. No new physical assumption or execution blocker was encountered.

**Engineering regression continuation, local only.** The existing `control-polar-2d` CI job now calls `python3 tools/control_polar_reference_transfer.py`, with no additional job or dependencies. The command always runs the 36 positive checks and two targeted negative variants on copies of the first training point's leading-edge output: reverse or omit its native-observed arm contribution. Both reuse the same independent wrench checker and must fail specifically at `moment/wmy`; acceptance or an unrelated rejection fails the regression. No new flow query, geometry, source row, physical assumption or loosened threshold is introduced.

Local validation passes all positive checks and rejects the two faults at normalized errors `.44454381062931897` and `.22227190531465948`. All five original transfer output files remain byte-identical, including residual/failure records. New separate evidence is `results/control-polar-transfer-negative-summary.json`; the existing artifact glob already covers it. Difference review confirms the independent FRD force/couple/arm formula, original `1e−12` tolerance and six/six split are preserved. This establishes regression sensitivity to the chosen faults only, not physical validation or exhaustive mutation coverage. The change is retained in a local commit without push/merge; remote CI execution of the new step remains unverified. No unrelated tests or CI were rerun.

### Other continuing prototype work

1. Finite-span/induced-flow reference is now measured independently. Next test an explicitly new opt-in section-data fixture and strip/cell load boundary, preserving real controls/structure; surface data meaning before any production adoption. Then prioritize local stall validation and calibrated propeller effects. The reference excludes arbitrary perforations, transient/stalled wakes and live 120 Hz cost. Clock experiments are finished: preserve the original failed 10% hypothesis and unresolved sampled-production versus continuous-limit interpretation; no new clock patch is selected.
2. Calibrate and extend the signed prop boundary before historical or transonic performance claims: independent blade/engine data, compressibility/tip/swirl/Re limits and shutdown/feathering damage. Numerical signed power balance, finite pitch governance and rotor/body flight ledgers now pass; this does not validate the synthetic airframe. Preserve the legacy three failed hypotheses. Broaden physically useful speed/load cases rather than tuning away those failures. Detailed sources/equations/measurements belong in [current decisions](../decisions/current-decisions.md).
3. Horizontal target-rate feedforward and consistent normal-load feedback merged in PR #11; retain their measured bounds. Keep legacy/off/P-only comparisons and failed altitude hypotheses visible. A later vertical-rate, stop/reversal or wider speed-envelope claim needs its own bounded evidence; no altitude-hold semantic is adopted.
4. Extend material mass bookkeeping and spanwise graph geometry/load mapping, then test large skin loss, critical spar hits and secondary detached-piece fracture. Explicit named material removal now updates the current component and accounts for removed parcels; automatic geometric damage-to-material mapping and full structural materialization remain pending. Current generalized graph statics are not calibrated local beam stresses.
5. Combined damaged linkage reversal, 20-flight handoff and 16-case sustained-load continuation are measured; preserve the negative full-stick-implies-5G observation and honest non-equilibrium entry residuals. Next improve physically justified load mapping/calibration or the local aero/control envelope, distinguishing aerodynamic bias from incremental authority. Do not retune damage/strength to manufacture a failure or a trim equilibrium.
6. Run the viewer on a desktop using the documented startup modes and record human observations. Cloud smoke validates transport/rendering only. A future FlightGear adapter still needs explicit aim/ownership and cell/structure telemetry.

## Blockers

No known build/test blocker; PR #13's configuration fix is integrated. This core iteration must observe its own PR and post-merge workflows; earlier green CI is not current-head evidence. The gh GraphQL PR-status call was forbidden; GitHub connector reads provide the remote workflow/status evidence instead. Cloud has no graphical display, so interactive human handling cannot be established here. The upstream APEX PowerShell package validator needs `pwsh` and the official `skill-creator` package; neither is installed, and neither is required to run this core. Historical aircraft data, detailed material calibration and full v0.1 acceptance remain project work, not completed claims.
