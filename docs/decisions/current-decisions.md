# Current decisions and evidence status

Updated: 2026-10-17. Core, Instructor and signed-propulsion iterations are on main; this is not full v0.1 acceptance.

## Authority and provenance

The user requests core-first development and testing in this repository's Codex Cloud environment, followed by a simple existing visualization/control tool. C++ and Python are acceptable according to fit. Preserve the control, flight, and damage requirements.

The [original 51-section v0.1 specification](../spec/flight-sim-core-v0.1.original.md) is an unchanged historical assistant-authored design baseline. Its Unity/C# recommendation is historical. [User requirements](../sources/user-requirements.md) distinguish actual user intent from assistant recommendations and unverified descriptions of commercial games.

## Confirmed direction

1. Force/moment-driven 6DoF, 120 Hz fixed steps, independent rendering cadence, and distributed local aero cells.
2. Independent aerodynamic, structural, and control-system representations, with explicit cell-to-structural-node and cell-to-control-chain mappings.
3. Keyboard owns only its active axes at full requested authority, with smooth release to the Instructor. Both paths pass through the same real surface/linkage/hinge constraints.
4. Remaining section changes stiffness and stress capacity; loads redistribute; accumulated material strain under actual loads can exhaust ductility. Separation requires loss of graph connectivity. No HP, direct pose control, or scripted wing destruction.
5. Start with one synthetic parameterized propeller validation airframe. Historical aircraft, more airframes, ballistics, and a polished combat game follow core validation.
6. Keep the original specification unchanged and track partial completion in the current plan. Open draft PRs while checks/review run. The user subsequently authorized automatic merging of bounded core changes that require no human testing, with physical review, checks and post-merge verification.

## Implementation choice and evidence

Use the owned C++20 core in `include/flight/` and `src/{core,propeller}.cpp`. GNU make/GCC were present in Cloud; no CMake, JSBSim, or FlightGear installation was required. Python's standard library drives the same compiled core via scenario CLI and a tick-based stdin/stdout protocol. Python contains no alternate flight model.

The choice now has practical fit evidence: the same core trims, integrates translational/rotational motion, evaluates 152 local cells, arbitrates mixed input, transmits damaged surface state, redistributes structural load, and updates mass/CG/full inertia at separation. `integrate_body(Body, MassProperties, Wrench, dt)` is an explicit replaceable mechanics boundary. This does not establish superiority over JSBSim; no JSBSim adapter/comparison has been measured. Retain it as an optional future reference as described in [engine research](../research/engine-options-research.md).

Matplotlib was available and provides a first engineering viewer with live aim, keyboard, damage injection, cell-force arrows and structural utilization. Its input/state/render transport passed an Agg smoke test. Cloud has no GUI display, so interactive human handling quality remains unverified. This viewer is not the original specification's finished third-person sky/terrain playable prototype; FlightGear remains a possible later external viewer.

APEX's two skills are copied unchanged into `.agents/skills`, with the upstream MIT license. The inspected upstream supplies a skill layout and validation script but no installer. This is a repository-local installation, not a global plugin/hook or security-policy change. Its official PowerShell package validator was not available; copy identity and local links were checked structurally.

## Physical model and uncertainty

- SI units; body x forward/y right/z down, local world north/east/down. Body position/velocity reference the current CG; aircraft geometry is measured from a fixed body datum. Quaternion maps body to world. Gravity is world +z.
- Every cell evaluates local velocity (`body velocity - wind`, plus angular velocity cross lever arm and axial propwash), local incidence, table-driven lift/drag/moment, actual deflection and skin state. Static post-stall tables and a bounded compressibility approximation are implemented. No dynamic stall hysteresis, inter-cell downwash, calibrated spin model, flaps or slats yet.
- Structural nodes carry generalized bending/torsion rotations, with parallel front/rear/skin connections. Graph statics solve stiffness-weighted rotations. Section scaling and a phenomenological plastic-strain/softening law change stiffness and load margins. Cell moments and translational/rotational inertial relief feed the graph. Elastic rotation feeds the next aero evaluation. This is a reduced load graph, not FEM or a validated fracture law; spanwise beam kinematics, shear/axial stresses, thermal strength and detailed crack mechanics remain incomplete.
- Separation partitions connected components. Each piece inherits the original rigid velocity field; mass, CG and parallel-axis inertia are recomputed. Linear/angular momentum at the split is tested. Detached pieces now evaluate owned cells and integrate their own force/moment-driven motion. Mapped surfaces lose pilot connectivity, retain mechanical jams or relax to neutral. Further fracture, detached propulsion and collisions remain pending.
- Each control chain has range, slew, pressure effort, friction, lag, hinge fraction, jam position and disconnect state. Direct input and Instructor share the chain. Disconnect, jam, axis override and release have focused tests; the other damage modes still need dedicated behavioral coverage. Trim fades with command magnitude to preserve full surface travel endpoints.
- The default viewer fixture uses signed blade-element propeller loads, uniform axial induction, output-shaft inertia and a finite pitch governor. Actual prop moments replace the legacy P-factor offset. A coupled rotor/body step accounts for internal motor work, external torque and gyroscopic momentum. Feathering, starter/engine thermodynamics, calibrated blade/engine data and richer propulsion damage remain pending. The earlier CT/CQ model stays available as an explicit comparison with known failures.
- `data/validation.airframe` contains synthetic geometry, mass/inertia, tables, mechanical limits, structure and propulsion parameters; `tools/make_fixture.py` reproduces it. No numbers are presented as historical measurements. Initial propulsion parameters could not balance drag at 70 m/s; the explicit synthetic prop coefficients were revised before acceptance. Elastic feedback was incorporated into trim after measuring the initial trim drift.
- The default Instructor tracks an air-relative flight-path direction, with a normal-load outer loop, coordinated body-rate requests and guarded PI rate feedback. Its requested bank limit is 40°; physical attitude may overshoot it. Keyboard authority is unaffected. Legacy and off modes and a P-only ablation remain available on the same corrected dynamics. It does not guarantee stall/spin avoidance or achievement of infeasible requests; envelope-aware guidance, richer trim assistance and subjective handling remain open.

## First-slice measurements (historical reference)

Run `python3 tools/experiments.py`; it builds, runs core assertions and produces CSV/JSON. `python3 tools/viewer.py --smoke` exercises the actual live input/damage transport and saves a debug image. `python3 tools/plot_results.py` plots the measured CSVs. Generated files are ignored under `results/`.

The following first-iteration measurements preceded the mechanics review and new Instructor. Re-running now uses the corrected core and default new controller; fresh JSON/CSV results are authoritative for current behavior.

| Experiment | Observed result | Evidence boundary |
|---|---|---|
| 70 m/s, 1000 m trimmed flight | Normalized residual ~7.9e-10; 10 s altitude change ~1.4e-7 m | Internal equilibrium of the same synthetic model, not historical fidelity |
| Mouse 90° heading request | ~89.43° after 30 s | Surface-driven guidance; altitude/handling still need improvement |
| Roll keyboard override | Request -1; direct roll weight 1; pitch/yaw direct weights 0 | Full per-axis request; actuator/physical limits still apply |
| Release | Smoothstep blend reaches Instructor in 0.25 s | Command continuity and finite handoff; viewer smoke uses actual transport |
| Remove 8 of 32 left outer-wing cells | At +0.1 s: ~1.58 kN less lift and -3.40 kN m roll-moment change | Clean, asymmetric and symmetric cases; skin loss leaves structure attached |
| Left aileron cable disconnect | At 2 s: ~50.7°/s roll vs ~71.4°/s intact | Same direct roll request, lower actual authority |
| Weaken front spar to 30% section | 1G bench survives 3 s; imposed 5G starts at 3 s, first failure ~3.317 s and separation ~3.325 s | Explicit structural bench, not aircraft maneuver |
| Coupled high-speed pull | Peak ~4.84 G; intact survives; weakened first failure ~4.775 s and separation ~4.792 s | 140 m/s transient entry; fixed-pitch power cannot sustain initial speed. Pull begins at 3 s, low-load phase survives |
| 40 m/s low-speed entry | Local cells enter stall and subsequently recover | Cell table states; not a validated historical stall/spin envelope |
| Same binary replay | Identical mixed-input telemetry | No randomness introduced yet; no cross-platform bitwise claim |
| 120 vs 240 Hz | Mixed-case final speed differs ~0.065 m/s, heading ~0.00044 rad, altitude ~0.69 m; structural first-failure time differs ~0.0042 s | Focused convergence checks only |

## Mechanics review and second-iteration choice

Independent counterexamples exposed four defects, then exercised the repaired authoritative core:

| Counterexample | Finding before repair | Repair / fresh evidence |
|---|---|---|
| Torque-free rotation with full off-diagonal inertia | 5 s energy drift ~2.75%, world angular momentum drift ~0.97%; halving dt only halved energy error | Implicit midpoint Euler equation solved with Newton plus matching Cayley quaternion increment. Energy/H drift is at rounding level; attitude error decreases ~4.2× at 240 Hz vs a 960 Hz reference. A slender high-rate body is also finite/conservative |
| Flexed wing with lateral local flow, zero drag | Rotating a computed lift vector afterward introduced ~6.78 kW spurious lift work | Evaluate incidence in the flexed chord/normal basis and lift from local flow cross span. Pure lift work is ~0 (normalized projection <4e-17) |
| Configured 0.5 s handoff with no key ever pressed | Initial direct ownership weight ~0.475 | Initialize release time from the configured handoff; initial Instructor ownership is complete |
| Damaged trim | Solver wrote nonzero positions through a disconnected or jammed elevator | Initialize actual surfaces through connectivity/range/jam/pressure/trim capability; regularized least squares tolerates absent axes and returns honest residual. Cut elevator stays 0, jam stays -0.07 rad, unavailable trim residual ~0.53 rather than fabricated equilibrium |

Additional checks pass: splitting each cell into two half-area cells preserves the wrench; explicit cell reduction reproduces aero totals once; mirrored geometry/state and yaw/wind transformations preserve expected signs; symmetric actual loads give opposing wing bending; weak structure in freefall with no aero/prop forces accumulates no strain despite full keyboard pull and 10 s elapsed. Two simultaneous groups split at a tilted attitude with off-diagonal inertia, preserving total mass, linear/angular momentum, kinetic energy and the surviving datum's position. The reduced graph still uses generalized moments about the fixed datum; local beam pivots/shear/axial stresses are not yet a validated spanwise model. At that mechanics-review stage, local material removal was not resolved in `MassPart`. The explicit material-inventory follow-up below adds a conservative second mass-change boundary; it does not infer material loss from old aero/section parameters.

`TrimResult.residual` is the Euclidean norm of six scaled equilibrium components: body `Fx,Fy,Fz` including gravity divided by current weight, and body `Mx,My,Mz` divided by **10,000 N m**. It is not an accuracy score or a stability/energy/stall-margin test. Raw force/moment residual vectors and rotor acceleration are now exposed separately; `converged` also requires near-zero rotor acceleration. The legacy-propulsion clean 70 m/s case has ~1.7e-6 N force residual, ~7.8e-6 N m moment residual and ~1.5e-14 rad/s² rotor acceleration. Damaged/disabled actuators cannot be bypassed by initialization.

### Aim semantics and comparative experiment

New mode interprets `aim_world` as the requested **air-relative velocity direction in world coordinates**. Its elevation requests flight-path angle, not a nose attitude or an altitude setpoint. Zero elevation requests level air-relative motion; vertical wind can change the ground trajectory. Direction is a request subject to available thrust/lift/control. No altitude-position feedback, direct pose write or corrective torque was added.

Normal-load feedback, gravity/turn-rate kinematics and rate PI generate normalized surface requests. Integral bias freezes on directly owned axes, during handoff, with unavailable control chains, at command saturation when error would wind up, and under positive-AoA protection. Surface range/slew/pressure/linkage/hinge constraints remain common to all inputs. Positive sideslip now requests positive yaw in FRD; the legacy negative feedback sign is retained only within the frozen comparison controller. Saturation diagnostics mean **normalized controller output saturation**, not proof of sufficient aerodynamic authority.

`python3 tools/instructor_experiments.py` compares off, legacy, new P-only and new PI on the same corrected dynamics, with identical aim/throttle/damage inputs. It also sweeps bank requests and compares 120/240 Hz. The rejected first P-only attempt lost ~106 m; the retained corrected P-only ablation at 40° loses ~89 m. PI bias is needed to hold the changed surface equilibrium without manufactured torque.

| 30 s horizontal 90° request | Lowest altitude change | Final altitude change | Actual velocity heading | Pitch output saturated |
|---|---:|---:|---:|---:|
| Legacy heuristic | -41.22 m | -31.57 m | 89.26° | 0 s |
| New P-only, 40° | -98.63 m | -89.43 m | 90.04° | 0 s |
| New PI, 40° | -18.60 m | -0.20 m | 89.99° | 0 s |
| New PI, 50° | -42.16 m | -20.94 m | 89.97° | 4.91 s |
| New PI, 60° | -83.55 m | -66.23 m | 89.95° | 5.76 s |
| New PI, 40°, 8/32 left outer cells lost at 1 s | -22.14 m | -3.48 m | 89.83° | 0 s |

40° is a reversible default chosen for this synthetic fixture's measured support/control margin, not a universal limit or a clamp on actual attitude. Keyboard full authority still overrides its axis and can exceed Instructor protections. The new clean case enters ±5° heading error at ~14.04 s versus ~18.88 s legacy, ends with ~-0.02° sideslip and 67.86 m/s, and still has a transient ~18.6 m drop. Recovery exchanges energy; no zero-loss or cost-free maneuver claim is made. At 240 Hz, final altitude differs ~0.0012 m, speed ~0.00055 m/s and velocity heading ~4.7e-8 rad.

A disconnected elevator still loses ~247 m by 10 s and reaches the reference ground plane at ~22 s; it cannot be restored by PI bias. No collision model exists, so later raw telemetry is a mathematical continuation below that reference plane, not validated post-impact flight. An 8° climb request ends around 3.64° and 50.18 m/s after 30 s with ~5.34 s pitch saturation: current power/angle/control limits prevent full tracking.

The broader hypothesis that every turn has less minimum height loss **failed**. In a 60 s continuously rotating horizontal aim (0.08 rad/s), legacy climbs ~13.87 m, while new PI initially descends ~8.10 m and finishes ~5.05 m down. New flight-path error RMS after 20 s is ~0.068° versus ~0.218° legacy, with heading lag ~13.19° versus ~29.61°. This supports improved direction/path tracking, not universal altitude preservation. The failed hypothesis stays in generated `instructor-summary.json`; original requirements were not rewritten to make it pass.

### Moving-heading tracking and consistent normal-load feedback

The next production control iteration starts after the diagnostic PRs #9/#10 merged. Two mechanisms are changed in the owned C++ Instructor; no aircraft polar, cell-force law, structure, control-chain damage semantics or rigid-body integrator changes.

**Moving aim:** the previous horizontal guidance requested only `heading_gain*heading_error`. Following a constant heading rate therefore requires `error=rate/heading_gain`: 0.08/0.35 predicts **13.096°**. The actual pre-change 60 s legacy-prop flight had **13.188°** heading RMS after 20 s. A wrapped difference of world aim headings now supplies a filtered rate (0.15 s exponential filter), added before the existing bank-request limit. The stored rate is bounded by the same guidance bank ceiling; this does not certify available aerodynamic authority. Filtering precedes limiting so 30/60 Hz sample-and-hold inputs are not discarded at short physics ticks. Inactive mouse/FlightPath guidance and vertical aim invalidate history; reactivation starts without a stale derivative. Keyboard ownership, handoff, PI guards and the common real actuator path remain authoritative.

**Rejected first route:** feedforward alone reduced heading RMS to 0.094° / 0.160° on the legacy/BEM fixtures, but BEM path-angle RMS was **0.1527°**, failing the retained **0.15°** guard. The first implementation commit and local first-attempt result retain that failure. Trying 0.5 s / 1 s filtering did not fix it (BEM path RMS **0.15314° / 0.15337°**); no slower-filter tuning is adopted and the temporary parameter is removed.

**Normal-load mechanism:** the existing sensor is aerodynamic **body-Z force divided by weight**, not total force normal to the flight path. The old `1/cos(bank)` conversion ignored body pitch/flow angle and propeller support. On actual force-balanced BEM trim it still generated a roughly **0.00555 rad/s** pitch request. With unit upward path normal `n` expressed in body axes, total cached force `F=Faero+Fprop`, weight `W`, and `k=max(-n.z,0.3)`, the consistent body-Z aerodynamic target is

`N_aero_target = [cos(gamma) + V*path_gain*(gamma_aim-gamma) - (F.x*n.x+F.y*n.y)/W]/k + Fprop.z/W`.

The same −0.5…5 target bounds and existing rate/actuator/AoA limits follow it. The measured `load_factor` keeps its old definition. No extra force evaluation, synthetic torque or attitude update is introduced: guidance consumes the existing cached force samples. A banked trim can still request a small coordinated body rate while leveling; force balance implies zero **normal-load error**, not universally zero total body-rate request.

The corrected normal feedback and moving-rate ablation give the following **real 60 s flights**, using the same 0.08 rad/s horizontal path. RMS is measured against the common continuous intent after 20 s; sampled inputs include their actual delay.

| Synthetic fixture | Corrected loads, rate FF off: heading RMS | Rate FF on: heading RMS | Rate FF on: path RMS | 30 Hz input: heading RMS | Local wing loss, 30 Hz: heading RMS |
|---|---:|---:|---:|---:|---:|
| Legacy propeller | 13.185° | 0.090° | 0.0574° | 0.161° | 0.383° |
| Signed BEM/material | 13.243° | 0.149° | 0.0804° | 0.220° | 0.459° |

All sampled-input comparisons remain below 1° heading RMS; these turns have no stalled cells or structural failure. For the same 30 Hz input, 120→240 Hz changes final altitude by **0.00632 / 0.00538 m** and heading by **0.000461 / 0.000245 rad**. Switching rate FF alone leaves each corrected fixed-aim flight identical. This does **not** mean the normal-load correction leaves the old stationary-aim trajectory unchanged. Original core/Instructor acceptance checks are retained separately.

Altitude is not held: corrected continuous turns end **12.15 / 13.86 m lower**, versus **11.82 / 13.37 m lower** with rate FF disabled in the same corrected controller. Peak loads with FF are about **1.224 / 1.223 G**. The earlier universal height-loss hypothesis remains false; no free maneuver, universal recovery, historical handling or human GUI acceptance is claimed.

Regression evidence uses the actual update/flight paths: the rotating-angle observer follows its analytic exponential response across ±π; reset/vertical-input cases clear it; full direct-axis ownership and initial handoff continuity hold. With every pilot linkage disconnected, enabling FF produces the same actual 6DoF trajectory. Controlled force-balanced attitude/wind/propeller-split cases and actual trim have zero normal-load deficit (the trim check uses <1e−9); the existing flat-bank nose-versus-path test still demands zero pitch request.

The new projection reads cached vector loads and current mass, so controller-history records now explicitly include those inputs as **`controller_loads_v3`**. Replay never invents missing forces from old scalar G: old histories are rejected with instructions to preserve them and record a new prefix. Full force vectors use the existing causal force-age reconstruction and current-mass normalization; native identity bounds and original clock/convergence thresholds are unchanged. Previous clock numbers below describe the earlier controller. They are not relabeled as current-controller results or used to claim the original 10% failure is repaired.

Reproduce with `python3 tools/aim_tracking_experiments.py`. It writes `aim-tracking-summary.json` plus real runner CSVs; CI has a parallel `moving-aim` job and retains representative default-fixture traces. The scenario runner accepts optional `[aim_hz=0] [aim_rate_ff=1]` after the bank limit, allowing the same physics/input-cadence and feedforward ablations. Existing `PilotIntent` aim-direction semantics are unchanged; vertical aim-rate feedforward is not implemented. For a new complete-input history without overwriting older traces, use `python3 tools/sensor_history_experiments.py --history-prefix results/path-projection-history --output-prefix results/path-projection-history`.

## Legacy propulsion bench (retained counterexamples)

The first experiment after the mechanics/Instructor iteration isolates `advance_propulsion`, the same authoritative C++ component called by flight. It drives controlled axial airspeeds 0/40/70/100/140 m/s, powered spin-up, shutdown and stopped/removed prop cases at 120/240 Hz. These are component benches, not constrained aircraft maneuvers or a second Python model.

It found a shaft-reaction mismatch during spin-up: rotor acceleration used engine torque computed before the RPM step, while the fuselage reaction was recomputed from power divided by post-step RPM. At 600 initial RPM, mismatch reached ~33.72 N m. Applying the same shaft torque to rotor and mount repairs the reaction pair; measured maximum error is ~6.1e-10 N m. This is an axial reaction consistency check, not full multi-axis gyroscopic/rotor-body energy validation.

Powered useful thrust power stays below shaft power in the sampled envelope (maximum useful ratio ~0.411); steady RPM differs ~0.0018 between 120/240 Hz. These bounds do **not** validate the aerodynamic propeller model. Three substantive hypotheses failed: a stopped intact prop at 80 m/s has zero drag, wind cannot start windmilling from rest, and fixed-pitch shaft load is unchanged across the airspeed sweep. At shutdown from rated RPM, the model decays to ~189 RPM after 30 s with ~617 N drag; this is rotor coasting with an extrapolated thrust curve, not a validated windmilling equilibrium. Identical RPM across speeds is an artifact of constant CQ, not an implemented governor.

`python3 tools/propulsion_experiments.py` preserves these failed hypotheses in `propulsion-summary.json` alongside implementation consistency predicates. No arbitrary drag correction or RPM lock was added to make them pass. The next discriminating model experiment is signed CT/CQ as a function of advance ratio and blade pitch, or explicit blade elements, including zero-RPM drag and wind-driven torque. Test that boundary before adding a governor that actuates blade pitch through finite range/rate limits. That legacy model has no governor. The following signed model addresses this boundary.

## Signed blade model and pitch governor

The changed hypothesis was that the same signed local blade loads can produce stopped-prop drag, wind-driven spin and airspeed-dependent shaft load, while a real pitch actuator regulates speed without assigning measured RPM. `src/propeller.cpp` implements this boundary; `tools/rotor_experiments.py` calls the authoritative C++ component and then free-flight scenarios. There is no test-only aerodynamic switch or second Python physics model.

### Sources, equations and assumptions

[MIT Unified propulsion notes](https://web.mit.edu/16.unified/www/FALL/thermodynamics/notes/node86.html) give blade lift/drag resolution into thrust and resisting torque, advance ratio `J=V/(nD)`, and ideal disk momentum/power. [NASA Glenn's propeller explanation](https://www.grc.nasa.gov/www/k-12/airplane/propth.html) distinguishes the rotating twisted airfoil from the ideal momentum disk. [JSBSim's public propeller interface](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGPropeller.html) and [implementation](https://jsbsim-team.github.io/jsbsim/FGPropeller_8cpp_source.html) expose advance/pitch-dependent loads and a governor acting on bounded pitch, including low-RPM actuation restrictions. They informed this boundary; no aircraft tables or JSBSim code were copied, and no adapter/comparison was tested.

Own implementation: 12 radial × 8 azimuth strips sample two blades. Local flow includes signed axial/crossflow, body rotation, relative rotor spin and a uniform induced velocity. Full-angle table lift is perpendicular to actual strip flow; drag opposes it. Summing forces at blade positions gives mount force and moment, with `Qa=-M·axis`. No division by RPM is required, including at rest. The signed closure `T=2ρA vi |V+vi axis|` is solved numerically; induced dissipation is `T vi`. Oblique/reversed/braking cases are mathematical continuations of this reduced closure, not validated crossflow or vortex-ring physics. Profile and induced loss close the full blade power balance, including body-rate moment work. There is no swirl, tip-loss, Reynolds or compressibility correction.

`validation-bem.airframe` retains the original cell/structure/control/mass geometry, with a **synthetic output-shaft map** of 500 kW / 1600 RPM (instead of legacy 850 kW / 2600 RPM), 1.5 m radius and 30 kg m² rotor inertia. It is a propulsion variant of the same validation body, not a historical aircraft. Locked body inertia includes rotor spin inertia; stationary fuselage inertia is that tensor minus `Ir aaᵀ`. Twist/chord stations and a synthetic full-angle polar are emitted by `tools/make_fixture.py`: attached-flow slope 5.7/rad, blended to a flat-plate curve over 15–30°, with positive profile/post-stall drag. This change avoids using the old rated prop-tip speed above Mach 1 with an incompressible blade model; it is not calibration to a historical engine or performance target. Clean tip Mach is still ~0.786 and high-speed entry ~0.874, so numerical consistency does not establish compressible accuracy.

Governor PI commands collective pitch through 8–55° bounds, ≤10°/s slew, travel/rate damage, jam and a low-RPM pump factor. Opposite error unwinds stored bias; saturation cannot override actuator limits. A first anti-windup attempt held a windmill at ~3755 RPM/minimum pitch because it blocked unwinding; the repaired sign-aware rule reaches the measured equilibrium below. `PilotIntent.prop_rpm_ratio` changes the target (0.5–1.1 rated), never measured spin. Offline trim solves actual pitch or shaft torque equilibrium and reports infeasible residuals; flight integrates spin once per tick.

For relative rotor speed Ω, locked body inertia I and axis a, total rotational momentum is `H=Iω+IrΩa` and kinetic energy is `½ω·Iω+IrΩ(ω·a)+½IrΩ²`. `integrate_rotor_body` jointly solves midpoint translation, world angular impulse and `Ir(ΔΩ+a·Δω)=dt(Qmotor−Qa)`, with matching Cayley attitude. The engine reaction is internal; aerodynamic blade torque is external. An earlier body-coordinate torque discretization left ~0.0247 N m s angular-impulse error in mixed input; using the same midpoint world orientation for angular impulse removes that inconsistency. Distributed aero was initially frozen at the start of each 120 Hz tick; the midpoint-aerodynamics correction below supersedes that discretization. The ledger audits the discrete rigid/rotor model, not continuous-model accuracy or aeroelastic energy conservation; structural elastic energy and a complete wake-energy ledger are not in it.

### Measured evidence

| Experiment | Actual observation | Boundary |
|---|---|---|
| Stopped intact prop, 80 m/s, ρ=1.1 | ~−1694 N axial force, ~−991 N m resisting torque; starts windmilling | Isolated imposed-flow bench; removed blade area gives zero force/torque |
| Signed spin/advance/crossflow/body-rate sweep | Full blade power residual <2.3e−15 relative; nonnegative profile+induced loss; induction converges | Includes reverse spin/flow as numerical extremes; not calibrated regimes |
| Fixed vs governed, 70→110 m/s at 15 s | Fixed RPM ~1983→2550, governed ~1591→1620; governed ends 1600.36 RPM at 36.07° pitch after throttle step | 40 s component bench, target 1600; fixed/governed use identical loads |
| Stopped, engine off, 80 m/s | Spins up from zero, overshoots ~2280 RPM and ends ~1600.23 RPM | Wind supplies energy; motor friction remains; no starter or feathering |
| Pitch jam / travel fraction 0.12 | Final ~2617 / 3864 RPM; actuator reports saturation; rate ≤10°/s | Overspeed numerically retained; incompressible model does not predict transonic reality |
| Free tilted rotor/body, full inertia, no prop area | Energy minus internal motor work <2.2e−14 relative; world H <3.3e−15 relative; power-off conserves both | C++ focused counterexample plus ASan/UBSan pass |
| Six pre-separation flight ledgers | Energy relative residual ≤7.9e−11; world torque-impulse residual ≤3.8e−5 N m s | Clean, mouse, mixed, local aero loss and intact/weakened high-speed entry; single-parent ledger stops before separation |
| 70 m/s clean trim | Scaled residual ~3.8e−10; raw force ~1.1e−6 N, moment ~3.8e−6 N m; 10 s level state maintained | Internal equilibrium, not historical validation |
| 30 s 90° mouse request with signed prop | ~89.919° actual velocity heading, minimum altitude −17.074 m, final +2.446 m; peak RPM error ~7.35 | Same surface-driven Instructor, changed synthetic propulsion; no universal handling/altitude claim |
| Weakened spar + coupled pull | Survives before pull at 3 s; first failure ~4.158 s, graph separation ~4.175 s; intact survives 10 s | Actual loads; 140 m/s entry is nonconverged (~5.76 kN / 5.14 kN m residual), not sustained cruise |
| 120/240 Hz | Governor final difference ~0.0026 RPM; mixed flight differences ~0.169 m altitude, 0.0382 m/s speed, 0.0137 RPM, 0.000214 rad heading | Focused discrete convergence |

The old stopped-prop, wind-start and speed-load failures remain reproducible with `validation.airframe` and `propulsion_experiments.py`; those false hypotheses are not relabeled as success. The new viewer/live client defaults to `validation-bem.airframe`. G toggles the governor, F7 jams pitch, PgUp/PgDn change requested RPM, and reset preserves selected controller/governor mode. Agg smoke checks actual target/rotor separation, jam and reset through the protocol; human handling remains untested.

## Reproduction and automation

`make test` runs analytic/core and independent mechanics regressions. `python3 tools/experiments.py` covers the existing core slice; `python3 tools/instructor_experiments.py` produces fresh controller comparisons and failed hypotheses. `python3 tools/rotor_experiments.py` adds signed loads, pitch damage/governor and coupled rotor flight ledgers. `python3 tools/viewer.py --smoke` tests mode selection, reset persistence, integrator freezing and prop inputs through the actual adapter. `python3 tools/viewer.py --controller legacy` or `--controller off` launches comparisons locally; M cycles modes. No GUI handling claim is justified by these automatic tests.

The first commit had no Actions runs/status checks. A minimal repository workflow builds these same tests, runs core/Instructor/legacy propulsion/signed rotor experiment entrypoints and the Agg viewer. The second-iteration remote run passed all build, experiment and render steps; each subsequent pushed head must be checked separately. Local passes are not a remote-CI result. Focused ASan/UBSan mechanics review also passes in Cloud.

## Viewer delivery checkpoint

[Viewer operations](../viewer-operations.md) now own the minimal isolated startup, complete controls, telemetry interpretation and manual review checklist. GUI delivery uses TkAgg with native FocusOut plus Matplotlib figure-leave input release/pause. All built-in keymaps are disabled; three trim inputs are available; reset clears held axes/knobs and presentation clock balance while preserving controller/governor/pause choices. Opposing keys keep their axis owned with zero request. Action latches and deferred Tk release/press pairing prevent repeat toggles. Pause drops wall-time backlog; slow presentation runs at most 12 fixed ticks per frame without changing core dt.

`--smoke` dispatches actual Matplotlib events through the viewer and C++ transport. It checks conflicting/held/opposing keys, mode/governor press repeat, knobs, reset, focus loss, pause/resume/fractional tick and long presentation stall, damage/reset and rendering. A real headless Tcl idle queue verifies the native repeat coalescer when tkinter is available (present in Cloud). The repository-local virtual environment also passed the same smoke with its installed Matplotlib. No core force/control/structure model changed in this delivery step. The marked PNG and JSON are automatic evidence, not native window focus or human flight-feel results.

The first delivery stopped at this review milestone; the user subsequently tested/fixed it, authorized merging PR #1, and resumed automatic bounded core iterations. Desktop Tk focus/Alt-Tab, key repeat/layout, realtime performance, aim/control clarity and damaged handling are the actual outstanding human checks. Later core limitations stay in the current plan; no automatic full-game expansion is implied.

Desktop review reported very low FPS and background memory after closing. Targeted Windows timing identified per-frame axes rebuilding/full rendering as the main cost, with long HUD text additionally expensive under installed Matplotlib 3.11.2. The viewer now reuses artists, blits dynamic layers over a cached background, leaves help text static and accounts for rendering time in its refresh timer. Viewer dependencies use `<3.11` following comparison with 3.10.9. Physics/control/telemetry semantics and fixed 120 Hz are unchanged. [Viewer operations](../viewer-operations.md) records measured timings and reproduction commands; the native short test measured ~13.8 FPS, so this is a measured improvement, not a 30 FPS guarantee or completed human handling acceptance.

Closing the window stops its timer and pending key-release callbacks; normal exit and exceptional GUI unwinding close the owned core. Core cleanup is idempotent, bounds graceful wait and escalates terminate/kill if needed, then reaps the child and closes pipes. Real-process regressions cover normal, already-exited and unresponsive children; a desktop Tk integration test covers resize/redraw and window shutdown. These passed locally along with Agg input smoke. The two Python background processes observed during investigation were VS Code autopep8 language servers, not viewer/core remnants; they were preserved. The user reported passing CI before and after the authorized merge of PR #1 into main `9f0bd7c`.

Desktop review also reported reversed Q/E rudder response. Direct probes found the core signs correct (after 24 ticks, yaw rate ~−0.150 rad/s for Q and +0.150 rad/s for E); the viewer's North/East/Up projection was a reflection. Cell positions, force arrows and fuselage now consistently use right-handed East/North/Up display axes. Bindings and physical dynamics remain unchanged. Agg callbacks now check both actual yaw signs and displayed nose headings for Q/E, plus the display basis handedness; this passes locally. Human confirmation of the corrected view remains pending.

## Independent aerodynamics after graph separation

The next owned C++ mechanism starts from merged main `9f0bd7c`, retaining user commit `1d6d837`. Previously, connected-component partition updated mass/CG/inertia but pieces then coasted ballistically. The same cell law is now shared by parent and detached owners. A detached body stores its disjoint cells, local mapped control branches and the elastic/incidence basis at separation. Each tick evaluates local `R⁻¹(V−wind)+ω×(cell−CG)`, density at its own altitude, cell force and moment about its own CG, then calls the existing rigid 6DoF integrator once. No scripted piece trajectory or extra applied torque is used.

Separation still follows actual graph disconnection, with the original rigid velocity field and mass/CG/parallel-axis partition. Parent cells become inactive and are never counted again there. New bodies are born at the end of the current tick and begin integration on the next tick; their birth load evaluation is diagnostic only. Stable cell-ID damage is routed to the current detached owner. Main and component control chains are independent: only mapped surfaces are copied/remapped, their pilot paths are disconnected, unpowered non-jammed surfaces use the existing reduced neutral/lag/slew/friction response, and mechanical jams remain. A surviving shared branch on the parent keeps its pilot path. No Instructor/RPM/attitude override drives a piece.

`python3 tools/detachment_experiments.py` builds and calls the authoritative C++ counterexamples and writes `detachment-summary.json` / `detached-flight.csv`. `plot_results.py` renders the measured continuation and body rates. Tests cover left wing and elevator/rudder partitions, shared surviving control branches, opposite pilot commands, frozen shape/damage, zero-airflow and own-body angular flow, wind/yaw covariance, stable-ID damage routing, passive work and timestep refinement. Existing simultaneous tilted full-inertia partition tests retain mass/linear/angular momentum/kinetic-energy checks.

| Measured experiment | Observation | Scope |
|---|---|---|
| Tilted split, equal-density cell reduction | Force error ~1.93e−12 N; moment error ~4.87e−11 N m | Sum each owner about the original CG; shared law, no duplicate cells |
| Actual CG-based density sampling at that split | Unscaled force difference ~0.0237 N | Initial exact-force-continuity hypothesis failed because the original atmosphere samples each body's CG. Equal-density normalization isolates ownership; the actual small jump remains reported. It is not a separation impulse |
| Opposite pilot roll commands after separation | Same detached pose/rates to <1e−10; surviving parent controls differ | Uniform wind and no inter-body interaction in this model |
| Unpowered component at 70 m/s | ~13.585 kJ mechanical-energy loss over 0.5 s; sampled air-relative wrench work ~−53.804 kW in a separate probe | Synthetic zero-CM tables; lift is perpendicular to local flow, drag dissipates work; not a calibrated debris model |
| 120/240 vs 480 Hz, 0.5 s component flight | Velocity error ~0.05293 / 0.01633 m/s, ratio ~0.308 | Earlier frozen-force measurements, superseded by midpoint aero below; supports refinement, not a universal second-order claim |
| Actual weakened-spar flight and full-authority pull | Lower-load phase survives; first failure ~4.158 s, separation ~4.175 s; piece is ~60.279 m from ballistic continuation at 10 s | Same birth state for the ballistic counterfactual; 140 m/s entry remains nonconverged/transient |

The 5 s clean BEM parent flight CSV is byte-identical before/after sharing the cell calculation. Parent/rotor/controller regressions are run on the same core. The user's viewer optimization, dependency constraint, cleanup tests and right-handed display correction are preserved. The line protocol adds `detached` body/force/cell/control records while preserving the existing `debris` count and parent fields; the viewer currently still draws only parent cells.

Limits are explicit: deformation is frozen at birth; no additional component flex/fracture, no inter-body propwash/wake/collision, no detached engine or aerodynamic body form-drag partition, and no free-hinge inertia/hinge-moment model. Existing per-body CG atmosphere sampling and static cell stall/compressibility approximations remain. At this separation-only stage, local material loss still did not remove mass; the follow-up below adds an explicit inventory event. This iteration improves the separation consequence without claiming full v0.1 or historical damage/debris fidelity.

## Explicit local material mass inventory (stacked on PR #2)

The material iteration originally stacked on PR #2 at `3799d4b`. Under the user's subsequent standing merge authorization, PR #2 merged as `decb406`, then PR #3 retargeted main and merged as `f07d235`; both resulting main push workflows passed. User work and the independent-piece mechanism are retained. The changed claim is bounded: an explicitly declared material removal updates its current rigid owner while conserving total mass, linear/angular momentum and instantaneous kinetic/potential energy including the removed parcel. Existing aero-area and structural-section events keep their original semantics.

Definition row: `material node name kg x y z Ixx Iyy Izz`. It adds a named removable mass element; ordinary `mass` rows remain carrier mass. A material-bearing node requires a positive non-removable carrier, names must be unique, and the positive intrinsic diagonal inertia must satisfy triangle inequalities. `validation-material.airframe` is the same BEM validation body with one 130 kg outer-wing lump decomposed into a 110 kg carrier and two 10 kg elements. Total initial mass/CG/full inertia agree with `validation-bem.airframe` to ~1.82e−12 in the aggregate comparison. Existing fixture files stay unchanged; this is not a new aircraft or historical mass calibration.

`remove_material(name, remaining_fraction)` / live `damage material name fraction` is monotone and idempotent. Remaining mass and intrinsic inertia scale uniformly at the fixed declared centroid. The removed fraction becomes a separately integrated gravity-only 6DoF material parcel (no aero cells), carrying `V+R(ω×r)` and its intrinsic rotational inertia. The owner is recentered to its new CG with the same rigid velocity field, attitude and angular rate; surviving geometry does not teleport. No ejection impulse, invented force or extra simulation tick is applied. Inventory plus owner/component/parcel mass remains accounted for, including repeated removal followed by structural separation, or removal from an already detached wing. Root rotor spin stays with the root; it is included in instantaneous energy/H tests.

Structural inertial relief uses actual remaining element mass. Cached wrenches are re-expressed about the new CG, and the cached load factor uses the new weight; the event does not evaluate guidance sensors off the fixed-tick schedule. Aero coefficients/areas, section/stiffness and connectivity do not change automatically. Restoration is rejected; normal simulation reset reloads the clean definition.

A new small-parcel counterexample found that the original absolute inertia pivot cutoff rejected a positive ~9.9998e−12 kg parcel as singular. `Mat::solve` now normalizes matrix and RHS by matrix scale before the pivot check, making its threshold relative to matrix scale. The same linear problem is solved without adding a material mass floor, while genuinely singular matrices still fail. Existing full-inertia, rotor and structural/control regressions are retained.

| Experiment | Measured result | Interpretation |
|---|---|---|
| 8 kg local removal, tilted/rotating body with spinning rotor | Total mass error 0; P error ~2.91e−11 kg m/s; H error ~1.03e−8 kg m²/s; energy difference 0 J at reported precision | A finite rigid partition, with removed intrinsic inertia and CG reference conversion |
| Same removal | CG y shifts +0.0114613 m; Ixy becomes ~−19.0831 kg m² | Full tensor changes, not a scalar mass-only approximation |
| Gravity-only 0.5 s after removal | Total mechanical-energy error ~1.71e−6 J | Actual root/rotor/parcel integration; discrete numerical tolerance, not exact arithmetic |
| Fixed-trim 5 s, event at 1 s | Material position effect ~1.12950 m (120 Hz), ~1.12803 m (240 Hz); matched effect-vector difference ~0.001555 m | Isolated force/mass/CG/inertia response with identical held trim |
| Strong roll + flight-path PI, matched clean/damaged pairs | Effects ~0.21374 / 0.30347 m at 120/240 Hz; vector difference ~0.19395 m | **10% effect-agreement hypothesis failed**. Bulk damaged-state difference is ~0.13326 m, but the smaller material-specific closed-loop effect is not resolved; do not claim general maneuver fidelity |
| Tiny positive parcel | ~9.9998e−12 kg now integrates | Regression of the absolute-pivot counterexample; no minimum material mass |

`python3 tools/material_experiments.py` writes `material-summary.json` with passing implementation checks and the separate failed full-roll hypothesis. `tests/material_live_tests.py` exercises transfer/reset, invalid/restoring events, tiny parcels, and current ownership after actual load-driven structural failure through the real JSON interface. Live snapshots add `materials` inventory plus a `material` name on removed parcels; empty names indicate ordinary structural components. The existing `debris` count includes both kinds of rigid body.

Limits: only named material elements can be removed; full carrier erosion/deletion is not implemented. Uniform thinning/perforation preserves each remaining centroid and radius of gyration, not the exact inertia of an arbitrary shaped hole. There is no implicit aero/section mapping, parcel geometry/drag/collision, projectile impulse, heat or calibrated fracture distribution. The current one-point CG atmosphere approximation and reduced spanwise structural model remain. Native GUI was not rerun in Cloud, and the viewer still renders parent aero geometry. These are a material mass-transfer boundary and demonstrated physical responses, not full wound/fragmentation or closed-loop convergence acceptance.

## Saturation-consistent rate-PI integration

After reviewing PR #2/#3 code, user changes, comments, CI and physical claims, the user-authorized merges were performed in dependency order. The full-roll small-material-effect disagreement was judged nonblocking for the explicit mass-transfer mechanism: instantaneous conservation and fixed-trim response have independent evidence, existing input/physics semantics remain, and the stronger closed-loop fidelity claim is explicitly unresolved. Post-merge main CI passed for [PR #2](https://github.com/4everWZ/flight-damage-sim/actions/runs/38752901298) and [PR #3](https://github.com/4everWZ/flight-damage-sim/actions/runs/38753182999).

A 120–960 Hz causal probe first refreshed aerodynamic load feedback from the current state before each control step. It only changed the 120/240 material-effect vector disagreement from ~0.19395 to ~0.18773 m; the 10% hypothesis still failed. This is evidence against stale-load sampling alone explaining the result. No new production sensor sampling or gain tuning was introduced.

A separate held-rate controller counterexample exposed a precise integration defect. A pre-step saturation gate allowed a whole integral increment to cross the command bound. With proportional contribution 0.1008 and initial pitch bias ±0.899, 120 Hz stored bias ±0.89972 and exceeded reachable raw command by 0.00052; 240 Hz stored ±0.89936 with overshoot 0.00016. The final actuator request was clipped, but the stored bias depended on the crossing tick.

Production now restricts each unfrozen increment to available normalized-command headroom: positive increments cannot exceed `max(1−raw,0)`, negative increments cannot pass `min(−1−raw,0)`. The existing ±0.9 bias limit, gains, direct-axis arbitration, handoff, unavailable-control freeze, AoA protection and physical actuator constraints remain. This is command-boundary integration, not arbitrary corrective torque or direct pose control, and does not implement full back-calculation from actual actuator position/range.

`control-tests` exercises all three axes and both signs at 60/120/240/480 Hz. The held-error integral lands at analytic bias ±0.8992 with zero reported command overshoot and zero timestep spread, while unsaturated increments and real direct/release/disconnected-control histories retain their prior behavior. `tools/control_experiments.py` runs the same C++ core with matched clean/material flights through 960 Hz and a separately labeled load-refresh ablation; JSON and CSV remain under ignored `results/` and CI preserves `control-evidence`.

The full-roll sensitivity remained unresolved at this stage. After clipping, default material-effect norms at 120/240/480/960 Hz are ~0.20525/0.29250/0.35480/0.39256 m. Adjacent effect-vector differences are ~0.16686/0.08628/0.04486 m. The 120/240 disagreement is smaller than ~0.19395 m before the change, but still fails the original 10% criterion; higher-rate differences also do not establish a general resolution bound at 120 Hz. The before/after held-rate invariant is the accepted improvement. Full roll, saturation and coupled-load convergence remain future experiments rather than claims manufactured by changing material parameters or weakening acceptance tests.

This iteration changes core controller mathematics only. User viewer performance/cleanup/display fixes and all input bindings/aim meanings remain. Native GUI/human flight-feel was not rerun in Cloud. Under standing user authorization, automatically verifiable bounded core changes can be merged after their physical review and CI; changes requiring human tests or material scope decisions are reported first.

## Causal sensitivity isolation and midpoint aerodynamic work

Starting from the merged PI-headroom implementation, the strong-roll material experiment was decomposed without changing the aircraft, gains, damage event, 120 Hz production step or original 10% hypothesis. Each case records normalized ownership requests at 120 Hz. Offline replays submit these requests as all-axis `PilotIntent` through the same actual trim fade, range/hinge effort, lag and slew. Replaying at 120 Hz reproduces both native trajectories with zero reported position error; higher rates hold each request for its original 1/120 s interval.

| 120/240 Hz effect-vector disagreement | Before midpoint aero | After midpoint aero |
|---|---:|---:|
| Native closed loop, each case responds to its own flight | 0.166858 m | 0.127965 m |
| Each case replays its own recorded 120 Hz requests | 0.010796 m | 0.009193 m |
| Both cases replay the recorded clean requests | 0.001601 m | 0.001902 m |

At 480 Hz, shifting material removal by ±1/120 and ±1/240 s around the aligned 1 s event changes the effect vector by at most 0.001712 m before / 0.001683 m after. The key press/release boundaries remain exactly 1 and 2 s. This excludes a simple event-boundary offset as the dominant cause here. Replay demonstrates the importance of closed-loop feedback and its sampling; it does **not** uniquely distinguish controller discretization from feedback amplification of plant/actuator integration error or prove a general resolution bound. A useful next discriminator is a held sensor-history controller bench spanning the roll release and saturation crossings, before any gain retuning.

An independent passive bench found a real integration defect: every cell's lift is perpendicular to its own local flow and the instantaneous total lift power is zero, but holding that old-state wrench over a midpoint rigid-body step evaluates work against a different velocity. With no motor/prop force, body drag, profile drag or aerodynamic pitching coefficient, half a second added 3328.42 / 1663.92 / 831.89 J at 120/240/480 Hz. The first-order scaling is numerical lift work, not engine power or structural damage.

The flight step now solves state-dependent distributed airframe loads together with midpoint body velocity, rates and matching Cayley attitude. Blade-element flight uses the existing seven-variable rotor/body solve, supplying each aero evaluation the same trial blade-induced velocity. Legacy comparison flight and independently owned detached cells use a six-variable body solve. Trial evaluations do not mutate cell diagnostics, controls or structure. Only the converged midpoint writes force/flow diagnostics; structural load assembly uses those forces and midpoint rates/spin, then advances the reduced graph once. New components still inherit the end-of-tick rigid state and start integration on the next tick. All changes remain force/moment integration; there is no energy projection, pose correction, damage switch or new production control mode.

At this stage, controls/governor advanced once per fixed tick and their endpoint positions were held through the solve (surface sampling is corrected below); elastic frames are held until the subsequent graph solve. `load_factor` now contains the previous step's midpoint aerodynamic sample, so its feedback latency is half a tick rather than the earlier start-of-step sample. It still excludes propeller force. Propeller atmosphere/motor altitude use their existing per-tick sample. This is a rigid-body aerodynamic integration repair, not fully coupled aeroelastic/actuator/wake energetics or a universal second-order flight claim.

`make test` includes persistent no-power/no-drag distributed-lift checks with initial body rotation on both production propulsion paths and on an actually separated wing. Across 120/240/480 Hz, worst half-second energy errors are ~1.49e−8 J for the parent and ~8.38e−9 J for the piece (1 mJ bound); adding positive cell drag dissipates at least 139 kJ. The original lift bench now errors by +7.45e−9 / −2.24e−8 / 0 J. Existing rotor free-body energy/angular momentum, mass-transfer, ownership, impossible-trim and load-driven failure checks also pass.

The unchanged fixed-trim material experiment gives 1.126545 / 1.126554 m effects and only 9.46e−6 m vector disagreement at 120/240 Hz. The native strong-roll effects are 0.240690 / 0.329692 / 0.380051 m at 120/240/480 Hz: the 120/240 difference of 0.127965 m still exceeds 10% of the 120 Hz effect. That hypothesis remains **false**. No thresholds or damage parameters were relaxed. Reduced-graph weakened flight still survives low load, begins failure at ~4.1667 s and separates at ~4.1833 s; its piece is ~59.65 m from the same-birth ballistic comparison at 10 s.

`python3 tools/control_experiments.py` now reproduces the fixed-request/event-time/lift ablations alongside the existing 120–960 Hz controller grid; `control-summary.json`, `control-sensitivity.csv` and `sensitivity-isolation.txt` are generated under ignored `results/` and retained by CI. User viewer changes, controls and original spec are preserved. Native GUI/human handling is not validated in Cloud.

## Fixed sensor histories, digital control timing and surface force samples

The follow-up keeps the original full-roll 10% hypothesis and all controller gains. A recorder captures the native 120/960 Hz clean/material cases, including the body seen by guidance, actually available normal-load sample, density, trim/throttle/prop requests, emitted axis requests, PI bias, ownership, actual surface endpoints and positions used for force. The known aim and keyboard schedule are reconstructed at exact source ticks; this is a targeted experiment, not a general pilot-input replay format. Histories live under ignored `results/`, use round-trip floating precision, and can be replayed unchanged after a core change. The source case must retain its available control topology; this bench does not silently replace control damage or separation history.

Controller-only instances consume recorded sensors without advancing a physical body. Sensor reads, request emission and ownership are aligned at exact left boundaries; requests are held over the declared output interval. Stored PI and mechanical endpoints compare at common 1/120 s **right** boundaries. Separately, prescribed-surface flights hold recorded applied positions using an offline mechanical-jam fixture; only this ablation removes the controller/actuator feedback. Their body/rotor/aero/structure still evolve through `Simulation::step`, with no pose writes. Ordinary and clock-locked flights continue through real control chains.

Frozen pre-change histories gave these discriminators (clean/material cases):

| Intervention | Fixed quantities | Observation |
|---|---|---|
| PI/ownership update 120–960 Hz | Common 120 Hz held sensors and 120 Hz request-output clock | PI right-boundary differences <8e−15; aligned ownership differences <3e−15 and no pause mismatches. Emitted 120 Hz requests nevertheless differ by ~0.01198 / 0.01180 normalized-unit·s in integrated max-channel difference |
| Sensor hold 120 vs 960 Hz | Controller and output at 960 Hz | Maximum bias difference ~0.00576 / 0.00459; request difference ~0.01955 / 0.01682 normalized-unit·s. One pause mismatch in the material case |
| Request hold 120 vs 960 Hz | Same recorded requests, pressure held at 120 Hz, actuator update 960 Hz | Max endpoint displacement difference ~0.01095 rad; this is input sample/hold, not a changed actuator law |
| Actuator update 120–960 Hz | Identical 120 Hz held requests and pressure | Common mechanical endpoints agree within 1e−15 rad; the values applied throughout the intervals differ |
| Prescribed applied-surface flight, 120 vs 240 Hz | Same original 120 Hz applied-position history | Material-effect vector difference 0.0002480 m; native 120 Hz replay identity is zero. Remaining variation includes rotor/structure/plant integration, not a controller rerun |

The production digital PI updates bias and then emits a request using that updated bias. Changing its update period changes the emitted waveform even when common-time stored integrals agree. The experiment found no extra PI-state accumulation defect in these histories. Calling every request difference an integrator bug, changing gains, or automatically changing output phase would exceed this evidence. The stage interventions interact; the numbers are not additive error contributions.

A distinct mechanical timing inconsistency is established: a rate-limited surface advances to its **right-end** state, then that future position was used for the whole interval while body loads were evaluated at the midpoint. For a held target and zero linkage lag, the exact ramp/arrival position-time integral is known independently. On the same frozen inputs, the old 120 Hz endpoint rule has summed local max-channel integral errors 0.006381 / 0.006398 rad·s. Midpoint sampling gives 0.001432 / 0.001469 rad·s. These units describe surface-position integration, not aerodynamic force or torque impulse.

`update_controls` now computes the existing full-step endpoint unchanged, plus a separate half-step sample starting from the **same** old linkage state and using the same held command/pressure. `ControlChain.midpoint_deflection` supplies the parent and detached-cell force solves; `deflection` and `filtered` remain actual full-step states. A half-step clone followed by a full step from the original state preserves the reduced lag+slew endpoint approximation; it does not advance the actuator twice. Trim fade, range, pressure effort, friction, hinge, jam, disconnect, direct-axis arbitration, handoff and PI mathematics retain their previous laws. Instantaneous `evaluate`/trim reads current endpoint positions; only interval integration uses the midpoint sample. No new live control mode or authority bypass is exposed.

`surface-sampling-tests` covers 66 analytic/production paths across both propulsion models, 60–480 Hz, constant rate, arrival before midpoint, lag, friction, partial range/hinge, pressure effort, jam, disconnect, ordinary evaluation and a genuinely separated wing. Endpoint error reports zero. On the frozen histories, native request and endpoint replay remains exact before/after. The separated-wing 0.5 s velocity errors relative to 480 Hz fall from 0.03446/0.01182 to 0.002668/0.000463 m/s at 120/240 Hz. This supports the local timing repair, not a universal convergence order. Existing passive lift, rotor conservation, mass transfer and progressive failure checks remain; weakened flight now starts failure at ~4.175 s and separates at ~4.1917 s.

The native strong-roll 120/240 Hz effects become 0.243726/0.323974 m, with disagreement **0.124183 m**, versus 0.127965 m before. The original 10% hypothesis remains **false**. Fixed trim and passive-lift results are unchanged. A separate closed-loop intervention fixes controller **and fresh sensor reads** at 120 Hz while refining plant/physical actuator/rotor/structure steps. Every control sample reads endpoint body/aerodynamics at the same physical instant, avoiding plant-dt-dependent load latency; its 120 Hz flight matches the explicitly refreshed native case exactly. Its 120/240 effect-vector disagreement falls from **0.036123 m before** the surface repair to **0.005537 m after**. This diagnostic changes sensor timing relative to the default previous-midpoint load feedback, so it cannot replace the original acceptance test.

Interpretation remains open for separate review: the original all-rates grid changes digital controller/sample clocks as well as physical integration. It measures full-system rate sensitivity, not just convergence of one fixed sampled control system. No acceptance flag, threshold or requirement is changed here. The next bounded discriminator is to lock the **original** load-sensor age as well as the 120 Hz controller clock during plant refinement, with explicit treatment of the 1 s mass-change boundary, before proposing a different convergence interpretation or controller output phase.

`python3 tools/sensor_history_experiments.py` records/replays current histories and writes `sensor-history-summary.json`, stage/release diagnostics and source histories; CI retains them in `control-evidence`. For a repeatable before/after run using this same probe against the earlier core, from the repository root:

```sh
mkdir -p results
sensor_before_dir=$(mktemp -d /tmp/flight-sensor-before.XXXXXX)
git archive c702fc7 include src | tar -x -C "$sensor_before_dir"
g++ -I"$sensor_before_dir/include" -std=c++20 -O2 "$sensor_before_dir/src/core.cpp" "$sensor_before_dir/src/propeller.cpp" tests/sensor_history_probe.cpp -o "$sensor_before_dir/probe"
"$sensor_before_dir/probe" data/validation-material.airframe results/sensor-baseline record
"$sensor_before_dir/probe" data/validation-material.airframe results/sensor-baseline replay > results/sensor-baseline-replay.txt
python3 tools/sensor_history_experiments.py --history-prefix results/sensor-baseline --reuse-history --before-replay results/sensor-baseline-replay.txt --output-prefix results/sensor-fixed-comparison
```

The probe can compile against either core; it observes the force sample actually supplied by that version. The synthetic fixture is unchanged between these versions. Neither the mechanical-jam stage boundary nor sensor assignment to an isolated controller instance is a production flight or damaged-control substitute. Original spec, user viewer code, local skills and fixed 120 Hz live integration remain intact. Native GUI/human handling was not validated.

## Original load-sample age under physical refinement

The next diagnostic holds the original 120 Hz control/request clock and the original raw aerodynamic-force age: at control time `t`, use body-normal `F_aero,z(t−1/240 s)` and compute `G=−F_sample/(M_current g)`. It deliberately preserves the material-event behavior: `remove_material` re-expresses cached moments about the new CG, leaves the cached force unchanged, and recomputes G using the new current mass. Holding an old normalized G would change this system. No production force, control, sensor, damage, gain or acceptance code changed in this iteration.

The offline observer uses integer ticks of `1/(2*plant_hz)`. Applied physical midpoint forces have odd timestamps; fixed sample age is exactly `plant_hz/120` ticks for 120/240/480/960 Hz plants. At 120 Hz the requested force is an exact stored sample. Finer plants linearly reconstruct the value from already available bracketing midpoint samples; no off-tick force evaluation, future control-time sample or physical pose assignment is used. Startup extends the existing trim cache as a constant prehistory. This reconstructs the reduced applied-force signal, not a newly calibrated accelerometer model.

An independently generated default native 120 Hz history is the positive control. Both clean and material cases reproduce **every** control-step body position/velocity/rates/quaternion, G, emitted requests, PI bias, ownership, pause flags, actuator endpoints and force-position samples with zero reported error, including the mass event and final body state. Exact/ramp interpolation and missing-bracket checks run before the flights; timestamp traces show all reads are causal, with displayed age error ≤6.6e−16 s from floating-point output.

At 1 s the material event changes mass from 2800 to 2792 kg. The cached force change and cached-G recomputation error report zero at every plant rate. In the default 120 Hz case the force time is 0.9958333 s, `Fz≈−27332.1509 N`, and the observed G is ≈0.9982463 using **2792 kg**. The fixed-age observer and default cache agree exactly. This verifies the event normalization rather than refreshing a more recent force to obtain agreement.

| Physical Hz; controller fixed at 120 Hz | Material-effect norm at 5 s | Adjacent effect-vector difference |
|---|---:|---:|
| 120 | 0.243725671 m | — |
| 240 | 0.243001390 m | 0.005601593 m |
| 480 | 0.244036585 m | 0.003883536 m |
| 960 | 0.243044008 m | 0.001885530 m |

The 120/240 controlled difference is ~2.30% of the 120 Hz effect; 120/960 differs by ~0.008907 m. These are **diagnostic measurements**, not a replacement acceptance test or a universal convergence-order claim. The original native all-rates grid still changes controller update/read/output periods and cached-force age together. Its 120/240 disagreement remains ~0.124183 m and the original 10% hypothesis remains **false**. That code and threshold are untouched.

No new production implementation defect is demonstrated. The same physical integration/actuation kernel, with the default 120 Hz chain preserved, refines much more closely when its sampled feedback contract is held fixed. This is evidence that the original discrepancy includes changing the sampled control system, not evidence to keep patching physics or retuning gains. If the intended target is the continuous-controller limit of the entire all-rates system, that original failure still matters; accepting a fixed digital 120 Hz controller instead would require a separately reviewed interpretation. This experiment does not choose between those targets.

The next bounded mechanism test can fix the physical plant at 960 Hz and the force age at 1/240 s, then compare: (1) internal controller/sensor/output all 120 Hz; (2) internal updates 960 Hz with sensor/output still 120 Hz; (3) sensor reads also 960 Hz with output still 120 Hz; (4) output also 960 Hz. Use live physical feedback and the same mass event to distinguish internal update/emission phase, observation cadence and output hold before any controller change. Treat the interventions as conditional comparisons, not additive error percentages.

`python3 tools/load_sample_experiments.py` builds the existing probe and runs its independent `age` mode. It writes `load-sample-summary.json`, case data and a control-time/force-time/bracket/mass/G trace under ignored `results/`; CI retains them in `control-evidence`. `python3 tools/control_experiments.py` continues to reproduce the separate original grid. No GUI or historical-aircraft fidelity claim follows from this one synthetic 5 s maneuver.

## Final clock decomposition and the remaining semantics choice

This completes the clock investigation. Production core, gains, damage/control requirements and the original acceptance script/10% threshold are unchanged. All experimental arms use the **same 960 Hz physical kernel**, real physical control chains, rotor and structural updates. The controller consumes a separate sensor packet and emits normalized requests through the existing physical input boundary; it does not write a physical pose or prescribe a surface position.

At a sensor event, capture current body/density and raw body-normal aerodynamic force at `sensor_time−declared_age`. Hold that packet. Each internal controller event reads it and normalizes raw force with **current** mass. An output event latches the latest computed request, which the physical actuators consume until the next output event. Events at one physical tick are ordered material/key event → sensor read → controller update → output latch → physical step. Rates are exact integer divisors of 960 Hz, and the keyboard/mass transitions align with every clock. The known initial trim force is explicitly recorded at t=0 and extended as constant prehistory; this supplies the newly exercised zero-time sample without extrapolating an unknown bracket.

| Arm | Internal update / sensor read / output (Hz) | Force age at sensor read | 5 s material-effect norm | Conditional vector shift from preceding arm |
|---|---|---:|---:|---:|
| A | 120 / 120 / 120 | 4.1667 ms | 0.243044 m | — |
| B | 960 / 120 / 120 | 4.1667 ms | 0.219375 m | A→B: 0.052270 m |
| C | 960 / 960 / 120 | 4.1667 ms | 0.223184 m | B→C: 0.023771 m |
| D | 960 / 960 / 960 | 4.1667 ms | 0.405566 m | C→D: 0.254552 m |
| E | 960 / 960 / 960 | 0.52083 ms (native 960) | 0.405052 m | D→E: 0.000574 m |
| F, separate pair cross-check | 240 / 240 / 240 | 2.0833 ms (native 240) | 0.323385 m | A→F: 0.121806 m |

The path differences are conditional interventions in a nonlinear feedback system; their norms are **not** additive percentages or a unique global ranking of factors. Output hold has the largest shift along this declared path, with internal/sensor rates already at 960 Hz. Faster internal updates with slower sensors repeatedly consume a held packet: B reaches 7.2917 ms packet age and 11.4583 ms raw-force age at internal updates, even though force age **at sensor read** remains 4.1667 ms. This is explicit sample/hold behavior, not a mislabeled fresh sensor.

Both endpoint controls pass. A exactly matches the previous fixed-age 960 Hz physical run in final position/velocity/rates/quaternion and in the full 120 Hz raw-force/G trace. E reproduces **every** default native 960 Hz physical/control step with zero reported position, velocity, rates, attitude, G, request, PI, ownership, pause and actuator errors, including material removal. Timestamp/latch counts, causal force brackets and current-mass normalization also pass.

Early traces identify the mechanisms before changed physical requests feed back:

- A/B begin with the same pitch rate error, ~0.00554604. The algorithm updates bias by `Ki*error*h` and then emits `Kp*error+bias_updated`. The unpaused pitch increment is 5.54604e−5 at 120 Hz and 6.93255e−6 at 960 Hz, ratio 8, immediately changing the request by 4.85278e−5. This is the existing discrete update/output convention with unchanged gains, not evidence of a new stored-integral defect.
- B/C apply identical requests for the first 1/120 s and have identical current error at that next common output. Their prior integral histories differ because B held the old observation while C sampled between output ticks. The pitch request then differs by ~6.43219e−7.
- C/D compute the same pitch request at 1/960 s, ~0.00776310. C still applies the previous ~0.00777139 request while D immediately latches the new one. Thus output cadence changes the actual physical input with the same computation and initial physical state.
- D/E initially share the same applied input; at 1/960 s their force timestamps are −0.003125 s versus +0.00052083 s. The different known old/applied force produces G≈0.988916 versus 0.988533. The force-age intervention is distinct from controller and output clocks.

The direct original-pair cross-check is decisive about the comparison's meaning. Default native 120/240 Hz flights disagree in the material-effect vector by **0.124183 m**. With physical integration/actuation held at 960 Hz, the corresponding 120/240 Hz sampled control contracts still disagree by **0.121806 m**. Physical refinement within each fixed contract shifts its vector only ~0.008907 m (120 Hz contract) and ~0.002336 m (240 Hz contract). E's exact native closure confirms that this decomposition reaches the actual all-rate high-frequency endpoint rather than a different simulated controller.

**Interpretation:** the original grid changes the sampled feedback system as well as the physical discretization. It is not a pure convergence study of one fixed sampled controller. No new production implementation defect is demonstrated, so this sequence ends without another physics/control patch. The original full-system 10% hypothesis remains **false**. If the intent is the continuous-controller limit of the whole all-rates system, that failure remains a legitimate unmet goal; the experiment does not dismiss it or adopt a replacement pass criterion.

**User review when awake:** decide what the 10% expectation is intended to constrain. One option is numerical convergence of the actual 120 Hz digital Instructor, with its update/read/output/force-age contract held fixed; another is an additional requirement for the entire controller/sensor/output system to be insensitive to rate changes or approximate a declared continuous law. The latter needs explicit PI state/output and measurement timing semantics before any algorithm change. The original §5 fixes simulation at 120 Hz; §46 gives Instructor 120 Hz as a future frequency example subject to measurement, but does not settle this benchmark interpretation. Recommendation: make the existing production timing explicit and retain the native all-rates result as separate rate-sensitivity evidence; **any acceptance reclassification requires user review and is not made here**.

`python3 tools/clock_experiments.py` runs the completed six-arm diagnostic and native baselines, producing `clock-summary.json`, raw cases, applied-request/timestamp/state traces and the fixed-age reference trace under ignored `results/`. CI retains them as `clock-evidence`. This one synthetic maneuver does not establish historical or universal closed-loop fidelity. No native GUI/human handling was validated.

## Aerodynamic validity: first finite-span counterexample

The user clarified that aerodynamic fidelity itself is a continuing development priority. PR #8 merged as `95ee703`; its original all-rate 10% hypothesis remains false and its unresolved interpretation is unchanged. The next investigation concerns physical-model validity, independently of midpoint work conservation, cell subdivision consistency and clock convergence.

`python3 tools/finite_span_experiments.py` compiles a small static probe of the **actual** C++ cell evaluator. Isolate the 128 wing cells from the loaded material fixture, disable propulsion/body/tail loads and control/incidence/elastic offsets, hold 70 m/s at 1,000 m with zero rates, and sweep 0°, 5°, 10°. Change span and inverse chord together at fixed area; also compare coincident half-area subdivision and two equal-area damage footprints. No time stepping, trim or Instructor is involved. The text fixture's rounded cell areas sum to 15.000064 m²; comparisons preserve that loaded area rather than rewriting it to 15.

| 10° test | Measured result |
|---|---|
| AR ≈ 3.333 / 6.667 / 13.333, equal area | All yield CL ≈ 0.927440, CD ≈ 0.073658; lift ≈ 37,118.26 N, drag ≈ 2,947.96 N |
| Symmetric outer-quarter removal versus 25% fractional loss over all cells | Both retain ≈ 11.250048 m² and yield lift ≈ 27,838.70 N, drag ≈ 2,210.97 N; differences < 1.2e−10 N |
| Surviving inner cells after tip loss | Normalized local load unchanged from intact wing |
| 120 versus 240 Hz static calls; 128 versus 256 coincident cells | Identical cadence result; subdivision CL/CD differences < 1e−12 |

The 5° sweep also lacks geometry/damage-location response. These are **failed physical-capability hypotheses**, reported as false in JSON; CI only checks executable measurement/isolation consistency, and a green job does not make these hypotheses pass. Damage is an aerodynamic area event with zero added roughness, not material removal or structural separation. Distributed fractional loss is a numerical footprint comparator, not a resolved perforation-flow model. The static isolation deliberately excludes real elastic, rate and control feedback; it does not imply all damaged flights produce the same moments or trajectories.

For an attached planar lifting wing, the established ideal elliptic-loading reference is `CDi_min = CL²/(π AR)`, with induced flow also changing effective incidence. See NASA Glenn's [induced drag](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/induced-drag-coefficient/) and [downwash](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/downwash-effects-on-lift/) explanations. This is a low-subsonic inviscid benchmark, not a calibrated rectangle, damage or stall model. At 10° and AR≈3.333 the reference gives CDi≈0.082138, already above the measured **total** CD≈0.073658. If all measured CD−CD0 were interpreted as induced drag, its implied e would be ≈1.425 / 0.712 / 0.356 across the three geometries. The conditional interpretation is explicit: the fixture has no documented profile/induced decomposition, and the fixed `.07*cl²` term cannot represent geometry-dependent induced loss across those geometries. Existing Mach lift scaling is included in measured CL; no claim of calibrated compressibility follows.

**Finding:** finite-span aerodynamic coupling is absent, not merely untested. The production shared `cell_load` used by parent and detached components sums independent polars; changing cell span does not change its force law. Numerical energy/covariance checks can pass while this physical capability is missing. Static post-stall curves, geometric area loss and real control-chain damage still exist; neither this probe nor previous low-speed recovery validates spin behavior or historical handling.

**Priorities and next bounded experiment:**

1. Finite-wing induced flow and damaged load redistribution first, because the same demonstrated gap affects clean performance and damaged controllability. Before changing production, explicitly separate two-dimensional section/profile tables from whole-wing empirical increments. Recommended next experiment: a diagnostic attached-flow lifting-line solution on the same span stations with declared section slope/profile drag, starting with an elliptic analytic reference, rectangle refinement, and partial tip loss. Check downwash/load redistribution, induced power and cell-resolution behavior; then test real control-chain and structural feedback in a bounded flight. This is a candidate model, not adopted production behavior.
2. Local stall/post-stall validity next: independently sourced section data, AoA/rate/sideslip sweeps and asymmetric recovery, keeping static versus dynamic-stall claims separate. Classical attached-flow lifting line is not a solution for deep stall/spins; no universal stall extension has been chosen.
3. Propeller-airframe effects and calibration follow: preserve the existing signed blade loads and coupled rotor accounting, then test measured blade/engine data and wake effects. Conservation alone does not validate propwash or historical performance.

Adding an induced-drag term to today's unsplit CD tables would risk double counting; replacing them silently would change trim, performance and damage interpretation. **No polar semantics, production force code, gains or acceptance thresholds change in this diagnostic.** The proposed section-table/finite-span contract must be surfaced before adopting it; the present work does not settle it. CSV/JSON are retained by CI as `finite-span-evidence`. No native GUI test or new historical-aircraft claim is made.

## Independent attached-flow lifting-line reference

### What the existing tables actually represent

Inspection of `tools/make_fixture.py`, its first implementation in `601851d`, the three loaded airframes and the production lookup establishes **synthetic operational cell polars**, not measured section data or calibrated whole-wing data. All three wing tables are identical. At the generated knots up to ±15°, `cl=5.2*alpha`, `cd=.016+.07*cl²`, `cm=0`; post-stall values and drag rise are prescribed synthetic functions. There is no airfoil identity, Reynolds number, experimental provenance, stated reference aspect ratio, or profile/induced split. Plausible-looking slopes and quadratic drag do not establish those meanings. In particular, `.07*cl²` is neither proven pure induced drag nor proven pure viscous profile drag. "Unsplit" describes that absence of a contract, not a discovered physical decomposition.

The loader also matters: production linearly interpolates **each coefficient in alpha**, then applies its separate bounded Mach multiplier to lift. At 2° the loaded/interpolated wing CD is **0.0217658**, versus **0.0183063** from evaluating the continuous generator formula between knots. The reference records both, and uses the loaded interpolation for its explicitly hypothetical profile interpretation. This difference is reported without changing the existing fixture or its lookup. The separate synthetic propeller polar is not used by this experiment.

### Reference model and falsification

`python3 tools/lifting_line_reference.py` is a standalone standard-library diagnostic; production never imports or calls it. Its declared model is steady, incompressible, small-angle, attached flow on straight planar connected wings, with an **explicit ideal section slope a0=2π/rad**, zero section zero-lift angle and no inferred profile drag. Speed is 70 m/s, density is the existing 1,000 m exponential value, and the free-stream angle is 2°. These are independently declared inputs, not a relabeling of the current polar.

The physical basis is MIT Unified Fluids [lecture 6](https://web.mit.edu/16.unified/www/SPRING/fluids/Spring2008/LectureNotes/f06.pdf) (distributed wake induction), [lecture 7](https://web.mit.edu/16.unified/www/SPRING/fluids/Spring2008/LectureNotes/f07.pdf) (elliptic load and separate profile/induced contributions), [lecture 8](https://web.mit.edu/16.unified/www/SPRING/fluids/Spring2008/LectureNotes/f08.pdf) (section closure) and [lecture 9](https://web.mit.edu/16.unified/www/SPRING/fluids/Spring2008/LectureNotes/f09.pdf) (general Fourier loading). The implementation uses `y=yc+b*cos(theta)/2`, `Gamma=2*b*V*Σ An*sin(n*theta)`, induced angle `Σ n*An*sin(n*theta)/sin(theta)`, and the section relation `2*Gamma/(V*c)=a0*(alpha−alpha_i)`. A small dense collocation solve supplies the coefficients. Lift/induced drag are `CL=π*AR*A1`, `CDi=π*AR*Σ n*An²`; no empirical induced term is added.

Checks and their limits:

- Untwisted elliptic planforms at AR=6/10/12 and 16/32/64 modes reproduce analytic CL, uniform downwash and elliptic induced drag; maximum absolute CL error is **2.8e−17**.
- A manufactured, smoothly twisted constant-chord wing has known first and second Fourier coefficients. Recovering both checks a non-elliptic loading mode and the roll normalization; it is an analytic code check, not a wind-tunnel validation.
- For actual untwisted rectangles and tip cuts, 64→96 modes changes CL by <1.6e−7 relative and CDi by <9e−7 relative. Independent span integrals close lift and induced drag to <3.3e−16 relative, and roll to <1e−12 N m. Induced power is positive and span efficiency ≤1.
- Integrated refinement is **not uniform local accuracy**: the intact rectangle's maximum off-collocation section-cl residual decreases from 0.0262 at 16 modes to 0.000811 at 96, concentrated near the sharp tip. At the centerline, 64→96 changes lift/span by <0.00006 N/m. No high-resolution tip-flow or viscous result is claimed.

### Low-angle geometry and damage findings

Rectangles use the fixture's geometric 10 m span and 1.5 m chord (15 m²); these geometric reference areas are distinct from its rounded sum of cell areas, 15.000064 m². Symmetric tip cutting removes the outer 1.25 m on each side. The equal-area comparator reduces chord to 1.125 m while retaining full span: it is a **span-preserving chord-reduction surrogate**, not a prediction for perforations or the earlier fractional-cell damage command.

| Independent geometry | Lift at the same 2° (N) | Centerline load (N/m) | Centerline downwash (m/s) | Alpha for intact lift | Induced drag at that matched lift (N) |
|---|---:|---:|---:|---:|---:|
| Intact, b=10 m | 6493.26 | 738.47 | 0.38779 | 2.000° | 53.06 |
| Symmetric tip cut, b=7.5 m | 4520.24 | 696.71 | 0.50406 | 2.873° | 92.85 |
| Equal-area chord reduction, b=10 m | 5175.59 | 579.05 | 0.29428 | 2.509° | 54.12 |

Thus induced coupling changes surviving local load and distinguishes equal-area geometries. At fixed angle the tip-cut induced drag actually falls from 53.06 to **45.00 N**, because lift falls; its drag penalty appears in the properly **matched-lift** comparison. Do not conflate these interventions. A one-sided left/right tip cut gives opposite FRD roll moments **∓3436.54 N m**, with equal lift ≈5498.47 N. This is static aerodynamic load asymmetry, not a demonstration that the real damaged pilot/control chain can trim it. Existing 6DoF, real controls and structural feedback were not exercised or weakened by this independent model.

### Supported next boundary and unresolved material choice

The experiment supports an **independent attached-flow strip-coupling reference** for contiguous wings and simple geometric tip loss. It does not support dropping this solver into the live core unchanged: no arbitrary missing patches, disconnected lifting surfaces, sweep, fuselage/tail interference, propwash, elastic iteration, transient wake, stall/spin, 120 Hz cost or damaged-control flight has been validated. In particular, a span-scale drag multiplier alone would miss the measured local load redistribution.

That next boundary now has an isolated experiment: `data/section-wing-reference.json` explicitly declares a new synthetic 2-D slope `2π/rad`, zero-lift angle 0°, profile `cd=.012+.01*cl²`, section cm=0 and quarter-chord aerodynamic center. None is inferred from old tables or claimed measured. Its consumers are the independent Python reference experiments. The transfer experiment integrates lifting-line circulation once per span strip, adds the declared profile drag once, and transfers equivalent wrenches to four (then eight) chordwise cells. Production does not load the JSON or call these scripts. This is a diagnostic boundary, **not** a live C++ flight adapter, structural feedback or an adopted production data contract.

The data choice remains material. For illustration only, treating old 5.2/rad as a **section** slope gives whole-wing CL≈**0.139954** at 2°, versus the old uniform incompressible value ≈**0.181514**. With that same hypothetical interpretation, using only .016 as profile plus computed induced drag gives total CD≈**0.016999**; using the **entire interpolated old CD** as profile plus induced gives ≈**0.021445**. The first discards an unexplained term; the second may double count it. Neither is licensed by provenance. Treating the old slope as a calibrated whole-wing result would instead require a separately specified inverse section calibration, for which no measurements exist. **Approval is required before adopting a production table meaning, migrating existing data, selecting a new default force law or changing acceptance interpretation.** No such choice is adopted here; ordinary independent reference experiments can continue without it.

### Localized section loss: wake feedback versus local force scaling

`python3 tools/local_section_coupling.py` asks one question: **does local scaling of an intact load remain a self-consistent solution after the local section law changes?** It reads only the explicit sectional fixture, holds its geometry/profile law/zero-lift angle and 2° incidence fixed, and reduces local section slope by the declared smooth factor `f(y)=1−0.5*exp(−((y+3)/0.6)²)`. This is a synthetic localized constitutive intervention centered at y=−3 m, **not** a perforation, lost area, structural damage, historical measurement or interpretation of legacy `lift_efficiency`. The Gaussian's far-wing tail is negligible, not a geometrically sharp damage boundary.

Three arms distinguish the mechanism. Intact solves the original section law. The frozen-load ablation sets `Gamma_f=f*Gamma_intact`, projects that circulation into Fourier modes, and reconstructs **its own** induced flow to test the new section closure; it does not let that flow feed back into the selected circulation. Recoupled solves circulation and induced flow together with `a0(y)=f(y)*a0`. Only the independent reference solver gains a scalar-or-station-function slope input. Scalar/callable equality and the existing analytic reference checks pass; production code and data are unchanged.

| 128-mode result at the same 2° | Total lift (N) | FRD roll (N m) | Induced drag (N) | Maximum section-cl closure residual in \|y\|≤4.5 m |
|---|---:|---:|---:|---:|
| Intact | 6493.255 | ≈0 | 53.058 | 1.17e−7 |
| Frozen local scaling, intentionally inconsistent ablation | 6134.994 | −1069.020 | 52.670 | 0.09465 |
| Recoupled sectional law/wake | 6147.548 | −867.198 | 48.587 | 1.05e−7 |

The ablation is **not a valid alternate flight model**. Its integral lift, roll and positive induced-power identities still pass (relative force/power errors ≤2.3e−16), demonstrating that conservation identities alone do not enforce the local constitutive relation. Its roughly 0.095 interior closure defect is falsifying evidence against local force scaling as a substitute for solving this declared coupled model.

Physical findings within the reference:

- The frozen and recoupled total lifts differ by only **12.55 N**, while their roll moments differ by **201.82 N m** (~19% of the frozen magnitude). Small total-lift disagreement does not imply similar asymmetric load/moment response.
- At y=−3 m, halving section slope gives recoupled lift/span **496.39 N/m**, versus intact **678.23** and frozen **339.11**; local induced flow becomes upwash ≈−0.320 m/s. That feedback partially restores the local load relative to simple scaling.
- At effectively untouched y=+1 m, recoupled load falls from **733.46 to 725.96 N/m** as induced downwash increases. Frozen scaling leaves it unchanged to <0.001 N/m. Redistribution need not mean every intact region gains load.
- Mirroring the intervention preserves lift and reverses roll within 1e−8 N/N m. Relative 96→128-mode changes are ≈2.1e−8 for lift, 2.0e−8 for roll and 1.0e−7 for induced drag. Interior closure residual falls from 0.00205 at 32 modes to 1.05e−7 at 128.
- The fixed off-grid probe still sees a **~0.00194** maximum closure residual very near the sharp tips. Integrated/interior convergence does not establish uniformly resolved tip flow. All probed effective angles remain inside the fixture's ±5° domain; no stall extension is made.

At fixed incidence the recoupled induced drag is lower because lift falls. Rescaling the linear reference to the intact **total lift** needs ≈2.11247° and induced drag ≈**54.205 N**, versus intact ≈53.058 N. This is an aerodynamic same-lift comparison, not trimmed flight or demonstrated remaining control authority. Power checks are steady `V*Di`/span-integral identities, not a time-integrated 6DoF energy ledger.

**Supported next step:** require both section/wake closure and conservative cell-wrench transfer before using an experimental coupled solution for feedback. The next informative bounded experiment is static remaining roll authority with a declared local control-incidence perturbation (ideally sourced from the actual damaged chain), while retaining this closure check; first establish that boundary independently of the live default. These results do not authorize mapping legacy damage coefficients to section slopes, deriving chordwise pressure, or changing production data/default semantics. That choice remains for user review with the impact examples above. CSV/JSON are retained in the existing `lifting-line-reference-evidence` artifact; no additional PR or solver framework is introduced.

### Static roll authority through actual actuator outputs

`python3 tools/roll_authority_reference.py` now compares the intact and locally reduced-section-slope models at **identical actual control outputs**. The same Gaussian section intervention from the preceding experiment is retained; mechanical chains are intact and identical in both arms. `build/section-controls-probe` calls the authoritative C++ `Simulation::update_controls` at 120 Hz on a held-body hardware bench, with direct roll owning its axis, no pitch/yaw ownership and Instructor off. It checks that actual aileron deflections are settled between 1 and 2 seconds. No body/aero/structure step is taken. Existing `.airframe` tables are loaded/validated but not used to compute aerodynamic forces; only their real control-chain configuration is used mechanically.

At 70 m/s and the reference density, q≈2668.14 Pa. Inputs −0.2/0/+0.2 produce actual left/right actuator positions −3.831985°/0/+3.831985°. The explicit sectional JSON declares a **new synthetic** control mapping: incidence gain 0.5 rad/rad, smooth Gaussian windows at y=±3 m with width 0.9 m, opposite aerodynamic signs on left/right. It is not inferred from legacy `control_gain` or calibrated as a historical aileron. Both aerodynamic arms consume the same sampled positions; no surface state is manufactured by the Python solver.

Separate the neutral bias from the increment caused by input. Moments below are FRD roll about the experimental wing's center datum, at fixed 2° base incidence:

| Same normalized direct input | Intact roll (N m) | Local slope loss, recoupled roll (N m) |
|---|---:|---:|
| −0.2 | −5077.30 | −5484.02 |
| 0 | ≈0 | −867.20 |
| +0.2 | +5077.30 | +3749.62 |

The finite-interval incremental gains are ≈**25,386.49** versus **23,084.10 N m/input**, so this damaged section model retains **90.93%** of the intact gain over the tested interval. Its positive-input increment from neutral is ≈4616.82 N m, plus a persistent −867.20 N m bias. The positive **total** moment ratio (~73.9%) is therefore not the incremental control-gain ratio. Likewise, the larger negative total magnitude does not imply stronger negative control effectiveness; the damage bias assists that direction.

A predicted bias-canceling input **+0.0375669** lies inside the measured interval. Resampling the real C++ chains at that input and solving again gives roll ≈2.9e−13 N m. Lift is still only ≈**6113.27 N**, versus intact ≈6493.26 N: this is static one-moment cancellation, not six-axis trim, achieved roll rate, stability, dynamic recovery or flight safety. Neither full-stick authority nor an aircraft-wide envelope is inferred from ±0.2 input.

Verification and falsifiers:

- Both physical arms keep section/wake closure (interior residual ≤1.32e−7) and stay within the unchanged ±5° effective section-angle domain (maximum ≈3.59°). Relative 96→128-mode roll changes at +0.2 are <1.5e−8. Sharp-tip local-resolution limitations remain as above.
- Strip quadrature agrees with spectral lift/roll/induced drag within the 1e−7 absolute check; four/eight chord-cell equivalent wrenches preserve force, moment about an offset CG and rigid-field power within 1e−8 (observed errors ≤4.1e−11). These are steady transfer identities, not a dynamic energy ledger or pressure/stress validation.
- With actual zero input, the prior frozen-versus-recoupled comparison reproduces **12.5537 N** lift difference and **201.8224 N m** roll difference. That earlier close-lift comparison was between two damaged-load treatments, not between intact and damaged aircraft. It remains a reason to measure moments and authority separately.
- The first attempted controlled frozen ablation at +0.2 reached effective angle **5.27977°**, outside the declared section domain. This failed domain check is preserved in JSON. It is excluded from authority estimation; only its neutral-point mechanism check is retained. No input bound, section limit or original acceptance threshold is relaxed.

This first comparison isolates aerodynamic section loss with identical intact hardware; the disconnect/jam follow-up below measures actual hardware faults at the same boundary. Neither establishes structural feedback, body motion or stall. Production defaults, legacy aerodynamic data and the original failed 10% hypothesis remain untouched; adoption or reinterpretation of production data is still a separate decision. Results are `roll-authority-summary.json`/`roll-authority.csv` in the existing evidence artifact and PR #10.

### Disconnect versus captured-angle jam

`python3 tools/control_fault_reference.py` crosses three real hardware states (normal, left link disconnected, left surface jammed) with clean versus the preceding local section-slope loss: **six static cases**, each at the same −0.2/0/+0.2 requests and declared sectional/control mapping. Only the independent probe/orchestration changes; production C++ mechanics, old aerodynamic data and failure thresholds do not.

Fault semantics come directly from existing `Simulation::disconnect` and `advance_surface`:

- Fault arms first hold +0.2 for one second. Both have actual pre-fault left angle **+3.831985°**. Immediately after either event the angle is unchanged; the probe never writes `deflection`.
- Disconnect calls the existing API, clearing `connected`; the existing actuator update then targets neutral. After settling, left angle is **0°** for every tested request. This is the current model's neutral-seeking behavior, not a validated free-floating hinge model.
- Jam sets the existing `jammed` flag and captures the actual pre-fault angle in `jam_position`; the authoritative update freezes it at **+3.831985°**, regardless of subsequent request. It does not teleport to an arbitrary specified angle.
- The right chain receives identical requests and has the same settled positions as the normal bench. Direct roll ownership remains 1, even when the left actuator cannot follow. Samples at one and two seconds after the event verify settlement; the full event/force transient is not simulated.

| Hardware | Section loss | Neutral roll bias (N m) | Incremental gain / clean intact | Static zero-roll input | Within tested interval |
|---|---|---:|---:|---:|---|
| Normal | No | ≈0 | 100% | ≈0 | Interior |
| Normal | Yes | −867.20 | 90.93% | +0.03757 | Interior |
| Left disconnected | No | ≈0 | 50.00% | ≈0 | Interior |
| Left disconnected | Yes | −867.20 | 50.26% | +0.06796 | Interior |
| Left jammed at captured angle | No | +2538.65 | 50.00% | −0.20000 | Negative boundary |
| Left jammed at captured angle | Yes | +1197.58 | 50.26% | −0.09385 | Interior |

**Finding:** disconnect and jam have the same incremental gain for a given section law because the same right actuator remains input-responsive, but their neutral biases and available command headroom differ. In clean aero their gains are ≈12,693.24 N m/input; with local section loss they are ≈12,760.18, versus the full clean value ≈25,386.49. The small single-right gain increase (~0.53%) is a result of this linear coupled reference, not recovery of the missing left input response or a general law for damaged aircraft. The reference's full two-actuator gain still decreases to the previously measured 90.93%.

Adding the local section loss partially opposes the positive jam bias, moving the zero-roll command inward. It **does not restore control gain**. A smaller neutral moment cannot stand in for healthier controls. Conversely, the clean jam case reaches zero roll at the edge of the **tested** interval: its near-zero negative command headroom is not a claim about the mechanical stop, full-stick authority or an aircraft-wide loss-of-control threshold.

All in-interval zero-roll commands are resampled through the actual faulted C++ bench and re-solved (|Mx|<1e−6 N m). Their lift differs substantially: clean jam cancellation gives ≈**8530.63 N** versus the clean intact ≈6493.26 N; section loss plus disconnect gives ≈**5803.23 N**. No result is called six-axis trim, dynamic recovery or structural survivability.

The 18 sampled physical states have maximum effective section angle ≈**3.66024°**, interior section/wake residual ≤**1.36e−7**, relative 96→128-mode roll changes <**1.5e−8**, and conservative strip/cell force/moment/rigid-power errors ≤**4.1e−11**. Existing ±5°, closure, transfer and original acceptance limits remain unchanged. No event energy impulse, time-integrated body dynamics, chordwise pressure or hinge free-floating physics is claimed. The default prior experiment still reproduces retained gain 0.90930636.

The held-rig transition follow-up below distinguishes transient from settled authority without a new production-model choice. Replacing neutral-seeking disconnect with aerodynamic free floating would still require an explicit hinge moment/inertia/damping model and a separately reviewed production semantic/data decision; these results do not choose it. CSV/JSON (`control-fault-summary.json`, `control-fault-curves.csv`) stay in the existing artifact and PR #10.

### Fault transitions: continuity and time sampling

`python3 tools/control_transition_reference.py` completes this bounded control-fault investigation. A new small C++ probe drives the **existing** control update on a held body with constant +0.2 direct-roll input for 0.25 s. At 120/240/480 Hz, compare normal hardware, disconnect and captured-angle jam at **1/60 s** (surface still moving) or **0.125 s** (surface settled). The normal trace is shared, giving 15 hardware traces. All events align with every tested grid: this tests the same physical input/event schedule, not arbitrary within-tick event quantization. Pressure is fixed and the loaded fixture has zero actuator lag/friction; no parameter is reset to make it fit the analytic comparison.

Before/after event rows have the **same timestamp**. The API/configuration changes neither physical deflection nor held-body position/velocity/rates/quaternion. Actual subsequent surface increments obey the existing slew bound. Internal `filtered` driver values can reset on the next update: for late disconnect at 120 Hz, the driver changes −4.4° while the physical angle changes only −0.725755°. For early/late jam the driver resets by −2.94849°/−0.568015°, while physical angle stays fixed. A driver/cache reset is not a surface-position jump.

Real endpoint and `midpoint_deflection` traces drive the **same declared steady linear sectional reference**, with clean and locally reduced section slope. The affine control response is obtained from the existing solver, checked against fresh solves at 15 distinct corner/event/first-step/end states per aero model (maximum coefficient difference <9e−19). Corner closure bounds are <1.34e−7, effective angles <3.63°, and representative wrench-transfer errors <3.3e−11. This uses linearity for efficient trace evaluation; it is no new unsteady-flight engine or pressure law.

Findings:

- Event-time physical angle and quasi-steady roll-torque jumps are **zero**. Endpoint angle differs from the continuous fixed-rate/target actuator solution by ≤2.8e−17 rad; common-time differences between the three grids are ≤1.8e−18 rad and ≤2.3e−13 N m in roll torque.
- Moving jam captures ≈**1.45151°**, versus ≈**3.83198°** after settling. Right hardware keeps moving toward the held target. An early clean disconnect can initially leave total roll torque unchanged because left/right rate contributions cancel; an early jam still permits torque to rise as the right surface moves. A fault does not always cause an immediate torque drop.
- Settled jam at the already-commanded position produces **no torque-trace change under this unchanged input**, despite the lost future input responsiveness measured in the previous static study. One present torque sample cannot establish healthy control authority.
- Late clean disconnect's first-step roll change is about **−480.80 / −240.40 / −120.20 N m** at 120/240/480 Hz. This finite rate-driven change scales with interval duration; it is not an event-time discontinuity.

Torque-time integration separates endpoint sampling from the actual control midpoint. For the late clean disconnect, compare the time integral over the same 0.25 s with the analytic piecewise-linear actuator trajectory and linear sectional response:

| Hz | Endpoint-sample integral error (N m s) | Actual-midpoint integral error (N m s) |
|---:|---:|---:|
| 120 | +10.17383 | +0.157063 |
| 240 | +5.16545 | +0.096962 |
| 480 | +2.63120 | +0.001803 |

All midpoint errors satisfy the declared piecewise-linear corner bound `(2*abs(K_left)*rate_left + abs(K_right)*rate_right)*h²/8`; event boundaries are common grid points. Settling-corner phase changes the observed error ratios, so the three measurements are not advertised as a universal 4× reduction per refinement. The endpoint rule shows the expected first-order bias. **No production defect is demonstrated or repaired here:** the existing production airframe and detached-force paths already consume midpoint surface samples. This control-only benchmark neither replaces the original all-rate 10% hypothesis (still false) nor proves full-system numerical/historical validity.

The force trace is quasi-steady on a held rig. Its torque-time integral is not body angular momentum, a dynamic flight-recovery result or an energy ledger. No actuator inertia/velocity-energy state or jam-reaction impulse is introduced; validating those phenomena would require a different model/data contract. Arbitrary substep events, varying pressure, nonzero actuator lag/friction, unsteady wakes and body/structural feedback are outside this experiment.

**Diagnostic merge assessment:** this group is a self-contained, reproducible **independent diagnostic increment**: it establishes and bounds its reference, keeps real control/fault semantics, records previous negative findings, and requires no human GUI/handling test for its stated claims. It contains no production force-law/default/data migration and is suitable for the user's authorized automatic integration after checks, **not** adoption of a new flight model or full v0.1 acceptance. PR #9's baseline has been merged as `62c2bc2`; PR #10 targets main and must preserve that evidence. Integration does not resolve the production-data/free-hinge choices. GitHub records the final merge and post-merge CI status.

No further patch is selected from these traces. A later body-dynamics or nonsteady-physics study is a separate bounded claim; free-floating hinges or a production section-data/default change need the explicit choices already described. The current group's raw hardware CSV, quasi-steady force CSV and summary JSON (`control-transition-*`) are retained in the same evidence artifact.

### Small decision proposal: opt-in sections, preserved legacy

**Physical change proposed:** for explicitly sectional experimental data only, solve coupled spanwise circulation/downwash and add separately declared profile drag. Geometry and tip loss then change surviving loads and matched-lift induced drag. Keep span-preserving chord reduction labeled a geometric comparator; it is **not skin perforation**. Arbitrary holes and stalled/transient flow remain outside this reference.

**Legacy preservation:** leave existing `.airframe` files, lookup, force/control/structure code, default viewer and acceptance scripts on their current path. The new JSON and Python diagnostics are opt-in and have no production call site. The old failed 10% hypothesis remains false. No old slope/drag term is reclassified and no semantic production change is merged.

**Acceptance evidence for the experimental boundary:**

| Test | Actual evidence and supported claim |
|---|---|
| Analytic flow/reference cases | Elliptic lift/downwash/induced drag, twisted-rectangle mode, positive induced power and refinement checks above pass |
| Strip quadrature versus whole-wing result | Across clean/symmetric/asymmetric tip cuts and chord surrogate, lift/induced drag close within 1e−9 relative; 16→32-point quadrature force changes <1e−9 N |
| Four versus eight chordwise cells | Equivalent summed force, moment about an offset CG, and power in a nonzero rigid velocity/rate field pass the <1e−8 absolute tolerance (local errors <2e−11 N, N m, W); no fourfold induced drag |
| Force-only deposition counterexample | Moving area-weighted forces from quarter-chord to cell centers creates **−2434.97 N m** pitch error and **+486.99 W** power discrepancy on the intact wing |
| Equivalent cell wrenches | Carry explicit couples as well as force, preserving the declared strip resultant and work; this fixes the transfer error without claiming a pressure distribution |
| Legacy flight/control/structure regressions | Existing full CI is observed separately; passing reference jobs alone does not establish legacy-suite completion or live physical validity |

At 2° the new intact fixture has lift ≈6493.26 N, induced drag ≈53.06 N and **declared synthetic profile drag ≈491.17 N** (total ≈544.23 N). Unlike the previous hypothetical split, that profile law is explicit new input. This is no historical performance measurement. The rigid-velocity power check tests equivalent wrench transfer, not a time-integrated energy ledger or a stability/handling test.

**Decision still needed before production adoption:** approve a new explicit section-data contract and its behavior outside the attached-flow domain, while retaining legacy semantics until a separate migration/default decision. Quarter-chord cm=0 plus area-weighted **equivalent wrenches** conserves total loads but does not determine chordwise pressure or local structural stress. A physical aerodynamic-center/section-moment/load-deposition model must be specified before using it to claim structural torsion fidelity. The next authorized independent test can exercise a deliberately new adapter/fixture with actual damaged chains and structural feedback; it must preserve those boundaries, and it is not implemented or accepted by these static results.

PR #9's original negative-capability evidence remains available at head `5a49d59`, with passing PR/push CI. This follow-up stays in PR #10 on `feature/lifting-line-reference`; no additional stacked PR or semantic production merge is needed. Outputs are `results/lifting-line-{summary.json,loads.csv}` and `results/section-transfer-{summary.json,strips.csv}`, retained by the focused CI job as `lifting-line-reference-evidence`. Numerical verification supports this declared linear reference and transfer, not historical airplane validity. Original production/acceptance code and the failed 10% hypothesis remain unchanged.

## Incremental build configuration and the libstdc++ ABI

The [PR #12 review finding](https://github.com/4everWZ/flight-damage-sim/pull/12#discussion_r4303470420) is confirmed on main `ecbd27e` with GCC 14.2.0. After a normal `make -j2`, `make CPPFLAGS='-Iinclude -D_GLIBCXX_DEBUG' build/finite-span-probe` compiled only the new consumer: neither shared core object was rebuilt. The probe returned 1 with `std::bad_alloc`. Cleaning and running the same debug build succeeded, producing all 36 data rows. Default CI exercised one configuration and did not establish incremental cross-configuration correctness. This was an omission in the shared-object optimization.

The [GCC debug-mode contract](https://gcc.gnu.org/onlinedocs/libstdc++/manual/debug_mode_using.html) explains why: the macro changes standard-container layouts, and container-bearing data crosses the core/consumer translation-unit boundary. A successful link does not establish ABI compatibility.

The correction compares effective `CXX`, `CPPFLAGS` and `CXXFLAGS` as shell-quoted, NUL-separated data. A missing or changed ignored settings record becomes a phony prerequisite for both core objects and requested executables. Otherwise the record is left untouched. Existing executable paths/default flags remain; both objects compile once per configuration, without a clean between supported setting changes. Python is not introduced into the normal C++ build.

`python3 tests/build_config_tests.py` builds real sources/fixtures in temporary trees. It checks new debug consumers after a base build, existing executables switching back, debug mode through `CXXFLAGS`, quoted `CPPFLAGS`, compiler-command changes, unchanged-build reuse, query mode and read-only dry-run. A small real `Definition` consumer reports its compiled mode/markers and reads container-bearing core data; the original finite-span probe also runs in each configuration. The regression uses `-O0 -g0` to bound cost; the standard optimized default is checked separately. No simulator source, aircraft data, force law or acceptance threshold changes.

With the original default `-O2 -g` settings, corrected debug and return-to-default probes both succeed; their CSV output equals the clean debug control. Existing `make -j2 test` passes, retaining the original false 10% hypothesis. The parallel `build-config` CI job retains `build-config-summary.json` with actual settings/observed modes. Local reproduction output remains in ignored `results/build-config-before.json`.

## Lagged damaged controls: within-tick reversal and aerodynamic impulse

After PR #13 merged, the next core experiment targets §17/31 and full-authority keyboard control with **simultaneous** linkage lag, friction and reduced range. Previous surface tests separately exercised unlimited-rate lag and zero-lag slew; they did not falsify the combined reversal case. The old endpoint approximation could miss a turn inside a tick: it advanced the filtered target to the endpoint, then clamped surface travel toward that endpoint. A still-positive filtered target can catch an initially rising surface before becoming negative, so the surface must first rise and then reverse inside the interval.

The held-pressure model is unchanged: the filter tends exponentially to `command*limit*range` with time constant `lag`; pressure multiplies that target by `effort=1/(1+q/q_limit)`; available speed is `R=slew*effort/(1+friction)`. The actual surface follows the target at no more than ±R. The correction finds the catch time between a constant-speed surface segment and the exponential, then advances the remaining interval from that physical catch. A monotone exponential has at most two catches (a reversal and subsequent smooth following). Half-step and full-step samples still start from the same old state; parent and detached cells consume the same mechanical midpoint interface. Zero-lag behavior, jam, existing neutral-seeking disconnect, trim fade, direct-axis ownership and force laws retain their meanings. This repairs integration of an existing actuator model, not a new hinge-moment/free-floating model or a production polar/data decision.

`build/lagged-control-tests data/validation-material.airframe [summary.json]` runs an independently specified piecewise analytic reversal: +1 direct roll for 0.2 s, then −1 to 0.8 s; left linkage lag 0.15 s, friction 4, range 0.6. Sweep both signs, 40/70/110 m/s and 120/240/480 Hz. Cross intact geometry/hinge with eight left outer-wing cells removed and hinge fraction 0.7 (36 cases). The **held-body force bench only** declares a linear synthetic CL=5.7α, constant CD and zero CM to isolate the actuator-to-cell wrench; it is not fitted data or a changed aircraft file. Its analytic angle has one turn, so its torque impulse has a known composite-midpoint corner bound `abs(K)*R*h²/4`. Instantaneous torque is checked separately from its time integral.

Compiling this same test against main `b3e0095` produces a genuine failing baseline: maximum endpoint/midpoint angle error ≈0.002299 rad and moment error ≈393.92 N m over the matrix. In the damaged 70 m/s, positive-first, 120 Hz case, the analytic incremental roll impulse is **42.1673 N m s**; the old evaluator gives **−16.3715**, while the correction gives **42.3474**. This near-canceling reversal makes the sign error visible; it is not a general percentage improvement or historical-aircraft prediction. Corrected endpoint/midpoint errors are <2.7e−16 rad and cell-moment error <5.5e−11 N m. The remaining midpoint torque-integral errors satisfy the unchanged derived corner bound; their matrix maximum is ≈0.6800 N m s, not zero. Further held-command interval-composition cases cover initial surface/filter disagreement, catch/reversal/following; smooth lag, disconnect, jam and zero-slew/changing-pressure checks also run.

Nine **actual 6DoF flights** use the unmodified production polar, BEM engine, structure and trimmed initial state: intact, linkage damage, or linkage plus the same cell loss, at 120/240/480 Hz. The 0.8 s keyboard reversal produces 120 Hz final roll rates ≈−0.86137/−0.21250/−0.42054 rad/s respectively. Extra roll magnitude with cell loss is an asymmetric aerodynamic bias, not restored linkage authority. Direct roll stays fully owned, actual midpoint angles feed production cells, and structure remains attached. The 120→480 Hz position differences are 3.54–4.30e−5 m, versus 0.919–1.285e−5 m for 240→480 Hz. These short measurements support this scenario only; they do not replace the original full-roll 10% hypothesis, which remains false, or establish stall/structural-failure envelopes.

`make -j2 test` runs the new regression and writes ignored `results/lagged-control-summary.json`, retained in the existing `control-evidence` artifact. Existing conservation, damaged trim, per-axis handoff, progressive high-load failure and original physics tests pass independently. For the before/after comparison, compile `src/core.cpp` from `b3e0095` with the current headers, `src/propeller.cpp` and `tests/lagged_control_tests.cpp`; the test writes a measured JSON summary and exits nonzero for the original defect. Preserve `results/lagged-control-before.json` separately from the fixed summary. Original specification, aircraft files, acceptance predicates and viewer bindings remain unchanged. No human test is needed to establish this numerical repair; desktop handling/focus and the broader subjective validation remain open.

The next useful coupling experiment is damaged linkage response during actual mouse/keyboard handoff or a sustained maneuver approaching available control/structural limits. Friction here remains a reduced rate penalty, pressure is held per actuator tick, and neither mechanical hinge energy nor a free-floating disconnected surface is modeled. Finite-span production adoption and the earlier sample-clock interpretation still require their separate decisions; this repair settles neither.

## How decisions change

Record a testable hypothesis, actual result, uncertainty and useful next experiment. Routine reversible implementation choices can proceed; any choice removing or weakening a core requirement must be reported first. Preserve failed evidence when it changes interpretation. Do not turn partial mechanisms, stubs or a rendered aircraft into a claim of full v0.1 completion.

No project distribution license has been selected.
