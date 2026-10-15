# Current decisions and evidence status

Updated: 2026-10-15. Core, Instructor and signed-propulsion iterations exist on a feature branch; this is not full v0.1 acceptance.

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

## How decisions change

Record a testable hypothesis, actual result, uncertainty and useful next experiment. Routine reversible implementation choices can proceed; any choice removing or weakening a core requirement must be reported first. Preserve failed evidence when it changes interpretation. Do not turn partial mechanisms, stubs or a rendered aircraft into a claim of full v0.1 completion.

No project distribution license has been selected.
