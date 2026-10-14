# Current decisions and evidence status

Updated: 2026-10-14. Two core iterations exist on a feature branch; this is not full v0.1 acceptance.

## Authority and provenance

The user requests core-first development and testing in this repository's Codex Cloud environment, followed by a simple existing visualization/control tool. C++ and Python are acceptable according to fit. Preserve the control, flight, and damage requirements.

The [original 51-section v0.1 specification](../spec/flight-sim-core-v0.1.original.md) is an unchanged historical assistant-authored design baseline. Its Unity/C# recommendation is historical. [User requirements](../sources/user-requirements.md) distinguish actual user intent from assistant recommendations and unverified descriptions of commercial games.

## Confirmed direction

1. Force/moment-driven 6DoF, 120 Hz fixed steps, independent rendering cadence, and distributed local aero cells.
2. Independent aerodynamic, structural, and control-system representations, with explicit cell-to-structural-node and cell-to-control-chain mappings.
3. Keyboard owns only its active axes at full requested authority, with smooth release to the Instructor. Both paths pass through the same real surface/linkage/hinge constraints.
4. Remaining section changes stiffness and stress capacity; loads redistribute; accumulated material strain under actual loads can exhaust ductility. Separation requires loss of graph connectivity. No HP, direct pose control, or scripted wing destruction.
5. Start with one synthetic parameterized propeller validation airframe. Historical aircraft, more airframes, ballistics, and a polished combat game follow core validation.
6. Keep the original specification unchanged and track partial completion in the current plan. Draft PRs only; no automatic merging.

## Implementation choice and evidence

Use the owned C++20 core in `include/flight/` and `src/core.cpp`. GNU make/GCC were present in Cloud; no CMake, JSBSim, or FlightGear installation was required. Python's standard library drives the same compiled core via scenario CLI and a tick-based stdin/stdout protocol. Python contains no alternate flight model.

The choice now has practical fit evidence: the same core trims, integrates translational/rotational motion, evaluates 152 local cells, arbitrates mixed input, transmits damaged surface state, redistributes structural load, and updates mass/CG/full inertia at separation. `integrate_body(Body, MassProperties, Wrench, dt)` is an explicit replaceable mechanics boundary. This does not establish superiority over JSBSim; no JSBSim adapter/comparison has been measured. Retain it as an optional future reference as described in [engine research](../research/engine-options-research.md).

Matplotlib was available and provides a first engineering viewer with live aim, keyboard, damage injection, cell-force arrows and structural utilization. Its input/state/render transport passed an Agg smoke test. Cloud has no GUI display, so interactive human handling quality remains unverified. This viewer is not the original specification's finished third-person sky/terrain playable prototype; FlightGear remains a possible later external viewer.

APEX's two skills are copied unchanged into `.agents/skills`, with the upstream MIT license. The inspected upstream supplies a skill layout and validation script but no installer. This is a repository-local installation, not a global plugin/hook or security-policy change. Its official PowerShell package validator was not available; copy identity and local links were checked structurally.

## Physical model and uncertainty

- SI units; body x forward/y right/z down, local world north/east/down. Body position/velocity reference the current CG; aircraft geometry is measured from a fixed body datum. Quaternion maps body to world. Gravity is world +z.
- Every cell evaluates local velocity (`body velocity - wind`, plus angular velocity cross lever arm and axial propwash), local incidence, table-driven lift/drag/moment, actual deflection and skin state. Static post-stall tables and a bounded compressibility approximation are implemented. No dynamic stall hysteresis, inter-cell downwash, calibrated spin model, flaps or slats yet.
- Structural nodes carry generalized bending/torsion rotations, with parallel front/rear/skin connections. Graph statics solve stiffness-weighted rotations. Section scaling and a phenomenological plastic-strain/softening law change stiffness and load margins. Cell moments and translational/rotational inertial relief feed the graph. Elastic rotation feeds the next aero evaluation. This is a reduced load graph, not FEM or a validated fracture law; spanwise beam kinematics, shear/axial stresses, thermal strength and detailed crack mechanics remain incomplete.
- Separation partitions connected components. Each piece inherits the original rigid velocity field; mass, CG and parallel-axis inertia are recomputed. Linear/angular momentum at the split is tested. Detached pieces currently coast ballistically; their aero/propulsion, collisions and detailed control-chain partitioning are pending.
- Each control chain has range, slew, pressure effort, friction, lag, hinge fraction, jam position and disconnect state. Direct input and Instructor share the chain. Disconnect, jam, axis override and release have focused tests; the other damage modes still need dedicated behavioral coverage. Trim fades with command magnitude to preserve full surface travel endpoints.
- Engine throttle lag, altitude/RPM-dependent power, rotor inertia, fixed-pitch thrust/shaft load, reaction torque, prop drag/slipstream and a simple P-factor thrust-offset approximation are present. Constant-speed governor, feathering and richer engine/propeller damage remain pending. The linear fixed-pitch advance model is crude at high advance ratios.
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

Additional checks pass: splitting each cell into two half-area cells preserves the wrench; explicit cell reduction reproduces aero totals once; mirrored geometry/state and yaw/wind transformations preserve expected signs; symmetric actual loads give opposing wing bending; weak structure in freefall with no aero/prop forces accumulates no strain despite full keyboard pull and 10 s elapsed. Two simultaneous groups split at a tilted attitude with off-diagonal inertia, preserving total mass, linear/angular momentum, kinetic energy and the surviving datum's position. The reduced graph still uses generalized moments about the fixed datum; local beam pivots/shear/axial stresses are not yet a validated spanwise model. Mass loss from local skin/spar material removal is not yet resolved inside `MassPart`; component separation is the implemented mass-change boundary.

`TrimResult.residual` is the Euclidean norm of six scaled equilibrium components: body `Fx,Fy,Fz` including gravity divided by current weight, and body `Mx,My,Mz` divided by **10,000 N m**. It is not an accuracy score or a stability/energy/stall-margin test. Raw force/moment residual vectors and rotor acceleration are now exposed separately; `converged` also requires near-zero rotor acceleration. The current clean 70 m/s case has ~1.7e-6 N force residual, ~7.8e-6 N m moment residual and ~1.5e-14 rad/s² rotor acceleration. Damaged/disabled actuators cannot be bypassed by initialization.

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

## Reproduction and automation

`make test` runs analytic/core and independent mechanics regressions. `python3 tools/experiments.py` covers the existing core slice; `python3 tools/instructor_experiments.py` produces fresh controller comparisons and failed hypotheses. `python3 tools/viewer.py --smoke` now also tests mode selection, reset persistence and integrator freezing through the actual adapter. `python3 tools/viewer.py --controller legacy` or `--controller off` launches comparisons locally; M cycles modes. No GUI handling claim is justified by these automatic tests.

The first commit had no Actions runs/status checks. A minimal repository workflow now builds these same tests, runs both experiment entrypoints and the Agg viewer. Remote completion must be checked after push; local passes are not a remote-CI result. Focused ASan/UBSan mechanics review also passes in Cloud.

## How decisions change

Record a testable hypothesis, actual result, uncertainty and useful next experiment. Routine reversible implementation choices can proceed; any choice removing or weakening a core requirement must be reported first. Preserve failed evidence when it changes interpretation. Do not turn partial mechanisms, stubs or a rendered aircraft into a claim of full v0.1 completion.

No project distribution license has been selected.
