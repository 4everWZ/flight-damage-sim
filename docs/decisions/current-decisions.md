# Current decisions and evidence status

Updated: 2026-10-14. First implementation exists on a feature branch; this is not full v0.1 acceptance.

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
- The Instructor is a basic bank/flight-path guidance and body-rate controller with local-angle protection, without hidden torques. Its 90° turn loses roughly 31 m altitude over 30 s. G limiting, adaptive authority estimation, trim assistance and subjective flight feel require further work. It does not guarantee stall/spin avoidance.

## Measured first-slice behavior

Run `python3 tools/experiments.py`; it builds, runs core assertions and produces CSV/JSON. `python3 tools/viewer.py --smoke` exercises the actual live input/damage transport and saves a debug image. `python3 tools/plot_results.py` plots the measured CSVs. Generated files are ignored under `results/`.

Cloud GCC measurements for the current synthetic fixture:

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

## How decisions change

Record a testable hypothesis, actual result, uncertainty and useful next experiment. Routine reversible implementation choices can proceed; any choice removing or weakening a core requirement must be reported first. Preserve failed evidence when it changes interpretation. Do not turn partial mechanisms, stubs or a rendered aircraft into a claim of full v0.1 completion.

No project distribution license has been selected.
