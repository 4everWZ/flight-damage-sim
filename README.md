# Flight Damage Sim

A renderer-free C++20 prototype for WWII-style propeller-aircraft flight, mixed mouse/keyboard control and physical aerodynamic/structural damage. Developed and tested in this repository's Codex Cloud environment.

The first synthetic fixture has 152 aerodynamic cells, 6 structural nodes with front/rear/skin load paths, and independent damaged control chains. Mouse aim and keyboard requests reach actual control surfaces; forces and moments drive 6DoF motion at 120 Hz with distributed airframe loads evaluated at the solved body midpoint. Structural section/stiffness and load exposure cause redistribution and delayed failure. Separation follows graph disconnection and updates mass, CG and inertia.

All flight experiments in this repository are dynamic simulated flights, not real-world flight tests.

This is a measured first slice, **not completed v0.1 or a historically validated aircraft**. No HP, direct attitude control or scripted wing destruction substitutes for physics. Known limits include reduced structural statics/material law, frozen detached geometry, uncalibrated blade/engine data and basic Instructor guidance.

## Run

Requires C++20 GCC, GNU make and Python 3:

```sh
python3 tools/experiments.py
python3 tools/instructor_experiments.py
python3 tools/aim_tracking_experiments.py  # moving-heading feedforward, dynamic simulated flight and sampled input
python3 tools/control_polar_calibration.py  # fixed-condition preparation; published program-output replay and separate synthetic 2D checks
python3 tools/control_polar_measured_subset.py  # measured curve holdout in coefficient space; unknown Mach blocks runtime export
python3 tools/control_polar_dense_slice.py  # preselected denser stations; preserves old failure and unseen-delta holdout
python3 tools/control_polar_experiments.py  # optional 2D component coefficients, baseline identity and actual damage/control paths
python3 tools/trim_stall_diagnostics.py  # independent table/roll checks, pitch envelope and dynamic simulated flights
python3 tools/glide_experiments.py  # fixed-throttle glide equilibrium, dynamic simulated damaged transients and separate static-flow diagnostics
python3 tools/cell_frame_experiments.py  # opt-in cell bases, identity compatibility and synthetic canted-cell flights
python3 tools/propulsion_experiments.py  # retained legacy-model counterexamples
python3 tools/rotor_experiments.py       # signed blade loads, pitch governor, free-flight ledgers
python3 tools/detachment_experiments.py  # owned-cell piece flight and control partition
python3 tools/material_experiments.py    # explicit local inventory and conservative mass transfer
python3 tools/control_experiments.py     # PI headroom, request/event ablations and sensitivity through 960 Hz
python3 tools/sensor_history_experiments.py  # fixed histories, stage clocks and surface midpoint sampling
python3 tools/load_sample_experiments.py  # original force-sample age/current-mass normalization at fixed 120 Hz control
python3 tools/clock_experiments.py  # final internal/sensor/output clock decomposition on fixed 960 Hz physics
python3 tools/finite_span_experiments.py  # physical-capability counterexample; failed finite-span hypotheses remain false
python3 tools/lifting_line_reference.py  # independent low-angle reference; no production polar reinterpretation
python3 tools/section_transfer_reference.py  # explicit sectional fixture; equivalent cell wrenches, not a pressure map
python3 tools/local_section_coupling.py  # localized section-slope loss: frozen loads versus self-consistent wake
python3 tools/roll_authority_reference.py  # real settled controls; static roll gain versus damage bias, independent section model
python3 tools/control_fault_reference.py  # actual disconnect/captured-angle jam crossed with local section loss; static only
python3 tools/control_transition_reference.py  # held-input fault timing; continuity and endpoint/midpoint torque quadrature
```

This builds and runs core assertions plus flight/control/damage/structure scenarios, identical replay and 120/240 Hz comparisons. CSV/JSON results are written under ignored `results/`. The core has no renderer or Python dependency.

GNU make records effective `CXX`, `CPPFLAGS` and `CXXFLAGS` values under ignored
`build/.compile-config`. Changed settings rebuild both shared objects and
requested executables, including when switching back; unchanged builds remain
incremental. The build uses the existing POSIX command environment (including
`printf`, `cmp` and `mv`). Run `python3 tests/build_config_tests.py` for isolated,
real-core configuration-switch regressions. Python is not needed for make alone.

An optional Matplotlib engineering viewer displays attitude, local cell forces, missing skin, actual controls, axis ownership and structural utilization:

```sh
make -j2 build/flight-live
python3 -m venv .venv-viewer
.venv-viewer/bin/python -m pip install --no-cache-dir -r requirements-viewer.txt
mkdir -p results
.venv-viewer/bin/python tools/viewer.py --smoke       # no display needed
MPLBACKEND=TkAgg .venv-viewer/bin/python tools/viewer.py  # desktop Python with Tk
```

[Viewer startup, controls, telemetry and manual checks](docs/viewer-operations.md) cover mouse aim, keyboard axes, throttle, three-axis trim, prop RPM/governor, damage, focus release, pause and reset. All Matplotlib keymaps are disabled. The viewer pauses on lost focus/pointer exit, clears held input and drops wall-time backlog; physics stays at 120 Hz. Automated callbacks and a real Tcl idle queue are tested; native desktop focus/keyboard behavior and human handling remain unverified. The smoke image is marked as an offline test.

On Windows PowerShell, follow the [native startup commands](docs/viewer-operations.md#windows-powershell), which build the viewer with MinGW `g++` and use `.venv-viewer/Scripts/python.exe`. The bash commands above are not PowerShell commands. The same manual includes expected observations for each desktop check and the full automated regression sequence for a bash/GNU make environment.

The default is the signed prop variant; use `--airframe validation.airframe` for legacy propulsion and `--controller legacy` / `--controller off` for guidance comparisons. Aim elevation requests air-relative flight-path angle. Desktop handling checks remain separate from automatic physics evidence.

## Minimal integration boundary

`PilotIntent` carries a world aim direction, normalized roll/pitch/yaw requests, one active flag per axis, throttle, available trim and requested prop RPM ratio. `Simulation::step` advances one fixed tick. Per-cell loads remain available before reduction to a `Wrench`; `integrate_body` is the rigid force/moment boundary, and `integrate_rotor_body` jointly integrates the free body and internal spinning shaft. SI units and frame conventions are documented in current decisions and the core header.

`Simulation::trim_fixed_throttle(TAS, altitude, throttle, prop_rpm_ratio=1)` initializes a force/moment/rotor-balanced air-relative climb or glide while keeping throttle prescribed. Check `converged` and the reported force/moment/rotor/governor residuals before using the entry. Flight-path angle is positive for climb; subsequent ticks integrate freely. This uses the existing synthetic shaft/friction and propeller model, without a calibrated historical idle condition. The legacy `trim(speed, altitude)` remains available. See [evidence and bounds](docs/decisions/current-decisions.md#fixed-throttle-trim-and-glide-controlload-evidence).

Optional control-surface coefficients use `control_polar NAME alpha_deg delta_deg CL CD CM` records on a rectangular grid and `cell_control_polar CELL_ID NAME` to bind a controlled cell. Merely declaring a table leaves the legacy model active; existing data files are unchanged. Each axis needs at least two stations; records may be unordered, while duplicate/missing grid points, invalid bindings, non-finite values and negative CD are rejected. Out-of-domain coordinates clamp to the table boundary and are reported rather than extrapolated.

The optional law samples local **flow** angle and the actual chain endpoint/midpoint angle multiplied by existing hinge effectiveness and cell sign. It does not apply `control_gain` again. Forces, moments, area/skin damage, structural loading and detached control mechanics retain their existing paths. Legacy `alpha` and the fixed-15-degree `stall` field remain diagnostic proxies; bound cells additionally expose `flow_alpha`, `delta_eff`, `control_polar_clamped` and a proxy label in live JSON. These static effective-component tables add neither downwash nor dynamic stall and are not historical sectional data. See [contract and measured limits](docs/decisions/current-decisions.md#optional-two-dimensional-control-polar).

`build/flight-live data/validation-bem.airframe` provides a line-oriented local transport. It emits an initial JSON snapshot, then one snapshot per command:

```text
step ticks aim_x aim_y aim_z mouse roll pitch yaw roll_active pitch_active yaw_active throttle trim_roll trim_pitch trim_yaw [prop_rpm_ratio=1]
damage cell id area_fraction roughness
damage spar edge_name remaining_section
damage link chain_name
damage engine power_fraction
damage prop prop_fraction
damage material material_name remaining_fraction
damage governor
governor on|off
controller flightpath|flightpath-p|legacy|off
reset
quit
```

A batch contains 0–120 fixed ticks. Damage applies at the current tick boundary. Snapshots include body pose/velocity/rates, CG/mass, prop pitch/RPM/target/saturation/tip Mach, actual surfaces, ownership, Instructor target rates, local cell flow/forces/stall/damage and structural section/stiffness/loads/strain/failure. `tools/live_client.py` demonstrates the transport; it never writes attitude or runs flight equations.

The independent mechanics review found and repaired torque-free rotation drift, flexed-wing lift work, initial handoff ownership and damaged trim bypass. New PI guidance reduces this fixture's 90° turn minimum height loss from ~41 m to ~19 m and final loss from ~32 m to ~0.2 m. This is a synthetic case; transient loss and damaged/energy-limited behavior remain. Raw trim residuals, failed hypotheses and comparative trajectories are documented below. The signed prop model produces stopped-blade drag, wind-driven startup and speed-dependent load; a finite pitch governor ends near 1600.36 RPM for a 1600 RPM target, while jam/travel damage retain real overspeed. Six flight cases close the discrete energy/world-torque-impulse ledgers. The old propulsion variant retains its failed stopped-prop/windmilling/load hypotheses. All prop values are synthetic, with no compressibility/tip/swirl correction; overspeed cases do not validate transonic behavior. The CI workflow runs the same reproducible entrypoints.

The optional `data/validation-material.airframe` discretizes the same body's outer-wing mass into named material elements, preserving initial mass/CG/inertia. `damage material left_outer_forward .2` leaves 20% of that element and transfers the removed 8 kg into a separately integrated passive body. Owner mass/CG/full inertia and inertial loads update, with total mass/P/H/instant energy checks. This does not implicitly change aero area or spar section. Material particles have no aero geometry; carriers remain non-removable. The current full-roll small-effect timestep hypothesis failed and is reported separately from passing fixed-trim and conservation checks. PR #2/#3 are integrated; their physical claims and verification limits remain recorded in current decisions.

Rate PI now limits integration to remaining normalized-command headroom within each tick, avoiding stored bias beyond a saturation crossing. Positive/negative three-axis controller benches agree at 60–480 Hz. This does not yet resolve the full-roll small-material-effect timestep hypothesis; its failed result and load-feedback ablation remain reported. Gains, input semantics and actual actuator limits are unchanged.

An airframe may add an optional `cell_basis <cell_id> <qw> <qx> <qy> <qz>` record before or after its cell records. It rotates the existing plane-specific chord/span/normal basis; omitted records preserve the old identity path. The loader normalizes finite nonzero quaternions and rejects duplicate/unknown cell IDs. Positions, mass and material data are separate. See the [rotation order, signs and measured boundary](docs/decisions/current-decisions.md#approved-per-cell-reference-frames).

## Project documents

- [Viewer operations and review milestone](docs/viewer-operations.md)
- [Current decisions, measured evidence and limitations](docs/decisions/current-decisions.md)
- [Implementation plan, original acceptance mapping and next experiments](docs/plans/codex-cloud-first-iteration.md)
- [Original 51-section engineering specification](docs/spec/flight-sim-core-v0.1.original.md)
- [Backend/viewer research](docs/research/engine-options-research.md)
- [Repository-local APEX installation](.agents/skills/README.md)

C++ owns the only simulation. Python drives/analyses that core. JSBSim remains an optional future reference; FlightGear remains an optional future viewer. The current viewer is a debugging tool, not the finished playable prototype. Detached bodies now integrate owned-cell aerodynamics and unpowered/jammed surfaces; per-body/cell diagnostics are in the additive `detached` live field. The viewer still draws parent geometry.

No project distribution license has been selected. Vendored APEX skills retain their own MIT license.
