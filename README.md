# Flight Damage Sim

A renderer-free C++20 prototype for WWII-style propeller-aircraft flight, mixed mouse/keyboard control and physical aerodynamic/structural damage. Developed and tested in this repository's Codex Cloud environment.

The first synthetic fixture has 152 aerodynamic cells, 6 structural nodes with front/rear/skin load paths, and independent damaged control chains. Mouse aim and keyboard requests reach actual control surfaces; forces and moments drive 6DoF motion at 120 Hz. Structural section/stiffness and load exposure cause redistribution and delayed failure. Separation follows graph disconnection and updates mass, CG and inertia.

This is a measured first slice, **not completed v0.1 or a historically validated aircraft**. No HP, direct attitude control or scripted wing destruction substitutes for physics. Known limits include reduced structural statics/material law, ballistic detached pieces, fixed-pitch propulsion and basic Instructor guidance.

## Run

Requires C++20 GCC, GNU make and Python 3:

```sh
python3 tools/experiments.py
```

This builds and runs core assertions plus flight/control/damage/structure scenarios, identical replay and 120/240 Hz comparisons. CSV/JSON results are written under ignored `results/`. The core has no renderer or Python dependency.

An optional Matplotlib engineering viewer displays attitude, local cell forces, missing skin, actual controls, axis ownership and structural utilization:

```sh
python3 -m pip install -r requirements-viewer.txt  # in an isolated local environment, if needed
python3 tools/viewer.py --smoke                  # headless transport/render test
python3 tools/viewer.py                          # requires a desktop GUI backend
python3 tools/plot_results.py                    # plots measured scenario CSVs
```

Click the heading/elevation panel for mouse aim. Arrows control pitch/roll; Q/E yaw; W/S throttle. F2–F6 inject local skin loss, damaged control links, weakened spar or engine power loss. Space pauses; R resets. Device keys exist only in the viewer adapter.

## Minimal integration boundary

`PilotIntent` carries a world aim direction, normalized roll/pitch/yaw requests, one active flag per axis, throttle and available trim. `Simulation::step` advances one fixed tick. Per-cell loads remain available before reduction to a `Wrench`; `integrate_body` is the independent force/moment-to-state boundary. SI units and frame conventions are documented in current decisions and the core header.

`build/flight-live data/validation.airframe` provides a line-oriented local transport. It emits an initial JSON snapshot, then one snapshot per command:

```text
step ticks aim_x aim_y aim_z mouse roll pitch yaw roll_active pitch_active yaw_active throttle trim_roll trim_pitch trim_yaw
damage cell id area_fraction roughness
damage spar edge_name remaining_section
damage link chain_name
damage engine power_fraction
damage prop prop_fraction
reset
quit
```

A batch contains 0–120 fixed ticks. Damage applies at the current tick boundary. Snapshots include body pose/velocity/rates, CG/mass, actual surfaces, ownership, Instructor target rates, local cell flow/forces/stall/damage and structural section/stiffness/loads/strain/failure. `tools/live_client.py` demonstrates the transport; it never writes attitude or runs flight equations.

## Project documents

- [Current decisions, measured evidence and limitations](docs/decisions/current-decisions.md)
- [Implementation plan, original acceptance mapping and next experiments](docs/plans/codex-cloud-first-iteration.md)
- [Unchanged recovered 51-section specification](docs/spec/flight-sim-core-v0.1.original.md)
- [Original user requirements](docs/sources/user-requirements.md)
- [Backend/viewer research](docs/research/engine-options-research.md)
- [Repository-local APEX installation](.agents/skills/README.md)

C++ owns the only simulation. Python drives/analyses that core. JSBSim remains an optional future reference; FlightGear remains an optional future viewer. The current viewer is a debugging tool, not the finished playable prototype.

No project distribution license has been selected. Vendored APEX skills retain their own MIT license.
