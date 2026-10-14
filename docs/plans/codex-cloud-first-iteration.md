# Codex Cloud: first implementation handoff

## Goal and environment

Develop the flight-control, aerodynamic, and damage simulation core in the dedicated Codex Cloud environment for `4everWZ/flight-damage-sim`. Start from the restored design rather than recreating a thinner design from a summary. Do not use an unrelated repository or saved environment.

Read these documents first:

1. `docs/decisions/current-decisions.md`
2. `docs/sources/user-requirements.md`
3. The complete `docs/spec/flight-sim-core-v0.1.original.md`
4. `docs/research/engine-options-research.md`

Inspect existing repository instructions and relevant `.agents/skills` through the execution environment if they are present. Read only skills relevant to this work. The historical Unity/C# suggestion is not the final technology choice; C++/Python fit should be established experimentally. The core controls, flight dynamics, and damage behavior must not be diluted for an easier demonstration.

## First useful outcome

Create a buildable, headless foundation with explicit control, aerodynamic, structural, and systems boundaries, plus one synthetic parameterized WWII-style propeller validation airframe. Deliver runnable scenarios and measured evidence. Documentation alone, a viewer moving a model, or a collection of unimplemented interfaces is not sufficient to claim the first working foundation.

Use a focused implementation branch and a draft PR for review. Keep changes restricted to this project. Do not merge or deploy without appropriate authorization. Do not modify or overwrite the preserved original specification.

## Boundaries to establish

- `PilotIntent`: mouse aim direction, direct pitch/roll/yaw requests, axis-active state, throttle and trim. Device keycodes stay outside the core.
- Guidance, body-rate control, per-axis arbitration, and damaged control-chain/actuator response have distinct responsibilities. Actual surface state, not a requested attitude or artificial torque, drives the aircraft model.
- Aircraft rigid-body state and mass properties use explicit coordinates and units. Force-at-position contributions produce a reduced force/moment only at the integration boundary.
- Each aero cell carries geometry, local flow, aerodynamic tables, control-surface state, stall state, effective area, and damage state. Keep per-cell outputs available to structure and telemetry before summing.
- Separate structural graph and systems graph; cell-to-structure mapping is explicit. Define how loads, stiffness/strength changes, connection failure, and geometry feedback are exchanged. Do not store a single wing-health value as the solver.
- Fixed 120 Hz stepping, simulation-tick input/damage events, explicit seed ownership, state reset, replayable scenarios, and telemetry independent of a viewer.
- A replaceable integration boundary must not make the distributed cells or structure graph dependent on a renderer.

Choose practical names and serialization formats after inspecting the environment. These boundaries are behavior requirements rather than a prescription for exact class names.

## Airframe and model scope

The first vehicle is a generic validation fixture with data-defined geometry, mass/inertia, control surfaces, engine/propeller parameters, aerodynamic tables, and structural connections. Identify assumptions and uncertain parameters explicitly. Do not label synthetic values as measured historical aircraft data.

The implementation must support multiple cells and asymmetric results from the outset; small analytical fixtures may accompany a representative 80–200-cell fixture. A smaller test fixture is a unit test, not grounds to replace the distributed model. Keep clean-flight, direct controls, mouse guidance, mixed ownership, aero damage, structural response, and damaged control chains in the original development order. Record incomplete parts honestly while progressing toward the integrated behavior.

Basic projectile penetration and later multiple-aircraft validation remain in the recovered scope. They can follow the core damage-injection scenarios; they should not displace them in the first iteration.

## Resolve only meaningful backend uncertainties

Use the engine-fit research to choose the smallest trial that can settle the backend. JSBSim's existing model or trim capability can be a reference, not proof of the new model's correctness. If using its external-force or mass APIs, test unit conversion, sign conventions, force duplication, and the tick on which updates affect dynamics. If those adapters compromise local cells or structural topology, retain JSBSim as a reference rather than forcing it into production.

Favor one authoritative simulation implementation. Python is useful for experiments, scenario definitions, analysis, and comparison; C++ is a plausible production core. Do not build two complete engines merely to compare them. A choice may be made with clear evidence and remaining uncertainty, rather than waiting for an exhaustive benchmark.

## Evidence to produce

Start with the tests relevant to the implemented stage and expand as it works:

1. Analytic force/moment fixtures: known force at a point, symmetric cells, asymmetric cells, explicit frame and unit conventions.
2. Headless clean-flight fixture: trim or a documented near-equilibrium condition, finite state, physically interpretable response and energy changes. Synthetic parameter assumptions must accompany plots or telemetry.
3. Direct-control response: surface limits and actuator response; control damage cannot be bypassed.
4. Mouse and keyboard behavior: mouse tracking through surfaces; full keyboard roll owns only roll; release is smooth and measurable. No direct attitude writes or hidden corrective torques.
5. Local damage: remove 25% of the left outer-wing aero cells, compare with clean and symmetric-damage cases, and measure force/moment changes and instructor saturation.
6. Structural load fixture: stiffness-dependent redistribution, low-load survival after spar weakening, and later high-load progressive failure. Distinguish a structural bench fixture from a full aircraft maneuver until coupled.
7. Repeatability and timestep convergence: identical scenario replay; compare 120 and 240 Hz with stated tolerances and measured differences. Do not promise cross-platform bitwise determinism without evidence.

Use meaningful physical assertions, not only tests that duplicate the implementation equations or check that the program runs. The original acceptance IDs should remain traceable, with each marked implemented, tested, failed, or pending. First-round progress is not full v0.1 completion.

## Research iteration and reporting

Work in short hypothesis-driven cycles: identify uncertainty, make the smallest model or data change, run a discriminating scenario, inspect measurements, and decide the next step. Report actual measured progress and adverse results. Avoid unrelated SHA/process gates or impressive-looking test counts without behavioral coverage.

Return the draft PR link, how to reproduce the scenarios, the observed outcomes, known failures, uncertainty, and the next experiment most likely to change the model or implementation. Send a concise update if a finding would require weakening a core requirement; do not silently make that tradeoff.

Only after the headless core provides useful evidence, attach a simple existing viewer such as FlightGear. Keep its own FDM disabled when external dynamics are authoritative, preserve the custom mixed-input path, and plan per-cell/structural telemetry rather than treating native pose output as full debug visualization.
