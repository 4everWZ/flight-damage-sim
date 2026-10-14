# Flight Damage Sim: engine fit research

Date: 2026-10-14. Status: documentation-based recommendation; no simulation was implemented or executed during this research. Development and empirical comparison belong in the dedicated Codex Cloud environment.

## Outcome

Created and verified the private repository [4everWZ/flight-damage-sim](https://github.com/4everWZ/flight-damage-sim), default branch `main`, initially containing only `README.md`. No project license was selected. A name/description check of all 70 pre-existing owner repositories found no flight/simulator/War Thunder/IL2 candidate.

The original [FlightLab v0.1 specification](https://chatgpt.com/c/8c79d2da-f148-8328-b951-93766a55c0cd?messageId=83e5c30d-f0ce-4700-9449-9095619ddfbc) was recovered separately and read in full. Preserve it unchanged as a historical engineering baseline. A separate current-decision document should record that the latest user request opens the implementation choice to C++/Python and asks for core-first Codex Cloud development with later integration into an existing visualization/control tool. This must not silently erase its control, aerodynamic, or damage requirements.

## Recommendation

Prefer an independently owned, renderer-free C++ simulation core, with Python for repeatable scenarios, parameter studies, analysis, and reference comparisons. Keep the rigid-body integration boundary replaceable. Use JSBSim as a clean-aircraft reference and, only if a small empirical adapter trial supports it, as an optional integration/propulsion backend. FlightGear is a plausible separate first flight viewer for either route.

This is an architectural inference from the recovered requirements and the documented interfaces, not a measured finding that custom C++ is faster or more accurate. Do not implement two complete engines. First test the few uncertainties that would actually change the backend decision, then commit to one implementation.

## Requirements that drive the choice

- 120 Hz fixed simulation step, independent rendering cadence.
- Distributed aero cells, initially about 80–200, with local velocity, local stall, local damage, and force application positions.
- Separate aerodynamic, structural, and systems topologies.
- Lightweight structural load graph with stiffness-dependent load redistribution, bending/torsion, delayed overload failure, and disconnection-driven detachment.
- Mouse aim through guidance and body-rate control to actual control surfaces. Keyboard owns each activated axis at full authority; release blends smoothly back to the instructor.
- Actuator/linkage/hinge damage must constrain both direct and instructor commands. No hidden corrective torque, artificial attitude lock, or restored lost authority.
- A damaged spar can survive 1G and fail later at high G. Skin loss can ruin aerodynamics while the wing stays attached.
- Data-driven reusable aircraft; the first single-aircraft milestone does not satisfy the eventual three-aircraft reuse criterion.

## Route A: JSBSim C++ core + Python experiments + FlightGear

### Verified interfaces

JSBSim provides a C++ nonlinear 6DoF library, standalone batch operation, XML-defined systems, and Python bindings. The upstream README explicitly cautions that bundled real-aircraft models are approximations; their presence is not validation of a WWII aircraft or damage model. [Official repository](https://github.com/JSBSim-Team/jsbsim)

The aerodynamic model supports summed contributions to six force/moment axes, expressed through functions and tables. Damage-dependent properties can therefore modify aerodynamic contributions. This is an extension mechanism, not an existing structural damage simulation. [Forces and moments](https://jsbsim-team.github.io/jsbsim-reference-manual/user/concepts/forces-and-moments/)

`FGExternalForce` accepts arbitrary forces and moments. The documented runtime properties control direction, magnitude, and force application location. A cell solver could retain its local results and feed either individual forces or an explicitly reduced wrench into this interface. Its frame and unit conventions need adapter tests. [External-force API](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGExternalForce.html)

The C++ mass-balance API exposes base inertia, base CG, and empty-weight setters, as well as point-mass machinery. This is relevant to detachment but does not by itself solve momentum transfer or detached-body simulation. The Python mass-balance reference inspected here does not list corresponding setters: verify actual bindings before depending on a pure-Python damage adapter. [C++ mass API](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGMassBalance.html), [Python mass API](https://jsbsim-team.github.io/jsbsim/python/FGMassBalance.html)

The executive supports explicit timestep, initialization, step execution, and reset. Its source binds `simulation/randomseed`; wind and sensor components can have separate seeds. These enable reproducibility controls, but do not establish bit-identical replay across operating systems or compilers. [Executive API](https://jsbsim-team.github.io/jsbsim/python/FGFDMExec.html), [Executive source](https://jsbsim-team.github.io/jsbsim/FGFDMExec_8cpp_source.html), [Wind source](https://jsbsim-team.github.io/jsbsim/FGWinds_8cpp_source.html), [Sensor API](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGSensor.html)

### Fit and risks

Advantages: mature 6DoF integration, atmosphere, propulsion options, initialization, trim, batch workflows, and a useful independent comparison point. A small external-force adapter may save writing generic infrastructure.

Missing project work remains substantial: the distributed aero cells, independent structural graph, control-system topology, damage propagation, load redistribution, mixed mouse/keyboard instructor, and damage telemetry must still be implemented. Merely scaling a whole-aircraft lift coefficient or wing strength is inadequate.

An acceptable hybrid keeps the project-owned aero/structure/systems state authoritative and uses JSBSim only for explicitly delegated subsystems. It must disable or replace overlapping built-in forces to avoid double counting. Verify the step at which external forces and changed mass properties enter integration, including whether an adapter adds a one-step lag. Detachment must update mass/inertia and the physical state consistently. Do not assume the existing API automatically provides these guarantees.

If the hybrid adapter requires extensive private model hooks or keeps forcing the local-cell/structural model back into whole-aircraft coefficients, drop it as the production backend and retain JSBSim only as a reference.

## Route B: independent C++ core + Python experiments + existing viewer

Advantages: local cells and independent graphs remain first-class; the order of force generation, structural response, geometry changes, control constraints, and integration is explicit. A narrow state/force interface lets the viewer and integrator change without rewriting the aircraft model.

Costs: rigid-body integration, atmosphere/propulsion boundaries, coordinate conventions, trim support, and numerical verification become project responsibilities. A custom solver is not credible merely because it is custom. Establish analytic force/torque and conservation tests, timestep convergence, clean-aircraft envelope comparisons, and damage acceptance scenarios before asserting fidelity.

Python is appropriate for scenario authoring, optimization, plotting, uncertainty analysis, and JSBSim comparisons. Avoid maintaining a separate Python flight model that can drift from production C++; call the same authoritative core. A Python-first cell experiment may be useful for a disputed model equation, but it is not grounds to postpone controls or damage indefinitely. Measure rather than assume that 80–200 cells require a particular optimization.

## FlightGear integration: useful but not sufficient by itself

FlightGear documents disabling its own FDM with `--fdm=null` and receiving an external FDM through native network interfaces. This permits the simulator process to restart independently of the viewer. [Official FlightGear external-FDM guidance](https://wiki.flightgear.org/Property_Tree/Sockets)

Its native FDM structure carries pose, rates, aircraft/engine state, and normalized control-surface positions. The inspected header has version 24 and mixed units; code must use the installed release's exact layout and conversion rules. It does not carry a distributed cell mesh or structural graph. [Native FDM header](https://github.com/FlightGear/flightgear/blob/next/src/Network/net_fdm.hxx)

The native control structure carries conventional control values and trim; the inspected header has version 27. It has no mouse aim vector or explicit per-axis ownership fields. Therefore the project still needs an input adapter and a bounded intent protocol. Reusing stock FlightGear controls cannot be reported as having implemented WT-style hybrid control. [Native controls header](https://github.com/FlightGear/flightgear/blob/next/src/Network/net_ctrls.hxx)

Keep the core at 120 Hz while the viewer receives snapshots at an independently measured rate. Add a separate debug channel for per-cell force/stall/damage and structural load-margin visualization. Displaying pose is only the first viewer milestone, not the required debugging interface or completed damage visualization. Input sampling, event timestamps, pause/reset, ownership, and handoff timing must be testable without opening FlightGear.

## First falsifiable experiments in Codex Cloud

1. Force adapter: apply known forces at known points and compare total force and moment against analytic values. Include left/right symmetry, coordinate conversion, and no duplicate built-in aerodynamic contribution.
2. Replay and convergence: fixed initial state, aircraft data, seeded events, and input trace at 120 Hz; fresh-instance replay must agree within an explicitly stated tolerance. Compare 120 vs 240 Hz on a bounded maneuver. Report tolerance and measured error rather than claiming generic determinism.
3. Input ownership: during mouse tracking, assert full roll-left owns only roll; pitch/yaw remain eligible for instructor control. On release, measure command continuity and handoff duration. Repeat after linkage damage to ensure no bypass.
4. Local damage: remove 25% of left outer-wing cells; measure asymmetric lift/drag and roll response. A symmetric-damage control case helps distinguish geometry effects from accidental numerical bias.
5. Structure: intact vs weakened front spar at low and high load. Record graph loads and stiffness redistribution; verify survival at 1G and progressive failure during the later high-load maneuver without damage-time detachment scripting.
6. Detachment boundary: verify mass/CG/inertia updates and force ownership after connectivity failure. For a JSBSim adapter, explicitly test when those updates affect the next state.
7. Viewer smoke test only after core evidence: external pose and surface positions in FlightGear, real input-to-intent-to-surface path, and debug telemetry. Do not wait for terrain, detailed models, weapons, or polished UI to obtain the core results.

Each iteration should state the hypothesis, changed model/data, scenario, measurements, uncertainty, conclusion, and next discriminating experiment. Keep checks tied to physics and player control, rather than generating unrelated process gates. A failed experiment should guide the next model correction; passing a smoke test must not be represented as validation of the complete original specification.

## Cloud launch blocker and smallest next step

At handoff time no saved coding environment matched this new repository. Do not use an unrelated PhD, research, or other-project environment. The supported environment-creation entry is [Create a cloud environment](https://chatgpt.com/cloud-environments/new), opened on web or desktop. The user should select `4everWZ/flight-damage-sim` and create its environment; the available environments can then be refreshed and the engineering task launched with the returned configuration ID. Do not claim that task execution has started until creation is confirmed.
