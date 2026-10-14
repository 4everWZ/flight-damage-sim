# Current decisions and evidence status

Updated: 2026-10-14

## Authority and provenance

The user's latest request is to recover the previous design, create a repository, and develop and test the prototype in Codex Cloud, starting with the core flight controls and damage engine. The core should later connect to an existing tool for simple visualization and interactive tests. C++ and Python are both acceptable according to fit. The operation, flight-control, and damage requirements must not be weakened.

The preserved [original v0.1 specification](../spec/flight-sim-core-v0.1.original.md) is a historical assistant-authored design proposal requested by the user. Its Unity 6/C# platform recommendation is historical, not an immutable user language requirement. Its physical and control requirements remain the baseline unless the user explicitly changes them. Original text is preserved unchanged; current decisions belong in this file or subsequent design records.

[Historical user requirements](../sources/user-requirements.md) distinguish the user's own intent from assistant recommendations. Comparisons the user made about commercial games express the desired design; they are not independently verified descriptions of those games' internal implementations.

## Confirmed direction

1. Core simulation precedes weapons, maps, polished graphics, AI, or a full combat game.
2. Use force/moment-driven 6DoF motion, distributed local aero cells, and independently represented aerodynamic, structural, and systems topologies.
3. Mouse aim operates through real control surfaces. Direct keyboard maneuvers retain full authority on each active axis, with smooth release back to the instructor. Neither input path may bypass damaged linkages, actuator limits, or lost aerodynamic authority.
4. Structural weakening and stiffness-dependent load redistribution must permit survival at low load and later failure at high load. Connectivity failure causes separation. HP, scripted destruction, direct attitude control, and aircraft-specific physics hacks are not substitutes.
5. Use 120 Hz fixed simulation timing, reproducible scenarios, explicit uncertainty, and data-driven aircraft definitions. Rendering has its own cadence.
6. Start with one generic parameterized WWII-style propeller validation airframe. Label it as a synthetic validation fixture, not a historically validated Bf 109, Spitfire, or other named aircraft. The historical multi-aircraft validation set remains a later requirement.
7. Implement and test in the repository's dedicated Codex Cloud environment. The repository bootstrap and source recovery are documentation work, not evidence that simulation development or testing has already happened.

## Provisional technical choice

The documentation review favors an independently owned C++ simulation core with Python scenario/analysis tooling. JSBSim is a candidate reference and possibly a replaceable rigid-body/propulsion backend; FlightGear is a candidate external viewer. These are recommendations to test, not a completed technology selection.

Resolve the C++/Python and backend details through small discriminating experiments. In particular, check local-force injection, units and frames, force double counting, step ordering, mass/CG/inertia updates on detachment, repeatability, and mixed-control ownership. Do not make the core conform to a convenient backend by reducing the required physical model. See [engine fit research](../research/engine-options-research.md).

Using Python for experiments does not authorize a separate, incompatible flight model to become the production behavior. Prefer calling the same authoritative core. A limited mathematical reference model may be useful if clearly labeled and compared.

## What is not yet established

- No flight model, instructor, structural solver, or damage model has been implemented or empirically validated at this documentation checkpoint.
- No performance, real-time budget, flight fidelity, or cross-platform determinism claim is justified yet.
- The original numerical examples and proposed aircraft list are design targets, not measured results.
- A viewer integration alone does not demonstrate correct controls, local damage, or structural failure.
- No project distribution license has been selected.

## How decisions change

Record the hypothesis, experiment, result, uncertainty, and reason when choosing a backend or changing a model. Preserve failed or inconclusive evidence where it explains the next step. Normal implementation choices can proceed within these requirements; a choice that would remove or weaken a core requirement must be reported before adoption.
