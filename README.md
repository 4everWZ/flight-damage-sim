# Flight Damage Sim

A reusable WWII propeller-aircraft simulation core, developed and tested in Codex Cloud.

The core combines force-based flight dynamics, War Thunder-style mouse aim with full-authority per-axis keyboard maneuvering, and physical aerodynamic, structural, and control-system damage. Hit points, scripted wing destruction, and direct attitude changes cannot substitute for those systems.

## Design and current direction

- [Current decisions and evidence status](docs/decisions/current-decisions.md)
- [Recovered original FlightLab v0.1 specification](docs/spec/flight-sim-core-v0.1.original.md)
- [Relevant original user requirements](docs/sources/user-requirements.md)
- [Engine fit research and testable uncertainties](docs/research/engine-options-research.md)
- [First Codex Cloud implementation handoff](docs/plans/codex-cloud-first-iteration.md)

The original specification is preserved unchanged. Its Unity 6/C# recommendation is historical; the current request permits C++ and Python according to fit. Language and backend choices should be settled with focused experiments while preserving the required controls, distributed aerodynamics, and structural damage behavior.

The simulation core comes first. A simple existing visualization/control tool follows. The first vehicle will be a generic parameterized propeller validation airframe, with historical-aircraft fidelity and multi-aircraft reuse evaluated later.

## Status

Repository initialized and prior design recovered. This documentation checkpoint contains no implemented or empirically validated simulation. Implementation and testing await the dedicated Codex Cloud task.

No project distribution license has been selected.
