# Flight Damage Sim

- Read `docs/decisions/current-decisions.md`, the original specification and the current implementation plan before core work. Preserve `docs/spec/flight-sim-core-v0.1.original.md` unchanged.
- Use the repository-local `.agents/skills/apex-harness` for substantial core work and `.agents/skills/managing-project-docs` when maintaining project artifacts. These are the unmodified APEX skills; existing user and project rules take precedence.
- Keep the authoritative simulation renderer-free. All motion is integrated from forces/moments; mouse and keyboard commands share actual damaged control chains. Do not substitute HP, attitude writes, or scripted separation.
- Keep aerodynamic cells, structural connections, and control systems independently represented. Synthetic fixture values are not historical aircraft measurements.
- Run `python3 tools/experiments.py` for the first headless slice. Build and generated telemetry live in ignored `build/` and `results/`.
- Work on a feature branch; PRs must be draft. Honor any session-specific pause on remote writes. Never merge automatically.
