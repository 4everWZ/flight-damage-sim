# Repository-local APEX skills

The two skill directories are copied unchanged from [4everWZ/apex-harness](https://github.com/4everWZ/apex-harness), upstream main inspected on 2026-10-14 (commit `77fbd57`). `APEX-LICENSE` preserves upstream's MIT license; it does not select a license for this simulation project.

The current upstream README describes the two-skill layout and a PowerShell package-validation workflow, but supplies no installation command or installer. Repository-local Codex discovery uses `.agents/skills/<name>/SKILL.md`; no global files, hooks, safety settings, credentials, or upstream AGENTS rules are installed. This project's AGENTS file keeps its own simulation requirements authoritative.

Inspection covered both skills, their references/templates, the plugin manifest, and `scripts/validate-skills.ps1`. The script performs validation only. No upstream script was executed. Source-to-copy comparison and all relative skill Markdown links were checked locally. The official package validator was not run: this Linux environment has neither `pwsh` nor the official `skill-creator` validator. This is a static installation check, not behavioral evaluation of the skills.

To update, inspect current upstream instructions and diff the two skill trees, then replace only these local directories while preserving project rules and the license.
