---
name: managing-project-docs
description: Keeps project documentation small and coherent by updating or consolidating existing specifications, decision records, work plans, and handoffs before creating new files. Use when those documents are created, changed, reorganized, consolidated, superseded, or retired. Does not define execution risk or verification policy.
---

# Managing Project Docs

Keep project documentation as small as possible. If new information already belongs in a current specification, decision record, work plan, or handoff, update that file instead of creating another one.

Follow explicit user instructions and established repository conventions first. Use this skill's default locations, linking, and lifecycle rules only when the repository has no convention for the case. When the user asks to review or consolidate the existing documentation structure, make the requested change without silently overriding unrelated conventions.

## Terms

- **Canonical** — the single authoritative identity or location for a document.
- **Current** — still applicable; a plan or handoff remains current while its execution or transfer remains open.
- **Artifact ownership** — the information a document is responsible for keeping current.
- **Input artifact** — material supplied to define or constrain a task, such as requirements, an external specification, an issue, a brief, a benchmark definition, or a user-provided plan.

Using a document as task input does not by itself establish or remove its maintained role. Preserve maintenance responsibilities established by user instructions or repository conventions. Repository location, filename, detail, or formal structure alone does not establish that role.

Consume inputs without such maintenance responsibilities as baselines. Do not rewrite, annotate, or synchronize them unless maintaining them is part of the authorized task.

Use this order:

1. Make no documentation change when no durable or working information changes.
2. Update or extend the existing canonical document.
3. Create a separate file only when its information needs to be accepted, maintained, consumed, replaced, or retired independently.
4. Split further only when each resulting file meets one of those independent needs.

The existence of a template, implementation unit, algorithm, development stage, experiment, review, date, or task is not by itself a reason to create a file. Create a work plan only for work that must remain coordinated or resumable, and a handoff only for an actual transfer of responsibility.

By default, an ordinary single-session change needs no project document. Use one specification for a durable contract. Add a decision record only when its choice or rationale needs to be accepted, maintained, or referenced independently. Use one current work plan for coordinated or multi-session work, and a handoff only for an actual transfer.

When enough independently trackable requirements make partial implementation or scope drift realistically easy to miss, add a compact spec-to-implementation mapping to the existing canonical specification or current work plan. For input baselines without maintenance responsibilities, keep the execution mapping in working context or the current plan. Do not create a separate matrix by default; follow an established repository matrix convention when one exists.

Writing or editing a document does not by itself make a proposed contract authoritative; follow the repository's normal acceptance process.

Read [topology.md](references/topology.md) when choosing which document is authoritative, resolving its path, consolidating documents, or retiring one. Read the lifecycle reference for every document type in scope: [specifications](references/specifications.md), [decisions](references/decisions.md), or [work plans and handoffs](references/working-docs.md).
