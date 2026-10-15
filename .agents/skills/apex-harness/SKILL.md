---
name: apex-harness
description: Use for substantial coding, cross-module refactors, architecture, research, or ML work where semantic risk, evidence, user decisions, or completion claims need explicit discipline. Do not use for trivial low-risk local edits.
---

# APEX Harness

Use this skill when the work needs execution discipline beyond ordinary coding judgment. It captures rules that are easy for a model to lose while iterating: semantic risk, evidence boundaries, material user decisions, and stopping.

## Core rules

- Classify by the consequence of being wrong, not by domain, file count, or apparent implementation size.
- Define the changed claim before choosing verification.
- The cost of verification must be proportionate to the uncertainty it removes and the consequence of being wrong.
- Do not silently simplify, omit, downgrade, or narrow the requested scope.
- Ask only before a choice that changes what the user gets, what an experiment means, or what the result can claim.
- Once the changed claim has credible evidence, stop; do not verify unrelated unchanged behavior for reassurance.

## Route the work

Read [workflow.md](references/workflow.md) for semantic tiers, consultation, completion claims, and the execution loop.

Read [verification.md](references/verification.md) for evidence selection, testing boundaries, anti-loop rules, and stopping conditions.

Read [research-ml.md](references/research-ml.md) when the work changes model, data, training, inference, evaluation, checkpoint, or research semantics.

If a canonical project document would become wrong or misleading, update it. When a task creates, splits, consolidates, supersedes, or retires project documents, use `managing-project-docs` for ownership and lifecycle.

For input requirements or specifications, use `managing-project-docs` to determine their ownership and maintenance responsibilities.

## Activation output

Before implementation, briefly orient the user in natural prose. State the semantic tier and the consequence that justifies it, then name the changed claim and the cheapest evidence that could falsify it. Mention a material user choice or canonical-document synchronization only when one actually exists. Omit empty categories, do not emit placeholders, and do not delay the work to format a fixed checklist.
