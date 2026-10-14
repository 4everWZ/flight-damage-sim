# Specification Lifecycle

## Select the contract boundary

A specification represents a stable contract, not the chronology of developing it. Prefer one current specification for one coherent contract. Give an internal algorithm, stage, or module its own specification only when the project maintains it as a contract of its own—for example, it is accepted, consumed, replaced, or versioned independently.

Implementation stages, experiments and run histories, candidate reviews, confirmation records, generated manifests, runtime snapshots, method-revision history, and task notes are not specifications by themselves.

## Resolve draft and current state

An accepted specification owns the current contract. A draft owns a proposed contract and does not claim that implementation already conforms. Draft and accepted states are established by Git context and the repository's acceptance process; the default specification template does not require a status field.

Input requirements may propose changes to the accepted contract; coordinate authorized changes in working context or a work plan when needed.

## Draft and synchronize

Prefer drafting a new or changed contract before implementation when its observable behavior, interfaces, invariants, or acceptance can reasonably be defined. Exploratory implementation may precede a draft, but it does not establish the contract.

At synchronization, the specification contains the accepted contract, independently owned choices link to their decisions, unaccepted deviations remain unresolved, and non-contract implementation details remain outside the specification.

Synchronize accepted changes with their owning specification in the same logical change.

Before accepting the specification or claiming that contract synchronization is complete, confirm that the specification, implementation, evidence, and active decisions agree. Open decisions are resolved and evidence supports acceptance. Reference existing durable evidence when useful; keep run logs in their external system. Do not create evidence artifacts or identity metadata solely to make documentation more traceable. Apply `apex-harness` evidence requirements during verification.

## Reject a draft

For a rejected or abandoned draft:

- discard the draft Git version; an existing accepted version remains canonical
- if no accepted version exists, retire the artifact under the topology procedure
- first transfer a still-current rejection constraint to a rejected decision record when the constraint needs to be maintained or referenced independently; otherwise keep it in the current document that still depends on it

A rejected draft has no legacy lifecycle.

## Supersede or retire

Resolve a retained specification's legacy path with the topology path precedence, using `docs/specs/legacy/<topic>-NN.md` as its fallback in place of the topology table's step 4. Use the next unused generation, beginning with `01`.

Deletion is the default after supersession. Retain a predecessor only when the topology retention rule applies. Before placing it under `legacy/`, record `Retention reason` and `Remove when`; add `Superseded by` only when navigation is required by the user, an audit requirement, or repository convention. Do not assign a legacy generation merely to preserve a complete sequence.

Exclude retained legacy specifications from normal current-document navigation. Read them only to resolve lineage, audit, or a documented historical dependency.

When retaining a superseded specification, the legacy artifact is the last accepted Git version, not the successor draft. Keep the successor at the canonical path. If a direct navigation link was intentionally retained, repair older lineage fields that named the predecessor's former path; otherwise Git history provides lineage. Apply the topology lineage rules.

When not retaining a superseded specification, apply the topology retirement procedure.

For legacy code, keep the current contract in its specification. Create a separate decision record for successor or retirement reasoning only when the choice or rationale needs to be accepted, maintained, or referenced independently; otherwise keep the reasoning with the specification. Record the target state, affected consumers, relevant recovery information, and evidence that the contract retired.

When a contract retires without a successor, apply the topology retirement procedure. Do not leave a retired contract as the current contract or invent a successor only to retain the file.
