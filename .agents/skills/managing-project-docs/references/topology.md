# Project Documentation Topology

Preserve an established documentation topology. Use this fallback only when no authoritative convention exists. Never reorganize canonical artifacts from model preference alone.

## Choose the artifact

| Artifact | Lifecycle | Create only when | Fallback path | Authoritative for |
|---|---|---|---|---|
| specification | durable | a stable contract meets the specification boundary rule below | `docs/specs/<topic>.md` | current accepted contract, interfaces, invariants, and acceptance |
| decision record | durable | a material choice or proposal needs independent acceptance or durable rationale | `docs/design/YYYY-MM-DD-<topic>-design.md` | the choice, tradeoff, and rationale |
| work plan | working | unfinished work must remain coordinated or resumable | `docs/plans/<topic>.md` | ordered execution and unresolved work |
| handoff | transient | responsibility is actually transferring | `docs/handoffs/<topic>.md` | the current state of that transfer |

Templates are under `assets/templates/`: [specification](../assets/templates/specification.md), [decision record](../assets/templates/decision-record.md), [work plan](../assets/templates/work-plan.md), and [handoff](../assets/templates/handoff.md). Copy only the applicable sections and remove unused placeholders.

## Task inputs

Task inputs without a maintained role established by user instructions or repository conventions are outside this documentation topology. Do not consolidate or retire them under the procedures below.

## Resolve paths

Resolve each artifact path in this order:

1. an explicit user-selected path
2. the repository's established convention for that artifact type
3. an explicit convention in the accepted project documentation
4. the fallback path in the topology table

Recognize an authoritative convention from repository instructions, a document index, consistent current artifacts of the same type, or a current specification or active decision that explicitly names it.

At the same priority, prefer the candidate marked canonical or current, then one linked by current project documents. If candidates remain tied, ask the user to name the canonical location.

Update an existing canonical artifact at its current path. Use stable lowercase kebab-case topics that name the owned contract, decision, or outcome.

## Choose specification boundaries

Use the contract-boundary criteria in the specification lifecycle. Otherwise keep an algorithm, stage, pipeline step, or internal module as a section of the owning specification.

## Resolve content conflicts

Compare documents as they exist in the same repository state when deciding which one is authoritative. Do not treat an unaccepted draft disagreeing with the accepted specification as a documentation conflict; the draft is only a proposal.

A task input proposing changed behavior may likewise differ from the current contract. Treat an authorized target's difference as work to coordinate, rather than as conflicting current specifications; supplying reference material alone does not authorize changing the contract.

Use an explicit user or repository canonical designation first. Otherwise use the artifact that owns the information: a specification for its contract, a decision record for the choice and rationale, a work plan for unfinished execution, or a handoff for the current transfer.

If two documents of the same type claim to define the same information, prefer one explicitly marked canonical or current, then one linked by current project documents. Prefer a current specification, work plan, or handoff, or an active decision, over draft, proposed, superseded, or legacy material. Recency alone does not establish authority. If documents that still appear current and canonical conflict, resolve the inconsistency with their named owner or ask the user rather than choosing by date.

## Preserve identity

After a canonical identity or location changes, no current reference may point to the retired identity, and only the resolved canonical artifact may remain current.

For a retained decision record, set `Status` to `superseded`. For a retained specification, record `Retention reason` and `Remove when`. Record a direct `Superseded by` link only when the user, an audit requirement, or a repository convention needs navigation; otherwise Git history is sufficient. Lineage alone does not require retention or a reciprocal link from the current artifact.

## Keep references sparse

Link only to direct authority or dependency; do not repeat transitive links. A specification may link to decisions that independently own material rationale; those decisions do not link back to the specification. A work plan may link to the minimum set of primary specifications and task inputs it directly uses; it does not link back to a handoff. A handoff links only to its current work plan or, when no plan exists, the primary canonical artifact. Omit a nonexistent primary artifact rather than creating one for the link. An index links to canonical artifacts, which do not link back.

Do not add reciprocal links. A direct successor link is optional and belongs only where navigation is explicitly needed.

## Consolidate an over-split set

When multiple artifacts describe one subject:

1. Identify the smallest set of stable contracts and material decisions.
2. Select one canonical artifact for each contract.
3. Merge still-current contract information into those artifacts.
4. Keep separate decision records only for independently durable rationale.
5. Merge unfinished execution into the minimum number of current work plans.
6. Remove generated data, runtime events, and experiment logs from project documents after confirming their existing owning systems; do not initiate an external move without authority.
7. Repair direct current references.
8. Delete artifacts that no longer own independently useful information.

Do not preserve one file per historical stage; Git owns chronology.

## Retire or delete an artifact

Before deleting any project artifact:

1. Transfer every still-current fact to the artifact that owns it.
2. Redirect or remove every current inbound reference.
3. Preserve required evidence in its designated evidence system.
4. Retain the artifact only for an established audit, current consumer, recovery, or repository-convention reason. Retention requires a concrete reason; historical chronology alone is preserved by Git.
5. Otherwise delete it; Git retains document chronology.

A superseded status does not itself require retention. If a retained predecessor is removed, any retained lineage bypasses it and current links resolve to the successor.

## Optional document index

Use `docs/README.md` only when the repository convention or the number of artifacts warrants a human navigation entry point. The index owns navigation, not contracts, decisions, status, rationale, or work state. Link to canonical current artifacts and avoid copying their content. Omit the index when the repository is already easy to navigate.
