# Working Document Lifecycle

## Decide whether a working document is needed

Do not create a work plan for an ordinary single-session change that can be completed and verified directly. Use one when work must resume across sessions, multiple contributors or systems must coordinate, ordered dependencies matter, blockers must remain visible, or the user explicitly requests a persisted plan.

Represent development stages as sections, milestones, or tasks in one plan unless they have independent owners and lifecycles. Resuming the same work later is not by itself a handoff; a handoff requires an actual transfer.

## Work plans

Keep at most one current work plan for one coherent initiative. Before creating another plan, locate and extend the existing current plan. Create a separate plan only when the work has an independent owner, scope, or lifecycle.

Use a work plan to drive unfinished work. During execution, keep only the claim, evidence state, blockers, and next action needed to resume. Transfer rationale when it becomes part of the current contract or an independently owned decision, then replace plan detail with a link to its durable owner.

When work uses input requirements, keep implementation state, clarifications, and approved deviations in working context or the current plan. In a plan, reference the minimum relevant inputs and map tasks to existing requirement IDs or source sections only when needed to avoid omissions. Use direct references instead of transcribing inputs into another requirements document.

Treat the plan's stated Goal as its lifecycle boundary. An open task or blocker keeps the plan current only while it belongs to that Goal. Follow-on work outside that Goal does not prevent retirement. Create or extend another plan only when the follow-on work independently meets the work-plan criteria.

## Retire working documents

Retire a work plan when:

- its outcome is delivered and its changed claim is credibly verified;
- the work is explicitly abandoned or cancelled;
- its scope is superseded or consolidated into another canonical plan or specification; or
- the initiative or a required dependency is closed and the plan is no longer resumable.

Code being written is not by itself delivery. If the changed claim remains unverified, do not retire the plan as delivered. Explicit supersession, abandonment, or cancellation remains possible only after transferring any still-current constraint, blocker, cancellation reason, or unverified claim that future work still depends on to the appropriate owner.

When a retirement condition is met, apply the topology retirement procedure. Do not repeat its fact-transfer, link-repair, evidence, or retention checklist here. Delete the plan by default; retain it only for a concrete audit, consumer, recovery, or repository-convention reason. Git preserves chronology. Do not add a retired status, archive directory, or retirement document.

## Handoffs

A handoff is a temporary record of a transfer, not a canonical project record.

Keep one handoff at the canonical path for the current transfer and update it in place instead of creating parallel handoffs. A later transfer may reuse the same path with new contents; Git preserves the prior transfer's history.

Reading a handoff does not constitute accepting it. Before acceptance, the handoff may temporarily be the primary transfer document.

After the recipient accepts the transfer, do not use the handoff as a long-term work tracker. If substantive unfinished work still needs to remain coordinated or resumable and meets the work-plan criteria, create or update the current work plan.

Otherwise move every still-current fact, evidence boundary, blocker, and next action into the relevant canonical specification, decision record, or other durable owner. If those facts have been absorbed but the transfer is not yet complete, the handoff may remain temporarily as a record of the transfer; do not use it to accumulate plan tasks.

Retire a handoff only when its transfer completion condition is met or it is explicitly abandoned, no still-current information exists only in the handoff, and no open blocker or named next action remains owned by it. For an abandoned transfer, first move still-valid constraints, blockers, and the cancellation reason to the appropriate owner. Apply the topology retirement procedure, then delete the handoff by default. Retain it only for a concrete audit, consumer, recovery, or repository-convention reason.
