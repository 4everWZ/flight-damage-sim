# Decision Record Lifecycle

## Decide when to create a record

Create a separate decision record only when a choice or its rationale needs to be accepted, maintained, or referenced independently. Otherwise keep the rationale in the specification or work plan that needs it.

Keep a local unresolved question in the specification's `Open decisions` while it blocks that contract. Link a separate proposal from the open item when the proposal meets the decision-record criteria above. When the decision becomes active, update the specification and remove the open item; when rejected, retain the open item only if the question remains unresolved.

A proposed decision is a candidate, not an authoritative current constraint. Treat it as active only after the repository's normal acceptance process accepts it; editing the document alone does not make it authoritative.

## Preserve path identity

The date in `docs/design/YYYY-MM-DD-<topic>-design.md` is when the record was first established, including as a proposal. Activation or rejection does not rename it.

Apply the topology path precedence when creating, moving, or deleting a record or successor, and its retention and lineage rules when superseding one.

For a material successor, create a new dated record. Link a retained predecessor forward to it only when the user, an audit requirement, or a repository convention needs navigation. If its base path already owns a different decision, append `-NN` before `.md`, treating the unsuffixed path as generation `01` and using the next unused suffix.

## Change status

Update a record in place when the governed choice is unchanged and only its context, rationale, or consequences need clarification. When the choice changes materially, create a successor and retire the prior record by default under the topology procedure. If the prior record is retained, mark it `superseded` and link it forward only when navigation is explicitly needed. Keep a retained decision record at its resolved path.

When an activated or superseded decision changes the current contract, update the owning specification in the same logical change. The decision owns the choice and rationale; the specification owns the resulting contract. Treat disagreement as an unresolved documentation conflict.

Keep a rejected decision while its rejection remains a current material constraint with independent maintenance or consumption. Retain a superseded decision only when it satisfies the topology retention rule; a successor alone does not justify retention. Retire any decision that no longer constrains the project and has no independent retention reason, whether or not it has a successor.
