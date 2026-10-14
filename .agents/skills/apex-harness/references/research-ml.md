# Research and ML Semantics

This reference supplements the general A/B/C workflow for ML and research work; it does not define the tier itself.

## Preserve experiment meaning

- Changing a loss changes the training objective and research semantics; it is not merely a local function edit.
- Changing preprocessing, data splits, or labels can change the data semantics and comparability with prior runs.
- Changing an evaluation filter or aggregation can change metric semantics; it is not harmless cleanup.
- Distinguish a bug fix that restores the intended experiment from a methodological change. Do not present the latter as merely an implementation fix.
- A shortcut that changes the experiment requires a narrower claim, not a silent redefinition of the experiment.

## Check the real experimental boundaries

- When tensor compatibility matters, trace the actual producer-consumer boundary rather than inferring shapes from names, comments, or paper notation.
- When checkpoint compatibility is part of the claim, verify it with a real representative checkpoint when available; otherwise state the unverified boundary.
- Compare before/after evaluation semantics, not only the resulting numbers.
- When preprocessing, data splits, labels, or evaluation semantics change, prior results are comparable only if their experimental meaning remains unchanged.

## Keep claims proportional to evidence

- A passing forward pass supports implementation compatibility, not training correctness.
- A successful training run supports executability, not improved performance.
- An improvement on one benchmark or setting does not by itself support a broader generalization claim.

Keep implementation evidence, training evidence, evaluation evidence, and research conclusions separate. State which layer the available evidence supports, and narrow the claim rather than promoting evidence beyond that layer.
