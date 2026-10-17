# Extraction from contact-solver

Reference: `https://github.com/jankammeth/contact-solver`, commit
`641a2341a6a59938e285b8ff872bd484713e0605`.

Seven of the eight files listed in `UPSTREAM.json` are byte-identical to the
reference. The collision wrapper changes only `contact-solver` to `sci-solver`
in the reinstall warning; its numerical code is unchanged.
They contain the joint solver, cubic representation, matching equations,
derivatives, nonlinear solve safeguards, recovery, and collision kernels.
The test suite retains independent polynomial, derivative, endpoint, reaction,
contact-removal, and larger-fleet checks from that implementation.

The package boundary changes from `contact_solver` to `sci_solver`; distribution
metadata changes to `sci-solver`. The new root import exposes only the joint solver
and its configuration/API. It does not import plotting, baselines, or scenario
generation. Configuration uses standard dataclasses and JSON. Original YAML
experiment configuration remains in the historical repository.

`solve` and `Solution` wrap the reference class. The CLI retains its saved-state
interface and deterministic smoke problems, with JSON replacing YAML and OSQP
removed from the environment report. Plotting is an optional example dependency.

## Numerical comparison

`tools/compare_upstream.py` compares the installed package against a checkout of
the reference commit. Install the reference package in a separate environment,
then pass its interpreter and checkout paths. The reference environment needs
the original package's declared dependencies.

```sh
python tools/compare_upstream.py \
  --upstream /path/to/contact-solver \
  --upstream-python /path/to/reference-venv/bin/python \
  --output validation/upstream-comparison.json
```

The comparison checks source hashes, statuses, contacts, removal/recovery events,
continuous separation, matching residuals, cost, and sampled derivatives. Timing
is recorded as environment-dependent evidence, not required to match. It includes
nonzero velocities, multidimensional cases, five saved 20-robot regressions, and
unsuccessful solves. Original and extracted solvers run in separate processes.

The existing `contact-solver` repository and the paper's pinned historical submodule
remain available for reproducibility. New joint-solver development belongs here;
the paper integration should select a recorded SCI commit explicitly.
