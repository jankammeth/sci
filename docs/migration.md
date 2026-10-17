# Migrating to 0.2.0

The public `solve`, `Config`, `SolverConfig`, and `ContactSolver` interfaces now
default to union-find contact components. Existing configurations that omit
`sci_fleet_mode` use this default, including older configuration objects supplied
directly to `ContactSolver`. Select `sci_fleet_mode="full"` explicitly to reproduce
the 0.1.0 formulation. Numerical tolerances are unchanged.

Returned trajectories, samples, and contact indices retain their fleet-wide
shape. Low-level users of `ContactSolver.system` should note that the component
mode exposes a tuple of local systems; `matching_components` maps them to robot
IDs. `metrics["system_size"]` counts the unknowns actually solved. The reference
full mode retains its single-system representation. See [the API guide](api.md)
for component reuse, simultaneous contacts, merge limitations, and stage timing.

# Extraction from contact-solver

Reference: `https://github.com/jankammeth/contact-solver`, commit
`641a2341a6a59938e285b8ff872bd484713e0605`.

Four of the eight files listed in `UPSTREAM.json` remain byte-identical to the
reference. The collision wrapper changes only `contact-solver` to `sci-solver`
in the reinstall warning. The C kernel uses `_alloca` for the same three stack
arrays under MSVC, which does not support C variable-length arrays. Other
compilers retain the original declarations. Numerical formulas and control flow
in the explicit full-fleet mode are retained; `UPSTREAM.json` records both
extraction edits as explicit text replacements. Since extraction, the contact
solver driver has gained stage timing and dispatch to active-robot and
connected-component reductions, with components selected by default in 0.2.0.
Its original extraction hash remains recorded;
`current_sha256` identifies the revised file and `post_extraction_changes`
describes the additions. The reductions live in a separate module and reuse the
matching equations and Newton algorithm. Shared contact initialization, recovery,
and validation operations were extracted from the driver without changing their
formulas. Newton now has optional stage profiling and LSMR iteration diagnostics;
its current hash and change description are also recorded.
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

`tools/compare_upstream.py` compares the installed package's explicit `full` mode
against a checkout of the reference commit. Component behavior is checked by
independent physical tests and comparisons with full mode. Install the reference
package in a separate environment,
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
