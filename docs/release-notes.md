# 0.2.0

- Makes union-find contact components the default through `solve`, `ContactSolver`,
  and the CLI, at every fleet size.
- Excludes robots without contacts from matching systems and reconstructs their
  unconstrained endpoint trajectories.
- Reuses unchanged component solutions, trajectories, and checks; rebuilds groups
  after contact insertion or removal, including merges and splits.
- Retains continuous collision checking across the entire fleet and the existing
  matching equations, tolerances, Newton solver, and reaction recovery.
- Records disjoint solve-stage timings, component sizes/builds/reuses, and optional
  detailed Newton profiling. Profiling is disabled by default.
- Retains explicit `full` and `active` modes for comparisons and reproducibility.
- Includes the component benchmarks and regression fixtures in source distributions.

The high-level trajectory and contact interfaces are unchanged. In the new
default, the low-level `ContactSolver.system` attribute is a tuple of component
systems. Choose `SolverConfig(sci_fleet_mode="full")` for the previous single-system
representation. Small contacting fleets can be slower because of component
bookkeeping; there is no fleet-size-specific algorithm switch. See the
[migration guide](migration.md) and [benchmarks](component-reuse.md).

# 0.1.0

Initial standalone release of the joint Sequential Contact Insertion solver.

- Preserves the numerical implementation from contact-solver commit `641a234`.
- Provides the `sci_solver.solve` interface and structured trajectory results.
- Retains contact removal, condensed reaction recovery, and continuous checking.
- Includes an optional native C collision checker and a portable Python fallback.
- Uses NumPy and SciPy at runtime, with optional plotting examples.
- Includes numerical regressions, migration comparisons, and installation tests.

The release supports the unconstrained robot-robot problem in dimensions at least
two. It does not guarantee convergence or a global minimum. Baselines, prioritized
planning, and historical experiment runners remain in the original repository.
