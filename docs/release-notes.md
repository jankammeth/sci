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
