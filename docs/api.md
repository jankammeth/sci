# API and numerical options

`solve(initial_positions, final_positions, *, time_horizon, min_distance,
initial_velocities=None, final_velocities=None, options=None)` returns a `Solution`.
Endpoint arrays have shape `(N, d)` with `N >= 2` and `d >= 2`.

## Results

| Member | Meaning |
| --- | --- |
| `success` | Matching checks and continuous separation both passed. |
| `sample(times, derivative=0)` | Array with shape `(N, len(times), d)`; derivative orders 0–3. |
| `trajectory` | Knot times, relative cubic coefficients, and barycenter coefficients. |
| `contacts` | Pair indices, times, normals, relative velocities, and reaction masses. |
| `acceleration_cost` | Integrated sum of squared robot accelerations. |
| `metrics` | Status, reason, residuals, separation, backend, timing, and recovery details. |

Times may be unordered but must be finite and inside the horizon. At a contact,
the third derivative uses the segment to its right; the final endpoint uses the
last segment. Orders 0–2 are continuous up to the matching tolerance.

## Options

Pass `SolverConfig(...)` through `options`. Defaults match the reference implementation.

| Option | Default | Meaning |
| --- | --- | --- |
| `sci_detection_tol` | `0.0` | Allowed absolute separation deficit. |
| `sci_residual_tol` | `1e-7` | Matching and endpoint residual tolerance. |
| `sci_mass_tol` | `1e-9` | Allowed negative reaction mass within tolerance. |
| `sci_min_contact_gap` | `0.0` | Minimum knot gap; zero still requires strict ordering. |
| `sci_root_max_iterations` | `100` | Newton iterations per matching solve. |
| `sci_reaction_recovery` | `True` | Enable condensed recovery when matching fails. |
| `timeout` | `300.0` | Cooperative solve deadline in seconds. |
| `use_analytical_jacobian` | `True` | Use the analytical matching Jacobian. |

Statuses are `success`, `unresolved`, `root_failure`, and `invalid_solution`.
`metrics["convergence_reason"]` supplies details. Unsuccessful iterates remain
available for diagnosis. The deadline is checked between numerical operations;
it does not interrupt an in-progress linear solve.

For direct control, use `Config`, `ProblemConfig`, and `ContactSolver`, followed by
`set_initial_states`, `set_final_states`, and `generate_trajectories`. This retains
the reference interface. Its `ProblemConfig.timestep` is a legacy configuration
field used by automatic horizon selection when the horizon is omitted; it does
not discretize contact matching or collision checking. The convenience `solve`
function requires an explicit horizon.

## Saved problems

```sh
sci-solve --states endpoints.npz --config examples/config.json \
  --output outputs/metrics.json --trajectories outputs/trajectories.npz
```

Input NPZ keys are `initial_positions`, `final_positions`, and optionally
`initial_velocities`, `final_velocities`. JSON configuration uses `problem` and
`solver` objects; unknown fields are rejected. `Config.to_json` and
`Config.from_json` support roundtrips without a YAML dependency.

The CLI exits zero only for a successful solve. Sampled output includes times,
positions, velocities, accelerations, contact states, and contact pairs.
